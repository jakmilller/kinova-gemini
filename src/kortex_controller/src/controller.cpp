#include "kortex_controller/controller.hpp"
#include <cmath>
#include <algorithm>
#include <future>
#include <rclcpp_action/rclcpp_action.hpp>
#include "ros2_interfaces/action/move_to_pose.hpp"
#include "ros2_interfaces/action/move_to_joints.hpp"
#include "ros2_interfaces/action/gripper_command.hpp"
#include <geometry_msgs/msg/point.hpp>
#include <yaml-cpp/yaml.h>

using namespace std;
using namespace std::placeholders;

namespace {
// Fallback if config.yaml has no `gripper` block: Robotiq 2F-140 fully-closed fingertip
// (0.230 m) minus fully-open fingertip (0.2002 m).
constexpr double DEFAULT_FINGERTIP_EXTENSION_M = 0.0298;

// Straight-line approach defaults. Deliberately slow -- this is the segment that ends with
// the fingers around an object, so it is the one where overshoot is most expensive.
constexpr float DEFAULT_LINEAR_SPEED_MPS = 0.05f;
constexpr float LINEAR_ORIENTATION_SPEED_DPS = 15.0f;

// Gripper settle detection: treat the fingers as moving above this speed, give the command
// this long to get them started, and never wait longer than the timeout in total.
constexpr float GRIPPER_MOVING_SPEED = 0.01f;
constexpr double GRIPPER_START_GRACE_S = 0.5;
constexpr double GRIPPER_SETTLE_TIMEOUT_S = 3.0;
}  // namespace

Controller::Controller() : Node("kinova_controller")
{
    // --- Parameters ---
    this->declare_parameter("robot_ip", "192.168.1.10");
    this->declare_parameter("username", "admin");
    this->declare_parameter("password", "admin");
    this->declare_parameter("config_path", std::string(std::getenv("HOME") ? std::getenv("HOME") : "") + "/kinova-gemini/config.yaml");

    string robot_ip = this->get_parameter("robot_ip").as_string();
    
    // --- Kortex API Setup ---
    mTransport = new k_api::TransportClientTcp();
    mRouter = new k_api::RouterClient(mTransport, [](k_api::KError err) {
        RCLCPP_ERROR(rclcpp::get_logger("kortex_api"), "Kortex Transport Error: %s", err.toString().c_str());
    });
    mTransport->connect(robot_ip, 10000);

    auto session_manager = new k_api::SessionManager(mRouter);
    auto create_session_info = k_api::Session::CreateSessionInfo();
    create_session_info.set_username(this->get_parameter("username").as_string());
    create_session_info.set_password(this->get_parameter("password").as_string());
    create_session_info.set_session_inactivity_timeout(60000);
    create_session_info.set_connection_inactivity_timeout(2000);
    session_manager->CreateSession(create_session_info);

    mBase = new k_api::Base::BaseClient(mRouter);
    mBaseCyclic = new k_api::BaseCyclic::BaseCyclicClient(mRouter);

    // Must run before the state timer starts: publishState uses the fingertip extension to
    // convert the measured TCP into the fingertip pose it publishes.
    loadGripperGeometryFromConfig();

    // Push config.yaml's static_obstacles into firmware Protection Zones on every
    // startup, so editing the config and relaunching is enough to keep the arm's
    // enforced no-go volumes in sync -- no separate setup script to remember to rerun.
    configureProtectionZonesFromConfig();

    // --- Action Servers ---
    this->mActionPoseServer = rclcpp_action::create_server<ros2_interfaces::action::MoveToPose>(
        this, "move_to_pose",
        std::bind(&Controller::handle_pose_goal, this, _1, _2),
        std::bind(&Controller::handle_pose_cancel, this, _1),
        std::bind(&Controller::handle_pose_accepted, this, _1));

    this->mActionJointsServer = rclcpp_action::create_server<ros2_interfaces::action::MoveToJoints>(
        this, "move_to_joints",
        std::bind(&Controller::handle_joints_goal, this, _1, _2),
        std::bind(&Controller::handle_joints_cancel, this, _1),
        std::bind(&Controller::handle_joints_accepted, this, _1));

    this->mActionLinearServer = rclcpp_action::create_server<ros2_interfaces::action::MoveLinear>(
        this, "move_linear",
        std::bind(&Controller::handle_linear_goal, this, _1, _2),
        std::bind(&Controller::handle_linear_cancel, this, _1),
        std::bind(&Controller::handle_linear_accepted, this, _1));

    this->mActionGripperServer = rclcpp_action::create_server<ros2_interfaces::action::GripperCommand>(
        this, "gripper_command",
        std::bind(&Controller::handle_gripper_goal, this, _1, _2),
        std::bind(&Controller::handle_gripper_cancel, this, _1),
        std::bind(&Controller::handle_gripper_accepted, this, _1));

    this->mActionGraspServer = rclcpp_action::create_server<ros2_interfaces::action::GripperCommand>(
        this, "grasp_object",
        std::bind(&Controller::handle_grasp_goal, this, _1, _2),
        std::bind(&Controller::handle_grasp_cancel, this, _1),
        std::bind(&Controller::handle_grasp_accepted, this, _1));

    // --- Telemetry Pub ---
    mPubState = this->create_publisher<ros2_interfaces::msg::RobotState>("robot_state", 10);
    mPubJointState = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    mTimer = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&Controller::publishState, this));

    // Protection zones are firmware-side and invisible to any other ROS message.
    // Read them back from the arm once (rather than reusing the config.yaml values
    // we just pushed above) so the startup log line and RViz wireframe reflect what's
    // actually enforced on hardware, catching e.g. a rejected/malformed zone.
    // transient_local durability means a late-starting RViz still picks up this one
    // publish, so there's no need to keep re-publishing/re-logging on a timer.
    mPubProtectionZones = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "protection_zones", rclcpp::QoS(1).transient_local());
    publishProtectionZones();

    // Subscribe to the firmware's own real-time protection zone events, so a zone-triggered
    // stop is reported directly rather than inferred from a stalled move.
    subscribeToProtectionZoneEvents();

    RCLCPP_INFO(this->get_logger(), "Kinova Controller Initialized");
}

// --- Action Callbacks ---
rclcpp_action::GoalResponse Controller::handle_pose_goal(const rclcpp_action::GoalUUID &, std::shared_ptr<const ros2_interfaces::action::MoveToPose::Goal>) {
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}
rclcpp_action::CancelResponse Controller::handle_pose_cancel(const std::shared_ptr<GoalHandlePose>) {
    return rclcpp_action::CancelResponse::ACCEPT;
}
void Controller::handle_pose_accepted(const std::shared_ptr<GoalHandlePose> goal_handle) {
    std::thread{std::bind(&Controller::execute_pose, this, std::placeholders::_1), goal_handle}.detach();
}

rclcpp_action::GoalResponse Controller::handle_joints_goal(const rclcpp_action::GoalUUID &, std::shared_ptr<const ros2_interfaces::action::MoveToJoints::Goal>) {
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}
rclcpp_action::CancelResponse Controller::handle_joints_cancel(const std::shared_ptr<GoalHandleJoints>) {
    return rclcpp_action::CancelResponse::ACCEPT;
}
void Controller::handle_joints_accepted(const std::shared_ptr<GoalHandleJoints> goal_handle) {
    std::thread{std::bind(&Controller::execute_joints, this, std::placeholders::_1), goal_handle}.detach();
}

rclcpp_action::GoalResponse Controller::handle_linear_goal(const rclcpp_action::GoalUUID &, std::shared_ptr<const MoveLinear::Goal>) {
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}
rclcpp_action::CancelResponse Controller::handle_linear_cancel(const std::shared_ptr<GoalHandleLinear>) {
    return rclcpp_action::CancelResponse::ACCEPT;
}
void Controller::handle_linear_accepted(const std::shared_ptr<GoalHandleLinear> goal_handle) {
    std::thread{std::bind(&Controller::execute_linear, this, std::placeholders::_1), goal_handle}.detach();
}

rclcpp_action::GoalResponse Controller::handle_gripper_goal(const rclcpp_action::GoalUUID &, std::shared_ptr<const GripperCommand::Goal>) {
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}
rclcpp_action::CancelResponse Controller::handle_gripper_cancel(const std::shared_ptr<GoalHandleGripper>) {
    return rclcpp_action::CancelResponse::ACCEPT;
}
void Controller::handle_gripper_accepted(const std::shared_ptr<GoalHandleGripper> goal_handle) {
    std::thread{std::bind(&Controller::execute_gripper, this, std::placeholders::_1), goal_handle}.detach();
}

rclcpp_action::GoalResponse Controller::handle_grasp_goal(const rclcpp_action::GoalUUID &, std::shared_ptr<const GripperCommand::Goal>) {
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}
rclcpp_action::CancelResponse Controller::handle_grasp_cancel(const std::shared_ptr<GoalHandleGripper>) {
    return rclcpp_action::CancelResponse::ACCEPT;
}
void Controller::handle_grasp_accepted(const std::shared_ptr<GoalHandleGripper> goal_handle) {
    std::thread{std::bind(&Controller::execute_grasp, this, std::placeholders::_1), goal_handle}.detach();
}

float Controller::get_gripper_position()
{
    // Note: Mutex should be locked by caller if this is part of a larger transaction
    try {
        k_api::Base::GripperRequest request;
        request.set_mode(k_api::Base::GripperMode::GRIPPER_POSITION);
        auto measured = mBase->GetMeasuredGripperMovement(request);
        if (measured.finger_size() > 0) {
            return measured.finger(0).value() * 100.0f; // 0.0 to 100.0
        }
    } catch (...) {}
    return 0.0f;
}

float Controller::get_gripper_speed()
{
    // Note: Mutex should be locked by caller, as with get_gripper_position().
    try {
        k_api::Base::GripperRequest request;
        request.set_mode(k_api::Base::GripperMode::GRIPPER_SPEED);
        auto measured = mBase->GetMeasuredGripperMovement(request);
        if (measured.finger_size() > 0) {
            return measured.finger(0).value();
        }
    } catch (...) {}
    return 0.0f;
}

// --- Fingertip <-> firmware TCP geometry ---

void Controller::loadGripperGeometryFromConfig()
{
    mFingertipExtension = DEFAULT_FINGERTIP_EXTENSION_M;
    std::string config_path = this->get_parameter("config_path").as_string();

    try {
        YAML::Node config = YAML::LoadFile(config_path);
        YAML::Node gripper = config["gripper"];
        if (gripper && gripper["tcp_open_m"] && gripper["tcp_closed_m"]) {
            double open_m = gripper["tcp_open_m"].as<double>();
            double closed_m = gripper["tcp_closed_m"].as<double>();
            double extension = closed_m - open_m;
            if (extension < 0.0) {
                RCLCPP_WARN(this->get_logger(),
                    "config.yaml gripper.tcp_closed_m (%.4f) is less than tcp_open_m (%.4f) -- the "
                    "fingertips cannot retract as they close. Ignoring and using %.4f m.",
                    closed_m, open_m, mFingertipExtension);
            } else {
                mFingertipExtension = extension;
                RCLCPP_INFO(this->get_logger(),
                    "Gripper geometry: fingertips reach %.4f m further when closed "
                    "(open TCP %.4f m -> closed %.4f m). The firmware tool offset must be set to the OPEN value.",
                    mFingertipExtension, open_m, closed_m);
                return;
            }
        }
    } catch (const std::exception& ex) {
        RCLCPP_WARN(this->get_logger(), "Could not read gripper geometry from '%s': %s",
                    config_path.c_str(), ex.what());
    }

    RCLCPP_WARN(this->get_logger(),
        "No usable 'gripper' block in config.yaml -- using default fingertip extension of %.4f m.",
        mFingertipExtension);
}

double Controller::fingertipExtension(float gripper_pct) const
{
    // gripper_pct is Kortex's 0 = fully open .. 100 = fully closed. The firmware tool offset
    // is configured at the fully-open fingertip, so the correction is zero there and grows to
    // the full extension as the fingers swing forward. Linear between the two measured anchors.
    double pct = std::max(0.0f, std::min(100.0f, gripper_pct));
    return mFingertipExtension * pct / 100.0;
}

void Controller::toolZAxis(float theta_x, float theta_y, float theta_z, double out[3]) const
{
    // Kortex expresses orientation as extrinsic x-y-z Tait-Bryan angles in degrees, so the
    // full rotation is R = Rz(theta_z) * Ry(theta_y) * Rx(theta_x). The tool's +Z expressed in
    // base coordinates is R * [0,0,1] -- i.e. R's third column, written out directly here to
    // avoid pulling in a matrix library for nine multiplications.
    const double a = theta_x * M_PI / 180.0;
    const double b = theta_y * M_PI / 180.0;
    const double g = theta_z * M_PI / 180.0;
    out[0] = cos(g) * sin(b) * cos(a) + sin(g) * sin(a);
    out[1] = sin(g) * sin(b) * cos(a) - cos(g) * sin(a);
    out[2] = cos(b) * cos(a);
}

k_api::Base::Pose Controller::shiftAlongToolZ(const k_api::Base::Pose& pose, double distance) const
{
    // The one geometric primitive in this file. Used three ways: -extension to turn a
    // fingertip goal into a TCP command, +extension to turn measured TCP feedback into the
    // fingertip pose, and +distance for the straight-line approach move.
    double z_tool[3];
    toolZAxis(pose.theta_x(), pose.theta_y(), pose.theta_z(), z_tool);

    k_api::Base::Pose shifted(pose);
    shifted.set_x(pose.x() + distance * z_tool[0]);
    shifted.set_y(pose.y() + distance * z_tool[1]);
    shifted.set_z(pose.z() + distance * z_tool[2]);
    return shifted;
}

// --- Action Execution ---
void Controller::execute_pose(const std::shared_ptr<GoalHandlePose> goal_handle)
{
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<ros2_interfaces::action::MoveToPose::Result>();

    // We reach a Cartesian goal by solving IK and moving in JOINT space, not with a Cartesian
    // reach_pose. reach_pose drives the TCP on a straight line and holds/interpolates orientation
    // for the whole path; because every caller passes the CURRENT wrist orientation as the target,
    // that pins the wrist at a fixed orientation across the entire translation. On a 7-DOF arm that
    // frequently over-constrains the path into a wrist singularity and the move stalls before
    // arriving. A joint-space move only guarantees the ENDPOINT pose -- each joint interpolates
    // freely in between, so the wrist is allowed to reorient along the way and simply lands on the
    // requested 6D pose. (Trade-off: the TCP path is no longer a Cartesian straight line, and the
    // Cartesian speed ceiling `speed_scaling` no longer applies -- joint moves run at the firmware
    // ANGULAR_TRAJECTORY speed, same as reach_joint_angles elsewhere.)
    //
    // The goal is a FINGERTIP target. The arm's firmware tool frame is a fixed offset set at
    // the fully-OPEN fingertip, and Kortex has no concept of the fingers moving it -- so at any
    // partial closure the real tips sit fingertipExtension() further along tool +Z than the TCP
    // does. Pull the commanded TCP back by exactly that much and the tips land on the goal.
    // With the gripper fully open the correction is zero and this is a no-op.
    k_api::Base::Pose fingertip_target;
    fingertip_target.set_x(goal->x);
    fingertip_target.set_y(goal->y);
    fingertip_target.set_z(goal->z);
    fingertip_target.set_theta_x(goal->theta_x);
    fingertip_target.set_theta_y(goal->theta_y);
    fingertip_target.set_theta_z(goal->theta_z);

    k_api::Base::JointAngles ik_guess;
    float grip_pct;
    {
        std::lock_guard<std::mutex> lock(mApiMutex);
        // Seed IK with the current configuration so it returns a nearby solution (same
        // elbow/wrist branch) rather than an arbitrary one that would swing the arm around.
        ik_guess = mBase->GetMeasuredJointAngles();
        grip_pct = get_gripper_position();
    }

    const double extension = fingertipExtension(grip_pct);
    const k_api::Base::Pose tcp_target = shiftAlongToolZ(fingertip_target, -extension);

    k_api::Base::IKData ik_data;
    *ik_data.mutable_guess() = ik_guess;
    auto target_pose = ik_data.mutable_cartesian_pose();
    target_pose->set_x(tcp_target.x());
    target_pose->set_y(tcp_target.y());
    target_pose->set_z(tcp_target.z());
    target_pose->set_theta_x(tcp_target.theta_x());
    target_pose->set_theta_y(tcp_target.theta_y());
    target_pose->set_theta_z(tcp_target.theta_z());

    k_api::Base::JointAngles ik_solution;
    try {
        std::lock_guard<std::mutex> lock(mApiMutex);
        ik_solution = mBase->ComputeInverseKinematics(ik_data);
    } catch (k_api::KDetailedException& ex) {
        // Log both frames: "no IK solution" is only diagnosable if you can see the fingertip
        // goal that was asked for AND the TCP pose that was actually handed to the solver.
        RCLCPP_ERROR(this->get_logger(),
                     "IK found no solution for fingertip target (%.3f, %.3f, %.3f) "
                     "[TCP target (%.3f, %.3f, %.3f), gripper at %.1f%% -> %.4f m extension]: %s -- aborting move.",
                     goal->x, goal->y, goal->z,
                     tcp_target.x(), tcp_target.y(), tcp_target.z(),
                     grip_pct, extension, ex.what());
        result->success = false;
        goal_handle->abort(result);
        return;
    }

    k_api::Base::Action action;
    action.set_name("Joint Move (from pose)");
    // ComputeInverseKinematics returns a JointAngles with per-joint identifiers already set,
    // which is exactly what reach_joint_angles expects -- copy it straight in.
    *action.mutable_reach_joint_angles()->mutable_joint_angles() = ik_solution;

    try {
        {
            std::lock_guard<std::mutex> lock(mApiMutex);
            mBase->ExecuteAction(action);
        }

        RCLCPP_INFO(this->get_logger(),
                    "Executing joint-space move to fingertip target: (%.3f, %.3f, %.3f)",
                    goal->x, goal->y, goal->z);

        // Poll against the TCP target, not the fingertip goal: GetMeasuredCartesianPose reports
        // the firmware TCP, so the completion check has to live in that same raw frame.
        // timeout_s = 0 keeps the existing behaviour of waiting indefinitely for this action.
        auto outcome = pollUntilCartesianTarget<MoveToPose>(
            goal_handle, tcp_target.x(), tcp_target.y(), tcp_target.z(), "Pose move", 0.0);

        if (outcome == PollOutcome::CANCELLED) {
            result->success = false;
            goal_handle->canceled(result);
            return;
        }
        if (outcome != PollOutcome::REACHED) {
            result->success = false;
            goal_handle->abort(result);
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Fingertip target reached.");
        result->success = true;
        goal_handle->succeed(result);
    } catch (k_api::KDetailedException& ex) {
        RCLCPP_ERROR(this->get_logger(), "Kortex Error: %s", ex.what());
        result->success = false;
        goal_handle->abort(result);
    }
}

void Controller::execute_joints(const std::shared_ptr<GoalHandleJoints> goal_handle)
{
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<ros2_interfaces::action::MoveToJoints::Result>();
    auto feedback = std::make_shared<ros2_interfaces::action::MoveToJoints::Feedback>();

    std::vector<double> target_joints;

    RCLCPP_INFO(this->get_logger(), "Executing single-point joint target...");

    if (goal->joint_angles.size() < 7) {
        RCLCPP_ERROR(this->get_logger(), "Joint target has invalid joint count: %zu (expected 7)", goal->joint_angles.size());
        result->success = false;
        goal_handle->abort(result);
        return;
    }

    k_api::Base::Action action;
    action.set_name("Joint Move");
    auto reach_joints = action.mutable_reach_joint_angles();
    auto joints = reach_joints->mutable_joint_angles();

    for (size_t i = 0; i < 7; ++i) {
        auto j = joints->add_joint_angles();
        j->set_joint_identifier(i);
        j->set_value(static_cast<float>(goal->joint_angles[i]));
    }

    // Target joints are in degrees directly
    target_joints.resize(7);
    for (size_t j = 0; j < 7; ++j) {
        target_joints[j] = goal->joint_angles[j];
    }

    try {
        {
            std::lock_guard<std::mutex> lock(mApiMutex);
            mBase->ExecuteAction(action);
        }
    } catch (k_api::KDetailedException& ex) {
        RCLCPP_ERROR(this->get_logger(), "Kortex Error: %s", ex.what());
        result->success = false;
        goal_handle->abort(result);
        return;
    }

    // Polling / completion check loop
    int log_counter = 0;
    try {
        while (rclcpp::ok()) {
            if (goal_handle->is_canceling()) {
                std::lock_guard<std::mutex> lock(mApiMutex);
                mBase->Stop();
                result->success = false;
                goal_handle->canceled(result);
                return;
            }
            
            k_api::Base::JointAngles current_joints;
            {
                std::lock_guard<std::mutex> lock(mApiMutex);
                current_joints = mBase->GetMeasuredJointAngles();
            }

            float max_diff = 0;
            for (int i = 0; i < 7; ++i) {
                float target = static_cast<float>(target_joints[i]);
                float actual = current_joints.joint_angles(i).value();
                float diff = std::abs(target - actual);
                // Handle 360 degree wrapping
                if (diff > 180.0f) {
                    diff = 360.0f - diff;
                }
                max_diff = std::max(max_diff, diff);
            }

            // Print logging progress to Terminal 1 every 2 seconds (20 * 100ms)
            if (++log_counter % 20 == 0) {
                RCLCPP_INFO(this->get_logger(), "Joint execution polling: max_diff = %.2f degrees", max_diff);
            }
            
            if (max_diff < 1.0f) break; // 1.0 degree threshold for reliable completion
            std::this_thread::sleep_for(100ms);
        }
        result->success = true;
        goal_handle->succeed(result);
    } catch (k_api::KDetailedException& ex) {
        RCLCPP_ERROR(this->get_logger(), "Kortex Error: %s", ex.what());
        result->success = false;
        goal_handle->abort(result);
    }
}

void Controller::execute_linear(const std::shared_ptr<GoalHandleLinear> goal_handle)
{
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<ros2_interfaces::action::MoveLinear::Result>();

    k_api::Base::Pose current;
    {
        std::lock_guard<std::mutex> lock(mApiMutex);
        current = mBase->GetMeasuredCartesianPose();
    }

    // NO fingertip compensation here, deliberately. This is a pure relative translation and the
    // gripper does not move during it, so translating the TCP by `distance` translates the
    // fingertips by exactly `distance`. Working in raw TCP space also keeps the target
    // consistent with the GetMeasuredCartesianPose readings the completion poll compares against.
    const k_api::Base::Pose target = shiftAlongToolZ(current, goal->distance);

    k_api::Base::Action action;
    action.set_name("Linear Move (tool Z)");
    auto* constrained_pose = action.mutable_reach_pose();
    auto* pose = constrained_pose->mutable_target_pose();
    pose->set_x(target.x());
    pose->set_y(target.y());
    pose->set_z(target.z());
    // Orientation copied from the current pose: this move translates only.
    pose->set_theta_x(current.theta_x());
    pose->set_theta_y(current.theta_y());
    pose->set_theta_z(current.theta_z());

    // This is the one place we WANT Kortex's Cartesian reach_pose rather than the IK +
    // joint-space path execute_pose uses. reach_pose drives the TCP along a straight line and
    // holds orientation for the whole path. Over a long transit that pins the wrist and can
    // walk a 7-DOF arm into a singularity (which is exactly why execute_pose stopped using it).
    // Over this segment it is precisely the behaviour we need: the orientation is already the
    // grasp orientation and is not changing, the span is a few centimeters, and the straight
    // line is the entire point -- joint interpolation would arc the fingers through the object
    // the pre-grasp standoff exists to avoid.
    const float speed = goal->speed > 0.0f ? goal->speed : DEFAULT_LINEAR_SPEED_MPS;
    auto* speed_constraint = constrained_pose->mutable_constraint()->mutable_speed();
    speed_constraint->set_translation(speed);
    speed_constraint->set_orientation(LINEAR_ORIENTATION_SPEED_DPS);

    try {
        {
            std::lock_guard<std::mutex> lock(mApiMutex);
            mBase->ExecuteAction(action);
        }

        RCLCPP_INFO(this->get_logger(),
                    "Linear move: %.3f m along tool +Z at %.3f m/s -> (%.3f, %.3f, %.3f)",
                    goal->distance, speed, target.x(), target.y(), target.z());

        // Unlike execute_pose this one is bounded. An orientation-locked Cartesian segment can
        // refuse to converge (e.g. started near a wrist singularity), and a grasp approach that
        // hangs forever is worse than one that fails: allow 3x the nominal travel time plus a
        // fixed margin for acceleration and the arm's own settling.
        const double nominal_s = std::abs(goal->distance) / speed;
        const double timeout_s = nominal_s * 3.0 + 5.0;

        auto outcome = pollUntilCartesianTarget<MoveLinear>(
            goal_handle, target.x(), target.y(), target.z(), "Linear move", timeout_s);

        if (outcome == PollOutcome::CANCELLED) {
            result->success = false;
            goal_handle->canceled(result);
            return;
        }
        if (outcome != PollOutcome::REACHED) {
            result->success = false;
            goal_handle->abort(result);
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Linear move complete.");
        result->success = true;
        goal_handle->succeed(result);
    } catch (k_api::KDetailedException& ex) {
        RCLCPP_ERROR(this->get_logger(), "Kortex Error during linear move: %s", ex.what());
        result->success = false;
        goal_handle->abort(result);
    }
}

void Controller::execute_gripper(const std::shared_ptr<GoalHandleGripper> goal_handle)
{
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<ros2_interfaces::action::GripperCommand::Result>();

    k_api::Base::GripperCommand command;
    command.set_mode(k_api::Base::GripperMode::GRIPPER_POSITION);
    auto finger = command.mutable_gripper()->add_finger();
    finger->set_finger_identifier(1);
    finger->set_value(goal->position / 100.0f);

    try {
        {
            std::lock_guard<std::mutex> lock(mApiMutex);
            mBase->SendGripperCommand(command);
        }

        // Wait for the fingers to actually settle rather than sleeping a fixed 2 s. Polling the
        // measured finger speed returns as soon as motion stops -- which covers both "reached the
        // commanded position" and "stalled against an object" -- so short moves stop costing a
        // flat two seconds, and success is no longer reported while the fingers are still closing.
        // The grace period exists because the speed is legitimately zero in the moment between
        // sending the command and the gripper starting to move.
        const auto start = std::chrono::steady_clock::now();
        bool has_moved = false;
        while (rclcpp::ok()) {
            double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();

            float finger_speed;
            {
                std::lock_guard<std::mutex> lock(mApiMutex);
                finger_speed = get_gripper_speed();
            }

            if (std::abs(finger_speed) > GRIPPER_MOVING_SPEED) {
                has_moved = true;
            } else if (has_moved || elapsed > GRIPPER_START_GRACE_S) {
                // Either the fingers moved and have now stopped, or the grace period expired
                // without motion (the gripper was already at the commanded position).
                break;
            }

            if (elapsed > GRIPPER_SETTLE_TIMEOUT_S) {
                RCLCPP_WARN(this->get_logger(),
                    "Gripper still moving after %.1f s -- continuing anyway.", elapsed);
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }

        result->success = true;
        goal_handle->succeed(result);
    } catch (...) {
        result->success = false;
        goal_handle->abort(result);
    }
}

void Controller::execute_grasp(const std::shared_ptr<GoalHandleGripper> goal_handle)
{
    execute_gripper(goal_handle);
}

void Controller::publishState()
{
    std::lock_guard<std::mutex> lock(mApiMutex);
    try {
        auto cartesian = mBase->GetMeasuredCartesianPose();
        auto joints = mBase->GetMeasuredJointAngles();
        float gripper_pos = get_gripper_position();

        // Report the FINGERTIP pose, not the raw firmware TCP. This is the other half of the
        // compensation applied in execute_pose: because commands and feedback now speak the same
        // frame, a read-modify-write like move_relative (read robot_state, add a delta, send it
        // back) stays consistent without the caller knowing any of this exists.
        // NOTE: this therefore DIVERGES from the Cartesian pose shown in the Kinova Web App by up
        // to the full fingertip extension (~3 cm) when the gripper is closed. That is intentional --
        // the Web App reports the firmware tool frame, this topic reports where the fingers are.
        auto tip = shiftAlongToolZ(cartesian, fingertipExtension(gripper_pos));

        auto msg = ros2_interfaces::msg::RobotState();
        msg.x = tip.x(); msg.y = tip.y(); msg.z = tip.z();
        msg.theta_x = cartesian.theta_x(); msg.theta_y = cartesian.theta_y(); msg.theta_z = cartesian.theta_z();
        for (int i = 0; i < 7; ++i) msg.joint_angles[i] = joints.joint_angles(i).value();
        msg.gripper_position = gripper_pos;
        mPubState->publish(msg);

        auto jmsg = sensor_msgs::msg::JointState();
        jmsg.header.stamp = this->get_clock()->now();
        for (int i=0; i<7; ++i) {
            jmsg.name.push_back("joint_" + std::to_string(i+1));
            jmsg.position.push_back(joints.joint_angles(i).value() * M_PI / 180.0);
        }
        
        // Add gripper joint state for RViz visualization
        // 0-100% position is mapped to 0.0-0.7 radians for Robotiq 2F-140
        jmsg.name.push_back("finger_joint");
        jmsg.position.push_back((gripper_pos / 100.0) * 0.7);

        mPubJointState->publish(jmsg);
    } catch (...) {}
}

void Controller::configureProtectionZonesFromConfig()
{
    std::string config_path = this->get_parameter("config_path").as_string();

    YAML::Node config;
    try {
        config = YAML::LoadFile(config_path);
    } catch (const std::exception& ex) {
        RCLCPP_WARN(this->get_logger(), "Could not load '%s' for protection zones: %s", config_path.c_str(), ex.what());
        return;
    }

    YAML::Node obstacles = config["static_obstacles"];
    if (!obstacles || obstacles.size() == 0) {
        RCLCPP_INFO(this->get_logger(), "No 'static_obstacles' in config.yaml -- skipping protection zone setup.");
        return;
    }

    std::lock_guard<std::mutex> lock(mApiMutex);
    try {
        // Delete any existing zone whose name we're about to (re)create, so simply
        // editing config.yaml and relaunching can't accumulate duplicate/stale zones.
        auto existing = mBase->ReadAllProtectionZones();
        for (const auto& zone : existing.protection_zones()) {
            if (obstacles[zone.name()]) {
                mBase->DeleteProtectionZone(zone.handle());
                RCLCPP_INFO(this->get_logger(), "Deleted existing protection zone '%s' before recreating it.", zone.name().c_str());
            }
        }

        for (const auto& kv : obstacles) {
            std::string name = kv.first.as<std::string>();
            auto center = kv.second["center"].as<std::vector<double>>();
            auto size = kv.second["size"].as<std::vector<double>>();
            if (center.size() != 3 || size.size() != 3) {
                RCLCPP_WARN(this->get_logger(), "Protection zone '%s' has malformed center/size in config.yaml -- skipping.", name.c_str());
                continue;
            }

            k_api::Base::ProtectionZone zone;
            zone.set_name(name);
            zone.set_is_enabled(true);

            auto* shape = zone.mutable_shape();
            shape->set_shape_type(k_api::Base::RECTANGULAR_PRISM);
            shape->mutable_origin()->set_x(center[0]);
            shape->mutable_origin()->set_y(center[1]);
            shape->mutable_origin()->set_z(center[2]);
            shape->add_dimensions(size[0]);
            shape->add_dimensions(size[1]);
            shape->add_dimensions(size[2]);

            // Identity orientation -- our boxes are axis-aligned with base_link
            auto* orientation = shape->mutable_orientation();
            orientation->mutable_row1()->set_column1(1.0);
            orientation->mutable_row2()->set_column2(1.0);
            orientation->mutable_row3()->set_column3(1.0);

            auto handle = mBase->CreateProtectionZone(zone);
            RCLCPP_INFO(this->get_logger(),
                "Configured protection zone '%s' (handle=%u): center=(%.3f, %.3f, %.3f) size=(%.3f, %.3f, %.3f)",
                name.c_str(), handle.identifier(), center[0], center[1], center[2], size[0], size[1], size[2]);
        }
    } catch (const std::exception& ex) {
        RCLCPP_WARN(this->get_logger(), "Failed to configure protection zones on firmware: %s", ex.what());
    }
}

void Controller::publishProtectionZones()
{
    k_api::Base::ProtectionZoneList zone_list;
    try {
        std::lock_guard<std::mutex> lock(mApiMutex);
        zone_list = mBase->ReadAllProtectionZones();
    } catch (const std::exception& ex) {
        RCLCPP_WARN(this->get_logger(), "Failed to read protection zones from firmware: %s", ex.what());
        return;
    }

    visualization_msgs::msg::MarkerArray marker_array;
    int id = 0;
    for (const auto& zone : zone_list.protection_zones()) {
        const auto& shape = zone.shape();
        if (shape.dimensions_size() < 3) continue;

        double cx = shape.origin().x(), cy = shape.origin().y(), cz = shape.origin().z();
        double dx = shape.dimensions(0), dy = shape.dimensions(1), dz = shape.dimensions(2);

        RCLCPP_INFO(this->get_logger(),
            "Protection zone '%s': origin=(%.3f, %.3f, %.3f) size=(%.3f, %.3f, %.3f) enabled=%s",
            zone.name().c_str(), cx, cy, cz, dx, dy, dz, zone.is_enabled() ? "true" : "false");

        if (zone.has_handle()) {
            mZoneNamesByHandle[zone.handle().identifier()] = zone.name();
        }

        // Assumes axis-aligned zones (identity orientation), matching how
        // configureProtectionZonesFromConfig() creates them.
        double hx = dx / 2.0, hy = dy / 2.0, hz = dz / 2.0;
        geometry_msgs::msg::Point corners[8];
        double signs[8][3] = {
            {-1,-1,-1}, {1,-1,-1}, {1,1,-1}, {-1,1,-1},
            {-1,-1, 1}, {1,-1, 1}, {1,1, 1}, {-1,1, 1},
        };
        for (int i = 0; i < 8; ++i) {
            corners[i].x = cx + signs[i][0] * hx;
            corners[i].y = cy + signs[i][1] * hy;
            corners[i].z = cz + signs[i][2] * hz;
        }
        static const int edges[12][2] = {
            {0,1},{1,2},{2,3},{3,0}, {4,5},{5,6},{6,7},{7,4}, {0,4},{1,5},{2,6},{3,7}
        };

        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "base_link";
        marker.header.stamp = this->now();
        marker.ns = "protection_zones";
        marker.id = id++;
        marker.type = visualization_msgs::msg::Marker::LINE_LIST;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.004; // line width in meters
        marker.color.r = 1.0f; marker.color.g = 0.2f; marker.color.b = 0.0f;
        marker.color.a = zone.is_enabled() ? 0.9f : 0.25f;
        for (const auto& e : edges) {
            marker.points.push_back(corners[e[0]]);
            marker.points.push_back(corners[e[1]]);
        }
        marker_array.markers.push_back(marker);
    }

    mPubProtectionZones->publish(marker_array);
}

void Controller::subscribeToProtectionZoneEvents()
{
    try {
        std::lock_guard<std::mutex> lock(mApiMutex);
        mProtectionZoneNotifHandle = mBase->OnNotificationProtectionZoneTopic(
            std::bind(&Controller::onProtectionZoneNotification, this, _1),
            k_api::Common::NotificationOptions());
    } catch (const std::exception& ex) {
        RCLCPP_WARN(this->get_logger(), "Failed to subscribe to protection zone notifications: %s", ex.what());
    }
}

// Called from a Kortex-internal notification thread, not the rclcpp executor thread.
// mZoneNamesByHandle is only ever written during startup (before this subscription
// exists), so reading it here without a lock is safe.
void Controller::onProtectionZoneNotification(k_api::Base::ProtectionZoneNotification notification)
{
    std::string name = "<unknown>";
    if (notification.has_handle()) {
        auto it = mZoneNamesByHandle.find(notification.handle().identifier());
        if (it != mZoneNamesByHandle.end()) name = it->second;
    }

    switch (notification.event()) {
        case k_api::Base::REACHED:
            RCLCPP_WARN(this->get_logger(),
                "Protection zone '%s' REACHED -- the arm is at the zone boundary and will not move further into it.",
                name.c_str());
            break;
        case k_api::Base::ENTERED:
            RCLCPP_WARN(this->get_logger(), "Protection zone '%s' ENTERED.", name.c_str());
            break;
        case k_api::Base::EXITED:
            RCLCPP_INFO(this->get_logger(), "Protection zone '%s' EXITED.", name.c_str());
            break;
        default:
            break;
    }
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Controller>());
    rclcpp::shutdown();
    return 0;
}
