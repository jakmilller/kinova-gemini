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

// --- Action Execution ---
void Controller::execute_pose(const std::shared_ptr<GoalHandlePose> goal_handle)
{
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<ros2_interfaces::action::MoveToPose::Result>();
    auto feedback = std::make_shared<ros2_interfaces::action::MoveToPose::Feedback>();

    k_api::Base::Action action;
    action.set_name("Cartesian Move");

    k_api::Base::Pose current_pose;
    {
        std::lock_guard<std::mutex> lock(mApiMutex);
        current_pose = mBase->GetMeasuredCartesianPose();
    }

    auto wrap_angle = [](float target, float current) {
        float diff = target - current;
        while (diff > 180.0f) { target -= 360.0f; diff -= 360.0f; }
        while (diff < -180.0f) { target += 360.0f; diff += 360.0f; }
        return target;
    };

    auto reach_pose = action.mutable_reach_pose();
    auto pose = reach_pose->mutable_target_pose();
    pose->set_x(goal->x);
    pose->set_y(goal->y);
    pose->set_z(goal->z);
    pose->set_theta_x(wrap_angle(goal->theta_x, current_pose.theta_x()));
    pose->set_theta_y(wrap_angle(goal->theta_y, current_pose.theta_y()));
    pose->set_theta_z(wrap_angle(goal->theta_z, current_pose.theta_z()));

    // No speed constraint by default: the arm's Cartesian speed is configured in the Kinova
    // Web App (the CARTESIAN_TRAJECTORY soft limits), and that is the only place to change it.
    // A constraint here is a ceiling BELOW that, never a way to exceed it -- an earlier
    // hard-coded one (0.1 m/s, 20 deg/s) is why Cartesian moves used to crawl while
    // execute_joints, which sets no constraint, ran at full speed.
    //
    // speed_scaling > 0 opts into a slower move (both fields must be set: Speed carries them
    // as a pair, so leaving orientation at 0 would command zero rotation speed). Kortex applies
    // the two independently and the move takes the longer of the two times.
    if (goal->speed_scaling > 0) {
        auto speed = reach_pose->mutable_constraint()->mutable_speed();
        speed->set_translation(goal->speed_scaling);
        speed->set_orientation(20.0f);
    }

    try {
        {
            std::lock_guard<std::mutex> lock(mApiMutex);
            mBase->ExecuteAction(action);
        }

        RCLCPP_INFO(this->get_logger(), "Executing Cartesian target: (%.3f, %.3f, %.3f)%s",
                    goal->x, goal->y, goal->z,
                    goal->speed_scaling > 0 ? " [speed-limited]" : "");

        // With no execution timeout, a move that never converges loops until it is cancelled --
        // so log distance_to_go every ~2s (20 * 100ms) to make a stalled move visible in Terminal 1.
        int log_counter = 0;
        while (rclcpp::ok()) {
            if (goal_handle->is_canceling()) {
                std::lock_guard<std::mutex> lock(mApiMutex);
                mBase->Stop();
                result->success = false;
                goal_handle->canceled(result);
                RCLCPP_WARN(this->get_logger(), "Cartesian move cancelled.");
                return;
            }

            k_api::Base::Pose current;
            {
                std::lock_guard<std::mutex> lock(mApiMutex);
                current = mBase->GetMeasuredCartesianPose();
            }

            double dist = sqrt(pow(goal->x - current.x(), 2) + pow(goal->y - current.y(), 2) + pow(goal->z - current.z(), 2));
            feedback->distance_to_go = dist;
            goal_handle->publish_feedback(feedback);
            if (dist < 0.01) break;
            if (++log_counter % 20 == 0) {
                RCLCPP_INFO(this->get_logger(), "Pose execution polling: distance_to_go = %.3f m", dist);
            }
            std::this_thread::sleep_for(100ms);
        }
        RCLCPP_INFO(this->get_logger(), "Cartesian target reached.");
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
        std::this_thread::sleep_for(std::chrono::seconds(2));
        // TCP is now updated proactively in publishState
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

        auto msg = ros2_interfaces::msg::RobotState();
        msg.x = cartesian.x(); msg.y = cartesian.y(); msg.z = cartesian.z();
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
