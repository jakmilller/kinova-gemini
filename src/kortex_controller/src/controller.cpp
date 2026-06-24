#include "kortex_controller/controller.hpp"
#include <cmath>
#include <algorithm>
#include <future>
#include <rclcpp_action/rclcpp_action.hpp>
#include "ros2_interfaces/action/move_to_pose.hpp"
#include "ros2_interfaces/action/move_to_joints.hpp"
#include "ros2_interfaces/action/gripper_command.hpp"

using namespace std;
using namespace std::placeholders;

Controller::Controller() : Node("kinova_controller")
{
    // --- Parameters ---
    this->declare_parameter("robot_ip", "192.168.1.10");
    this->declare_parameter("username", "admin");
    this->declare_parameter("password", "admin");
    
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

    // --- Services ---
    mSrvComputeIK = this->create_service<ros2_interfaces::srv::ComputeIK>(
        "compute_ik", std::bind(&Controller::computeIK, this, _1, _2));

    // --- Telemetry Pub ---
    mPubState = this->create_publisher<ros2_interfaces::msg::RobotState>("robot_state", 10);
    mPubJointState = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    mTimer = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&Controller::publishState, this));

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

    auto speed = reach_pose->mutable_constraint()->mutable_speed();
    speed->set_translation(goal->speed_scaling > 0 ? goal->speed_scaling : 0.1f);
    speed->set_orientation(20.0f);

    try {
        {
            std::lock_guard<std::mutex> lock(mApiMutex);
            mBase->ExecuteAction(action);
        }

        auto start_time = std::chrono::steady_clock::now();
        while (rclcpp::ok()) {
            if (goal_handle->is_canceling()) {
                std::lock_guard<std::mutex> lock(mApiMutex);
                mBase->Stop();
                result->success = false;
                goal_handle->canceled(result);
                return;
            }

            if (std::chrono::steady_clock::now() - start_time > std::chrono::seconds(15)) {
                RCLCPP_WARN(this->get_logger(), "Pose execution timed out after 15 seconds.");
                std::lock_guard<std::mutex> lock(mApiMutex);
                mBase->Stop();
                result->success = false;
                goal_handle->succeed(result); // Return what we got
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

void Controller::execute_joints(const std::shared_ptr<GoalHandleJoints> goal_handle)
{
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<ros2_interfaces::action::MoveToJoints::Result>();
    auto feedback = std::make_shared<ros2_interfaces::action::MoveToJoints::Feedback>();

    std::vector<double> target_joints;
    bool is_trajectory = !goal->trajectory_points.empty();

    if (is_trajectory) {
        RCLCPP_INFO(this->get_logger(), "Executing cuRobo multi-point trajectory (%zu waypoints)...", goal->trajectory_points.size());

        // --- Build the angular waypoint list ---
        // For Kinova AngularWaypoints, set ONLY the per-waypoint duration. Do NOT mix in
        // maximum_velocities or optimal blending: the firmware trajectory generator requires
        // the first and last waypoints to be at rest, and an over-constrained list is rejected
        // (INITIAL/FINAL_WAYPOINT_NO_STOP, INVALID_DURATION, INVALID_JOINT_SPEED).
        k_api::Base::WaypointList waypoint_list;
        waypoint_list.set_duration(0.0f);
        waypoint_list.set_use_optimal_blending(false);

        for (size_t i = 0; i < goal->trajectory_points.size(); ++i) {
            const auto& pt = goal->trajectory_points[i];
            if (pt.positions.size() < 7) {
                RCLCPP_ERROR(this->get_logger(), "Waypoint %zu has invalid joint count: %zu (expected 7)", i, pt.positions.size());
                result->success = false;
                goal_handle->abort(result);
                return;
            }

            auto wp = waypoint_list.add_waypoints();
            wp->set_name("waypoint_" + std::to_string(i));

            auto angular_wp = wp->mutable_angular_waypoint();
            for (size_t j = 0; j < 7; ++j) {
                // Convert radians (cuRobo / ROS standard) to degrees (Kortex standard)
                angular_wp->add_angles(static_cast<float>(pt.positions[j] * 180.0 / M_PI));
            }

            // Per-segment duration. cuRobo populates time_from_start = i * interpolation_dt on
            // every point, so for i>=1 we use the real planned dt and the executed motion
            // tracks cuRobo's interpolated trajectory. Waypoint 0 is the start (current)
            // config: its time_from_start is 0, so there is no previous delta - give it one
            // interpolation step instead of letting the old heuristic stamp a 1.0s dwell that
            // made every move begin with a 1-second pause.
            float segment_duration = 0.0f;
            double t_curr = pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9;

            if (i == 0) {
                // Use the spacing to the next point (the nominal interpolation_dt) so the
                // start is consistent with the rest of the trajectory; fall back to 0.05s if
                // it's the only point.
                double t_next = 0.05;
                if (goal->trajectory_points.size() > 1) {
                    const auto& next = goal->trajectory_points[1];
                    double tn = next.time_from_start.sec + next.time_from_start.nanosec * 1e-9;
                    if (tn > 1e-3) t_next = tn;
                }
                segment_duration = static_cast<float>(t_next);
            } else {
                const auto& prev = goal->trajectory_points[i-1];
                double t_prev = prev.time_from_start.sec + prev.time_from_start.nanosec * 1e-9;
                double dt = t_curr - t_prev;
                if (dt > 1e-3) {
                    segment_duration = static_cast<float>(dt);
                } else {
                    // cuRobo should always provide timing; reaching here means the
                    // interpolation_dt handoff regressed. Warn (throttled) instead of
                    // silently slowing the trajectory.
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                        "Waypoint %zu missing time_from_start delta; using ~20 deg/s fallback timing. "
                        "Check cuRobo interpolation_dt handoff.", i);
                    double max_delta_deg = 0.0;
                    for (size_t j = 0; j < 7; ++j) {
                        double prev_deg = goal->trajectory_points[i-1].positions[j] * 180.0 / M_PI;
                        double curr_deg = pt.positions[j] * 180.0 / M_PI;
                        max_delta_deg = std::max(max_delta_deg, std::abs(curr_deg - prev_deg));
                    }
                    segment_duration = std::max(0.05f, static_cast<float>(max_delta_deg / 20.0));
                }
            }
            angular_wp->set_duration(segment_duration);
        }

        if (!goal->trajectory_points.empty()) {
            const auto& last = goal->trajectory_points.back();
            double total_s = last.time_from_start.sec + last.time_from_start.nanosec * 1e-9;
            RCLCPP_INFO(this->get_logger(),
                "Built %zu-waypoint trajectory, total planned duration %.2fs (cuRobo interpolation_dt timing).",
                goal->trajectory_points.size(), total_s);
        }

        // --- Validate BEFORE executing: the robot silently refuses an invalid list ---
        try {
            k_api::Base::WaypointValidationReport report;
            {
                std::lock_guard<std::mutex> lock(mApiMutex);
                report = mBase->ValidateWaypointList(waypoint_list);
            }
            int err_count = report.trajectory_error_report().trajectory_error_elements_size();
            if (err_count > 0) {
                RCLCPP_ERROR(this->get_logger(), "Waypoint trajectory REJECTED by validator (%d errors):", err_count);
                for (int e = 0; e < err_count; ++e) {
                    const auto& el = report.trajectory_error_report().trajectory_error_elements(e);
                    RCLCPP_ERROR(this->get_logger(), "  [wp %u] type=%d value=%.3f (min=%.3f max=%.3f): %s",
                                 el.waypoint_index(), static_cast<int>(el.error_type()),
                                 el.error_value(), el.min_value(), el.max_value(), el.message().c_str());
                }
                result->success = false;
                goal_handle->abort(result);
                return;
            }
        } catch (k_api::KDetailedException& ex) {
            RCLCPP_ERROR(this->get_logger(), "Kortex error during waypoint validation: %s", ex.what());
            result->success = false;
            goal_handle->abort(result);
            return;
        } catch (const std::exception& ex) {
            // Kortex's generated RPC stubs (e.g. ValidateWaypointList's default 3000ms
            // timeout) throw a bare std::runtime_error on timeout, not a KDetailedException.
            // Uncaught, this would propagate out of execute_joints() - which runs in a
            // detached std::thread - and call std::terminate(), killing the entire
            // controller node over a single bad goal. Abort just this goal instead.
            RCLCPP_ERROR(this->get_logger(), "Error during waypoint validation: %s", ex.what());
            result->success = false;
            goal_handle->abort(result);
            return;
        }

        // --- Execute and wait on the action notification (ACTION_END / ACTION_ABORT) ---
        auto finished = std::make_shared<std::promise<k_api::Base::ActionEvent>>();
        auto finished_future = finished->get_future();
        auto event_cb = [finished](k_api::Base::ActionNotification notif) {
            auto ev = notif.action_event();
            if (ev == k_api::Base::ACTION_END || ev == k_api::Base::ACTION_ABORT) {
                try { finished->set_value(ev); } catch (...) { /* already set */ }
            }
        };

        k_api::Common::NotificationHandle notif_handle;
        try {
            std::lock_guard<std::mutex> lock(mApiMutex);
            notif_handle = mBase->OnNotificationActionTopic(event_cb, k_api::Common::NotificationOptions());
            mBase->ExecuteWaypointTrajectory(waypoint_list);
        } catch (k_api::KDetailedException& ex) {
            RCLCPP_ERROR(this->get_logger(), "Kortex Waypoint Error during execution: %s", ex.what());
            result->success = false;
            goal_handle->abort(result);
            return;
        } catch (const std::exception& ex) {
            // See the matching catch above ValidateWaypointList: a bare std::runtime_error
            // (e.g. an RPC timeout) here would otherwise crash the whole node.
            RCLCPP_ERROR(this->get_logger(), "Error during waypoint execution: %s", ex.what());
            result->success = false;
            goal_handle->abort(result);
            return;
        }

        auto traj_start = std::chrono::steady_clock::now();
        bool got_event = false;
        while (rclcpp::ok()) {
            if (finished_future.wait_for(std::chrono::milliseconds(100)) == std::future_status::ready) {
                got_event = true;
                break;
            }
            if (goal_handle->is_canceling()) {
                std::lock_guard<std::mutex> lock(mApiMutex);
                mBase->Stop();
                mBase->Unsubscribe(notif_handle);
                result->success = false;
                goal_handle->canceled(result);
                return;
            }
            if (std::chrono::steady_clock::now() - traj_start > std::chrono::seconds(60)) {
                RCLCPP_WARN(this->get_logger(), "Waypoint trajectory timed out after 60 seconds.");
                std::lock_guard<std::mutex> lock(mApiMutex);
                mBase->Stop();
                mBase->Unsubscribe(notif_handle);
                result->success = false;
                goal_handle->abort(result);
                return;
            }
        }

        {
            std::lock_guard<std::mutex> lock(mApiMutex);
            mBase->Unsubscribe(notif_handle);
        }

        auto event = got_event ? finished_future.get() : k_api::Base::ACTION_ABORT;
        if (event == k_api::Base::ACTION_END) {
            RCLCPP_INFO(this->get_logger(), "Waypoint trajectory completed.");
            result->success = true;
            goal_handle->succeed(result);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Waypoint trajectory aborted by robot (ACTION_ABORT).");
            result->success = false;
            goal_handle->abort(result);
        }
        return;

    } else {
        RCLCPP_INFO(this->get_logger(), "Executing single-point joint target (legacy mode)...");
        
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
    }

    // Polling / completion check loop (identical for both modes)
    auto start_time = std::chrono::steady_clock::now();
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

            // Implement a 10-second safety timeout to prevent infinite hanging when goals are rejected or unreachable
            if (std::chrono::steady_clock::now() - start_time > std::chrono::seconds(10)) {
                RCLCPP_WARN(this->get_logger(), "Joint execution timed out after 10 seconds.");
                std::lock_guard<std::mutex> lock(mApiMutex);
                mBase->Stop();
                result->success = false;
                goal_handle->succeed(result); // Return progress made
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

void Controller::computeIK(const std::shared_ptr<ros2_interfaces::srv::ComputeIK::Request> request,
                           std::shared_ptr<ros2_interfaces::srv::ComputeIK::Response> response)
{
    std::lock_guard<std::mutex> lock(mApiMutex);
    try {
        k_api::Base::IKData ik_data;
        
        // 1. Set the target Cartesian pose
        auto cartesian_pose = ik_data.mutable_cartesian_pose();
        cartesian_pose->set_x(static_cast<float>(request->x));
        cartesian_pose->set_y(static_cast<float>(request->y));
        cartesian_pose->set_z(static_cast<float>(request->z));
        cartesian_pose->set_theta_x(static_cast<float>(request->theta_x));
        cartesian_pose->set_theta_y(static_cast<float>(request->theta_y));
        cartesian_pose->set_theta_z(static_cast<float>(request->theta_z));

        // 2. Use current joint angles as the SEED for IK to find the closest solution
        auto current_joints = mBase->GetMeasuredJointAngles();
        ik_data.mutable_guess()->CopyFrom(current_joints);

        // 3. Compute IK
        auto computed_joints = mBase->ComputeInverseKinematics(ik_data);
        
        for (auto j : computed_joints.joint_angles()) {
            response->joint_angles.push_back(j.value());
        }
        response->success = true;
        response->message = "IK Successful";
        
    } catch (k_api::KDetailedException& ex) {
        response->success = false;
        response->message = "Kortex IK Error: " + std::string(ex.what());
    } catch (const std::exception& ex) {
        response->success = false;
        response->message = "Error: " + std::string(ex.what());
    }
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

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Controller>());
    rclcpp::shutdown();
    return 0;
}
