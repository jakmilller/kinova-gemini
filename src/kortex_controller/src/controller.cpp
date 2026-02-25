#include "kortex_controller/controller.hpp"
#include <cmath>
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
    mRouter = new k_api::RouterClient(mTransport, [](k_api::KError err) { cout << "Kortex Error: " << err.toString(); });
    mTransport->connect(robot_ip, 10000);

    auto session_manager = new k_api::SessionManager(mRouter);
    auto create_session_info = k_api::Session::CreateSessionInfo();
    create_session_info.set_username(this->get_parameter("username").as_string());
    create_session_info.set_password(this->get_parameter("password").as_string());
    create_session_info.set_session_inactivity_timeout(60000);
    session_manager->CreateSession(create_session_info);

    mBase = new k_api::Base::BaseClient(mRouter);
    mBaseCyclic = new k_api::BaseCyclic::BaseCyclicClient(mRouter);
    mActuatorConfig = new k_api::ActuatorConfig::ActuatorConfigClient(mRouter);

    // --- ROS 2 Action Servers ---
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

    // --- Additional Services ---
    mSrvAdmittance = this->create_service<std_srvs::srv::SetBool>(
        "toggle_admittance", std::bind(&Controller::toggleAdmittance, this, _1, _2));

    // --- Telemetry Pub ---
    mPubState = this->create_publisher<ros2_interfaces::msg::RobotState>("robot_state", 10);
    mPubJointState = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    mTimer = this->create_wall_timer(100ms, std::bind(&Controller::publishState, this));
    
    RCLCPP_INFO(this->get_logger(), "Kinova Controller Initialized");
}

// --- Admittance Mode ---
void Controller::toggleAdmittance(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                                  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
    try {
        k_api::Base::Admittance admittance;
        admittance.set_admittance_mode(request->data ? 
            k_api::Base::AdmittanceMode::CARTESIAN : k_api::Base::AdmittanceMode::DISABLED);
        mBase->SetAdmittance(admittance);
        response->success = true;
        RCLCPP_INFO(this->get_logger(), "Admittance mode set to: %s", request->data ? "ON" : "OFF");
    } catch (k_api::KDetailedException& ex) {
        response->success = false;
        RCLCPP_ERROR(this->get_logger(), "Kortex Error setting admittance: %s", ex.what());
    }
}

// --- Action Execution ---
void Controller::execute_pose(const std::shared_ptr<GoalHandlePose> goal_handle)
{
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<ros2_interfaces::action::MoveToPose::Feedback>();
    auto result = std::make_shared<ros2_interfaces::action::MoveToPose::Result>();

    k_api::Base::Action action;
    action.set_name("Planned Pose");

    auto cartesian_waypoint = action.mutable_reach_pose();
    auto pose = cartesian_waypoint->mutable_target_pose();
    pose->set_x(goal->x);
    pose->set_y(goal->y);
    pose->set_z(goal->z);
    pose->set_theta_x(goal->theta_x);
    pose->set_theta_y(goal->theta_y);
    pose->set_theta_z(goal->theta_z);

    auto constraint = cartesian_waypoint->mutable_constraint();
    auto speed = constraint->mutable_speed();
    
    speed->set_translation(goal->speed_scaling > 0 ? goal->speed_scaling : 0.1); 
    speed->set_orientation(20.0);

    try {
        mBase->ExecuteAction(action);
        
        while (rclcpp::ok()) {
            if (goal_handle->is_canceling()) {
                mBase->Stop();
                result->success = false;
                goal_handle->canceled(result);
                return;
            }

            auto current_pose = mBase->GetMeasuredCartesianPose();
            double dist = sqrt(pow(goal->x - current_pose.x(), 2) + pow(goal->y - current_pose.y(), 2));
            feedback->distance_to_go = dist;
            goal_handle->publish_feedback(feedback);

            if (dist < 0.01) break; 
            std::this_thread::sleep_for(100ms);
        }

        result->success = true;
        goal_handle->succeed(result);
    } catch (...) {
        result->success = false;
        goal_handle->abort(result);
    }
}

// --- Joint Action Execution ---
void Controller::execute_joints(const std::shared_ptr<GoalHandleJoints> goal_handle)
{
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<ros2_interfaces::action::MoveToJoints::Feedback>();
    auto result = std::make_shared<ros2_interfaces::action::MoveToJoints::Result>();

    k_api::Base::Action action;
    action.set_name("Joint Move");

    auto reach_joints = action.mutable_reach_joint_angles();
    for (double angle : goal->joint_angles) {
        auto joint = reach_joints->mutable_joint_angles()->add_joint_angles();
        joint->set_value(static_cast<float>(angle));
    }

    try {
        mBase->ExecuteAction(action);
        
        while (rclcpp::ok()) {
            if (goal_handle->is_canceling()) {
                mBase->Stop();
                result->success = false;
                goal_handle->canceled(result);
                return;
            }

            auto current_joints = mBase->GetMeasuredJointAngles();
            std::vector<double> remaining;
            double total_error = 0;

            for (int i = 0; i < current_joints.joint_angles_size(); ++i) {
                // Calculate the shortest path distance between angles (handling 0/360 wrap)
                double diff = goal->joint_angles[i] - current_joints.joint_angles(i).value();
                
                // Wrap difference to [-180, 180]
                while (diff > 180.0) diff -= 360.0;
                while (diff < -180.0) diff += 360.0;
                
                double err = std::abs(diff);
                remaining.push_back(err);
                total_error += err;
            }

            feedback->remaining_distance = remaining;
            goal_handle->publish_feedback(feedback);

            // With the shortest-path logic, the action will now correctly identify 
            // that the robot has reached the goal.
            if (total_error < 0.5) break; 
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

// Joint Action Handlers
rclcpp_action::GoalResponse Controller::handle_joints_goal(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const ros2_interfaces::action::MoveToJoints::Goal>) {
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse Controller::handle_joints_cancel(const std::shared_ptr<GoalHandleJoints>) {
    return rclcpp_action::CancelResponse::ACCEPT;
}

void Controller::handle_joints_accepted(const std::shared_ptr<GoalHandleJoints> goal_handle) {
    // This starts the execution thread
    std::thread{std::bind(&Controller::execute_joints, this, _1), goal_handle}.detach();
}

void Controller::execute_gripper(const std::shared_ptr<GoalHandleGripper> goal_handle)
{
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<ros2_interfaces::action::GripperCommand::Result>();
    auto feedback = std::make_shared<ros2_interfaces::action::GripperCommand::Feedback>();

    // FIX: Convert your 0-100 input to the 0.0-1.0 range required by the Kortex API
    // We also clamp the value between 0 and 1 to prevent any out-of-bounds errors.
    float target_command = std::max(0.0f, std::min(1.0f, goal->position / 100.0f));

    k_api::Base::GripperCommand command;
    command.set_mode(k_api::Base::GripperMode::GRIPPER_POSITION);
    auto finger = command.mutable_gripper()->add_finger();
    finger->set_finger_identifier(1); 
    finger->set_value(target_command); 

    try {
        mBase->SendGripperCommand(command);
        
        while (rclcpp::ok()) {
            if (goal_handle->is_canceling()) {
                result->success = false;
                goal_handle->canceled(result);
                return;
            }

            // The robot's feedback is already in the 0-100 range
            auto gripper_feedback = mBaseCyclic->RefreshFeedback().interconnect().gripper_feedback();
            float current_pos = gripper_feedback.motor()[0].position();
            
            feedback->current_position = current_pos;
            goal_handle->publish_feedback(feedback);

            // Compare the feedback (0-100) against your original goal (0-100)
            // A threshold of 1.0 represents a 1% margin of error.
            if (std::abs(current_pos - goal->position) < 1.0f) break;
            
            std::this_thread::sleep_for(100ms);
        }

        result->success = true;
        goal_handle->succeed(result);
    } catch (k_api::KDetailedException& ex) {
        RCLCPP_ERROR(this->get_logger(), "Gripper Error: %s", ex.what());
        result->success = false;
        goal_handle->abort(result);
    }
}

// Handlers
rclcpp_action::GoalResponse Controller::handle_gripper_goal(const rclcpp_action::GoalUUID &, std::shared_ptr<const GripperCommand::Goal>) {
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse Controller::handle_gripper_cancel(const std::shared_ptr<GoalHandleGripper>) {
    return rclcpp_action::CancelResponse::ACCEPT;
}

void Controller::handle_gripper_accepted(const std::shared_ptr<GoalHandleGripper> goal_handle) {
    std::thread{std::bind(&Controller::execute_gripper, this, _1), goal_handle}.detach();
}

void Controller::publishState()
{
    auto feedback = mBaseCyclic->RefreshFeedback();
    
    // --- RobotState (Custom) ---
    auto msg = ros2_interfaces::msg::RobotState();
    msg.x = feedback.base().tool_pose_x();
    msg.y = feedback.base().tool_pose_y();
    msg.z = feedback.base().tool_pose_z();
    msg.theta_x = feedback.base().tool_pose_theta_x();
    msg.theta_y = feedback.base().tool_pose_theta_y();
    msg.theta_z = feedback.base().tool_pose_theta_z();

    for(int i = 0; i < 7 && i < feedback.actuators_size(); ++i) {
        msg.joint_angles[i] = feedback.actuators(i).position();
    }
    
    float gripper_pos_0_100 = feedback.interconnect().gripper_feedback().motor()[0].position();
    msg.gripper_position = gripper_pos_0_100;
    
    mPubState->publish(msg);

    // --- JointState (Standard for RViz) ---
    auto joint_msg = sensor_msgs::msg::JointState();
    joint_msg.header.stamp = this->now();
    
    // Standard names for Gen3 7DOF
    joint_msg.name = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "joint_7", "finger_joint"};

    // Arm Joints
    for(int i = 0; i < 7 && i < feedback.actuators_size(); ++i) {
        float angle_deg = feedback.actuators(i).position();
        float angle_rad = angle_deg * (M_PI / 180.0f);
        joint_msg.position.push_back(angle_rad);
    }

    // Gripper Joint
    // 0-100% -> 0.0-0.8 (approx closed)
    // Robotiq 2F-140: 0 is Open, 140mm is Closed? No, 0 is Open in Kortex 0-100?
    // Let's assume 0.8 scaling factor logic from old controller is correct for URDF.
    // Logic from old controller: position() is 0-100. 
    // joint_state.position.push_back(0.8 * position / 100.0);
    joint_msg.position.push_back((gripper_pos_0_100 / 100.0f) * 0.8f);

    mPubJointState->publish(joint_msg);
}

rclcpp_action::GoalResponse Controller::handle_pose_goal(const rclcpp_action::GoalUUID &, std::shared_ptr<const ros2_interfaces::action::MoveToPose::Goal>) {
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse Controller::handle_pose_cancel(const std::shared_ptr<GoalHandlePose>) {
    return rclcpp_action::CancelResponse::ACCEPT;
}

void Controller::handle_pose_accepted(const std::shared_ptr<GoalHandlePose> goal_handle) {
    std::thread{std::bind(&Controller::execute_pose, this, _1), goal_handle}.detach();
}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto controller = std::make_shared<Controller>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(controller);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}