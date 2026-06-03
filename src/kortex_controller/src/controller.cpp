#include "kortex_controller/controller.hpp"
#include <cmath>
#include <algorithm>
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

        while (rclcpp::ok()) {
            if (goal_handle->is_canceling()) {
                std::lock_guard<std::mutex> lock(mApiMutex);
                mBase->Stop();
                result->success = false;
                goal_handle->canceled(result);
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

    k_api::Base::Action action;
    action.set_name("Joint Move");
    auto reach_joints = action.mutable_reach_joint_angles();
    auto joints = reach_joints->mutable_joint_angles();

    for (size_t i = 0; i < goal->joint_angles.size(); ++i) {
        auto j = joints->add_joint_angles();
        j->set_joint_identifier(i);
        j->set_value(static_cast<float>(goal->joint_angles[i]));
    }

    try {
        {
            std::lock_guard<std::mutex> lock(mApiMutex);
            mBase->ExecuteAction(action);
        }

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
                float target = static_cast<float>(goal->joint_angles[i]);
                float actual = current_joints.joint_angles(i).value();
                float diff = std::abs(target - actual);
                // Handle 360 degree wrapping
                if (diff > 180.0f) {
                    diff = 360.0f - diff;
                }
                max_diff = std::max(max_diff, diff);
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
        mPubJointState->publish(jmsg);
    } catch (...) {}
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Controller>());
    rclcpp::shutdown();
    return 0;
}
