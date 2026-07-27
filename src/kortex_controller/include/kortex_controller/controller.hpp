#pragma once

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// Kortex API
#include <BaseClientRpc.h>
#include <BaseCyclicClientRpc.h>
#include <SessionManager.h>
#include <RouterClient.h>
#include <TransportClientTcp.h>
#include <mutex>
#include <unordered_map>

// Custom Interfaces
#include "ros2_interfaces/msg/robot_state.hpp"
#include "ros2_interfaces/action/move_to_pose.hpp"
#include "ros2_interfaces/action/move_to_joints.hpp"
#include "ros2_interfaces/action/gripper_command.hpp"

namespace k_api = Kinova::Api;

class Controller : public rclcpp::Node
{
public:
    using MoveToPose = ros2_interfaces::action::MoveToPose;
    using GoalHandlePose = rclcpp_action::ServerGoalHandle<MoveToPose>;
    
    using MoveToJoints = ros2_interfaces::action::MoveToJoints;
    using GoalHandleJoints = rclcpp_action::ServerGoalHandle<MoveToJoints>;

    using GripperCommand = ros2_interfaces::action::GripperCommand;
    using GoalHandleGripper = rclcpp_action::ServerGoalHandle<GripperCommand>;

    Controller();
    
private:
    // Kortex Members
    k_api::TransportClientTcp* mTransport;
    k_api::RouterClient* mRouter;
    k_api::Base::BaseClient* mBase;
    k_api::BaseCyclic::BaseCyclicClient* mBaseCyclic;

    // Mutex for thread-safe API access
    std::mutex mApiMutex;

    // Action Servers
    rclcpp_action::Server<MoveToPose>::SharedPtr mActionPoseServer;
    rclcpp_action::Server<MoveToJoints>::SharedPtr mActionJointsServer;
    rclcpp_action::Server<GripperCommand>::SharedPtr mActionGripperServer;
    rclcpp_action::Server<GripperCommand>::SharedPtr mActionGraspServer;

    // Publishers and Timers
    rclcpp::Publisher<ros2_interfaces::msg::RobotState>::SharedPtr mPubState;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr mPubJointState;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr mPubProtectionZones;
    rclcpp::TimerBase::SharedPtr mTimer;

    // Protection zone name lookup (zone handle identifier -> config.yaml name) and
    // the live firmware subscription that reports real-time zone crossings.
    std::unordered_map<uint32_t, std::string> mZoneNamesByHandle;
    k_api::Common::NotificationHandle mProtectionZoneNotifHandle;

    // Callbacks
    void publishState();
    float get_gripper_position();
    void configureProtectionZonesFromConfig();
    void publishProtectionZones();
    void subscribeToProtectionZoneEvents();
    void onProtectionZoneNotification(k_api::Base::ProtectionZoneNotification notification);

    // Action Handlers (Pose)
    rclcpp_action::GoalResponse handle_pose_goal(const rclcpp_action::GoalUUID &, std::shared_ptr<const MoveToPose::Goal>);
    rclcpp_action::CancelResponse handle_pose_cancel(const std::shared_ptr<GoalHandlePose>);
    void handle_pose_accepted(const std::shared_ptr<GoalHandlePose> goal_handle);
    void execute_pose(const std::shared_ptr<GoalHandlePose> goal_handle);

    // Action Handlers (Joints)
    rclcpp_action::GoalResponse handle_joints_goal(const rclcpp_action::GoalUUID &, std::shared_ptr<const MoveToJoints::Goal>);
    rclcpp_action::CancelResponse handle_joints_cancel(const std::shared_ptr<GoalHandleJoints>);
    void handle_joints_accepted(const std::shared_ptr<GoalHandleJoints> goal_handle);
    void execute_joints(const std::shared_ptr<GoalHandleJoints> goal_handle);

    // Action Handlers (Gripper)
    rclcpp_action::GoalResponse handle_gripper_goal(const rclcpp_action::GoalUUID &, std::shared_ptr<const GripperCommand::Goal>);
    rclcpp_action::CancelResponse handle_gripper_cancel(const std::shared_ptr<GoalHandleGripper>);
    void handle_gripper_accepted(const std::shared_ptr<GoalHandleGripper> goal_handle);
    void execute_gripper(const std::shared_ptr<GoalHandleGripper> goal_handle);

    // Action Handlers (Grasp Object)
    rclcpp_action::GoalResponse handle_grasp_goal(const rclcpp_action::GoalUUID &, std::shared_ptr<const GripperCommand::Goal>);
    rclcpp_action::CancelResponse handle_grasp_cancel(const std::shared_ptr<GoalHandleGripper>);
    void handle_grasp_accepted(const std::shared_ptr<GoalHandleGripper> goal_handle);
    void execute_grasp(const std::shared_ptr<GoalHandleGripper> goal_handle);
};