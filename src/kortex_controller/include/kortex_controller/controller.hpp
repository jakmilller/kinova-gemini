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
#include <chrono>
#include <thread>
#include <cmath>

// Custom Interfaces
#include "ros2_interfaces/msg/robot_state.hpp"
#include "ros2_interfaces/action/move_to_pose.hpp"
#include "ros2_interfaces/action/move_to_joints.hpp"
#include "ros2_interfaces/action/move_linear.hpp"
#include "ros2_interfaces/action/gripper_command.hpp"
#include "ros2_interfaces/srv/compute_ik.hpp"

namespace k_api = Kinova::Api;

class Controller : public rclcpp::Node
{
public:
    using MoveToPose = ros2_interfaces::action::MoveToPose;
    using GoalHandlePose = rclcpp_action::ServerGoalHandle<MoveToPose>;
    
    using MoveToJoints = ros2_interfaces::action::MoveToJoints;
    using GoalHandleJoints = rclcpp_action::ServerGoalHandle<MoveToJoints>;

    using MoveLinear = ros2_interfaces::action::MoveLinear;
    using GoalHandleLinear = rclcpp_action::ServerGoalHandle<MoveLinear>;

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
    rclcpp_action::Server<MoveLinear>::SharedPtr mActionLinearServer;
    rclcpp_action::Server<GripperCommand>::SharedPtr mActionGripperServer;
    rclcpp_action::Server<GripperCommand>::SharedPtr mActionGraspServer;

    // Services
    rclcpp::Service<ros2_interfaces::srv::ComputeIK>::SharedPtr mComputeIKService;

    // Publishers and Timers
    rclcpp::Publisher<ros2_interfaces::msg::RobotState>::SharedPtr mPubState;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr mPubJointState;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr mPubProtectionZones;
    rclcpp::TimerBase::SharedPtr mTimer;

    // Protection zone name lookup (zone handle identifier -> config.yaml name) and
    // the live firmware subscription that reports real-time zone crossings.
    std::unordered_map<uint32_t, std::string> mZoneNamesByHandle;
    k_api::Common::NotificationHandle mProtectionZoneNotifHandle;

    // How far past the firmware tool frame the fingertips reach when the gripper is FULLY
    // CLOSED, in meters. The Robotiq 2F-140's fingers swing forward as they close, and the
    // arm's tool offset is configured at the fully-open fingertip, so this is the full range
    // of that shift. Loaded from config.yaml's `gripper` block at startup.
    double mFingertipExtension;

    // Callbacks
    void publishState();
    float get_gripper_position();
    float get_gripper_speed();

    // --- Fingertip <-> firmware TCP geometry ---
    // The arm's tool frame is a FIXED offset set in the Web App; Kortex has no concept of
    // the fingers moving it. These convert between that fixed TCP and where the fingertips
    // physically are, so that every coordinate crossing this node's action interface means
    // "fingertips" regardless of what the gripper is doing.
    void loadGripperGeometryFromConfig();
    double fingertipExtension(float gripper_pct) const;
    void toolZAxis(float theta_x, float theta_y, float theta_z, double out[3]) const;
    k_api::Base::Pose shiftAlongToolZ(const k_api::Base::Pose& pose, double distance) const;

    // Fingertip pose (+ the gripper opening it will be commanded at) -> joint solution.
    // Shared by execute_pose and the compute_ik service so that a pose scored through the
    // service is guaranteed to produce the same joint solution when it is actually executed:
    // same TCP correction, same measured-joint IK seed, same call. Takes mApiMutex internally.
    // gripper_pct < 0 means "use the gripper's current measured position".
    // Returns false if IK found no solution; out_tcp_target is filled either way for logging.
    bool solveFingertipIK(const k_api::Base::Pose& fingertip, float gripper_pct,
                          k_api::Base::JointAngles& out_solution,
                          k_api::Base::Pose& out_tcp_target,
                          float& out_gripper_pct, std::string& out_error);

    void handleComputeIK(const std::shared_ptr<ros2_interfaces::srv::ComputeIK::Request> request,
                         std::shared_ptr<ros2_interfaces::srv::ComputeIK::Response> response);
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

    // Action Handlers (Linear / straight-line move along the tool's own +Z)
    rclcpp_action::GoalResponse handle_linear_goal(const rclcpp_action::GoalUUID &, std::shared_ptr<const MoveLinear::Goal>);
    rclcpp_action::CancelResponse handle_linear_cancel(const std::shared_ptr<GoalHandleLinear>);
    void handle_linear_accepted(const std::shared_ptr<GoalHandleLinear> goal_handle);
    void execute_linear(const std::shared_ptr<GoalHandleLinear> goal_handle);

    // Outcome of the shared Cartesian completion poll below.
    enum class PollOutcome { REACHED, CANCELLED, TIMED_OUT };

    // Shared completion check for both Cartesian actions (MoveToPose and MoveLinear).
    // Templated on the action type because the two are different generated C++ types but
    // carry the same `distance_to_go` feedback field -- one implementation keeps their
    // completion, cancellation and logging behaviour identical instead of letting two
    // near-copies drift apart.
    //
    // target_* is in RAW firmware TCP space, matching GetMeasuredCartesianPose. Callers that
    // accept a fingertip goal must convert it BEFORE calling this.
    // timeout_s <= 0 waits indefinitely (the historical MoveToPose behaviour).
    template<typename ActionT>
    PollOutcome pollUntilCartesianTarget(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> goal_handle,
        double target_x, double target_y, double target_z,
        const char* label, double timeout_s)
    {
        auto feedback = std::make_shared<typename ActionT::Feedback>();
        const auto start = std::chrono::steady_clock::now();
        int log_counter = 0;

        while (rclcpp::ok()) {
            if (goal_handle->is_canceling()) {
                std::lock_guard<std::mutex> lock(mApiMutex);
                mBase->Stop();
                RCLCPP_WARN(this->get_logger(), "%s cancelled.", label);
                return PollOutcome::CANCELLED;
            }

            k_api::Base::Pose current;
            {
                std::lock_guard<std::mutex> lock(mApiMutex);
                current = mBase->GetMeasuredCartesianPose();
            }

            double dist = sqrt(pow(target_x - current.x(), 2) +
                               pow(target_y - current.y(), 2) +
                               pow(target_z - current.z(), 2));
            feedback->distance_to_go = dist;
            goal_handle->publish_feedback(feedback);
            if (dist < 0.01) return PollOutcome::REACHED;

            if (timeout_s > 0.0) {
                double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
                if (elapsed > timeout_s) {
                    std::lock_guard<std::mutex> lock(mApiMutex);
                    mBase->Stop();
                    RCLCPP_ERROR(this->get_logger(),
                        "%s timed out after %.1f s with %.3f m still to go -- stopping the arm.",
                        label, elapsed, dist);
                    return PollOutcome::TIMED_OUT;
                }
            }

            // Log every ~2 s (20 * 100ms) so a stalled move is visible in Terminal 1.
            if (++log_counter % 20 == 0) {
                RCLCPP_INFO(this->get_logger(), "%s polling: distance_to_go = %.3f m", label, dist);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        return PollOutcome::TIMED_OUT;
    }

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