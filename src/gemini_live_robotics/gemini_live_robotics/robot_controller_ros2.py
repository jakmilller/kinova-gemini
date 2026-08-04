#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import asyncio
import yaml
import os

# Import custom action interfaces
from ros2_interfaces.action import MoveToPose, MoveToJoints, MoveLinear, GripperCommand
from ros2_interfaces.srv import ComputeIK
from ros2_interfaces.msg import RobotState


class KinovaRobotControllerROS2(Node):
    def __init__(self):
        super().__init__('kinova_robot_controller_client')

        # --- Load Configuration ---
        self.config_path = os.path.join(os.path.expanduser('~'), 'kinova-gemini', 'config.yaml')
        with open(self.config_path, 'r') as f:
            self.robot_config = yaml.safe_load(f)

        self.home = self.robot_config['joint positions']['home']
        self.user_position = self.robot_config['joint positions']['user']

        # --- Action Clients ---
        self._action_pose_client = ActionClient(self, MoveToPose, 'move_to_pose')
        self._action_joints_client = ActionClient(self, MoveToJoints, 'move_to_joints')
        self._action_linear_client = ActionClient(self, MoveLinear, 'move_linear')
        self._action_gripper_client = ActionClient(self, GripperCommand, 'gripper_command')
        self._action_grasp_object = ActionClient(self, GripperCommand, 'grasp_object')

        # --- Service Clients ---
        self._compute_ik_client = self.create_client(ComputeIK, 'compute_ik')

        # --- Subscription (to monitor state) ---
        self.state_sub = self.create_subscription(
            RobotState, 'robot_state', self._state_callback, 10)
        self.current_state = None

        # --- Active Goals for Cancellation ---
        self._active_goals = []

        self.get_logger().info('Minimal Kinova Action Controller Initialized')

    def _state_callback(self, msg):
        self.current_state = msg

    def get_joint_angles(self):
        """Live joint angles (degrees, Kortex 0-360 convention) as a list of 7, or None if
        robot state has not arrived yet."""
        if self.current_state is None:
            return None
        return list(self.current_state.joint_angles)

    async def _await_rclpy_future(self, rclpy_future):
        """Converts an rclpy.task.Future to an asyncio.Future to prevent deadlocking the asyncio event loop."""
        loop = asyncio.get_running_loop()
        asyncio_future = loop.create_future()

        def callback(future):
            def set_res():
                if not asyncio_future.done():
                    asyncio_future.set_result(future.result())
            loop.call_soon_threadsafe(set_res)

        rclpy_future.add_done_callback(callback)
        return await asyncio_future

    async def move_to_pose(self, x, y, z, theta_x=0.0, theta_y=0.0, theta_z=0.0, speed=0.0):
        """Sends a MoveToPose action goal. x/y/z are FINGERTIP coordinates: the controller
        corrects for how far the 2F-140's tips reach at the current gripper opening, so the
        fingers land here whether they are open or closed.

        This guarantees the ENDPOINT only -- the controller solves IK and moves through joint
        space, so the path between here and there is not a straight line. Use move_linear when
        the path matters (e.g. approaching an object that is already close to the jaws).

        speed is currently IGNORED: it was a Cartesian translation ceiling back when this action
        used Kortex's reach_pose, but the joint-space move runs at the firmware's
        ANGULAR_TRAJECTORY speed, which is set in the Kinova Web App rather than here.
        """
        goal_msg = MoveToPose.Goal()
        goal_msg.x = x
        goal_msg.y = y
        goal_msg.z = z
        goal_msg.theta_x = theta_x
        goal_msg.theta_y = theta_y
        goal_msg.theta_z = theta_z
        goal_msg.speed_scaling = float(speed)

        self.get_logger().info(f'Sending Pose goal: X={x}, Y={y}, Z={z}')
        return await self._send_action_goal(self._action_pose_client, goal_msg)

    async def compute_ik(self, x, y, z, theta_x, theta_y, theta_z, gripper_percent=-1.0):
        """Ask the controller what joint angles reach this FINGERTIP pose, without moving.

        Returns (success, joint_angles_degrees) — success=False means the arm cannot reach the
        pose, which is a normal answer when filtering grasp candidates, not an error.

        gripper_percent is the opening the gripper WILL be at for the move (the fingertip->TCP
        correction depends on it); leave it negative to use the gripper's current position.
        Because the controller solves this with the same helper and the same measured-joint seed
        that move_to_pose uses, the angles returned here are the ones that move will produce.
        """
        if not self._compute_ik_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('compute_ik service not available')
            return False, None

        request = ComputeIK.Request()
        request.x = float(x)
        request.y = float(y)
        request.z = float(z)
        request.theta_x = float(theta_x)
        request.theta_y = float(theta_y)
        request.theta_z = float(theta_z)
        request.gripper_percent = float(gripper_percent)

        response = await self._await_rclpy_future(self._compute_ik_client.call_async(request))
        if not response.success:
            return False, None
        return True, list(response.joint_angles)

    async def move_linear(self, distance: float, speed: float = 0.0):
        """Straight-line Cartesian move along the tool's own +Z, holding the current orientation.

        distance is signed and in meters: positive drives the fingertips forward (the grasp
        approach), negative backs them off (the retreat). Unlike move_to_pose -- which only
        guarantees the endpoint, since it moves through joint space -- this guarantees the PATH,
        which is what makes it safe to run with an object a few centimeters in front of the jaws.

        speed is an m/s translation ceiling; 0.0 uses the controller's default approach speed.
        """
        goal_msg = MoveLinear.Goal()
        goal_msg.distance = float(distance)
        goal_msg.speed = float(speed)

        self.get_logger().info(f'Sending Linear goal: {distance:+.3f} m along tool +Z')
        return await self._send_action_goal(self._action_linear_client, goal_msg)

    async def move_to_joints(self, joint_angles: list):
        """Sends a MoveToJoints action goal (angles in degrees)."""
        goal_msg = MoveToJoints.Goal()
        goal_msg.joint_angles = [float(a) for a in joint_angles]

        self.get_logger().info(f'Sending Joint goal: {joint_angles}')
        return await self._send_action_goal(self._action_joints_client, goal_msg)

    async def move_to_home(self):
        """Convenience function to move to the 'home' position defined in config.yaml."""
        return await self.move_to_joints(self.home)

    async def move_to_user(self):
        """Moves the robot to a pre-defined position near the user."""
        return await self.move_to_joints(self.user_position)

    async def set_gripper(self, position: float):
        """Sends a GripperCommand action goal (0-100)."""
        goal_msg = GripperCommand.Goal()
        goal_msg.position = float(position)

        self.get_logger().info(f'Sending Gripper goal: {position}')
        return await self._send_action_goal(self._action_gripper_client, goal_msg)

    async def grasp_object(self):
        """Closes the gripper until it detects contact with an object"""
        goal_msg = GripperCommand.Goal()
        goal_msg.position = 99.0  # try to full close, but the action server will stop when it detects contact
        return await self._send_action_goal(self._action_grasp_object, goal_msg)

    async def stop_robot(self):
        """Cancels any active goals to stop the physical robot safely."""
        self.get_logger().info('Stopping the robot by canceling all active goals.')
        for goal_handle in self._active_goals:
            try:
                await self._await_rclpy_future(goal_handle.cancel_goal_async())
            except Exception as e:
                self.get_logger().error(f'Error canceling goal: {e}')
        self._active_goals.clear()
        return True

    async def _send_action_goal(self, client, goal_msg):
        """Generic internal helper to send action goals and wait for results."""
        if not client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error(f'Action server {client._action_name} not available')
            return False

        goal_handle = await self._await_rclpy_future(client.send_goal_async(goal_msg))

        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by server')
            return False

        self._active_goals.append(goal_handle)

        self.get_logger().info('Goal accepted, waiting for result...')
        result = await self._await_rclpy_future(goal_handle.get_result_async())

        if goal_handle in self._active_goals:
            self._active_goals.remove(goal_handle)

        return result.result.success
