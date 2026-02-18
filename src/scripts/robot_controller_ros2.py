#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import asyncio
import yaml
import os

# Import custom action interfaces
from ros2_interfaces.action import MoveToPose, MoveToJoints, GripperCommand
from ros2_interfaces.msg import RobotState
from std_srvs.srv import SetBool

class KinovaRobotControllerROS2(Node):
    def __init__(self):
        super().__init__('kinova_robot_controller_client')

        # --- Load Configuration ---
        self.config_path = os.path.join(os.path.expanduser('~'), 'kinova-gemini', 'config.yaml')
        with open(self.config_path, 'r') as f:
            self.robot_config = yaml.safe_load(f)

        self.home = self.robot_config['joint positions']['home']

        # --- Action Clients ---
        self._action_pose_client = ActionClient(self, MoveToPose, 'move_to_pose')
        self._action_joints_client = ActionClient(self, MoveToJoints, 'move_to_joints')
        self._action_gripper_client = ActionClient(self, GripperCommand, 'gripper_command')

        # --- Service Client (for Admittance) ---
        self.admittance_client = self.create_client(SetBool, 'toggle_admittance')

        # --- Subscription (to monitor state) ---
        self.state_sub = self.create_subscription(
            RobotState, 'robot_state', self._state_callback, 10)
        self.current_state = None

        self.get_logger().info('Minimal Kinova Action Controller Initialized')

    def _state_callback(self, msg):
        self.current_state = msg

    async def move_to_pose(self, x, y, z, theta_x=0.0, theta_y=0.0, theta_z=0.0, speed=0.1):
        """Sends a MoveToPose action goal."""
        goal_msg = MoveToPose.Goal()
        goal_msg.x = x
        goal_msg.y = y
        goal_msg.z = z
        goal_msg.theta_x = theta_x
        goal_msg.theta_y = theta_y
        goal_msg.theta_z = theta_z
        goal_msg.speed_scaling = speed

        self.get_logger().info(f'Sending Pose goal: X={x}, Y={y}, Z={z}')
        return await self._send_action_goal(self._action_pose_client, goal_msg)

    async def move_to_joints(self, joint_angles: list):
        """Sends a MoveToJoints action goal (angles in degrees)."""
        goal_msg = MoveToJoints.Goal()
        goal_msg.joint_angles = [float(a) for a in joint_angles]

        self.get_logger().info(f'Sending Joint goal: {joint_angles}')
        return await self._send_action_goal(self._action_joints_client, goal_msg)
    
    async def move_to_home(self):
        """Convenience function to move to the 'home' position defined in config.yaml."""
        return await self.move_to_joints(self.home)

    async def set_gripper(self, position: float):
        """Sends a GripperCommand action goal (0-100)."""
        goal_msg = GripperCommand.Goal()
        goal_msg.position = float(position)

        self.get_logger().info(f'Sending Gripper goal: {position}')
        return await self._send_action_goal(self._action_gripper_client, goal_msg)

    async def toggle_admittance(self, enable: bool):
        """Calls the toggle_admittance service."""
        request = SetBool.Request()
        request.data = enable
        
        if not self.admittance_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Admittance service not available')
            return False
            
        result = await self.admittance_client.call_async(request)
        return result.success

    async def _send_action_goal(self, client, goal_msg):
        """Generic internal helper to send action goals and wait for results."""
        if not client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error(f'Action server {client._action_name} not available')
            return False

        send_goal_future = await client.send_goal_async(goal_msg)

        if not send_goal_future.accepted:
            self.get_logger().error('Goal rejected by server')
            return False

        self.get_logger().info('Goal accepted, waiting for result...')
        result_future = await send_goal_future.get_result_async()
        
        return result_future.result.success

async def main(args=None):
    rclpy.init(args=args)
    controller = KinovaRobotControllerROS2()
    
    # Example usage:
    # await controller.move_to_joints([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    # await controller.set_gripper(50.0)

    # Note: To keep the node alive for callbacks while using async functions, 
    # you typically use a Task or spin in a separate thread.
    
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    import sys
    asyncio.run(main(sys.argv))