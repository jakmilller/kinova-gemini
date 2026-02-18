import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from google import genai
from google.genai import types
import os
import yaml
import time
from dotenv import load_dotenv
import asyncio

# We can import the existing controller client
from .robot_controller_ros2 import KinovaRobotControllerROS2

class GeminiBrainNode(Node):
    def __init__(self):
        super().__init__('gemini_brain_node')
        
        # --- Configuration ---
        workspace_path = os.path.expanduser('~/kinova-gemini')
        load_dotenv(os.path.join(workspace_path, '.env'))

        with open(os.path.join(workspace_path, 'config.yaml'), 'r') as f:
            self.robot_config = yaml.safe_load(f)
        
        self.client = genai.Client(api_key=os.getenv('gemini_api_key'))
        self.model_id = self.robot_config['model']['name']

        # --- Robot Controller ---
        self.controller = KinovaRobotControllerROS2()
        
        # --- ROS2 Communication ---
        self.subscription = self.create_subscription(
            String,
            '/user_instructions',
            self.instruction_callback,
            10)
            
        # --- Tools (Function Definitions) ---
        self.move_to_joints_tool = types.FunctionDeclaration(
            name="move_to_joints",
            description="Moves the robot arm to specific joint angles in degrees.",
            parameters=types.Schema(
                type="OBJECT",
                properties={
                    "joint_angles": types.Schema(
                        type="ARRAY",
                        items=types.Schema(type="NUMBER"),
                        description="A list of 7 joint angles in degrees."
                    ),
                },
                required=["joint_angles"]
            )
        )
        
        self.set_gripper_tool = types.FunctionDeclaration(
            name="set_gripper",
            description="Opens or closes the robot gripper.",
            parameters=types.Schema(
                type="OBJECT",
                properties={
                    "position": types.Schema(
                        type="NUMBER",
                        description="The gripper position from 0 (fully open) to 100 (fully closed)."
                    ),
                },
                required=["position"]
            )
        )

        self.move_to_home_tool = types.FunctionDeclaration(
            name="move_to_home",
            description="Moves the robot arm to pre-defined home position. Anyim",
            parameters=types.Schema(
                type="OBJECT",
                properties={},
            )
        )

        self.tools = [types.Tool(function_declarations=[self.move_to_joints_tool, self.set_gripper_tool, self.move_to_home_tool])]
        
        self.get_logger().info('Gemini Brain Node initialized and waiting for instructions...')

    async def instruction_callback(self, msg):
        user_text = msg.data
        self.get_logger().info(f'Processing instruction: "{user_text}"')
        
        start_time = time.time()

        # Prompt construction
        prompt = f"""You are a robot controller for a Kinova Gen3 7DOF arm.
        Your task is to take the user instructions and convert them into specific robot commands, based on the function calls you have been given.
        
        User instruction: {user_text}
        """

        try:
            response = self.client.models.generate_content(
                model=self.model_id,
                contents=prompt,
                config=types.GenerateContentConfig(
                    tools=self.tools,
                )
            )

            processing_time = time.time() - start_time
            self.get_logger().info(f"Gemini processing time: {processing_time:.4f} seconds")

            for part in response.candidates[0].content.parts:
                if part.function_call:
                    await self.execute_function(part.function_call)
        
        except Exception as e:
            self.get_logger().error(f"Error during Gemini API call or execution: {str(e)}")

    async def execute_function(self, function_call):
        name = function_call.name
        args = function_call.args
        
        self.get_logger().info(f"Gemini requested function: {name} with args: {args}")
        
        success = False
        if name == "move_to_joints":
            success = await self.controller.move_to_joints(args['joint_angles'])
        elif name == "set_gripper":
            success = await self.controller.set_gripper(args['position'])
        elif name == "move_to_home":
            success = await self.controller.move_to_home()
            
        if success:
            self.get_logger().info(f"Successfully executed {name}")
        else:
            self.get_logger().error(f"Failed to execute {name}")

def main(args=None):
    rclpy.init(args=args)
    node = GeminiBrainNode()
    
    # Use a MultiThreadedExecutor to allow the async callback to run while spinning
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    executor.add_node(node.controller) # Also spin the controller node
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
