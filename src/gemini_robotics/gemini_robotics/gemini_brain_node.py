import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from tf2_ros import Buffer, TransformListener
from tf2_ros import TransformException
from cv_bridge import CvBridge
from google import genai
from google.genai import types
import os
import yaml
import time
from dotenv import load_dotenv
import asyncio
import textwrap
import json
import cv2
import numpy as np
from PIL import Image as PILImage

# We can import the existing controller client
from .robot_controller_ros2 import KinovaRobotControllerROS2
from . import vision_utils
from ros2_interfaces.msg import RobotState

class GeminiBrainNode(Node):
    def __init__(self):
        super().__init__('gemini_brain_node')
        
        # --- Configuration ---
        workspace_path = os.path.expanduser('~/kinova-gemini')
        load_dotenv(os.path.join(workspace_path, '.env'))

        with open(os.path.join(workspace_path, 'config.yaml'), 'r') as f:
            self.robot_config = yaml.safe_load(f)

        # --- Load System Prompt ---
        prompt_path = os.path.join(workspace_path, 'src', 'prompts', 'system_prompt.txt')
        with open(prompt_path, 'r') as f:
            self.system_prompt = f.read()
        
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
            
        # --- Vision/Robot State Setup ---
        self.bridge = CvBridge()
        self.latest_rgb_image = None
        self.latest_depth_image = None
        self.latest_robot_state = None
        self.latest_camera_info = None

        # TF2 Buffer and Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Subscribers for RealSense
        self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.rgb_callback,
            10
        )
        self.create_subscription(
            Image,
            '/camera/camera/aligned_depth_to_color/image_raw',
            self.depth_callback,
            10
        )
        self.create_subscription(
            CameraInfo,
            '/camera/camera/aligned_depth_to_color/camera_info',
            self.camera_info_callback,
            10
        )

        # --- Robot State ---
        self.create_subscription(
            RobotState,
            '/robot_state',
            self.robot_state_callback,
            10
        )

        self.command_start_time = 0.0

        # --- Tools (Function Definitions) ---
        self.move_to_joints_tool = types.FunctionDeclaration(
            name="move_to_joints",
            description="Moves the robot arm to specific joint angles in degrees. Arguments must be between 0-360 degrees.",
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
                        description="The gripper position from 0 (fully open) to 100 (fully closed). The gripper closes roughly 1 cm with every 10 units."
                    ),
                },
                required=["position"]
            )
        )

        self.move_to_home_tool = types.FunctionDeclaration(
            name="move_to_home",
            description="Moves the robot arm to pre-defined home position. Call anytime the user mentions resetting or going back to the home position.",
            parameters=types.Schema(
                type="OBJECT",
                properties={},
            )
        )

        self.move_to_object_tool = types.FunctionDeclaration(
            name="move_to_object",
            description="Locates an object in the robot's view and returns its coordinates.",
            parameters=types.Schema(
                type="OBJECT",
                properties={
                    "description": types.Schema(
                        type="STRING",
                        description="The name or description of the object to move to (e.g., 'red block', 'plate')."
                    ),
                },
                required=["description"]
            )
        )

        self.tools = [types.Tool(function_declarations=[self.move_to_joints_tool, self.set_gripper_tool, self.move_to_home_tool, self.move_to_object_tool])]
        
        self.get_logger().info('Gemini Brain Node initialized and waiting for instructions...')

    def rgb_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.latest_rgb_image = cv_image
        except Exception as e:
            self.get_logger().error(f"Error processing RGB image: {e}")

    def robot_state_callback(self, msg):
        self.latest_robot_state = msg

    def depth_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image (16-bit integers for depth)
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            self.latest_depth_image = cv_image
        except Exception as e:
            self.get_logger().error(f"Error processing Depth image: {e}")

    def camera_info_callback(self, msg):
        self.latest_camera_info = msg

    async def instruction_callback(self, msg):
        user_text = msg.data
        self.get_logger().info(f'Processing instruction: "{user_text}"')
        
        start_time = time.time()
        self.command_start_time = start_time # Track total command time

        # Build Robot State String
        if self.latest_robot_state:
            s = self.latest_robot_state
            # Format joint angles to be more readable (e.g., list of floats)
            formatted_joints = [round(x, 3) for x in s.joint_angles]
            
            state_str = textwrap.dedent(f"""
                Current Robot State:
                - Joint Angles (degrees): {formatted_joints}
                - Tool Pose (x, y, z): ({s.x:.3f}, {s.y:.3f}, {s.z:.3f})
                - Tool Orientation (theta_x, theta_y, theta_z): ({s.theta_x:.3f}, {s.theta_y:.3f}, {s.theta_z:.3f})
                - Gripper Position (0-100): {s.gripper_position:.1f}
                """)
        else:
            state_str = "Current Robot State: Unknown (Waiting for /robot_state topic...)"

        # Combine System Prompt + State + User Instruction
        prompt = f"{self.system_prompt}\n\n{state_str}\n\nUser Instruction: {user_text}"

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
        elif name == "move_to_object":
            await self.move_to_object(args['description'])
            success = True # Assume success if no exception, logic inside handles logging
            
        if success:
            self.get_logger().info(f"Successfully executed {name}")
        else:
            self.get_logger().error(f"Failed to execute {name}")

    async def move_to_object(self, description):
        self.get_logger().info(f"Locating object: {description}")

        # note to self: handle case where the model cant find the requested object
        
        # 1. Capture Image (Wait for fresh data)
        for _ in range(10):
            if self.latest_rgb_image is not None and self.latest_depth_image is not None:
                break
            self.get_logger().info("Waiting for images from RealSense...")
            await asyncio.sleep(0.5)
            
        if self.latest_rgb_image is None:
             self.get_logger().error("No RGB image received from RealSense.")
             return

        # Convert OpenCV BGR to PIL RGB
        cv_image_rgb = cv2.cvtColor(self.latest_rgb_image, cv2.COLOR_BGR2RGB)
        pil_img = PILImage.fromarray(cv_image_rgb)
        
        # resize PIL to be consistent with Gemini examples
        width, height = pil_img.size
        new_width = 800
        new_height = int(new_width * height / width)
        img_resized = pil_img.resize((new_width, new_height), PILImage.Resampling.LANCZOS)

        # Store the depth map (associated with this capture)
        captured_depth = self.latest_depth_image.copy()

        # 2. Query Gemini (Requesting Segmentation Masks)
        prompt = textwrap.dedent(f"""\
            Return segmentation masks for: {description}.
            The answer should follow the JSON format:
            [{{"box_2d": [ymin, xmin, ymax, xmax], "mask": "png_base64_str", "label": "{description}"}}]
            
            The coordinates are normalized to 0-1000.
            """)
        
        try:
            # We use the same client/model to query for vision
            response = self.client.models.generate_content(
                model=self.model_id,
                contents=[img_resized, prompt],
                config=types.GenerateContentConfig(
                    temperature=0.5,
                )
            )
            
            # 3. Parse and Visualize
            json_output = response.text
            self.get_logger().info(f"Gemini Vision Response: {json_output}")
            
            # Visualize Segmentation Masks (Pop up window)
            try:
                masks = vision_utils.parse_segmentation_masks(json_output, img_height=new_height, img_width=new_width)
                
                # Calculate total time
                total_time = time.time() - self.command_start_time
                self.get_logger().info(f"Total time from command to visualization: {total_time:.4f} seconds")

                if not masks:
                    self.get_logger().warn(f"No objects found for {description}")
                    return

                # 4. Process Mask for Depth and Visualization on ORIGINAL Image
                mask_obj = masks[0] # Take the first mask
                
                # Resize the mask to match the original image dimensions
                orig_h, orig_w = captured_depth.shape
                # mask_obj.mask is (new_height, new_width)
                # cv2.resize expects (width, height)
                upscaled_mask = cv2.resize(mask_obj.mask, (orig_w, orig_h), interpolation=cv2.INTER_NEAREST)
                
                # Ensure it's binary (0 or 255)
                _, binary_mask_orig = cv2.threshold(upscaled_mask, 127, 255, cv2.THRESH_BINARY)
                
                # --- Depth Calculation (Median of Mask) ---
                # Create a boolean mask where the object is
                object_mask_bool = (binary_mask_orig > 0)
                
                # Extract depth values within the mask
                depth_values = captured_depth[object_mask_bool]
                
                # Filter out invalid depth values (0 often indicates no data in RealSense)
                valid_depth_values = depth_values[depth_values > 0]
                
                if len(valid_depth_values) == 0:
                    self.get_logger().warn("No valid depth values found in the masked region.")
                    depth_val = 0
                else:
                    depth_val = np.median(valid_depth_values)
                
                # Calculate Centroid (for reporting/logic if needed, though depth is now median)
                M = cv2.moments(binary_mask_orig)
                if M["m00"] != 0:
                    pixel_x = int(M["m10"] / M["m00"])
                    pixel_y = int(M["m01"] / M["m00"])
                else:
                    # Fallback to bbox center of the resized mask scaled up
                    pixel_x = int(((mask_obj.x0 + mask_obj.x1) / 2) * (orig_w / new_width))
                    pixel_y = int(((mask_obj.y0 + mask_obj.y1) / 2) * (orig_h / new_height))

                # --- VISUALIZATION: Pop-up the Original Image with Mask ---
                if self.latest_rgb_image is not None:
                    final_img = self.latest_rgb_image.copy()

                    # Create a red overlay
                    mask_overlay = np.zeros_like(final_img)
                    mask_overlay[object_mask_bool] = [0, 0, 255] # Red (BGR)
                    
                    # Weighted sum to make it transparent
                    cv2.addWeighted(mask_overlay, 0.5, final_img, 1.0, 0, final_img)
                    
                    # Draw the centroid as a green circle for reference
                    cv2.circle(final_img, (pixel_x, pixel_y), 10, (0, 255, 0), -1)

                    # BETTER POP-UP: Convert to PIL and use .show() 
                    # This opens the system's default image viewer and DOES NOT block ROS.
                    final_pil = PILImage.fromarray(cv2.cvtColor(final_img, cv2.COLOR_BGR2RGB))
                    final_pil.show(title=f"Locate Result: {description}")

                    self.get_logger().info(f"Visualized {description} at ({pixel_x}, {pixel_y}) on original resolution.")
                    self.get_logger().info(f"Median Depth of Segment: {depth_val:.2f} mm")

                # --- Camera Intrinsics & Coordinate Transformation ---
                if self.latest_camera_info is None:
                    self.get_logger().error("Camera info not received yet. Cannot transform to 3D.")
                    return

                # Convert depth to meters
                depth_meters = depth_val / 1000.0

                if depth_meters == 0:
                    self.get_logger().error("Invalid depth for transformation. Aborting move.")
                    return

                # 1. Pixel to 3D in Camera Frame
                cam_x, cam_y, cam_z = vision_utils.get_3d_point_from_pixel(
                    pixel_x, pixel_y, depth_meters, self.latest_camera_info
                )
                self.get_logger().info(f"Object 3D coordinates in realsense_link: ({cam_x:.3f}, {cam_y:.3f}, {cam_z:.3f})")

                # 2. Transform Camera Frame to Base Frame using TF2
                try:
                    # Look up transform from base_link to realsense_link
                    # Target frame is base_link, source frame is realsense_link
                    transform = self.tf_buffer.lookup_transform(
                        'base_link',
                        'realsense_link',
                        rclpy.time.Time(),
                        timeout=rclpy.duration.Duration(seconds=1.0)
                    )
                    
                    # Apply transform
                    base_x, base_y, base_z = vision_utils.transform_point(
                        cam_x, cam_y, cam_z, transform
                    )
                    self.get_logger().info(f"Object 3D coordinates in base_link: ({base_x:.3f}, {base_y:.3f}, {base_z:.3f})")

                    # 3. Move Robot
                    if self.latest_robot_state is None:
                        self.get_logger().error("Robot state not available. Cannot get current orientation.")
                        return

                    # Get current orientation to maintain it
                    current_theta_x = self.latest_robot_state.theta_x
                    current_theta_y = self.latest_robot_state.theta_y
                    current_theta_z = self.latest_robot_state.theta_z
                    
                    self.get_logger().info(f"Sending move_to_pose command to base_link coordinates.")
                    
                    success = await self.controller.move_to_pose(
                        base_x, base_y, base_z,
                        current_theta_x, current_theta_y, current_theta_z
                    )
                    
                    if success:
                        self.get_logger().info("Successfully moved to object.")
                    else:
                        self.get_logger().error("Failed to move to object.")

                except TransformException as ex:
                    self.get_logger().error(f"Could not transform realsense_link to base_link: {ex}")
                    return

            except Exception as e:
                self.get_logger().error(f"Error parsing or plotting masks: {e}")
                # Fallback to bounding box if parsing fails? 
                # For now, let's stick to the requested mask workflow.

        except Exception as e:
            self.get_logger().error(f"Error in vision pipeline: {e}")



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
