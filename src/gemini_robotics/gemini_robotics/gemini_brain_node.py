import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
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
import subprocess
from dotenv import load_dotenv
import asyncio
import textwrap
import json
import cv2
import numpy as np
import base64
from io import BytesIO
from PIL import Image as PILImage, ImageDraw, ImageFont, ImageColor

# We can import the existing controller client
from .robot_controller_ros2 import KinovaRobotControllerROS2
from . import vision_utils
from . import gemini_tools
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

        # for general tasks
        self.flash_model_id = self.robot_config['model']['flash']

        # for tasks requiring embodied reasoning/vision
        self.robot_model_id = self.robot_config['model']['robot']

        # --- Robot Controller ---
        self.controller = KinovaRobotControllerROS2()
        
        # --- ROS2 Communication ---
        self.subscription = self.create_subscription(
            String,
            '/user_instructions',
            self.instruction_callback,
            10)
            
        self.status_pub = self.create_publisher(String, '/brain_status', 10)
        self.chat_pub = self.create_publisher(String, '/gemini_chat', 10)
        self.semantic_pub = self.create_publisher(String, '/semantic_position', 10)
            
        # --- Vision/Robot State Setup ---
        self.state_cb_group = MutuallyExclusiveCallbackGroup()
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
            '/camera/realsense/color/image_raw',
            self.rgb_callback,
            10,
            callback_group=self.state_cb_group
        )
        self.create_subscription(
            Image,
            '/camera/realsense/aligned_depth_to_color/image_raw',
            self.depth_callback,
            10,
            callback_group=self.state_cb_group
        )
        self.create_subscription(
            CameraInfo,
            '/camera/realsense/aligned_depth_to_color/camera_info',
            self.camera_info_callback,
            10,
            callback_group=self.state_cb_group
        )

        # --- Robot State ---
        self.create_subscription(
            RobotState,
            '/robot_state',
            self.robot_state_callback,
            10,
            callback_group=self.state_cb_group
        )

        self.command_start_time = 0.0

        # --- Tools (Function Definitions) ---
        self.tools = [types.Tool(function_declarations=gemini_tools.ALL_TOOLS)]

        self.chat_session = self.client.chats.create(
            model=self.robot_model_id,
            config=types.GenerateContentConfig(
                system_instruction=self.system_prompt,
                tools=self.tools,
                temperature=0.0,
            )
        )
        
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
        # Publish semantic position to UI
        semantic_pos = self.get_semantic_position()
        self.semantic_pub.publish(String(data=semantic_pos))

    def get_semantic_position(self):
        """Maps current joint angles to a semantic name from config.yaml."""
        if not self.latest_robot_state:
            return "Unknown"
        
        current_joints = list(self.latest_robot_state.joint_angles)
        threshold = 5.0 # degrees tolerance
        
        for name, pos in self.robot_config.get('joint positions', {}).items():
            if len(pos) == len(current_joints):
                # Calculate absolute difference for each joint
                diffs = [abs(a - b) for a, b in zip(current_joints, pos)]
                # Handle 360-degree wrapping for joints
                wrapped_diffs = [min(d, 360.0 - d) for d in diffs]
                
                if all(d < threshold for d in wrapped_diffs):
                    return name
        return "N/A"

    def depth_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image (16-bit integers for depth)
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            self.latest_depth_image = cv_image
        except Exception as e:
            self.get_logger().error(f"Error processing Depth image: {e}")

    def camera_info_callback(self, msg):
        self.latest_camera_info = msg

    def publish_chat_message(self, role, text, image=None):
        """Publishes a structured chat message to the UI."""
        msg_data = {
            "role": role,
            "text": text,
            "timestamp": time.time()
        }
        if image is not None:
            # Convert PIL image to base64 using optimized JPEG to prevent rosbridge WebSocket from dropping large messages
            buffered = BytesIO()
            image = image.convert('RGB') # Ensure format is RGB for JPEG
            image.save(buffered, format="JPEG", quality=60, optimize=True)
            img_str = base64.b64encode(buffered.getvalue()).decode()
            msg_data["image"] = f"data:image/jpeg;base64,{img_str}"
        
        self.chat_pub.publish(String(data=json.dumps(msg_data)))

    async def instruction_callback(self, msg):
        user_text = msg.data
        self.get_logger().info(f'--- New User Goal: "{user_text}" ---')
        
        # Notify UI that instruction is being processed
        self.publish_chat_message("user", user_text)
        
        start_time = time.time()
        self.command_start_time = start_time # Track total command time

        loop_count = 0
        max_loops = 20
        current_message = f"User Goal: {user_text}"
        current_image = None

        while loop_count < max_loops:
            loop_count += 1
            
            # Simple wait to ensure background subscriber has updated state
            time.sleep(0.1)

            if self.latest_robot_state:
                s = self.latest_robot_state
                formatted_joints = [round(x, 3) for x in s.joint_angles]
                semantic_pos = self.get_semantic_position()
                
                state_str = textwrap.dedent(f"""
                    Current Robot State:
                    - Semantic Position: {semantic_pos}
                    - Joint Angles (degrees): {formatted_joints}
                    - Tool Pose (x, y, z): ({s.x:.3f}, {s.y:.3f}, {s.z:.3f})
                    - Tool Orientation (theta_x, theta_y, theta_z): ({s.theta_x:.3f}, {s.theta_y:.3f}, {s.theta_z:.3f})
                    - Gripper Position (0-100): {s.gripper_position:.1f}
                    """)
            else:
                state_str = "Current Robot State: Unknown (Waiting for /robot_state topic...)"

            prompt = [f"{state_str}\n\n{current_message}"]

            # 1. Capture and append the latest realsense RGB image
            if self.latest_rgb_image is not None:
                cv_image_rgb = cv2.cvtColor(self.latest_rgb_image, cv2.COLOR_BGR2RGB)
                pil_img = PILImage.fromarray(cv_image_rgb)
                width, height = pil_img.size
                new_width = 800
                new_height = int(new_width * height / width)
                img_resized = pil_img.resize((new_width, new_height), PILImage.Resampling.LANCZOS)
                prompt.append("RealSense Camera View:")
                prompt.append(img_resized)

            # 2. Capture and append RViz snapshot for spatial context
            try:
                # Find ONLY visible RViz window IDs
                window_search = subprocess.check_output(['xdotool', 'search', '--onlyvisible', '--name', 'RViz']).decode().split()
                if window_search:
                    # Use the last visible window found (often the main one)
                    window_id = window_search[-1]
                    snapshot_path = '/tmp/rviz_snapshot.png'
                    
                    # Ensure the file is clean
                    if os.path.exists(snapshot_path):
                        os.remove(snapshot_path)
                    
                    # Take snapshot using maim (best for OpenGL/RViz)
                    # -i specifies the window ID
                    subprocess.run(['maim', '-i', window_id, snapshot_path], check=True)
                    
                    if os.path.exists(snapshot_path):
                        rviz_img = PILImage.open(snapshot_path)
                        # Resize for token efficiency
                        rw, rh = rviz_img.size
                        nrw = 800
                        nrh = int(nrw * rh / rw)
                        rviz_resized = rviz_img.resize((nrw, nrh), PILImage.Resampling.LANCZOS)
                        prompt.append("RViz Visualization (Spatial Context):")
                        prompt.append(rviz_resized)
            except Exception as e:
                self.get_logger().warn(f"Could not capture RViz snapshot: {e}")

            if current_image:
                prompt.append(current_image)
                current_image = None # Consume the image for this turn

            try:
                response = self.chat_session.send_message(prompt)

                task_is_complete = False
                action_taken = False
                
                if not response.candidates or not response.candidates[0].content.parts:
                    break

                for part in response.candidates[0].content.parts:
                    if part.text and part.text.strip():
                        self.publish_chat_message("model", part.text.strip())
                        if not part.function_call:
                            current_message = "Please decide the next step or call task_complete."

                    if part.function_call:
                        if part.function_call.name == "task_complete":
                            self.get_logger().info("Task marked as complete by Gemini.")
                            task_is_complete = True
                            break
                        
                        # Execute function and capture result
                        action_taken = True
                        action_result, current_image = await self.execute_function(part.function_call)
                        
                        # Report result back to UI chat
                        self.publish_chat_message("system", action_result, image=current_image)

                        current_message = f"Action {part.function_call.name} resulted in: {action_result}. Based on the goal, what is the next action?"

                if task_is_complete:
                    break
                    
                if not action_taken:
                    # No function call in response
                    self.get_logger().warn("No action taken by Gemini. Breaking loop.")
                    break
            
            except Exception as e:
                self.get_logger().error(f"Error during Gemini Chat API call or execution: {str(e)}")
                self.publish_chat_message("system", f"Error: {str(e)}")
                break

        if loop_count >= max_loops:
            self.get_logger().warn("Reached maximum loop iterations for single task.")

        processing_time = time.time() - start_time
        self.get_logger().info(f"Total task time: {processing_time:.4f} seconds\n")

    async def execute_function(self, function_call):
        name = function_call.name
        args = function_call.args
        
        self.get_logger().info(f"Executing: {name}({args})")
        
        # For direct movement commands, report latency now (start of move)
        if name not in ["move_to_object", "take_picture"]:
            latency = time.time() - self.command_start_time
            self.status_pub.publish(String(data=f"LATENCY: {latency:.2f}s"))

        success = False
        error_msg = ""
        success_msg = "Success"
        captured_img = None
        
        try:
            if name == "grasp_object":
                success = await self.controller.grasp_object()
                if success: success_msg = "Success. Gripper closed and object grasped."
                else: error_msg = "Failed to grasp object"
            elif name == "open_gripper":
                success = await self.controller.set_gripper(0.0)
                if success: success_msg = "Success. Gripper fully opened."
                else: error_msg = "Failed to open gripper"
            elif name == "move_to_home":
                success = await self.controller.move_to_home()
                if success: success_msg = "Success. Robot returned to the home observation pose."
                else: error_msg = "Failed to move to home position"
            elif name == "move_to_user":
                success = await self.controller.move_to_user()
                if success: success_msg = "Success. Robot moved to the user interaction pose."
                else: error_msg = "Failed to move to user position"
                
            elif name == "move_to_position":
                time.sleep(0.1) # Ensure fresh state
                if self.latest_robot_state:
                    target_x = float(args.get('x', self.latest_robot_state.x))
                    target_y = float(args.get('y', self.latest_robot_state.y))
                    target_z = float(args.get('z', self.latest_robot_state.z))
                    
                    success = await self.controller.move_to_pose(
                        target_x, target_y, target_z, 
                        self.latest_robot_state.theta_x, 
                        self.latest_robot_state.theta_y, 
                        self.latest_robot_state.theta_z
                    )
                    if success: success_msg = f"Success. Moved to ({target_x:.3f}, {target_y:.3f}, {target_z:.3f})."
                    else: error_msg = "Failed to reach target pose"
                else:
                    error_msg = "Robot state not available"
                    success = False

            elif name == "adjust_joints":
                time.sleep(0.1) # Ensure fresh state
                if self.latest_robot_state:
                    current_angles = list(self.latest_robot_state.joint_angles)
                    joint_idx = int(args.get('joint number', 0)) - 1
                    amount = float(args.get('amount', 0.0))
                    
                    if 0 <= joint_idx < len(current_angles):
                        current_angles[joint_idx] += amount
                        success = await self.controller.move_to_joints(current_angles)
                        if success: success_msg = f"Success. Joint {joint_idx + 1} adjusted by {amount} degrees."
                        else: error_msg = "Failed to move to target joint angles"
                    else:
                        error_msg = f"Invalid joint number: {joint_idx + 1}"
                        success = False
                else:
                    error_msg = "Robot state not available"
                    success = False

            elif name == "inspect_scene":
                success, error_msg, captured_img = await self.inspect_scene()
                if success: success_msg = f"Success.\n{error_msg}" # For inspect_scene, error_msg holds the report when successful
                
        except Exception as e:
            success = False
            error_msg = f"Exception occurred: {str(e)}"
            
        img_to_return = captured_img if (name in ["inspect_scene"] and success) else None
            
        if success:
            return success_msg, img_to_return
        else:
            full_error = f"Failed: {error_msg}" if error_msg else "Failed to execute action"
            self.get_logger().error(f"{name} {full_error}")
            return full_error, None

    async def inspect_scene(self):
        """Identifies all objects in the scene and returns their 3D poses using high-precision masks."""
        self.get_logger().info("Performing high-precision scene inspection...")

        # 1. Capture Image (Wait for fresh data)
        for _ in range(10):
            if self.latest_rgb_image is not None and self.latest_depth_image is not None:
                break
            self.get_logger().info("Waiting for images from RealSense...")
            time.sleep(0.5)
            
        if self.latest_rgb_image is None:
             return False, "No RGB image received from RealSense.", None

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
        orig_h, orig_w = captured_depth.shape

        # 2. Query Gemini for Masks
        prompt = textwrap.dedent("""\
            Identify all prominent items in the scene. 
            For each item, provide a bounding box, a descriptive label, and a segmentation mask.
            Return the result as a JSON list:
            [{"box_2d": [ymin, xmin, ymax, xmax], "label": "item name", "mask": "png_base64_str"}]
            The coordinates are normalized to 0-1000.
            """)
        
        try:
            response = self.client.models.generate_content(
                model=self.robot_model_id,
                contents=[img_resized, prompt],
                config=types.GenerateContentConfig(temperature=0.0)
            )
            
            json_output = response.text
            self.get_logger().info(f"Vision response received. Processing masks...")
            
            # Use the segmentation parser from vision_utils
            masks = vision_utils.parse_segmentation_masks(json_output, img_height=new_height, img_width=new_width)

            if not masks:
                return True, "No items detected in the scene.", img_resized

            # 3. Process each mask for 3D pose (Median of Mask Math)
            scene_report = "Scene Inspection Report:\n"
            annotated_img = img_resized.copy()
            
            # Prepare transformation (from camera to base)
            try:
                transform = self.tf_buffer.lookup_transform(
                    'base_link',
                    'physical_realsense_link',
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=1.0)
                )
            except TransformException as ex:
                return False, f"Transformation error: {ex}", None

            if self.latest_camera_info is None:
                return False, "Camera info not available.", None

            for i, mask_obj in enumerate(masks):
                label = mask_obj.label
                
                # Upscale mask to original resolution for depth mapping
                upscaled_mask = cv2.resize(mask_obj.mask, (orig_w, orig_h), interpolation=cv2.INTER_NEAREST)
                _, binary_mask_orig = cv2.threshold(upscaled_mask, 127, 255, cv2.THRESH_BINARY)
                
                # Extract median depth within the mask (robust to noise)
                object_mask_bool = (binary_mask_orig > 0)
                depth_values = captured_depth[object_mask_bool]
                valid_depths = depth_values[depth_values > 0]
                
                if len(valid_depths) == 0:
                    scene_report += f"- {label}: Pose unknown (no valid depth in mask).\n"
                    continue

                depth_meters = np.median(valid_depths) / 1000.0

                # Calculate Centroid for 3D projection
                M = cv2.moments(binary_mask_orig)
                if M["m00"] != 0:
                    pixel_x = int(M["m10"] / M["m00"])
                    pixel_y = int(M["m01"] / M["m00"])
                else:
                    # Fallback to bbox center
                    pixel_x = int(((mask_obj.x0 + mask_obj.x1) / 2) * (orig_w / new_width))
                    pixel_y = int(((mask_obj.y0 + mask_obj.y1) / 2) * (orig_h / new_height))

                # 1. Pixel to 3D in Camera Frame
                cam_x, cam_y, cam_z = vision_utils.get_3d_point_from_pixel(
                    pixel_x, pixel_y, depth_meters, self.latest_camera_info
                )
                # 2. Transform to Base Frame
                base_x, base_y, base_z = vision_utils.transform_point(
                    cam_x, cam_y, cam_z, transform
                )
                
                scene_report += f"- {label}: Pose(x={base_x:.3f}, y={base_y:.3f}, z={base_z:.3f}) relative to base.\n"

                # 4. Annotate image for user verification
                draw = ImageDraw.Draw(annotated_img)
                colors = ["red", "green", "blue", "yellow", "orange", "pink", "purple", "cyan", "magenta"]
                color = colors[i % len(colors)]
                
                # Draw the box on the resized image
                box = [mask_obj.y0, mask_obj.x0, mask_obj.y1, mask_obj.x1]
                draw.rectangle(((box[1], box[0]), (box[3], box[2])), outline=color, width=3)
                draw.text((box[1] + 5, box[0] + 5), label, fill=color)

            return True, scene_report, annotated_img

        except Exception as e:
            self.get_logger().error(f"Error in inspect_scene: {str(e)}")
            return False, f"Inspection failed: {str(e)}", None





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
