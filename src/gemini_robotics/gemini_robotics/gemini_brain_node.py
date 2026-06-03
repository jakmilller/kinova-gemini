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

import sys
import argparse
import torch
from scipy.spatial.transform import Rotation as R
from ros2_interfaces.srv import ComputeIK

# AnyGrasp imports
workspace_path = os.path.expanduser('~/kinova-gemini')
sys.path.append(os.path.join(workspace_path, 'anygrasp_sdk'))
sys.path.append(os.path.join(workspace_path, 'anygrasp_sdk', 'grasp_detection'))
try:
    from gsnet import AnyGrasp
except ImportError as e:
    print(f"Error importing AnyGrasp: {e}")

# SAM2 imports
sam2_repo_path = "/home/mcrr-lab/raf-live/SAM2_streaming"
sys.path.append(sam2_repo_path)
try:
    from sam2.build_sam import build_sam2
    from sam2.sam2_image_predictor import SAM2ImagePredictor
    SAM2_AVAILABLE = True
except ImportError as e:
    print(f"Error importing SAM2: {e}")
    SAM2_AVAILABLE = False

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

        # --- Initialize Models ---
        self.get_logger().info('Initializing AnyGrasp...')
        args = argparse.Namespace()
        args.checkpoint_path = os.path.join(workspace_path, 'anygrasp_sdk/grasp_detection/log/checkpoint_detection.tar')
        args.max_gripper_width = 0.14
        args.gripper_height = 0.07
        args.top_down_grasp = False
        args.debug = False
        
        self.anygrasp = AnyGrasp(args)
        self.anygrasp.load_net()

        if SAM2_AVAILABLE:
            self.get_logger().info('Initializing SAM2...')
            sam2_checkpoint = os.path.join(sam2_repo_path, "checkpoints/sam2/sam2_hiera_large.pt")
            sam2_model_cfg = "sam2/sam2_hiera_l.yaml"
            self.sam_predictor = SAM2ImagePredictor(build_sam2(sam2_model_cfg, sam2_checkpoint, device="cuda"))
        else:
            self.sam_predictor = None

        self.ik_cb_group = MutuallyExclusiveCallbackGroup()
        self.ik_client = self.create_client(ComputeIK, 'compute_ik', callback_group=self.ik_cb_group)

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
            model=self.flash_model_id,
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
        max_loops = 30
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
            # because the camera is wrist mounted, i though the Rviz screenshot could provide cool spatial context without a scene camera, however perrformance is fine and quicker without this
            # try:
            #     # Find ONLY visible RViz window IDs
            #     window_search = subprocess.check_output(['xdotool', 'search', '--onlyvisible', '--name', 'RViz']).decode().split()
            #     if window_search:
            #         # Use the last visible window found (often the main one)
            #         window_id = window_search[-1]
            #         snapshot_path = '/tmp/rviz_snapshot.png'
                    
            #         # Ensure the file is clean
            #         if os.path.exists(snapshot_path):
            #             os.remove(snapshot_path)
                    
            #         # Take snapshot using maim (best for OpenGL/RViz)
            #         # -i specifies the window ID
            #         subprocess.run(['maim', '-i', window_id, snapshot_path], check=True)
                    
            #         if os.path.exists(snapshot_path):
            #             rviz_img = PILImage.open(snapshot_path)
            #             # Resize for token efficiency
            #             rw, rh = rviz_img.size
            #             nrw = 800
            #             nrh = int(nrw * rh / rw)
            #             rviz_resized = rviz_img.resize((nrw, nrh), PILImage.Resampling.LANCZOS)
            #             prompt.append("RViz Visualization (Spatial Context):")
            #             prompt.append(rviz_resized)
            # except Exception as e:
            #     self.get_logger().warn(f"Could not capture RViz snapshot: {e}")

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

    async def move_to_pose(self, object_label):
        self.get_logger().info(f"Executing 6D Grasp for: {object_label}")
        
        for _ in range(10):
            if self.latest_rgb_image is not None and self.latest_depth_image is not None and self.latest_camera_info is not None and self.latest_robot_state is not None:
                break
            time.sleep(0.5)
            
        if self.latest_rgb_image is None: return False, "No RGB image"

        cv_image_rgb = cv2.cvtColor(self.latest_rgb_image, cv2.COLOR_BGR2RGB)
        pil_img = PILImage.fromarray(cv_image_rgb)
        
        prompt = f"Identify the most easily graspable part of the '{object_label}' (e.g. handle, rim, or the whole body if it is small). Return ONLY a JSON with a bounding box in [ymin, xmin, ymax, xmax] format (normalized 0-1000): {{\"box_2d\": [ymin, xmin, ymax, xmax], \"label\": \"part name\"}}"
        
        response = self.client.models.generate_content(
            model=self.flash_model_id,
            contents=[prompt, pil_img],
            config=types.GenerateContentConfig(response_mime_type="application/json", temperature=0.0)
        )
        
        try:
            res = json.loads(response.text)
            ymin_norm, xmin_norm, ymax_norm, xmax_norm = res['box_2d']
            label = res['label']
            
            h, w = self.latest_rgb_image.shape[:2]
            ymin, xmin, ymax, xmax = int(ymin_norm * h / 1000), int(xmin_norm * w / 1000), int(ymax_norm * h / 1000), int(xmax_norm * w / 1000)
        except Exception as e:
            return False, f"Gemini box parsing failed: {e}"

        if not self.sam_predictor:
            return False, "SAM2 not available"

        self.get_logger().info(f"Refining '{label}' segmentation with SAM2 box-prompt...")
        with torch.inference_mode(), torch.autocast(device_type="cuda", dtype=torch.bfloat16):
            self.sam_predictor.set_image(cv_image_rgb)
            input_box = np.array([xmin, ymin, xmax, ymax])
            masks, _, _ = self.sam_predictor.predict(box=input_box, multimask_output=False)
        
        if torch.is_tensor(masks): binary_mask = masks[0].cpu().numpy() > 0
        else: binary_mask = masks[0] > 0

        box_mask = np.zeros((h, w), dtype=bool)
        box_mask[max(0, ymin):min(h, ymax), max(0, xmin):min(w, xmax)] = True
        binary_mask = binary_mask & box_mask

        torch.set_default_dtype(torch.float32)
        torch.cuda.empty_cache()

        depths = self.latest_depth_image.astype(np.float32)
        fx, fy = self.latest_camera_info.k[0], self.latest_camera_info.k[4]
        cx, cy = self.latest_camera_info.k[2], self.latest_camera_info.k[5]
        scale = 1000.0 

        xmap, ymap = np.meshgrid(np.arange(w), np.arange(h))
        points_z = depths / scale
        points_x = (xmap - cx) / fx * points_z
        points_y = (ymap - cy) / fy * points_z
        points = np.stack([points_x, points_y, points_z], axis=-1).astype(np.float32)
        colors = cv_image_rgb.astype(np.float32) / 255.0

        target_points = points[binary_mask].reshape(-1, 3)
        valid_target_idx = (target_points[:, 2] > 0.1) & (target_points[:, 2] < 1.0)
        target_points = target_points[valid_target_idx]

        if target_points.shape[0] < 10: return False, "Target point cloud is too small."

        obj_min, obj_max = target_points.min(axis=0), target_points.max(axis=0)
        margin = 0.005
        lims = [obj_min[0]-margin, obj_max[0]+margin, obj_min[1]-margin, obj_max[1]+margin, obj_min[2]-margin, obj_max[2]+margin]

        scene_mask = (points[:, :, 2] > 0.2) & (points[:, :, 2] < 0.75)
        full_points = points[scene_mask].reshape(-1, 3)
        full_colors = colors[scene_mask].reshape(-1, 3)
        full_obj_mask = binary_mask[scene_mask]
        
        in_lims = (full_points[:, 0] >= lims[0]) & (full_points[:, 0] <= lims[1]) & \
                  (full_points[:, 1] >= lims[2]) & (full_points[:, 1] <= lims[3]) & \
                  (full_points[:, 2] >= lims[4]) & (full_points[:, 2] <= lims[5])
        
        keep_mask = (in_lims & full_obj_mask) | (~in_lims)
        points_for_anygrasp = full_points[keep_mask]
        colors_for_anygrasp = full_colors[keep_mask]
        
        torch.cuda.empty_cache()
        self.get_logger().info("Running AnyGrasp...")
        gg, _ = self.anygrasp.get_grasp(points_for_anygrasp, colors_for_anygrasp, lims=lims, apply_object_mask=True, dense_grasp=True, collision_detection=True)
        torch.cuda.empty_cache()

        if len(gg) == 0: return False, "No grasps detected."

        gg = gg.nms().sort_by_score()
        best_grasp = gg[0]
        self.get_logger().info(f"Best Grasp: Width={best_grasp.width:.3f}m, Depth={best_grasp.depth:.3f}m")

        desired_width = best_grasp.width + 0.02
        target_gripper_pos = max(0.0, min(100.0, (0.14 - desired_width) / 0.14 * 100.0))
        self.get_logger().info(f"Setting gripper to {target_gripper_pos:.1f}%")
        await self.controller.set_gripper(target_gripper_pos)
        time.sleep(1.0)

        def transform_to_matrix(t_msg):
            mat = np.eye(4)
            q = [t_msg.transform.rotation.x, t_msg.transform.rotation.y, t_msg.transform.rotation.z, t_msg.transform.rotation.w]
            mat[:3, :3] = R.from_quat(q).as_matrix()
            mat[0, 3] = t_msg.transform.translation.x; mat[1, 3] = t_msg.transform.translation.y; mat[2, 3] = t_msg.transform.translation.z
            return mat

        try:
            t_base_cam = self.tf_buffer.lookup_transform('base_link', self.latest_camera_info.header.frame_id, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=2.0))
        except: return False, "TF lookup failed"

        T_base_cam = transform_to_matrix(t_base_cam)
        R_align = np.array([[0,0,1],[-1,0,0],[0,-1,0]])
        T_cam_grasp = np.eye(4); T_cam_grasp[:3,:3] = best_grasp.rotation_matrix; T_cam_grasp[:3,3] = best_grasp.translation
        T_base_grasp = T_base_cam @ T_cam_grasp
        r_base_grasp = T_base_grasp[:3,:3] @ R_align
        grasp_position = T_base_grasp[:3,3]

        try:
            t_current = self.tf_buffer.lookup_transform('base_link', 'end_effector_link', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0))
            R_curr = transform_to_matrix(t_current)[:3, :3]
            R_flip = r_base_grasp @ R.from_euler('z', 180, degrees=True).as_matrix()
            def angular_dist(R1, R2):
                trace = np.trace(np.dot(R1.T, R2))
                return np.arccos(np.clip((trace - 1.0) / 2.0, -1.0, 1.0))
            if angular_dist(R_curr, R_flip) < angular_dist(R_curr, r_base_grasp): r_base_grasp = R_flip
        except: pass

        dynamic_offset = 0.0243 * (self.latest_robot_state.gripper_position / 100.0)
        approach_dist = best_grasp.depth 
        approach_vector = r_base_grasp[:, 2] 
        insertion_offset = 0.04 

        # If the target is a cup or bottle, we add a 4cm offset to the insertion 
        # so the gripper goes deeper into/around the object for a more secure grasp.
        target_lower = object_label.lower()
        if "cup" in target_lower or "bottle" in target_lower:
            insertion_offset += 0.05
            self.get_logger().info(f"Target '{object_label}' is a cup/bottle, increasing insertion offset to {insertion_offset}m")
        
        pre_grasp_palm_pos = grasp_position - (approach_dist + dynamic_offset + 0.02) * approach_vector

        # Solve IK
        async def solve_ik_with_retry(pos, rot_matrix):
            if not self.ik_client.wait_for_service(timeout_sec=3.0):
                self.get_logger().error("ComputeIK service is not available.")
                return None, None

            euler = R.from_matrix(rot_matrix).as_euler('xyz', degrees=True)
            req = ComputeIK.Request(); req.x, req.y, req.z = float(pos[0]), float(pos[1]), float(pos[2])
            req.theta_x, req.theta_y, req.theta_z = float(euler[0]), float(euler[1]), float(euler[2])
            future = self.ik_client.call_async(req); res = await future
            if res and res.success: return res.joint_angles, euler
            rot_flip = rot_matrix @ R.from_euler('z', 180, degrees=True).as_matrix(); euler_flip = R.from_matrix(rot_flip).as_euler('xyz', degrees=True)
            req.theta_x, req.theta_y, req.theta_z = float(euler_flip[0]), float(euler_flip[1]), float(euler_flip[2])
            future = self.ik_client.call_async(req); res = await future
            if res and res.success: return res.joint_angles, euler_flip
            return None, None

        self.get_logger().info("Calculating and moving to Pre-Grasp...")
        joint_angles, final_euler = await solve_ik_with_retry(pre_grasp_palm_pos, r_base_grasp)
        if joint_angles:
            await self.controller.move_to_joints(joint_angles)
        else: return False, "IK Failed"

        final_rot_matrix = R.from_euler('xyz', final_euler, degrees=True).as_matrix()
        final_approach_vector = final_rot_matrix[:, 2]
        grasp_palm_pos = pre_grasp_palm_pos + (insertion_offset * final_approach_vector)
        
        self.get_logger().info("Sliding forward to Grasp...")
        await self.controller.move_to_pose(grasp_palm_pos[0], grasp_palm_pos[1], grasp_palm_pos[2], final_euler[0], final_euler[1], final_euler[2])
        self.get_logger().info("Pinching object...")
        await self.controller.grasp_object()

        return True, ""

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

            elif name == "move_to_pose":
                success, error_msg = await self.move_to_pose(args.get('object_label', 'object'))
                if success: success_msg = f"Success. Grasped {args.get('object_label', 'object')}."

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
        """Identifies all objects in the scene and returns their 3D poses using high-precision masks from SAM2."""
        self.get_logger().info("Performing high-precision scene inspection with SAM2...")

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

        # 2. Query Gemini for Bounding Boxes
        prompt = textwrap.dedent("""\
            Identify all prominent items in the scene. 
            For each item, provide a bounding box and a descriptive label.
            Return the result as a JSON list:
            [{"box_2d": [ymin, xmin, ymax, xmax], "label": "item name"}]
            The coordinates are normalized to 0-1000.
            """)
        
        try:
            response = self.client.models.generate_content(
                model=self.flash_model_id,
                contents=[img_resized, prompt],
                config=types.GenerateContentConfig(response_mime_type="application/json", temperature=0.0)
            )
            
            items = json.loads(response.text)
            self.get_logger().info(f"Vision response received. Found {len(items)} items. Processing SAM2 masks...")

            if not items:
                return True, "No items detected in the scene.", img_resized

            # 3. Process each item for 3D pose using SAM2
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

            for i, item in enumerate(items):
                if 'label' not in item or 'box_2d' not in item:
                    continue
                label = item['label']
                ymin_norm, xmin_norm, ymax_norm, xmax_norm = item['box_2d']
                
                ymin = int(ymin_norm * orig_h / 1000)
                xmin = int(xmin_norm * orig_w / 1000)
                ymax = int(ymax_norm * orig_h / 1000)
                xmax = int(xmax_norm * orig_w / 1000)
                
                if self.sam_predictor:
                    with torch.inference_mode(), torch.autocast(device_type="cuda", dtype=torch.bfloat16):
                        self.sam_predictor.set_image(cv_image_rgb)
                        input_box = np.array([xmin, ymin, xmax, ymax])
                        masks, _, _ = self.sam_predictor.predict(box=input_box, multimask_output=False)
                    
                    if torch.is_tensor(masks): binary_mask_orig = masks[0].cpu().numpy() > 0
                    else: binary_mask_orig = masks[0] > 0

                    box_mask = np.zeros((orig_h, orig_w), dtype=bool)
                    box_mask[max(0, ymin):min(orig_h, ymax), max(0, xmin):min(orig_w, xmax)] = True
                    binary_mask_orig = binary_mask_orig & box_mask
                    
                    torch.set_default_dtype(torch.float32)
                    torch.cuda.empty_cache()
                else:
                    return False, "SAM2 not available.", None
                
                # Extract median depth within the mask (robust to noise)
                object_mask_bool = (binary_mask_orig > 0)
                depth_values = captured_depth[object_mask_bool]
                valid_depths = depth_values[depth_values > 0]
                
                if len(valid_depths) == 0:
                    scene_report += f"- {label}: Pose unknown (no valid depth in mask).\n"
                    continue

                depth_meters = np.median(valid_depths) / 1000.0

                # Calculate Centroid for 3D projection
                M = cv2.moments(binary_mask_orig.astype(np.uint8))
                if M["m00"] != 0:
                    pixel_x = int(M["m10"] / M["m00"])
                    pixel_y = int(M["m01"] / M["m00"])
                else:
                    # Fallback to bbox center
                    pixel_x = int((xmin + xmax) / 2)
                    pixel_y = int((ymin + ymax) / 2)

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
                
                new_ymin = ymin * new_height / orig_h
                new_xmin = xmin * new_width / orig_w
                new_ymax = ymax * new_height / orig_h
                new_xmax = xmax * new_width / orig_w
                
                # Draw the box on the resized image
                draw.rectangle(((new_xmin, new_ymin), (new_xmax, new_ymax)), outline=color, width=3)
                draw.text((new_xmin + 5, new_ymin + 5), label, fill=color)

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