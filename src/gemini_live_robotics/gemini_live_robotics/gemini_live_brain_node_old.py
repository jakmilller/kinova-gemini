#!/home/mcrr-lab/anaconda3/envs/kinova-gemini/bin/python3

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from ros2_interfaces.msg import RobotState
from ros2_interfaces.srv import ComputeIK
from cv_bridge import CvBridge
from google import genai
from google.genai import types
import os
import yaml
import sys
import json
import cv2
import time
import base64
from io import BytesIO
from datetime import datetime
import numpy as np
import torch
import textwrap
import argparse
import asyncio
import threading
import pyaudio
from PIL import Image as PILImage, ImageDraw, ImageFont
from scipy.spatial.transform import Rotation as R
from dotenv import load_dotenv
from tf2_ros import Buffer, TransformListener, TransformException

# Import local copied modules (completely isolated from old gemini_robotics)
from .robot_controller_ros2 import KinovaRobotControllerROS2
from .gemini_live_tools import ALL_TOOLS
from . import vision_utils

# AnyGrasp and SAM2 path settings
workspace_path = os.path.expanduser('~/kinova-gemini')
sys.path.append(os.path.join(workspace_path, 'anygrasp_sdk'))
sys.path.append(os.path.join(workspace_path, 'anygrasp_sdk', 'grasp_detection'))

try:
    from gsnet import AnyGrasp
    ANYGRASP_AVAILABLE = True
except ImportError as e:
    ANYGRASP_AVAILABLE = False

sam2_repo_path = "/home/mcrr-lab/raf-live/SAM2_streaming"
sys.path.append(sam2_repo_path)
try:
    from sam2.build_sam import build_sam2
    from sam2.sam2_image_predictor import SAM2ImagePredictor
    SAM2_AVAILABLE = True
except ImportError as e:
    ANYGRASP_AVAILABLE = False
    SAM2_AVAILABLE = False


class GeminiLiveBrainNode(Node):
    def __init__(self):
        super().__init__('gemini_live_brain_node')

        workspace_path = os.path.expanduser('~/kinova-gemini')
        load_dotenv(os.path.join(workspace_path, '.env'))
        with open(os.path.join(workspace_path, 'config.yaml'), 'r') as f:
            self.robot_config = yaml.safe_load(f)

        self.client = genai.Client(api_key=os.getenv('gemini_api_key'))
        self.live_model_id = self.robot_config['model']['live']
        self.flash_model_id = self.robot_config['model']['flash']

        # Read tailored system instruction
        prompt_path = os.path.join(workspace_path, 'src', 'prompts', 'system_live_prompt.txt')
        with open(prompt_path, 'r') as f:
            self.system_instruction = f.read()

        # --- Initialize Models ---
        if ANYGRASP_AVAILABLE:
            self.get_logger().info('Initializing AnyGrasp...')
            args = argparse.Namespace()
            args.checkpoint_path = os.path.join(workspace_path, 'anygrasp_sdk/grasp_detection/log/checkpoint_detection.tar')
            args.max_gripper_width = 0.14
            args.gripper_height = 0.07
            args.top_down_grasp = False
            args.debug = False
            self.anygrasp = AnyGrasp(args)
            self.anygrasp.load_net()
        else:
            self.get_logger().warn('AnyGrasp NOT available.')
            self.anygrasp = None

        if SAM2_AVAILABLE:
            self.get_logger().info('Initializing SAM2...')
            sam2_checkpoint = os.path.join(sam2_repo_path, "checkpoints/sam2/sam2_hiera_tiny.pt")
            sam2_model_cfg = "sam2/sam2_hiera_t.yaml"
            self.sam_predictor = SAM2ImagePredictor(build_sam2(sam2_model_cfg, sam2_checkpoint, device="cuda"))
        else:
            self.get_logger().warn('SAM2 NOT available.')
            self.sam_predictor = None

        self.speaker_queue = asyncio.Queue()
        self.p = pyaudio.PyAudio()
        self.controller = KinovaRobotControllerROS2()
        self.bridge = CvBridge()

        # Camera & State Cache
        self.latest_rgb_image = None
        self.latest_depth_image = None
        self.latest_camera_info = None
        self.current_robot_state = None
        self.transcription_buffer = ""
        self.is_moving = False

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Reentrant Callback Group to process multiple streams & inputs in parallel
        self.reentrant_group = ReentrantCallbackGroup()

        # Inverse Kinematics Client
        self.ik_cb_group = MutuallyExclusiveCallbackGroup()
        self.ik_client = self.create_client(ComputeIK, 'compute_ik', callback_group=self.ik_cb_group)

        # Camera feed subscriptions
        self.create_subscription(
            Image, '/camera/realsense/color/image_raw', self.rgb_callback, 10, callback_group=self.reentrant_group
        )
        self.create_subscription(
            Image, '/camera/realsense/aligned_depth_to_color/image_raw', self.depth_callback, 10, callback_group=self.reentrant_group
        )
        self.create_subscription(
            CameraInfo, '/camera/realsense/aligned_depth_to_color/camera_info', self.camera_info_callback, 10, callback_group=self.reentrant_group
        )

        # Robot state monitoring subscription
        self.create_subscription(
            RobotState, 'robot_state', self.state_callback, 10, callback_group=self.reentrant_group
        )

        self.tools = [types.Tool(function_declarations=ALL_TOOLS)]

        # --- UI Integration Setup ---
        self.loop = None
        self.session_instance = None
        self.text_input_queue = asyncio.Queue()
        
        # Publisher for Gemini chat messages to UI
        self.chat_pub = self.create_publisher(String, '/gemini_chat', 10)
        
        # Subscription for user instructions from UI
        self.create_subscription(
            String, '/user_instructions', self.instruction_callback, 10, callback_group=self.reentrant_group
        )

        # Background task tracking for safety and interruptions
        self.active_robot_task = None

        self.get_logger().info('Gemini Live Brain Node initialized.')

    def rgb_callback(self, msg):
        try:
            self.latest_rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"RGB callback error: {e}")

    def depth_callback(self, msg):
        try:
            self.latest_depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f"Depth callback error: {e}")

    def camera_info_callback(self, msg):
        self.latest_camera_info = msg

    def state_callback(self, msg):
        if self.current_robot_state is not None:
            diff = np.abs(np.array(msg.joint_angles) - np.array(self.current_robot_state.joint_angles))
            self.is_moving = bool(np.any(diff > 0.05))
        else:
            self.is_moving = False
        self.current_robot_state = msg

    def publish_chat_message(self, role, text, image=None):
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

    def instruction_callback(self, msg):
        user_text = msg.data
        self.get_logger().info(f'--- New User Goal: "{user_text}" ---')
        self.publish_chat_message("user", user_text)

        if self.loop is not None:
            self.loop.call_soon_threadsafe(self.text_input_queue.put_nowait, user_text)
        else:
            self.get_logger().warn("Async loop is not set up yet. Ignoring instruction.")

    async def send_text_task(self, session):
        """Sends text input from UI to the Gemini Live session."""
        while True:
            text = await self.text_input_queue.get()
            ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
            self.get_logger().info(f"[{ts}] - Sent to model (User Input): {text}")
            await session.send_realtime_input(text=text)

    async def play_speaker_task(self):
        """Plays output audio chunks from speaker queue"""
        try:
            stream = await asyncio.to_thread(
                self.p.open,
                format=pyaudio.paInt16,
                channels=1,
                rate=24000,
                output=True,
            )
            try:
                while True:
                    chunk = await self.speaker_queue.get()
                    await asyncio.to_thread(stream.write, chunk)
            finally:
                stream.stop_stream()
                stream.close()
        except Exception as e:
            self.get_logger().error(f"Speaker stream error: {e}")

    async def video_stream_task(self, session):
        """Streams camera frames to Gemini at 1fps without prompting state to prevent loops"""
        while True:
            if self.latest_rgb_image is not None:
                try:
                    frame_resized = cv2.resize(self.latest_rgb_image, (768, 768))
                    success, encoded_img = cv2.imencode(
                        '.jpg', frame_resized, [cv2.IMWRITE_JPEG_QUALITY, 50]
                    )
                    if success:
                        ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                        await session.send_realtime_input(
                            video=types.Blob(
                                data=encoded_img.tobytes(),
                                mime_type="image/jpeg"
                            )
                        )
                        # Log status to the user using the calculated is_moving property
                        #is_moving_str = "MOVING" if self.is_moving else "STATIONARY"
                        #self.get_logger().info(f"[{ts}] - Sent to model: Video frame only (Robot status: {is_moving_str})")
                except Exception as e:
                    self.get_logger().error(f"Video stream error: {e}")
            await asyncio.sleep(1.0)

    # async def cancel_active_task(self):
    #     """Cancels any running background robot task safely."""
    #     if self.active_robot_task and not self.active_robot_task.done():
    #         self.get_logger().info("Interrupting active robot movement...")
    #         self.active_robot_task.cancel()
    #         try:
    #             await self.active_robot_task
    #         except asyncio.CancelledError:
    #             pass
    #         self.active_robot_task = None

    async def execute_tool_task(self, name, args):
        """Asynchronously executes physical movements and vision tasks in the background"""
        try:
            self.get_logger().info(f"Background task execution started for: {name}")

            success = False
            result_detail = ""

            if name == "move_to_home":
                success = await self.controller.move_to_home()
                result_detail = "Robot has returned to the home position." if success else "Failed to move to home position."

            elif name == "stop_robot":
                success = await self.controller.stop_robot()
                result_detail = "Robot has stopped (not moving)." if success else "Failed to stop robot."

            elif name == "move_to_user":
                success = await self.controller.move_to_user()
                result_detail = "Robot has moved to the user position." if success else "Failed to move to user position."

            elif name == "grasp_object":
                success = await self.controller.grasp_object()
                result_detail = "Gripper has closed and detected contact." if success else "Failed to close gripper."

            elif name == "open_gripper":
                success = await self.controller.set_gripper(0.0)
                result_detail = "Gripper is now fully opened." if success else "Failed to open gripper."

            elif name == "move_to_position":
                if self.current_robot_state:
                    target_x = float(args.get('x', self.current_robot_state.x))
                    target_y = float(args.get('y', self.current_robot_state.y))
                    target_z = float(args.get('z', self.current_robot_state.z))

                    success = await self.controller.move_to_pose(
                        target_x, target_y, target_z,
                        self.current_robot_state.theta_x,
                        self.current_robot_state.theta_y,
                        self.current_robot_state.theta_z
                    )
                    result_detail = f"Moved to coordinates (x={target_x:.3f}, y={target_y:.3f}, z={target_z:.3f})" if success else "Failed to reach target coordinates."
                else:
                    success = False
                    result_detail = "Failed: Robot state is not currently available."

            elif name == "adjust_joints":
                if self.current_robot_state:
                    current_angles = list(self.current_robot_state.joint_angles)
                    joint_idx = int(args.get('joint number', 0)) - 1
                    amount = float(args.get('amount', 0.0))

                    if 0 <= joint_idx < len(current_angles):
                        current_angles[joint_idx] += amount
                        success = await self.controller.move_to_joints(current_angles)
                        result_detail = f"Adjusted joint {joint_idx + 1} by {amount} degrees." if success else "Failed joint adjustment movement."
                    else:
                        success = False
                        result_detail = f"Failed: Invalid joint number {joint_idx + 1}"
                else:
                    success = False
                    result_detail = "Failed: Robot state is not currently available."

            elif name == "inspect_scene":
                self.get_logger().info("Running high-precision scene inspection locally...")
                success, report, annotated_img = await self.inspect_scene()
                result_detail = report if success else f"Scene inspection failed: {report}"
                self.publish_chat_message("system", report, annotated_img if success else None)

            elif name == "move_to_pose":
                obj_label = args.get('object_label', 'object')
                self.get_logger().info(f"Running autonomous 6D grasp sequence for '{obj_label}'...")
                success, report = await self.move_to_pose(obj_label)
                result_detail = f"Robot has moved to the best detected grasp position for '{obj_label}', and closed the gripper." if success else f"Could not move to grasp position for the '{obj_label}': {report}"

            # Post the event feedback back into the Live Session as text input
            if self.session_instance:
                status_str = "SUCCESS" if success else "FAILED"
                event_message = f"SYSTEM EVENT: {name} finished. Status: {status_str}. Detail: {result_detail}"
                ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                self.get_logger().info(f"[{ts}] - Sent to model: {event_message}")
                
                if name != "inspect_scene":
                    self.publish_chat_message("system", f"Tool Result ({name}): {status_str}\n{result_detail}")
                    
                await self.session_instance.send_realtime_input(text=event_message)

        except asyncio.CancelledError:
            self.get_logger().warn(f"Task {name} was cancelled during background execution.")
        except Exception as e:
            self.get_logger().error(f"Error in background task {name}: {e}")
            if name != "inspect_scene":
                self.publish_chat_message("system", f"Tool Error ({name}): {e}")
            if self.session_instance:
                ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                self.get_logger().info(f"[{ts}] - Sent to model: SYSTEM EVENT: {name} failed with exception: {e}")
                await self.session_instance.send_realtime_input(text=f"SYSTEM EVENT: {name} failed with exception: {e}")

    async def inspect_scene(self):
        # 1. Capture Image (Wait for fresh data)
        for _ in range(10):
            if self.latest_rgb_image is not None and self.latest_depth_image is not None:
                break
            self.get_logger().info("Waiting for images from RealSense...")
            await asyncio.sleep(0.5)

        if self.latest_rgb_image is None:
             return False, "No RGB image received from RealSense.", None

        # Convert OpenCV BGR to PIL RGB
        cv_image_rgb = cv2.cvtColor(self.latest_rgb_image, cv2.COLOR_BGR2RGB)
        pil_img = PILImage.fromarray(cv_image_rgb)
        
        width, height = pil_img.size
        new_width = 800
        new_height = int(new_width * height / width)
        img_resized = pil_img.resize((new_width, new_height), PILImage.Resampling.LANCZOS)

        # Store the depth map (associated with this capture)
        captured_depth = self.latest_depth_image.copy()
        orig_h, orig_w = captured_depth.shape

        # 2. Query Gemini for Bounding Boxes
        prompt = textwrap.dedent("""\
            You are viewing the workspace of a general manipulator robot. Identify all of the prominent items in the scene. 
            For each item, provide a bounding box and a descriptive label.
            Return the result as a JSON list:
            [{"box_2d": [ymin, xmin, ymax, xmax], "label": "item name"}]
            The coordinates are normalized to 0-1000.
            """)
        
        try:
            # Use synchronous API to avoid async PIL Image serialization bugs
            response = self.client.models.generate_content(
                model=self.flash_model_id,
                contents=[img_resized, prompt],
                config=types.GenerateContentConfig(response_mime_type="application/json", temperature=0.0)
            )
            
            self.get_logger().info(f"Vision response received. Raw output: {response.text}")
            items = json.loads(response.text)
            
            # Defensive unwrap in case the model returns a dict wrapper instead of a raw list
            if isinstance(items, dict):
                for key, val in items.items():
                    if isinstance(val, list):
                        items = val
                        break
                        
            self.get_logger().info(f"Vision response processed. Found {len(items)} items. Processing SAM2 masks...")

            if not items:
                return True, "No items detected in the scene.", img_resized

            scene_report = "Scene Inspection Report:\n"
            annotated_img = img_resized.copy()
            
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
                    
                    if torch.is_tensor(masks): 
                        binary_mask_orig = masks[0].cpu().numpy() > 0
                    else: 
                        binary_mask_orig = masks[0] > 0

                    box_mask = np.zeros((orig_h, orig_w), dtype=bool)
                    box_mask[max(0, ymin):min(orig_h, ymax), max(0, xmin):min(orig_w, xmax)] = True
                    binary_mask_orig = binary_mask_orig & box_mask
                    
                    torch.set_default_dtype(torch.float32)
                    torch.cuda.empty_cache()
                else:
                    return False, "SAM2 not available.", None
                
                object_mask_bool = (binary_mask_orig > 0)
                depth_values = captured_depth[object_mask_bool]
                valid_depths = depth_values[depth_values > 0]
                
                if len(valid_depths) == 0:
                    scene_report += f"- {label}: Pose unknown (no valid depth in mask).\n"
                    continue

                depth_meters = np.median(valid_depths) / 1000.0

                M = cv2.moments(binary_mask_orig.astype(np.uint8))
                if M["m00"] != 0:
                    pixel_x = int(M["m10"] / M["m00"])
                    pixel_y = int(M["m01"] / M["m00"])
                else:
                    pixel_x = int((xmin + xmax) / 2)
                    pixel_y = int((ymin + ymax) / 2)

                cam_x, cam_y, cam_z = vision_utils.get_3d_point_from_pixel(
                    pixel_x, pixel_y, depth_meters, self.latest_camera_info
                )
                base_x, base_y, base_z = vision_utils.transform_point(
                    cam_x, cam_y, cam_z, transform
                )
                
                scene_report += f"- {label}: Pose(x={base_x:.3f}, y={base_y:.3f}, z={base_z:.3f}) relative to base.\n"

                draw = ImageDraw.Draw(annotated_img)
                colors = ["red", "green", "blue", "yellow", "orange", "pink", "purple", "cyan", "magenta"]
                color = colors[i % len(colors)]
                
                new_ymin = ymin * new_height / orig_h
                new_xmin = xmin * new_width / orig_w
                new_ymax = ymax * new_height / orig_h
                new_xmax = xmax * new_width / orig_w
                
                draw.rectangle(((new_xmin, new_ymin), (new_xmax, new_ymax)), outline=color, width=3)
                draw.text((new_xmin + 5, new_ymin + 5), label, fill=color)

            return True, scene_report, annotated_img

        except Exception as e:
            self.get_logger().error(f"Error in local inspect_scene: {str(e)}")
            return False, f"Inspection failed: {str(e)}", None

    async def move_to_pose(self, object_label):
        self.get_logger().info(f"Executing 6D Grasp for: {object_label}")

        for _ in range(10):
            if self.latest_rgb_image is not None and self.latest_depth_image is not None and self.latest_camera_info is not None and self.current_robot_state is not None:
                break
            await asyncio.sleep(0.5)
            
        if self.latest_rgb_image is None: 
            return False, "No RGB image available from RealSense."

        cv_image_rgb = cv2.cvtColor(self.latest_rgb_image, cv2.COLOR_BGR2RGB)
        pil_img = PILImage.fromarray(cv_image_rgb)
        
        prompt = f"Identify the most easily graspable part of the '{object_label}' (e.g. handle, rim, or the whole body if it is small). Return ONLY a JSON with a bounding box in [ymin, xmin, ymax, xmax] format (normalized 0-1000): {{\"box_2d\": [ymin, xmin, ymax, xmax], \"label\": \"part name\"}}"
        
        try:
            # Use synchronous API to avoid async PIL Image serialization bugs
            response = self.client.models.generate_content(
                model=self.flash_model_id,
                contents=[prompt, pil_img],
                config=types.GenerateContentConfig(response_mime_type="application/json", temperature=0.0)
            )
            
            self.get_logger().info(f"Grasp box response: {response.text}")
            res = json.loads(response.text)
            
            # Defensive unwrap in case the model wraps the object in a dict key
            if isinstance(res, dict) and 'box_2d' not in res:
                # Look for inner dictionary containing box_2d
                for key, val in res.items():
                    if isinstance(val, dict) and 'box_2d' in val:
                        res = val
                        break
                    elif isinstance(val, list) and len(val) > 0 and isinstance(val[0], dict) and 'box_2d' in val[0]:
                        res = val[0]
                        break
            
            ymin_norm, xmin_norm, ymax_norm, xmax_norm = res['box_2d']
            label = res['label']

            h, w = self.latest_rgb_image.shape[:2]
            ymin, xmin, ymax, xmax = int(ymin_norm * h / 1000), int(xmin_norm * w / 1000), int(ymax_norm * h / 1000), int(xmax_norm * w / 1000)
        except Exception as e:
            return False, f"Gemini box parsing failed: {e}"

        if not self.sam_predictor:
            return False, "SAM2 model is not initialized."

        self.get_logger().info(f"Refining '{label}' segmentation with SAM2 box-prompt...")
        
        with torch.inference_mode(), torch.autocast(device_type="cuda", dtype=torch.bfloat16):
            self.sam_predictor.set_image(cv_image_rgb)
            input_box = np.array([xmin, ymin, xmax, ymax])
            masks, _, _ = self.sam_predictor.predict(box=input_box, multimask_output=False)
        
        if torch.is_tensor(masks): 
            binary_mask = masks[0].cpu().numpy() > 0
        else: 
            binary_mask = masks[0] > 0

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

        if target_points.shape[0] < 10: 
            return False, "Target point cloud is too small."

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
        
        if not self.anygrasp:
            return False, "AnyGrasp not initialized."

        self.get_logger().info("🔎 Status: Querying AnyGrasp for 6D grasp generation...")

        torch.cuda.empty_cache()
        
        gg, _ = self.anygrasp.get_grasp(
            points_for_anygrasp, colors_for_anygrasp, lims=lims, 
            apply_object_mask=True, dense_grasp=True, collision_detection=True
        )
        torch.cuda.empty_cache()

        if len(gg) == 0: 
            return False, "No grasps detected."

        gg = gg.nms().sort_by_score()
        best_grasp = gg[0]
        self.get_logger().info(f"Best Grasp: Width={best_grasp.width:.3f}m, Depth={best_grasp.depth:.3f}m")

        desired_width = best_grasp.width + 0.02
        target_gripper_pos = max(0.0, min(100.0, (0.14 - desired_width) / 0.14 * 100.0))
        
        self.get_logger().info(f"🔎 Status: Pre-shaping gripper to {target_gripper_pos:.1f}%...")

        await self.controller.set_gripper(target_gripper_pos)
        await asyncio.sleep(1.0)

        def transform_to_matrix(t_msg):
            mat = np.eye(4)
            q = [t_msg.transform.rotation.x, t_msg.transform.rotation.y, t_msg.transform.rotation.z, t_msg.transform.rotation.w]
            mat[:3, :3] = R.from_quat(q).as_matrix()
            mat[0, 3] = t_msg.transform.translation.x; mat[1, 3] = t_msg.transform.translation.y; mat[2, 3] = t_msg.transform.translation.z
            return mat

        try:
            t_base_cam = self.tf_buffer.lookup_transform(
                'base_link', self.latest_camera_info.header.frame_id, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=2.0)
            )
        except: 
            return False, "TF lookup failed"

        T_base_cam = transform_to_matrix(t_base_cam)
        R_align = np.array([[0,0,1],[-1,0,0],[0,-1,0]])
        T_cam_grasp = np.eye(4); T_cam_grasp[:3,:3] = best_grasp.rotation_matrix; T_cam_grasp[:3,3] = best_grasp.translation
        T_base_grasp = T_base_cam @ T_cam_grasp
        r_base_grasp = T_base_grasp[:3,:3] @ R_align
        grasp_position = T_base_grasp[:3,3]

        try:
            t_current = self.tf_buffer.lookup_transform(
                'base_link', 'end_effector_link', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0)
            )
            R_curr = transform_to_matrix(t_current)[:3, :3]
            R_flip = r_base_grasp @ R.from_euler('z', 180, degrees=True).as_matrix()
            def angular_dist(R1, R2):
                trace = np.trace(np.dot(R1.T, R2))
                return np.arccos(np.clip((trace - 1.0) / 2.0, -1.0, 1.0))
            if angular_dist(R_curr, R_flip) < angular_dist(R_curr, r_base_grasp): 
                r_base_grasp = R_flip
        except: 
            pass

        dynamic_offset = 0.0243 * (self.controller.current_state.gripper_position / 100.0)
        approach_dist = best_grasp.depth 
        approach_vector = r_base_grasp[:, 2] 
        insertion_offset = 0.04 

        target_lower = object_label.lower()
        if "cup" in target_lower or "bottle" in target_lower:
            insertion_offset += 0.05
            self.get_logger().info(f"Target '{object_label}' is a cup/bottle, increasing insertion offset to {insertion_offset}m")
        
        pre_grasp_palm_pos = grasp_position - (approach_dist + dynamic_offset + 0.02) * approach_vector

        async def solve_ik_with_retry(pos, rot_matrix):
            if not self.ik_client.wait_for_service(timeout_sec=3.0):
                self.get_logger().error("ComputeIK service is not available.")
                return None, None

            euler = R.from_matrix(rot_matrix).as_euler('xyz', degrees=True)
            req = ComputeIK.Request(); req.x, req.y, req.z = float(pos[0]), float(pos[1]), float(pos[2])
            req.theta_x, req.theta_y, req.theta_z = float(euler[0]), float(euler[1]), float(euler[2])
            
            future = self.ik_client.call_async(req)
            res = await self.controller._await_rclpy_future(future)
            if res and res.success: 
                return res.joint_angles, euler
            
            rot_flip = rot_matrix @ R.from_euler('z', 180, degrees=True).as_matrix()
            euler_flip = R.from_matrix(rot_flip).as_euler('xyz', degrees=True)
            req.theta_x, req.theta_y, req.theta_z = float(euler_flip[0]), float(euler_flip[1]), float(euler_flip[2])
            
            future = self.ik_client.call_async(req)
            res = await self.controller._await_rclpy_future(future)
            if res and res.success: 
                return res.joint_angles, euler_flip
            return None, None

        self.get_logger().info("🔎 Status: Solving Inverse Kinematics for Pre-Grasp palm pose...")
        joint_angles, final_euler = await solve_ik_with_retry(pre_grasp_palm_pos, r_base_grasp)
        if joint_angles:
            self.get_logger().info("🔎 Status: Moving robot arm to pre-grasp position...")
            await self.controller.move_to_joints(joint_angles)
        else: 
            return False, "Inverse Kinematics solver failed."

        final_rot_matrix = R.from_euler('xyz', final_euler, degrees=True).as_matrix()
        final_approach_vector = final_rot_matrix[:, 2]
        grasp_palm_pos = pre_grasp_palm_pos + (insertion_offset * final_approach_vector)
        
        self.get_logger().info("🔎 Status: Slide forward to target object grasp position...")
        await self.controller.move_to_pose(grasp_palm_pos[0], grasp_palm_pos[1], grasp_palm_pos[2], final_euler[0], final_euler[1], final_euler[2])
        
        self.get_logger().info("🔎 Status: Grasping object...")
        await self.controller.grasp_object()

        return True, ""

    async def receive_loop_task(self, session):
        """Receives responses, audio, transcription and tool calls from Gemini"""
        self.session_instance = session
        try:
            while True:
                response = await session._receive()
                if response is None:
                    self.get_logger().warn("Session ended by server")
                    break

                if response.go_away is not None:
                    self.get_logger().warn(
                        f"Server closing connection in: {response.go_away.time_left}"
                    )

                if response.server_content:
                    if response.server_content.output_transcription:
                        self.transcription_buffer += response.server_content.output_transcription.text
    
                    if response.server_content.model_turn:
                        for part in response.server_content.model_turn.parts:
                            if part.inline_data:
                                await self.speaker_queue.put(part.inline_data.data)

                    if response.server_content.turn_complete:
                        if self.transcription_buffer:
                            ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                            self.get_logger().info(
                                f"[{ts}] - Received from model (Gemini): {self.transcription_buffer.strip()}"
                            )
                            self.publish_chat_message("model", self.transcription_buffer.strip())
                            self.transcription_buffer = ""

                if response.tool_call:
                    func_responses = []
                    for fc in response.tool_call.function_calls:
                        name = fc.name
                        args = fc.args
                        ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                        self.get_logger().info(f"[{ts}] - Received from model: Tool request '{name}' with args: {args}")
                        self.publish_chat_message("system", f"Tool Call: {name}\nArgs: {args}")

                        # Return immediate acknowledgment to the Live API to keep it responsive
                        result = f"{name} initiated in background. Keep talking to the user."
                        
                        # Dispatch the actual task as a background execution
                        self.active_robot_task = asyncio.create_task(self.execute_tool_task(name, args))

                        func_responses.append(
                            types.FunctionResponse(
                                name=fc.name,
                                id=fc.id,
                                response={"result": result}
                            )
                        )

                    if func_responses:
                        ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                        names = [fr.name for fr in func_responses]
                        self.get_logger().info(f"[{ts}] - Sent to model: Tool acknowledgment for {names}")
                        await session.send_tool_response(function_responses=func_responses)

        except asyncio.CancelledError:
            pass
        except Exception as e:
            self.get_logger().error(f"Receive loop error: {e}")

    async def run_live_session(self):
        self.loop = asyncio.get_running_loop()
        config = types.LiveConnectConfig(
            response_modalities=["AUDIO"],
            output_audio_transcription=types.AudioTranscriptionConfig(),
            system_instruction=self.system_instruction,
            tools=self.tools
        )

        try:
            async with self.client.aio.live.connect(
                model=self.live_model_id, config=config
            ) as session:
                self.get_logger().info("Connected to Gemini Live!")
                await asyncio.gather(
                    self.video_stream_task(session),
                    self.send_text_task(session),
                    self.play_speaker_task(),
                    self.receive_loop_task(session)
                )
        except Exception as e:
            self.get_logger().error(f"Live session failed: {e}")
        finally:
            self.p.terminate()


def main(args=None):
    rclpy.init(args=args)
    node = GeminiLiveBrainNode()

    # Two nodes need to process callbacks at the same time:
    # the Brain node itself and its child robot controller node
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    executor.add_node(node.controller)

    # Spin the ROS2 callback threads in the background
    ros_thread = threading.Thread(target=executor.spin, daemon=True)
    ros_thread.start()

    # Run the main asyncio event loop (Gemini Live WebSocket Client)
    try:
        asyncio.get_event_loop().run_until_complete(node.run_live_session())
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
