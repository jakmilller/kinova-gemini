#!/home/mcrr-lab/anaconda3/envs/kinova-gemini/bin/python3

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from ros2_interfaces.msg import RobotState
from cv_bridge import CvBridge
from google import genai
from google.genai import types
import os
import yaml
import sys
import json
import re
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

from .robot_controller_ros2 import KinovaRobotControllerROS2
from .gemini_live_tools import ALL_TOOLS
from . import vision_utils

WORKSPACE_PATH = os.path.expanduser('~/kinova-gemini')

# Optional model dependencies
sys.path.append(os.path.join(WORKSPACE_PATH, 'anygrasp_sdk'))
sys.path.append(os.path.join(WORKSPACE_PATH, 'anygrasp_sdk', 'grasp_detection'))
try:
    from gsnet import AnyGrasp
    ANYGRASP_AVAILABLE = True
except ImportError:
    ANYGRASP_AVAILABLE = False

SAM2_REPO_PATH = "/home/mcrr-lab/raf-live/SAM2_streaming"
sys.path.append(SAM2_REPO_PATH)
try:
    from sam2.build_sam import build_sam2
    from sam2.sam2_image_predictor import SAM2ImagePredictor
    SAM2_AVAILABLE = True
except ImportError:
    SAM2_AVAILABLE = False


def _ts():
    return datetime.now().strftime("%H:%M:%S.%f")[:-3]


def _transform_to_matrix(t_msg):
    mat = np.eye(4)
    q = [t_msg.transform.rotation.x, t_msg.transform.rotation.y,
         t_msg.transform.rotation.z, t_msg.transform.rotation.w]
    mat[:3, :3] = R.from_quat(q).as_matrix()
    mat[:3,  3] = [t_msg.transform.translation.x,
                   t_msg.transform.translation.y,
                   t_msg.transform.translation.z]
    return mat


def _angular_dist(R1, R2):
    trace = np.trace(R1.T @ R2)
    return np.arccos(np.clip((trace - 1.0) / 2.0, -1.0, 1.0))


class GeminiLiveBrainNode(Node):
    def __init__(self):
        super().__init__('gemini_live_brain_node')
        load_dotenv(os.path.join(WORKSPACE_PATH, '.env'))
        with open(os.path.join(WORKSPACE_PATH, 'config.yaml'), 'r') as f:
            self.robot_config = yaml.safe_load(f)

        self.client = genai.Client(api_key=os.getenv('gemini_api_key'))
        self.live_model_id = self.robot_config['model']['live']
        self.flash_model_id = self.robot_config['model']['flash']
        with open(os.path.join(WORKSPACE_PATH, 'src/prompts/system_live_prompt.txt')) as f:
            self.system_instruction = f.read()
        self.tools = [types.Tool(function_declarations=ALL_TOOLS)]

        self._init_vision_models()
        self._init_state()
        self._init_ros()

        # Tool dispatch table — name → async handler returning (success, detail_string).
        self._tool_handlers = {
            "move_to_home":     self._handle_move_to_home,
            "stop_robot":       self._handle_stop_robot,
            "move_to_user":     self._handle_move_to_user,
            "grasp_object":     self._handle_grasp_object,
            "open_gripper":     self._handle_open_gripper,
            "move_to_position": self._handle_move_to_position,
            "adjust_joints":    self._handle_adjust_joints,
            "inspect_scene":    self._handle_inspect_scene,
        }
        # Handlers in this set publish their own chat output (e.g. with an image),
        # so the dispatcher skips the generic "Tool Result" chat message.
        self._handlers_owning_chat = {"inspect_scene"}

        self.get_logger().info('Gemini Live Brain Node initialized.')

    # ---- Initialization ----

    def _init_vision_models(self):
        if ANYGRASP_AVAILABLE:
            self.get_logger().info('Initializing AnyGrasp...')
            args = argparse.Namespace(
                checkpoint_path=os.path.join(WORKSPACE_PATH, 'anygrasp_sdk/grasp_detection/log/checkpoint_detection.tar'),
                max_gripper_width=0.14,
                gripper_height=0.07,
                top_down_grasp=False,
                debug=False,
            )
            self.anygrasp = AnyGrasp(args)
            self.anygrasp.load_net()
        else:
            self.get_logger().warn('AnyGrasp NOT available.')
            self.anygrasp = None

        if SAM2_AVAILABLE:
            self.get_logger().info('Initializing SAM2...')
            sam2_ckpt = os.path.join(SAM2_REPO_PATH, "checkpoints/sam2/sam2_hiera_tiny.pt")
            self.sam_predictor = SAM2ImagePredictor(
                build_sam2("sam2/sam2_hiera_t.yaml", sam2_ckpt, device="cuda")
            )
        else:
            self.get_logger().warn('SAM2 NOT available.')
            self.sam_predictor = None

    def _init_state(self):
        self.controller = KinovaRobotControllerROS2()
        self.bridge = CvBridge()
        self.p = pyaudio.PyAudio()

        self.latest_rgb_image = None
        self.latest_depth_image = None
        self.latest_camera_info = None
        self.current_robot_state = None
        self.transcription_buffer = ""
        self.is_moving = False

        # Asyncio state — populated when run_live_session starts
        self.loop = None
        self.session_instance = None
        self.text_input_queue = asyncio.Queue()
        self.speaker_queue = asyncio.Queue()
        self.active_robot_task = None

    def _init_ros(self):
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.reentrant_group = ReentrantCallbackGroup()

        self.create_subscription(Image, '/camera/realsense/color/image_raw',
                                 self.rgb_callback, 10, callback_group=self.reentrant_group)
        self.create_subscription(Image, '/camera/realsense/aligned_depth_to_color/image_raw',
                                 self.depth_callback, 10, callback_group=self.reentrant_group)
        self.create_subscription(CameraInfo, '/camera/realsense/aligned_depth_to_color/camera_info',
                                 self.camera_info_callback, 10, callback_group=self.reentrant_group)
        self.create_subscription(RobotState, 'robot_state',
                                 self.state_callback, 10, callback_group=self.reentrant_group)
        self.create_subscription(String, '/user_instructions',
                                 self.instruction_callback, 10, callback_group=self.reentrant_group)

        self.chat_pub = self.create_publisher(String, '/gemini_chat', 10)

    # ---- ROS callbacks ----

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
        self.current_robot_state = msg

    def instruction_callback(self, msg):
        user_text = msg.data
        self.get_logger().info(f'--- New User Goal: "{user_text}" ---')
        self.publish_chat_message("user", user_text)
        if self.loop is not None:
            self.loop.call_soon_threadsafe(self.text_input_queue.put_nowait, user_text)
        else:
            self.get_logger().warn("Async loop is not set up yet. Ignoring instruction.")

    # ---- UI helpers ----

    def publish_chat_message(self, role, text, image=None):
        msg_data = {"role": role, "text": text, "timestamp": time.time()}
        if image is not None:
            # JPEG-encode at quality 60 to keep rosbridge messages under the WebSocket size cap
            buffered = BytesIO()
            image.convert('RGB').save(buffered, format="JPEG", quality=60, optimize=True)
            img_str = base64.b64encode(buffered.getvalue()).decode()
            msg_data["image"] = f"data:image/jpeg;base64,{img_str}"
        self.chat_pub.publish(String(data=json.dumps(msg_data)))

    # ---- Sensor / vision helpers ----

    async def _wait_for_frames(self, need_camera_info=False, need_state=False, attempts=10):
        """Wait up to attempts*0.5s for fresh sensor data. Returns True if all required data is present."""
        for _ in range(attempts):
            ok = self.latest_rgb_image is not None and self.latest_depth_image is not None
            if need_camera_info and self.latest_camera_info is None:
                ok = False
            if need_state and self.current_robot_state is None:
                ok = False
            if ok:
                return True
            self.get_logger().info("Waiting for sensor data...")
            await asyncio.sleep(0.5)
        return False

    async def _query_gemini_json(self, prompt, pil_image):
        """Run a synchronous Gemini vision call in a worker thread so the event loop keeps moving.
        Falls back to a salvage parser when Gemini emits malformed JSON (it does, sometimes).
        """
        def _call():
            return self.client.models.generate_content(
                model=self.flash_model_id,
                contents=[pil_image, prompt],
                config=types.GenerateContentConfig(
                    response_mime_type="application/json", temperature=0.0,
                ),
            )
        response = await asyncio.to_thread(_call)
        self.get_logger().info(f"Vision response: {response.text}")
        try:
            return json.loads(response.text)
        except json.JSONDecodeError as e:
            self.get_logger().warn(f"Gemini returned invalid JSON ({e}); salvaging items via regex.")
            return self._salvage_json_objects(response.text)

    @staticmethod
    def _salvage_json_objects(text):
        """Pull individual {...} objects out of possibly-malformed JSON.
        Tries strict parse per chunk first; if that fails, extracts box + last-quoted-string as label.
        """
        items = []
        for match in re.finditer(r'\{[^{}]*\}', text):
            chunk = match.group(0)
            try:
                items.append(json.loads(chunk))
                continue
            except json.JSONDecodeError:
                pass
            # Field-level extraction as a last resort
            box_match = re.search(r'"box(?:_2d)?"\s*:\s*\[([^\]]+)\]', chunk)
            if not box_match:
                continue
            try:
                box = [int(float(x.strip())) for x in box_match.group(1).split(',')]
            except ValueError:
                continue
            if len(box) != 4:
                continue
            # Last quoted string in the chunk is reliably the semantic name,
            # even when the model emits malformed pairs like `"label": "item name": "cup"`.
            labels = re.findall(r'"([^"]+)"', chunk)
            label = labels[-1] if labels else 'object'
            items.append({'box_2d': box, 'label': label})
        return items

    @staticmethod
    def _normalize_box_label(item):
        """Return (box, label) from one Gemini-shaped dict, tolerating common key variants.
        Returns (None, None) if no usable box is present.
        """
        if not isinstance(item, dict):
            return None, None
        box = item.get('box_2d') or item.get('box')
        label = (item.get('label') or item.get('item name') or item.get('part name')
                 or item.get('item_name') or item.get('part_name') or 'object')
        if isinstance(box, list) and len(box) == 4:
            return box, label
        return None, None

    @classmethod
    def _iter_box_label_pairs(cls, value):
        """Yield (box_2d, label) pairs out of any reasonable Gemini response shape:
        bare list, single dict, dict-wrapped list, list of dicts, etc.
        """
        if isinstance(value, list):
            for item in value:
                yield from cls._iter_box_label_pairs(item)
        elif isinstance(value, dict):
            box, label = cls._normalize_box_label(value)
            if box is not None:
                yield box, label
            else:
                for v in value.values():
                    if isinstance(v, (list, dict)):
                        yield from cls._iter_box_label_pairs(v)

    async def _run_sam2(self, rgb_image, box):
        """Box-prompt SAM2 to refine a Gemini bounding box into a binary mask.
        rgb_image: HxWx3 RGB; box: (xmin, ymin, xmax, ymax). Returns HxW bool mask, or None if SAM2 missing.
        """
        if not self.sam_predictor:
            return None
        h, w = rgb_image.shape[:2]
        xmin, ymin, xmax, ymax = box

        def _predict():
            with torch.inference_mode(), torch.autocast(device_type="cuda", dtype=torch.bfloat16):
                self.sam_predictor.set_image(rgb_image)
                masks, _, _ = self.sam_predictor.predict(
                    box=np.array([xmin, ymin, xmax, ymax]),
                    multimask_output=False,
                )
            return masks

        masks = await asyncio.to_thread(_predict)
        mask = masks[0].cpu().numpy() > 0 if torch.is_tensor(masks) else masks[0] > 0

        # Clip the mask to inside its own bounding box (SAM2 sometimes spills slightly)
        box_mask = np.zeros((h, w), dtype=bool)
        box_mask[max(0, ymin):min(h, ymax), max(0, xmin):min(w, xmax)] = True
        mask = mask & box_mask

        torch.set_default_dtype(torch.float32)
        torch.cuda.empty_cache()
        return mask

    @staticmethod
    def _mask_centroid(mask, fallback_box):
        M = cv2.moments(mask.astype(np.uint8))
        if M["m00"] != 0:
            return int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])
        xmin, ymin, xmax, ymax = fallback_box
        return int((xmin + xmax) / 2), int((ymin + ymax) / 2)

    @staticmethod
    def _load_annotation_font(size=18):
        for path in ("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
                     "DejaVuSans-Bold.ttf"):
            try:
                return ImageFont.truetype(path, size)
            except Exception:
                continue
        return ImageFont.load_default()

    # ---- Tool handlers (one method per tool) ----

    async def _handle_move_to_home(self, args):
        ok = await self.controller.move_to_home()
        return ok, "Robot has returned to the home position." if ok else "Failed to move to home position."

    async def _handle_stop_robot(self, args):
        ok = await self.controller.stop_robot()
        return ok, "Robot has stopped (not moving)." if ok else "Failed to stop robot."

    async def _handle_move_to_user(self, args):
        ok = await self.controller.move_to_user()
        return ok, "Robot has moved to the user position." if ok else "Failed to move to user position."

    async def _handle_grasp_object(self, args):
        object_label = (args or {}).get('object_label') or None
        x = (args or {}).get('x')
        y = (args or {}).get('y')
        z = (args or {}).get('z')
        if object_label:
            ok, detail = await self.grasp_object_pipeline(object_label=object_label)
        elif x is not None and y is not None and z is not None:
            ok, detail = await self.grasp_object_pipeline(x=float(x), y=float(y), z=float(z))
        else:
            return False, "grasp_object requires either object_label or x/y/z coordinates."
        return ok, (f"Grasped '{object_label or 'object at coordinates'}'." if ok else detail)

    async def _handle_open_gripper(self, args):
        self.controller.release_grasped_object()
        ok = await self.controller.set_gripper(0.0)
        return ok, "Gripper is now fully opened." if ok else "Failed to open gripper."

    async def _handle_move_to_position(self, args):
        if not self.current_robot_state:
            return False, "Failed: Robot state is not currently available."
        tcp_x, tcp_y, tcp_z, theta_x, theta_y, theta_z = self.controller.get_tcp_pose()
        x = float(args.get('x', tcp_x))
        y = float(args.get('y', tcp_y))
        z = float(args.get('z', tcp_z))
        ok = await self.controller.move_to_pose(x, y, z, theta_x, theta_y, theta_z)
        return ok, (f"Moved to coordinates (x={x:.3f}, y={y:.3f}, z={z:.3f})"
                    if ok else "Failed to reach target coordinates.")

    async def _handle_adjust_joints(self, args):
        if not self.current_robot_state:
            return False, "Failed: Robot state is not currently available."
        angles = list(self.current_robot_state.joint_angles)
        joint_idx = int(args.get('joint number', 0)) - 1
        amount = float(args.get('amount', 0.0))
        if not (0 <= joint_idx < len(angles)):
            return False, f"Failed: Invalid joint number {joint_idx + 1}"
        angles[joint_idx] += amount
        ok = await self.controller.move_to_joints(angles)
        return ok, (f"Adjusted joint {joint_idx + 1} by {amount} degrees."
                    if ok else "Failed joint adjustment movement.")

    async def _handle_inspect_scene(self, args):
        target_location = (args or {}).get('target_location') or None
        if target_location:
            self.get_logger().info(f"Running scene inspection (target location: '{target_location}')...")
        else:
            self.get_logger().info("Running high-precision scene inspection locally...")
        success, report, annotated_img = await self.inspect_scene(target_location=target_location)
        # inspect_scene owns its own chat output so the annotated image goes through
        self.publish_chat_message("system", report, annotated_img if success else None)
        return success, report if success else f"Scene inspection failed: {report}"

    # ---- Tool dispatcher ----

    async def execute_tool_task(self, name, args):
        try:
            self.get_logger().info(f"Background task execution started for: {name}")
            handler = self._tool_handlers.get(name)
            if handler is None:
                self.get_logger().error(f"Unknown tool: {name}")
                success, detail = False, f"Unknown tool: {name}"
            else:
                success, detail = await handler(args)

            if name not in self._handlers_owning_chat:
                status_str = "SUCCESS" if success else "FAILED"
                self.publish_chat_message("system", f"Tool Result ({name}): {status_str}\n{detail}")

            await self._send_event_to_model(name, success, detail)

        except asyncio.CancelledError:
            self.get_logger().warn(f"Task {name} was cancelled during background execution.")
        except Exception as e:
            self.get_logger().error(f"Error in background task {name}: {e}")
            if name not in self._handlers_owning_chat:
                self.publish_chat_message("system", f"Tool Error ({name}): {e}")
            await self._send_event_to_model(name, False, f"exception: {e}")

    async def _send_event_to_model(self, name, success, detail):
        if not self.session_instance:
            return
        status_str = "SUCCESS" if success else "FAILED"
        event_message = f"SYSTEM EVENT: {name} finished. Status: {status_str}. Detail: {detail}"
        self.get_logger().info(f"[{_ts()}] - Sent to model: {event_message}")
        await self.session_instance.send_realtime_input(text=event_message)

    # ---- Vision: scene inspection ----

    async def inspect_scene(self, target_location=None):
        if not await self._wait_for_frames():
            return False, "No RGB image received from RealSense.", None
        if self.latest_camera_info is None:
            return False, "Camera info not available.", None

        # Fresh snapshot: clear stale obstacles so objects that moved/left don't linger.
        self.controller.clear_dynamic_obstacles()

        # Snapshot all inputs so they can't change mid-pipeline
        cv_rgb = cv2.cvtColor(self.latest_rgb_image, cv2.COLOR_BGR2RGB)
        captured_depth = self.latest_depth_image.copy()
        orig_h, orig_w = captured_depth.shape

        # Precompute 3D points map for cuboid dimension extraction
        fx, fy = self.latest_camera_info.k[0], self.latest_camera_info.k[4]
        cx, cy = self.latest_camera_info.k[2], self.latest_camera_info.k[5]
        scale = 1000.0
        xmap, ymap = np.meshgrid(np.arange(orig_w), np.arange(orig_h))
        points_z = captured_depth.astype(np.float32) / scale
        points_x = (xmap - cx) / fx * points_z
        points_y = (ymap - cy) / fy * points_z
        points_3d = np.stack([points_x, points_y, points_z], axis=-1)

        # Resized PIL image is what we send to Gemini AND what we annotate for the UI
        pil_img = PILImage.fromarray(cv_rgb)
        new_width = 800
        new_height = int(new_width * pil_img.size[1] / pil_img.size[0])
        img_resized = pil_img.resize((new_width, new_height), PILImage.Resampling.LANCZOS)

        prompt = textwrap.dedent("""\
            You are viewing the workspace of a general manipulator robot.
            Identify all of the prominent items in the scene.

            Return ONLY a valid JSON list. Each list element MUST be an object with EXACTLY these two keys:
              "box_2d": a list [ymin, xmin, ymax, xmax] of integers normalized to 0-1000
              "label":  a short descriptive string naming the item

            Do NOT use any other key names (do not use "box", "item name", or "part name").
            Do NOT nest objects. Each item is one flat object with the two keys above.

            Example:
            [
              {"box_2d": [100, 200, 300, 400], "label": "cup"},
              {"box_2d": [500, 600, 700, 800], "label": "spoon"}
            ]
            """)

        if target_location:
            prompt += textwrap.dedent(f"""

                ADDITIONALLY, include ONE extra item in the same list representing a SPECIFIC SPATIAL
                REGION described as: "{target_location}"

                This region may NOT be a discrete physical object — it could be a square on a structured
                layout (e.g. a chess board), a compartment of a container, or a spot defined relative
                to other objects in the scene. Use the same {{"box_2d": [...], "label": "..."}} shape,
                but prefix the label with the literal string "TARGET: " so it can be told apart from
                the regular items. This bounding box should be around the CENTER of the location, so it can be much smaller and more precise.

                Example with target_location = "third chess square forward from queen":
                [
                  {{"box_2d": [100, 200, 300, 400], "label": "queen"}},
                  {{"box_2d": [120, 700, 200, 780], "label": "rook"}},
                  {{"box_2d": [410, 380, 500, 470], "label": "TARGET: third square forward from queen"}}
                ]
                """)

        try:
            raw = await self._query_gemini_json(prompt, img_resized)
        except Exception as e:
            self.get_logger().error(f"Error querying Gemini: {e}")
            return False, f"Inspection failed: {e}", None

        items = list(self._iter_box_label_pairs(raw))
        self.get_logger().info(f"Processing {len(items)} detected items with SAM2...")

        if not items:
            return True, "No items detected in the scene.", img_resized

        try:
            transform = self.tf_buffer.lookup_transform(
                'base_link', 'physical_realsense_link',
                rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0),
            )
        except TransformException as ex:
            return False, f"Transformation error: {ex}", None

        annotated_img = img_resized.copy()
        draw = ImageDraw.Draw(annotated_img)
        font = self._load_annotation_font(size=18)
        colors = ["red", "green", "blue", "yellow", "orange", "pink", "purple", "cyan", "magenta"]

        scene_report = "Scene Inspection Report:\n"

        for i, (box_2d, label) in enumerate(items):
            ymin_norm, xmin_norm, ymax_norm, xmax_norm = box_2d
            # Original-resolution coords for SAM2 / depth lookup
            ymin = int(ymin_norm * orig_h / 1000)
            xmin = int(xmin_norm * orig_w / 1000)
            ymax = int(ymax_norm * orig_h / 1000)
            xmax = int(xmax_norm * orig_w / 1000)

            # Items labeled "TARGET: ..." are the user-specified spatial region.
            # Draw them in white with a thicker outline, and report them under a separate heading.
            is_target = label.upper().startswith("TARGET")
            if is_target:
                clean_label = label.split(":", 1)[1].strip() if ":" in label else label
                color = "white"
                box_width = 5
                draw_label = f"TARGET: {clean_label}"
                report_prefix = f"TARGET LOCATION '{clean_label}'"
            else:
                clean_label = label
                color = colors[i % len(colors)]
                box_width = 3
                draw_label = label
                report_prefix = f"- {label}"

            # Draw the box FIRST — independent of whether depth succeeds. This used to live after
            # the depth check, so items without valid depth silently dropped off the annotated image.
            r_ymin = ymin_norm * new_height / 1000
            r_xmin = xmin_norm * new_width / 1000
            r_ymax = ymax_norm * new_height / 1000
            r_xmax = xmax_norm * new_width / 1000
            draw.rectangle(((r_xmin, r_ymin), (r_xmax, r_ymax)), outline=color, width=box_width)
            draw.text((r_xmin + 5, r_ymin + 5), draw_label, fill=color, font=font)

            mask = await self._run_sam2(cv_rgb, (xmin, ymin, xmax, ymax))
            if mask is None:
                scene_report += f"{report_prefix}: Pose unknown (SAM2 not available).\n"
                continue

            valid_depths = captured_depth[mask][captured_depth[mask] > 0]
            if len(valid_depths) == 0:
                scene_report += f"{report_prefix}: Pose unknown (no valid depth in mask).\n"
                continue

            depth_m = float(np.median(valid_depths)) / 1000.0
            px, py = self._mask_centroid(mask, (xmin, ymin, xmax, ymax))
            cx, cy, cz = vision_utils.get_3d_point_from_pixel(px, py, depth_m, self.latest_camera_info)
            bx, by, bz = vision_utils.transform_point(cx, cy, cz, transform)
            scene_report += f"{report_prefix}: Pose(x={bx:.3f}, y={by:.3f}, z={bz:.3f}) relative to base.\n"

            # Compute a tight PCA-oriented bounding box (OBB) for cuRobo obstacle registration.
            # We keep Z world-vertical (upright table objects) and only rotate around Z (yaw OBB),
            # which is both tighter than AABB for diagonal objects and avoids tilted boxes on the table.
            if not is_target:
                obj_points_cam = points_3d[mask].reshape(-1, 3)
                obj_points_cam = obj_points_cam[(obj_points_cam[:, 2] > 0.1) & (obj_points_cam[:, 2] < 1.0)]
                if obj_points_cam.shape[0] >= 10:
                    T_base_cam = _transform_to_matrix(transform)
                    ones = np.ones((obj_points_cam.shape[0], 1))
                    obj_points_base = (T_base_cam @ np.hstack([obj_points_cam, ones]).T).T[:, :3]

                    # PCA on XY footprint only → principal yaw angle
                    xy = obj_points_base[:, :2]
                    xy_centered = xy - xy.mean(axis=0)
                    cov = (xy_centered.T @ xy_centered) / max(len(xy_centered) - 1, 1)
                    _, vecs = np.linalg.eigh(cov)
                    # eigh returns ascending eigenvalues; primary axis = last column
                    primary = vecs[:, -1]
                    yaw = float(np.arctan2(primary[1], primary[0]))

                    # Rotate points into the OBB frame, compute tight extents, rotate center back
                    c_yaw, s_yaw = np.cos(-yaw), np.sin(-yaw)
                    rot2d = np.array([[c_yaw, -s_yaw], [s_yaw, c_yaw]])
                    pts_obb = (rot2d @ obj_points_base[:, :2].T).T
                    obb_min = pts_obb.min(axis=0)
                    obb_max = pts_obb.max(axis=0)
                    obb_center_2d = (obb_min + obb_max) / 2.0
                    # Rotate OBB center back to base frame
                    center_base_xy = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]]) @ obb_center_2d
                    z_min = obj_points_base[:, 2].min()
                    z_max = obj_points_base[:, 2].max()
                    cuboid_center = [float(center_base_xy[0]), float(center_base_xy[1]), float((z_min + z_max) / 2.0)]
                    cuboid_dims = [float(obb_max[0] - obb_min[0]), float(obb_max[1] - obb_min[1]), float(z_max - z_min)]

                    # Convert yaw to wxyz quaternion (rotation around Z axis)
                    quat_wxyz = [float(np.cos(yaw / 2)), 0.0, 0.0, float(np.sin(yaw / 2))]

                    self.controller.update_dynamic_obstacle(clean_label, cuboid_center, cuboid_dims, quat_wxyz)

        self.get_logger().info(f"Annotated {len(items)} bounding boxes on inspection image.")
        return True, scene_report, annotated_img

    # ---- Vision: 6D grasp ----

    async def grasp_object_pipeline(self, object_label=None, x=None, y=None, z=None):
        """
        Unified grasp pipeline. Two modes:
          - object_label: AnyGrasp 6D vision pipeline (preferred for complex objects).
          - x/y/z: SAM2+depth centroid fallback using given TCP contact coordinates,
                   or if SAM2 also unavailable, direct Cartesian approach.
        All Cartesian moves go through controller.move_to_pose() → cuRobo for collision avoidance.
        """
        if object_label:
            return await self._grasp_anygrasp(object_label)
        else:
            return await self._grasp_cartesian(x, y, z)

    async def _grasp_anygrasp(self, object_label):
        self.get_logger().info(f"Executing AnyGrasp 6D grasp for: {object_label}")

        if not await self._wait_for_frames(need_camera_info=True, need_state=True):
            return False, "Sensor data not available."

        cv_rgb = cv2.cvtColor(self.latest_rgb_image, cv2.COLOR_BGR2RGB)
        pil_img = PILImage.fromarray(cv_rgb)

        prompt = textwrap.dedent(f"""\
            Identify the most easily graspable part of the '{object_label}'
            (e.g. handle, rim, or the whole body if it is small).

            Return ONLY a single valid JSON object — NOT a list — with EXACTLY these two keys:
              "box_2d": a list [ymin, xmin, ymax, xmax] of integers normalized to 0-1000
              "label":  the part's name (a short string)

            Do NOT use any other key names (do not use "box", "item name", or "part name").

            Example: {{"box_2d": [100, 200, 300, 400], "label": "handle"}}
            """)

        try:
            raw = await self._query_gemini_json(prompt, pil_img)
        except Exception as e:
            return False, f"Gemini box parsing failed: {e}"

        pairs = list(self._iter_box_label_pairs(raw))
        if not pairs:
            return False, f"Gemini did not return a valid bounding box. Raw: {raw}"
        box_2d, label = pairs[0]
        ymin_norm, xmin_norm, ymax_norm, xmax_norm = box_2d

        h, w = self.latest_rgb_image.shape[:2]
        ymin = int(ymin_norm * h / 1000); xmin = int(xmin_norm * w / 1000)
        ymax = int(ymax_norm * h / 1000); xmax = int(xmax_norm * w / 1000)

        self.get_logger().info(f"Refining '{label}' segmentation with SAM2 box-prompt...")
        binary_mask = await self._run_sam2(cv_rgb, (xmin, ymin, xmax, ymax))
        if binary_mask is None:
            return False, "SAM2 not available; cannot run AnyGrasp without a mask. Use coordinate mode instead."

        ag_points, ag_colors, lims = self._build_anygrasp_clouds(cv_rgb, binary_mask)
        if ag_points is None:
            return False, "Target point cloud is too small."

        if not self.anygrasp:
            # AnyGrasp unavailable: fall back to depth-centroid approach using the SAM2 mask
            self.get_logger().info("AnyGrasp not available — falling back to SAM2+depth centroid grasp.")
            return await self._grasp_sam2_centroid(object_label, cv_rgb, binary_mask)

        self.get_logger().info("Querying AnyGrasp for 6D grasp generation...")
        torch.cuda.empty_cache()
        gg, _ = await asyncio.to_thread(
            self.anygrasp.get_grasp,
            ag_points, ag_colors,
            lims=lims, apply_object_mask=True, dense_grasp=True, collision_detection=True,
        )
        torch.cuda.empty_cache()
        if len(gg) == 0:
            return False, "No grasps detected."

        gg = gg.nms().sort_by_score()
        best_grasp = gg[0]
        self.get_logger().info(f"Best Grasp: Width={best_grasp.width:.3f}m, Depth={best_grasp.depth:.3f}m")

        # Pre-shape gripper to grasp width before approach
        desired_width = best_grasp.width + 0.02
        target_gripper_pos = max(0.0, min(100.0, (0.14 - desired_width) / 0.14 * 100.0))
        self.get_logger().info(f"Pre-shaping gripper to {target_gripper_pos:.1f}%...")
        await self.controller.set_gripper(target_gripper_pos)
        await asyncio.sleep(1.0)

        try:
            t_base_cam = self.tf_buffer.lookup_transform(
                'base_link', self.latest_camera_info.header.frame_id,
                rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=2.0),
            )
        except Exception:
            return False, "TF lookup failed"

        T_base_cam = _transform_to_matrix(t_base_cam)
        R_align = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]])
        T_cam_grasp = np.eye(4)
        T_cam_grasp[:3, :3] = best_grasp.rotation_matrix
        T_cam_grasp[:3,  3] = best_grasp.translation
        T_base_grasp = T_base_cam @ T_cam_grasp
        r_base_grasp = T_base_grasp[:3, :3] @ R_align
        grasp_tcp = T_base_grasp[:3, 3]

        # Pick wrist orientation closer to current to minimize unnecessary rotation
        try:
            t_current = self.tf_buffer.lookup_transform(
                'base_link', 'end_effector_link',
                rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0),
            )
            R_curr = _transform_to_matrix(t_current)[:3, :3]
            R_flip = r_base_grasp @ R.from_euler('z', 180, degrees=True).as_matrix()
            if _angular_dist(R_curr, R_flip) < _angular_dist(R_curr, r_base_grasp):
                r_base_grasp = R_flip
        except Exception:
            pass

        approach_dist = best_grasp.depth + 0.02
        approach_vector = r_base_grasp[:, 2]
        orient_euler = R.from_matrix(r_base_grasp).as_euler('xyz', degrees=True)

        pre_grasp_tcp = grasp_tcp - approach_dist * approach_vector

        # Remove target obstacle so cuRobo doesn't block the approach
        self.controller.remove_dynamic_obstacle(object_label)

        self.get_logger().info("Moving to pre-grasp TCP position via cuRobo...")
        ok = await self.controller.move_to_pose(
            pre_grasp_tcp[0], pre_grasp_tcp[1], pre_grasp_tcp[2],
            orient_euler[0], orient_euler[1], orient_euler[2],
        )
        if not ok:
            return False, "Pre-grasp move failed."

        self.get_logger().info("Sliding to grasp TCP position...")
        ok = await self.controller.move_to_pose(
            grasp_tcp[0], grasp_tcp[1], grasp_tcp[2],
            orient_euler[0], orient_euler[1], orient_euler[2],
        )
        if not ok:
            return False, "Grasp approach move failed."

        self.get_logger().info("Closing gripper...")
        await self.controller.grasp_object()

        # Record the held object for cuRobo carry collision checking
        dims = [best_grasp.width, best_grasp.width, best_grasp.depth]
        self.controller.attach_grasped_object(object_label, dims)
        return True, ""

    async def _grasp_sam2_centroid(self, object_label, cv_rgb, binary_mask):
        """Fallback grasp: uses SAM2 mask centroid + depth for the TCP target, current EE orientation."""
        self.get_logger().info(f"SAM2 centroid fallback grasp for '{object_label}'")

        if self.latest_camera_info is None:
            return False, "Camera info not available."

        try:
            transform = self.tf_buffer.lookup_transform(
                'base_link', self.latest_camera_info.header.frame_id,
                rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=2.0),
            )
        except Exception as ex:
            return False, f"TF lookup failed: {ex}"

        # Centroid from mask + median depth
        captured_depth = self.latest_depth_image.copy()
        valid_depths = captured_depth[binary_mask][captured_depth[binary_mask] > 0]
        if len(valid_depths) == 0:
            return False, "No valid depth in SAM2 mask."
        depth_m = float(np.median(valid_depths)) / 1000.0
        h_img, w_img = cv_rgb.shape[:2]
        M = cv2.moments(binary_mask.astype(np.uint8))
        px = int(M["m10"] / M["m00"]) if M["m00"] != 0 else w_img // 2
        py = int(M["m01"] / M["m00"]) if M["m00"] != 0 else h_img // 2
        cx_c, cy_c, cz_c = vision_utils.get_3d_point_from_pixel(px, py, depth_m, self.latest_camera_info)
        grasp_tcp = np.array(vision_utils.transform_point(cx_c, cy_c, cz_c, transform))

        # Use current EE orientation for approach
        _, _, _, orient_x, orient_y, orient_z = self.controller.get_tcp_pose()
        approach_dist = 0.08

        # Approach vector is the EE Z axis in world frame
        r_current = R.from_euler('xyz', [orient_x, orient_y, orient_z], degrees=True)
        approach_vector = r_current.as_matrix()[:, 2]
        pre_grasp_tcp = grasp_tcp - approach_dist * approach_vector

        # Open gripper fully (no width info without AnyGrasp)
        await self.controller.set_gripper(0.0)
        await asyncio.sleep(1.0)

        self.controller.remove_dynamic_obstacle(object_label)

        self.get_logger().info("Moving to pre-grasp TCP position (SAM2 centroid) via cuRobo...")
        ok = await self.controller.move_to_pose(
            pre_grasp_tcp[0], pre_grasp_tcp[1], pre_grasp_tcp[2],
            orient_x, orient_y, orient_z,
        )
        if not ok:
            return False, "Pre-grasp move failed."

        self.get_logger().info("Sliding to grasp TCP position...")
        ok = await self.controller.move_to_pose(
            grasp_tcp[0], grasp_tcp[1], grasp_tcp[2],
            orient_x, orient_y, orient_z,
        )
        if not ok:
            return False, "Grasp approach move failed."

        self.get_logger().info("Closing gripper...")
        await self.controller.grasp_object()
        self.controller.attach_grasped_object(object_label, [0.05, 0.05, 0.05])
        return True, ""

    async def _grasp_cartesian(self, x, y, z):
        """Grasp using explicit TCP contact coordinates. Uses current EE orientation."""
        self.get_logger().info(f"Executing Cartesian grasp at TCP ({x:.3f}, {y:.3f}, {z:.3f})")

        if not await self._wait_for_frames(need_state=True):
            return False, "Robot state not available."

        grasp_tcp = np.array([x, y, z])
        _, _, _, orient_x, orient_y, orient_z = self.controller.get_tcp_pose()
        approach_dist = 0.08
        r_current = R.from_euler('xyz', [orient_x, orient_y, orient_z], degrees=True)
        approach_vector = r_current.as_matrix()[:, 2]
        pre_grasp_tcp = grasp_tcp - approach_dist * approach_vector

        await self.controller.set_gripper(0.0)
        await asyncio.sleep(1.0)

        self.get_logger().info("Moving to pre-grasp TCP position via cuRobo...")
        ok = await self.controller.move_to_pose(
            pre_grasp_tcp[0], pre_grasp_tcp[1], pre_grasp_tcp[2],
            orient_x, orient_y, orient_z,
        )
        if not ok:
            return False, "Pre-grasp move failed."

        self.get_logger().info("Sliding to grasp TCP position...")
        ok = await self.controller.move_to_pose(
            grasp_tcp[0], grasp_tcp[1], grasp_tcp[2],
            orient_x, orient_y, orient_z,
        )
        if not ok:
            return False, "Grasp approach move failed."

        self.get_logger().info("Closing gripper...")
        await self.controller.grasp_object()
        self.controller.attach_grasped_object("grasped_object", [0.05, 0.05, 0.05])
        return True, ""

    def _build_anygrasp_clouds(self, cv_rgb, binary_mask):
        """Lift depth + mask into the (points, colors, lims) tuple AnyGrasp wants.
        Returns (None, None, None) if the target mask has too few valid points.
        """
        h, w = cv_rgb.shape[:2]
        depths = self.latest_depth_image.astype(np.float32)
        fx, fy = self.latest_camera_info.k[0], self.latest_camera_info.k[4]
        cx, cy = self.latest_camera_info.k[2], self.latest_camera_info.k[5]
        scale = 1000.0

        xmap, ymap = np.meshgrid(np.arange(w), np.arange(h))
        points_z = depths / scale
        points_x = (xmap - cx) / fx * points_z
        points_y = (ymap - cy) / fy * points_z
        points = np.stack([points_x, points_y, points_z], axis=-1).astype(np.float32)
        colors = cv_rgb.astype(np.float32) / 255.0

        target_points = points[binary_mask].reshape(-1, 3)
        target_points = target_points[(target_points[:, 2] > 0.1) & (target_points[:, 2] < 1.0)]
        if target_points.shape[0] < 10:
            return None, None, None

        margin = 0.005
        obj_min, obj_max = target_points.min(axis=0), target_points.max(axis=0)
        lims = [obj_min[0]-margin, obj_max[0]+margin,
                obj_min[1]-margin, obj_max[1]+margin,
                obj_min[2]-margin, obj_max[2]+margin]

        scene_mask = (points[:, :, 2] > 0.2) & (points[:, :, 2] < 0.75)
        full_points = points[scene_mask].reshape(-1, 3)
        full_colors = colors[scene_mask].reshape(-1, 3)
        full_obj_mask = binary_mask[scene_mask]

        in_lims = ((full_points[:, 0] >= lims[0]) & (full_points[:, 0] <= lims[1]) &
                   (full_points[:, 1] >= lims[2]) & (full_points[:, 1] <= lims[3]) &
                   (full_points[:, 2] >= lims[4]) & (full_points[:, 2] <= lims[5]))
        keep_mask = (in_lims & full_obj_mask) | (~in_lims)
        return full_points[keep_mask], full_colors[keep_mask], lims

    # ---- Live session async tasks ----

    async def send_text_task(self, session):
        """Forwards text from the UI text-input queue to the Gemini Live session."""
        while True:
            text = await self.text_input_queue.get()
            self.get_logger().info(f"[{_ts()}] - Sent to model (User Input): {text}")
            await session.send_realtime_input(text=text)

    async def play_speaker_task(self):
        """Plays output audio chunks from the speaker queue."""
        try:
            stream = await asyncio.to_thread(
                self.p.open, format=pyaudio.paInt16, channels=1, rate=24000, output=True,
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
        """Streams camera frames to Gemini at ~1 fps without prompting state, to avoid output loops."""
        while True:
            if self.latest_rgb_image is not None:
                try:
                    frame_resized = cv2.resize(self.latest_rgb_image, (768, 768))
                    ok, encoded = cv2.imencode('.jpg', frame_resized, [cv2.IMWRITE_JPEG_QUALITY, 50])
                    if ok:
                        await session.send_realtime_input(
                            video=types.Blob(data=encoded.tobytes(), mime_type="image/jpeg")
                        )
                except Exception as e:
                    self.get_logger().error(f"Video stream error: {e}")
            await asyncio.sleep(1.0)

    async def receive_loop_task(self, session):
        """Receives audio, transcription, and tool calls from Gemini."""
        self.session_instance = session
        try:
            while True:
                response = await session._receive()
                if response is None:
                    self.get_logger().warn("Session ended by server")
                    break

                if response.go_away is not None:
                    self.get_logger().warn(f"Server closing connection in: {response.go_away.time_left}")

                if response.server_content:
                    sc = response.server_content
                    if sc.output_transcription:
                        self.transcription_buffer += sc.output_transcription.text
                    if sc.model_turn:
                        for part in sc.model_turn.parts:
                            if part.inline_data:
                                await self.speaker_queue.put(part.inline_data.data)
                    if sc.turn_complete and self.transcription_buffer:
                        text = self.transcription_buffer.strip()
                        self.get_logger().info(f"[{_ts()}] - Received from model (Gemini): {text}")
                        self.publish_chat_message("model", text)
                        self.transcription_buffer = ""

                if response.tool_call:
                    func_responses = []
                    for fc in response.tool_call.function_calls:
                        self.get_logger().info(f"[{_ts()}] - Received from model: Tool request '{fc.name}' with args: {fc.args}")
                        self.publish_chat_message("system", f"Tool Call: {fc.name}\nArgs: {fc.args}")

                        # Acknowledge immediately and run the actual work in the background
                        self.active_robot_task = asyncio.create_task(self.execute_tool_task(fc.name, fc.args))
                        func_responses.append(types.FunctionResponse(
                            name=fc.name, id=fc.id,
                            response={"result": f"{fc.name} initiated in background. Keep talking to the user."},
                        ))

                    if func_responses:
                        names = [fr.name for fr in func_responses]
                        self.get_logger().info(f"[{_ts()}] - Sent to model: Tool acknowledgment for {names}")
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
            tools=self.tools,
        )
        try:
            async with self.client.aio.live.connect(model=self.live_model_id, config=config) as session:
                self.get_logger().info("Connected to Gemini Live!")
                await asyncio.gather(
                    self.video_stream_task(session),
                    self.send_text_task(session),
                    self.play_speaker_task(),
                    self.receive_loop_task(session),
                )
        except Exception as e:
            self.get_logger().error(f"Live session failed: {e}")
        finally:
            self.p.terminate()


def main(args=None):
    rclpy.init(args=args)
    node = GeminiLiveBrainNode()

    # Brain + controller both need to spin so their callbacks process concurrently
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    executor.add_node(node.controller)

    ros_thread = threading.Thread(target=executor.spin, daemon=True)
    ros_thread.start()

    try:
        asyncio.run(node.run_live_session())
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
