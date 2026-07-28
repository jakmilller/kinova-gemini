#!/home/mcrr-lab/anaconda3/envs/kinova-gemini/bin/python3
"""Gemini-Live Brain Node.

The central async reasoning loop: connects to Gemini Live, streams camera frames to the model, plays back the model's audio
replies, and calls tools to move the robot. It serves to orchestrate all of the robot behavior.
See gemini_live_tools.py for in-depth description of available tools.
"""

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
from PIL import Image as PILImage, ImageDraw
from dotenv import load_dotenv
from tf2_ros import Buffer, TransformListener, TransformException

from .robot_controller_ros2 import KinovaRobotControllerROS2
from .gemini_live_tools import ALL_TOOLS
from . import perception
from . import grasp

WORKSPACE_PATH = os.path.expanduser('~/kinova-gemini')

# Gemini Live returns audio replies as 24kHz mono PCM. (Mic input is handled by the
# separate voice_interface_node, not here — this node never opens an input stream.)
SPEAKER_RATE = 24000

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
            "inspect_scene":        self._handle_inspect_scene,
            "move_to_position":     self._handle_move_to_position,
            "move_relative":        self._handle_move_relative,
            "grasp_simple_object":  self._handle_grasp_simple_object,
            "grasp_complex_object": self._handle_grasp_complex_object,
            "adjust_gripper":       self._handle_adjust_gripper,
            "adjust_joints":        self._handle_adjust_joints,
            "move_to_home":         self._handle_move_to_home,
            "move_to_user":         self._handle_move_to_user,
            "stop_robot":           self._handle_stop_robot,
        }
        # Handlers in this set publish their own chat output (e.g. with an image),
        # so the dispatcher skips the generic "Tool Result" chat message.
        self._handlers_owning_chat = {"inspect_scene"}

        self.get_logger().info('Gemini Live Brain Node initialized. Tools: '
                               + ', '.join(self._tool_handlers.keys()))

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
        # Transcribed voice commands and typed commands arrive on the same topic 
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

    def publish_chat_message(self, role, text, image=None, source=None):
        msg_data = {"role": role, "text": text, "timestamp": time.time()}
        if source is not None:
            msg_data["source"] = source
        if image is not None:
            # JPEG-encode at quality 60 to keep rosbridge messages under the WebSocket size cap
            buffered = BytesIO()
            image.convert('RGB').save(buffered, format="JPEG", quality=60, optimize=True)
            img_str = base64.b64encode(buffered.getvalue()).decode()
            msg_data["image"] = f"data:image/jpeg;base64,{img_str}"
        self.chat_pub.publish(String(data=json.dumps(msg_data)))

    # ---- Sensor helpers ----

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

    # ---- Tool handlers (one method per tool) ----

    async def _handle_move_to_home(self, args):
        ok = await self.controller.move_to_home()
        return ok, "Robot has returned to the home position." if ok else "Failed to move to home position."

    async def _handle_move_to_user(self, args):
        ok = await self.controller.move_to_user()
        return ok, "Robot has moved to the user position." if ok else "Failed to move to user position."

    async def _handle_stop_robot(self, args):
        ok = await self.controller.stop_robot()
        return ok, "Robot has stopped (not moving)." if ok else "Failed to stop robot."

    async def _handle_move_to_position(self, args):
        if not self.current_robot_state:
            return False, "Failed: Robot state is not currently available."
        s = self.current_robot_state
        x = float(args['x'])
        y = float(args['y'])
        z = float(args['z'])
        # Keep the current wrist orientation, read straight from robot_state.
        ok = await self.controller.move_to_pose(x, y, z, s.theta_x, s.theta_y, s.theta_z)
        return ok, (f"Moved to coordinates (x={x:.3f}, y={y:.3f}, z={z:.3f})."
                    if ok else "Failed to reach target coordinates.")

    async def _handle_move_relative(self, args):
        if not self.current_robot_state:
            return False, "Failed: Robot state is not currently available."
        s = self.current_robot_state
        dx = float((args or {}).get('dx', 0.0))
        dy = float((args or {}).get('dy', 0.0))
        dz = float((args or {}).get('dz', 0.0))
        x, y, z = s.x + dx, s.y + dy, s.z + dz
        ok = await self.controller.move_to_pose(x, y, z, s.theta_x, s.theta_y, s.theta_z)
        return ok, (f"Moved by (dx={dx:.3f}, dy={dy:.3f}, dz={dz:.3f}) to (x={x:.3f}, y={y:.3f}, z={z:.3f})."
                    if ok else "Failed relative move.")

    async def _handle_adjust_joints(self, args):
        if not self.current_robot_state:
            return False, "Failed: Robot state is not currently available."
        angles = list(self.current_robot_state.joint_angles)
        joint_idx = int(args.get('joint', 0)) - 1
        amount = float(args.get('amount', 0.0))
        if not (0 <= joint_idx < len(angles)):
            return False, f"Failed: Invalid joint number {joint_idx + 1}"
        # robot_state (and the controller's completion-polling wrap math) always represents joint
        # angles in [0, 360) -- wrap here so an adjustment can never produce an out-of-range
        # absolute target (e.g. 359 + 180 = 539), which breaks the C++ side's 360-degree-wrap
        # distance check and stalls until its safety timeout instead of detecting completion.
        angles[joint_idx] = (angles[joint_idx] + amount) % 360.0
        ok = await self.controller.move_to_joints(angles)
        return ok, (f"Adjusted joint {joint_idx + 1} by {amount} degrees."
                    if ok else "Failed joint adjustment movement.")

    async def _handle_adjust_gripper(self, args):
        action = (args or {}).get('action')
        if action == 'open':
            ok = await self.controller.set_gripper(0.0)
            return ok, "Gripper is now fully opened." if ok else "Failed to open gripper."
        if action == 'close':
            ok = await self.controller.grasp_object()
            if not ok:
                return False, "Failed to close gripper."
            return True, "Gripper closed." + self._gripper_closed_warning()
        if action == 'custom':
            width = (args or {}).get('width')
            if width is None:
                return False, "adjust_gripper action='custom' requires a width (meters)."
            pct = grasp.width_to_gripper_percent(float(width))
            ok = await self.controller.set_gripper(pct)
            return ok, (f"Gripper set to accommodate {float(width) * 100:.1f} cm." if ok else "Failed to set gripper width.")
        return False, f"adjust_gripper: unknown action {action!r} (expected 'open', 'close', or 'custom')."

    async def _handle_grasp_simple_object(self, args):
        if not self.current_robot_state:
            return False, "Failed: Robot state is not currently available."
        x = args.get('x'); y = args.get('y'); z = args.get('z')
        if x is None or y is None or z is None:
            return False, "grasp_simple_object requires x, y and z coordinates (from inspect_scene)."
        width = (args or {}).get('width')
        depth = (args or {}).get('depth')
        ok, detail = await self._grasp_simple(float(x), float(y), float(z),
                                              width=float(width) if width is not None else None,
                                              depth=float(depth) if depth is not None else None)
        return ok, (f"Grasped object at ({float(x):.3f}, {float(y):.3f}, {float(z):.3f}).{detail}" if ok else detail)

    async def _handle_grasp_complex_object(self, args):
        object_label = (args or {}).get('object_label')
        if not object_label:
            return False, "grasp_complex_object requires object_label."
        ok, detail = await self._grasp_anygrasp(object_label)
        return ok, (f"Grasped '{object_label}'.{detail}" if ok else detail)

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

        # Snapshot all inputs so they can't change mid-pipeline
        cv_rgb = cv2.cvtColor(self.latest_rgb_image, cv2.COLOR_BGR2RGB)
        captured_depth = self.latest_depth_image.copy()
        orig_h, orig_w = captured_depth.shape

        # Precompute 3D points map so each item's mask can be lifted into base_link for a width estimate
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

            Return ONLY a valid JSON list. Each list element MUST be an object with EXACTLY these three keys:
              "box_2d":   a list [ymin, xmin, ymax, xmax] of integers normalized to 0-1000
              "label":    a short descriptive string naming the item
              "depth_cm": a number -- how THICK the item is front-to-back through its middle, in
                          centimeters. Estimate this from what you know about the real object
                          (a soda can is about 6.6 cm through, a marker about 1.5 cm, a book
                          about 3 cm). This is a physical size estimate, NOT a distance from
                          the camera.

            Do NOT use any other key names (do not use "box", "item name", or "part name").
            Do NOT nest objects. Each item is one flat object with the three keys above.

            Example:
            [
              {"box_2d": [100, 200, 300, 400], "label": "soda can", "depth_cm": 6.6},
              {"box_2d": [500, 600, 700, 800], "label": "marker", "depth_cm": 1.5}
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
                A region is not a physical object, so give it "depth_cm": 0.

                Example with target_location = "third chess square forward from queen":
                [
                  {{"box_2d": [100, 200, 300, 400], "label": "queen", "depth_cm": 3.0}},
                  {{"box_2d": [120, 700, 200, 780], "label": "rook", "depth_cm": 2.5}},
                  {{"box_2d": [410, 380, 500, 470], "label": "TARGET: third square forward from queen", "depth_cm": 0}}
                ]
                """)

        try:
            raw = await perception.query_gemini_json(self.client, self.flash_model_id, prompt,
                                                     img_resized, logger=self.get_logger())
        except Exception as e:
            self.get_logger().error(f"Error querying Gemini: {e}")
            return False, f"Inspection failed: {e}", None

        items = list(perception.iter_scene_items(raw))
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
        font = perception.load_annotation_font(size=18)
        colors = ["red", "green", "blue", "yellow", "orange", "pink", "purple", "cyan", "magenta"]

        scene_report = "Scene Inspection Report:\n"

        for i, (box_2d, label, depth_cm) in enumerate(items):
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

            # Draw the box FIRST — independent of whether depth succeeds, so items without valid
            # depth still appear on the annotated image.
            r_ymin = ymin_norm * new_height / 1000
            r_xmin = xmin_norm * new_width / 1000
            r_ymax = ymax_norm * new_height / 1000
            r_xmax = xmax_norm * new_width / 1000
            draw.rectangle(((r_xmin, r_ymin), (r_xmax, r_ymax)), outline=color, width=box_width)
            draw.text((r_xmin + 5, r_ymin + 5), draw_label, fill=color, font=font)

            mask = await perception.run_sam2(self.sam_predictor, cv_rgb, (xmin, ymin, xmax, ymax))
            if mask is None:
                scene_report += f"{report_prefix}: Pose unknown (SAM2 not available).\n"
                continue

            valid_depths = captured_depth[mask][captured_depth[mask] > 0]
            if len(valid_depths) == 0:
                scene_report += f"{report_prefix}: Pose unknown (no valid depth in mask).\n"
                continue

            # Median over the masked pixels: a robust point on the object's camera-facing
            # surface. The grasp standoff and forward insertion are both measured from this
            # point, so it is the reference the whole approach geometry hangs off.
            depth_m = float(np.median(valid_depths)) / 1000.0
            px, py = perception.mask_centroid(mask, (xmin, ymin, xmax, ymax))
            cx, cy, cz = perception.get_3d_point_from_pixel(px, py, depth_m, self.latest_camera_info)
            bx, by, bz = perception.transform_point(cx, cy, cz, transform)

            size_str = ""
            if not is_target:
                width_m = perception.estimate_object_width(mask, points_3d, transform)
                if width_m is not None:
                    size_str += f" Width: {width_m * 100:.1f} cm."
                # depth_cm is Gemini's semantic estimate, not a measurement, so sanity-check it
                # before it reaches the model's reasoning. Wildly wrong values are dropped rather
                # than reported; grasp_simple_object falls back to width when depth is missing.
                if depth_cm is not None and 0.2 <= depth_cm <= 50.0:
                    size_str += f" Depth: {depth_cm:.1f} cm."

            scene_report += f"{report_prefix}: Pose(x={bx:.3f}, y={by:.3f}, z={bz:.3f}) relative to base.{size_str}\n"

        self.get_logger().info(f"Annotated {len(items)} bounding boxes on inspection image.")
        return True, scene_report, annotated_img

    def _gripper_closed_warning(self):
        """Returns a warning string if the gripper closed almost all the way after a grasp attempt
        (common if robot missed the object). This can't distinguish "missed" from "grasped something very thin/soft" —
        the system prompt tells Gemini to cross-check the camera image when it sees this warning.
        """
        if self.current_robot_state is None:
            return ""
        if self.current_robot_state.gripper_position >= grasp.GRIPPER_FULLY_CLOSED_THRESHOLD:
            return (" WARNING: the gripper closed almost all the way. This can happen normally when "
                    "grasping a thin or soft item, but may also mean the object was missed. Check the "
                    "camera image to confirm the grasp before continuing.")
        return ""

    # ---- Grasp: simple top-down pick at a known point ----

    async def _grasp_simple(self, x, y, z, width=None, depth=None):
        """Pre-shape, approach from a standoff, drive straight in, close.

        (x, y, z) is the object's surface point from inspect_scene (median depth over the visible
        mask), and the approach axis is the gripper's own +Z at whatever orientation the wrist
        currently holds — so this works for a top-down pick or an angled one without caring which
        it is. Both the standoff and the insertion are measured from that reported point.

        The sequence exists to keep the fingers away from the object until they are lined up
        with it: the pre-grasp sits GRASP_STANDOFF_M back along the approach axis, and only then
        do the fingertips travel in along a guaranteed straight line. Going directly to the
        object instead would let the joint-space path arc a finger through it on the way.

        No fallbacks — a failed step logs and returns a diagnostic string.
        """
        if not await self._wait_for_frames(need_state=True):
            self.get_logger().error("grasp_simple_object: robot state not available.")
            return False, "Robot state not available."
        # Orientation is captured once and held for the whole grasp: it defines the approach
        # axis, and nothing below changes it. Pre-shaping the gripper does not move the wrist.
        s = self.current_robot_state

        if width is not None:
            pct = grasp.width_to_gripper_percent(width + 0.02)
            self.get_logger().info(f"Pre-shaping gripper to {pct:.1f}% for {width * 100:.1f} cm object...")
            # No sleep needed: the gripper action now returns once the fingers have settled, and
            # the controller reads the gripper straight from the arm when it compensates the
            # fingertip offset, so there is no topic-staleness race to wait out.
            await self.controller.set_gripper(pct)

        approach = grasp.shift_along_tool_z(x, y, z, s.theta_x, s.theta_y, s.theta_z,
                                            -grasp.GRASP_STANDOFF_M)
        self.get_logger().info(
            f"Moving to pre-grasp {grasp.GRASP_STANDOFF_M * 100:.0f} cm off the surface at "
            f"({approach[0]:.3f}, {approach[1]:.3f}, {approach[2]:.3f})...")
        ok = await self.controller.move_to_pose(approach[0], approach[1], approach[2],
                                                s.theta_x, s.theta_y, s.theta_z)
        if not ok:
            self.get_logger().error("grasp_simple_object: move to pre-grasp pose failed.")
            return False, "Move to pre-grasp pose failed."

        insertion = grasp.insertion_depth_m(depth, width)
        travel = grasp.GRASP_STANDOFF_M + insertion
        self.get_logger().info(
            f"Approaching {travel * 100:.1f} cm along the gripper axis "
            f"({grasp.GRASP_STANDOFF_M * 100:.0f} cm standoff + {insertion * 100:.1f} cm into the object)...")
        ok = await self.controller.move_linear(travel)
        if not ok:
            self.get_logger().error("grasp_simple_object: straight-line approach failed.")
            return False, "Straight-line approach to the object failed."

        self.get_logger().info("Closing gripper...")
        await self.controller.grasp_object()
        return True, self._gripper_closed_warning()

    # ---- Grasp: 6D AnyGrasp pick (complex objects, direct move, no fallbacks) ----

    async def _grasp_anygrasp(self, object_label):
        """Barebones AnyGrasp pick: segment the object, take the single highest-confidence 6D grasp,
        move directly to it, and close. No pre-grasp standoff, no candidate ranking, no fallbacks —
        every failure logs an error and returns a failure detail so it is easy to diagnose.
        """
        self.get_logger().info(f"AnyGrasp grasp for: {object_label}")

        if not await self._wait_for_frames(need_camera_info=True, need_state=True):
            self.get_logger().error("grasp_complex_object: sensor data not available.")
            return False, "Sensor data not available."
        if self.sam_predictor is None:
            self.get_logger().error("grasp_complex_object: SAM2 not available.")
            return False, "SAM2 not available; cannot grasp."
        if self.anygrasp is None:
            self.get_logger().error("grasp_complex_object: AnyGrasp not available.")
            return False, "AnyGrasp not available; cannot grasp."

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
            raw = await perception.query_gemini_json(self.client, self.flash_model_id, prompt,
                                                     pil_img, logger=self.get_logger())
        except Exception as e:
            self.get_logger().error(f"grasp_complex_object: Gemini box query failed: {e}")
            return False, f"Gemini box query failed: {e}"

        pairs = list(perception.iter_scene_items(raw))
        if not pairs:
            self.get_logger().error(f"grasp_complex_object: Gemini returned no bounding box. Raw: {raw}")
            return False, "Gemini did not return a valid bounding box."
        box_2d, label, _ = pairs[0]  # AnyGrasp derives its own depth, so the estimate is unused here
        ymin_norm, xmin_norm, ymax_norm, xmax_norm = box_2d

        h, w = self.latest_rgb_image.shape[:2]
        ymin = int(ymin_norm * h / 1000); xmin = int(xmin_norm * w / 1000)
        ymax = int(ymax_norm * h / 1000); xmax = int(xmax_norm * w / 1000)

        self.get_logger().info(f"Refining '{label}' segmentation with SAM2 box-prompt...")
        binary_mask = await perception.run_sam2(self.sam_predictor, cv_rgb, (xmin, ymin, xmax, ymax))
        if binary_mask is None:
            self.get_logger().error("grasp_complex_object: SAM2 returned no mask.")
            return False, "SAM2 segmentation failed."

        ag_points, ag_colors, lims = grasp.build_anygrasp_clouds(
            self.latest_depth_image, self.latest_camera_info, cv_rgb, binary_mask)
        if ag_points is None:
            self.get_logger().error("grasp_complex_object: target point cloud too small.")
            return False, "Target point cloud is too small."

        self.get_logger().info("Querying AnyGrasp for 6D grasp generation...")
        torch.cuda.empty_cache()
        gg, _ = await asyncio.to_thread(
            self.anygrasp.get_grasp,
            ag_points, ag_colors,
            lims=lims, apply_object_mask=True, dense_grasp=True, collision_detection=True,
        )
        torch.cuda.empty_cache()
        if gg is None or len(gg) == 0:
            self.get_logger().error("grasp_complex_object: AnyGrasp detected no grasps.")
            return False, "No grasps detected."

        # Barebones selection: highest-confidence candidate, nothing else.
        best = gg.nms().sort_by_score()[0]

        try:
            t_base_cam = self.tf_buffer.lookup_transform(
                'base_link', self.latest_camera_info.header.frame_id,
                rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=2.0),
            )
        except Exception as e:
            self.get_logger().error(f"grasp_complex_object: TF lookup failed: {e}")
            return False, "TF lookup failed."
        T_base_cam = perception.transform_to_matrix(t_base_cam)

        xyz, euler = grasp.anygrasp_grasp_to_base_pose(best, T_base_cam)

        self.get_logger().info(
            f"Best grasp (score={best.score:.3f}, width={best.width:.3f}m): moving directly to "
            f"({xyz[0]:.3f}, {xyz[1]:.3f}, {xyz[2]:.3f})...")
        ok = await self.controller.move_to_pose(xyz[0], xyz[1], xyz[2], euler[0], euler[1], euler[2])
        if not ok:
            self.get_logger().error("grasp_complex_object: move to grasp pose failed.")
            return False, "Move to grasp pose failed."

        self.get_logger().info("Closing gripper...")
        await self.controller.grasp_object()
        return True, self._gripper_closed_warning()

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
                self.p.open, format=pyaudio.paInt16, channels=1, rate=SPEAKER_RATE, output=True,
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
        """Streams camera frames to Gemini at ~1 fps. This is the model's only live sense — no
        numeric robot state is ever sent."""
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

                    if sc.interrupted:
                        # New turn started — drop queued speech so the robot stops talking over it.
                        while not self.speaker_queue.empty():
                            self.speaker_queue.get_nowait()

                    if sc.output_transcription:
                        self.transcription_buffer += sc.output_transcription.text
                    if sc.model_turn:
                        for part in sc.model_turn.parts:
                            if part.inline_data:
                                await self.speaker_queue.put(part.inline_data.data)
                    if sc.turn_complete:
                        if self.transcription_buffer:
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
        tasks = []
        try:
            async with self.client.aio.live.connect(model=self.live_model_id, config=config) as session:
                self.get_logger().info("Connected to Gemini Live!")
                tasks = [
                    asyncio.create_task(self.video_stream_task(session)),
                    asyncio.create_task(self.send_text_task(session)),
                    asyncio.create_task(self.play_speaker_task()),
                    asyncio.create_task(self.receive_loop_task(session)),
                ]
                await asyncio.gather(*tasks)
        except Exception as e:
            self.get_logger().error(f"Live session failed: {e}")
        finally:
            # Cancel siblings before terminating PyAudio — gather() leaves the other tasks
            # running when one raises, and terminate() would yank the device out from under
            # a live stream.
            for t in tasks:
                t.cancel()
            if tasks:
                await asyncio.gather(*tasks, return_exceptions=True)
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
