import sys
import os
import argparse
import numpy as np
import open3d as o3d
from PIL import Image as PILImage
import cv2
import yaml
import json
import time
import torch
from dotenv import load_dotenv

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
from scipy.spatial.transform import Rotation as R
import tf2_ros

# Custom interfaces
from ros2_interfaces.action import MoveToPose, MoveToJoints, GripperCommand
from ros2_interfaces.srv import ComputeIK
from ros2_interfaces.msg import RobotState

from google import genai
from google.genai import types

# AnyGrasp imports
workspace_path = os.path.expanduser('~/kinova-gemini')
sys.path.append(os.path.join(workspace_path, 'anygrasp_sdk'))
sys.path.append(os.path.join(workspace_path, 'anygrasp_sdk', 'grasp_detection'))
try:
    from gsnet import AnyGrasp
except ImportError as e:
    print(f"Error importing AnyGrasp: {e}")
    sys.exit(1)

# SAM2 imports
sam2_repo_path = "/home/mcrr-lab/raf-live/SAM2_streaming"
sys.path.append(sam2_repo_path)
try:
    from sam2.build_sam import build_sam2
    from sam2.sam2_image_predictor import SAM2ImagePredictor
    SAM2_AVAILABLE = True
except ImportError as e:
    print(f"Warning: Could not import SAM2 from {sam2_repo_path}: {e}")
    SAM2_AVAILABLE = False

class AnyGraspKinovaFinal(Node):
    def __init__(self, args):
        super().__init__('ag_kinova_final_node')
        self.args = args
        self.bridge = CvBridge()
        load_dotenv(os.path.join(workspace_path, '.env'))

        with open(os.path.join(workspace_path, 'config.yaml'), 'r') as f:
            self.robot_config = yaml.safe_load(f)

        self.client = genai.Client(api_key=os.getenv('gemini_api_key'))
        # Use Flash for fast bounding box detection
        self.model_id = self.robot_config['model']['flash']

        # ROS 2 Action & Service Clients
        self.ik_client = self.create_client(ComputeIK, 'compute_ik')
        self.joints_client = ActionClient(self, MoveToJoints, 'move_to_joints')
        self.pose_client = ActionClient(self, MoveToPose, 'move_to_pose')
        self.gripper_client = ActionClient(self, GripperCommand, 'gripper_command')
        self.grasp_client = ActionClient(self, GripperCommand, 'grasp_object')

        # AnyGrasp initialization
        self.anygrasp = AnyGrasp(args)
        self.anygrasp.load_net()

        # SAM2 initialization
        if SAM2_AVAILABLE:
            self.get_logger().info("Initializing SAM2 Predictor...")
            sam2_checkpoint = os.path.join(sam2_repo_path, "checkpoints/sam2/sam2_hiera_large.pt")
            sam2_model_cfg = "sam2/sam2_hiera_l.yaml" 
            self.sam_predictor = SAM2ImagePredictor(build_sam2(sam2_model_cfg, sam2_checkpoint, device="cuda"))
        else:
            self.sam_predictor = None

        # TF2 Setup
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # State Variables
        self.latest_rgb_image = None
        self.latest_depth_image = None
        self.latest_camera_info = None
        self.camera_frame_id = None
        self.latest_robot_state = None

        # Subscriptions
        self.create_subscription(Image, '/camera/realsense/color/image_raw', self.rgb_callback, 10)
        self.create_subscription(Image, '/camera/realsense/aligned_depth_to_color/image_raw', self.depth_callback, 10)
        self.create_subscription(CameraInfo, '/camera/realsense/aligned_depth_to_color/camera_info', self.camera_info_callback, 10)
        self.create_subscription(RobotState, '/robot_state', self.robot_state_callback, 10)

        # Publishers
        self.marker_pub = self.create_publisher(Marker, 'grasp_marker', 10)

        self.get_logger().info('Final AnyGrasp Pipeline Node Initialized.')

    def robot_state_callback(self, msg):
        self.latest_robot_state = msg

    def rgb_callback(self, msg):
        self.latest_rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.camera_frame_id = msg.header.frame_id

    def depth_callback(self, msg):
        self.latest_depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def camera_info_callback(self, msg):
        self.latest_camera_info = msg

    # --- Helper Methods for ROS Actions ---
    def send_gripper_cmd(self, position):
        self.get_logger().info(f"Moving gripper to position: {position:.1f}%")
        goal = GripperCommand.Goal()
        goal.position = float(position)
        if not self.gripper_client.wait_for_server(timeout_sec=2.0):
            return False
        future = self.gripper_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()
        if not goal_handle.accepted:
            return False
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        return result_future.result().result.success

    def send_grasp_cmd(self):
        self.get_logger().info("Executing specialized grasp_object action (pinch until contact)...")
        goal = GripperCommand.Goal()
        goal.position = 99.0 # Logic in controller.cpp uses hardware contact detection
        if not self.grasp_client.wait_for_server(timeout_sec=2.0):
            return False
        future = self.grasp_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()
        if not goal_handle.accepted:
            return False
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        return result_future.result().result.success

    def send_joints_cmd(self, joint_angles):
        self.get_logger().info("Sending Joint Move Goal...")
        goal = MoveToJoints.Goal()
        goal.joint_angles = [float(a) for a in joint_angles]
        if not self.joints_client.wait_for_server(timeout_sec=2.0):
            return False
        future = self.joints_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()
        if not goal_handle.accepted:
            return False
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        return result_future.result().result.success

    def send_pose_cmd(self, x, y, z, tx, ty, tz):
        self.get_logger().info(f"Sending Cartesian Move Goal: X={x:.3f}, Y={y:.3f}, Z={z:.3f}")
        goal = MoveToPose.Goal()
        goal.x, goal.y, goal.z = float(x), float(y), float(z)
        goal.theta_x, goal.theta_y, goal.theta_z = float(tx), float(ty), float(tz)
        goal.speed_scaling = 0.05 # slow approach for safety
        if not self.pose_client.wait_for_server(timeout_sec=2.0):
            return False
        future = self.pose_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()
        if not goal_handle.accepted:
            return False
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        return result_future.result().result.success

    def publish_pose_marker(self, position, rotation_matrix):
        marker = Marker()
        marker.header.frame_id = 'base_link'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'grasp_pose'
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.005
        marker.color.a = 1.0
        marker.lifetime.nanosec = 0

        p0 = Point(x=float(position[0]), y=float(position[1]), z=float(position[2]))
        axis_length = 0.1

        # X axis (Red)
        x_end = position + rotation_matrix[:, 0] * axis_length
        p_x = Point(x=float(x_end[0]), y=float(x_end[1]), z=float(x_end[2]))
        marker.points.append(p0); marker.points.append(p_x)
        marker.colors.append(ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)); marker.colors.append(ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0))

        # Y axis (Green)
        y_end = position + rotation_matrix[:, 1] * axis_length
        p_y = Point(x=float(y_end[0]), y=float(y_end[1]), z=float(y_end[2]))
        marker.points.append(p0); marker.points.append(p_y)
        marker.colors.append(ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)); marker.colors.append(ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0))

        # Z axis (Blue)
        z_end = position + rotation_matrix[:, 2] * axis_length
        p_z = Point(x=float(z_end[0]), y=float(z_end[1]), z=float(z_end[2]))
        marker.points.append(p0); marker.points.append(p_z)
        marker.colors.append(ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0)); marker.colors.append(ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0))

        self.marker_pub.publish(marker)

    # --- Main Pipeline ---
    def run_pipeline(self):
        # 1. Wait for Sensor Data
        while rclpy.ok() and (self.latest_rgb_image is None or self.latest_depth_image is None or self.latest_camera_info is None or self.latest_robot_state is None):
            self.get_logger().info("Waiting for sensor data (RGB-D and Robot State)...", throttle_duration_sec=2.0)
            rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().info("Data received! Starting AnyGrasp Kinova Pipeline...")

        # 2. Get Object Part Bounding Box from Gemini
        cv_image_rgb = cv2.cvtColor(self.latest_rgb_image, cv2.COLOR_BGR2RGB)
        pil_img = PILImage.fromarray(cv_image_rgb)
        
        prompt = "Identify the most easily graspable part of the object (e.g. handle, rim, or the whole body if it is small). Return ONLY a JSON with a bounding box in [ymin, xmin, ymax, xmax] format (normalized 0-1000): {\"box_2d\": [ymin, xmin, ymax, xmax], \"label\": \"part name\"}"
        
        self.get_logger().info(f"Querying Gemini for graspable part bounding box...")
        response = self.client.models.generate_content(
            model=self.model_id,
            contents=[prompt, pil_img],
            config=types.GenerateContentConfig(response_mime_type="application/json", temperature=0.0)
        )
        
        try:
            res = json.loads(response.text)
            ymin_norm, xmin_norm, ymax_norm, xmax_norm = res['box_2d']
            label = res['label']
            
            h, w = self.latest_rgb_image.shape[:2]
            ymin, xmin, ymax, xmax = int(ymin_norm * h / 1000), int(xmin_norm * w / 1000), int(ymax_norm * h / 1000), int(xmax_norm * w / 1000)
            self.get_logger().info(f"Gemini identified '{label}' at box [{ymin}, {xmin}, {ymax}, {xmax}]")
        except Exception as e:
            self.get_logger().error(f"Gemini box parsing failed: {e}")
            return

        # 3. Segment with SAM2 using the box as a prompt
        if self.sam_predictor:
            self.get_logger().info(f"Refining '{label}' segmentation with SAM2 box-prompt...")
            with torch.inference_mode(), torch.autocast(device_type="cuda", dtype=torch.bfloat16):
                self.sam_predictor.set_image(cv_image_rgb)
                input_box = np.array([xmin, ymin, xmax, ymax]) # [x1, y1, x2, y2]
                masks, scores, _ = self.sam_predictor.predict(
                    box=input_box,
                    multimask_output=False
                )
            
            if torch.is_tensor(masks):
                binary_mask = masks[0].cpu().numpy() > 0
            else:
                binary_mask = masks[0] > 0

            # --- Intersection Logic ---
            # Ensure the segment is strictly bounded by the Gemini box to avoid background noise
            box_mask = np.zeros((h, w), dtype=bool)
            box_mask[max(0, ymin):min(h, ymax), max(0, xmin):min(w, xmax)] = True
            binary_mask = binary_mask & box_mask

            torch.set_default_dtype(torch.float32)
            torch.cuda.empty_cache()
        else:
            self.get_logger().error("SAM2 not available.")
            return

        # 4. Filter Point Cloud with the Refined Mask
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

        # Points specifically belonging to the target segment
        target_points = points[binary_mask].reshape(-1, 3)
        valid_target_idx = (target_points[:, 2] > 0.1) & (target_points[:, 2] < 1.0)
        target_points = target_points[valid_target_idx]

        if target_points.shape[0] < 10:
            self.get_logger().error("Target point cloud is too small.")
            return

        # 5. AnyGrasp Inference with tight limits and MASKING
        obj_min, obj_max = target_points.min(axis=0), target_points.max(axis=0)
        margin = 0.005 # 5mm margin
        lims = [obj_min[0]-margin, obj_max[0]+margin, obj_min[1]-margin, obj_max[1]+margin, obj_min[2]-margin, obj_max[2]+margin]

        # Scene for collision (tight cropping to prevent OOM)
        scene_mask = (points[:, :, 2] > 0.2) & (points[:, :, 2] < 0.75)
        full_points = points[scene_mask].reshape(-1, 3)
        full_colors = colors[scene_mask].reshape(-1, 3)
        
        # Create a boolean mask for full_points that says "this point is part of the segment"
        full_obj_mask = binary_mask[scene_mask]
        
        # --- Targeted Point Selection Logic ---
        # AnyGrasp doesn't take a mask, it plans on every point in 'points' that is within 'lims'.
        # To force it to ONLY plan on the segment, we must remove background points that are INSIDE the box.
        # However, we must keep points OUTSIDE the box for collision detection.
        
        # Identify points inside the 3D bounding box
        in_lims = (full_points[:, 0] >= lims[0]) & (full_points[:, 0] <= lims[1]) & \
                  (full_points[:, 1] >= lims[2]) & (full_points[:, 1] <= lims[3]) & \
                  (full_points[:, 2] >= lims[4]) & (full_points[:, 2] <= lims[5])
        
        # Points to keep: (Points inside the box AND in segment) OR (Points outside the box)
        keep_mask = (in_lims & full_obj_mask) | (~in_lims)
        
        points_for_anygrasp = full_points[keep_mask]
        colors_for_anygrasp = full_colors[keep_mask]
        
        torch.cuda.empty_cache()
        self.get_logger().info(f"Running AnyGrasp on {points_for_anygrasp.shape[0]} points (targeting segment within box)...")
        
        # Call AnyGrasp with the filtered cloud
        gg, _ = self.anygrasp.get_grasp(points_for_anygrasp, colors_for_anygrasp, lims=lims, 
                                        apply_object_mask=True, 
                                        dense_grasp=True, collision_detection=True)
        torch.cuda.empty_cache()

        if len(gg) == 0:
            self.get_logger().error("No grasps detected."); return

        gg = gg.nms().sort_by_score()
        best_grasp = gg[0]
        self.get_logger().info(f"Best Grasp: Width={best_grasp.width:.3f}m, Depth={best_grasp.depth:.3f}m")

        # --- Visualization ---
        self.get_logger().info("Displaying 3D Visualization. Close the window to proceed.")
        cloud = o3d.geometry.PointCloud()
        cloud.points = o3d.utility.Vector3dVector(points_for_anygrasp)
        debug_colors = colors_for_anygrasp.copy()
        
        # Re-calculate mask for the filtered points to highlight them red
        final_in_lims = (points_for_anygrasp[:, 0] >= lims[0]) & (points_for_anygrasp[:, 0] <= lims[1]) & \
                        (points_for_anygrasp[:, 1] >= lims[2]) & (points_for_anygrasp[:, 1] <= lims[3]) & \
                        (points_for_anygrasp[:, 2] >= lims[4]) & (points_for_anygrasp[:, 2] <= lims[5])
        debug_colors[final_in_lims] = [1, 0, 0] # Red for the actual target segment
        
        cloud.colors = o3d.utility.Vector3dVector(debug_colors)
        grippers = gg[:3].to_open3d_geometry_list()
        if len(grippers) > 0: grippers[0].paint_uniform_color([0, 1, 0])
        o3d.visualization.draw_geometries([cloud, *grippers])

        # 6. Open Gripper to Width
        desired_width = best_grasp.width + 0.02
        target_gripper_pos = max(0.0, min(100.0, (0.14 - desired_width) / 0.14 * 100.0))
        self.send_gripper_cmd(target_gripper_pos)
        for _ in range(10): rclpy.spin_once(self, timeout_sec=0.1)

        # 7. Coordinate Transformation to Base Link
        def transform_to_matrix(t_msg):
            mat = np.eye(4)
            q = [t_msg.transform.rotation.x, t_msg.transform.rotation.y, t_msg.transform.rotation.z, t_msg.transform.rotation.w]
            mat[:3, :3] = R.from_quat(q).as_matrix()
            mat[0, 3] = t_msg.transform.translation.x; mat[1, 3] = t_msg.transform.translation.y; mat[2, 3] = t_msg.transform.translation.z
            return mat

        t_base_cam = self.tf_buffer.lookup_transform('base_link', self.camera_frame_id, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=2.0))
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

        r_euler = R.from_matrix(r_base_grasp).as_euler('xyz', degrees=True)
        self.publish_pose_marker(grasp_position, r_base_grasp)
        
        # 8. Apply Dynamic Offset
        gripper_pos = self.latest_robot_state.gripper_position
        dynamic_offset = 0.0243 * (gripper_pos / 100.0)
        self.get_logger().info(f"Fingertip Offset: {dynamic_offset*1000:.2f}mm")

        # 9. Calculate Pre-Grasp and Grasp Palm Poses
        approach_dist = best_grasp.depth 
        approach_vector = r_base_grasp[:, 2] 
        insertion_offset = 0.05 
        
        def solve_ik_with_retry(pos, rot_matrix):
            euler = R.from_matrix(rot_matrix).as_euler('xyz', degrees=True)
            req = ComputeIK.Request(); req.x, req.y, req.z = float(pos[0]), float(pos[1]), float(pos[2])
            req.theta_x, req.theta_y, req.theta_z = float(euler[0]), float(euler[1]), float(euler[2])
            future = self.ik_client.call_async(req); rclpy.spin_until_future_complete(self, future); res = future.result()
            if res and res.success: return res.joint_angles, euler
            self.get_logger().warn("Primary IK failed. Retrying flipped...")
            rot_flip = rot_matrix @ R.from_euler('z', 180, degrees=True).as_matrix(); euler_flip = R.from_matrix(rot_flip).as_euler('xyz', degrees=True)
            req.theta_x, req.theta_y, req.theta_z = float(euler_flip[0]), float(euler_flip[1]), float(euler_flip[2])
            future = self.ik_client.call_async(req); rclpy.spin_until_future_complete(self, future); res = future.result()
            if res and res.success: return res.joint_angles, euler_flip
            return None, None

        pre_grasp_palm_pos = grasp_position - (approach_dist + dynamic_offset + 0.02) * approach_vector
        self.get_logger().info("Step 1/3: Calculating and moving to Pre-Grasp...")
        joint_angles, final_euler = solve_ik_with_retry(pre_grasp_palm_pos, r_base_grasp)
        if joint_angles: self.send_joints_cmd(joint_angles)
        else:
            self.get_logger().error("IK failed for both orientations."); return

        # 10. Step 2: Slotted Approach (Cartesian Space)
        final_rot_matrix = R.from_euler('xyz', final_euler, degrees=True).as_matrix()
        final_approach_vector = final_rot_matrix[:, 2]
        grasp_palm_pos = pre_grasp_palm_pos + (insertion_offset * final_approach_vector)
        self.get_logger().info(f"Step 2/3: Sliding forward by {approach_dist*1000:.1f}mm...")
        self.send_pose_cmd(grasp_palm_pos[0], grasp_palm_pos[1], grasp_palm_pos[2], final_euler[0], final_euler[1], final_euler[2])

        # 11. Step 3: Pinch Object
        self.get_logger().info("Step 3/3: Pinching object until contact...")
        self.send_grasp_cmd()
        self.get_logger().info("AnyGrasp Kinova Execution Successful!")

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--checkpoint_path', default='/home/mcrr-lab/kinova-gemini/anygrasp_sdk/grasp_detection/log/checkpoint_detection.tar', help='AnyGrasp checkpoint')
    parser.add_argument('--max_gripper_width', type=float, default=0.14)
    parser.add_argument('--gripper_height', type=float, default=0.07)
    parser.add_argument('--top_down_grasp', action='store_true')
    parser.add_argument('--debug', action='store_true')
    args = parser.parse_args()

    rclpy.init()
    node = AnyGraspKinovaFinal(args)
    try:
        node.run_pipeline()
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()
