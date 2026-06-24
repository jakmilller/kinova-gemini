#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import asyncio
import yaml
import os
import numpy as np
from scipy.spatial.transform import Rotation as R

# Import custom action/visualization interfaces
from ros2_interfaces.action import MoveToPose, MoveToJoints, GripperCommand
from ros2_interfaces.msg import RobotState
from std_srvs.srv import SetBool
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

# The TCP (gripper fingertip point) is not the same point as the URDF's end_effector_link,
# which is just the bare wrist flange cuRobo solves IK for - and the TCP moves further from
# end_effector_link as the gripper closes (fingers swing on a 4-bar linkage). Per the
# Robotiq 2F-140 official spec, fingertip distance along the end_effector_link local Z axis
# is 209.8mm fully open and 233.3mm fully closed (X/Y offset is ~0 by bilateral symmetry,
# confirmed via FK). An earlier pure-URDF-geometry estimate (pad link origins, not the true
# fingertip) undershot this by ~2.6cm and caused the gripper to overshoot targets in testing.
TCP_OFFSET_OPEN_LOCAL_Z = 0.2098
GRIPPER_CLOSE_OFFSET_GROWTH = 0.2333 - 0.2098  # meters of additional local-Z travel, 0% -> 100% closed


def _tcp_offset_local(gripper_position):
    """Local-frame (end_effector_link) offset to the TCP, given gripper_position (0-100, 0=open)."""
    z = TCP_OFFSET_OPEN_LOCAL_Z + GRIPPER_CLOSE_OFFSET_GROWTH * (gripper_position / 100.0)
    return np.array([0.0, 0.0, z])

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
        self._action_gripper_client = ActionClient(self, GripperCommand, 'gripper_command')
        self._action_grasp_object = ActionClient(self, GripperCommand, 'grasp_object')

        # --- Service Client (for Admittance) ---
        self.admittance_client = self.create_client(SetBool, 'toggle_admittance')

        # --- Subscription (to monitor state) ---
        self.state_sub = self.create_subscription(
            RobotState, 'robot_state', self._state_callback, 10)
        self.current_state = None

        # --- Active Goals for Cancellation ---
        self._active_goals = []

        # --- Publisher for RViz visualization ---
        self.marker_pub = self.create_publisher(MarkerArray, '/curobo_obstacles', 10)
        self.sphere_marker_pub = self.create_publisher(MarkerArray, '/curobo_robot_spheres', 10)

        # --- Initialize cuRobo ---
        self.planner = None
        self.table = None
        self.dynamic_obstacles = {}
        self._held_object = None
        
        try:
            import torch
            from curobo.geom.types import Cuboid, WorldConfig
            from curobo.wrap.reacher.motion_gen import MotionGen, MotionGenConfig
            from curobo.types.base import TensorDeviceType
            from curobo.types.state import JointState
            from curobo.geom.sdf.world import CollisionCheckerType
            
            self.tensor_args = TensorDeviceType(device=torch.device("cuda:0"), dtype=torch.float32)
            
            # Phase 3.1: Static table representation (centered directly under the robot base)
            self.table = Cuboid(
                name="table",
                pose=[0.0, 0.0, -0.05, 1.0, 0.0, 0.0, 0.0],
                dims=[1.2, 1.2, 0.1]
            )
            self.world_config = WorldConfig(cuboid=[self.table])
            
            # Load YAML configuration for cuRobo kinematics and link limits
            self.robot_cfg_path = os.path.join(os.path.expanduser('~'), 'kinova-gemini', 'config', 'kinova_gen3.yml')
            
            self.get_logger().info("Initializing cuRobo MotionGen...")
            # Cache must cover the static table plus every object the vision pipeline may
            # register as a dynamic obstacle; cuRobo allocates this GPU buffer once and never
            # grows it, so undersizing causes "number of OBB is larger than collision cache".
            COLLISION_CACHE_OBB_SIZE = 30
            # trajopt optimizes over a fixed 32-step horizon; maximum_trajectory_dt sets the
            # per-step time budget (default 0.15s -> ~4.8s total horizon), sized for cuRobo's
            # default fast-robot assumptions. config/kinova_gen3.yml's cspace now reflects
            # this robot's much slower real operational speed/acceleration limits (read from
            # the Kinova web app), so a feasible smooth trajectory may genuinely need more
            # total time than the default horizon can represent - widen it here rather than
            # via velocity_scale/acceleration_scale kwargs, which would double-scale the
            # already-absolute max_acceleration values set in the yaml.
            motion_gen_config = MotionGenConfig.load_from_robot_config(
                self.robot_cfg_path,
                self.world_config,
                self.tensor_args,
                collision_checker_type=CollisionCheckerType.MESH,
                # use_cuda_graph=True captures the planning kernels into a static CUDA graph at
                # warmup(), when the world holds only the static table. inspect_scene then
                # mutates the world via update_world() (per detected object), so the captured
                # graph no longer matches the live world and the next plan_single() fails with
                # "CUDA error: operation failed due to a previous error during capture". We
                # register/clear obstacles every cycle, so a static graph is fundamentally
                # incompatible with this dynamic-obstacle workflow. Eager mode keeps
                # update_world() + plan_single() reliable; the per-plan latency cost (tens of ms
                # on GPU) is negligible for this interactive task.
                use_cuda_graph=False,
                collision_cache={"obb": COLLISION_CACHE_OBB_SIZE},
                maximum_trajectory_dt=2.5,
            )
            self.planner = MotionGen(motion_gen_config)
            self.planner.warmup()  # Pre-compiles CUDA kernels to eliminate runtime latency spikes
            self.get_logger().info("cuRobo MotionGen Initialized and Warmed Up successfully!")
            
            # Publish initial table visualization to RViz
            self.publish_rviz_markers()
        except Exception as e:
            self.get_logger().error(f"Failed to initialize cuRobo: {e}")
            self.planner = None

        self.get_logger().info('Minimal Kinova Action Controller Initialized')

    def _state_callback(self, msg):
        self.current_state = msg
        self.publish_robot_sphere_markers()

    def _current_joints_rad(self):
        """
        Kortex reports all 7 joints in raw [0,360) convention even for the mechanically
        bounded joints (2, 4, 6), whose URDF/cuRobo limits are tight (roughly +/-120 to
        +/-147 deg) - wrap into [-180,180) first or a value like 350 deg (i.e. -10 deg)
        gets rejected by cuRobo as outside the joint limits.
        """
        return [
            float(a - 360.0 if a > 180.0 else a) * np.pi / 180.0
            for a in self.current_state.joint_angles
        ]

    def get_tcp_pose(self):
        """
        Live TCP (gripper finger-pad midpoint) pose, as (x, y, z, theta_x, theta_y, theta_z).
        Primary path: cuRobo FK on current joint angles.
        Fallback (cuRobo unavailable): Kortex-reported EE pose from robot_state + TCP offset.
        Both sources use the same kinematic model so results agree to sub-mm.
        """
        if self.current_state is None:
            raise RuntimeError("Cannot compute TCP pose: robot state not yet received.")
        if self.planner is not None:
            import torch
            q = torch.tensor([self._current_joints_rad()], **self.tensor_args.as_torch_dict())
            state = self.planner.kinematics.get_state(q)
            ee_pos = state.ee_position[0].cpu().numpy()
            ee_quat_wxyz = state.ee_quaternion[0].cpu().numpy()
            r = R.from_quat([ee_quat_wxyz[1], ee_quat_wxyz[2], ee_quat_wxyz[3], ee_quat_wxyz[0]])
        else:
            # cuRobo unavailable — use Kortex-reported EE pose directly
            s = self.current_state
            ee_pos = np.array([s.x, s.y, s.z])
            r = R.from_euler('xyz', [s.theta_x, s.theta_y, s.theta_z], degrees=True)
        tcp_pos = ee_pos + r.apply(_tcp_offset_local(self.current_state.gripper_position))
        theta = r.as_euler('xyz', degrees=True)
        return float(tcp_pos[0]), float(tcp_pos[1]), float(tcp_pos[2]), float(theta[0]), float(theta[1]), float(theta[2])

    def publish_robot_sphere_markers(self):
        """
        Publishes cuRobo's own collision-sphere model of the robot (the actual geometry the
        planner checks for self/world collision) to RViz, since cuRobo has no MoveIt-style
        plan preview. Lets you visually confirm the sphere model tracks the real arm/gripper
        and never overlaps a registered obstacle.
        """
        if self.planner is None or self.current_state is None:
            return
        try:
            import torch
            q = torch.tensor([self._current_joints_rad()], **self.tensor_args.as_torch_dict())
            spheres = self.planner.kinematics.get_robot_as_spheres(q)[0]

            marker_array = MarkerArray()
            for i, sphere in enumerate(spheres):
                marker = Marker()
                marker.header.frame_id = "base_link"
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = "curobo_robot_spheres"
                marker.id = i
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
                marker.pose.position.x = float(sphere.pose[0])
                marker.pose.position.y = float(sphere.pose[1])
                marker.pose.position.z = float(sphere.pose[2])
                marker.pose.orientation.w = 1.0
                diameter = float(sphere.radius) * 2.0
                marker.scale.x = diameter
                marker.scale.y = diameter
                marker.scale.z = diameter
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 1.0
                marker.color.a = 0.5
                marker_array.markers.append(marker)

            self.sphere_marker_pub.publish(marker_array)
        except Exception as e:
            self.get_logger().error(f"Failed to publish robot sphere markers: {e}")

    def publish_rviz_markers(self):
        """
        Publishes the static table and all dynamic obstacles as red/orange wireframe boxes in RViz.
        """
        try:
            marker_array = MarkerArray()
            
            # Helper to create a wireframe box marker using LINE_LIST.
            # quat_wxyz rotates corners before translating to center, used for OBB obstacles.
            def create_wireframe_marker(center, dims, marker_id, is_table=False, quat_wxyz=None):
                marker = Marker()
                marker.header.frame_id = "base_link"
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = "curobo_obstacles"
                marker.id = marker_id
                marker.type = Marker.LINE_LIST
                marker.action = Marker.ADD

                marker.scale.x = 0.005

                if is_table:
                    marker.color.r = 1.0
                    marker.color.g = 0.5
                    marker.color.b = 0.0
                    marker.color.a = 1.0
                else:
                    marker.color.r = 1.0
                    marker.color.g = 0.0
                    marker.color.b = 0.0
                    marker.color.a = 1.0

                x_h, y_h, z_h = dims[0]/2.0, dims[1]/2.0, dims[2]/2.0
                rel_corners = np.array([
                    [-x_h, -y_h, -z_h], [x_h, -y_h, -z_h], [x_h, y_h, -z_h], [-x_h, y_h, -z_h],
                    [-x_h, -y_h,  z_h], [x_h, -y_h,  z_h], [x_h, y_h,  z_h], [-x_h, y_h,  z_h],
                ])

                if quat_wxyz is not None and not np.allclose(quat_wxyz, [1, 0, 0, 0]):
                    w, xi, yi, zi = quat_wxyz
                    rot = R.from_quat([xi, yi, zi, w])
                    rel_corners = rot.apply(rel_corners)

                abs_corners = [
                    Point(x=float(rc[0] + center[0]), y=float(rc[1] + center[1]), z=float(rc[2] + center[2]))
                    for rc in rel_corners
                ]

                for s, e in [(0,1),(1,2),(2,3),(3,0),(4,5),(5,6),(6,7),(7,4),(0,4),(1,5),(2,6),(3,7)]:
                    marker.points.append(abs_corners[s])
                    marker.points.append(abs_corners[e])

                return marker

            marker_id = 0
            # Static Table
            if self.table is not None:
                t_pose = self.table.pose.cpu().numpy().tolist() if hasattr(self.table.pose, 'cpu') else self.table.pose
                t_dims = self.table.dims.cpu().numpy().tolist() if hasattr(self.table.dims, 'cpu') else self.table.dims
                marker_array.markers.append(
                    create_wireframe_marker(t_pose[:3], t_dims, marker_id, is_table=True)
                )
                marker_id += 1

            # Dynamic Obstacles
            for obs_name, cuboid in self.dynamic_obstacles.items():
                c_pose = cuboid.pose.cpu().numpy().tolist() if hasattr(cuboid.pose, 'cpu') else cuboid.pose
                c_dims = cuboid.dims.cpu().numpy().tolist() if hasattr(cuboid.dims, 'cpu') else cuboid.dims
                # c_pose is [x, y, z, qw, qx, qy, qz]
                quat = c_pose[3:7] if len(c_pose) >= 7 else None
                marker_array.markers.append(
                    create_wireframe_marker(c_pose[:3], c_dims, marker_id, is_table=False, quat_wxyz=quat)
                )
                marker_id += 1
                
            self.marker_pub.publish(marker_array)
        except Exception as e:
            self.get_logger().error(f"Failed to publish RViz markers: {e}")

    def update_dynamic_obstacle(self, label: str, center: list, dims: list, quat: list = None):
        """
        Registers or updates a dynamic obstacle (as a Cuboid) in the cuRobo environment.
        center: [x, y, z] in base_link coordinates
        dims: [size_x, size_y, size_z] physical size of the cuboid
        quat: [w, x, y, z] orientation quaternion (wxyz); defaults to identity if None
        """
        if self.planner is None:
            self.get_logger().warn(f"cuRobo planner is not initialized. Cannot register obstacle '{label}'")
            return

        if quat is None:
            quat = [1.0, 0.0, 0.0, 0.0]

        self.get_logger().info(f"Registering dynamic obstacle '{label}' with center {center}, dims {dims}, quat {quat}")

        try:
            from curobo.geom.types import Cuboid
            cuboid = Cuboid(
                name=label,
                pose=[center[0], center[1], center[2], quat[0], quat[1], quat[2], quat[3]],
                dims=dims
            )
            self.dynamic_obstacles[label] = cuboid
            self._rebuild_world()
            self.publish_rviz_markers()
        except Exception as e:
            self.get_logger().error(f"Failed to update dynamic obstacle '{label}' in cuRobo: {e}")

    def clear_dynamic_obstacles(self):
        """Removes all dynamic obstacles and resets the world to table-only."""
        if self.planner is None:
            return
        self.dynamic_obstacles.clear()
        try:
            self._rebuild_world()
            self.publish_rviz_markers()
            self.get_logger().info("Cleared all dynamic obstacles.")
        except Exception as e:
            self.get_logger().error(f"Failed to clear dynamic obstacles: {e}")

    def remove_dynamic_obstacle(self, label: str):
        """Removes a single obstacle by label (no-op if not found) and updates the world."""
        if self.planner is None:
            return
        # Fuzzy match: remove the first registered label that contains the given label as a substring
        match = next((k for k in self.dynamic_obstacles if label.lower() in k.lower() or k.lower() in label.lower()), None)
        if match is None:
            self.get_logger().info(f"remove_dynamic_obstacle: no match for '{label}' — skipping.")
            return
        del self.dynamic_obstacles[match]
        try:
            self._rebuild_world()
            self.publish_rviz_markers()
            self.get_logger().info(f"Removed dynamic obstacle '{match}'.")
        except Exception as e:
            self.get_logger().error(f"Failed to remove obstacle '{match}': {e}")

    def attach_grasped_object(self, label: str, dims: list):
        """Records the held object so subsequent moves know what is being carried.
        Also attempts cuRobo's attach API so the planner checks the carried object's
        collision geometry during transit; degrades gracefully if the API differs."""
        self._held_object = {'label': label, 'dims': dims}
        self.get_logger().info(f"Recorded '{label}' as held object (dims={dims}).")
        if self.planner is None:
            return
        try:
            import torch
            from curobo.geom.types import Cuboid, WorldConfig
            # Re-add the object to the world so cuRobo can transfer it to the robot link.
            cuboid = Cuboid(
                name=label,
                pose=[0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
                dims=dims,
            )
            tmp_cuboids = ([self.table] if self.table else []) + list(self.dynamic_obstacles.values()) + [cuboid]
            self.planner.update_world(WorldConfig(cuboid=tmp_cuboids))
            q = torch.tensor([self._current_joints_rad()], **self.tensor_args.as_torch_dict())
            from curobo.types.state import JointState
            js = JointState.from_position(q)
            self.planner.attach_objects_to_robot(joint_state=js, object_names=[label])
            self.get_logger().info(f"Attached '{label}' to robot in cuRobo world.")
        except Exception as e:
            self.get_logger().warn(f"cuRobo object attach skipped (API may differ): {e}")

    def release_grasped_object(self):
        """Clears the held-object record and detaches from the cuRobo robot link."""
        self._held_object = None
        if self.planner is None:
            return
        try:
            self.planner.detach_object_from_robot()
            self._rebuild_world()
            self.get_logger().info("Released attached object from robot.")
        except Exception as e:
            self.get_logger().warn(f"cuRobo object detach skipped: {e}")

    def _rebuild_world(self):
        """Rebuilds and pushes the cuRobo WorldConfig from the static table + current dynamic obstacles."""
        from curobo.geom.types import WorldConfig
        all_cuboids = []
        if self.table is not None:
            all_cuboids.append(self.table)
        all_cuboids.extend(list(self.dynamic_obstacles.values()))
        self.world_config = WorldConfig(cuboid=all_cuboids)
        self.planner.update_world(self.world_config)
        self.get_logger().info(f"cuRobo world updated: {len(all_cuboids)} obstacle(s).")

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

    async def move_to_pose(self, x, y, z, theta_x=0.0, theta_y=0.0, theta_z=0.0, speed=0.1):
        """Sends a Cartesian pose goal. Intercepted by cuRobo for collision-free trajectory planning."""
        if self.planner is not None and self.current_state is not None:
            self.get_logger().info(f"🔎 Status: Querying cuRobo to plan collision-free path to Cartesian (X={x:.3f}, Y={y:.3f}, Z={z:.3f})...")
            try:
                import torch
                from curobo.types.math import Pose
                from curobo.wrap.reacher.motion_gen import MotionGenPlanConfig
                from curobo.types.state import JointState
                from trajectory_msgs.msg import JointTrajectoryPoint
                
                start_state = JointState.from_position(torch.tensor([self._current_joints_rad()], **self.tensor_args.as_torch_dict()))

                
                # Convert target Euler angles to quaternion (wxyz) for cuRobo goal pose
                q_xyzw = R.from_euler('xyz', [theta_x, theta_y, theta_z], degrees=True).as_quat()
                q_wxyz = [q_xyzw[3], q_xyzw[0], q_xyzw[1], q_xyzw[2]]

                # (x, y, z) is where the gripper's TCP should end up, but cuRobo solves IK
                # for end_effector_link - rotate the current gripper-state TCP offset into
                # world frame using the goal orientation and subtract it to get the
                # equivalent end_effector_link target (see _tcp_offset_local comment above).
                tcp_offset = _tcp_offset_local(self.current_state.gripper_position)
                ee_offset_world = R.from_quat(q_xyzw).apply(tcp_offset)
                ee_target = np.array([x, y, z]) - ee_offset_world

                goal_pose = Pose(
                    position=torch.tensor([ee_target.tolist()], **self.tensor_args.as_torch_dict()),
                    quaternion=torch.tensor([q_wxyz], **self.tensor_args.as_torch_dict())
                )
                
                # Plan trajectory. timeout was previously 1.0s, fine under the old, much
                # looser max_acceleration (25.0 rad/s^2) - now that cspace's acceleration/
                # velocity limits match the robot's real, tighter operational caps, the
                # finetuning optimizer needs more room to converge on a feasible trajectory.
                result = self.planner.plan_single(start_state, goal_pose, MotionGenPlanConfig(timeout=1.0))
                
                if result.success:
                    interpolated_plan = result.get_interpolated_plan()
                    positions = interpolated_plan.position.cpu().numpy().tolist()
                    
                    # interpolated_plan is a JointState, which has no per-step .time field -
                    # the uniform step spacing lives on result.interpolation_dt instead. Without
                    # this, time_from_start was silently never set, so controller.cpp fell back
                    # to a 1-second-per-waypoint floor (a 49-waypoint move took ~49s instead of
                    # ~1s at the real interpolation_dt, typically 0.02-0.05s).
                    dt = float(result.interpolation_dt)

                    # The Kortex high-level waypoint API (ExecuteWaypointTrajectory) only needs
                    # checkpoint angle+duration pairs - the firmware does its own interpolation
                    # between them (see controller.cpp's waypoint builder, which deliberately
                    # sets no per-waypoint velocity/blending). cuRobo's raw interpolation_dt
                    # (~0.02-0.05s) is sized for high-rate setpoint streaming, not this
                    # checkpoint API, so sending every sample produces hundreds of waypoints.
                    # Downsample to ~TARGET_WAYPOINT_SPACING_S between checkpoints.
                    # Keep this at 0.1s (not coarser): the Kortex firmware infers per-segment
                    # acceleration numerically from consecutive (Δθ, Δt) pairs. At 0.3s spacing
                    # the angle deltas are large enough that fast cuRobo trajectories exceed
                    # the firmware's acceleration limit and get rejected, triggering costly retries.
                    # 0.1s gives finer sampling so consecutive segments have similar velocities
                    # and the inferred acceleration stays within firmware limits.
                    TARGET_WAYPOINT_SPACING_S = 0.1
                    step = max(1, round(TARGET_WAYPOINT_SPACING_S / dt))
                    sampled_indices = list(range(0, len(positions), step))
                    if sampled_indices[-1] != len(positions) - 1:
                        sampled_indices.append(len(positions) - 1)  # land exactly on the at-rest goal

                    trajectory_points = []
                    for i in sampled_indices:
                        point = JointTrajectoryPoint()
                        point.positions = [float(p) for p in positions[i]]

                        t = i * dt
                        point.time_from_start.sec = int(t)
                        point.time_from_start.nanosec = int((t - int(t)) * 1e9)
                        trajectory_points.append(point)
                    
                    self.get_logger().info(f"cuRobo planned successfully with {len(trajectory_points)} waypoints. Sending trajectory to joints action server...")
                    
                    # Dispatch trajectory to MoveToJoints action server
                    goal_msg = MoveToJoints.Goal()
                    goal_msg.trajectory_points = trajectory_points
                    return await self._send_action_goal(self._action_joints_client, goal_msg)
                else:
                    self.get_logger().error(f"cuRobo trajectory planning failed: {result.status}. Aborting motion for safety.")
                    return False
            except Exception as e:
                self.get_logger().error(f"Error during cuRobo motion planning: {e}. Falling back to legacy MoveToPose Cartesian move.")
                # Fall through to legacy point-to-point if there is a python planning error
        
        # Standard Point-to-Point legacy fallback
        goal_msg = MoveToPose.Goal()
        goal_msg.x = x
        goal_msg.y = y
        goal_msg.z = z
        goal_msg.theta_x = theta_x
        goal_msg.theta_y = theta_y
        goal_msg.theta_z = theta_z
        goal_msg.speed_scaling = speed

        self.get_logger().info(f'Sending Legacy Pose goal: X={x}, Y={y}, Z={z}')
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


    async def toggle_admittance(self, enable: bool):
        """Calls the toggle_admittance service."""
        request = SetBool.Request()
        request.data = enable
        
        if not self.admittance_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Admittance service not available')
            return False
            
        result = await self._await_rclpy_future(self.admittance_client.call_async(request))
        return result.success

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
