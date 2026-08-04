"""Grasp geometry helpers for the Gemini Live brain node.

Stateless math for the two grasp pipelines: mapping a desired finger opening to a Kortex
gripper command, building the point cloud AnyGrasp consumes, and turning an AnyGrasp 6D
grasp into a base-frame tool pose. No ROS-node state — inputs are passed explicitly.
"""

import numpy as np
from scipy.spatial.transform import Rotation as R


GRIPPER_MAX_WIDTH_M = 0.14  # Robotiq 2F-140 fully-open finger spacing
GRIPPER_FULLY_CLOSED_THRESHOLD = 95.0  # gripper_position (0-100) above which we warn "may have missed"
GRIPPER_PRESHAPE_MARGIN_M = 0.02  # how much wider than the object to pre-shape the jaws

# --- Approach geometry ---
# Both distances are measured from the surface point inspect_scene reports (the median depth
# over the object's visible mask), along the gripper's own +Z.
#
# How far back from that point the pre-grasp pose sits. Far enough that the open jaws clear
# the object and its neighbours, near enough to stay reachable. Note the median sits slightly
# inside the object's nearest face, so the true clearance is a little less than this.
GRASP_STANDOFF_M = 0.04
# Ceiling on how far the fingertips travel PAST that point before closing, bounded by how much
# of the finger can usefully wrap an object. Deep objects get gripped at this depth rather than
# swallowed whole.
MAX_INSERTION_M = 0.05

# --- AnyGrasp candidate ranking ---
# How many of AnyGrasp's highest-confidence grasps to evaluate. Tried in order: score the first
# 4, and only if none of them (in either wrist roll) turns out to be reachable, widen to 8
# before giving up. Widening is the fallback rather than the default because a low-confidence
# grasp that happens to be convenient is not a trade we want to make routinely.
ANYGRASP_CANDIDATE_POOLS = (4, 8)

# Per-joint weights for joint_motion_cost, ordered base -> wrist. Proximal joints swing more
# mass through more of the workspace, so reaching a pose via the shoulder is a worse deal than
# reaching it via the wrist even when the total degrees are equal.
JOINT_MOTION_WEIGHTS = (3.0, 3.0, 2.0, 2.0, 1.0, 1.0, 1.0)

# Remaps AnyGrasp's gripper axes onto the Kortex end-effector axes: AnyGrasp's local +X
# (its approach direction) becomes the tool's +Z. Applied to the base-frame grasp rotation.
R_ALIGN = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]])


def width_to_gripper_percent(width_m):
    """Converts a desired finger opening (meters) to a Kortex gripper_position value (0=open, 100=closed)."""
    return max(0.0, min(100.0, (GRIPPER_MAX_WIDTH_M - width_m) / GRIPPER_MAX_WIDTH_M * 100.0))


def preshape_percent(width_m):
    """Gripper opening to pre-shape to before approaching an object of this width.

    The margin leaves the jaws slightly wider than the object so the approach does not clip it
    (or its neighbours) on the way in, while still being narrow enough to avoid bumping whatever
    is beside it. Both grasp pipelines go through here so the two can never drift apart -- and
    so the AnyGrasp ranking scores its candidates at the same opening they will be executed at.
    """
    return width_to_gripper_percent(width_m + GRIPPER_PRESHAPE_MARGIN_M)


def tool_z_axis(theta_x, theta_y, theta_z):
    """Unit vector along the tool's own +Z (the gripper's approach direction), in base coordinates.

    Kortex reports orientation as extrinsic x-y-z Tait-Bryan angles in degrees, which is
    scipy's lowercase 'xyz'. Mirrors Controller::toolZAxis in controller.cpp -- the same
    convention has to hold on both sides or approach poses and moves disagree.
    """
    return R.from_euler('xyz', [theta_x, theta_y, theta_z], degrees=True).apply([0.0, 0.0, 1.0])


def shift_along_tool_z(x, y, z, theta_x, theta_y, theta_z, distance):
    """Translate a point along the tool's own +Z by `distance` meters, returning (x, y, z).

    Positive moves toward whatever the gripper is pointing at, negative backs away from it --
    so a pre-grasp standoff is just a negative shift from the object's surface.
    """
    axis = tool_z_axis(theta_x, theta_y, theta_z)
    return (float(x + distance * axis[0]),
            float(y + distance * axis[1]),
            float(z + distance * axis[2]))


def flip_tool_roll(theta_x, theta_y, theta_z):
    """The other wrist roll that produces an IDENTICAL grasp: 180 degrees about the tool's own +Z.

    A parallel gripper is symmetric — swapping which jaw is which leaves the grasp unchanged —
    so every 6D grasp pose has a twin that puts the fingers in exactly the same place. Same
    fingertip position, same approach axis, only the wrist roll differs. Offering both to the
    candidate ranking is what stops the arm spinning the wrist half a turn for no reason.

    Right-multiplying rotates about the tool's OWN axis rather than the base's, which is the
    whole point: scipy composes so that (p * q) applies q first, in p's frame.
    """
    rotated = (R.from_euler('xyz', [theta_x, theta_y, theta_z], degrees=True)
               * R.from_euler('z', 180.0, degrees=True))
    return rotated.as_euler('xyz', degrees=True)


def joint_motion_cost(current_deg, target_deg, weights=JOINT_MOTION_WEIGHTS):
    """Weighted total joint travel between two 7-DOF configurations, in weighted degrees.

    Kortex reports joint angles in [0, 360), so 350 -> 10 degrees is a 20 degree move, not a
    340 degree one; each delta is wrapped the short way round, matching the completion check in
    controller.cpp and the wrap in _handle_adjust_joints.
    """
    total = 0.0
    for i, weight in enumerate(weights):
        delta = abs(float(current_deg[i]) - float(target_deg[i])) % 360.0
        if delta > 180.0:
            delta = 360.0 - delta
        total += weight * delta
    return total


def insertion_depth_m(object_depth_m, width_m=None):
    """How far past inspect_scene's reported surface point the fingertips travel before closing.

    Half the object's thickness aims the fingertips at its centre along the approach axis,
    which is where a parallel-jaw grip wants them. That is exact when the reported point is the
    object's nearest face and runs slightly deep otherwise, which is what MAX_INSERTION_M bounds.
    Falls back to the object's width when no depth estimate is available (a roughly round
    cross-section is the safest guess), and to the cap when neither is known.
    """
    depth = object_depth_m if object_depth_m else width_m
    if not depth or depth <= 0.0:
        return MAX_INSERTION_M
    return max(0.005, min(MAX_INSERTION_M, depth / 2.0))


def build_anygrasp_clouds(depth_image, camera_info, cv_rgb, binary_mask):
    """Lift depth + mask into the (points, colors, lims) tuple AnyGrasp wants.
    Returns (None, None, None) if the target mask has too few valid points.
    """
    h, w = cv_rgb.shape[:2]
    depths = depth_image.astype(np.float32)
    fx, fy = camera_info.k[0], camera_info.k[4]
    cx, cy = camera_info.k[2], camera_info.k[5]
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


def anygrasp_grasp_to_base_pose(grasp, T_base_cam):
    """Convert an AnyGrasp grasp (camera frame) into a base-frame tool pose.
    Returns (xyz, euler_xyz_degrees) with AnyGrasp's approach axis remapped onto the tool's +Z.
    """
    T_cam_grasp = np.eye(4)
    T_cam_grasp[:3, :3] = grasp.rotation_matrix
    T_cam_grasp[:3,  3] = grasp.translation
    T_base_grasp = T_base_cam @ T_cam_grasp
    R_tool = T_base_grasp[:3, :3] @ R_ALIGN
    xyz = T_base_grasp[:3, 3]
    euler = R.from_matrix(R_tool).as_euler('xyz', degrees=True)
    return xyz, euler
