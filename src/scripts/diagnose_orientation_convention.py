#!/usr/bin/env python3
"""
Diagnostic (read-only, makes no robot motion): checks whether the Kortex API's reported
theta_x/theta_y/theta_z Cartesian orientation, fed through scipy's R.from_euler('xyz', ...,
degrees=True) (as robot_controller_ros2.py's move_to_pose does), actually matches the
end_effector_link orientation that cuRobo's own forward kinematics computes from the SAME
joint state. If they don't match, the Euler convention/frame used to build cuRobo's goal
quaternion from Kortex's theta values is wrong, which would explain IK_FAIL on Cartesian
targets that are physically reachable.

Run with the robot powered on and kortex_controller already running (no need for
gemini_live_brain):
    python3 diagnose_orientation_convention.py
"""

import numpy as np
import rclpy
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
import torch

from curobo.cuda_robot_model.cuda_robot_model import CudaRobotModel, CudaRobotModelConfig
from curobo.types.base import TensorDeviceType

from ros2_interfaces.msg import RobotState

ROBOT_CFG_PATH = "/home/mcrr-lab/kinova-gemini/config/kinova_gen3.yml"


def quat_angle_diff_deg(q1_wxyz, q2_wxyz):
    """Angle (deg) between two orientations given as wxyz quaternions."""
    r1 = R.from_quat([q1_wxyz[1], q1_wxyz[2], q1_wxyz[3], q1_wxyz[0]])
    r2 = R.from_quat([q2_wxyz[1], q2_wxyz[2], q2_wxyz[3], q2_wxyz[0]])
    rel = r1.inv() * r2
    return np.degrees(rel.magnitude())


class OrientationDiagnostic(Node):
    def __init__(self):
        super().__init__("orientation_diagnostic")
        self.sub = self.create_subscription(RobotState, "robot_state", self.callback, 10)
        self.done = False

        tensor_args = TensorDeviceType(device=torch.device("cuda:0"), dtype=torch.float32)
        cfg = CudaRobotModelConfig.from_robot_yaml_file(ROBOT_CFG_PATH, tensor_args=tensor_args)
        self.kin_model = CudaRobotModel(cfg)
        self.tensor_args = tensor_args

    def callback(self, msg: RobotState):
        if self.done:
            return
        self.done = True

        joints_rad = [
            float(a - 360.0 if a > 180.0 else a) * np.pi / 180.0 for a in msg.joint_angles
        ]
        q = torch.tensor([joints_rad], **self.tensor_args.as_torch_dict())
        state = self.kin_model.get_state(q)
        fk_quat_wxyz = state.ee_quaternion[0].cpu().numpy().tolist()
        fk_pos = state.ee_position[0].cpu().numpy().tolist()

        print(f"\nKortex-reported Cartesian pose: x={msg.x:.4f} y={msg.y:.4f} z={msg.z:.4f} "
              f"theta_x={msg.theta_x:.3f} theta_y={msg.theta_y:.3f} theta_z={msg.theta_z:.3f}")
        print(f"cuRobo FK end_effector_link pose: pos={fk_pos} quat_wxyz={fk_quat_wxyz}")
        world_delta = [msg.x - fk_pos[0], msg.y - fk_pos[1], msg.z - fk_pos[2]]
        print(f"Position delta (Kortex x,y,z vs cuRobo FK pos), meters, in BASE frame: {world_delta}")

        # Express the same delta in end_effector_link's own local frame (rotate by the
        # inverse of the FK orientation). If Kortex's reported position is end_effector_link
        # plus a fixed tool/TCP offset (e.g. for the gripper), this local-frame vector should
        # come out the same regardless of which pose the arm is in - run this script from a
        # second, differently-oriented pose and compare.
        r_fk = R.from_quat([fk_quat_wxyz[1], fk_quat_wxyz[2], fk_quat_wxyz[3], fk_quat_wxyz[0]])
        local_delta = r_fk.inv().apply(world_delta)
        print(f"Same delta expressed in end_effector_link's LOCAL frame: {local_delta.tolist()}")
        print(f"Magnitude: {np.linalg.norm(world_delta):.4f} m")

        theta = [msg.theta_x, msg.theta_y, msg.theta_z]
        conventions = {
            "extrinsic xyz (current code)": "xyz",
            "intrinsic XYZ": "XYZ",
            "extrinsic zyx": "zyx",
            "intrinsic ZYX": "ZYX",
        }
        print("\nAngle between cuRobo FK quaternion and quaternion decoded from "
              "Kortex theta_x/y/z under various Euler conventions:")
        for label, seq in conventions.items():
            q_xyzw = R.from_euler(seq, theta, degrees=True).as_quat()
            q_wxyz = [q_xyzw[3], q_xyzw[0], q_xyzw[1], q_xyzw[2]]
            diff = quat_angle_diff_deg(fk_quat_wxyz, q_wxyz)
            print(f"  {label:35s}: {diff:7.2f} deg difference")

        print("\nIf one convention shows ~0 deg difference, that's the correct decode. "
              "If all are large and roughly constant, look for a fixed rotation offset "
              "(e.g. a 90 deg gripper-frame correction) instead of a convention mismatch.")

        rclpy.shutdown()


def main():
    rclpy.init()
    node = OrientationDiagnostic()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
