#!/usr/bin/env python3
"""
One-time generator for config/spheres/kinova_gen3_robotiq.yml.

cuRobo needs a sphere approximation of every collision link to do collision checking.
This project's arm links reuse the spheres cuRobo ships for its own Gen3 example robot
(same kinematics), but the Robotiq 2F-140 gripper has no equivalent upstream - it never
existed in cuRobo's bundled config - so its spheres are fit here directly from the
gripper's own collision meshes using cuRobo's mesh-to-sphere fitting utility.

Run once (and again only if the gripper meshes/URDF change):
    python3 generate_gripper_collision_spheres.py

Requires the kinova-gemini conda env (curobo + trimesh installed).
"""

import yaml
from curobo.geom.sphere_fit import SphereFitType, fit_spheres_to_mesh
from curobo.util_file import get_robot_configs_path, join_path, load_yaml
import trimesh

# Arm links: copy verbatim from cuRobo's bundled Gen3 config rather than refitting -
# same arm, same kinematics.
ARM_LINK_NAMES = [
    "base_link",
    "shoulder_link",
    "half_arm_1_link",
    "half_arm_2_link",
    "forearm_link",
    "spherical_wrist_1_link",
    "spherical_wrist_2_link",
    "bracelet_link",
]

GRIPPER_MESH_ROOT = (
    "/home/mcrr-lab/workspace/ros2_kortex_ws/install/robotiq_description/"
    "share/robotiq_description/meshes/collision/2f_140"
)

# Gripper links to fit spheres for, and which collision STL each one uses. Collision
# origins in the URDF are all identity, so spheres fit in mesh-local space line up
# directly with link-local space - no extra transform needed.
GRIPPER_MESH_LINKS = {
    "robotiq_140_base_link": "robotiq_2f_140_base_link.stl",
    "left_outer_knuckle": "robotiq_2f_140_outer_knuckle.stl",
    "left_outer_finger": "robotiq_2f_140_outer_finger.stl",
    "left_inner_finger": "robotiq_2f_140_inner_finger.stl",
    "left_inner_knuckle": "robotiq_2f_140_inner_knuckle.stl",
    "right_outer_knuckle": "robotiq_2f_140_outer_knuckle.stl",
    "right_outer_finger": "robotiq_2f_140_outer_finger.stl",
    "right_inner_finger": "robotiq_2f_140_inner_finger.stl",
    "right_inner_knuckle": "robotiq_2f_140_inner_knuckle.stl",
}
SPHERES_PER_GRIPPER_LINK = 5

# The two finger pads are plain URDF boxes (0.03 x 0.07 x 0.0075 m collision box, no
# mesh) - approximate each with spheres sized to the box rather than mesh-fitting.
PAD_LINKS = ["left_inner_finger_pad", "right_inner_finger_pad"]
PAD_SPHERES = [
    {"center": [0.0, 0.02, 0.0], "radius": 0.012},
    {"center": [0.0, -0.02, 0.0], "radius": 0.012},
]


def main():
    bundled_path = join_path(get_robot_configs_path(), "spheres/kinova_gen3.yml")
    bundled_spheres = load_yaml(bundled_path)["collision_spheres"]

    collision_spheres = {name: bundled_spheres[name] for name in ARM_LINK_NAMES}

    for link_name, mesh_file in GRIPPER_MESH_LINKS.items():
        mesh_path = join_path(GRIPPER_MESH_ROOT, mesh_file)
        mesh = trimesh.load(mesh_path)
        centers, radii = fit_spheres_to_mesh(
            mesh,
            n_spheres=SPHERES_PER_GRIPPER_LINK,
            surface_sphere_radius=0.005,
            fit_type=SphereFitType.VOXEL_VOLUME_SAMPLE_SURFACE,
        )
        collision_spheres[link_name] = [
            {"center": [float(c) for c in center], "radius": float(radius)}
            for center, radius in zip(centers, radii)
        ]
        print(f"{link_name}: fit {len(centers)} spheres from {mesh_file}")

    for link_name in PAD_LINKS:
        collision_spheres[link_name] = PAD_SPHERES

    out = {"collision_spheres": collision_spheres}
    out_path = "/home/mcrr-lab/kinova-gemini/config/spheres/kinova_gen3_robotiq.yml"
    with open(out_path, "w") as f:
        yaml.safe_dump(out, f, sort_keys=False)
    print(f"Wrote {out_path}")


if __name__ == "__main__":
    main()
