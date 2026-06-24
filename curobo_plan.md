# NVIDIA cuRobo Integration Plan for Kinova Gen3

This document outlines the detailed implementation plan to integrate **NVIDIA cuRobo** into the `kinova-gemini` workspace. cuRobo is a GPU-accelerated motion generation library that will act as a safe, collision-free trajectory planner below the high-level Gemini reasoning loop.

---

## Architectural Overview

```
                      +-----------------------------+
                      |      Gemini Live Brain      |
                      |   (High-level reasoning)    |
                      +--------------+--------------+
                                     |
                                     | Call tool (e.g., move_to_pose)
                                     v
                      +-----------------------------+
                      |  robot_controller_ros2.py   | <---+ Get Obstacles from
                      |   (Interceptors & cuRobo)   |     | Gemini/SAM 2 Bounding Boxes
                      +--------------+--------------+
                                     |
                                     | Plans collision-free joint waypoints (~30ms)
                                     v
                      +-----------------------------+
                      |      kortex_controller      |
                      |  (C++ Direct-to-TCP Driver) |
                      +--------------+--------------+
                                     |
                                     | Executes via ExecuteWaypointList RPC
                                     v
                      +-----------------------------+
                      |    Kinova Gen3 Hardware     |
                      +-----------------------------+
```

By intercepting high-level movement commands in the Python client layer (`robot_controller_ros2.py`), we can query cuRobo for a collision-free joint trajectory before passing the path down to the C++ Kortex controller.

---

## Phase 1: Trajectory Execution in C++ Driver

Currently, `kortex_controller` executes joint commands point-to-point. To support multi-point trajectory execution, we must update the C++ node to support Kortex's native waypoint execution.

### 1.1 Update ROS 2 Interface
Update `MoveToJoints.action` in `ros2_interfaces` to allow passing a sequence of joint configurations rather than a single goal.
Alternatively, we can create a new action called `FollowJointTrajectory.action` or use the standard `control_msgs/action/FollowJointTrajectory`. Given our direct TCP structure, a custom simple action of joint waypoint lists is cleanest:

```protobuf
# ros2_interfaces/action/MoveToJoints.action
# Goal
float64[] joint_angles          # Single target (legacy support)
float64[][] trajectory_points   # 2D list of shape [num_waypoints, 7] (new)
---
# Result
bool success
---
# Feedback
float32 progress
```

### 1.2 Implement `ExecuteWaypointList` in `controller.cpp`
Modify the `execute_joints` thread inside `src/kortex_controller/src/controller.cpp` to check if a trajectory is provided. If so, convert the joint waypoints into Kortex `AngularWaypoint` objects and execute them using `EXECUTE_WAYPOINT_LIST`:

```cpp
// Pseudocode for controller.cpp waypoint conversion
k_api::Base::Action action;
action.set_name("Trajectory Execution");
auto waypoints = action.mutable_execute_waypoint_list();
waypoints->set_duration(0.0); // Optimal timing
waypoints->set_use_optimal_blending(true);

for (size_t i = 0; i < trajectory_points.size(); ++i) {
    auto wp = waypoints->add_waypoints();
    wp->set_name("waypoint_" + std::to_string(i));
    auto angular_wp = wp->mutable_angular_waypoint();
    
    for (size_t j = 0; j < 7; ++j) {
        angular_wp->add_angles(static_cast<float>(trajectory_points[i][j]));
    }
    // Set moderate joint speed limits for safety (e.g., 30 deg/s)
    for (size_t j = 0; j < 7; ++j) {
        angular_wp->add_maximum_velocities(30.0f);
    }
}

// Execute on the hardware via the TCP client
mBase->ExecuteAction(action);
```

---

## Phase 2: cuRobo Installation & Setup

Since you are running SAM 2 and AnyGrasp, your machine is already equipped with an NVIDIA GPU and PyTorch/CUDA.

### 2.1 Installation
Install cuRobo inside your Conda environment (`kinova-gemini`):

```bash
conda activate kinova-gemini
pip install curobo --extra-index-url https://pypi.nvidia.com
```

### 2.2 Configure Robot Representation
cuRobo needs a configuration file for the Kinova Gen3 (7 DOF) with the Robotiq gripper. We will create a `kinova_gen3.yml` in a config directory matching your physical robot's joint limits and link collision meshes.

Example snippet of `kinova_gen3.yml`:
```yaml
robot_cfg:
  kinematics:
    urdf_path: "package://kortex_description/robots/kinova.urdf.xacro"
    base_link: "base_link"
    ee_link: "end_effector_link"
    link_names:
      - "joint_1"
      - "joint_2"
      - "joint_3"
      - "joint_4"
      - "joint_5"
      - "joint_6"
      - "joint_7"
    cspace_limits:
      lower: [-3.1415, -2.24, -3.1415, -2.57, -3.1415, -2.09, -3.1415]
      upper: [3.1415, 2.24, 3.1415, 2.57, 3.1415, 2.09, 3.1415]
```

---

## Phase 3: Obstacle Representation and World Sync

Safe motion planning requires a representation of the physical environment. We will represent obstacles in two ways:

### 3.1 Static Environment
A pre-defined table/workspace obstacle so the robot never collides with its mounting surface or surrounding walls.

```python
# In python controller
from curobo.geom.types import Cuboid, WorldConfig

table = Cuboid(
    name="table",
    pose=[0.0, 0.0, -0.05, 1, 0, 0, 0], # centered under robot base
    dims=[1.2, 1.2, 0.1]
)
world_config = WorldConfig(cuboid=[table])
```

### 3.2 Dynamic Semantic Obstacles (SAM 2 & Gemini Bounding Boxes)
Whenever SAM 2 or Gemini Live identifies objects in the scene (e.g., cups, bowls, obstacles):
1. Project the 2D bounding boxes to 3D workspace coordinates using `vision_utils.py` (pixel-to-3D projection).
2. Generate a 3D `Cuboid` obstacle representing the object’s bounding volume.
3. Call `motion_gen.update_world(new_world_config)` before planning the path.

### 3.3 Wrist-Mounted Camera Coordination (Transform & Memory Management)
Because the Intel RealSense camera is mounted on the robot wrist, its viewpoint changes dynamically during movement. Without careful design, this leads to **blind spots** (obstacles disappear from the camera’s view and are subsequently forgotten by the planner).

To resolve this, we leverage **Object-Level Memory Management**:
1. **Coordinate Frame Synchronization (TF):** Every time a frame is processed, we query ROS 2's `tf2` library to get the exact transform matrix from the camera frame (`camera_depth_optical_frame`) to the static base frame (`base_link`) at that precise nanosecond.
2. **Static Mapping:** We project the detected target objects into `base_link` coordinates.
3. **Persistent World State:** Once an object's position is mapped to `base_link`, we register it as a persistent `Cuboid` or `Sphere` obstacle in cuRobo's active `WorldConfig`.
4. **Memory Lifetime:** These obstacles remain in the planner's world representation even when the wrist camera rotates away. They are only updated if the camera re-observes the same volume and detects a state change (space-carving or object movement), or if explicitly removed by the high-level brain node when a task is completed.

### 3.4 The Grasping/Interaction Challenge (Collision Filtering & Attachment)
A major challenge in motion planning is that the target object of a grasp is technically an **obstacle** to be avoided. If we attempt to plan a trajectory directly to the final grasp pose, cuRobo will fail because the target pose results in a collision between the gripper fingers and the object.

To solve this, we implement a **Multi-Stage Grasping State Machine**:

```
[Start Pose]
     │
     │ Stage 1: cuRobo motion plan (cup is a solid obstacle)
     ▼
[Pre-Grasp Pose]  <-- Offset 5-10cm along gripper approach vector
     │
     │ Stage 2: Disable target obstacle, plan linear Cartesian approach
     ▼
[Grasp Pose]      <-- Gripper fingers surrounding object
     │
     │ Stage 3: Close Gripper, detect contact
     ▼
[Grasped State]
     │
     │ Stage 4: Attach object to gripper link in cuRobo world representation
     ▼
[Lift & Navigate] <-- Object becomes part of robot; checked for environmental collisions
```

#### 1. Stage 1: Planning to the Pre-Grasp Pose
We compute an offset pose $5\text{ to }10\text{ cm}$ away from the target object along the gripper's Z-axis (approach vector). We query cuRobo to find a safe, collision-free path from the current position to this **Pre-Grasp Pose**, keeping the target object active as an obstacle. This ensures the robot never hits the target during its main travel path.

#### 2. Stage 2: Final Approach (Collision Filtering)
From the Pre-Grasp Pose, we perform a short, direct, linear Cartesian translation to the final grasp pose. During this stage, we handle collisions in one of two ways:
* **Temporary Removal:** We call `planner.remove_obstacle("target_cup")` so cuRobo doesn't block the motion.
* **Collision Masking:** We update cuRobo's Collision Matrix to allow collisions *specifically* between the robot's gripper links and the `"target_cup"`, while still preventing collision with other obstacles (like the table).

#### 3. Stage 3: Attachment (Post-Grasp Navigation)
Once the gripper is closed and physical contact is secured, the object moves *with* the robot. To prevent cuRobo from flagging self-collisions or hitting the table with the held object during subsequent movements, we attach the object to the robot's kinematic chain:
```python
# Attach object to gripper base in cuRobo
planner.attach_obstacle(
    name="target_cup",
    link_name="end_effector_link",
    pose=relative_grasp_pose # Relative offset transform
)
```
cuRobo will now treat the object as a dynamic extension of the gripper mesh. When planning future movements (e.g., placing the cup elsewhere), cuRobo will actively prevent the *cup itself* from colliding with the table, walls, or other objects!

---

## Phase 4: Intercepting & Planning inside Python Node

We modify `robot_controller_ros2.py` to seamlessly execute plans under the hood.

### 4.1 Instantiating the Motion Generator
Within `KinovaRobotControllerROS2`:

```python
import torch
from curobo.wrap.planner.motion_gen import MotionGen, MotionGenConfig, MotionGenPlanConfig
from curobo.types.base import TensorDeviceType

class KinovaRobotControllerROS2(Node):
    def __init__(self):
        # ... legacy initialization ...
        
        # Initialize cuRobo
        self.tensor_args = TensorDeviceType(device=torch.device("cuda:0"), dtype=torch.float32)
        
        motion_gen_config = MotionGenConfig.load_from_robot_config(
            robot_cfg_path,
            world_config,
            self.tensor_args,
            collision_checker_type="mesh",
            use_cuda_graph=True
        )
        self.planner = MotionGen(motion_gen_config)
        self.planner.warmup() # pre-compile CUDA kernels
```

### 4.2 Intercepting `move_to_pose`
Instead of sending a direct Cartesian action down to C++, we query the planner:

```python
async def move_to_pose(self, x, y, z, theta_x=0.0, theta_y=0.0, theta_z=0.0, speed=0.1):
    # 1. Get current joint states from /robot_state topic
    current_joints = self.current_state.joint_angles # list of 7 values
    
    # 2. Define goal pose
    goal_pose = Pose(
        position=torch.tensor([[x, y, z]], **self.tensor_args.as_torch_dict()),
        quaternion=euler_to_quaternion(theta_x, theta_y, theta_z) # Helper function
    )
    
    # 3. Generate optimal, collision-free joint trajectory
    start_state = torch.tensor([current_joints], **self.tensor_args.as_torch_dict())
    result = self.planner.plan_single(start_state, goal_pose, MotionGenPlanConfig(timeout=0.5))
    
    if result.success:
        interpolated_plan = result.get_interpolated_plan()
        joint_trajectory = interpolated_plan.position.cpu().numpy().tolist()
        
        # 4. Send this 2D list of waypoints down to updated C++ joint controller
        goal_msg = MoveToJoints.Goal()
        goal_msg.trajectory_points = joint_trajectory
        return await self._send_action_goal(self._action_joints_client, goal_msg)
    else:
        self.get_logger().error(f"Planning failed: {result.status}")
        return False
```

---

## Phase 5: Verification and Testing

1. **Dry-Run (Simulation/Rviz):**
   Start `robot.launch.py` with `use_fake_hardware:=true`. Observe inside RViz that the robot smoothly paths around the simulated table and any custom cuboid obstacles you spawn.
2. **Obstacle Collision-Avoidance Test:**
   - Place a virtual bounding box obstacle directly in the line of sight between the Home position and the User position.
   - Call `move_to_user()`.
   - Verify that the robot doesn't move in a straight line, but dynamically arches over/around the virtual box.
3. **Hardware Integration:**
   - Execute the movement at extremely slow scaling speeds (e.g., `speed=0.05`) on the physical robot first.
   - Keep your hand close to the physical emergency stop button on the Kinova Gen3 controller while executing the initial trajectories.
