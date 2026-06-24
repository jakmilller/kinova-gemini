# Gemini Project: Kinova Gen3 + Gemini Vision Integration

This project integrates a **7 DOF Kinova Gen3 robot** (using the Kortex API) with a **Gemini Live** model for natural language and vision-guided robotics tasks. The robot has an attached Intel RealSense camera and uses a Robotiq 2F-140 gripper.

## Project Goal
The goal of this project is to create a robust, natural language interface for a robot arm that can perform complex, multi-step caregiving tasks. By allowing Gemini to call certain robot functions (defined in a Tool Library), the system is able to trasnform user requests into actual robot actions. The overall goal is to combine the high-level reasoning with task-specific caregiving tools (feeding, grooming, etc) to create a general caregiving robot.

## Project Status
- **Low-Level Control**: Fully functional ROS 2 action servers in C++ for Joint, Pose, and Gripper control.
- **Reasoning Loop**: Functional multi-turn conversational interface using **Gemini 3.1 Live**. The model orchestrates tasks, maintains state, and sequences actions.
- **Advanced Vision and Grasping**: Integrated **SAM 2 (Segment Anything Model 2)** for high-precision object segmentation and **AnyGrasp** for precise 6D grasp detection.
- **State Feedback**: The model receives live RGB images from RealSense and precise numerical robot state (poses, joints, gripper) once a second.
- **Web UI**: A local web interface to interact with the robot.

## Project Overview

- **kortex_controller (C++)**: The core ROS 2 node interfacing with the Kinova hardware. Includes the `compute_ik` service for converting Cartesian poses to joint angles.
- **gemini_live_robotics (Python)**: The primary ROS 2 intelligence package.
    - `gemini_live_brain_node.py`: The central reasoning center. Uses Gemini Live for high-level logic, receiving state info and user commands to decide the best tool to call. Orchestrates SAM2, AnyGrasp, and more advanced Gemini models for complex vision tasks.
    - `gemini_live_tools.py`: Defines the tools available to Gemini (inspect_scene, move_to_pose, move_to_position, etc.).
    - `vision_utils.py`: Contains utilities for pixel-to-3D projection and mask parsing.
    - `web_ui/`: A modern graphical interface for chat and visual feedback.
- **gemini_robotics (Python)**: OUT OF USE. The OLD ROS 2 intelligence package. Still contains some voice interface stuff, but has been replaced by gemini_live_robotics
- **ros2_interfaces**: Custom ROS 2 action and message definitions (`MoveToPose`, `RobotState`, `ComputeIK`, etc.).
- **config.yaml**: Model selection, robot settings, and pre-defined positions.

## Building and Running

### Building
```bash
colcon build --symlink-install
# After first build or when scripts change, fix shebangs:
sed -i '1s|.*|#!/home/mcrr-lab/anaconda3/envs/kinova-gemini/bin/python3|' install/gemini_robotics/lib/gemini_robotics/*
```

### Running the System
1. **Terminal 1: Robot & Camera Bringup**
   ```bash
   ros2 launch kinova_bringup robot.launch.py
   ```
2. **Terminal 2: Gemini Brain**
   ```bash
   ros2 run gemini_live_robotics gemini_live_brain
   ```


## Architecture

1.  **Input**: User commands via Web UI, text, or voice.
2.  **Reasoning**: A primary Gemini Live model processes text + images, maintains state, and sequences tool calls.
3.  **Vision (SAM 2 & AnyGrasp)**: 
    - In some tools, specialized Gemini models provide a bounding box for objects.
    - **SAM 2** refines this into a high-precision segmentation mask.
    - For grasping, **AnyGrasp** processes the segmented point cloud to determine the optimal 6D grasp pose.
4.  **Kinematics**: The `compute_ik` service (C++) converts the 6D grasp pose into executable joint angles. This is only for the 6D grasps, other grasps move in Cartesian space.
5.  **Execution**: `kortex_controller` moves the physical hardware.

## Key Files
- `src/kortex_controller/src/controller.cpp`: C++ ROS 2 action servers and the `compute_ik` service.
- `src/gemini_robotics/gemini_live_robotics/gemini_live_brain_node.py`: Central reasoning, tool orchestration, and vision pipeline integration (SAM 2, AnyGrasp).
- `src/gemini_robotics/gemini_live_robotics/vision_utils.py`: Pixel-to-3D logic and mask parsing utilities.
- `src/gemini_robotics/gemini_live_robotics/gemini_live_tools.py`: Tool definitions (Function Declarations) for the Gemini API.
- `config.yaml`: Model selection, joint positions, and robot configuration parameters.
- `src/ros2_interfaces/srv/ComputeIK.srv`: Service definition for the Inverse Kinematics solver.

## Detailed Tool Reference

### Vision & Inspection
- **`inspect_scene()`**: Performs a semantic analysis of the workspace.
    1. Custom Gemini model (not Gemini Live) on a target object.
    1. Gemini identifies the best part to grasp (e.g., "handle"), and draws a bounding box around it.
    2. SAM 2 segments the part with high precision.
    3. AnyGrasp evaluates the segmented point cloud to find the optimal 6D pose, only considering the object point clouds within the bounding box.
    4. `compute_ik` solves for the required joint angles.
    5. The robot executes a multi-stage grasp (Pre-Grasp -> Grasp -> Pinch).

### Motion & Manipulation
- **`move_to_position(x, y, z)`**: Moves the gripper to an absolute (x, y, z) coordinate in the base frame. Does not change griper orientation.
- **`move_to_home()`**: Returns the robot to a pre-defined observation pose.
- **`move_to_user()`**: Moves the robot to a pose optimized for user interaction.
- **`adjust_joints(joint_number, amount)`**: Adjusts a specific joint angle (1-7) by a relative amount in degrees.
- **`grasp_object()`**: Closes the gripper until contact is detected.
- **`open_gripper()`**: Fully opens the gripper.

### Task Management
- **`task_complete()`**: Signals that the current user instruction has been fully executed.

## User-Specific Instructions
- The Python scripts in `install/` require their shebang lines updated to point to the conda environment to find the `google-genai`, `torch`, and `sam2` libraries. Use the `sed` command provided in the Building section.
- Please do not try to run shell commands (like `colcon build` or anything else), leave that to me to handle. 
- When making code changes, ALWAYS provide a text explanation in the terminal before you do them. They should be explanatory, with the goal of teaching critical software engineering concepts to me.