# Gemini Project: Kinova Gen3 + Gemini Vision Integration

This project integrates a **7 DOF Kinova Gen3 robot** (using the Kortex API) with **Google Gemini** models (using `gemini-robotics-er-1.5-preview`) for natural language and vision-guided robotics tasks. The robot has an attached Intel RealSense camera and uses a Robotiq 2F-140 gripper. The overall goal of this project is to use the Gemini-Robotics-ER model to control the robot in response to voice commands. This repository will focus on low-level actions (e.g. "close gripper", "move to pre-set position, move to ), not grasping or long-horizon tasks.

## Project Status
- **Low-Level Control**: Fully functional ROS2 action servers in C++ for Joint, Pose, and Gripper control.
- **Text-to-Command**: Functional natural language interface. Gemini decodes user intent into ROS2 action calls using Function Calling (Tools).
- **Vision (WIP)**: RealSense is launchable through a ROS2 command, but pixel-to-3D coordinate transformation and Gemini Vision integration into the "Brain" node are not yet complete. Vision is necessary to complete any `move_to_pose` task (e.g. "move to the blue object" requires the Gemini model to find the object, output the desired coordinates, map the coordinates to the dimensions of the RS camera frame, get the depth at that point, and construct the desired pose command to send to the robot, taking into account the current robot pose). This will require knowledge of the robot's state through ROS2 transforms for the Kinova 7DOF Gen3 (must know the current robot pose to move to the new pose), which is not set up yet.
- **Voice (WIP)**: Voice input is not yet implemented; currently uses a CLI-based text interface.


## Project Overview

- **kortex_controller (C++)**: The core ROS2 node interfacing with the Kinova hardware.
- **gemini_robotics (Python)**: The primary ROS2 intelligence package.
    - `gemini_brain_node.py`: The "Reasoning" center. Connects to Google Gemini, handles tool calling, and commands the robot controller.
    - `text_interface_node.py`: A CLI-based node for entering commands (published to `/user_instructions`).
    - `robot_controller_ros2.py`: The Python action client that bridges the Brain node to the C++ Controller.
- **ros2_interfaces**: Custom ROS2 action and message definitions (`MoveToPose`, `MoveToJoints`, `GripperCommand`).
- **config.yaml**: Robot settings (e.g., home joint positions).

## Building and Running

### Prerequisites
- ROS 2 (Jazzy/Humble)
- Kinova Kortex API
- Conda Environment: `kinova-gemini` (contains `google-genai`, `python-dotenv`, etc.)

### Building
```bash
colcon build --symlink-install
# After first build or when scripts change, fix shebangs:
sed -i '1s|.*|#!/home/mcrr-lab/anaconda3/envs/kinova-gemini/bin/python3|' install/gemini_robotics/lib/gemini_robotics/*
```

### Running the System
You need three terminals (each sourced with `source install/setup.bash`):

1. **Terminal 1: Kortex Controller**
   ```bash
   ros2 run kortex_controller controller --ros-args -p robot_ip:=192.168.1.10
   ```
2. **Terminal 2: Gemini Brain**
   ```bash
   ros2 run gemini_robotics gemini_brain
   ```
3. **Terminal 3: Text Interface (User CLI)**
   ```bash
   ros2 run gemini_robotics text_interface
   ```
4. **RealSense Camera**
   ```bash
   ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true
   ```
   While not necessary for `move_to_joint` or `set_gripper` commands, running this command exposes these ROS2 topics from the RealSense camera:
   ```bash
   /camera/camera/aligned_depth_to_color/camera_info
   /camera/camera/aligned_depth_to_color/image_raw
   /camera/camera/color/camera_info
   /camera/camera/color/image_raw
   /camera/camera/color/metadata
   /camera/camera/depth/camera_info
   /camera/camera/depth/image_rect_raw
   /camera/camera/depth/metadata
   /camera/camera/extrinsics/depth_to_color
   ```


## Architecture

1.  **Input (text_interface)**: Captures user text and publishes it to the `/user_instructions` topic.
2.  **Reasoning (gemini_brain)**: 
    - Subscribes to instructions.
    - Sends text + Tool definitions to Gemini.
    - Receives Function Calls (e.g., `move_to_joints`).
3.  **Command Execution (robot_controller)**: Translates AI function calls into ROS2 Action goals.
4.  **Hardware Control (kortex_controller)**: Executes the physical motion via Kortex API.

## Key Files
- `src/kortex_controller/src/controller.cpp`: C++ ROS2 action servers.
- `src/gemini_robotics/gemini_robotics/gemini_brain_node.py`: Gemini API logic and tool calling.
- `src/gemini_robotics/gemini_robotics/robot_controller_ros2.py`: ROS2 Action Client wrapper.
- `config.yaml`: Predefined positions and robot IP.
-`scripts/gemini-robotics-er.ipynb`: Defines different use cases and proper visualization for the output of `gemini-robotics-1.5-preview` model. All future visualiztion should model this use case.

## User-Specific Instructions
- I am a mechanical engineer by training, and unfamiliar with many concepts that may be obvious to software engineers. Please be sure to explain what you are doing and why you are doing it through the process so I can better learn robotics software development.
- The Python scripts in `install/` require their shebang lines updated to point to the conda environment to find the `google-genai` library. Use the `sed` command provided in the Building section.
