# Gemini Project: Kinova Gen3 + Gemini Vision Integration

This project integrates a **7 DOF Kinova Gen3 robot** (using the Kortex API) with **Google Gemini** models (using `gemini-1.5-flash` and `gemini-robotics-er-1.5-preview`) for natural language and vision-guided robotics tasks. The robot has an attached Intel RealSense camera and uses a Robotiq 2F-140 gripper.

## Project Goal
The goal of this project is to create a robust, natural language interface for a robot arm that can perform complex, multi-step tasks in unstructured environments. By combining high-level reasoning (Gemini Flash) with specialized robotics vision (Gemini ER), the system can autonomously perceive its workspace and execute long-horizon pick-and-place or manipulation tasks.

## Project Status
- **Low-Level Control**: Fully functional ROS 2 action servers in C++ for Joint, Pose, and Gripper control.
- **Reasoning Loop**: Functional multi-turn conversational interface. Gemini Flash orchestrates tasks, maintains state, and sequences actions.
- **Vision Integration**: Functional "Tool-Based Vision" pipeline. The `inspect_scene` tool uses Gemini ER for segmentation and maps pixels to 3D coordinates using "Median of Mask" depth processing and TF transforms.
- **State Feedback**: The model receives live RGB images from RealSense, spatial context via RViz snapshots, and precise numerical robot state (poses, joints, gripper).
- **Long-Horizon Tasks**: Successfully demonstrated tasks like "Put the toy in the box" and "Move X between Y and Z," requiring multiple sequenced tool calls and intermediate visual inspections.

## Project Overview

- **kortex_controller (C++)**: The core ROS 2 node interfacing with the Kinova hardware.
- **gemini_robotics (Python)**: The primary ROS 2 intelligence package.
    - `gemini_brain_node.py`: The "Reasoning" center. Connects to Google Gemini, handles tool calling, and commands the robot controller. Uses a dual-model approach (Flash + ER).
    - `gemini_tools.py`: Defines the tools available to Gemini (inspect_scene, move_to_position, etc.).
    - `vision_utils.py`: Contains math for pixel-to-3D projection and mask parsing.
    - `web_ui/`: A modern graphical interface for chat and visual feedback.
- **ros2_interfaces**: Custom ROS 2 action and message definitions (`MoveToPose`, `RobotState`, etc.).
- **config.yaml**: Robot settings and pre-defined positions.

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
   ros2 run gemini_robotics gemini_brain
   ```
3. **Terminal 3: Voice/Text Interface (if not using Web UI)**
   ```bash
   ros2 run gemini_robotics voice_interface
   ```

## Architecture

1.  **Input**: User commands via Web UI, text, or voice.
2.  **Reasoning (Gemini 1.5 Flash)**: Processes text + images (RealSense & RViz), maintains state, and sequences tool calls.
3.  **Vision (Gemini 1.5 ER)**: When `inspect_scene` is called, provides high-precision segmentation masks.
4.  **Math**: `vision_utils` projects masks to 3D base coordinates.
5.  **Execution**: `kortex_controller` moves the physical hardware.

## Key Files
- `src/kortex_controller/src/controller.cpp`: C++ ROS 2 action servers.
- `src/gemini_robotics/gemini_robotics/gemini_brain_node.py`: Central reasoning and tool orchestration.
- `src/gemini_robotics/gemini_robotics/vision_utils.py`: Pixel-to-3D logic and mask parsing.
- `src/gemini_robotics/gemini_robotics/gemini_tools.py`: Tool definitions for the Gemini API.
- `config.yaml`: Joint positions and robot configuration.
- `scripts/gemini-robotics-er.ipynb`: Reference for ER model output and visualization.


## User-Specific Instructions
- The Python scripts in `install/` require their shebang lines updated to point to the conda environment to find the `google-genai` library. Use the `sed` command provided in the Building section.
- Please do not try to run shell commands (like `colcon build` or anything else), leave that to me to handle. 