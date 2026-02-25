# Gemini VLM Integration for Kinova Gen3

This repository integrates a 7 DOF Kinova Gen3 robot arm with Google's Gemini-Robotics-ER model (`gemini-robotics-er-1.5-preview`) for natural language and vision-guided robotics tasks. The robot uses an attached Intel RealSense camera and a Robotiq 2F-140 gripper.

Users can type natural language instructions, and the Gemini model will decode the intent into a series of function calls (ROS 2 actions) for movement, gripper control, and vision-based object location.

## Architecture Overview

The system consists of three main components:
1.  **kortex_controller (C++)**: The core ROS 2 node executing low-level actions via the Kinova hardware API.
2.  **gemini_robotics (Python)**: The "Brain" of the operation. Connects to the Gemini API, handles function calling, requests vision data, calculates 3D coordinates, and orchestrates the action servers.
3.  **text_interface (Python)**: A simple CLI node for user input.

For a more detailed breakdown, refer to the [architecture plan](architecture_plan.md).

## Prerequisites

### Hardware
*   Kinova Gen3 7-DOF Robot Arm
*   Robotiq 2F-140 Gripper
*   Intel RealSense D435i, mounted to the robot with this [camera mount](utils/robotiq_camera_mount.stl)

### Software & Dependencies
*   **OS:** Ubuntu 22.04 or 24.04
*   **ROS 2:** Humble or Jazzy
*   **Kinova Kortex API:** Ensure the C++ Kortex API is installed on your system, and placed in `src/kortex_controller`
*   **ros2_kortex:** The official [ROS 2 packages for Kinova](https://github.com/Kinovarobotics/ros2_kortex). You will need the `kortex_description` package to launch the robot state publisher.
*   **realsense2_camera:** The official [ROS 2 wrapper for Intel RealSense](https://github.com/realsenseai/realsense-ros).
  
*   **Python Dependencies (Conda Environment Recommended):**
    We recommend using a Conda environment (e.g., `kinova-gemini`) to manage the Python dependencies for the Gemini API and image processing.
    ```bash
    conda create -n kinova-gemini python=3.12
    conda activate kinova-gemini
    pip install google-genai python-dotenv pyyaml opencv-python pillow numpy
    ```

## Setup & Configuration

1.  **Clone the Repository:**
    Clone this repository into your ROS 2 workspace `src/` directory.

2.  **API Key Configuration:**
    Create a `.env` file in the root of your workspace (e.g., `~/kinova-gemini/.env`) and add your Gemini API key:
    ```env
    gemini_api_key="YOUR_API_KEY_HERE"
    ```

3.  **Robot Configuration:**
    Ensure the `config.yaml` file in the root of your workspace has the correct robot IP address and default home joint positions.

## Building

Build the workspace using `colcon`. Because the Python nodes run inside a Conda environment, you must update their shebang lines after building so ROS 2 executes them with the correct Python interpreter.

```bash
# Source your ROS 2 installation
source /opt/ros/<distro>/setup.bash

# Build the workspace
colcon build --symlink-install

# Fix the Python shebangs to point to your Conda environment
sed -i '1s|.*|#!/path/to/your/anaconda3/envs/kinova-gemini/bin/python3|' install/gemini_robotics/lib/gemini_robotics/*
```
*Note: Replace `/path/to/your/anaconda3/...` with the actual path to your Conda environment.*

## Running the System

You will need three separate terminals. In each terminal, be sure to source your ROS 2 installation and your workspace overlay, and activate the conda environment:
```bash
source /opt/ros/<distro>/setup.bash
source install/setup.bash
conda activate kinova-gemini
```

**Terminal 1: Robot Description & Camera Transform**
Launch the custom controller, robot state publisher, and RealSense camera.
```bash
ros2 launch kinova_bringup robot.launch.py
```

**Terminal 2: Gemini Brain Node**
Launch the central reasoning node.
```bash
ros2 run gemini_robotics gemini_brain
```

**Terminal 3: User Text Interface**
Launch the interactive CLI to send commands.
```bash
ros2 run gemini_robotics text_interface
```

## Usage Examples
In the Text Interface terminal, you can try commands like:
*   *"Go to the blue toy"*
*   *"Rotate the last joint 30 degrees"*
*   *"Open the gripper slightly"*
