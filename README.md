# Gemini Integration for Kinova Gen3

This repository integrates a 7 DOF Kinova Gen3 robot arm with Google's Gemini models for advanced natural language and vision-guided robotics. The system employs a multi-turn reasoning loop, allowing the robot to perform complex, multi-step tasks like "Pick up the toy and put it in the box."

## High-Level Features
*   **Model Intelligence**: Uses **Gemini 1.5 Robotics ER** for multi-step reasoning and high-precision visual object segmentation.
*   **Embodied Awareness**: The model receives live RGB images from an arm-mounted RealSense camera and spatial "third-person" context via RViz snapshots.
*   **Multi-Turn Interaction**: The robot can ask for clarification, perform intermediate inspection steps, and adjust its plan based on visual feedback.

## Architecture Overview

The system consists of three main components:
1.  **kortex_controller (C++)**: Low-level execution of robot actions via the Kinova Kortex API.
2.  **gemini_robotics (Python)**: The central reasoning node. Orchestrates the vision pipeline, manages tool-calling logic, and maintains the conversational state with Gemini.
3.  **Input Interfaces**:
    *   **Web UI**: A modern web interface for text commands and live chat feedback (including annotated images).
    *   **Voice Interface**: Push-to-talk verbal commands (Spacebar).

For a deep dive into the software design, see [system_architecture.md](system_architecture.md).

## Usage Examples
The robot is now capable of multi-step tasks:
*   *"Put the red toy in the blue bin."*
*   *"Move the apple between the banana and the orange."*
*   *"Give me the toy on the left."*
*   *"Clear the table by moving everything to the box."*

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

Build the workspace using `colcon`.

```bash
# Source your ROS 2 installation
source /opt/ros/<distro>/setup.bash

# Build the workspace
colcon build --symlink-install
```

### Troubleshooting: Python Shebangs
Because the Python nodes run inside a Conda environment, you may need to update their shebang lines after building so ROS 2 executes them with the correct Python interpreter. If you encounter module import errors (like `google-genai` not found), run the following to point the scripts to your Conda environment:

```bash
# Fix the Python shebangs to point to your Conda environment
sed -i '1s|.*|#!/path/to/your/anaconda3/envs/kinova-gemini/bin/python3|' install/gemini_robotics/lib/gemini_robotics/*
```
*Note: Replace `/path/to/your/anaconda3/...` with the actual path to your Conda environment.*

## Running the System

You will need multiple separate terminals. In each terminal, be sure to source your ROS 2 installation and your workspace overlay, and activate the conda environment:
```bash
source /opt/ros/<distro>/setup.bash
source install/setup.bash
conda activate kinova-gemini
```

**Terminal 1: Launch Core Components**
Launch the custom controller, robot state publisher, RealSense camera, and UI dependencies.
```bash
ros2 launch kinova_bringup robot.launch.py
```

**Terminal 2: Gemini Brain Node**
Launch the central reasoning node.
```bash
ros2 run gemini_robotics gemini_brain
```

**Terminal 3 (optional): Voice Interface**
Launch the voice command node.
```bash
ros2 run gemini_robotics voice_interface
```

**Web Interface:**
Open `/path/to/ws/kinova-gemini/web_ui/index.html` in your browser.
*Hold the **Spacebar** to record a voice command, or use the text interface.*

## Usage Examples
In the Web Interface, you can try commands like:
*   *"Put the red toy in the blue bin."*
*   *"Move the apple between the banana and the orange."*
*   *"Give me the toy on the left."*
*   *"Clear the table by moving everything to the box."*


## Limitations
Manipulation has been simplified by keeping the current end-effector orientation before and after the `move_to_position` call (think arcade claw machine). We are currently working on reasoning about the correct object affordances (i.e. picking up a box by the side), as well as adjusting the orientation of the end-effector intelligently.
