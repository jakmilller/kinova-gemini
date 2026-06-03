# Gemini Integration for Kinova Gen3

This repository integrates a 7 DOF Kinova Gen3 robot arm with Google Gemini for advanced natural language and vision-guided robotics. The system employs a multi-turn reasoning loop, allowing the robot to perform complex, multi-step tasks like "Put the toys away" by calling a variety of custom tools ()

## High-Level Features
*   **Model Intelligence**: Uses a choice Gemini model (currently Gemini 3 Flash) for multi-step reasoning and precise object identification via bounding boxes.
*   **Intelligent Grasping**: The new `move_to_pose` tool combines Gemini's semantic understanding, SAM 2's segmentation, and AnyGrasp's 6D pose estimation to execute complex grasps on unstructured objects.
*   **Advanced Vision**: Integrates **SAM 2 (Segment Anything Model 2)** for high-precision mask refinement and **AnyGrasp** for autonomous 6D grasp detection from point clouds.
*   **Autonomous Grasping**: The new `move_to_pose` tool combines Gemini's semantic understanding, SAM 2's segmentation, and AnyGrasp's 6D pose estimation to execute complex grasps on unstructured objects.
*   **Multi-Turn Interaction**: The robot can ask for clarification, perform intermediate inspection steps, and adjust its plan based on visual feedback.

## Architecture Overview

The system consists of three main components:
1.  **kortex_controller (C++)**: Low-level execution of robot actions via the Kinova Kortex API. Includes a custom **IK Solver** service (`compute_ik`) for 6D pose-to-joint conversion.
2.  **gemini_robotics (Python)**: The central reasoning node. Orchestrates Gemini 1.5 Flash, SAM 2, and AnyGrasp.
3.  **Input Interfaces**:
    *   **Web UI**: A modern web interface for text commands and live chat feedback (including annotated images).
    *   **Voice Interface**: Push-to-talk verbal commands.

For a deep dive into the software design, see [system_architecture.md](system_architecture.md).

## Prerequisites

### Hardware
*   Kinova Gen3 7-DOF Robot Arm
*   Robotiq 2F-140 Gripper
*   Intel RealSense D435i, mounted to the robot with this [camera mount](utils/robotiq_camera_mount.stl)

### Software & Dependencies
*   **OS:** Ubuntu 22.04 or 24.04
*   **ROS 2:** Humble or Jazzy
*   **Python Dependencies (Conda Environment Recommended):**
    ```bash
    conda create -n kinova-gemini python=3.12
    conda activate kinova-gemini
    pip install google-genai python-dotenv pyyaml opencv-python pillow numpy torch torchvision torchaudio
    ```

#### SAM 2 (Segment Anything Model 2) Installation
1.  Clone the SAM 2 repository:
    ```bash
    git clone https://github.com/facebookresearch/segment-anything-2.git
    cd segment-anything-2
    pip install -e .
    ```
2.  Download the checkpoints (e.g., `sam2_hiera_large.pt`) and place them in the `checkpoints/` directory.
3.  Update the `sam2_repo_path` in `gemini_brain_node.py` to match your installation.

#### AnyGrasp SDK Setup
The system expects the AnyGrasp SDK to be located in the root of the workspace:
1.  Place the `anygrasp_sdk` folder in `~/kinova-gemini/anygrasp_sdk`.
2.  Ensure the grasp detection checkpoint is at `anygrasp_sdk/grasp_detection/log/checkpoint_detection.tar`.
3.  Install AnyGrasp dependencies (refer to AnyGrasp documentation for specific CUDA requirements).

## Setup & Configuration

1.  **Clone the Repository:**
    ```bash
    git clone https://github.com/your-repo/kinova-gemini.git ~/kinova-gemini
    ```

2.  **API Key Configuration:**
    Create a `.env` file in the root of your workspace (`~/kinova-gemini/.env`):
    ```env
    gemini_api_key="YOUR_API_KEY_HERE"
    ```

3.  **Pathing in Code:**
    Ensure the `workspace_path` in `gemini_brain_node.py` points to your project root (default is `~/kinova-gemini`).

## Building

```bash
colcon build --symlink-install
# Fix the Python shebangs to point to your Conda environment
sed -i '1s|.*|#!/home/mcrr-lab/anaconda3/envs/kinova-gemini/bin/python3|' install/gemini_robotics/lib/gemini_robotics/*
```

## Running the System

**Terminal 1: Launch Core Components**
```bash
ros2 launch kinova_bringup robot.launch.py
```

**Terminal 2: Gemini Brain Node**
```bash
ros2 run gemini_robotics gemini_brain
```

**Web Interface:**
Open `~/kinova-gemini/web_ui/index.html` in your browser.

## Limitations
*   **Compute Requirements**: Running SAM 2 and AnyGrasp locally requires a modern NVIDIA GPU with at least 8GB of VRAM.
*   **Workspace Constraints**: The `move_to_pose` tool assumes objects are within the reach of the Kinova arm and visible to the RealSense camera.
