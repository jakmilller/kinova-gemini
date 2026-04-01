# Software Architecture: Gemini Function Calling for Kinova Gen3

## Overview
This document describes the modular system architecture where a user can type natural language instructions into a web interface (or terminal), and the **gemini-robotics-er** model uses **Function Calling** to trigger ROS2 actions and vision-based tasks on the Kinova Gen3 robot.

## Architecture

### 1. Core Package: `gemini_robotics` (Python)
The central "Brain" of the integration.

*   **`gemini_brain_node.py`**:
    *   **Gemini Client**: Manages communication with the Google GenAI SDK (using `gemini-robotics-er`).
    *   **Tool Definitions (Gemini-Callable Functions)**:
        *   `move_to_joints(joint_angles)`: Moves the 7 DOF arm to specific joint angles (in degrees). Used for hardcoded or precisely known configurations.
        *   `set_gripper(position)`: Opens or closes the Robotiq 2F-140 gripper. Position is a value from 0 (fully open) to 100 (fully closed).
        *   `move_to_home()`: A convenience function that commands the robot to return to a predefined safe "home" configuration defined in `config.yaml`.
        *   `move_to_user()`: A convenience function that commands the robot to move to a pre-defined position near the user.
        *   `move_to_object(description)`: Triggers the complete vision-to-action pipeline. It uses the RealSense camera to capture RGB-D images, asks Gemini to segment the object described by the `description` string, calculates the object's 3D coordinate relative to the base, and automatically moves the robot's end-effector to that location.
        *   `adjust_position(direction, amount)`: Adjusts the robot's pose by moving along a Cartesian axis (x, y, or z) by a specified amount in meters.
        *   `adjust_joints(joint_number, amount)`: Adjusts a specific joint angle (1-7) relative to its current position by a specified amount in degrees.
    *   **Function Executor**: Maps Gemini's requested function calls to the ROS 2 Action Client (`KinovaRobotControllerROS2`) or executes the internal Vision Pipeline.
    *   **Instruction Subscriber**: Listens to the `/user_instructions` ROS 2 topic for user commands.
### 2. Input Nodes: Web UI, `voice_interface_node.py` & `text_interface_node.py`
*   **Purpose**: Multiple interfaces capture user input and publish it to the `/user_instructions` topic.
    *   **Web UI**: The primary graphical interface for text-based commands.
    *   **`voice_interface_node.py`**: Enables verbal commands using a push-to-talk mechanism (Spacebar).
    *   **`text_interface_node.py`**: A CLI for manual testing and text input.

### 3. Execution Node: `kortex_controller` (C++)
*   **Role**: Low-level executor hosting ROS 2 Action Servers (`MoveToPose`, `MoveToJoints`, `GripperCommand`). Connects directly to the Kinova robot via the Kortex API over TCP.

---

## Workflow Diagram

```mermaid
graph TD
    subgraph Inputs
        User((User)) -->|Types or Speaks| UI[Input Interfaces: Web UI / Voice / Text]
        Camera[RealSense Camera] -->|RGB & Depth Streams| Brain[gemini_brain_node]
        RobotState[Robot Feedback] -->|Current Pose & Joints| Brain
    end

    subgraph Intelligence
        UI -->|/user_instructions| Brain
        Brain <-->|Prompts & Images <br/> Function Calling| GeminiAPI[Google Gemini API]
        Brain <-->|Pixel-to-3D Math <br/> & TF Transforms| Vision[vision_utils.py]
    end

    subgraph Execution
        Brain -->|ROS 2 Actions| Client[robot_controller_ros2.py]
        Client -->|ROS 2 Actions| Kortex[kortex_controller C++]
        Kortex -->|Hardware API| Robot[Kinova Gen3 Robot]
    end
```

---

## Tool Diagram

This diagram provides a high-level view of the inputs the Gemini Brain receives and the specific tools it can choose from to carry out user instructions. It abstracts away the complex APIs, focusing on the core capabilities.

```mermaid
graph TD
    %% Inputs
    User[User Command<br/>e.g., Speak 'Move to the blue block'] --> Brain((Gemini Brain<br/>AI Reasoning))
    State[Robot State<br/>Current position and status] --> Brain

    %% Tools
...
```

        Brain -.->|Chooses tool based on command| T1[<b>move_to_home</b><br/>Moves arm to a safe starting position]
        Brain -.->|Chooses tool based on command| T2[<b>set_gripper</b><br/>Opens or closes the robot's hand]
        Brain -.->|Chooses tool based on command| T3[<b>move_to_joints</b><br/>Moves the arm to precise angles]
        Brain -.->|Chooses tool based on command| T4[<b>move_to_object</b><br/>Uses camera to find and reach for an object]
        Brain -.->|Chooses tool based on command| T5[<b>move_to_user</b><br/>Moves arm to a position near the user]
        Brain -.->|Chooses tool based on command| T6[<b>adjust_position</b><br/>Adjusts arm pose in Cartesian space]
        Brain -.->|Chooses tool based on command| T7[<b>adjust_joints</b><br/>Adjusts specific joint angles]
    end
```

---

## Execution Flow Examples

### Example 1: Simple Movement
1.  **User**: "Move to home and open the gripper."
2.  **Gemini**: Analyzes the request and sequentially calls `move_to_home()` and `set_gripper(position: 0)`.
3.  **Executor**: Triggers the respective robot actions via the `KinovaRobotControllerROS2` action client.

### Example 2: Vision-Guided Task
1.  **User**: "Move the arm over to the blue block."
2.  **Gemini**: Recognizes a spatial target and calls `move_to_object(description: 'blue block')`.
3.  **Vision Pipeline**:
    *   Waits for and captures synchronized RGB and Aligned Depth frames from the RealSense topics.
    *   Sends the RGB image and a prompt requesting a segmentation mask for 'blue block' to the Gemini API.
    *   Gemini returns the segmentation mask coordinates as JSON.
    *   The pipeline overlays the mask on the depth image and calculates the **median depth** of the segmented object to ignore noise or outliers.
    *   Calculates the centroid pixel $(u, v)$ of the mask.
    *   Uses RealSense camera intrinsics (focal length, principal point) to project the pixel and depth into a 3D coordinate $(X, Y, Z)$ in the `realsense_link` frame.
    *   Uses ROS 2 TF2 (`tf_buffer.lookup_transform`) to transform the 3D coordinate from `realsense_link` to the robot's `base_link`.
    *   Sends a `MoveToPose` action goal to the controller with the new base coordinates, maintaining the current end-effector orientation.
4.  **Executor**: Triggers the robot motion to the object.

---

## Vision Integration
**"Tool-Based Vision" Approach**
To minimize latency and token usage, we do **not** send a continuous video feed to the model. Vision is treated as a tool, invoked on-demand when spatial understanding is required.

*   **Model**: `gemini-robotics-er` (Handles both logical reasoning and 2D spatial extraction).
*   **Workflow**:
    1.  **Capture**: `gemini_brain_node` pulls the latest frame from the `/camera/...` topics.
    2.  **Query**: The image is sent to Gemini with a prompt: *"Return segmentation masks for: [description]."*.
    3.  **Mask Processing**: The JSON response is decoded into a bitmask and resized to match the original image resolution. 
    4.  **Depth Lookup**: The depth values corresponding to the "True" pixels of the mask are extracted from the aligned depth image, and the median depth is computed.
    5.  **Projection & Transform**: The 2D centroid and median depth are projected into 3D space using the camera's intrinsic parameters (dynamically subscribed via `CameraInfo`). The point is then transformed to the robot's base frame using the static TF tree.
    6.  **Action**: A movement command is dispatched autonomously by the pipeline using the calculated coordinates.