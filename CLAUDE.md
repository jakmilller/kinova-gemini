# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

ROS 2 workspace integrating a 7-DOF Kinova Gen3 arm (Robotiq 2F-140 gripper, wrist-mounted Intel RealSense D435i) with Google Gemini for natural-language and vision-guided manipulation. A Gemini Live model reasons over live RGB frames + numerical robot state and sequences calls to a small set of robot tools (`inspect_scene`, `move_to_pose`, `move_to_position`, `adjust_joints`, etc.) to execute multi-step instructions like "put the toys away."

## User-Specific Instructions

- Do not run shell commands yourself (e.g. `colcon build`, `ros2 run`) — leave execution to the user.
- Before making code changes, give a concise explanation of the change in the terminal first. Explanations should teach the underlying software engineering concept, not just describe the diff — the user is using this project to learn.

## Build

```bash
colcon build --symlink-install
# After first build, or whenever scripts change, the installed Python entrypoints need their
# shebang repointed at the conda env (colcon writes the system python3 shebang by default):
sed -i '1s|.*|#!/home/mcrr-lab/anaconda3/envs/kinova-gemini/bin/python3|' install/gemini_live_robotics/lib/gemini_live_robotics/*
```

Conda env `kinova-gemini` (Python 3.12) holds all Python deps: `google-genai`, `torch`, `opencv-python`, `pyaudio`, SAM 2, AnyGrasp, cuRobo. ROS 2 Humble/Jazzy on Ubuntu 22.04/24.04.

## Running

Two terminals, in order:

```bash
# Terminal 1 — hardware bringup: kortex_controller, RealSense, rosbridge (port 9090), web_video_server, RViz
ros2 launch kinova_bringup robot.launch.py

# Terminal 2 — Gemini Live reasoning node
ros2 run gemini_live_robotics gemini_live_brain
```

Then open `web_ui/index.html` in a browser — it talks to ROS over rosbridge (`ws://localhost:9090`) via roslibjs, no separate web server needed.

Requires `.env` at workspace root with `gemini_api_key="..."`, and `config.yaml` for model IDs / joint positions.

## Architecture

Three layers, decreasing abstraction:

```
gemini_live_brain_node.py (Python, async)   — reasoning + vision pipeline
        | ROS 2 actions/services/topics
robot_controller_ros2.py (Python)            — cuRobo motion planning + action client
        | ROS 2 actions/services
controller.cpp (C++)                          — direct Kortex TCP driver
        | Kortex API
Kinova Gen3 hardware
```

**`src/kortex_controller/src/controller.cpp`** — the only process that talks to the robot's Kortex API directly (`mBase`, `mBaseCyclic`, single `mApiMutex` guarding all calls). Hosts action servers `move_to_pose`, `move_to_joints`, `gripper_command`, `grasp_object`, and service `compute_ik`. `move_to_joints` accepts either a single legacy `joint_angles` target or a `trajectory_points` waypoint list (used by cuRobo plans) — see `MoveToJoints.action`. Polls and publishes `robot_state` (custom msg) and `joint_states` at 20 Hz.

**`src/gemini_live_robotics/gemini_live_robotics/robot_controller_ros2.py`** — `KinovaRobotControllerROS2`, the ROS 2 action client wrapping the C++ servers. Owns the cuRobo `MotionGen` planner (GPU, warmed up at startup): `move_to_pose()` first tries to plan a collision-free joint trajectory through cuRobo and dispatch it as waypoints; if cuRobo is unavailable or planning fails it falls back to the legacy single-shot Cartesian `MoveToPose` action. Maintains a static table obstacle plus dynamic per-object obstacles (`update_dynamic_obstacle`, registered by `inspect_scene`) and publishes them to RViz as `MarkerArray` on `/curobo_obstacles`. cuRobo's robot config lives in `config/kinova_gen3.yml`; see `curobo.md` for the integration's original design rationale.

**`src/gemini_live_robotics/gemini_live_robotics/gemini_live_brain_node.py`** — `GeminiLiveBrainNode`, the central async reasoning loop. Connects to Gemini Live (`client.aio.live.connect`), streams camera frames at ~1 fps and forwards user text/tool results as realtime input, and receives audio + transcription + tool calls back. Tool calls are acknowledged immediately and executed as background `asyncio` tasks (`execute_tool_task`) dispatched through `_tool_handlers`, so the model can keep talking while a multi-second move/grasp executes; completion is reported back via `send_realtime_input` as a `SYSTEM EVENT`. Also runs the locally-orchestrated vision pipelines:
  - `inspect_scene()`: Gemini (flash model) returns 2D bounding boxes + labels for the scene (and optionally one `TARGET:`-prefixed region from a natural-language `target_location`), each box is refined into a mask by SAM 2, projected to a 3D pose via depth + `vision_utils`, and reported back to the model; non-target object masks also get their 3D cuboid extents registered as cuRobo obstacles.
  - `move_to_pose()`: full 6D grasp pipeline — Gemini picks the graspable part, SAM 2 segments it, the masked point cloud is handed to AnyGrasp for 6D grasp pose generation, `compute_ik` solves joints for a pre-grasp offset pose, the arm approaches, then slides in along the grasp approach vector and closes the gripper.
  - Gemini's vision calls (`_query_gemini_json`) run in a worker thread and fall back to a regex-based salvage parser (`_salvage_json_objects`) when the model returns malformed JSON — this happens often enough in practice that the fallback isn't optional.
  - `gemini_live_brain_node_old.py` is a previous version kept for reference; it is not registered as a console script.

**`src/ros2_interfaces`** — all custom action/msg/srv definitions: `MoveToPose`, `MoveToJoints` (legacy single-target + waypoint trajectory), `GripperCommand`, `PerceptionTask`, `RobotState` (cartesian pose + 7 joint angles + gripper position), `ComputeIK`.

**`web_ui/`** — vanilla JS + roslibjs talking directly to rosbridge over WebSocket (`ws://localhost:9090`); no backend server. Publishes to `/user_instructions` and `/voice_trigger`, subscribes to `/gemini_chat`, `robot_state`, brain/voice status topics.

**`src/scripts/`** — miscellaneous standalone scripts (not a colcon package, no `package.xml`): a simplified non-cuRobo `robot_controller_ros2.py`, `text_interface_node.py`, `voice_interface_node.py`, `arduino_trigger_node.py`. These overlap with code inside `gemini_live_robotics` and are kept as simpler/legacy references rather than the active path.

**`config.yaml`** — Gemini model selection (`flash` and `live` model IDs) and named joint positions (`home`, `user`). **`config/kinova_gen3.yml`** — cuRobo robot kinematics/collision config. **`src/prompts/system_live_prompt.txt`** — the Gemini Live system instruction.

## Key constraints to know before touching motion code

- All physical Kortex API calls are serialized behind one mutex in `controller.cpp` — any new action/service touching `mBase`/`mBaseCyclic` must take `mApiMutex`.
- Kinova firmware rejects waypoint lists where the first/last waypoint isn't at rest, or that mix per-waypoint duration with max-velocity/blending constraints — `execute_joints` validates with `ValidateWaypointList` before executing for this reason; don't add velocity constraints to waypoints without checking this still passes.
- `robot_controller_ros2.py`'s cuRobo path and the C++ legacy path both exist and are expected to diverge in behavior (cuRobo plans go through the `trajectory_points` field, legacy single-target through `joint_angles`) — code touching `MoveToJoints` needs to handle both.
- AnyGrasp and SAM 2 are optional imports (`ANYGRASP_AVAILABLE`/`SAM2_AVAILABLE` flags) — code paths using them must degrade gracefully when the dependency or its checkpoint isn't present, matching the existing pattern in `gemini_live_brain_node.py`.
