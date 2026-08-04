# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

ROS 2 workspace integrating a 7-DOF Kinova Gen3 arm (Robotiq 2F-140 gripper, wrist-mounted Intel RealSense D435i) with Google Gemini for natural-language and vision-guided manipulation. A Gemini Live model reasons over live RGB frames and sequences calls to a small set of robot tools (`inspect_scene`, `move_to_position`, `move_relative`, `grasp_simple_object`, `grasp_complex_object`, `adjust_gripper`, `adjust_joints`, `move_to_home`, `move_to_user`, `stop_robot`) to execute multi-step instructions like "put the toys away."

**Voice is the primary interface, and it is push-to-talk**: the user holds a button (a physical Arduino button or the web UI's press-and-hold mic button), speaks a command, and releases; the recorded clip is transcribed locally (faster-whisper) and fed to the reasoning model as a text instruction (see the Voice path section below). Gemini Live does reasoning and text-to-speech natively — the robot talks back out loud — but speech-to-*input* is a deliberate, held-button action rather than always-on listening. The web UI's text box is a backup input path.

**The model's only live sense is the camera** — no numeric robot state is streamed to it. It reads coordinates and widths back from `inspect_scene` results (delivered as tool-result text), not from a continuous state feed.

Motion goes directly through the Kinova Kortex API — no MoveIt, no cuRobo, no sampling motion planner anywhere in this repo. Every move is a single deterministic Cartesian or joint-space command (`Base::ExecuteAction`), which gets Kortex's built-in self-collision and singularity avoidance for free. Static-obstacle safety (table/wall/ceiling) comes from Kortex **Protection Zones**, configured once into the arm's firmware rather than checked per-motion in software — there is no per-object dynamic obstacle tracking and no collision-aware trajectory planning; grasps are barebones (move TCP to target coordinates, close gripper).

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

Conda env `kinova-gemini` (Python 3.12) holds all Python deps: `google-genai`, `torch`, `opencv-python`, `pyaudio`, `faster-whisper` (local push-to-talk transcription), `pyserial` (Arduino button), SAM 2, AnyGrasp. ROS 2 Humble/Jazzy on Ubuntu 22.04/24.04.

## Running

The bringup launch file can start everything, so a minimal run is two terminals:

```bash
# Terminal 1 — hardware + interfaces: kortex_controller, RealSense, rosbridge (port 9090),
# web_video_server, RViz. Add voice:=true (and arduino:=true for the physical button) to also
# start the push-to-talk input nodes from this one launch.
ros2 launch kinova_bringup robot.launch.py voice:=true

# Terminal 2 — Gemini Live reasoning node
ros2 run gemini_live_robotics gemini_live_brain
```

**Launch flags** (`robot.launch.py`): `rviz` (default true), `voice` (default false), `arduino` (default false). e.g. `ros2 launch kinova_bringup robot.launch.py rviz:=false voice:=true`. The voice/arduino nodes are now packaged entry points in `gemini_live_robotics`, so they can also be run standalone with `ros2 run gemini_live_robotics voice_interface` / `ros2 run gemini_live_robotics arduino_trigger`. The web UI's mic button and the physical Arduino button both drive `/voice_trigger` interchangeably; the physical button needs the Arduino flashed with `src/arduino/voice_button/voice_button.ino` (set `BUTTON_PIN` to match your wiring).

Then open `web_ui/index.html` in a browser — it talks to ROS over rosbridge (`ws://localhost:9090`) via roslibjs, no separate web server needed.

Requires `.env` at workspace root with `gemini_api_key="..."`, and `config.yaml` for model IDs / transcription settings / joint positions / static obstacle boxes.

`kortex_controller` pushes `config.yaml`'s `static_obstacles` into the arm's firmware as Kortex Protection Zones on every startup (`Controller::configureProtectionZonesFromConfig`, called from the constructor, delete-by-name then recreate so it can't accumulate duplicates) — add/remove/edit boxes in `config.yaml` and relaunch `robot.launch.py` to apply, no separate setup script. It also reads the zones back from firmware once at startup (a second, independent read, so it catches a rejected/malformed zone), logs each one, and publishes them as a wireframe `visualization_msgs/MarkerArray` on `/protection_zones` (transient-local durability, so a late-starting RViz still sees them) — already wired into `robot.rviz`'s "ProtectionZones" display, `base_link` frame, axis-aligned boxes only.

## Architecture

Two layers:

```
gemini_live_brain_node.py (Python, async)   — reasoning + vision pipeline (orchestrator)
        | uses perception.py, grasp.py, robot_controller_ros2.py
        | ROS 2 actions/services/topics
controller.cpp (C++)                          — direct Kortex TCP driver
        | Kortex API
Kinova Gen3 hardware
```

**`src/kortex_controller/src/controller.cpp`** — the only process that talks to the robot's Kortex API directly (`mBase`, `mBaseCyclic`, single `mApiMutex` guarding all calls). Hosts action servers `move_to_pose` (single Cartesian fingertip target, reached via `ComputeInverseKinematics` + `reach_joint_angles`), `move_linear` (relative straight-line move along the tool's +Z, `reach_pose`), `move_to_joints` (single joint-space target, `reach_joint_angles` — no multi-waypoint trajectory support), `gripper_command` (waits on measured finger speed rather than a fixed sleep), and `grasp_object` (a thin alias of `gripper_command`), plus the `compute_ik` **service** (`ComputeIK.srv`) — a read-only "what joints reach this fingertip pose, at this gripper opening?" lookup used to rank and reachability-filter grasp candidates before the arm commits to a move. Polls and publishes `robot_state` (custom msg) and `joint_states` at 20 Hz. The two Cartesian actions share one completion loop, the templated `Controller::pollUntilCartesianTarget` in `controller.hpp`, which polls in **raw firmware TCP space** — callers taking a fingertip goal must convert before calling it.

**Move speed is configured in the Kinova Web App, not in this repo.** The arm's Cartesian speed comes from its `CARTESIAN_TRAJECTORY` kinematic *soft limits*, which live in firmware and are edited through the Web App. Soft limits are per control mode, so `reach_pose` (Cartesian moves) and `reach_joint_angles` (preset joint moves, `ANGULAR_TRAJECTORY`) are governed independently — one being slow while the other is fast is expected and is a Web App setting, not a bug in this code. Firmware clamps an over-large commanded speed **silently**, so "I raised the number and nothing changed" means the soft limit is the binding constraint. To read the real values, `ControlConfig::GetKinematicHardLimits()` / `GetKinematicSoftLimits(mode)` return ground truth; the codebase has no `ControlConfigClient`, so add one temporarily if you need to check.

A `Speed` constraint on `reach_pose` (`execute_pose`) is only ever a ceiling *below* the soft limit — it can never make a move faster than the Web App allows. `execute_joints` sets no constraint at all, which is why it inherits the firmware default. `execute_pose` attaches one only when a goal passes `speed_scaling > 0`, i.e. to make a single move deliberately slower (nothing currently does; `robot_controller_ros2.py`'s `move_to_pose` defaults to `speed=0.0`, meaning unconstrained). Kortex applies translation and orientation ceilings independently and a constrained move takes the *longer* of the two times, so a rotation-heavy move runs on the orientation clock.

**`src/gemini_live_robotics/gemini_live_robotics/robot_controller_ros2.py`** — `KinovaRobotControllerROS2`, the ROS 2 action client wrapping the C++ servers. Every method (`move_to_pose`, `move_to_joints`, `move_to_home`, `move_to_user`, `set_gripper`, `grasp_object`, `stop_robot`) is `async def` and awaits an action-client round trip via `_await_rclpy_future` (bridges `rclpy` futures into `asyncio`). `get_joint_angles()` returns the latest `robot_state` joint angles.

**`src/gemini_live_robotics/gemini_live_robotics/gemini_live_brain_node.py`** — `GeminiLiveBrainNode`, the central async reasoning loop and **orchestrator**. It holds all node state (sensor frames, robot state, the Gemini client, the SAM2/AnyGrasp models, the tool dispatch table) and wires the stateless helpers in `perception.py` / `grasp.py` to the robot controller. It connects to Gemini Live (`client.aio.live.connect`), streams camera frames at ~1 fps as the model's only live sense, and forwards user text/tool results as realtime input; it receives audio + transcription + tool calls back. Tool calls are acknowledged immediately and executed as background `asyncio` tasks (`execute_tool_task`) dispatched through `_tool_handlers`, so the model can keep talking while a multi-second move/grasp executes; completion is reported back via `send_realtime_input` as a `SYSTEM EVENT`. The node also runs the locally-orchestrated vision pipelines:
  - `inspect_scene()`: Gemini (flash model) returns 2D bounding boxes + labels + a semantic `depth_cm` thickness estimate for the scene (and optionally one `TARGET:`-prefixed region from a natural-language `target_location`); each box is refined into a mask by SAM 2, projected to a 3D base-frame pose via depth + `perception.py`'s projection helpers, given a PCA-based width estimate, and reported back to the model. The reported pose is a point on the object's camera-facing surface (median depth over the mask), and the grasp standoff and insertion are both measured from it.
  - `_grasp_simple()` (`grasp_simple_object` tool): pick at a known surface x/y/z — pre-shape the gripper to the object width, move to a pre-grasp pose `grasp.GRASP_STANDOFF_M` back along the gripper's own +Z, `move_linear` straight in by `standoff + insertion`, then close. `insertion` is half the object's depth (so the fingertips end at its middle), clamped to `grasp.MAX_INSERTION_M`, falling back to width when no depth estimate arrived. Works at any wrist orientation, not just top-down, since the approach axis is the tool's +Z. No fallbacks.
  - `_grasp_anygrasp()` (`grasp_complex_object` tool): for a named object, Gemini picks the graspable part, SAM 2 segments it, and the masked point cloud (`grasp.build_anygrasp_clouds`) goes to AnyGrasp for 6D grasp generation. Candidates are then **ranked, not just top-scored** (`_select_anygrasp_candidate`): the top `grasp.ANYGRASP_CANDIDATE_POOLS[0]` by confidence, each expanded into its two equivalent wrist rolls via `grasp.flip_tool_roll` (a parallel gripper cannot distinguish a 180° roll about its own +Z, so offering both stops the arm spinning the wrist for nothing), filtered by whether the `compute_ik` service solves **both** the pre-grasp and the grasp pose, and won by the lowest `grasp.joint_motion_cost`. The pool widens once if nothing is reachable, then fails. The winner is then executed with the same pre-shape → pre-grasp → `move_linear` → close sequence as the simple path. Every failure logs and returns a diagnostic detail string.
  - **AnyGrasp's `translation` is the grasp contact centre, not the fingertip position** — per `graspnetAPI`'s own gripper model the fingertips sit `grasp.depth` (1–4 cm) further along the approach axis, and no standoff is included. So the tip target is `translation + depth · approach`, and the pre-grasp is `GRASP_STANDOFF_M` back from *that*. Because `grasp.anygrasp_grasp_to_base_pose` applies `R_ALIGN` (AnyGrasp's local +X → tool +Z), the approach axis is just the tool +Z of the returned euler angles and both offsets are plain `grasp.shift_along_tool_z` calls. Unlike the simple path there is no extra insertion to add: AnyGrasp's `depth` already is the insertion.

**`perception.py`** and **`grasp.py`** are the stateless halves of the node's vision/grasp math, extracted as plain module functions that take their dependencies (Gemini client, SAM2 predictor, camera_info, transforms) as explicit arguments — so they carry no ROS-node state and are testable in isolation:
  - `perception.py`: `query_gemini_json` (worker-thread Gemini vision call with a regex-based salvage parser for malformed JSON — the fallback isn't optional, the model returns bad JSON often enough), `iter_scene_items`/`normalize_item`/`salvage_json_objects` (tolerant box/label/depth extraction), `run_sam2` (box-prompt SAM2 → mask), `mask_centroid`, `load_annotation_font`, `estimate_object_width` (PCA footprint width), `transform_to_matrix`, `get_3d_point_from_pixel`, `transform_point`.
  - `grasp.py`: `GRIPPER_MAX_WIDTH_M`/`GRIPPER_FULLY_CLOSED_THRESHOLD`/`R_ALIGN`/`GRASP_STANDOFF_M`/`MAX_INSERTION_M`/`ANYGRASP_CANDIDATE_POOLS`/`JOINT_MOTION_WEIGHTS` constants, `width_to_gripper_percent`, `tool_z_axis`/`shift_along_tool_z` (approach-axis geometry, mirroring `Controller::toolZAxis` — the euler convention must match on both sides), `insertion_depth_m`, `flip_tool_roll`/`joint_motion_cost` (AnyGrasp candidate ranking), `build_anygrasp_clouds`, `anygrasp_grasp_to_base_pose`.

  **Voice path** (`run_live_session` gathers four tasks — video, send-text, speaker, receive):
  - Voice input is **push-to-talk and lives in a separate node**, `voice_interface_node.py` (packaged in `gemini_live_robotics`, entry point `voice_interface`). It records 16kHz mono PCM while `/voice_trigger` is True (button held), and on release transcribes the clip and publishes the text to `/user_instructions` — so a spoken command enters the brain node identically to a typed one, via `instruction_callback`. There is no always-on mic streaming and no server-side VAD; the brain node never opens an input stream.
  - **Transcription is local faster-whisper by default** (GPU, `base.en`, ~100–300ms — no network round trip), configured under `config.yaml`'s `transcription:` block (`backend`, `whisper_model`, `whisper_device`, `whisper_compute_type`). The model is loaded + warmed up once at node startup (`_load_whisper`), never per call. It degrades gracefully: `cuda` → `cpu` → the cloud Gemini fallback (`model.transcribe`, default `gemini-2.5-flash-lite`, which wraps the PCM in a WAV via `_frames_to_wav`). faster-whisper is an optional import (`WHISPER_AVAILABLE`), matching the SAM2/AnyGrasp pattern. On an 8GB GPU shared with SAM2/AnyGrasp, drop to `tiny.en` or `whisper_device: cpu` if VRAM is tight.
  - `play_speaker_task` plays Gemini's 24kHz PCM replies (the robot talks back), and `output_audio_transcription` → `role="model"` chat bubbles. The Live config keeps `response_modalities=["AUDIO"]` + `output_audio_transcription` only.
  - `/voice_trigger` (Bool: True=start recording, False=stop+transcribe) is driven by `arduino_trigger_node.py` (physical button, entry point `arduino_trigger`) and the web UI's press-and-hold mic button, interchangeably. `/voice_status` (`IDLE`/`LISTENING`/`PROCESSING`) is published by `voice_interface_node` and drives the UI indicator.

**`src/ros2_interfaces`** — custom action/msg definitions: `MoveToPose`, `MoveToJoints` (single joint-angle target only), `GripperCommand`, `RobotState` (cartesian pose + 7 joint angles + gripper position).

**`web_ui/`** — vanilla JS + roslibjs talking directly to rosbridge over WebSocket (`ws://localhost:9090`); no backend server. Publishes to `/user_instructions` (text box) and `/voice_trigger` (the mic button, **press-and-hold**: True on mousedown/touchstart, False on mouseup/touchend, mirroring the physical button). Subscribes to `/gemini_chat`, `robot_state`, and `/voice_status` (drives the mic indicator). The chat log renders the conversation from `/gemini_chat`.

**`src/arduino/voice_button/voice_button.ino`** — firmware for the push-to-talk button. Streams `1` while held / `0` while released over USB serial at 115200 baud; `arduino_trigger_node.py` edge-detects this into `/voice_trigger`. Set `BUTTON_PIN` to match your wiring before flashing.

**`config.yaml`** — Gemini model selection (`flash`, `live`, `transcribe` IDs), `transcription` (whisper backend/model/device/compute type), `audio` (mic `input_device` override), named joint positions (`home`, `user`), and `static_obstacles` (table/wall/ceiling boxes as `{size: [x,y,z], center: [x,y,z]}` in `base_link` coordinates — read directly by `kortex_controller` at startup, see above; not read by the Gemini Live node). **`src/prompts/system_live_prompt.txt`** — the Gemini Live system instruction.

## Key constraints to know before touching motion code

- All physical Kortex API calls are serialized behind one mutex in `controller.cpp` — any new action/service touching `mBase`/`mBaseCyclic` must take `mApiMutex`.
- `move_to_joints` only accepts a single joint-angle target (`goal->joint_angles`) — there is no multi-waypoint trajectory support in this codebase.
- Protection Zones live in firmware and are independent of this codebase at runtime once pushed — they never appear in `robot_state`, and nothing consumes `/protection_zones` for planning (it's a read-only mirror for logging/RViz, published by `Controller::publishProtectionZones`). They're pushed from `config.yaml`'s `static_obstacles` by `Controller::configureProtectionZonesFromConfig` every time `kortex_controller` starts (delete-by-name then recreate). If a move behaves unexpectedly near a configured zone boundary, check the zones directly (`base.ReadAllProtectionZones()`) or the controller's startup log rather than assuming the issue is in this code.
- **`move_to_pose` coordinates are fingertip coordinates, and `kortex_controller` makes that true rather than assuming it.** The firmware tool offset (set in the Web App, `config.yaml`'s `gripper.tcp_open_m`) sits at the fingertip *with the gripper fully open*; because the 2F-140's fingers swing forward as they close, the real tips reach up to `tcp_closed_m - tcp_open_m` (~3 cm) further. Kortex has no concept of this — its tool frame is a fixed transform. So `Controller::shiftAlongToolZ` is applied twice, in opposite directions: `execute_pose` subtracts the current extension before solving IK, and `publishState` adds it so `robot_state` reports where the fingers actually are. Commands and feedback therefore share one frame (a read-modify-write like `move_relative` stays consistent for free), and **`robot_state` deliberately diverges from the Cartesian pose shown in the Kinova Web App** by up to 3 cm. Edit the two anchors in `config.yaml` to recalibrate — no rebuild.
- **`move_to_pose` guarantees the endpoint; `move_linear` guarantees the path.** `move_to_pose` solves IK and moves through joint space, so its Cartesian path is not a straight line (and `speed_scaling` is ignored). `move_linear` (`Controller::execute_linear`) is the opposite: a relative signed distance along the tool's own +Z, executed with Kortex `reach_pose` so the TCP travels in a straight line at a bounded speed with the orientation held. That orientation lock is what made `reach_pose` unusable for long transit moves, but it is exactly right for a short grasp approach. Unlike `execute_pose`, it has an execution timeout and aborts rather than hanging.
- AnyGrasp and SAM 2 are optional imports (`ANYGRASP_AVAILABLE`/`SAM2_AVAILABLE` flags) — code paths using them must degrade gracefully when the dependency or its checkpoint isn't present, matching the existing pattern in `gemini_live_brain_node.py`. faster-whisper follows the same optional-import pattern in `voice_interface_node.py`.
