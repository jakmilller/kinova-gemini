# Kinova Gen3 × Gemini Live

Natural-language, vision-guided manipulation for a 7-DOF **Kinova Gen3** arm (Robotiq 2F-140
gripper, wrist-mounted Intel RealSense D435i) driven by **Google Gemini Live**. You speak a
command — "put the toys away", "pick up the red cup" — and a Gemini Live model reasons over the
live camera feed and sequences calls to a small set of robot tools to carry it out, talking back
out loud as it works.

**Push-to-talk Voice Interface:** hold a button (a physical Arduino
button or the web UI's press-and-hold mic), speak, and release. The clip is transcribed locally
(faster-whisper) and fed to the model as a text instruction. A web UI text box is the backup input.

Motion goes **directly through the Kinova Kortex API**. Static-obstacle safety (table/wall/ceiling)
comes from **Kortex Protection Zones** configured into the arm's firmware.

---

## Architecture

```
gemini_live_brain_node.py (Python, async)   reasoning + vision pipeline
  ├─ perception.py   Gemini box detection · SAM2 masks · depth→3D projection
  ├─ grasp.py        gripper-width mapping · AnyGrasp cloud + pose math
  └─ robot_controller_ros2.py   ROS 2 action client
        │  ROS 2 actions / topics
controller.cpp (C++)   direct Kortex TCP driver (single API mutex)
        │  Kortex API
Kinova Gen3 hardware
```

**Robot tools** the model can call: `inspect_scene`, `move_to_position`, `move_relative`,
`grasp_simple_object` (top-down pick at a known point), `grasp_complex_object` (6D AnyGrasp
pick), `adjust_gripper` (open/close/custom width), `adjust_joints`, `move_to_home`,
`move_to_user`, `stop_robot`.

- **`inspect_scene`** — Gemini returns 2D boxes + labels, SAM 2 refines each into a mask, depth +
  TF lift it into a 3D base-frame pose (with a PCA width estimate), reported back to the model.
- **`grasp_complex_object`** — Gemini picks the graspable part, SAM 2 segments it, the masked
  point cloud goes to AnyGrasp for a 6D grasp; the arm moves directly to it and closes.

## Repository layout

| Path | What it is |
|------|-----------|
| `src/gemini_live_robotics/` | Reasoning node, perception/grasp modules, robot controller client, and the packaged voice/arduino nodes |
| `src/kortex_controller/` | C++ Kortex driver: Cartesian/joint action servers, gripper, robot-state telemetry, Protection Zones |
| `src/ros2_interfaces/` | Custom actions (`MoveToPose`, `MoveToJoints`, `GripperCommand`) and `RobotState` msg |
| `src/kinova_bringup/` | `robot.launch.py` (single-command bringup) and the RViz config |
| `src/prompts/system_live_prompt.txt` | Gemini Live system instruction |
| `src/arduino/voice_button/` | Firmware for the push-to-talk button |
| `web_ui/` | Vanilla-JS + roslibjs front-end (talks to rosbridge, no backend server) |
| `config.yaml` | Model IDs, transcription settings, named joint positions, static obstacles |

---

## Prerequisites

**Hardware:** Kinova Gen3 7-DOF arm · Robotiq 2F-140 gripper · Intel RealSense D435i (wrist-mounted).

**Software:** Ubuntu 22.04/24.04 · ROS 2 Humble/Jazzy · an NVIDIA GPU with ≥8 GB VRAM (for local
SAM 2 / AnyGrasp / whisper).

**Conda environment** (holds all Python deps):

```bash
conda create -n kinova-gemini python=3.12
conda activate kinova-gemini
pip install google-genai python-dotenv pyyaml opencv-python pillow numpy \
            torch torchvision torchaudio pyaudio faster-whisper pyserial scipy
```

**[SAM 2](https://github.com/facebookresearch/sam2)** — clone the streaming SAM2 repo and download the tiny checkpoint. The node expects the
repo at the path in `SAM2_REPO_PATH` (top of `gemini_live_brain_node.py`) with the checkpoint at
`checkpoints/sam2/sam2_hiera_tiny.pt`. Adjust `SAM2_REPO_PATH` to match your install.

**[AnyGrasp](https://github.com/graspnet/anygrasp_sdk)** — place the SDK at `~/kinova-gemini/anygrasp_sdk` with the detection checkpoint at
`anygrasp_sdk/grasp_detection/log/checkpoint_detection.tar` (see AnyGrasp docs for its CUDA setup).

**[Kortex C++ API](https://github.com/Kinovarobotics/Kinova-kortex2_Gen3_G3L)** — place under `src/kortex_controller/kortex_api` (gitignored).

## Configuration

Create `.env` at the workspace root:

```env
gemini_api_key="YOUR_API_KEY_HERE"
```

Edit `config.yaml` for model IDs, transcription backend (`whisper` local vs `gemini` cloud),
named `home`/`user` joint positions, and `static_obstacles` (Protection Zone boxes in `base_link`
coordinates — `kortex_controller` pushes these into firmware on every startup).

## Build

```bash
colcon build --symlink-install
# optional - in case colcon writes the system python3 shebang into the installed entry points, repoint them at the conda env:
sed -i '1s|.*|#!/home/mcrr-lab/anaconda3/envs/kinova-gemini/bin/python3|' install/gemini_live_robotics/lib/gemini_live_robotics/*
```

## Run

**Terminal 1 — hardware + interfaces** (controller, RealSense, rosbridge on :9090, web_video_server, RViz):

```bash
ros2 launch kinova_bringup robot.launch.py
```

Feature toggles (all optional):

```bash
ros2 launch kinova_bringup robot.launch.py rviz:=false voice:=true arduino:=true
```

| Flag | Default | Effect |
|------|---------|--------|
| `rviz` | `true` | Launch RViz |
| `voice` | `false` | Launch the push-to-talk voice interface node |
| `arduino` | `false` | Launch the physical Arduino button bridge |

**Terminal 2 — Gemini Live reasoning node:**

```bash
ros2 run gemini_live_robotics gemini_live_brain
```

**Web UI:** open `web_ui/index.html` in a browser — it talks to ROS over rosbridge
(`ws://localhost:9090`) via roslibjs, no separate web server needed. The mic button is
**press-and-hold**: hold, speak, release. The text box is the backup input path.

---

## Notes & limitations

- **Compute:** SAM 2 + AnyGrasp + whisper run locally and want ≥8 GB VRAM. Drop the whisper model
  to `tiny.en` or `whisper_device: cpu` in `config.yaml` if VRAM is tight.
- **Move speed** is a firmware soft limit set in the Kinova Web App, not in this repo.
- **No collision-aware planning:** each move is a single direct Kortex command, protected only by
  Kortex's self-collision/singularity avoidance and any configured Protection Zones. Future work will implement more advanced manipulation strategies
