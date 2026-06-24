# Motion Planning: From `inspect_scene` to a Moving Arm

This document traces exactly what happens, end to end, when the Gemini brain calls
`move_to_position` (or the grasp pipeline's `move_to_pose`) after `inspect_scene` has located
an object. It explains `config/kinova_gen3.yml`'s `cspace` section in detail, how cuRobo decides
waypoint count and timing, how `controller.cpp`'s waypoint executor works and what safety nets
exist, and which Kinova Kortex "control mode" applies to our waypoint trajectories — including
where that mapping is genuinely uncertain and needs verification against Kinova's own docs.

---

## 1. The full call chain

```
Gemini tool call (move_to_position / move_to_pose)
        |
        v
gemini_live_brain_node.py: _handle_move_to_position()          [line 395]
        |  reads current TCP pose, fills in x/y/z, calls controller.move_to_pose()
        v
robot_controller_ros2.py: KinovaRobotControllerROS2.move_to_pose()   [line 340]
        |  1. converts goal (x,y,z,theta) -> end_effector_link target pose
        |     (subtracting the gripper's current TCP offset, since cuRobo solves
        |      IK for the URDF's end_effector_link, not the fingertip)
        |  2. builds a cuRobo JointState from the robot's *current* joint angles
        |  3. self.planner.plan_single(start_state, goal_pose, MotionGenPlanConfig(timeout=5.0))
        |        -> cuRobo runs IK + graph search + trajectory optimization on GPU
        |  4. on success: result.get_interpolated_plan() -> dense list of joint
        |     positions, spaced result.interpolation_dt seconds apart
        |  5. packs these into trajectory_points (JointTrajectoryPoint[]), each with
        |     positions (rad) and time_from_start = i * interpolation_dt
        |  6. sends a MoveToJoints action goal with trajectory_points populated
        v
ros2_interfaces/action/MoveToJoints.action
        |  trajectory_points: trajectory_msgs/JointTrajectoryPoint[]
        v
controller.cpp: Controller::execute_joints()                    [line 214]
        |  is_trajectory = true (trajectory_points is non-empty)
        |  1. builds a Kortex WaypointList of AngularWaypoint objects (degrees),
        |     one per cuRobo trajectory point, each with a per-segment `duration`
        |     derived from time_from_start deltas
        |  2. mBase->ValidateWaypointList(waypoint_list)  <-- firmware-side check
        |     against the robot's actuator speed/accel limits; aborts the goal
        |     locally (without ever touching the motors) if it reports errors
        |  3. mBase->ExecuteWaypointTrajectory(waypoint_list)  <-- this is the
        |     actual motor command
        |  4. waits on a Kortex ACTION_END / ACTION_ABORT notification (up to 60s)
        v
Kinova Gen3 firmware executes the waypoint trajectory natively
```

`inspect_scene` (vision/grasp pipeline) only determines the *target pose*; it never touches
motion planning directly. Once a target pose or joint target exists, every motion in this
codebase funnels through `move_to_pose()` (cuRobo path) or `move_to_joints()` (point-to-point,
used by `move_to_home`/`move_to_user`/`adjust_joints`, which send a single `joint_angles` target
and skip cuRobo and the waypoint path entirely — see `is_trajectory` in `execute_joints`).

---

## 2. `config/kinova_gen3.yml` — what cuRobo actually does with it

cuRobo's `MotionGenConfig.load_from_robot_config()` (called once at node startup in
`robot_controller_ros2.py:107`) parses this file into the kinematic model, collision model, and
the trajectory optimizer's joint-space constraints. Everything under `cspace` (lines 105–129)
directly bounds the joint-space trajectory cuRobo is allowed to produce.

### `joint_names`
The 7 actuated DOFs cuRobo plans over, in the order the URDF/cspace vectors below are indexed.
This must match the order `_current_joints_rad()` in `robot_controller_ros2.py` reports state in
(joint_1..joint_7), since that's what seeds `plan_single`'s start state.

### `retract_config`
The "home"/seed joint configuration cuRobo falls back to for things like null-space optimization
and warmup. Not directly relevant to per-call planning, but it's the pose cuRobo warms up its
CUDA graphs against at startup (`self.planner.warmup()`), so an unreachable or singular
`retract_config` can produce misleading warmup timing/behavior.

### `null_space_weight` / `cspace_distance_weight`
Per-joint weights used inside the optimizer's cost function:
- `cspace_distance_weight` penalizes how far a candidate trajectory point's joint configuration
  is from the *previous* point (a kind of "prefer smooth, minimal joint motion" cost). All 1.0
  here means no joint is preferentially favored or penalized for moving — uniform cost per
  radian moved, for every joint.
- `null_space_weight` only matters when the IK solution is redundant (a 7-DOF arm has 1 extra
  DOF beyond a 6D pose target) — it biases which of the infinitely many joint-space solutions
  that reach the same Cartesian pose cuRobo prefers, pulling toward `retract_config`. Uniform 1.0
  again means no joint is preferred over another for absorbing that redundancy.

Neither of these affects *timing* — they only affect which collision-free path cuRobo chooses
among valid ones, not how fast it's allowed to move along it.

### `max_acceleration` and `velocity_scale` — the actual speed/accel limits

These are the two values you doubled, and they're what's now causing the validator rejection.
They work very differently from each other:

- **`max_acceleration`** is an **absolute** value in rad/s², applied directly as the joint's
  acceleration bound inside cuRobo's trajectory optimizer. The comment in the YAML
  (lines 117–124) documents how these were derived: the Kortex firmware's `ValidateWaypointList`
  rejected early trajectories with messages reporting hard accel ceilings of **~51.566 deg/s²**
  for joints 1–4 (large actuators) and **~515.662 deg/s²** for joints 5–7 (small wrist
  actuators) — i.e. the firmware itself told us, via rejection error messages, what its real
  limits are. The YAML's `1.7` (joints 1-4) and `17.0` (joints 5-7) rad/s² were chosen as a
  *safety-margined* fraction of those firmware ceilings (51.566°/s² ≈ 0.8999 rad/s², so 1.7
  rad/s² already exceeds the joints-1-4 firmware limit by ~2x once you account for the
  comment's stated margin — this is suspicious and explained below).
- **`velocity_scale`** is *not* absolute — it's a multiplier (0.0-1.0 nominally) applied to each
  joint's **URDF-rated maximum velocity** (1.3963 rad/s for joints 1-4, 1.2218 rad/s for joints
  5-7, per the comment at line 126). `0.60` and `0.68` were tuned to land near the robot's
  configured operational cap of ~25 deg/s (0.4364 rad/s) with margin.

**This asymmetry is exactly why doubling both values broke things.** Doubling `velocity_scale`
just doubles a multiplier against the URDF's *generic* hardware ceiling — still bounded above by
1.0 and the URDF's absolute max, so it can't blow past hardware limits by much. But doubling
`max_acceleration` doubles an *already-converted, already-validated-against-firmware* absolute
number — 1.7 rad/s² (joints 1-4) was already close to (and per the math above, possibly already
over) the firmware's hard ceiling of 0.8999 rad/s², so doubling it to 3.4 rad/s² (≈195 deg/s²)
is roughly **4x** the firmware's actual 51.566 deg/s² limit. That lines up with what you saw:
`[wp 60] value=208.832 (min=0.000 max=51.566)` is ~4x over the limit, consistent with squaring
effects in how acceleration propagates through the trajectory (a faster planned velocity also
raises peak acceleration at direction changes, compounding the doubled `max_acceleration` itself).

The `[wp 32] value=32.220 (min=0.000 max=25.004)` velocity error confirms `velocity_scale`'s
doubling also pushed past the firmware's real ~25.004 deg/s cap on joint 6 (a small actuator),
even though `velocity_scale` is nominally a fraction-of-URDF-max value, because the URDF's rated
max for joints 5-7 (1.2218 rad/s ≈ 70°/s) is itself well above the firmware's configured 25°/s
operational cap — `velocity_scale` was tuned down specifically to compensate for that gap, and
doubling it removed the compensation.

**Practically:** treat `max_acceleration` as a hard, already-safety-margined ceiling you
back-calculate from `ValidateWaypointList`'s own rejection messages (each rejection literally
tells you `min`/`max` per actuator), not a knob to tune by intuition. If you want faster motion,
the validator's `max` values *are* the ceiling — go up to just under them, not past them. Right
now the safest path back is to halve `max_acceleration` to its original `[0.85, 0.85, 0.85,
0.85, 8.5, 8.5, 8.5]` (or directly under the firmware ceilings: ~0.85 / ~8.5 rad/s²) and likewise
revert `velocity_scale` to `[0.30, 0.30, 0.30, 0.30, 0.34, 0.34, 0.34]`, then re-tune upward in
small increments while watching the validator's reported `max` values rather than doubling
blindly.

### `lock_joints`
Collapses the gripper's 6 mimic-joint DOFs (only `finger_joint` is a real actuator; the rest
follow it via URDF `<mimic>`) into fixed transforms at `finger_joint = 0.0` (open). This doesn't
affect arm timing, but it does affect collision geometry: cuRobo always treats the gripper as
fully open for planning purposes, regardless of its actual real-time state. This is a
simplification, not a tracked limitation — see Section 5 for why it matters less than it sounds.

---

## 3. How cuRobo decides waypoint count and timing

This is the most important section for understanding the 69-waypoint trajectory in your log.

### Step A — `plan_single` does IK + graph planning + trajectory optimization
`self.planner.plan_single(start_state, goal_pose, MotionGenPlanConfig(timeout=5.0))`
(`robot_controller_ros2.py:375`) runs cuRobo's full pipeline: solve IK for the goal pose, find a
collision-free path through joint space (graph search / nvblox-style search depending on
config), then hand that rough path to a GPU trajectory optimizer (`trajopt`) that refines it into
a smooth, time-parameterized trajectory respecting `max_acceleration`, `velocity_scale`-derived
velocity limits, and jerk limits.

### Step B — the optimizer's internal step count vs. interpolation
cuRobo's trajopt internally optimizes over a **fixed-size horizon** of discrete steps (a
constant baked into the loaded `MotionGenConfig`, commonly 32 steps by default for cuRobo's
"fast robot" presets). The comment at `robot_controller_ros2.py:99-106` makes this explicit:
*"trajopt optimizes over a fixed 32-step horizon; `maximum_trajectory_dt` sets the per-step time
budget (default 0.15s -> ~4.8s total horizon)."* This optimization horizon is **not** the
69 waypoints you saw — it's the coarse, fixed-size representation cuRobo solves the optimization
problem over.

`maximum_trajectory_dt=2.5` (set explicitly at line 114, overriding cuRobo's faster-robot
default) widens that per-step time budget specifically because this robot's `cspace` limits are
much slower than cuRobo's defaults assume — without widening it, a feasible smooth trajectory at
this robot's real velocity/acceleration limits might need more total time than the default
horizon (32 steps x default per-step dt) could represent, and the optimizer would either fail to
converge or be forced into an infeasible solution.

### Step C — interpolation produces the dense waypoint list you actually see
After the (coarse, 32-step) trajopt solution converges, `result.get_interpolated_plan()`
resamples that solution into a **dense, fixed-timestep trajectory** at `result.interpolation_dt`
intervals (typically a small constant like 0.02-0.05s, chosen internally by cuRobo based on the
optimized trajectory's duration and smoothness requirements — see the comment at
`robot_controller_ros2.py:381-385`). **This is where your 69 waypoints come from**: total
optimized trajectory duration ÷ `interpolation_dt`. A longer, slower trajectory (because you
widened `max_acceleration`/`velocity_scale` limits down) produces *more* interpolated waypoints
at the same `interpolation_dt`, not fewer — so waypoint count alone isn't a proxy for speed.

### Step D — per-waypoint duration in `controller.cpp`
Each interpolated point gets `time_from_start = i * interpolation_dt` in
`robot_controller_ros2.py:393-395`. `controller.cpp:255-276` then converts consecutive
`time_from_start` deltas into each `AngularWaypoint`'s `duration` field — this is the *only*
timing information sent to the firmware per segment (see Section 4's note on why velocity/
blending constraints are deliberately not also set). **So the real chain governing timing is:**
`cspace.max_acceleration` / `cspace.velocity_scale` → trajopt's optimized trajectory duration →
`interpolation_dt` spacing → per-segment `duration` in degrees/duration sent to firmware →
firmware's own `ValidateWaypointList` re-checks that segment-to-segment angular deltas divided by
duration don't exceed *its* configured speed/accel ceilings (the actual rejection point in your
log). Three independent systems (cuRobo's optimizer, the interpolator, and the firmware
validator) all reason about timing using **different limit values** — cuRobo's `cspace` config is
the only one of the three you control directly, and it must stay under the firmware's real
ceilings for the validator to ever pass, which is exactly the failure you hit.

---

## 4. `controller.cpp`'s waypoint executor in detail

`execute_joints()` (line 214) branches on whether `goal->trajectory_points` is non-empty
(cuRobo path) or only `goal->joint_angles` is set (legacy single-target path, used by
`move_to_home`/`move_to_user`/`adjust_joints`).

### What it builds
A Kortex `WaypointList` (`k_api::Base::WaypointList`) containing one `AngularWaypoint` per
trajectory point. Each `AngularWaypoint` holds:
- 7 joint angles, **converted from radians (cuRobo/ROS convention) to degrees** (Kortex
  convention) — line 250.
- A single scalar `duration` for that segment, derived either from the planner's real
  `time_from_start` deltas, or (if those are absent/zero) a conservative fallback that sizes
  duration so no joint moves faster than ~20°/s (lines 266-275). This fallback exists for
  robustness against any caller that doesn't populate `time_from_start` — but as the comment at
  `robot_controller_ros2.py:381-385` notes, this fallback was actually silently engaged for a
  while due to a bug, producing a 1s-per-waypoint floor instead of the real interpolation timing.

`waypoint_list.set_duration(0.0f)` and `set_use_optimal_blending(false)` (lines 232-233) are
deliberate: the comment explains the firmware's trajectory generator requires the first and last
waypoints to be at rest, and an over-constrained list (mixing per-waypoint `duration` with
`maximum_velocities` or blending) gets rejected outright with errors like
`INITIAL/FINAL_WAYPOINT_NO_STOP` or `INVALID_DURATION`. So **only `duration` is set per
waypoint** — no per-waypoint velocity constraints, no blending — leaving the firmware free to
interpolate between consecutive angle/duration pairs however its internal trajectory generator
sees fit, as long as the implied angular rate stays under its ceilings.

### Built-in safety features
1. **Pre-execution validation** (lines 280-304): `mBase->ValidateWaypointList()` is called and
   checked *before* `ExecuteWaypointTrajectory()` is ever invoked. If the firmware reports any
   errors, the goal is aborted locally — the robot **never receives the trajectory** and never
   moves. This is exactly the safety net that caught your doubled `cspace` values; nothing
   physically happened.
2. **Per-waypoint joint count check** (line 237): rejects malformed trajectory points with the
   wrong DOF count before even building the waypoint list.
3. **60-second execution timeout** (line 343): if the firmware never sends an `ACTION_END`/
   `ACTION_ABORT` notification, the controller calls `mBase->Stop()` itself rather than hanging
   forever.
4. **Cancellation handling** (line 335): a ROS 2 action cancel request triggers `mBase->Stop()`
   mid-trajectory.
5. **The single `mApiMutex`** guarding every Kortex API call (per `CLAUDE.md`'s documented
   constraint) — `ValidateWaypointList`, `ExecuteWaypointTrajectory`, and `Stop()` all serialize
   against any other in-flight Kortex call (e.g. a concurrent `gripper_command` or
   `publishState` poll), so a trajectory execution can't race with another action touching the
   same robot session.

### Could it be redesigned to work better with cuRobo?
A few concrete gaps worth knowing about:
- **No velocity-aware blending.** Because `use_optimal_blending` is off and no per-waypoint
  `maximum_velocities` are set, the firmware is free to choose its own interpolation between
  angle/duration pairs — this is *not* guaranteed to match the smooth, jerk-limited curve cuRobo
  actually optimized. cuRobo computed a careful velocity/acceleration profile; the firmware only
  receives discrete angle+duration checkpoints and re-derives its own motion in between. For most
  purposes the dense (`interpolation_dt`-spaced) sampling makes this approximation error small,
  but it means the firmware's executed trajectory is not bit-for-bit cuRobo's plan.
- **No mid-trajectory feedback/replanning.** Once `ExecuteWaypointTrajectory` is sent, the
  controller only polls for `ACTION_END`/`ACTION_ABORT` — there's no mechanism to react to a new
  obstacle appearing mid-motion (the dynamic obstacle registration in
  `update_dynamic_obstacle` only affects *future* `plan_single` calls, not an in-flight one).
- **The `time_from_start`-absent fallback (lines 263-275) silently degrades to ~20°/s,** which
  could mask a future bug the same way the `interpolation_dt` bug did — worth a `RCLCPP_WARN` if
  that branch is ever taken, since right now it fails silently into a working-but-much-slower
  trajectory rather than surfacing the underlying data loss.
- **The validator's full per-actuator limits aren't introspected at startup.** Right now you only
  learn the real firmware ceilings by provoking a rejection and reading the error message. A
  one-time startup query (if Kortex exposes a "get configured speed/accel limits" API distinct
  from `ValidateWaypointList`) could let `robot_controller_ros2.py` assert at launch that
  `cspace.max_acceleration`/`velocity_scale` are under the real firmware ceilings, catching
  exactly the kind of misconfiguration you hit before any plan is even attempted.

---

## 5. Which Kortex "control mode" applies to our waypoint trajectories?

This is the section where the repo's existing comments stop short, and where I want to be
explicit about what's confirmed vs. inferred — verify the inferred part against Kinova's own
Kortex API documentation or support before tuning `cspace` against it further.

The Kinova web app's **Speed Limits → Advanced** section exposes five separately-configurable
control-mode limit tables, because the Kortex firmware genuinely enforces different speed/accel
ceilings depending on which command type is in flight:

| Web app mode | Kortex API call it corresponds to | Used in this codebase? |
|---|---|---|
| Angular joystick | Manual `SendJoystickCommand`-style joint jogging | No |
| Cartesian joystick | Manual `SendJoystickCommand`-style Cartesian jogging | No |
| Angular trajectory | `ExecuteAction` with `reach_joint_angles` (single joint target) | **Yes** — legacy single-target path (`move_to_home`, `move_to_user`, `adjust_joints`), the `else` branch of `execute_joints` (line 371) |
| Cartesian trajectory | `ExecuteAction` with `reach_pose` (single Cartesian target) | **Yes** — `execute_pose()` (line 133), the legacy `MoveToPose` fallback |
| Cartesian waypoint trajectory | `ExecuteWaypointTrajectory` with a `WaypointList` of **`CartesianWaypoint`** entries | **No** |

Our cuRobo-driven multi-point path (`execute_joints`'s `is_trajectory` branch, line 223) calls
`mBase->ExecuteWaypointTrajectory()` — the same RPC the "Cartesian waypoint trajectory" mode
uses — **but populates each waypoint's `angular_waypoint` field, not `cartesian_waypoint`**
(line 247: `wp->mutable_angular_waypoint()`). This is a genuine gap in the table above: the web
app's five categories don't cleanly include "angular waypoint trajectory" as a distinct,
separately-configurable mode. Two plausible readings, and I can't fully resolve which is correct
without Kinova's firmware source or direct documentation:

1. **The firmware reuses the "Angular trajectory" limits for angular waypoints**, since both
   ultimately command joint-space motion via the same internal trajectory generator family —
   just with one (1) or many (N) checkpoints. Under this reading, the `min`/`max` values
   `ValidateWaypointList` reported in your error log (51.566 deg/s², 515.662 deg/s², 25.004
   deg/s) **are** the robot's configured "Angular trajectory" limits, and that's the web-app
   table you should read/edit if you want to deliberately raise the ceiling (with appropriate
   caution).
2. **The firmware applies a distinct, not-directly-exposed limit set for angular waypoint
   lists**, separate from both "Angular trajectory" and "Cartesian waypoint trajectory," in which
   case the web app simply doesn't expose a UI for tuning it directly, and the only ground truth
   you have is what `ValidateWaypointList`'s rejection messages report.

**What's actually confirmed, independent of which reading is correct:** the `min=0.000
max=51.566`/`max=515.662`/`max=25.004` values in `ValidateWaypointList`'s error report **are
exactly the real, currently-configured ceiling for this exact RPC/waypoint-type combination**,
queried live from the firmware at validation time — that's what `ValidateWaypointList` is *for*.
So regardless of which named web-app category backs it, those reported numbers are the
authoritative limit to tune `cspace.max_acceleration`/`velocity_scale` against; the web app's
"Angular trajectory" table is the most likely place to cross-check or adjust them, but the
validator's own error output is strictly more reliable for this purpose than reading the web UI,
because it reflects exactly the RPC and waypoint type this code actually uses.

**Recommendation:** don't tune `cspace.max_acceleration`/`velocity_scale` by guessing at web-app
categories — pick a candidate value, attempt a move, and read the exact `min`/`max` the validator
reports for whichever actuator triggers first. That number is ground truth for this code path,
independent of how Kinova's UI happens to label it.
