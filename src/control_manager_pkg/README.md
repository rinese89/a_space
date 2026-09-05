# control_manager_pkg — supervision-action version

ROS 2 Humble package containing the per-UAS node:

```text
/<flight_zone>/<uas_namespace>/control_manager_node
```

This version removes the direct subscription to:

```text
/active_trajectories
```

Trajectory acquisition and supervision commands now arrive exclusively through:

```text
/<flight_zone>/<uas_namespace>/supervision_control
```

Type:

```text
flight_zone_supervision/action/SupervisionControl
```

The action contract is defined in the `flight_zone_supervision` package.

## Why this change

The flight-zone `supervision_node` is now the authority that decides:

- which active trajectory belongs to each UAS;
- whether that trajectory is supervised or normal;
- whether the non-supervised counterpart must be paused;
- when that counterpart may resume.

`control_manager_node` therefore no longer independently reads
`/active_trajectories`.

## Supervision action commands

### EXECUTE

The goal must contain exactly one:

```text
static_trajectory_manager/msg/StaticTrajectory
```

inside:

```text
goal.trajectories.trajectories[]
```

The trajectory must belong to this node's:

```text
/<flight_zone>/<uas_namespace>
```

and must pass the same geometry/action/time validation used by the previous
`/active_trajectories` callback.

The occurrence key remains:

```text
trajectory_id + operation_start_utc
```

so duplicate `EXECUTE` commands for the same occurrence are idempotent.

A different occurrence is rejected while this node is busy executing/storing
another one.

The action result acknowledges acquisition of the command. It does **not**
remain open until the complete flight finishes. The actual flight continues
asynchronously through the existing state machine.

### PAUSE

PAUSE is now supervision-aware.

It can be applied:

```text
WAITING_START
TAKEOFF_REQUEST
TAKEOFF_RESPONSE
MISSION_DISPATCH
MISSION_ACTIVE
```

as well as idempotently while already paused.

New state:

```text
PAUSED_BEFORE_START
```

If PAUSE is received before takeoff, no takeoff command is sent until RESUME.

If PAUSE arrives while TAKEOFF is already being processed, the takeoff is
allowed to finish and the UAS remains in HOLD before mission dispatch.

If PAUSE arrives during MISSION_ACTIVE, the existing FollowWaypoints goal is
canceled and the controller remains in HOLD exactly as in the previous
implementation.

### Preemptive PAUSE latch

`supervision_node` may send EXECUTE and PAUSE in the same evaluation cycle.

DDS/action request scheduling should not be relied upon to guarantee which
command reaches the control manager first.

Therefore, if PAUSE arrives while no trajectory is stored:

```text
PAUSE
  -> preemptive_pause_latch = true
```

and the next EXECUTE is acquired directly as:

```text
PAUSED_BEFORE_START
```

This makes EXECUTE/PAUSE ordering deterministic from a safety perspective.

### RESUME

RESUME performs:

```text
PAUSED_BEFORE_START -> WAITING_START
PAUSED              -> MISSION_DISPATCH
```

If a mission-action cancellation for PAUSE is already in flight, RESUME is
queued and the remaining mission is dispatched as soon as cancellation
finishes.

RESUME also clears a preemptive PAUSE latch if no trajectory has been acquired
yet.

### STOP

STOP preserves the existing total-stop behavior:

- before takeoff: cancel the occurrence without flight commands;
- during mission: cancel the FollowWaypoints action;
- after takeoff/while paused: return toward the landing entry;
- finally send the landing endpoint through `ArmTakeoff`.

## Supervision-authorized start delay

The original node rejects an operation when:

```text
now - operation_start_utc > max_start_lateness_s
```

That remains the default behavior.

However, if supervision intentionally pauses the UAS **before takeoff**, the
operation may legitimately resume later than the normal lateness limit.

The new state machine records that this delay was supervision-authorized.
After RESUME, the late start is accepted once and TAKEOFF proceeds.

This avoids turning a deliberate safety pause into a false
`operation start was missed` failure.

## Existing internal control flow retained

Once EXECUTE has been acquired, the main control sequence is still:

```text
WAITING_START
    |
    v
TAKEOFF_REQUEST
    |
    v
ArmTakeoff service
    |
    v
MISSION_DISPATCH
    |
    v
controllers_pkg/action/FollowWaypoints
    |
    v
LANDING_REQUEST
    |
    v
ArmTakeoff service
    |
    v
COMPLETED
```

Mission repetition expansion, TF transforms, feedback monitoring, path
deviation detection, return-and-land fallback and completed-occurrence
deduplication are preserved.

## State machine

States:

```text
WAITING_TRAJECTORY
WAITING_START
PAUSED_BEFORE_START      <-- new
TAKEOFF_REQUEST
TAKEOFF_RESPONSE
MISSION_DISPATCH
MISSION_ACTIVE
PAUSE_CANCELING
PAUSED
STOP_CANCELING
RETURN_DISPATCH
RETURN_ACTIVE
LANDING_REQUEST
LANDING_RESPONSE
COMPLETED
FAILED
```

## Existing manual service retained

The previous service is intentionally kept:

```text
/<flight_zone>/<uas_namespace>/trajectory_control
```

Type:

```text
control_manager_pkg/srv/ControlTrajectory
```

Commands:

```text
0 = PAUSE
1 = RESUME
2 = STOP
```

It now uses the same pause/resume/stop state-transition helpers as the
supervision action, so automatic supervision and manual operator control do not
diverge in behavior.

## Underlying controller APIs

The node still uses:

```text
/<flight_zone>/<uas_namespace>/arm_takeoff
controllers_pkg/srv/ArmTakeoff
```

and:

```text
/<flight_zone>/<uas_namespace>/follow_waypoints
controllers_pkg/action/FollowWaypoints
```

`StaticTrajectory.action_name` is still validated against the per-UAS
`follow_waypoints` action name.

## Expected integration

```text
/active_trajectories
        |
        v
/<flight_zone>/supervision_node
        |
        | SupervisionControl::EXECUTE
        | SupervisionControl::PAUSE
        | SupervisionControl::RESUME
        v
/<flight_zone>/<uas>/control_manager_node
        |
        +--> ArmTakeoff service
        |
        `--> FollowWaypoints action
```

`control_manager_node` itself has **no `/active_trajectories` subscription**.

## Build

The `flight_zone_supervision` package must be available because it owns the
`SupervisionControl.action` interface.

From the workspace root:

```bash
cd ~/a_space_ws

rm -rf build/control_manager_pkg
rm -rf install/control_manager_pkg

colcon build --symlink-install \
  --packages-up-to control_manager_pkg

source install/setup.bash
```

## Launch

Example:

```bash
ros2 launch control_manager_pkg control_manager_node.launch.py \
  flight_zone_id:=inspection_1 \
  uas_namespace:=ua_ins_1
```

Expected action server:

```text
/inspection_1/ua_ins_1/supervision_control
```

Check it with:

```bash
ros2 action info /inspection_1/ua_ins_1/supervision_control
```

## Direct action test

An EXECUTE goal is normally produced by `supervision_node`.

PAUSE can be tested with:

```bash
ros2 action send_goal \
  /inspection_1/ua_ins_1/supervision_control \
  flight_zone_supervision/action/SupervisionControl \
  "{command: 1, trajectories: {trajectories: []}, reason: 'test pause'}"
```

RESUME:

```bash
ros2 action send_goal \
  /inspection_1/ua_ins_1/supervision_control \
  flight_zone_supervision/action/SupervisionControl \
  "{command: 2, trajectories: {trajectories: []}, reason: 'test resume'}"
```

The action constants are:

```text
EXECUTE=0
PAUSE=1
RESUME=2
STOP=3
```
