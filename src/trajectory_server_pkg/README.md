# trajectory_server_pkg 0.9.0

Adds a minimum valid ARM duration before a DISARM event can complete an active
trajectory.

## Motivation

The UAS may fail during takeoff because OFFBOARD messages lose continuity.

PX4 can then leave OFFBOARD, recover/land and become DISARMED. The same
trajectory is still going to be retried by the control layer.

The previous trajectory server logic was:

```text
ARM observed
   ->
DISARM observed
   ->
trajectory completed
```

That incorrectly removed `/active_trajectories` after a failed takeoff.

## New rule

Parameter:

```yaml
minimum_armed_duration_for_completion_s: 20.0
```

Completion is now evaluated per ARM cycle:

```text
NEW ARM
   |
   v
start steady-clock timer
   |
   +-- DISARM before 20 s
   |      |
   |      +--> ignore DISARM
   |      +--> keep trajectory ACTIVE
   |      +--> invalidate this ARM cycle
   |      +--> wait for NEW ARM from retry
   |
   +-- DISARM after >= 20 s
          |
          +--> complete trajectory
          +--> remove from ACTIVE
```

## Why the ARM cycle is invalidated after a short DISARM

Suppose:

```text
ARM
2 s later -> DISARM
```

If the server kept the original ARM timestamp while the UAS remained DISARMED,
then 18 s later the same DISARMED state could accidentally satisfy the 20 s
threshold.

Therefore a short DISARM executes:

```text
armed_seen = false
armed_since = empty
```

The trajectory remains active, but no later DISARM can complete it until a
fresh `ARMING_STATE_ARMED` sample is observed.

The retry therefore creates:

```text
ARM #2
   ->
new 20 s timer
```

## Active trajectory lifecycle example

```text
trajectory enters /active_trajectories
        |
        v
ARM #1
        |
        | 4 s
        v
DISARM #1                 <- failed takeoff
        |
        +--> ignored
        +--> ACTIVE remains published
        |
        v
control layer retries takeoff
        |
        v
ARM #2
        |
        | 75 s operation
        v
DISARM #2                 <- real landing
        |
        v
trajectory completed
```

## Existing runtime arbitration

All previous behavior remains unchanged:

- `/available_static_trajectories`
- `/supervised_trajectories`
- `/active_trajectories`
- runtime spatial arbitration
- priority waiting/rejection
- original-trajectory registration
- `/non_priority_adjustment_trajectories`
- multi-mission geometry semantics

## Parameter

```yaml
vehicle_status_suffix: fmu/out/vehicle_status
vehicle_status_timeout_s: 2.0
minimum_armed_duration_for_completion_s: 20.0
```

Set:

```yaml
minimum_armed_duration_for_completion_s: 0.0
```

to recover the legacy behavior.

## Build

```bash
cd ~/a_space_ws

rm -rf build/trajectory_server_pkg
rm -rf install/trajectory_server_pkg

colcon build --symlink-install \
  --packages-up-to trajectory_server_pkg

source install/setup.bash
```
