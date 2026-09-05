# flight_zone_supervision 0.3.0

Per-flight-zone supervision node:

```text
/<flight_zone>/supervision_node
```

## Inputs

```text
/available_static_trajectories
  static_trajectory_manager/msg/StaticTrajectoryArray

/supervised_trajectories
  collision_detection/msg/DetectedCollisionTrajectoryArray

/active_trajectories
  static_trajectory_manager/msg/StaticTrajectoryArray
```

The node dynamically creates:

```text
/<flight_zone>/<uas>/zone_status
  flight_zone_msgs/msg/VehicleZoneStatus
```

subscriptions and:

```text
/<flight_zone>/<uas>/supervision_control
  flight_zone_supervision/action/SupervisionControl
```

action clients for UAS participating in supervised conflicts.

## EXECUTE behavior

Normal active trajectory:

```text
/active_trajectories
  -> EXECUTE the active StaticTrajectory
```

Supervised active trajectory:

```text
/active_trajectories contains the cropped trajectory
/supervised_trajectories contains the original DetectedCollisionTrajectory
  -> EXECUTE the ORIGINAL complete StaticTrajectory
```

The cropped geometry remains a planning reservation. The per-UAS
`control_manager_node` receives the full original trajectory.

# Two-stage deconfliction

For a pair:

```text
S = supervised active trajectory
N = non-supervised active conflicting trajectory
```

the control sequence is now explicit.

## Phase 1 — primary supervision of S

On pair creation:

```text
PAUSE S
```

Then the usual distance rule applies:

```text
distance(S,N) <= security_distance_m
  -> PAUSE S

distance(S,N) > security_distance_m
  -> RESUME S
```

When S is paused at/below the security threshold, the node measures whether the
separation is actually improving.

The implementation defines "the primary measure is effective" as:

```text
distance increases by >= distance_progress_epsilon_m
within distance_progress_timeout_s
```

Defaults:

```yaml
distance_progress_timeout_s: 0.5
distance_progress_epsilon_m: 0.05
```

If the separation does not improve, the node escalates.

## Phase 2 — hold N and clear S

Escalation is strictly sequenced:

```text
PAUSE N
wait for action acknowledgement
RESUME S
```

S remains paused until the PAUSE request for N has been positively accepted.
N then remains paused while S clears the collision region.

S is then allowed to continue through the original collision section.

The node calculates the minimum distance from the current `zone_status.position`
of S to all original:

```text
collision_nodes[]
collision_segments[]
```

stored in `/supervised_trajectories`.

The counterpart N remains paused until:

```text
distance(S, all collision geometry)
  > collision_exit_distance_m + collision_exit_hysteresis_m
```

Defaults:

```yaml
collision_exit_distance_m: 1.0
collision_exit_hysteresis_m: 0.20
```

Once S has left the collision section:

```text
RESUME N
```

and the pair enters `CLEARED`.

## Pair state machine

```text
PRIMARY_SUPERVISED_CONTROL
        |
        | separation does not improve
        v
WAITING_COUNTERPART_PAUSE
        |
        | PAUSE(N) action accepted
        v
COUNTERPART_HELD_FOR_CLEARANCE
        |
        | S entered and then exited collision geometry
        v
CLEARED
```

### PRIMARY_SUPERVISED_CONTROL
Controlled UAS: `S`.

### COUNTERPART_HELD_FOR_CLEARANCE
Commands:

```text
N -> PAUSE
S -> RESUME
```

### CLEARED
Commands:

```text
S -> RESUME
N -> RESUME
```

The pair remains cleared until one of the trajectories disappears from
`/active_trajectories`.

## Fail-safe behavior

### Phase 1
Missing/stale `zone_status` or incompatible frames:

```text
S remains PAUSED
```

### Phase 2
Missing/stale status, collision-frame mismatch, or no usable collision
nodes/segments:

```text
N remains PAUSED
S remains released
```

The node never resumes N unless it can geometrically verify that S has cleared
the original collision section.

## Multiple pairs

Control requests are aggregated across all active pairs. `PAUSE` dominates
`RESUME`.

## RViz

Relative topic:

```text
supervision_markers
```

Pair labels show:

```text
PRIMARY_SUPERVISED_CONTROL
WAITING_COUNTERPART_PAUSE
COUNTERPART_HELD_FOR_CLEARANCE
CLEARED
```

## Main parameters

```yaml
security_distance_m: 1.0
use_3d_distance: true
zone_status_timeout_s: 2.0

distance_progress_timeout_s: 0.5
distance_progress_epsilon_m: 0.05

collision_exit_distance_m: 1.0
collision_exit_hysteresis_m: 0.20

evaluation_period_ms: 100
```

## Build

```bash
cd ~/a_space_ws

rm -rf build/flight_zone_supervision
rm -rf install/flight_zone_supervision

colcon build --symlink-install \
  --packages-up-to flight_zone_supervision

source install/setup.bash
```

## Launch

```bash
ros2 launch flight_zone_supervision supervision_node.launch.py \
  flight_zone_id:=inspection_1
```
