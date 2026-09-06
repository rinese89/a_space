# trajectory_endtime_adjustment 0.4.0

Adaptation to the multi-mission
`static_trajectory_manager/msg/StaticTrajectory` interface.

## Upstream message change

`StaticTrajectory` now contains:

```text
TrajectorySegment takeoff
TrajectorySegment[] mission
TrajectorySegment landing
```

instead of a single `TrajectorySegment mission`.

## Why this node requires almost no geometry logic change

`trajectory_endtime_adjustment_node` does not:

- build geometric segments;
- discretize trajectories;
- detect collisions;
- calculate path length;
- access `mission.x/y/z`;
- concatenate trajectory phases.

It stores and republishes complete `StaticTrajectory` objects.

The normal adjustment is based on:

```cpp
StaticTrajectory adjusted = tracked.trajectory;
adjusted.extra_time = ...;
adjusted.operation_end_utc = ...;
```

The special delayed-priority adjustment is based on:

```cpp
StaticTrajectory adjusted = tracked.original;
adjusted.extra_time = ...;
adjusted.operation_end_utc = ...;
```

Therefore C++ message copy semantics automatically preserve:

```text
takeoff
mission[0]
mission[1]
...
mission[N-1]
landing
```

without any conversion.

## Critical invariant

This node changes only timing metadata:

```text
extra_time
operation_end_utc
```

and, in the special delayed-priority path, preserves the ORIGINAL:

```text
operation_start_utc
```

It does not alter:

```text
takeoff
mission[]
landing
priority
trajectory_id
ua_id
uas_namespace
flight_zone_id
frame_id
action_name
goal_tolerance
slowdown_radius
repetitions
operation_frequency
average_speed_mps
estimated_distance_m
estimated_duration_s
spacial_conflict
spacial_conflicting_trajectory_id
```

This is particularly important for supervised trajectories whose `mission[]`
may contain several disconnected collision-free fragments.

## Normal occurrence

Lifecycle:

```text
/active_trajectories
      |
      | trajectory present
      v
TrackedOccurrence
      |
      | trajectory disappears
      v
actual_end = observation time
      |
      +--> extra_time =
      |      actual_end - planned_end
      |
      +--> operation_end_utc = actual_end
      |
      v
/adjusted_trajectories
```

The complete multi-mission geometry is copied unchanged.

## Delayed high-priority occurrence

Before `trajectory_server_node` delays a high-priority candidate, it registers
the ORIGINAL trajectory through:

```text
/trajectory_endtime_adjustment/register_original_trajectory
```

The service request remains:

```text
static_trajectory_manager/StaticTrajectory trajectory
```

so the registered original includes the complete `mission[]`.

When the delayed copy later appears in `/active_trajectories`, the node binds:

```text
ORIGINAL complete StaticTrajectory
        <->
ACTIVE delayed StaticTrajectory
```

When ACTIVE disappears:

```text
original_duration =
    original_end - original_start

active_duration =
    actual_active_end - active_start

extra_time =
    active_duration - original_duration
```

The published adjustment starts from the ORIGINAL complete trajectory, so all
mission components are preserved.

## Multi-mission runtime diagnostics

The INFO logs now include:

```text
missions=<trajectory.mission.size()>
```

when storing an original priority trajectory and when beginning normal tracking.

This is diagnostic only and does not change the trajectory.

## Topics

Input:

```text
/active_trajectories
static_trajectory_manager/msg/StaticTrajectoryArray
```

Output:

```text
/adjusted_trajectories
static_trajectory_manager/msg/StaticTrajectoryArray
```

Service:

```text
/trajectory_endtime_adjustment/register_original_trajectory
trajectory_endtime_adjustment/srv/RegisterOriginalTrajectory
```

QoS for the trajectory topics remains:

```text
RELIABLE
TRANSIENT_LOCAL
KeepLast(1)
```

## Build

Because `StaticTrajectory.msg` changed, this package must be rebuilt even though
its timing algorithm did not require structural geometry changes.

```bash
cd ~/a_space_ws

rm -rf build/static_trajectory_manager \
       build/trajectory_endtime_adjustment

rm -rf install/static_trajectory_manager \
       install/trajectory_endtime_adjustment

colcon build --symlink-install \
  --packages-up-to trajectory_endtime_adjustment

source install/setup.bash
```

If other already-migrated packages are in the dependency chain, rebuild the
workspace or the corresponding package set as appropriate.

## Verification

Inspect the interface:

```bash
ros2 interface show static_trajectory_manager/msg/StaticTrajectory
```

and verify:

```text
static_trajectory_manager/TrajectorySegment[] mission
```

Then compare an ACTIVE trajectory and its later adjustment:

```bash
ros2 topic echo /active_trajectories
ros2 topic echo /adjusted_trajectories
```

For the same occurrence, `mission[]` must be identical. Only the timing fields
described above should change.
