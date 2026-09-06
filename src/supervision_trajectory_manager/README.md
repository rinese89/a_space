# supervision_trajectory_manager 0.5.0

Adapted to the new multi-mission `StaticTrajectory` and to the new
`deconfliction_manager/CroppedNetTrajectory` contract.

## Architectural change

The crop is now fully materialized upstream by `deconfliction_manager_node`:

```text
/requested_supervision_trajectories
  RequestedSupervisionTrajectory
    detected_collision
      -> ORIGINAL DetectedCollisionTrajectory

    cropped_trajectory
      complete
      trajectory_materialized
      trajectory
        -> READY-TO-USE cropped StaticTrajectory
           with TrajectorySegment[] mission
```

This node no longer performs any geometric crop/materialization.

Removed responsibilities include:

```text
phase_segments()
split_runs()
materialize_run()
materialize_phase_exact()
materialize_adjusted()
point-to-polyline orientation logic
```

## Exact acquisition rule

A request can be injected into `/adjusted_trajectories` only when:

```text
cropped_trajectory.complete == true
AND
cropped_trajectory.trajectory_materialized == true
```

When both are true:

```cpp
entry.adjusted = cropped_trajectory.trajectory;
```

No waypoint, edge, mission component or ordering is changed locally.

## Multi-mission preservation

If the upstream crop contains:

```text
mission[0]: A -> B
mission[1]: E -> F
mission[2]: K -> L -> M
```

`/adjusted_trajectories` contains those same three mission components in the
same order.

There is no local run selection and no artificial connection between them.

## TAKEOFF/LANDING materialization failure

`deconfliction_manager_node` may produce:

```text
complete = true
trajectory_materialized = false
```

when TAKEOFF or LANDING is split into more than one disconnected retained run,
because those phases are still represented by one `TrajectorySegment` each.

In that case this node stores the exact crop for diagnostics but does not
publish a false `StaticTrajectory` to `/adjusted_trajectories`.

## AVAILABLE confirmation

The state flow remains:

```text
complete + materialized request
        -> PENDING
        -> /adjusted_trajectories
        -> static_trajectory_manager
        -> /available_static_trajectories
        -> exact multi-mission geometry match
        -> SUPERVISED
```

`same_geometry()` now compares:

```text
trajectory_id
frame_id
repetitions
takeoff
mission.size()
mission[0]
mission[1]
...
landing
```

Mission order is significant.

Operation timestamps are intentionally not part of the geometry comparison.

## `/supervised_trajectories`

The output remains:

```text
collision_detection/msg/DetectedCollisionTrajectoryArray
```

and publishes the ORIGINAL complete collision payload, including the ORIGINAL
multi-mission trajectory.

The cropped trajectory is used only for the AVAILABLE reservation path.

## Periodic trajectories

For `operation_frequency > 0`, once the cropped trajectory is observed in
AVAILABLE, the current:

```text
operation_start_utc
operation_end_utc
```

are copied to the ORIGINAL trajectory published on `/supervised_trajectories`.

The stored adjusted copy also receives those timestamps so a later requeue uses
the current occurrence rather than the original occurrence.

## Persistent lifecycle

Entries are not deleted merely because they disappear from
`/requested_supervision_trajectories`.

That disappearance is expected once the adjusted trajectory enters AVAILABLE
and no longer appears as an upstream collision.

## Markers

Pending markers continue to show the exact retained virtual-net crop as a
`LINE_LIST`.

With the new interface each retained item is a `CroppedNetSegment`, so marker
geometry is read from:

```text
cropped_segment.segment.start
cropped_segment.segment.end
```

No visual bridge is generated between mission fragments.

## Build

Because both `StaticTrajectory` and the deconfliction messages changed, rebuild
the dependency chain cleanly:

```bash
cd ~/a_space_ws

rm -rf build/static_trajectory_manager \\
       build/a_space_virtual_net \\
       build/collision_detection \\
       build/deconfliction_manager \\
       build/supervision_trajectory_manager

rm -rf install/static_trajectory_manager \\
       install/a_space_virtual_net \\
       install/collision_detection \\
       install/deconfliction_manager \\
       install/supervision_trajectory_manager

colcon build --symlink-install \\
  --packages-up-to supervision_trajectory_manager

source install/setup.bash
```

## Runtime check

```bash
ros2 topic echo /requested_supervision_trajectories
ros2 topic echo /adjusted_trajectories
```

For a request with `trajectory_materialized: true`, compare:

```text
requested.trajectories[i].cropped_trajectory.trajectory.mission
```

against:

```text
adjusted.trajectories[j].mission
```

They must be identical except for later periodic start/end synchronization.
