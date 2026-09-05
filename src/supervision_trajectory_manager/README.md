# supervision_trajectory_manager 0.4.1

This revision adapts `supervision_trajectory_manager_node` to the new
`deconfliction_manager` supervision-request message.

## Input change

Old input:

```text
/requested_supervision_trajectories
collision_detection/msg/DetectedCollisionTrajectoryArray
```

New input:

```text
/requested_supervision_trajectories
deconfliction_manager/msg/RequestedSupervisionTrajectoryArray
```

Each request already contains:

```text
detected_collision
  -> ORIGINAL DetectedCollisionTrajectory

cropped_trajectory
  -> virtual-net crop calculated by deconfliction_manager_node
```

This node no longer decides which collision nodes/segments must be removed and
no longer maps collision nodes back to sparse original waypoints.

## Responsibilities

The node now performs only:

```text
1. Acquire detected_collision + cropped_trajectory.
2. Materialize the already-cropped virtual-net geometry into the legacy
   StaticTrajectory representation required by /adjusted_trajectories.
3. Publish the adjusted StaticTrajectory while PENDING.
4. Confirm that exact geometry in /available_static_trajectories.
5. Publish the ORIGINAL DetectedCollisionTrajectory on /supervised_trajectories.
6. Synchronize periodic operation_start_utc / operation_end_utc from AVAILABLE.
```

## Important representation constraint

`CroppedNetTrajectory` can preserve multiple disconnected retained pieces in one
phase.

`static_trajectory_manager/msg/TrajectorySegment` cannot. It is one continuous
x/y/z polyline.

The previous implementation selected the longest continuous retained run of
each phase. That was incorrect because it changed the crop received from
`deconfliction_manager_node`.

This version performs an **exact-only** conversion:

```text
0 retained runs in phase -> empty phase
1 retained run           -> exact TrajectorySegment
2+ retained runs         -> NOT representable as StaticTrajectory
```

Two retained net edges are considered contiguous only when:

```text
they share a node
AND
next.net_segment_index == previous.net_segment_index + 1
```

The node never discards a fragment and never reconnects a gap. If the exact
upstream crop is disconnected inside one phase, it is kept internally and
displayed in RViz, but nothing is published to `/adjusted_trajectories` for
that entry because the current `StaticTrajectory` schema cannot encode it
without changing the geometry.

This is a representation/materialization step, not a new crop decision. The
upstream `deconfliction_manager_node` remains the only node that decides which
virtual-net nodes and edges are removed.

The exact complete upstream crop is retained internally and shown in RViz as a
`LINE_LIST`.

## Incomplete crop handling

If:

```text
cropped_trajectory.complete == false
```

the request is stored but is not sent to `/adjusted_trajectories`.

The node waits for the authoritative transient-local request to be republished
with:

```text
complete == true
```

An older incomplete snapshot cannot overwrite a complete crop that has already
been acquired.

## Persistent lifecycle

Requests are not deleted merely because they disappear from
`/requested_supervision_trajectories`.

That disappearance is expected after the adjusted trajectory enters the
available flow and is no longer detected as an upstream collision.

Lifecycle:

```text
new complete request
    -> PENDING
    -> /adjusted_trajectories

expected adjusted geometry appears in AVAILABLE
    -> SUPERVISED
    -> /supervised_trajectories

expected adjusted geometry disappears from AVAILABLE
    -> PENDING
    -> /adjusted_trajectories again
```

## `/supervised_trajectories`

The output type remains:

```text
collision_detection/msg/DetectedCollisionTrajectoryArray
```

The published object is the ORIGINAL complete `detected_collision` payload.

The virtual-net crop is not substituted into this output.

## Periodic trajectories

For:

```text
operation_frequency > 0
```

once the adjusted geometry is found in `/available_static_trajectories`, only:

```text
operation_start_utc
operation_end_utc
```

are copied from the AVAILABLE cropped occurrence.

Those updated times are applied to the ORIGINAL trajectory published on
`/supervised_trajectories` and to the stored adjusted copy used for reinjection.

This preserves the previously implemented total-period progression.

## QoS

All trajectory subscriptions/publications use:

```text
RELIABLE
TRANSIENT_LOCAL
KeepLast(1)
```

The node also republishes every second by default.

## Build order

`supervision_trajectory_manager` now depends on `deconfliction_manager` because
the latter owns `RequestedSupervisionTrajectoryArray`.

```bash
cd ~/a_space_ws

rm -rf build/supervision_trajectory_manager
rm -rf install/supervision_trajectory_manager

colcon build --symlink-install \
  --packages-up-to supervision_trajectory_manager

source install/setup.bash
```

## Check

```bash
ros2 topic info /requested_supervision_trajectories
```

Expected:

```text
Type: deconfliction_manager/msg/RequestedSupervisionTrajectoryArray
```

The downstream output remains:

```bash
ros2 topic info /supervised_trajectories
```

Expected:

```text
Type: collision_detection/msg/DetectedCollisionTrajectoryArray
```
