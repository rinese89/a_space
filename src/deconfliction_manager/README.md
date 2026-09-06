# deconfliction_manager 0.6.0

Adaptation to the new multi-mission `static_trajectory_manager/msg/StaticTrajectory`.

## Classification

The threshold remains based only on unique collision-node IDs:

```text
unique(collision_nodes[].node_id) < collision_node_threshold
    -> /requested_supervision_trajectories

unique(collision_nodes[].node_id) >= collision_node_threshold
    -> /manual_adjustment_trajectories
```

Partial repetitions therefore do not inflate the threshold.

## Multi-mission crop

Every `mission[i]` is an independent continuous polyline.

The crop still removes a net edge when:

```text
edge_id is explicitly conflicting
OR start_node_id is a collision node
OR end_node_id is a collision node
```

The old de-duplication key `(phase, edge_id)` is replaced by:

```text
(phase, mission_index, edge_id)
```

so repeated partial passes collapse geometrically, while distinct mission
components remain distinct even if they traverse the same edge.

## CroppedNetSegment

Each retained/removed edge now carries:

```text
mission_index
source_repetition
segment
```

`NO_MISSION` is used for TAKEOFF/LANDING.

## Ready-to-use cropped StaticTrajectory

`CroppedNetTrajectory` now also contains:

```text
bool trajectory_materialized
static_trajectory_manager/StaticTrajectory trajectory
```

MISSION retained edges are grouped by original mission index and split at every
removed edge/connectivity gap. Every surviving continuous run becomes one new
element of `trajectory.mission[]`.

Example:

```text
original mission[0]:
A -> B -> C -> D -> E -> F

retained:
A -> B

E -> F
```

becomes:

```text
trajectory.mission[0]:
A -> B

trajectory.mission[1]:
E -> F
```

No artificial `B -> E` connector is created.

The original metadata is preserved; only takeoff/mission[]/landing geometry is
replaced.

TAKEOFF and LANDING are still single polylines in `StaticTrajectory`. Therefore
`trajectory_materialized=false` only when either phase is split into more than
one retained run. The exact lattice crop remains available in that exceptional
case.

## Mission identity

`NetTrajectorySegment` does not yet expose `mission_index` directly. This node
recovers it from `original_phase_segment_index` and the source-edge counts of
the original `trajectory.mission[]`.

The repetition number is removed from the geometry de-duplication key, so only
the first geometric copy is kept.

## Supervision output

```text
/requested_supervision_trajectories
deconfliction_manager/msg/RequestedSupervisionTrajectoryArray
```

Each item contains the complete ORIGINAL `detected_collision` plus the
authoritative `cropped_trajectory`.

Downstream `supervision_trajectory_manager_node` should consume directly:

```text
requested.cropped_trajectory.trajectory
```

when:

```text
complete == true
trajectory_materialized == true
```

It should no longer reconstruct the crop itself.

## Manual markers

Manual-adjustment geometry is now drawn with `LINE_LIST` over the internal edges
of takeoff, every mission[i], and landing. No visual connectors are introduced
between independent mission components.

## Build

```bash
cd ~/a_space_ws

rm -rf build/deconfliction_manager
rm -rf install/deconfliction_manager

colcon build --symlink-install \
  --packages-up-to deconfliction_manager

source install/setup.bash
```

Because the deconfliction messages changed, rebuild downstream consumers after
this package.
