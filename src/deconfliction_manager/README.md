# deconfliction_manager 0.5.0

ROS 2 Humble package implementing collision-count classification and the first
geometry crop for supervised trajectories.

## Inputs

```text
/detected_collision_trajectories
  collision_detection/msg/DetectedCollisionTrajectoryArray

/net_loaded_trajectories
  a_space_virtual_net/msg/NetLoadedTrajectoryArray
```

Both subscriptions use:

```text
RELIABLE + TRANSIENT_LOCAL + KeepLast(1)
```

`/detected_collision_trajectories` is the authoritative collision snapshot.

`/net_loaded_trajectories` provides the complete route already discretized into
adjacent virtual-net edges. It is used only to build the crop; the node does not
repeat the virtual-net projection algorithm.

## Classification change

The old implementation used:

```text
collision_nodes.size + collision_segments.size
```

as its threshold resource count.

This version uses only:

```text
number of UNIQUE collision node_id values
```

Therefore:

```text
unique_collision_nodes < collision_node_threshold
    -> REQUESTED_SUPERVISION

unique_collision_nodes >= collision_node_threshold
    -> MANUAL_ADJUSTMENT
```

Default:

```yaml
collision_node_threshold: 5
```

### Partial repetitions

A partial mission repetition may traverse the same virtual-net geometry several
times. It must not multiply the collision count.

The manager stores all `collision_nodes[].node_id` values in a set before
classification. The same lattice node is therefore counted once regardless of
how many repeated mission traversals touch it.

The `collision_segments[]` count is not part of the threshold.

## New `/requested_supervision_trajectories` type

The topic is now:

```text
/requested_supervision_trajectories
deconfliction_manager/msg/RequestedSupervisionTrajectoryArray
```

Each item contains:

```text
RequestedSupervisionTrajectory
├── detected_collision
│   └── collision_detection/msg/DetectedCollisionTrajectory
│       ├── trajectory
│       ├── loaded_on_net
│       ├── has_shared_net_space
│       ├── conflicting_trajectory_ids[]
│       ├── collision_nodes[]
│       └── collision_segments[]
│
└── cropped_trajectory
    └── deconfliction_manager/msg/CroppedNetTrajectory
        ├── retained_nodes[]
        ├── retained_segments[]
        ├── removed_node_ids[]
        ├── removed_segments[]
        └── removed_edge_ids[]
```

So the complete previous collision-detection payload is preserved, and the crop
is added alongside it.

## Why the crop is represented as virtual-net segments

A `static_trajectory_manager/msg/TrajectorySegment` is one contiguous x/y/z
polyline.

If a middle section is removed, for example:

```text
(3,3) -> (3,4) -> [removed 3,5 ... 3,8] -> (3,9) -> (3,10)
```

putting the remaining points into one `TrajectorySegment` would incorrectly
create a new artificial edge:

```text
(3,4) -> (3,9)
```

Therefore the crop is intentionally represented as independent
`NetTrajectorySegment` edges. It can represent disconnected surviving pieces
without reconnecting across the removed collision region.

## Exact crop rule

Let:

```text
C_nodes = all collision_nodes[].node_id
C_edges = all collision_segments[].edge_id
```

A complete virtual-net edge is removed if:

```text
edge_id in C_edges
OR
start_node_id in C_nodes
OR
end_node_id in C_nodes
```

The last two conditions remove the boundary edges adjacent to a collision node.

### Example

Complete discretized route:

```text
(3,3)
 -> (3,4)
 -> (3,5)
 -> (3,6)
 -> (3,7)
 -> (3,8)
 -> (3,9)
 -> (3,10)
```

Collision nodes:

```text
(3,5), (3,6), (3,7), (3,8)
```

Removed edges:

```text
(3,4) -> (3,5)
(3,5) -> (3,6)
(3,6) -> (3,7)
(3,7) -> (3,8)
(3,8) -> (3,9)
```

Retained geometry:

```text
(3,3) -> (3,4)

(3,9) -> (3,10)
```

The non-collision boundary nodes `(3,4)` and `(3,9)` remain in
`retained_nodes[]`.

## Geometry only once

The crop is a geometry representation, not a list of every repeated temporal
traversal.

`retained_segments[]` and `removed_segments[]` emit only the first occurrence
of each:

```text
(phase, edge_id)
```

Thus partial mission repetitions do not duplicate the same lattice geometry in
the crop.

The original `source_repetitions` value is retained in the message.

## Crop availability

If the collision snapshot arrives before the retained
`/net_loaded_trajectories` snapshot, the requested item is still published with:

```text
cropped_trajectory.complete = false
```

When the net snapshot arrives, the manager automatically recomputes and
republishes the same supervision request with:

```text
complete = true
```

Consumers must not use an incomplete crop for reinsertion.

## Manual output

The manual branch remains unchanged:

```text
/manual_adjustment_trajectories
collision_detection/msg/DetectedCollisionTrajectoryArray
```

## QoS

Outputs:

```text
/requested_supervision_trajectories
/manual_adjustment_trajectories
/deconfliction_manager_markers
```

use:

```text
RELIABLE + TRANSIENT_LOCAL + KeepLast(1)
```

and are also republished every `publish_period_ms`.

## Important downstream change

Because `/requested_supervision_trajectories` has a new message type,
`supervision_trajectory_manager_node` must be updated to subscribe to:

```text
deconfliction_manager/msg/RequestedSupervisionTrajectoryArray
```

instead of:

```text
collision_detection/msg/DetectedCollisionTrajectoryArray
```

It should consume `cropped_trajectory` directly rather than recomputing the crop
from the original sparse `StaticTrajectory`.

## Build

```bash
cd ~/a_space_ws

rm -rf build/deconfliction_manager
rm -rf install/deconfliction_manager

colcon build --symlink-install \
  --packages-up-to deconfliction_manager

source install/setup.bash
```

## Check topics

```bash
ros2 topic info /requested_supervision_trajectories
ros2 topic info /manual_adjustment_trajectories
```

Expected:

```text
/requested_supervision_trajectories
Type: deconfliction_manager/msg/RequestedSupervisionTrajectoryArray

/manual_adjustment_trajectories
Type: collision_detection/msg/DetectedCollisionTrajectoryArray
```
