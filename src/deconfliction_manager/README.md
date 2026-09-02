# deconfliction_manager

ROS 2 Humble package containing:

```text
deconfliction_manager_node
```

## Purpose

The node classifies the authoritative snapshot received on:

```text
/detected_collision_trajectories
```

Type:

```text
collision_detection/msg/DetectedCollisionTrajectoryArray
```

into exactly two outputs:

```text
/requested_supervision_trajectories
/manual_adjustment_trajectories
```

Both outputs now use exactly the same type as the input:

```text
collision_detection/msg/DetectedCollisionTrajectoryArray
```

This preserves all collision evidence for downstream managers.

## Classification

For every `DetectedCollisionTrajectory`:

```text
collision_resource_count =
    collision_nodes.size()
  + collision_segments.size()
```

With the default:

```yaml
collision_resource_threshold: 5
```

classification is:

```text
resources < 5  -> /requested_supervision_trajectories
resources >= 5 -> /manual_adjustment_trajectories
```

The threshold comparison is strict.

## Preserved data

The node does not reduce either output to `StaticTrajectory`.

Every output entry keeps the complete original:

```text
DetectedCollisionTrajectory
```

including:

```text
trajectory
loaded_on_net
has_shared_net_space
conflicting_trajectory_ids[]
collision_nodes[]
collision_segments[]
```

This is required by `supervision_trajectory_manager_node`, which consumes:

```text
/requested_supervision_trajectories
```

and needs `collision_nodes[]` and `collision_segments[]` to generate the cropped
trajectory published on `/adjusted_trajectories`.

## Snapshot semantics

The input is authoritative.

Every new `/detected_collision_trajectories` snapshot completely rebuilds both
classification outputs. Therefore:

```text
trajectory disappears from input
        -> disappears from both outputs
```

and a trajectory can change classification if its current collision-resource
count changes.

## Ordering

Output arrays are deterministic:

```text
1. priority ascending
2. ua_id ascending
3. trajectory_id ascending
```

Lower numerical priority has higher precedence.

## QoS

Input and outputs use:

```text
RELIABLE
TRANSIENT_LOCAL
KeepLast(1)
```

The two classification snapshots and the RViz markers are also republished
every:

```yaml
publish_period_ms: 1000
```

Empty snapshots are intentionally published.

## RViz markers

Topic:

```text
/deconfliction_manager_markers
```

### Requested supervision

Only collision nodes are visualized:

```text
yellow/orange sphere + label
```

The label contains:

```text
trajectory_id | node=<node_id> | vs=<conflicting trajectory IDs>
```

### Manual adjustment

The complete original static trajectory is visualized in red together with its
trajectory ID.

Every marker snapshot starts with `DELETEALL` so stale markers are removed.

## Topics

### Input

```text
/detected_collision_trajectories
collision_detection/msg/DetectedCollisionTrajectoryArray
```

### Outputs

```text
/requested_supervision_trajectories
collision_detection/msg/DetectedCollisionTrajectoryArray
```

```text
/manual_adjustment_trajectories
collision_detection/msg/DetectedCollisionTrajectoryArray
```

```text
/deconfliction_manager_markers
visualization_msgs/msg/MarkerArray
```

## Configuration

```yaml
deconfliction_manager_node:
  ros__parameters:
    detected_collision_trajectories_topic: /detected_collision_trajectories

    requested_supervision_trajectories_topic: /requested_supervision_trajectories
    manual_adjustment_trajectories_topic: /manual_adjustment_trajectories

    deconfliction_markers_topic: /deconfliction_manager_markers

    collision_resource_threshold: 5
    publish_period_ms: 1000

    supervision_node_scale: 0.30
    supervision_text_height: 0.28

    manual_line_width: 0.14
    manual_text_height: 0.36
```

## Build

```bash
cd ~/a_space_ws

rm -rf build/deconfliction_manager
rm -rf install/deconfliction_manager

colcon build --symlink-install \
  --packages-select deconfliction_manager

source install/setup.bash
```

## Verify topic types

```bash
ros2 topic info /requested_supervision_trajectories
ros2 topic info /manual_adjustment_trajectories
```

Both should report:

```text
Type: collision_detection/msg/DetectedCollisionTrajectoryArray
```

## Downstream compatibility

`supervision_trajectory_manager_node` is directly compatible with the new
`/requested_supervision_trajectories` type.

Any node currently subscribing to `/manual_adjustment_trajectories` as:

```text
static_trajectory_manager/msg/StaticTrajectoryArray
```

must be updated to consume:

```text
collision_detection/msg/DetectedCollisionTrajectoryArray
```

and access the embedded static trajectory as:

```cpp
detected.trajectory
```
