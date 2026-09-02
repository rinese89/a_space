# supervision_trajectory_manager

ROS 2 Humble package containing `supervision_trajectory_manager_node`.

## Inputs

### `/requested_supervision_trajectories`

Type:

```text
collision_detection/msg/DetectedCollisionTrajectoryArray
```

The node stores the complete original `DetectedCollisionTrajectory`, including:

```text
trajectory
loaded_on_net
has_shared_net_space
conflicting_trajectory_ids[]
collision_nodes[]
collision_segments[]
```

### `/available_static_trajectories`

Type:

```text
static_trajectory_manager/msg/StaticTrajectoryArray
```

This authoritative snapshot is used to verify that the cropped trajectory has
actually entered the available flow.

## Adjusted trajectory generation

For every requested supervision trajectory, the node constructs a cropped
`StaticTrajectory` by removing the original path resources corresponding to the
detected collision nodes/segments.

The cropped trajectory is published in:

```text
/adjusted_trajectories
```

Type:

```text
static_trajectory_manager/msg/StaticTrajectoryArray
```

The `static_trajectory_manager_node` consumes this update and replaces the
stored geometry.

The cropped geometry is internal state of this node. It is NOT the trajectory
published later on `/supervised_trajectories`.

## AVAILABLE verification

Each stored entry starts as:

```text
PENDING
```

It remains in `/adjusted_trajectories` until the same `trajectory_id` appears
in `/available_static_trajectories` with geometry matching the expected cropped
geometry.

Then:

```text
PENDING -> SUPERVISED
```

## `/supervised_trajectories`

Type:

```text
collision_detection/msg/DetectedCollisionTrajectoryArray
```

QoS:

```text
RELIABLE
TRANSIENT_LOCAL
KeepLast(1)
```

For each entry in state `SUPERVISED`, this topic publishes the ORIGINAL
`DetectedCollisionTrajectory` that was received from
`/requested_supervision_trajectories`.

Therefore its `trajectory` field contains the original, uncut geometry, while
its collision evidence is also preserved:

```text
collision_nodes[]
collision_segments[]
conflicting_trajectory_ids[]
```

The cropped trajectory is used only for insertion/verification in the automatic
flow.

## Supervised lifetime

A trajectory is published in `/supervised_trajectories` only while its expected
cropped geometry remains present in `/available_static_trajectories`.

If it disappears from `AVAILABLE`, or the available geometry no longer matches:

```text
SUPERVISED -> PENDING
```

It immediately disappears from `/supervised_trajectories` and its stored
cropped geometry is queued again in `/adjusted_trajectories`.

If it later becomes available again with the expected geometry, it returns to
`SUPERVISED`.

## Markers

Topic:

```text
/supervision_trajectory_manager_markers
```

Pending entries show the cropped trajectory. Confirmed supervised entries show
the original collision nodes and labels with the conflicting trajectory IDs.

## QoS

All state subscriptions and publishers use:

```text
RELIABLE
TRANSIENT_LOCAL
KeepLast(1)
```

Snapshots are also republished every second by default:

```yaml
publish_period_ms: 1000
```

## Build

```bash
cd ~/a_space_ws
rm -rf build/supervision_trajectory_manager
rm -rf install/supervision_trajectory_manager

colcon build --symlink-install \
  --packages-up-to supervision_trajectory_manager

source install/setup.bash
```

## Verify

```bash
ros2 topic info /supervised_trajectories
```

Expected:

```text
Type: collision_detection/msg/DetectedCollisionTrajectoryArray
```
