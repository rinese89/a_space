# a_space_virtual_net

ROS 2 Humble package implementing the persistent Cartesian virtual net of the
A-space.

## New role

This revision removes the old `/a_space_virtual_net/free_routes` output.
Instead, every upstream static trajectory is projected onto the lattice and
published already segmented by virtual-net edges.

## Inputs

```text
/available_static_trajectories
/collision_static_trajectories
```

The node no longer subscribes to `/flight_zones`.

## Main output

```text
/net_loaded_trajectories
```

Type:

```text
a_space_virtual_net/msg/NetLoadedTrajectoryArray
```

The output is a `RELIABLE + TRANSIENT_LOCAL + KeepLast(1)` authoritative
snapshot and is also republished every second by default.

## Projection

The complete operation is expanded as:

```text
TAKEOFF -> MISSION x repetitions -> LANDING
```

Every original finite segment is clipped to the configured grid volume,
its endpoints are snapped to the nearest lattice nodes and it is represented
as a deterministic sequence of adjacent X/Y/Z edges.

Therefore `/net_loaded_trajectories` contains the route actually loaded on the
virtual net, not merely the original continuous waypoints.

## NetTrajectorySegment

Every discretized edge publishes:

```text
edge_id
axis
start_node_id
end_node_id
start
end
length_m
phase
original_phase_segment_index
net_segment_index
```

The pair:

```text
phase + original_phase_segment_index
```

links every grid edge back to the corresponding segment of the expanded
original trajectory. `net_segment_index` is the order inside the complete
projected route.

## NetLoadedTrajectory

Each trajectory contains:

```text
source_state
trajectory
segments[]
```

`source_state` is one of:

```text
AVAILABLE
COLLISION
```

The original `StaticTrajectory` is preserved together with its lattice
projection.

If an inconsistent upstream state contains the same `trajectory_id` in both
input topics, `COLLISION` wins.

## Node IDs

For grid index `(x,y,z)`:

```text
node_id = z * nodes_y * nodes_x + y * nodes_x + x
```

For a fixed origin, dimensions and `density_net`, IDs are deterministic.

## Virtual-net state

The previous structured state remains available on:

```text
/a_space_virtual_net/state
```

It tracks occupancy and capacity of every edge.

With the new simplified conflict manager, effective edge states are:

```text
FREE
AVAILABLE
COLLISION
```

`COLLISION_CLEAR` remains in the message only for source compatibility.

## RViz markers

Published on:

```text
/a_space_virtual_net/markers
```

There are only two conceptual marker classes:

1. Free net edges: gray, thin.
2. Loaded trajectories: one thicker `LINE_LIST` per trajectory, each
   `trajectory_id` assigned a deterministic distinct color.

The trajectory color does not encode AVAILABLE/COLLISION. That information is
carried by `source_state` in `/net_loaded_trajectories`.

## Initial configuration

```yaml
size_x: 20.0
size_y: 20.0
size_z: 10.0
density_net: 1.0
publish_period_ms: 1000
```

## Build

```bash
cd ~/a_space_ws
rm -rf build/a_space_virtual_net install/a_space_virtual_net
colcon build --symlink-install --packages-select a_space_virtual_net
source install/setup.bash
```

## Launch

```bash
ros2 launch a_space_virtual_net a_space_virtual_net.launch.py
```

## Inspect

```bash
ros2 topic echo /net_loaded_trajectories
```

This topic is intended to become the geometric input of the next
`static_trajectory_collision_clasiffier_node`.
