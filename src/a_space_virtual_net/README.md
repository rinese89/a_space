# a_space_virtual_net 0.4.0

ROS 2 Humble package implementing the persistent Cartesian virtual net of the
A-space, adapted to the new multi-mission `StaticTrajectory` interface.

## StaticTrajectory change

Upstream now publishes:

```text
TrajectorySegment takeoff
TrajectorySegment[] mission
TrajectorySegment landing
```

instead of one single `mission` segment.

## Inputs

```text
/available_static_trajectories
  static_trajectory_manager/msg/StaticTrajectoryArray

/collision_static_trajectories
  static_trajectory_conflict_manager/msg/CollisionStaticTrajectoryArray
```

Both inputs remain authoritative retained snapshots.

## Main output

```text
/net_loaded_trajectories
a_space_virtual_net/msg/NetLoadedTrajectoryArray
```

The output remains:

```text
RELIABLE
TRANSIENT_LOCAL
KeepLast(1)
```

and is also republished periodically.

## Multi-mission projection

A complete operation is now interpreted as:

```text
TAKEOFF

repetition 0:
  mission[0]
  mission[1]
  ...
  mission[N-1]

repetition 1:
  mission[0]
  mission[1]
  ...
  mission[N-1]

...

LANDING
```

However, each `mission[i]` is an **independent continuous polyline**.

The virtual net projects only edges that exist inside each component.

It never creates a connector between:

```text
takeoff.back()        -> mission[0].front()
mission[i].back()     -> mission[i+1].front()
mission[N-1].back()   -> mission[0].front() of the next repetition
mission.back().back() -> landing.front()
```

### Example

Input:

```text
mission[0]:
(3,3) -> (3,4)

mission[1]:
(3,9) -> (3,10)
```

The projected net contains only the lattice routes corresponding to:

```text
(3,3) -> (3,4)

(3,9) -> (3,10)
```

It never recreates:

```text
(3,4) -> ... -> (3,9)
```

This is essential for supervised cropped trajectories.

## Partial repetitions

Unlike the static conflict manager and runtime server, this node deliberately
keeps the temporal expansion produced by:

```text
repetitions
```

because `/net_loaded_trajectories` represents every traversal loaded onto the
virtual net.

Therefore, if:

```yaml
repetitions: 3
```

the mission collection is projected three times.

The downstream `deconfliction_manager_node` is responsible for collapsing
repeated collision geometry when it counts unique `node_id` values.

## Geometry discretization

Every internal continuous source edge is:

1. clipped against the configured virtual-net volume;
2. snapped to its nearest lattice nodes;
3. converted to a deterministic adjacent X/Y/Z lattice path;
4. published as one or more `NetTrajectorySegment` messages.

The virtual-net discretization itself is unchanged.

## `original_phase_segment_index`

The existing message interface is retained.

For MISSION, the index is now global and monotonically increasing across all
mission components and repetitions.

Example:

```text
mission[0] has 2 source edges
mission[1] has 3 source edges
repetitions = 2
```

The MISSION source indices become:

```text
repetition 0
  mission[0] -> 0,1
  mission[1] -> 2,3,4

repetition 1
  mission[0] -> 5,6
  mission[1] -> 7,8,9
```

These consecutive numbers are identifiers only.

There is **no implied geometric connection** between index `1` and `2`, or
between `4` and `5`.

Connectivity is defined by:

```text
start_node_id
end_node_id
```

## `net_segment_index`

`net_segment_index` remains the order of emitted lattice edges in the complete
projection stream.

Again, consecutive values do not imply continuity. Consumers must use node IDs
when they need to reconstruct continuous runs.

This is already compatible with the supervision crop logic, which checks both
ordering and node connectivity.

## NetLoadedTrajectory

Each output item still contains:

```text
source_state
trajectory
segments[]
```

The complete new multi-mission `StaticTrajectory` is preserved in
`trajectory`.

## Edge occupancy

Available and collision trajectories are registered on the same persistent
virtual-net edges as before.

Effective edge states remain:

```text
FREE
AVAILABLE
COLLISION
```

Collision state wins when an inconsistent upstream snapshot contains the same
trajectory ID in both AVAILABLE and COLLISION.

## RViz

The existing trajectory markers already use:

```text
Marker::LINE_LIST
```

and are built directly from `NetTrajectorySegment[]`.

Therefore disconnected mission components remain visually disconnected without
any additional marker workaround.

## Build

Because `static_trajectory_manager/StaticTrajectory.msg` changed, clean this
package and its upstream interface packages before compiling:

```bash
cd ~/a_space_ws

rm -rf build/static_trajectory_manager \
       build/static_trajectory_conflict_manager \
       build/a_space_virtual_net

rm -rf install/static_trajectory_manager \
       install/static_trajectory_conflict_manager \
       install/a_space_virtual_net

colcon build --symlink-install \
  --packages-up-to a_space_virtual_net

source install/setup.bash
```

## Inspect

```bash
ros2 topic echo /net_loaded_trajectories
```

For a trajectory with several `mission[]` components, verify that no lattice
segments exist across the intended gaps.
