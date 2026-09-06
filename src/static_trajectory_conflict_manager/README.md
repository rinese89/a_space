# static_trajectory_conflict_manager 0.4.0

Adapted to:

```text
static_trajectory_manager/TrajectorySegment[] mission
```

## Geometry semantics

`takeoff`, every `mission[i]`, and `landing` are independent continuous
polylines.

The conflict manager generates only their internal edges.

It never creates:

```text
takeoff.back()        -> mission[0].front()
mission[i].back()     -> mission[i+1].front()
mission.back().back() -> landing.front()
```

Therefore a cropped trajectory such as:

```text
mission[0]: A -> B
mission[1]: E -> F
```

remains:

```text
A -> B

E -> F
```

and is never converted into:

```text
A -> B -> E -> F
```

## Partial repetitions

`repetitions >= 1` is still validated.

This node performs a Boolean spatial-overlap test. Repeating the exact same
mission geometry several times cannot change that Boolean result, so identical
edges are not duplicated for partial repetitions.

Temporal overlap still uses the existing operation interval and
`operation_frequency` logic.

## State signature

The FNV-1a trajectory signature now includes:

```text
mission.size()
mission[0]
mission[1]
...
```

in order.

Changes to the number, order or coordinates of mission polylines therefore
trigger reclassification.

## Arbitration behavior

The existing simplified behavior is preserved:

```text
REQUESTED
   |
   +-- no collision with current AVAILABLE --> AVAILABLE
   |
   +-- collision with current AVAILABLE ----> COLLISION
```

Only trajectories already classified AVAILABLE block following candidates.
Collision trajectories are output state only and do not block later
trajectories.

## Topics

```text
/requested_static_trajectories
  static_trajectory_manager/msg/StaticTrajectoryArray

/available_static_trajectories
  static_trajectory_manager/msg/StaticTrajectoryArray

/collision_static_trajectories
  static_trajectory_conflict_manager/msg/CollisionStaticTrajectoryArray
```

QoS remains RELIABLE + TRANSIENT_LOCAL + KeepLast(1).

## Build

Because `StaticTrajectory.msg` changed, clean both interface-dependent packages:

```bash
cd ~/a_space_ws

rm -rf build/static_trajectory_manager
rm -rf install/static_trajectory_manager
rm -rf build/static_trajectory_conflict_manager
rm -rf install/static_trajectory_conflict_manager

colcon build --symlink-install \
  --packages-up-to static_trajectory_conflict_manager

source install/setup.bash
```
