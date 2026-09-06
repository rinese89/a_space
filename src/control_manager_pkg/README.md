# control_manager_pkg 0.3.0 — multi-mission execution

Per-UAS ROS 2 Humble control manager adapted to:

```text
static_trajectory_manager/msg/StaticTrajectory
```

with:

```text
TrajectorySegment takeoff
TrajectorySegment[] mission
TrajectorySegment landing
```

## Main change

The previous control manager assumed:

```text
one mission polyline
```

and expanded:

```text
mission x repetitions
```

into one large `mission_path_map_`.

That is no longer valid because independent `mission[]` components must not be
concatenated into one artificial polyline.

The new execution order is:

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

Every `mission[i]` is sent as its own:

```text
controllers_pkg/action/FollowWaypoints
```

goal.

## No mission-array flattening

The node never creates:

```text
mission[0].back() -> mission[1].front()
```

inside its stored path representation.

`mission_path_map_` now contains only the mission component currently being
executed.

When `mission[i]` completes, a new FollowWaypoints action is dispatched for
`mission[i+1]`.

This is also important for path-deviation monitoring: the current vehicle pose
is compared only against the active mission component, not against a flattened
union of all mission geometries.

## FollowWaypoints height

`FollowWaypoints` exposes one scalar:

```text
height
```

per action.

The previous implementation therefore required the complete mission to have one
constant Z.

With multi-mission dispatch the rule is now:

```text
mission[0].z must be constant
mission[1].z must be constant
...
```

but different mission components may use different heights.

Example:

```yaml
mission:
  - x: [0.0, 5.0]
    y: [0.0, 0.0]
    z: [3.0, 3.0]

  - x: [5.0, 10.0]
    y: [5.0, 5.0]
    z: [6.0, 6.0]
```

is valid.

The first action is sent with:

```text
height = 3.0
```

and the second with:

```text
height = 6.0
```

A single `mission[i]` with changing Z is still rejected because the current
FollowWaypoints interface cannot encode it.

## Validation

A received EXECUTE trajectory now requires:

```text
valid takeoff
mission.size() >= 1
every mission[i] valid
valid landing
repetitions >= 1
```

Every mission component needs at least two finite XYZ points.

## Pause / resume

The execution cursor is now:

```text
current_repetition_
current_mission_index_
current_mission_waypoint_
```

When PAUSE cancels the active FollowWaypoints goal, the node retains all three
values.

RESUME therefore dispatches:

```text
the remaining waypoints of the same mission[i]
```

rather than restarting the whole multi-mission operation.

Once that component succeeds, normal sequencing continues with the following
mission component.

## Repetitions

`repetitions` applies to the complete ordered mission collection.

For:

```text
mission.size() = 3
repetitions = 2
```

execution is:

```text
r0/m0
r0/m1
r0/m2
r1/m0
r1/m1
r1/m2
```

Each item above is one independent FollowWaypoints goal.

## Path-deviation monitoring

The old implementation measured deviation against one flattened mission path.

The new implementation stores only the active `mission[i]` in
`mission_path_map_`.

Therefore:

```text
distance_to_mission_path()
```

cannot accidentally use a nearby segment from another mission component and
cannot construct an artificial line across a gap.

## STOP behavior

STOP semantics are unchanged.

During any mission component:

```text
cancel current FollowWaypoints
    ->
return toward landing entry
    ->
landing endpoint through ArmTakeoff
```

The remaining mission components/repetitions are abandoned.

## Supervised trajectories

`supervision_node` still sends the ORIGINAL complete trajectory to
`control_manager_node`.

The cropped multi-mission trajectory is used in planning/reservation upstream;
it is not substituted for the original execution trajectory.

The SupervisionControl EXECUTE transport already contains a complete
StaticTrajectoryArray, so no action-interface change was required here.

## State machine

The high-level states remain:

```text
WAITING_TRAJECTORY
WAITING_START
PAUSED_BEFORE_START
TAKEOFF_REQUEST
TAKEOFF_RESPONSE
MISSION_DISPATCH
MISSION_ACTIVE
PAUSE_CANCELING
PAUSED
STOP_CANCELING
RETURN_DISPATCH
RETURN_ACTIVE
LANDING_REQUEST
LANDING_RESPONSE
COMPLETED
FAILED
```

`MISSION_DISPATCH`/`MISSION_ACTIVE` now refer to the current pair:

```text
(repetition, mission_index)
```

rather than one globally flattened mission.

## Build

Because `StaticTrajectory.msg` changed, rebuild this package against the
migrated interface chain:

```bash
cd ~/a_space_ws

rm -rf build/control_manager_pkg
rm -rf install/control_manager_pkg

colcon build --symlink-install \
  --packages-up-to control_manager_pkg

source install/setup.bash
```

## Useful runtime log

Acquisition now reports:

```text
missions=<N>
repetitions=<R>
total_mission_points=<...>
```

Each FollowWaypoints action reports:

```text
repetition=<r>
mission=<i>/<N>
base_waypoint=<k>
```

which makes PAUSE/RESUME and multi-mission sequencing directly traceable.
