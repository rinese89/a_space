# control_manager_pkg 0.4.0

Per-UAS control manager for the current A-space supervision workflow.

## SupervisionControl contract

This package must be built against the updated `flight_zone_supervision`
interface:

```text
EXECUTE = 0
PAUSE   = 1
RESUME  = 2
ELEVATE = 3
DESCEND = 4
STOP    = 5
```

The operator compatibility service `ControlTrajectory.srv` is unchanged.

## Vertical deconfliction behavior

The normal mission cursor is:

```text
repetition
mission_index
mission_waypoint
```

PAUSE cancels the current mission `FollowWaypoints` goal and preserves that
cursor.

### ELEVATE = 3

ELEVATE is valid only for an acquired mission that is paused or whose PAUSE is
already being processed.

If ELEVATE arrives while PAUSE cancellation is still in flight, it is accepted
and queued.

Once the controller reaches HOLD:

1. The control manager captures the latest real mission feedback position in
   the trajectory/map frame.
2. That position becomes the stored vertical reference.
3. A one-point auxiliary `FollowWaypoints` goal is sent:

```text
x = reference.x
y = reference.y
z = reference.z + vertical_grid_step_m
```

4. `repetition`, `mission_index` and `mission_waypoint` are not modified.
5. After the auxiliary goal succeeds, the state returns to PAUSED and the UAS
   stays elevated.

Default:

```yaml
vertical_grid_step_m: 1.0
```

### DESCEND = 4

DESCEND uses the same stored reference:

```text
x = reference.x
y = reference.y
z = reference.z
```

The mission remains paused.

After successful descent:

- the stored vertical reference is cleared;
- the mission cursor is still unchanged;
- state returns to PAUSED.

If RESUME arrives while DESCEND is still completing, it is accepted and queued.
The remaining mission is dispatched only after DESCEND finishes successfully.

## State machine

```text
MISSION_ACTIVE
     |
     | PAUSE
     v
PAUSE_CANCELING
     |
     | ELEVATE may already be queued here
     v
PAUSED
     |
     | ELEVATE=3
     v
VERTICAL_ELEVATE_DISPATCH
     |
     v
VERTICAL_ELEVATE_ACTIVE
     |
     | success
     v
PAUSED (elevated)
     |
     | DESCEND=4
     v
VERTICAL_DESCEND_DISPATCH
     |
     v
VERTICAL_DESCEND_ACTIVE
     |
     | success
     v
PAUSED
     |
     | RESUME=2
     v
MISSION_DISPATCH
     |
     v
MISSION_ACTIVE
```

## Important safeguards

- `RESUME` is rejected while the UAS is elevated or elevating.
- `RESUME` may be queued while DESCEND is active.
- ELEVATE/DESCEND auxiliary actions do not advance mission indices.
- An auxiliary action rejected because the lower controller is not yet in HOLD
  is retried using `retry_period_ms`.
- Failed vertical auxiliary actions are retried while the mission remains
  paused.
- STOP can cancel an active vertical auxiliary goal and proceeds to the
  existing return-and-land sequence.

## Build order

Because `SupervisionControl.action` changed, rebuild the interface package and
this package together:

```bash
cd ~/a_space_ws

rm -rf build/flight_zone_supervision install/flight_zone_supervision
rm -rf build/control_manager_pkg install/control_manager_pkg

colcon build --symlink-install \
  --packages-up-to control_manager_pkg

source install/setup.bash
```

`--packages-up-to control_manager_pkg` should rebuild the updated
`flight_zone_supervision` dependency first when both packages are present in
the workspace.
