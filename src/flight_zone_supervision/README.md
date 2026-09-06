# flight_zone_supervision 0.4.0

Per-flight-zone runtime supervision adapted to multi-mission
`StaticTrajectory` and proximity-triggered safety supervision.

## Activation rule

The UAS-to-UAS safety-distance state machine no longer starts as soon as a
supervised/non-supervised pair is active.

It starts only when the UAS executing the supervised trajectory is at most:

```yaml
collision_monitor_activation_distance_m: 2.0
```

from at least one ORIGINAL collision segment contained in
`/supervised_trajectories`.

The distance uses the existing `use_3d_distance` option.

### Collision segments only

The activation calculation uses only:

```text
collision_segments[]
```

and deliberately ignores `collision_nodes[]`.

The later collision-clearance calculation remains unchanged and still uses the
complete collision region: nodes plus segments.

## Latched activation

Activation is stored in `PairState`:

```text
safety_monitoring_active
activation_segment_distance_m
```

Once the distance threshold is crossed, safety supervision remains active for
that trajectory pair until the pair disappears. Moving back above 2 m does not
deactivate it.

## Before activation

```text
supervised pair active
        |
        v
distance(supervised UAS, closest collision segment)
        |
        +-- > 2.0 m --> monitor only
        |              no pair PAUSE/RESUME
        |              no UAS-to-UAS safety check
        |
        +-- <= 2.0 m --> ARM safety supervision
```

## After activation

The existing two-stage state machine is preserved:

```text
ARM
 |
 v
initial PAUSE cycle for supervised UAS
 |
 v
UAS-to-UAS distance check
 |
 +-- d > security_distance_m
 |      -> supervised UAS may RESUME
 |
 +-- d <= security_distance_m
        -> supervised UAS PAUSED
        -> check whether separation improves
        |
        +-- improves
        |      -> continue primary supervision
        |
        +-- does not improve
               -> PAUSE counterpart
               -> wait for positive PAUSE acknowledgement
               -> RESUME supervised UAS
               -> hold counterpart
               -> release counterpart after collision clearance
```

`collision_monitor_activation_distance_m` and `security_distance_m` are
independent thresholds.

## Frame requirement

The activation distance is evaluated only when the supervised UAS
`VehicleZoneStatus.header.frame_id` matches the collision trajectory
`frame_id`.

A mismatch leaves the proximity gate unarmed and emits a throttled warning.

## Multi-mission adaptation

`StaticTrajectory` now contains:

```text
TrajectorySegment[] mission
```

The action path already transports the complete multi-mission trajectory by
copy, so the ORIGINAL supervised trajectory is still executed without
conversion.

Active trajectory RViz markers were updated from a single `LINE_STRIP` to
`LINE_LIST` built from the internal edges of:

```text
takeoff
mission[0]
mission[1]
...
mission[N-1]
landing
```

No artificial line is drawn between independent mission components.

## Diagnostic marker

Pair labels now expose:

```text
safety=WAITING
segment_d=<distance>
```

before activation and:

```text
safety=ARMED
```

after activation.

## Parameters

```yaml
collision_monitor_activation_distance_m: 2.0
security_distance_m: 1.0
use_3d_distance: true

distance_progress_timeout_s: 0.5
distance_progress_epsilon_m: 0.05

collision_exit_distance_m: 1.0
collision_exit_hysteresis_m: 0.20
```

## Build

```bash
cd ~/a_space_ws

rm -rf build/flight_zone_supervision
rm -rf install/flight_zone_supervision

colcon build --symlink-install \
  --packages-up-to flight_zone_supervision

source install/setup.bash
```

## Runtime check

Before the supervised UAS reaches 2 m from a collision segment, this pair
should not generate safety PAUSE/RESUME commands.

At activation the node logs:

```text
Safety-distance monitoring ARMED ...
distance_to_collision_segment=... <= 2.000 m
```
