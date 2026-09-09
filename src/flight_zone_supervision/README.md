# flight_zone_supervision 0.6.0

This version keeps the existing `supervision_node` command/state machine but
moves UAS-to-UAS geometric acquisition completely to `distance_control_node`.

## Node namespace

The node is still instantiated once per flight zone:

```text
/<flight_zone_id>/supervision_node
```

Example:

```text
/inspection_1/supervision_node
```

The `flight_zone_id` parameter is therefore still mandatory.

## Inputs

The node keeps:

```text
/available_static_trajectories
/supervised_trajectories
/active_trajectories
```

and adds:

```text
/uas_related_distance
```

Type:

```text
distance_control/msg/UasRelatedDistanceArray
```

It no longer subscribes to:

```text
/<flight_zone>/<uas>/zone_status
```

and has no dependency on `flight_zone_msgs`.

## Responsibilities

### distance_control_node

```text
/active_trajectories
      ↓
dynamic zone_status subscriptions
      ↓
pairwise XY distance
      ↓
/uas_related_distance
```

### supervision_node

```text
/active_trajectories
/supervised_trajectories
/uas_related_distance
      ↓
identify supervised/counterpart UAS
      ↓
read supplied distance_xy_m and positions
      ↓
same SupervisionControl state machine as before
```

## Pair association

`/supervised_trajectories` supplies:

```text
supervised trajectory_id
supervised uas_namespace
conflicting_trajectory_ids[]
```

`/active_trajectories` is still used to resolve each active
`conflicting_trajectory_id` to its `uas_namespace`.

Then `/uas_related_distance` is searched for the exact unordered UAS pair in
this node's `flight_zone_id`.

Therefore no trajectory ID needs to be added to the distance message.

## Deconfliction state machine

The command sequence is unchanged:

```text
MONITORING
    |
    | distance_xy < 1.5 m
    v
WAITING_BOTH_PAUSE
    |
    | PAUSE supervised
    | PAUSE counterpart
    | wait ACK both
    v
WAITING_ELEVATE
    |
    | ELEVATE=3 supervised
    | wait ACK
    | verify supplied supervised_position.z
    v
WAITING_COUNTERPART_RESUME
    |
    | RESUME counterpart
    | wait ACK
    v
COUNTERPART_RUNNING
    |
    | keep reading distance_xy_m from /uas_related_distance
    | distance_xy > 2.0 m
    v
WAITING_DESCEND
    |
    | DESCEND=4 supervised
    | wait ACK
    | verify supplied supervised_position.z returned to z_ref
    v
WAITING_SUPERVISED_RESUME
    |
    | RESUME supervised
    | wait ACK
    v
CLEARED
```

Strict boundaries are preserved:

```text
distance_xy < 1.5 m  -> trigger
distance_xy > 2.0 m  -> clear
```

## ELEVATE / DESCEND physical verification

The old implementation read the supervised UAS Z coordinate directly from
`VehicleZoneStatus`.

This version uses the position already embedded in the matching
`distance_control/msg/UasPairDistance`.

At trigger:

```text
reference_z = supervised_position.z
```

After ELEVATE:

```text
supervised_position.z >=
reference_z + vertical_grid_step_m - vertical_position_tolerance_m
```

After DESCEND:

```text
abs(supervised_position.z - reference_z)
<= vertical_position_tolerance_m
```

So removal of direct `zone_status` subscriptions does not weaken the existing
vertical verification.

## SupervisionControl

Unchanged:

```text
EXECUTE = 0
PAUSE   = 1
RESUME  = 2
ELEVATE = 3
DESCEND = 4
STOP    = 5
```

The node keeps action clients at:

```text
/<flight_zone>/<uas>/supervision_control
```

## ACTIVE execution

The previous execution behavior is preserved:

- non-supervised ACTIVE trajectory -> execute ACTIVE trajectory;
- supervised ACTIVE trajectory -> execute the ORIGINAL full trajectory retained
  in `/supervised_trajectories`, not the cropped planning geometry.

## RViz2 markers

Effective marker topic:

```text
/<flight_zone>/supervision_markers
```

The node publishes:

```text
supervision/active_normal
supervision/active_supervised
supervision/collision_nodes
supervision/collision_segments
supervision/drone_states
supervision/drone_state_labels
supervision/security_links
supervision/security_pair_labels
```

### Drone state markers

Every tracked UAS belonging to trajectories in this flight zone gets a state
label whenever a fresh position is available from `/uas_related_distance`.

Examples:

```text
ua_ins_1 | SUPERVISED_MONITORING | cmd=NONE | ack=NONE
ua_ins_1 | SUPERVISED_PAUSE_PENDING | cmd=PAUSE | ack=NONE | IN_FLIGHT
ua_ins_1 | SUPERVISED_ELEVATED_HOLD | cmd=ELEVATE | ack=ELEVATE
ua_ins_2 | COUNTERPART_RUNNING | cmd=RESUME | ack=RESUME
ua_ins_1 | SUPERVISED_DESCEND_PENDING | cmd=DESCEND | ack=ELEVATE | IN_FLIGHT
ua_ins_1 | SUPERVISED_RESUMED_CLEARED | cmd=RESUME | ack=RESUME
```

Pair labels continue to expose the complete `PairMode` and current
`distance_xy_m`.

## Configuration

```yaml
/**:
  ros__parameters:
    available_static_trajectories_topic: /available_static_trajectories
    supervised_trajectories_topic: /supervised_trajectories
    active_trajectories_topic: /active_trajectories
    related_distance_topic: /uas_related_distance

    control_action_suffix: supervision_control

    pause_distance_xy_m: 1.5
    release_distance_xy_m: 2.0

    vertical_grid_step_m: 1.0
    vertical_position_tolerance_m: 0.15

    related_distance_timeout_s: 2.0

    evaluation_period_ms: 100
    action_wait_timeout_ms: 20

    markers_topic: supervision_markers

    trajectory_line_width: 0.10
    collision_segment_width: 0.20
    collision_node_scale: 0.34
    drone_scale: 0.42
    pair_line_width: 0.06
    text_height: 0.30
```

## Build order

`distance_control` must exist in the same sourced workspace because this package
now consumes its messages.

```bash
cd ~/a_space_ws

rm -rf build/distance_control install/distance_control
rm -rf build/flight_zone_supervision install/flight_zone_supervision

colcon build --symlink-install \
  --packages-up-to flight_zone_supervision

source install/setup.bash
```

## Launch

```bash
ros2 launch flight_zone_supervision \
  supervision_node.launch.py \
  flight_zone_id:=inspection_1
```

## Important note about single active UAS

The current `UasRelatedDistanceArray` expresses positions through pair records.
With fewer than two valid active UAS there is no pair and therefore no current
position available to this supervisor for a drone marker.

This does not affect collision supervision because collision logic necessarily
requires a supervised/counterpart pair.
