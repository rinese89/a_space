# distance_control 0.1.0

ROS 2 Humble package that separates UAS-to-UAS distance acquisition from the
runtime supervision/deconfliction state machine.

## Responsibility

`distance_control_node` performs only this pipeline:

```text
/active_trajectories
        |
        | discover active UAS
        v
/<flight_zone>/<uas>/zone_status
        |
        | fresh positions
        v
all unique active-UAS pairs
        |
        | XY distance
        v
/uas_related_distance

and RViz2 markers on:

/uas_related_distance/markers
```

It does not depend on `SupervisionControl` and does not send PAUSE, RESUME,
ELEVATE, DESCEND or STOP commands.

## Dynamic discovery

For every `StaticTrajectory` in `/active_trajectories`, the node reads:

```text
ua_id
uas_namespace
flight_zone_id
```

and dynamically creates one subscription to:

```text
/<flight_zone_id>/<uas_namespace>/zone_status
```

If the UAS disappears from the active snapshot, the corresponding subscription
is removed. Multiple active records for the same UAS still create only one
subscription.

## Pair generation

Only fresh `zone_status` samples are used. With `N` valid UAS the node generates
all unique unordered pairs:

```text
N=2 -> 1 pair
N=3 -> 3 pairs
N=4 -> 6 pairs
N=5 -> 10 pairs
```

For each pair:

```text
distance_xy_m = sqrt((x1-x2)^2 + (y1-y2)^2)
```

Z is deliberately excluded.

A pair is published only when both `zone_status.header.frame_id` values are
non-empty and equal.

## Output

Topic:

```text
/uas_related_distance
```

Type:

```text
distance_control/msg/UasRelatedDistanceArray
```

Snapshot fields:

```text
std_msgs/Header header
uint32 active_uas_count
uint32 valid_uas_count
uint32 valid_pair_count
distance_control/UasPairDistance[] pairs
```

Each pair contains both UAS identities, both current positions and
`distance_xy_m`.

## RViz2

Topic:

```text
/uas_related_distance/markers
```

Namespaces:

```text
distance_control/drones
distance_control/drone_labels
distance_control/pairs
distance_control/pair_labels
```

Each valid pair is shown as a line between both current UAS positions, with a
label such as:

```text
ua_ins_1 <-> ua_ins_2 | d_xy=1.42 m
```

## Configuration

```yaml
/**:
  ros__parameters:
    active_trajectories_topic: /active_trajectories
    zone_status_suffix: zone_status
    related_distance_topic: /uas_related_distance
    zone_status_timeout_s: 2.0
    evaluation_period_ms: 100
    markers_topic: /uas_related_distance/markers
    drone_scale: 0.42
    pair_line_width: 0.06
    text_height: 0.30
```

## Build

```bash
cd ~/a_space_ws
colcon build --symlink-install --packages-up-to distance_control
source install/setup.bash
```

## Run

```bash
ros2 launch distance_control distance_control.launch.py
```

Inspect:

```bash
ros2 topic echo /uas_related_distance
ros2 topic echo /uas_related_distance/markers
```
