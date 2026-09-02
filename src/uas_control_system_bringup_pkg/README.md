# uas_control_system_bringup_pkg

ROS 2 Humble launch-only package for the modules belonging to **one UAS**.

The resulting ROS namespace is always:

```text
/<flight_zone_id>/<ua_id>/...
```

The package includes:

- `uas_bringup_pkg/launch/uas_bringup.launch.py`
- `controllers_pkg/launch/control_waypoints.launch.py`
- `control_manager_pkg/launch/control_manager_node.launch.py`

`micro_ros_agent` and `ros_gz_bridge` are deliberately kept outside this
bringup because they belong to shared infrastructure.

## Arguments

### `ua_id`

Complete UAS ROS identifier, including its numeric suffix. Example:

```text
ua_ins_1
```

The wrapper derives:

```text
uas_namespace    = ua_ins_1
namespace_prefix = ua_ins_
numeric_id       = 1
```

The inherited multi-UAS launch files therefore receive runtime YAML data with:

```yaml
group:
  num_uas: 1
  start_id: 1
  namespace_prefix: ua_ins_
```

They consequently create **ua_ins_1**, never `ua_1`.

`ua_id` must end in a positive integer.

### `flight_zone_id`

Assigned flight-zone identifier, for example:

```text
inspection_1
```

With:

```text
ua_id:=ua_ins_1
flight_zone_id:=inspection_1
```

all per-UAS ROS entities use:

```text
/inspection_1/ua_ins_1
```

and PX4 uses the flat DDS namespace:

```text
/inspection_1_ua_ins_1
```

## Usage

```bash
ros2 launch uas_control_system_bringup_pkg \
  uas_control_system_bringup.launch.py \
  ua_id:=ua_ins_1 \
  flight_zone_id:=inspection_1
```

## Flight-zone assignment

The package is now single-UAS. Therefore the base YAML no longer contains an
`assignments:` map. It only keeps:

```yaml
flight_zone_assignment:
  required: true
  default_zone_id: test
```

The wrapper always supplies `flight_zone_id` explicitly to the inherited launch
files and also rewrites `default_zone_id` in their runtime YAML copies.

## Infrastructure exclusion

For the runtime copy consumed by `uas_bringup.launch.py`, the wrapper forces:

```yaml
micro_ros_agent:
  start: false

gazebo_bridge:
  start: false
```

and also passes:

```text
start_agent:=false
start_bridge:=false
```
