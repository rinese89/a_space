# control_manager_node

ROS 2 Humble node that consumes `/active_trajectories` and dispatches the trajectory belonging to its own `/<flight_zone>/<uas>` namespace to `control_waypoints_node`.

## Execution sequence

1. Receives and stores the matching `StaticTrajectory` occurrence from `/active_trajectories`.
2. At exactly `operation_start_utc` (within `max_start_lateness_s`) sends the TAKEOFF endpoint through `controllers_pkg/srv/ArmTakeoff`.
3. Sends the expanded mission (`mission × repetitions`) through `controllers_pkg/action/FollowWaypoints`.
4. Uses action feedback (`pose`, `current_waypoint`) to monitor the mission corridor and to retain the current waypoint for pause/resume.
5. On normal mission completion sends the LANDING endpoint through `ArmTakeoff`.

The `ArmTakeoff` service is the existing controller contract. As in the previous injection manager, the landing phase is represented by sending the final landing endpoint to that service; this package does not add a new PX4 landing/disarm interface.

## Operator service

Relative service:

`trajectory_control`

Absolute example:

`/inspection_1/ua_ins_1/trajectory_control`

Type:

`control_manager_node/srv/ControlTrajectory`

Commands:

- `0` = PAUSE: cancel active mission action. `control_waypoints_node` enters HOLD at the current position.
- `1` = RESUME: sends only the remaining expanded mission, starting at the waypoint that was active when paused.
- `2` = STOP: cancel mission if needed, send an action to the first landing point, then send the final landing endpoint through `ArmTakeoff`.

Examples:

```bash
ros2 service call /inspection_1/ua_ins_1/trajectory_control \
  control_manager_node/srv/ControlTrajectory "{command: 0}"

ros2 service call /inspection_1/ua_ins_1/trajectory_control \
  control_manager_node/srv/ControlTrajectory "{command: 1}"

ros2 service call /inspection_1/ua_ins_1/trajectory_control \
  control_manager_node/srv/ControlTrajectory "{command: 2}"
```

## Feedback monitoring

The action feedback pose is interpreted in the UAS odom frame, transformed to the trajectory frame, and checked against the complete expanded mission polyline. There is no independent odometry health monitor in this package. A deviation larger than `max_path_deviation_m` for `deviation_hold_time_s` triggers the same total-stop return-and-land sequence.

## Launch

```bash
ros2 launch control_manager_node control_manager_node.launch.py \
  flight_zone_id:=inspection_1 uas_namespace:=ua_ins_1
```
