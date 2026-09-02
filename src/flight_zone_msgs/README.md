# flight_zone_msgs

Paquete de interfaces ROS 2 Humble utilizado por el sistema A-space.

## Mensajes

- `PolyhedronFace`
- `FlightZone`
- `FlightZoneArray`
- `ZoneConflict`
- `ZoneConflictArray`
- `ZoneContainment`
- `VehicleZoneStatus`
- `TakeoffWaypoint`

`TakeoffWaypoint` se publica en `/<flight_zone_id>/takeoff_waypoints`.

## Compilacion

```bash
cd ~/a_space_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select flight_zone_msgs --symlink-install
source install/setup.bash
```

## Comprobacion

```bash
ros2 interface list | grep flight_zone_msgs
ros2 interface show flight_zone_msgs/msg/TakeoffWaypoint
```


## Dynamic takeoff interfaces

- `TakeoffTrajectory.msg`
- `TakeoffTrajectoryArray.msg`

These interfaces carry the takeoff path plus aligned geometric-conflict
metadata between the static supervisor, dynamic supervisor and mission manager.
