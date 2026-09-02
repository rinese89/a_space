# flight_zone_global_manager

Globalización de la primera capa de gestión de flight zones.

## Ejecutables

- `flight_zone_server_node`: carga un único YAML con todas las flight zones y publica `/flight_zones` y `/flight_zones/markers`.
- `flight_zone_monitor_node`: un único monitor descubre todos los UAS mediante `/tf`, infiere su flight zone asignada del primer segmento de namespace y publica estado por UAS.

## Asociación UAS -> zona

En esta fase se conserva la convención existente:

```text
<flight_zone_id>/<uas>/odom
    ->
<flight_zone_id>/<uas>/base_link
```

Ejemplos:

```text
inspection_1/ua_ins_1/odom -> inspection_1/ua_ins_1/base_link
logistic_1/ua_log_3/odom    -> logistic_1/ua_log_3/base_link
survillance_1/ua_sur_4/odom -> survillance_1/ua_sur_4/base_link
```

El monitor obtiene `flight_zone_id` directamente del namespace y busca esa zona en el último `FlightZoneArray` global.

## Salidas por UAS

```text
/<flight_zone>/<uas>/zone_status
/<flight_zone>/<uas>/inside_flight_zone
/<flight_zone>/<uas>/zone_status/markers
```

`zone_status` está fijado en código para evitar la variante errónea `status_zone`.

## Launch

```bash
ros2 launch flight_zone_global_manager flight_zone_server.launch.py
ros2 launch flight_zone_global_manager flight_zone_monitor.launch.py
ros2 launch flight_zone_global_manager flight_zone_system.launch.py
```

## Compilación

```bash
cd ~/a_space_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select flight_zone_global_manager --symlink-install
source install/setup.bash
```

## Alcance temporal

Esta versión todavía no añade intervalos temporales a `FlightZone` ni a las trayectorias. El objetivo de este paso es dejar la gestión geométrica y de monitorización global preparada para introducir después la dimensión temporal y la deconflicción 4D.
