# A-space / UAS bringup workspace

Workspace ROS 2 Humble dividido en dos niveles:

- `aspace_bringup_pkg`: procesos comunes del sistema: QGroundControl, Gazebo y RViz2.
- `uas_bringup_pkg`: PX4 SITL, Micro XRCE-DDS Agent, bridge Gazebo–ROS 2, odometría, URDF y TF de uno o varios UAS.

No existe lógica de líder/follower. Cada aeronave se identifica únicamente por:

```text
namespace = ua_<id>
id        = <id>
```

Ejemplos: `ua_1`/`1`, `ua_2`/`2`, etc.

## Dependencia externa

El workspace referencia el ejecutable:

```text
me_drone_pkg/drone_odom_broadcaster
```

Por tanto, `me_drone_pkg` debe encontrarse en el mismo workspace o en un overlay previamente compilado y cargado.

## Preparación

Edita las rutas locales en:

```text
src/aspace_bringup_pkg/config/aspace_bringup.yaml
src/uas_bringup_pkg/config/uas_bringup.yaml
```

En particular:

```yaml
/home/rinese/PX4-Autopilot
/home/rinese/QGroundControl-x86_64.AppImage
```

PX4 SITL debe haberse compilado al menos una vez:

```bash
cd ~/PX4-Autopilot
make px4_sitl_default
```

## Compilación

```bash
cd ~/aspace_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 1. Lanzar el sistema A-space

```bash
ros2 launch aspace_bringup_pkg aspace_bringup.launch.py
```

Opciones habituales:

```bash
ros2 launch aspace_bringup_pkg aspace_bringup.launch.py headless:=true
ros2 launch aspace_bringup_pkg aspace_bringup.launch.py start_qgc:=false
```

Gazebo se inicia de forma independiente. Los PX4 posteriores usan `PX4_GZ_STANDALONE=1` y se conectan a esta instancia.

## 2. Lanzar el primer grupo de UAS

Ejemplo con tres UAS:

```bash
ros2 launch uas_bringup_pkg uas_bringup.launch.py \
  num_uas:=3 \
  start_id:=1
```

Se crean:

```text
/ua_1
/ua_2
/ua_3
```

La relación entre ids e instancias PX4 es:

```text
ua_1, id=1 -> px4 -i 0
ua_2, id=2 -> px4 -i 1
ua_3, id=3 -> px4 -i 2
```

Esto mantiene `MAV_SYS_ID == id` y fuerza el namespace DDS con `PX4_UXRCE_DDS_NS=ua_<id>`.

## 3. Lanzar un segundo grupo sin detener el primero

El segundo grupo debe comenzar en un id no utilizado. El agente y el bridge son servicios compartidos, por lo que no deben duplicarse:

```bash
ros2 launch uas_bringup_pkg uas_bringup.launch.py \
  num_uas:=2 \
  start_id:=4 \
  start_agent:=false \
  start_bridge:=false
```

Se añaden:

```text
/ua_4
/ua_5
```

La posición de aparición se calcula a partir del id global. Así, el segundo grupo continúa la cuadrícula del primero en lugar de reutilizar las posiciones iniciales.

## Tópicos y nodos esperados

Para `ua_2`:

```text
Nodo odometría:       /ua_2/drone_odom_broadcaster
Odometría:            /ua_2/odom
Robot state publisher:/ua_2/robot_state_publisher
TF estática:          map -> ua_2/odom
PX4 DDS:              /ua_2/fmu/in/* y /ua_2/fmu/out/*
```

El remapeo `/odom -> odom` convierte una publicación absoluta del nodo de odometría en un tópico relativo al namespace.

## Nota sobre frames TF

Los tópicos ROS 2 sí quedan aislados mediante namespaces. Los nombres de frame contenidos dentro de los mensajes no se modifican automáticamente. La arquitectura espera que `drone_odom_broadcaster` utilice frames compatibles con:

```text
ua_<id>/odom
ua_<id>/base_link
```

El `robot_state_publisher` ya aplica `frame_prefix=ua_<id>/`. Si el nodo de odometría todavía publica literalmente `odom` o `base_link`, conviene modificarlo para construir sus frame ids a partir del parámetro `ns`.

## Bridge Gazebo

`config/gz_bridge.yaml` contiene inicialmente únicamente `/clock`. Los sensores se añadirán cuando se definan los nombres finales de los modelos y de sus sensores en Gazebo.
