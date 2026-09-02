#!/usr/bin/env python3

from __future__ import annotations

import os
import re
import shlex
from pathlib import Path
from typing import Any, Dict, List

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    LogInfo,
    OpaqueFunction,
    TimerAction,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _as_bool(value: Any) -> bool:
    """Convierte valores YAML/launch habituales a bool."""
    if isinstance(value, bool):
        return value

    return str(value).strip().lower() in {"1", "true", "yes", "on"}


def _override(context, name: str, default: Any) -> Any:
    """
    Devuelve el argumento del launch cuando no está vacío.
    En caso contrario utiliza el valor definido en el YAML.
    """
    raw = LaunchConfiguration(name).perform(context).strip()
    return default if raw == "" else raw


def _load_yaml(path: str) -> Dict[str, Any]:
    """Carga la configuración maestra del lanzamiento UAS."""
    with open(path, "r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream) or {}

    if "uas_bringup" not in data:
        raise RuntimeError(
            f"El YAML '{path}' no contiene la clave raíz 'uas_bringup'."
        )

    return data["uas_bringup"]


def _xyz(values: List[Any], field_name: str) -> List[float]:
    """Valida una lista tridimensional [x, y, z]."""
    if not isinstance(values, list) or len(values) != 3:
        raise RuntimeError(
            f"'{field_name}' debe ser una lista de tres elementos [x, y, z]."
        )

    return [float(value) for value in values]


def _resolve_flight_zone_id(
    ua_id: int,
    namespace: str,
    assignment_cfg: Dict[str, Any],
    launch_override: str,
) -> str:
    """
    Resuelve la flight zone asignada a un UAS.

    Orden de prioridad:
      1. Argumento launch flight_zone_id.
      2. Mapa YAML assignments usando namespace (ua_1).
      3. Mapa YAML assignments usando id ("1").
      4. YAML default_zone_id.
    """
    if launch_override:
        return launch_override.strip()

    assignments = assignment_cfg.get("assignments", {}) or {}
    if not isinstance(assignments, dict):
        raise RuntimeError(
            "flight_zone_assignment.assignments debe ser un mapa YAML."
        )

    candidate_keys = (
        namespace,
        str(ua_id),
        ua_id,
    )

    for key in candidate_keys:
        if key in assignments:
            value = str(assignments[key]).strip()
            if value:
                return value

    return str(
        assignment_cfg.get("default_zone_id", "")
    ).strip()


def _validate_flight_zone_id(
    flight_zone_id: str,
    ua_id: int,
    namespace: str,
    required: bool,
) -> None:
    """Valida que la asignación exista cuando se configura como obligatoria."""
    if required and not flight_zone_id:
        raise RuntimeError(
            f"No se ha asignado ninguna flight zone a {namespace} "
            f"(id={ua_id}). Indique flight_zone_id en el launch, "
            "default_zone_id en el YAML o una entrada en assignments."
        )

    if not flight_zone_id:
        return

    if not re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", flight_zone_id):
        raise RuntimeError(
            f"El flight_zone_id '{flight_zone_id}' asignado a {namespace} "
            "se utilizará como namespace ROS y debe ser un único segmento "
            "válido: letras, números y guiones bajos, sin barras ni espacios, "
            "y sin comenzar por un número."
        )


def _spawn_pose(ua_id: int, cfg: Dict[str, Any]) -> List[float]:
    """
    Calcula la posición inicial utilizando el identificador global del UAS.

    Esto permite lanzar grupos adicionales sin reutilizar las posiciones
    asignadas a grupos anteriores.
    """
    origin = _xyz(
        cfg.get("origin", [0.0, 0.0, 0.0]),
        "spawn.origin",
    )
    spacing = _xyz(
        cfg.get("spacing", [3.0, 3.0, 0.0]),
        "spawn.spacing",
    )
    row_length = max(1, int(cfg.get("row_length", 5)))

    global_index = ua_id - 1
    column = global_index % row_length
    row = global_index // row_length

    x = origin[0] + column * spacing[0]
    y = origin[1] + row * spacing[1]
    z = origin[2] + global_index * spacing[2]

    return [
        x,
        y,
        z,
        float(cfg.get("roll", 0.0)),
        float(cfg.get("pitch", 0.0)),
        float(cfg.get("yaw", 0.0)),
    ]


def _launch_setup(context, *_args, **_kwargs):
    config_file = LaunchConfiguration("config_file").perform(context)
    cfg = _load_yaml(config_file)

    group_cfg = cfg.get("group", {})
    px4_cfg = cfg.get("px4", {})
    spawn_cfg = cfg.get("spawn", {})
    agent_cfg = cfg.get("micro_ros_agent", {})
    bridge_cfg = cfg.get("gazebo_bridge", {})
    odom_cfg = cfg.get("drone_odom_broadcaster", {})
    rsp_cfg = cfg.get("robot_state_publisher", {})
    assignment_cfg = cfg.get("flight_zone_assignment", {})

    num_uas = int(
        _override(
            context,
            "num_uas",
            group_cfg.get("num_uas", 1),
        )
    )
    start_id = int(
        _override(
            context,
            "start_id",
            group_cfg.get("start_id", 1),
        )
    )
    start_agent = _as_bool(
        _override(
            context,
            "start_agent",
            agent_cfg.get("start", True),
        )
    )
    start_bridge = _as_bool(
        _override(
            context,
            "start_bridge",
            bridge_cfg.get("start", True),
        )
    )

    flight_zone_override = LaunchConfiguration(
        "flight_zone_id"
    ).perform(context).strip()

    flight_zone_required = _as_bool(
        assignment_cfg.get("required", True)
    )

    if num_uas < 1:
        raise RuntimeError("num_uas debe ser mayor o igual que 1.")

    if start_id < 1:
        raise RuntimeError("start_id debe ser mayor o igual que 1.")

    use_sim_time = _as_bool(cfg.get("use_sim_time", True))
    namespace_prefix = str(group_cfg.get("namespace_prefix", "ua_"))

    package_share = get_package_share_directory("uas_bringup_pkg")

    # -------------------------------------------------------------------------
    # URDF
    # -------------------------------------------------------------------------
    urdf_cfg = str(rsp_cfg.get("urdf_file", "")).strip()
    urdf_file = (
        os.path.expanduser(urdf_cfg)
        if urdf_cfg
        else os.path.join(package_share, "urdf", "x500_base.urdf")
    )

    with open(urdf_file, "r", encoding="utf-8") as stream:
        robot_description = stream.read()

    # -------------------------------------------------------------------------
    # Gazebo bridge
    # -------------------------------------------------------------------------
    bridge_file_cfg = str(bridge_cfg.get("config_file", "")).strip()
    bridge_file = (
        os.path.expanduser(bridge_file_cfg)
        if bridge_file_cfg
        else os.path.join(package_share, "config", "gz_bridge.yaml")
    )

    # -------------------------------------------------------------------------
    # PX4
    # -------------------------------------------------------------------------
    if "directory" not in px4_cfg:
        raise RuntimeError(
            "Falta el parámetro obligatorio 'uas_bringup.px4.directory'."
        )

    px4_dir = Path(
        os.path.expanduser(str(px4_cfg["directory"]))
    ).resolve()

    px4_binary_cfg = Path(
        str(
            px4_cfg.get(
                "binary",
                "build/px4_sitl_default/bin/px4",
            )
        )
    )
    px4_binary = (
        px4_binary_cfg
        if px4_binary_cfg.is_absolute()
        else px4_dir / px4_binary_cfg
    )

    actions = [
        LogInfo(
            msg=(
                f"UAS bringup: ids {start_id}..{start_id + num_uas - 1}; "
                f"namespaces {namespace_prefix}{start_id}.."
                f"{namespace_prefix}{start_id + num_uas - 1}; "
                f"agent={start_agent}; bridge={start_bridge}"
            )
        )
    ]

    # -------------------------------------------------------------------------
    # Componentes compartidos
    # -------------------------------------------------------------------------
    if start_agent:
        actions.append(
            Node(
                package="micro_ros_agent",
                executable="micro_ros_agent",
                name="micro_ros_agent",
                output="screen",
                arguments=[
                    str(agent_cfg.get("transport", "udp4")),
                    "--port",
                    str(int(agent_cfg.get("port", 8888))),
                ],
            )
        )

    if start_bridge:
        actions.append(
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name=str(
                    bridge_cfg.get(
                        "node_name",
                        "gz_parameter_bridge",
                    )
                ),
                output="screen",
                parameters=[
                    {"use_sim_time": use_sim_time},
                    {"config_file": bridge_file},
                ],
            )
        )

    px4_start_delay = float(
        px4_cfg.get("start_delay_s", 1.0)
    )
    px4_stagger = float(
        px4_cfg.get("stagger_s", 0.35)
    )
    odom_start_delay = float(
        odom_cfg.get("start_delay_s", 3.0)
    )
    odom_stagger = float(
        odom_cfg.get("stagger_s", 0.10)
    )

    # -------------------------------------------------------------------------
    # Instancias UAS
    # -------------------------------------------------------------------------
    for local_index in range(num_uas):
        ua_id = start_id + local_index

        # Namespace relativo propio del UAS, independiente de la zona.
        uas_namespace = f"{namespace_prefix}{ua_id}"

        assigned_flight_zone_id = _resolve_flight_zone_id(
            ua_id=ua_id,
            namespace=uas_namespace,
            assignment_cfg=assignment_cfg,
            launch_override=flight_zone_override,
        )

        _validate_flight_zone_id(
            flight_zone_id=assigned_flight_zone_id,
            ua_id=ua_id,
            namespace=uas_namespace,
            required=flight_zone_required,
        )

        # Namespace ROS jerárquico definitivo:
        #
        #   logistic_1/ua_1
        #   logistic_1/ua_2
        #   inspection_1/ua_3
        #
        # Si se permite un UAS sin asignación, conserva únicamente ua_N.
        namespace = (
            f"{assigned_flight_zone_id}/{uas_namespace}"
            if assigned_flight_zone_id
            else uas_namespace
        )

        # PX4/uXRCE-DDS no debe recibir un namespace jerárquico con '/'.
        # Se aplana el namespace ROS completo:
        #
        #   test/ua_1 -> test_ua_1
        #   logistic_1/ua_3 -> logistic_1_ua_3
        px4_dds_namespace = namespace.replace("/", "_")

        if not re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", px4_dds_namespace):
            raise RuntimeError(
                f"El namespace DDS de PX4 calculado '{px4_dds_namespace}' "
                "no es un único segmento ROS válido."
            )

        actions.append(
            LogInfo(
                msg=(
                    f"{uas_namespace}: assigned flight zone = "
                    f"'{assigned_flight_zone_id or '<unassigned>'}'; "
                    f"ROS namespace = '/{namespace}'; "
                    f"PX4 DDS namespace = '/{px4_dds_namespace}'"
                )
            )
        )

        # Con instance_offset=-1:
        #   ua_1, id=1 -> px4 -i 0
        #   ua_2, id=2 -> px4 -i 1
        instance_offset = int(
            px4_cfg.get("instance_offset", -1)
        )
        px4_instance = ua_id + instance_offset

        if px4_instance < 0:
            raise RuntimeError(
                f"La instancia PX4 calculada para {px4_dds_namespace} "
                f"es negativa: {px4_instance}."
            )

        pose = _spawn_pose(ua_id, spawn_cfg)
        pose_text = ",".join(
            f"{value:.6f}" for value in pose
        )

        env_parts = [
            (
                "PX4_SYS_AUTOSTART="
                f"{int(px4_cfg.get('sys_autostart', 4001))}"
            ),
            (
                "PX4_SIM_MODEL="
                f"{shlex.quote(str(px4_cfg.get('sim_model', 'gz_x500')))}"
            ),
            (
                "PX4_GZ_MODEL_POSE="
                f"{shlex.quote(pose_text)}"
            ),
            (
                # PX4 publicará directamente bajo el namespace plano:
                # /<flight_zone_id>_<ua_N>/fmu/...
                "PX4_UXRCE_DDS_NS="
                f"{shlex.quote(px4_dds_namespace)}"
            ),
        ]

        if _as_bool(
            px4_cfg.get("standalone_gazebo", True)
        ):
            env_parts.append("PX4_GZ_STANDALONE=1")

        shell_command = (
            "set -e; "
            f"test -x {shlex.quote(str(px4_binary))} || "
            "{ "
            f"echo 'No existe el binario PX4 {shlex.quote(str(px4_binary))}.' >&2; "
            "exit 2; "
            "}; "
            + " ".join(env_parts)
            + f" exec {shlex.quote(str(px4_binary))} -i {px4_instance}"
        )

        px4_process = ExecuteProcess(
            cmd=["bash", "-lc", shell_command],
            cwd=str(px4_dir),
            output="screen",
            name=f"px4_sitl_{namespace.replace('/', '_')}",
        )

        actions.append(
            TimerAction(
                period=(
                    px4_start_delay
                    + local_index * px4_stagger
                ),
                actions=[px4_process],
            )
        )

        # ---------------------------------------------------------------------
        # robot_state_publisher
        # Frames resultantes:
        #   logistic_1/ua_1/base_link
        #   logistic_1/ua_1/<resto_frames_URDF>
        # ---------------------------------------------------------------------
        if _as_bool(rsp_cfg.get("enabled", True)):
            actions.append(
                Node(
                    package="robot_state_publisher",
                    executable="robot_state_publisher",
                    namespace=namespace,
                    name="robot_state_publisher",
                    output="screen",
                    parameters=[
                        {"use_sim_time": use_sim_time},
                        {
                            "robot_description":
                            robot_description
                        },
                        {
                            "frame_prefix":
                            f"{namespace}/"
                        },
                        {
                            "publish_frequency":
                            float(
                                rsp_cfg.get(
                                    "publish_frequency",
                                    30.0,
                                )
                            )
                        },
                    ],
                )
            )

        # ---------------------------------------------------------------------
        # Transformación estática global calculada desde el spawn:
        #
        #   map -> <flight_zone_id>/ua_N/odom
        #
        # El frame odom de cada UAS se coloca en la misma posición utilizada
        # por PX4_GZ_MODEL_POSE. De este modo, la odometría local del dron parte
        # del punto donde el modelo fue creado en Gazebo.
        #
        # La rotación map -> odom es identidad. El drone_odom_broadcaster ya
        # convierte NED/FRD a ENU/FLU y publica la actitud del vehículo.
        # Aplicar aquí también el RPY del spawn duplicaría esa orientación.
        # ---------------------------------------------------------------------
        odom_frame = f"{namespace}/odom"

        actions.append(
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                namespace=namespace,
                name="map_to_odom",
                output="screen",
                arguments=[
                    "--x",
                    str(pose[0]),
                    "--y",
                    str(pose[1]),
                    "--z",
                    str(pose[2]),
                    "--roll",
                    "0.0",
                    "--pitch",
                    "0.0",
                    "--yaw",
                    "0.0",
                    "--frame-id",
                    "map",
                    "--child-frame-id",
                    odom_frame,
                ],
                parameters=[
                    {"use_sim_time": use_sim_time}
                ],
            )
        )

        actions.append(
            LogInfo(
                msg=(
                    f"{namespace}: TF map -> {odom_frame}; "
                    f"traslación=({pose[0]:.3f}, "
                    f"{pose[1]:.3f}, {pose[2]:.3f})"
                )
            )
        )

        # ---------------------------------------------------------------------
        # La flight zone forma parte del namespace de infraestructura del UAS.
        #
        # Ejemplo:
        #   namespace = logistic_1/ua_1
        #
        # La odometría sigue siendo independiente de la geometría de la zona,
        # pero sus nodos, tópicos, servicios y frames quedan agrupados bajo
        # dicho namespace jerárquico.
        # ---------------------------------------------------------------------

        # ---------------------------------------------------------------------
        # drone_odom_broadcaster
        #
        # El nodo se encuentra en uas_bringup_pkg y se lanza dentro del
        # namespace <flight_zone_id>/ua_N.
        #
        # Como utiliza nombres relativos, ROS 2 resuelve:
        #
        #   Entrada PX4:
        #   fmu/out/vehicle_odometry
        #       -> /<flight_zone_id>_<ua_N>/fmu/out/vehicle_odometry
        #
        #   odom
        #       -> /<flight_zone_id>/ua_N/odom
        #
        #   reset_odom_srv
        #       -> /<flight_zone_id>/ua_N/reset_odom_srv
        #
        # No se aplican remappings de /odom.
        # ---------------------------------------------------------------------
        if _as_bool(odom_cfg.get("enabled", True)):
            vehicle_odometry_suffix = str(
                odom_cfg.get(
                    "vehicle_odometry_topic",
                    "fmu/out/vehicle_odometry",
                )
            ).strip("/")

            if not vehicle_odometry_suffix:
                raise RuntimeError(
                    "drone_odom_broadcaster.vehicle_odometry_topic "
                    "no puede estar vacío."
                )

            if "//" in vehicle_odometry_suffix:
                raise RuntimeError(
                    "drone_odom_broadcaster.vehicle_odometry_topic "
                    "contiene un segmento vacío."
                )

            px4_vehicle_odometry_topic = (
                f"/{px4_dds_namespace}/{vehicle_odometry_suffix}"
            )

            odom_parameters = {
                "use_sim_time": use_sim_time,
                "ns": namespace,
                "id": ua_id,
                # Entrada PX4 absoluta bajo el namespace DDS plano.
                "vehicle_odometry_topic": px4_vehicle_odometry_topic,
                "odom_topic": str(
                    odom_cfg.get(
                        "odom_topic",
                        "odom",
                    )
                ),
                "reset_service": str(
                    odom_cfg.get(
                        "reset_service",
                        "reset_odom_srv",
                    )
                ),
                "odom_frame": str(
                    odom_cfg.get(
                        "odom_frame",
                        "odom",
                    )
                ),
                "base_frame": str(
                    odom_cfg.get(
                        "base_frame",
                        "base_link",
                    )
                ),
            }

            odom_node = Node(
                package="uas_bringup_pkg",
                executable="drone_odom_broadcaster",
                namespace=namespace,
                name="drone_odom_broadcaster",
                output="screen",
                emulate_tty=True,
                parameters=[odom_parameters],
            )

            actions.append(
                TimerAction(
                    period=(
                        odom_start_delay
                        + local_index * odom_stagger
                    ),
                    actions=[odom_node],
                )
            )

    return actions


def generate_launch_description():
    package_share = get_package_share_directory(
        "uas_bringup_pkg"
    )
    default_config = os.path.join(
        package_share,
        "config",
        "uas_bringup.yaml",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=default_config,
                description=(
                    "YAML maestro del lanzamiento UAS."
                ),
            ),
            DeclareLaunchArgument(
                "num_uas",
                default_value="",
                description=(
                    "Número de UAS. "
                    "Vacío = valor definido en el YAML."
                ),
            ),
            DeclareLaunchArgument(
                "start_id",
                default_value="",
                description=(
                    "Primer identificador global del grupo. "
                    "Vacío = valor definido en el YAML."
                ),
            ),
            DeclareLaunchArgument(
                "start_agent",
                default_value="",
                description=(
                    "Iniciar Micro XRCE-DDS Agent. "
                    "Vacío = valor definido en el YAML."
                ),
            ),
            DeclareLaunchArgument(
                "start_bridge",
                default_value="",
                description=(
                    "Iniciar ros_gz_bridge. "
                    "Vacío = valor definido en el YAML."
                ),
            ),
            DeclareLaunchArgument(
                "flight_zone_id",
                default_value="",
                description=(
                    "Flight zone asignada a todos los UAS de esta ejecución. "
                    "También se utiliza como namespace ROS padre de ua_N. "
                    "Vacío = resolver mediante assignments/default_zone_id "
                    "del YAML."
                ),
            ),
            OpaqueFunction(
                function=_launch_setup
            ),
        ]
    )