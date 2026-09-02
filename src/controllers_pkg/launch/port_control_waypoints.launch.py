#!/usr/bin/env python3

from __future__ import annotations

import os
import re
from typing import Any, Dict, List, Set

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    LogInfo,
    OpaqueFunction,
    TimerAction,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


_VALID_SEGMENT = re.compile(r"^[A-Za-z_][A-Za-z0-9_]*$")


def _as_bool(value: Any) -> bool:
    if isinstance(value, bool):
        return value

    return str(value).strip().lower() in {
        "1",
        "true",
        "yes",
        "on",
    }


def _load_config(path: str) -> Dict[str, Any]:
    with open(path, "r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream) or {}

    root = data.get("control_waypoints")
    if not isinstance(root, dict):
        raise RuntimeError(
            f"El YAML '{path}' no contiene la clave raíz "
            "'control_waypoints'."
        )

    return root


def _validate_ros_namespace(value: str, controller_id: str) -> str:
    namespace = value.strip().strip("/")

    if not namespace:
        raise RuntimeError(
            f"{controller_id}: ros_namespace no puede estar vacío."
        )

    segments = namespace.split("/")
    invalid = [
        segment
        for segment in segments
        if not _VALID_SEGMENT.fullmatch(segment)
    ]

    if invalid:
        raise RuntimeError(
            f"{controller_id}: ros_namespace='{value}' contiene "
            f"segmentos ROS inválidos: {invalid}."
        )

    return namespace


def _validate_flat_namespace(value: str, controller_id: str) -> str:
    namespace = value.strip().strip("/")

    if not _VALID_SEGMENT.fullmatch(namespace):
        raise RuntimeError(
            f"{controller_id}: px4_dds_namespace='{value}' debe ser "
            "un único segmento ROS válido, sin barras ni espacios."
        )

    return namespace


def _validate_relative_name(
    value: str,
    field_name: str,
    controller_id: str,
) -> str:
    name = value.strip()

    if not name:
        raise RuntimeError(
            f"{controller_id}: '{field_name}' no puede estar vacío."
        )

    if name.startswith("/"):
        raise RuntimeError(
            f"{controller_id}: '{field_name}' debe ser relativo."
        )

    if "//" in name:
        raise RuntimeError(
            f"{controller_id}: '{field_name}' contiene un segmento vacío."
        )

    return name.strip("/")


def _absolute_px4_topic(
    px4_dds_namespace: str,
    suffix: Any,
    field_name: str,
    controller_id: str,
) -> str:
    relative_suffix = _validate_relative_name(
        str(suffix),
        field_name,
        controller_id,
    )

    return f"/{px4_dds_namespace}/{relative_suffix}"


def _axis_parameters(
    defaults: Dict[str, Any],
    controller_cfg: Dict[str, Any],
    axis: str,
    fallback: Dict[str, float],
) -> Dict[str, float]:
    default_pid = defaults.get("pid", {}) or {}
    default_axis = default_pid.get(axis, {}) or {}

    controller_pid = controller_cfg.get("pid", {}) or {}
    controller_axis = controller_pid.get(axis, {}) or {}

    def resolve(name: str) -> float:
        return float(
            controller_axis.get(
                name,
                default_axis.get(name, fallback[name]),
            )
        )

    return {
        f"kp_{axis}": resolve("kp"),
        f"ki_{axis}": resolve("ki"),
        f"kd_{axis}": resolve("kd"),
        f"integral_limit_{axis}": resolve("integral_limit"),
    }


def _nested_value(
    defaults: Dict[str, Any],
    controller_cfg: Dict[str, Any],
    section: str,
    key: str,
    fallback: Any,
) -> Any:
    default_section = defaults.get(section, {}) or {}
    controller_section = controller_cfg.get(section, {}) or {}

    return controller_section.get(
        key,
        default_section.get(key, fallback),
    )


def _launch_setup(context, *_args, **_kwargs):
    config_file = LaunchConfiguration(
        "config_file"
    ).perform(context)

    start_delay_override = LaunchConfiguration(
        "start_delay_s"
    ).perform(context).strip()

    cfg = _load_config(config_file)
    defaults = cfg.get("defaults", {}) or {}
    controllers = cfg.get("controllers", [])

    if not isinstance(defaults, dict):
        raise RuntimeError(
            "control_waypoints.defaults debe ser un mapa YAML."
        )

    if not isinstance(controllers, list) or not controllers:
        raise RuntimeError(
            "control_waypoints.controllers debe ser una lista no vacía."
        )

    use_sim_time = _as_bool(
        cfg.get("use_sim_time", True)
    )

    package_name = str(
        defaults.get("package", "controllers_pkg")
    )
    executable = str(
        defaults.get(
            "executable",
            "control_waypoints_node",
        )
    )
    node_name = str(
        defaults.get(
            "node_name",
            "control_waypoints_node",
        )
    )

    default_start_delay = float(
        defaults.get("start_delay_s", 0.0)
    )
    if start_delay_override:
        default_start_delay = float(start_delay_override)

    stagger_s = float(
        defaults.get("stagger_s", 0.10)
    )

    default_px4_topics = defaults.get(
        "px4_topics",
        {},
    ) or {}
    default_ros_interfaces = defaults.get(
        "ros_interfaces",
        {},
    ) or {}

    seen_controller_ids: Set[str] = set()
    seen_ros_namespaces: Set[str] = set()
    seen_px4_namespaces: Set[str] = set()
    seen_target_systems: Set[int] = set()

    actions: List[Any] = [
        LogInfo(
            msg=(
                "Port control-waypoints bringup: "
                f"config='{config_file}'"
            )
        )
    ]

    enabled_index = 0

    for entry_index, controller_cfg in enumerate(controllers):
        if not isinstance(controller_cfg, dict):
            raise RuntimeError(
                f"controllers[{entry_index}] debe ser un mapa YAML."
            )

        controller_id = str(
            controller_cfg.get(
                "controller_id",
                f"controller_{entry_index + 1}",
            )
        ).strip()

        if not _VALID_SEGMENT.fullmatch(controller_id):
            raise RuntimeError(
                f"controller_id='{controller_id}' no es válido."
            )

        if controller_id in seen_controller_ids:
            raise RuntimeError(
                f"controller_id duplicado: '{controller_id}'."
            )
        seen_controller_ids.add(controller_id)

        if not _as_bool(
            controller_cfg.get("enabled", True)
        ):
            actions.append(
                LogInfo(
                    msg=f"{controller_id}: disabled."
                )
            )
            continue

        ros_namespace = _validate_ros_namespace(
            str(controller_cfg.get("ros_namespace", "")),
            controller_id,
        )
        px4_dds_namespace = _validate_flat_namespace(
            str(controller_cfg.get("px4_dds_namespace", "")),
            controller_id,
        )

        expected_flat_namespace = ros_namespace.replace("/", "_")
        if px4_dds_namespace != expected_flat_namespace:
            raise RuntimeError(
                f"{controller_id}: px4_dds_namespace debe ser "
                f"'{expected_flat_namespace}' para corresponder con "
                f"ros_namespace='{ros_namespace}', pero se recibió "
                f"'{px4_dds_namespace}'."
            )

        target_system = int(
            controller_cfg.get("target_system", 0)
        )
        if target_system < 1 or target_system > 255:
            raise RuntimeError(
                f"{controller_id}: target_system debe estar en [1, 255]."
            )

        if ros_namespace in seen_ros_namespaces:
            raise RuntimeError(
                f"ros_namespace duplicado: '{ros_namespace}'."
            )
        if px4_dds_namespace in seen_px4_namespaces:
            raise RuntimeError(
                "px4_dds_namespace duplicado: "
                f"'{px4_dds_namespace}'."
            )
        if target_system in seen_target_systems:
            raise RuntimeError(
                f"target_system duplicado: {target_system}."
            )

        seen_ros_namespaces.add(ros_namespace)
        seen_px4_namespaces.add(px4_dds_namespace)
        seen_target_systems.add(target_system)

        controller_px4_topics = controller_cfg.get(
            "px4_topics",
            {},
        ) or {}
        controller_ros_interfaces = controller_cfg.get(
            "ros_interfaces",
            {},
        ) or {}

        def px4_suffix(key: str, fallback: str) -> Any:
            return controller_px4_topics.get(
                key,
                default_px4_topics.get(key, fallback),
            )

        def ros_interface(key: str, fallback: str) -> str:
            value = controller_ros_interfaces.get(
                key,
                default_ros_interfaces.get(key, fallback),
            )
            return _validate_relative_name(
                str(value),
                key,
                controller_id,
            )

        parameters: Dict[str, Any] = {
            "use_sim_time": use_sim_time,
            "target_system": target_system,

            "vehicle_command_topic": _absolute_px4_topic(
                px4_dds_namespace,
                px4_suffix(
                    "vehicle_command",
                    "fmu/in/vehicle_command",
                ),
                "vehicle_command",
                controller_id,
            ),
            "offboard_control_mode_topic": _absolute_px4_topic(
                px4_dds_namespace,
                px4_suffix(
                    "offboard_control_mode",
                    "fmu/in/offboard_control_mode",
                ),
                "offboard_control_mode",
                controller_id,
            ),
            "trajectory_setpoint_topic": _absolute_px4_topic(
                px4_dds_namespace,
                px4_suffix(
                    "trajectory_setpoint",
                    "fmu/in/trajectory_setpoint",
                ),
                "trajectory_setpoint",
                controller_id,
            ),
            "vehicle_status_topic": _absolute_px4_topic(
                px4_dds_namespace,
                px4_suffix(
                    "vehicle_status",
                    "fmu/out/vehicle_status",
                ),
                "vehicle_status",
                controller_id,
            ),

            "odom_topic": ros_interface(
                "odom_topic",
                "odom",
            ),
            "marker_topic": ros_interface(
                "marker_topic",
                "mission_markers",
            ),
            "arm_takeoff_service": ros_interface(
                "arm_takeoff_service",
                "arm_takeoff",
            ),
            "follow_waypoints_action": ros_interface(
                "follow_waypoints_action",
                "follow_waypoints",
            ),

            "marker_frame_suffix": str(
                controller_cfg.get(
                    "marker_frame_suffix",
                    defaults.get(
                        "marker_frame_suffix",
                        "odom",
                    ),
                )
            ),
            "marker_frame": str(
                controller_cfg.get(
                    "marker_frame",
                    defaults.get("marker_frame", ""),
                )
            ),

            "mission_frame": str(
                controller_cfg.get(
                    "mission_frame",
                    defaults.get("mission_frame", "map"),
                )
            ),
            "odom_frame_suffix": str(
                controller_cfg.get(
                    "odom_frame_suffix",
                    defaults.get("odom_frame_suffix", "odom"),
                )
            ),
            "transform_timeout_s": float(
                controller_cfg.get(
                    "transform_timeout_s",
                    defaults.get("transform_timeout_s", 0.20),
                )
            ),

            "max_vx": float(
                _nested_value(
                    defaults,
                    controller_cfg,
                    "limits",
                    "max_vx",
                    1.5,
                )
            ),
            "max_vy": float(
                _nested_value(
                    defaults,
                    controller_cfg,
                    "limits",
                    "max_vy",
                    1.5,
                )
            ),
            "max_vz": float(
                _nested_value(
                    defaults,
                    controller_cfg,
                    "limits",
                    "max_vz",
                    0.8,
                )
            ),
            "max_speed_xy": float(
                _nested_value(
                    defaults,
                    controller_cfg,
                    "limits",
                    "max_speed_xy",
                    1.8,
                )
            ),
            "control_period_ms": float(
                _nested_value(
                    defaults,
                    controller_cfg,
                    "timing",
                    "control_period_ms",
                    50.0,
                )
            ),
            "takeoff_tolerance_xy": float(
                _nested_value(
                    defaults,
                    controller_cfg,
                    "takeoff",
                    "tolerance_xy",
                    0.30,
                )
            ),
            "takeoff_tolerance_z": float(
                _nested_value(
                    defaults,
                    controller_cfg,
                    "takeoff",
                    "tolerance_z",
                    0.20,
                )
            ),
        }

        parameters.update(
            _axis_parameters(
                defaults,
                controller_cfg,
                "x",
                {
                    "kp": 0.9,
                    "ki": 0.0,
                    "kd": 0.15,
                    "integral_limit": 2.0,
                },
            )
        )
        parameters.update(
            _axis_parameters(
                defaults,
                controller_cfg,
                "y",
                {
                    "kp": 0.9,
                    "ki": 0.0,
                    "kd": 0.15,
                    "integral_limit": 2.0,
                },
            )
        )
        parameters.update(
            _axis_parameters(
                defaults,
                controller_cfg,
                "z",
                {
                    "kp": 0.8,
                    "ki": 0.0,
                    "kd": 0.10,
                    "integral_limit": 2.0,
                },
            )
        )

        controller_node = Node(
            package=str(
                controller_cfg.get(
                    "package",
                    package_name,
                )
            ),
            executable=str(
                controller_cfg.get(
                    "executable",
                    executable,
                )
            ),
            namespace=ros_namespace,
            name=str(
                controller_cfg.get(
                    "node_name",
                    node_name,
                )
            ),
            output="screen",
            emulate_tty=True,
            parameters=[parameters],
        )

        controller_delay = float(
            controller_cfg.get(
                "start_delay_s",
                default_start_delay +
                enabled_index * stagger_s,
            )
        )

        actions.append(
            TimerAction(
                period=controller_delay,
                actions=[controller_node],
            )
        )
        actions.append(
            LogInfo(
                msg=(
                    f"{controller_id}: "
                    f"node='/{ros_namespace}/{node_name}', "
                    f"PX4='/{px4_dds_namespace}/fmu/...', "
                    f"target_system={target_system}, "
                    f"delay={controller_delay:.2f}s"
                )
            )
        )

        enabled_index += 1

    if enabled_index == 0:
        raise RuntimeError(
            "No hay ningún controlador habilitado."
        )

    return actions


def generate_launch_description():
    package_share = get_package_share_directory(
        "controllers_pkg"
    )
    default_config = os.path.join(
        package_share,
        "config",
        "port_control_waypoints.yaml",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=default_config,
                description=(
                    "YAML conjunto de los controladores "
                    "de la operación del puerto."
                ),
            ),
            DeclareLaunchArgument(
                "start_delay_s",
                default_value="",
                description=(
                    "Override opcional del retardo inicial común. "
                    "Vacío = defaults.start_delay_s del YAML."
                ),
            ),
            OpaqueFunction(
                function=_launch_setup
            ),
        ]
    )
