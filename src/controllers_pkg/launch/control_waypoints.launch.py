#!/usr/bin/env python3

from __future__ import annotations

import os
import re
from typing import Any, Dict

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


def _as_bool(value: Any) -> bool:
    if isinstance(value, bool):
        return value

    return str(value).strip().lower() in {
        "1",
        "true",
        "yes",
        "on",
    }


def _override(context, name: str, default: Any) -> Any:
    value = LaunchConfiguration(name).perform(context).strip()
    return default if value == "" else value


def _load_config(path: str) -> Dict[str, Any]:
    with open(path, "r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream) or {}

    if "uas_bringup" not in data:
        raise RuntimeError(
            f"El YAML '{path}' no contiene la clave raíz "
            "'uas_bringup'."
        )

    return data["uas_bringup"]


def _resolve_flight_zone_id(
    ua_id: int,
    uas_namespace: str,
    assignment_cfg: Dict[str, Any],
    launch_override: str,
) -> str:
    if launch_override:
        return launch_override

    assignments = assignment_cfg.get("assignments", {}) or {}
    if not isinstance(assignments, dict):
        raise RuntimeError(
            "flight_zone_assignment.assignments debe ser un mapa."
        )

    for key in (uas_namespace, str(ua_id), ua_id):
        if key in assignments:
            value = str(assignments[key]).strip()
            if value:
                return value

    return str(
        assignment_cfg.get("default_zone_id", "")
    ).strip()


def _validate_namespace_segment(value: str, field_name: str) -> None:
    if not re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", value):
        raise RuntimeError(
            f"'{field_name}' debe ser un segmento de namespace ROS "
            "válido. Valor recibido: "
            f"'{value}'."
        )


def _relative_topic_suffix(value: Any, field_name: str) -> str:
    topic = str(value).strip()

    if not topic:
        raise RuntimeError(
            f"'{field_name}' no puede estar vacío."
        )

    if topic.startswith("/"):
        raise RuntimeError(
            f"'{field_name}' debe ser un sufijo relativo. "
            "El launch añadirá el namespace DDS plano de PX4."
        )

    topic = topic.strip("/")

    if not topic or "//" in topic:
        raise RuntimeError(
            f"'{field_name}' no es un nombre relativo válido: "
            f"'{value}'."
        )

    return topic


def _absolute_px4_topic(
    px4_dds_namespace: str,
    topic_suffix: str,
) -> str:
    return f"/{px4_dds_namespace}/{topic_suffix}"


def _axis_parameters(
    controller_cfg: Dict[str, Any],
    axis: str,
    defaults: Dict[str, float],
) -> Dict[str, float]:
    pid_cfg = controller_cfg.get("pid", {})
    axis_cfg = pid_cfg.get(axis, {})

    return {
        f"kp_{axis}": float(
            axis_cfg.get("kp", defaults["kp"])
        ),
        f"ki_{axis}": float(
            axis_cfg.get("ki", defaults["ki"])
        ),
        f"kd_{axis}": float(
            axis_cfg.get("kd", defaults["kd"])
        ),
        f"integral_limit_{axis}": float(
            axis_cfg.get(
                "integral_limit",
                defaults["integral_limit"],
            )
        ),
    }


def _launch_setup(context, *_args, **_kwargs):
    config_file = LaunchConfiguration(
        "config_file"
    ).perform(context)

    cfg = _load_config(config_file)

    group_cfg = cfg.get("group", {})
    assignment_cfg = cfg.get(
        "flight_zone_assignment",
        {},
    )
    controller_cfg = cfg.get(
        "control_waypoints",
        {},
    )

    if not _as_bool(
        controller_cfg.get("enabled", True)
    ):
        return [
            LogInfo(
                msg="control_waypoints disabled in YAML."
            )
        ]

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
    flight_zone_override = LaunchConfiguration(
        "flight_zone_id"
    ).perform(context).strip()

    if num_uas < 1:
        raise RuntimeError(
            "num_uas debe ser mayor o igual que 1."
        )

    if start_id < 1:
        raise RuntimeError(
            "start_id debe ser mayor o igual que 1."
        )

    namespace_prefix = str(
        group_cfg.get("namespace_prefix", "ua_")
    )
    use_sim_time = _as_bool(
        cfg.get("use_sim_time", True)
    )

    start_delay_s = float(
        controller_cfg.get("start_delay_s", 0.0)
    )
    stagger_s = float(
        controller_cfg.get("stagger_s", 0.10)
    )
    target_system_offset = int(
        controller_cfg.get(
            "target_system_offset",
            0,
        )
    )

    limits_cfg = controller_cfg.get("limits", {})
    timing_cfg = controller_cfg.get("timing", {})
    takeoff_cfg = controller_cfg.get("takeoff", {})

    # El YAML conserva estos valores como sufijos relativos. Para cada UAS,
    # el launch construye los nombres absolutos:
    #
    #   /test_ua_1/fmu/in/vehicle_command
    #   /test_ua_1/fmu/out/vehicle_status
    px4_topic_suffixes = {
        "vehicle_command_topic": _relative_topic_suffix(
            controller_cfg.get(
                "vehicle_command_topic",
                "fmu/in/vehicle_command",
            ),
            "control_waypoints.vehicle_command_topic",
        ),
        "offboard_control_mode_topic": _relative_topic_suffix(
            controller_cfg.get(
                "offboard_control_mode_topic",
                "fmu/in/offboard_control_mode",
            ),
            "control_waypoints.offboard_control_mode_topic",
        ),
        "trajectory_setpoint_topic": _relative_topic_suffix(
            controller_cfg.get(
                "trajectory_setpoint_topic",
                "fmu/in/trajectory_setpoint",
            ),
            "control_waypoints.trajectory_setpoint_topic",
        ),
        "vehicle_status_topic": _relative_topic_suffix(
            controller_cfg.get(
                "vehicle_status_topic",
                "fmu/out/vehicle_status",
            ),
            "control_waypoints.vehicle_status_topic",
        ),
    }

    common_parameters: Dict[str, Any] = {
        "use_sim_time": use_sim_time,
        "odom_topic": str(
            controller_cfg.get(
                "odom_topic",
                "odom",
            )
        ),
        "marker_topic": str(
            controller_cfg.get(
                "marker_topic",
                "mission_markers",
            )
        ),
        "arm_takeoff_service": str(
            controller_cfg.get(
                "arm_takeoff_service",
                "arm_takeoff",
            )
        ),
        "follow_waypoints_action": str(
            controller_cfg.get(
                "follow_waypoints_action",
                "follow_waypoints",
            )
        ),
        "marker_frame_suffix": str(
            controller_cfg.get(
                "marker_frame_suffix",
                "odom",
            )
        ),
        "marker_frame": str(
            controller_cfg.get(
                "marker_frame",
                "",
            )
        ),
        "max_vx": float(
            limits_cfg.get("max_vx", 1.5)
        ),
        "max_vy": float(
            limits_cfg.get("max_vy", 1.5)
        ),
        "max_vz": float(
            limits_cfg.get("max_vz", 0.8)
        ),
        "max_speed_xy": float(
            limits_cfg.get("max_speed_xy", 1.8)
        ),
        "control_period_ms": float(
            timing_cfg.get(
                "control_period_ms",
                50.0,
            )
        ),
        "takeoff_tolerance_xy": float(
            takeoff_cfg.get(
                "tolerance_xy",
                0.30,
            )
        ),
        "takeoff_tolerance_z": float(
            takeoff_cfg.get(
                "tolerance_z",
                0.20,
            )
        ),
    }

    common_parameters.update(
        _axis_parameters(
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
    common_parameters.update(
        _axis_parameters(
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
    common_parameters.update(
        _axis_parameters(
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

    package_name = str(
        controller_cfg.get(
            "package",
            "controllers_pkg",
        )
    )
    executable = str(
        controller_cfg.get(
            "executable",
            "control_waypoints_node",
        )
    )
    node_name = str(
        controller_cfg.get(
            "node_name",
            "control_waypoints_node",
        )
    )

    actions = [
        LogInfo(
            msg=(
                "Control-waypoints bringup: "
                f"ids {start_id}.."
                f"{start_id + num_uas - 1}; "
                f"config='{config_file}'"
            )
        )
    ]

    for local_index in range(num_uas):
        ua_id = start_id + local_index
        uas_namespace = (
            f"{namespace_prefix}{ua_id}"
        )

        flight_zone_id = _resolve_flight_zone_id(
            ua_id,
            uas_namespace,
            assignment_cfg,
            flight_zone_override,
        )

        if not flight_zone_id:
            raise RuntimeError(
                f"No se ha asignado una flight zone "
                f"a '{uas_namespace}'."
            )

        _validate_namespace_segment(
            flight_zone_id,
            "flight_zone_id",
        )
        _validate_namespace_segment(
            uas_namespace,
            "uas_namespace",
        )

        namespace = (
            f"{flight_zone_id}/{uas_namespace}"
        )

        # Debe coincidir exactamente con PX4_UXRCE_DDS_NS calculado por
        # uas_bringup.launch.py.
        px4_dds_namespace = namespace.replace("/", "_")

        _validate_namespace_segment(
            px4_dds_namespace,
            "px4_dds_namespace",
        )

        target_system = (
            ua_id + target_system_offset
        )

        if target_system < 1 or target_system > 255:
            raise RuntimeError(
                f"target_system={target_system} no es "
                "válido para "
                f"'{namespace}'."
            )

        parameters = dict(common_parameters)
        parameters["target_system"] = target_system

        for parameter_name, topic_suffix in px4_topic_suffixes.items():
            parameters[parameter_name] = _absolute_px4_topic(
                px4_dds_namespace,
                topic_suffix,
            )

        controller_node = Node(
            package=package_name,
            executable=executable,
            namespace=namespace,
            name=node_name,
            output="screen",
            emulate_tty=True,
            parameters=[parameters],
        )

        actions.append(
            TimerAction(
                period=(
                    start_delay_s +
                    local_index * stagger_s
                ),
                actions=[controller_node],
            )
        )

        actions.append(
            LogInfo(
                msg=(
                    f"{namespace}: controller "
                    f"target_system={target_system}; "
                    f"PX4 DDS namespace='/{px4_dds_namespace}'"
                )
            )
        )

    return actions


def generate_launch_description():
    uas_package_share = get_package_share_directory(
        "controllers_pkg"
    )
    default_config = os.path.join(
        uas_package_share,
        "config",
        "controller_waypoints.yaml",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=default_config,
                description=(
                    "YAML maestro del UAS que contiene "
                    "group, flight_zone_assignment y "
                    "control_waypoints."
                ),
            ),
            DeclareLaunchArgument(
                "num_uas",
                default_value="",
                description=(
                    "Número de controladores. Vacío = "
                    "group.num_uas del YAML."
                ),
            ),
            DeclareLaunchArgument(
                "start_id",
                default_value="",
                description=(
                    "Primer id global. Vacío = "
                    "group.start_id del YAML."
                ),
            ),
            DeclareLaunchArgument(
                "flight_zone_id",
                default_value="",
                description=(
                    "Override de la flight zone para todos "
                    "los UAS. Vacío = asignación del YAML. "
                    "La configuración de prueba resuelve "
                    "el valor 'test'."
                ),
            ),
            OpaqueFunction(
                function=_launch_setup
            ),
        ]
    )
