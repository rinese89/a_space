#!/usr/bin/env python3

from __future__ import annotations

import copy
import os
import re
import tempfile
from pathlib import Path
from typing import Any, Dict, Tuple

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


_VALID_SEGMENT = re.compile(r"^[A-Za-z_][A-Za-z0-9_]*$")
_TRAILING_ID = re.compile(r"([0-9]+)$")


def _load_yaml(path: str) -> Dict[str, Any]:
    with open(path, "r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream) or {}

    if not isinstance(data, dict):
        raise RuntimeError(f"El YAML '{path}' debe contener un mapa en la raíz.")

    return data


def _root_uas_bringup(data: Dict[str, Any], path: str) -> Dict[str, Any]:
    root = data.get("uas_bringup")
    if not isinstance(root, dict):
        raise RuntimeError(
            f"El YAML '{path}' debe contener la clave raíz 'uas_bringup'."
        )
    return root


def _validate_segment(value: str, field_name: str) -> str:
    value = value.strip().strip("/")
    if not _VALID_SEGMENT.fullmatch(value):
        raise RuntimeError(
            f"'{field_name}' debe ser un único segmento ROS válido. "
            f"Valor recibido: '{value}'."
        )
    return value


def _resolve_uas_identity(ua_id_arg: str) -> Tuple[str, str, int]:
    """Return (uas_namespace, namespace_prefix, numeric_id).

    ``ua_id`` is intentionally the complete ROS UAS identifier, for example
    ``ua_ins_1``.  The trailing integer is extracted for PX4 instance,
    target_system and the inherited ``start_id`` argument.  Everything before
    that integer is the namespace prefix consumed by the legacy multi-UAS
    launch files.
    """
    uas_namespace = _validate_segment(ua_id_arg, "ua_id")

    match = _TRAILING_ID.search(uas_namespace)
    if not match:
        raise RuntimeError(
            "'ua_id' debe terminar en un identificador numérico, por ejemplo "
            "'ua_ins_1' o 'ua_3'."
        )

    numeric_id = int(match.group(1))
    if numeric_id < 1:
        raise RuntimeError("El identificador numérico derivado de 'ua_id' debe ser >= 1.")

    namespace_prefix = uas_namespace[: match.start(1)]
    if not namespace_prefix:
        raise RuntimeError(
            "'ua_id' debe contener un prefijo antes del identificador numérico."
        )

    # The prefix is not a complete ROS segment because it can end in '_'.
    # Validate the exact final segment instead and only reject malformed prefix
    # characters here.
    if not re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", namespace_prefix):
        raise RuntimeError(
            f"El prefijo derivado de 'ua_id' no es válido: '{namespace_prefix}'."
        )

    return uas_namespace, namespace_prefix, numeric_id


def _write_runtime_yaml(data: Dict[str, Any], stem: str) -> str:
    runtime_dir = Path(tempfile.gettempdir()) / "uas_control_system_bringup_pkg"
    runtime_dir.mkdir(parents=True, exist_ok=True)

    fd, path = tempfile.mkstemp(
        prefix=f"{stem}_",
        suffix=".yaml",
        dir=str(runtime_dir),
        text=True,
    )
    os.close(fd)

    with open(path, "w", encoding="utf-8") as stream:
        yaml.safe_dump(data, stream, sort_keys=False)

    return path


def _single_uas_config(
    source_path: str,
    namespace_prefix: str,
    numeric_id: int,
    flight_zone_id: str,
    disable_infrastructure: bool,
) -> str:
    """Create the exact one-UAS configuration consumed by inherited launches."""
    data = copy.deepcopy(_load_yaml(source_path))
    root = _root_uas_bringup(data, source_path)

    group = root.setdefault("group", {})
    group["num_uas"] = 1
    group["start_id"] = numeric_id
    group["namespace_prefix"] = namespace_prefix

    # The per-UAS wrapper supplies the flight zone explicitly.  There is no
    # longer a need for a map of assignments inside a single-UAS YAML.
    assignment = root.setdefault("flight_zone_assignment", {})
    assignment["required"] = True
    assignment["default_zone_id"] = flight_zone_id
    assignment.pop("assignments", None)

    if disable_infrastructure:
        # Agent and Gazebo bridge are shared infrastructure and must never be
        # started by the per-UAS control-system bringup.
        root.setdefault("micro_ros_agent", {})["start"] = False
        root.setdefault("gazebo_bridge", {})["start"] = False

    return _write_runtime_yaml(data, Path(source_path).stem)


def _launch_setup(context, *_args, **_kwargs):
    uas_namespace, namespace_prefix, numeric_id = _resolve_uas_identity(
        LaunchConfiguration("ua_id").perform(context)
    )
    flight_zone_id = _validate_segment(
        LaunchConfiguration("flight_zone_id").perform(context),
        "flight_zone_id",
    )

    uas_config_source = LaunchConfiguration("uas_config_file").perform(context)
    controller_config_source = LaunchConfiguration(
        "controller_config_file"
    ).perform(context)
    control_manager_config = LaunchConfiguration(
        "control_manager_config_file"
    ).perform(context)

    ros_namespace = f"{flight_zone_id}/{uas_namespace}"
    px4_dds_namespace = ros_namespace.replace("/", "_")

    runtime_uas_config = _single_uas_config(
        source_path=uas_config_source,
        namespace_prefix=namespace_prefix,
        numeric_id=numeric_id,
        flight_zone_id=flight_zone_id,
        disable_infrastructure=True,
    )

    runtime_controller_config = _single_uas_config(
        source_path=controller_config_source,
        namespace_prefix=namespace_prefix,
        numeric_id=numeric_id,
        flight_zone_id=flight_zone_id,
        disable_infrastructure=False,
    )

    uas_bringup_launch = os.path.join(
        get_package_share_directory("uas_bringup_pkg"),
        "launch",
        "uas_bringup.launch.py",
    )
    control_waypoints_launch = os.path.join(
        get_package_share_directory("controllers_pkg"),
        "launch",
        "control_waypoints.launch.py",
    )
    control_manager_launch = os.path.join(
        get_package_share_directory("control_manager_pkg"),
        "launch",
        "control_manager_node.launch.py",
    )

    return [
        LogInfo(
            msg=(
                "UAS control-system bringup | "
                f"UAS='{uas_namespace}' numeric_id={numeric_id} | "
                f"namespace_prefix='{namespace_prefix}' | "
                f"flight_zone='{flight_zone_id}' | "
                f"ROS namespace='/{ros_namespace}' | "
                f"PX4 DDS namespace='/{px4_dds_namespace}' | "
                "micro_ros_agent=EXTERNAL | ros_gz_bridge=EXTERNAL | "
                "control_manager_input=SupervisionControl action"
            )
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(uas_bringup_launch),
            launch_arguments={
                "config_file": runtime_uas_config,
                "num_uas": "1",
                "start_id": str(numeric_id),
                "start_agent": "false",
                "start_bridge": "false",
                "flight_zone_id": flight_zone_id,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(control_waypoints_launch),
            launch_arguments={
                "config_file": runtime_controller_config,
                "num_uas": "1",
                "start_id": str(numeric_id),
                "flight_zone_id": flight_zone_id,
            }.items(),
        ),
        # control_manager_node no longer subscribes to /active_trajectories.
        # It exposes:
        #   /<flight_zone>/<uas_namespace>/supervision_control
        # using flight_zone_supervision/action/SupervisionControl.
        # The flight-zone supervision node is responsible for forwarding
        # EXECUTE / PAUSE / RESUME / STOP commands to this action server.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(control_manager_launch),
            launch_arguments={
                "flight_zone_id": flight_zone_id,
                "uas_namespace": uas_namespace,
                "config_file": control_manager_config,
            }.items(),
        ),
    ]


def generate_launch_description():
    package_share = get_package_share_directory(
        "uas_control_system_bringup_pkg"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "ua_id",
                default_value="ua_1",
                description=(
                    "Identificador ROS completo del UAS. Debe terminar en su "
                    "id numérico, por ejemplo 'ua_ins_1'."
                ),
            ),
            DeclareLaunchArgument(
                "flight_zone_id",
                default_value="test",
                description=(
                    "Identificador de la flight zone asignada al UAS, por "
                    "ejemplo 'inspection_1'."
                ),
            ),
            DeclareLaunchArgument(
                "uas_config_file",
                default_value=os.path.join(
                    package_share, "config", "uas_bringup.yaml"
                ),
                description="Configuración base para uas_bringup.launch.py.",
            ),
            DeclareLaunchArgument(
                "controller_config_file",
                default_value=os.path.join(
                    package_share, "config", "controller_waypoints.yaml"
                ),
                description="Configuración base para control_waypoints.launch.py.",
            ),
            DeclareLaunchArgument(
                "control_manager_config_file",
                default_value=os.path.join(
                    package_share, "config", "control_manager_node.yaml"
                ),
                description="Configuración del control_manager_node.",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
