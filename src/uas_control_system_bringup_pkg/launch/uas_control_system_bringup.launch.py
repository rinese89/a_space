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


def _controller_takeoff_settings(
    source_path: str,
) -> Dict[str, Any]:
    """Return effective TAKEOFF-retry settings used by control_waypoints."""
    data = _load_yaml(source_path)
    root = _root_uas_bringup(data, source_path)

    controller_cfg = root.get("control_waypoints", {}) or {}
    if not isinstance(controller_cfg, dict):
        raise RuntimeError(
            f"El YAML '{source_path}' contiene 'control_waypoints' inválido."
        )

    takeoff_cfg = controller_cfg.get("takeoff", {}) or {}
    if not isinstance(takeoff_cfg, dict):
        raise RuntimeError(
            f"El YAML '{source_path}' contiene 'control_waypoints.takeoff' inválido."
        )

    # These defaults intentionally match the current controllers_pkg launch.
    return {
        "reach_timeout_s": float(
            takeoff_cfg.get("reach_timeout_s", 15.0)
        ),
        "arm_retry_period_s": float(
            takeoff_cfg.get("arm_retry_period_s", 2.0)
        ),
        "ground_settle_s": float(
            takeoff_cfg.get("ground_settle_s", 1.0)
        ),
        "max_retries": int(
            takeoff_cfg.get("max_retries", 0)
        ),
    }


def _control_manager_settings(
    source_path: str,
) -> Dict[str, Any]:
    """Read the current standalone control_manager ROS-parameter YAML."""
    data = _load_yaml(source_path)

    wildcard = data.get("/**", {}) or {}
    if not isinstance(wildcard, dict):
        raise RuntimeError(
            f"El YAML '{source_path}' debe contener la clave '/**'."
        )

    params = wildcard.get("ros__parameters", {}) or {}
    if not isinstance(params, dict):
        raise RuntimeError(
            f"El YAML '{source_path}' debe contener '/**/ros__parameters'."
        )

    # Defaults match control_manager_pkg 0.4.x:
    # multi-mission execution + vertical deconfliction commands.
    settings = {
        "tick_period_ms": int(
            params.get("tick_period_ms", 100)
        ),
        "retry_period_ms": int(
            params.get("retry_period_ms", 500)
        ),
        "action_wait_timeout_ms": int(
            params.get("action_wait_timeout_ms", 50)
        ),
        "max_start_lateness_s": float(
            params.get("max_start_lateness_s", 5.0)
        ),
        "vertical_grid_step_m": float(
            params.get("vertical_grid_step_m", 1.0)
        ),
        "supervision_action_suffix": str(
            params.get(
                "supervision_action_suffix",
                "supervision_control",
            )
        ),
        "arm_takeoff_service_suffix": str(
            params.get(
                "arm_takeoff_service_suffix",
                "arm_takeoff",
            )
        ),
        "follow_waypoints_action_suffix": str(
            params.get(
                "follow_waypoints_action_suffix",
                "follow_waypoints",
            )
        ),
    }

    if settings["tick_period_ms"] <= 0:
        raise RuntimeError(
            f"El YAML '{source_path}' contiene tick_period_ms <= 0."
        )

    if settings["retry_period_ms"] <= 0:
        raise RuntimeError(
            f"El YAML '{source_path}' contiene retry_period_ms <= 0."
        )

    if settings["action_wait_timeout_ms"] < 0:
        raise RuntimeError(
            f"El YAML '{source_path}' contiene action_wait_timeout_ms < 0."
        )

    if settings["max_start_lateness_s"] < 0.0:
        raise RuntimeError(
            f"El YAML '{source_path}' contiene max_start_lateness_s < 0."
        )

    if settings["vertical_grid_step_m"] <= 0.0:
        raise RuntimeError(
            f"El YAML '{source_path}' contiene vertical_grid_step_m <= 0."
        )

    return settings


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

    controller_takeoff = _controller_takeoff_settings(
        controller_config_source
    )
    control_manager_settings = _control_manager_settings(
        control_manager_config
    )

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
                "control_manager_input=SupervisionControl action | "
                "mission_execution=multi_mission | "
                f"takeoff_reach_timeout={controller_takeoff['reach_timeout_s']:.2f}s | "
                f"arm_retry_period={controller_takeoff['arm_retry_period_s']:.2f}s | "
                f"ground_settle={controller_takeoff['ground_settle_s']:.2f}s | "
                f"takeoff_max_retries={controller_takeoff['max_retries']} | "
                f"control_manager_tick={control_manager_settings['tick_period_ms']}ms | "
                f"control_manager_retry={control_manager_settings['retry_period_ms']}ms | "
                f"vertical_grid_step={control_manager_settings['vertical_grid_step_m']:.2f}m | "
                "supervision_commands="
                "EXECUTE=0,PAUSE=1,RESUME=2,ELEVATE=3,DESCEND=4,STOP=5"
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
        # controllers_pkg current behavior:
        # - TAKEOFF endpoint timeout starts only after PX4 confirms ARMED.
        # - ARM is retried periodically when PX4 rejects it.
        # - OFFBOARD-loss recovery waits passively for PX4 landing/DISARMED.
        # - FollowWaypoints is accepted only from HOLD.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(control_waypoints_launch),
            launch_arguments={
                "config_file": runtime_controller_config,
                "num_uas": "1",
                "start_id": str(numeric_id),
                "flight_zone_id": flight_zone_id,
            }.items(),
        ),
        # control_manager_node current A-space contract (0.4.x):
        # - does not subscribe directly to /active_trajectories;
        # - receives SupervisionControl:
        #     EXECUTE=0, PAUSE=1, RESUME=2,
        #     ELEVATE=3, DESCEND=4, STOP=5;
        # - executes StaticTrajectory.mission[] component-by-component;
        # - repeats the complete mission[] collection according to repetitions;
        # - preserves repetition/mission/waypoint across PAUSE and the auxiliary
        #   vertical ELEVATE/DESCEND maneuver;
        # - ELEVATE moves one vertical_grid_step_m while the mission is paused;
        # - DESCEND returns to the stored pre-elevation vertical reference;
        # - retries FollowWaypoints until control_waypoints reaches HOLD.
        #
        # The vertical_grid_step_m value is a ROS parameter from
        # control_manager_config_file, not a separate child-launch argument.
        #
        # Action server:
        #   /<flight_zone>/<uas_namespace>/supervision_control
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
                description=(
                    "Configuración base para control_waypoints.launch.py. "
                    "Incluye TAKEOFF retry: reach_timeout_s, "
                    "arm_retry_period_s, ground_settle_s y max_retries."
                ),
            ),
            DeclareLaunchArgument(
                "control_manager_config_file",
                default_value=os.path.join(
                    package_share, "config", "control_manager.yaml"
                ),
                description=(
                    "Configuración actual del control_manager_node 0.4.x: "
                    "multi-mission, SupervisionControl y deconflicción vertical "
                    "(vertical_grid_step_m; ELEVATE=3/DESCEND=4)."
                ),
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
