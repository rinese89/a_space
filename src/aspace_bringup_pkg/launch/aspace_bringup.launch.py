#!/usr/bin/env python3

from __future__ import annotations

import os
import shlex
from pathlib import Path
from typing import Any, Dict

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
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in {"1", "true", "yes", "on"}


def _optional_override(context, name: str, default: Any) -> Any:
    raw = LaunchConfiguration(name).perform(context).strip()
    return default if raw == "" else raw


def _load_yaml(path: str) -> Dict[str, Any]:
    with open(path, "r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream) or {}
    if "aspace" not in data:
        raise RuntimeError(f"El YAML '{path}' no contiene la clave raíz 'aspace'.")
    return data["aspace"]


def _launch_setup(context, *_args, **_kwargs):
    config_file = LaunchConfiguration("config_file").perform(context)
    cfg = _load_yaml(config_file)

    qgc_cfg = cfg.get("qgroundcontrol", {})
    gz_cfg = cfg.get("gazebo", {})
    rviz_cfg = cfg.get("rviz", {})

    start_qgc = _as_bool(_optional_override(context, "start_qgc", qgc_cfg.get("enabled", True)))
    start_gazebo = _as_bool(
        _optional_override(context, "start_gazebo", gz_cfg.get("enabled", True))
    )
    start_rviz = _as_bool(
        _optional_override(context, "start_rviz", rviz_cfg.get("enabled", True))
    )
    headless = _as_bool(
        _optional_override(context, "headless", gz_cfg.get("headless", False))
    )
    use_sim_time = _as_bool(cfg.get("use_sim_time", True))

    actions = [
        LogInfo(
            msg=(
                "A-space bringup: "
                f"QGC={start_qgc}, Gazebo={start_gazebo}, RViz2={start_rviz}, "
                f"headless={headless}"
            )
        )
    ]

    if start_gazebo:
        px4_dir = Path(os.path.expanduser(str(gz_cfg["px4_dir"]))).resolve()
        env_script_cfg = str(
            gz_cfg.get("gz_environment_script", "build/px4_sitl_default/rootfs/gz_env.sh")
        )
        env_script = Path(env_script_cfg)
        if not env_script.is_absolute():
            env_script = px4_dir / env_script

        world_file_cfg = str(gz_cfg.get("world_file", "")).strip()
        if world_file_cfg:
            world_file = Path(os.path.expanduser(world_file_cfg))
            if not world_file.is_absolute():
                world_file = px4_dir / world_file
        else:
            world_name = str(gz_cfg.get("world_name", "default"))
            world_file = px4_dir / "Tools" / "simulation" / "gz" / "worlds" / f"{world_name}.sdf"

        gz_args = ["gz", "sim"]
        if headless:
            gz_args.append("-s")
        if _as_bool(gz_cfg.get("run_immediately", True)):
            gz_args.append("-r")
        gz_args += ["-v", str(int(gz_cfg.get("verbosity", 3))), str(world_file)]

        # Se usa bash porque hay que cargar gz_env.sh antes de iniciar Gazebo.
        shell_command = (
            "set -e; "
            f"test -f {shlex.quote(str(env_script))} || "
            f"{{ echo 'No existe {shlex.quote(str(env_script))}. Compile PX4 SITL primero.' >&2; exit 2; }}; "
            f"source {shlex.quote(str(env_script))}; "
            f"exec {shlex.join(gz_args)}"
        )

        gazebo_process = ExecuteProcess(
            cmd=["bash", "-lc", shell_command],
            cwd=str(px4_dir),
            output="screen",
            name="aspace_gazebo",
        )
        actions.append(
            TimerAction(
                period=float(gz_cfg.get("startup_delay_s", 0.0)),
                actions=[gazebo_process],
            )
        )

    if start_qgc:
        qgc_executable = os.path.expanduser(str(qgc_cfg["executable"]))
        qgc_process = ExecuteProcess(
            cmd=[qgc_executable],
            output="screen",
            name="qgroundcontrol",
        )
        actions.append(
            TimerAction(
                period=float(qgc_cfg.get("startup_delay_s", 2.0)),
                actions=[qgc_process],
            )
        )

    if start_rviz:
        rviz_file_cfg = str(rviz_cfg.get("config_file", "")).strip()
        if rviz_file_cfg:
            rviz_file = os.path.expanduser(rviz_file_cfg)
        else:
            package_share = get_package_share_directory("aspace_bringup_pkg")
            rviz_file = os.path.join(package_share, "rviz", "aspace.rviz")

        rviz_node = Node(
            package="rviz2",
            executable="rviz2",
            name="aspace_rviz2",
            output="screen",
            arguments=["-d", rviz_file],
            parameters=[{"use_sim_time": use_sim_time}],
        )
        actions.append(
            TimerAction(
                period=float(rviz_cfg.get("startup_delay_s", 3.0)),
                actions=[rviz_node],
            )
        )

    return actions


def generate_launch_description():
    package_share = get_package_share_directory("aspace_bringup_pkg")
    default_config = os.path.join(package_share, "config", "aspace_bringup.yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=default_config,
                description="YAML de configuración del sistema A-space.",
            ),
            DeclareLaunchArgument(
                "start_qgc",
                default_value="",
                description="Override opcional de qgroundcontrol.enabled.",
            ),
            DeclareLaunchArgument(
                "start_gazebo",
                default_value="",
                description="Override opcional de gazebo.enabled.",
            ),
            DeclareLaunchArgument(
                "start_rviz",
                default_value="",
                description="Override opcional de rviz.enabled.",
            ),
            DeclareLaunchArgument(
                "headless",
                default_value="",
                description="Override opcional de gazebo.headless.",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
