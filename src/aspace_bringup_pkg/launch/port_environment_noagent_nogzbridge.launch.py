#!/usr/bin/env python3

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _include_python_launch(
    launch_file: str,
    launch_arguments: dict[str, object] | None = None,
) -> IncludeLaunchDescription:
    """Crea la inclusión de un launch Python con argumentos opcionales."""
    arguments = launch_arguments or {}

    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_file),
        launch_arguments=arguments.items(),
    )


def generate_launch_description() -> LaunchDescription:
    aspace_share = get_package_share_directory("aspace_bringup_pkg")
    flight_zone_share = get_package_share_directory("flight_zone_global_manager")

    aspace_launch = os.path.join(
        aspace_share,
        "launch",
        "aspace_bringup.launch.py",
    )

    flight_zone_system_launch = os.path.join(
        flight_zone_share,
        "launch",
        "flight_zone_system.launch.py",
    )

    use_sim_time = LaunchConfiguration("use_sim_time")

    start_qgc = LaunchConfiguration("start_qgc")
    start_gazebo = LaunchConfiguration("start_gazebo")
    start_rviz = LaunchConfiguration("start_rviz")
    headless = LaunchConfiguration("headless")

    # ---------------------------------------------------------------------
    # 1. Infraestructura general A-space
    # ---------------------------------------------------------------------
    aspace_bringup = _include_python_launch(
        aspace_launch,
        {
            "start_qgc": start_qgc,
            "start_gazebo": start_gazebo,
            "start_rviz": start_rviz,
            "headless": headless,
        },
    )

    # ---------------------------------------------------------------------
    # 2. Sistema global de flight zones
    #
    # flight_zone_system.launch.py lanza una única instancia global de:
    #   - flight_zone_server_node
    #   - flight_zone_monitor_node
    # ---------------------------------------------------------------------
    flight_zone_system = _include_python_launch(
        flight_zone_system_launch,
        {
            "use_sim_time": use_sim_time,
        },
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description=(
                    "Utilizar el reloj de simulación en el sistema global "
                    "de flight zones."
                ),
            ),
            DeclareLaunchArgument(
                "start_qgc",
                default_value="",
                description=(
                    "Override opcional de QGroundControl. Vacío mantiene el "
                    "valor del YAML de aspace_bringup."
                ),
            ),
            DeclareLaunchArgument(
                "start_gazebo",
                default_value="",
                description=(
                    "Override opcional de Gazebo. Vacío mantiene el valor "
                    "del YAML de aspace_bringup."
                ),
            ),
            DeclareLaunchArgument(
                "start_rviz",
                default_value="",
                description=(
                    "Override opcional de RViz2. Vacío mantiene el valor "
                    "del YAML de aspace_bringup."
                ),
            ),
            DeclareLaunchArgument(
                "headless",
                default_value="",
                description=(
                    "Override opcional del modo headless. Vacío mantiene el "
                    "valor del YAML de aspace_bringup."
                ),
            ),
            LogInfo(
                msg=(
                    "Launching A-space Port environment with the global "
                    "flight-zone system."
                )
            ),
            aspace_bringup,
            TimerAction(
                period=1.0,
                actions=[flight_zone_system],
            ),
        ]
    )