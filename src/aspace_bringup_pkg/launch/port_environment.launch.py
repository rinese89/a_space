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
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


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
    uas_bringup_share = get_package_share_directory("uas_bringup_pkg")

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

    default_bridge_config = os.path.join(
        uas_bringup_share,
        "config",
        "gz_bridge.yaml",
    )

    use_sim_time = LaunchConfiguration("use_sim_time")

    start_qgc = LaunchConfiguration("start_qgc")
    start_gazebo = LaunchConfiguration("start_gazebo")
    start_rviz = LaunchConfiguration("start_rviz")
    headless = LaunchConfiguration("headless")

    start_agent = LaunchConfiguration("start_agent")
    agent_transport = LaunchConfiguration("agent_transport")
    agent_port = LaunchConfiguration("agent_port")

    start_bridge = LaunchConfiguration("start_bridge")
    bridge_config_file = LaunchConfiguration("bridge_config_file")

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
    # 2. Micro XRCE-DDS Agent global
    #
    # Se lanza una única instancia para toda la infraestructura. Los UAS
    # individuales no deben arrancar su propio agent.
    # ---------------------------------------------------------------------
    micro_ros_agent = Node(
        package="micro_ros_agent",
        executable="micro_ros_agent",
        name="micro_ros_agent",
        output="screen",
        arguments=[
            agent_transport,
            "--port",
            agent_port,
        ],
        condition=IfCondition(start_agent),
    )

    # ---------------------------------------------------------------------
    # 3. Gazebo <-> ROS bridge global
    #
    # Se utiliza la misma configuración de bridge que antes pertenecía al
    # uas_bringup_pkg. Se mantiene fuera de cualquier namespace de UAS.
    # ---------------------------------------------------------------------
    gazebo_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_parameter_bridge",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"config_file": bridge_config_file},
        ],
        condition=IfCondition(start_bridge),
    )

    # ---------------------------------------------------------------------
    # 4. Sistema global de flight zones
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
                    "Utilizar el reloj de simulación en la infraestructura "
                    "global del puerto."
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
            DeclareLaunchArgument(
                "start_agent",
                default_value="true",
                description=(
                    "Lanzar una única instancia global de Micro XRCE-DDS Agent."
                ),
            ),
            DeclareLaunchArgument(
                "agent_transport",
                default_value="udp4",
                description=(
                    "Transporte utilizado por micro_ros_agent, por ejemplo udp4."
                ),
            ),
            DeclareLaunchArgument(
                "agent_port",
                default_value="8888",
                description="Puerto del Micro XRCE-DDS Agent.",
            ),
            DeclareLaunchArgument(
                "start_bridge",
                default_value="true",
                description=(
                    "Lanzar una única instancia global de ros_gz_bridge."
                ),
            ),
            DeclareLaunchArgument(
                "bridge_config_file",
                default_value=default_bridge_config,
                description=(
                    "Fichero YAML utilizado por ros_gz_bridge. Por defecto se "
                    "usa uas_bringup_pkg/config/gz_bridge.yaml."
                ),
            ),
            LogInfo(
                msg=(
                    "Launching A-space Port environment | "
                    "Micro XRCE-DDS Agent=global | ros_gz_bridge=global | "
                    "flight-zone system=global."
                )
            ),
            aspace_bringup,
            micro_ros_agent,
            TimerAction(
                period=0.5,
                actions=[gazebo_bridge],
            ),
            TimerAction(
                period=1.0,
                actions=[flight_zone_system],
            ),
        ]
    )
