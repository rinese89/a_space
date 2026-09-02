#!/usr/bin/env python3

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


def generate_launch_description():
    package_share = get_package_share_directory("uas_bringup_pkg")

    uas_bringup_launch = os.path.join(
        package_share,
        "launch",
        "uas_bringup.launch.py",
    )

    default_inspection_config = os.path.join(
        package_share,
        "config",
        "port_inspection_uas_bringup.yaml",
    )
    default_logistics_config = os.path.join(
        package_share,
        "config",
        "port_logistics_uas_bringup.yaml",
    )
    default_survillance_config = os.path.join(
        package_share,
        "config",
        "port_survillance_uas_bringup.yaml",
    )

    inspection_config = LaunchConfiguration("inspection_config_file")
    logistics_config = LaunchConfiguration("logistics_config_file")
    survillance_config = LaunchConfiguration("survillance_config_file")

    start_agent = LaunchConfiguration("start_agent")
    start_bridge = LaunchConfiguration("start_bridge")

    inspection_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(uas_bringup_launch),
        launch_arguments={
            "config_file": inspection_config,

            # El primer grupo inicia los componentes compartidos.
            "start_agent": start_agent,
            "start_bridge": start_bridge,

            # Vacío: num_uas, start_id y flight_zone_id se leen del YAML.
            "num_uas": "",
            "start_id": "",
            "flight_zone_id": "",
        }.items(),
    )

    logistics_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(uas_bringup_launch),
        launch_arguments={
            "config_file": logistics_config,

            # Ya los inició el primer grupo.
            "start_agent": "false",
            "start_bridge": "false",

            "num_uas": "",
            "start_id": "",
            "flight_zone_id": "",
        }.items(),
    )

    survillance_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(uas_bringup_launch),
        launch_arguments={
            "config_file": survillance_config,

            # Ya los inició el primer grupo.
            "start_agent": "false",
            "start_bridge": "false",

            "num_uas": "",
            "start_id": "",
            "flight_zone_id": "",
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "inspection_config_file",
                default_value=default_inspection_config,
                description=(
                    "YAML del grupo de UAS asignado a la flight zone "
                    "de inspección."
                ),
            ),
            DeclareLaunchArgument(
                "logistics_config_file",
                default_value=default_logistics_config,
                description=(
                    "YAML del grupo de UAS asignado a la flight zone "
                    "de logística."
                ),
            ),
            DeclareLaunchArgument(
                "survillance_config_file",
                default_value=default_survillance_config,
                description=(
                    "YAML del grupo de UAS asignado a la flight zone "
                    "de survillance."
                ),
            ),
            DeclareLaunchArgument(
                "start_agent",
                default_value="true",
                description=(
                    "Iniciar un único Micro XRCE-DDS Agent desde el primer "
                    "grupo de UAS."
                ),
            ),
            DeclareLaunchArgument(
                "start_bridge",
                default_value="true",
                description=(
                    "Iniciar un único ros_gz_bridge desde el primer grupo "
                    "de UAS."
                ),
            ),
            LogInfo(
                msg=(
                    "Port UAS scenario: inspection group first; "
                    "logistics and survillance groups reuse the shared "
                    "Micro XRCE-DDS Agent and Gazebo bridge."
                )
            ),

            # El grupo de inspección contiene los primeros ids globales
            # y pone en marcha los componentes compartidos.
            inspection_launch,

            # Se escalonan las inclusiones para facilitar el arranque y los logs.
            TimerAction(
                period=0.50,
                actions=[
                    LogInfo(msg="Starting logistics UAS group."),
                    logistics_launch,
                ],
            ),
            TimerAction(
                period=1.00,
                actions=[
                    LogInfo(msg="Starting survillance UAS group."),
                    survillance_launch,
                ],
            ),
        ]
    )
