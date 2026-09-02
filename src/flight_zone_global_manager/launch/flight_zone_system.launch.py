#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = get_package_share_directory("flight_zone_global_manager")

    server_launch = os.path.join(share, "launch", "flight_zone_server.launch.py")
    monitor_launch = os.path.join(share, "launch", "flight_zone_monitor.launch.py")

    default_server_config = os.path.join(share, "config", "flight_zone_server.yaml")
    default_monitor_config = os.path.join(share, "config", "flight_zone_monitor.yaml")

    use_sim_time = LaunchConfiguration("use_sim_time")
    server_config = LaunchConfiguration("server_config")
    monitor_config = LaunchConfiguration("monitor_config")

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
        ),
        DeclareLaunchArgument(
            "server_config",
            default_value=default_server_config,
        ),
        DeclareLaunchArgument(
            "monitor_config",
            default_value=default_monitor_config,
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(server_launch),
            launch_arguments={
                "config_file": server_config,
                "use_sim_time": use_sim_time,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(monitor_launch),
            launch_arguments={
                "config_file": monitor_config,
                "use_sim_time": use_sim_time,
            }.items(),
        ),
    ])
