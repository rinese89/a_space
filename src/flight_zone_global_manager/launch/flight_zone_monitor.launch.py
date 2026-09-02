#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = get_package_share_directory("flight_zone_global_manager")
    default_config = os.path.join(
        package_share, "config", "flight_zone_monitor.yaml"
    )

    config_file = LaunchConfiguration("config_file")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription([
        DeclareLaunchArgument(
            "config_file",
            default_value=default_config,
            description="YAML global de flight_zone_monitor_node.",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
        ),
        Node(
            package="flight_zone_global_manager",
            executable="flight_zone_monitor_node",
            name="flight_zone_monitor_node",
            output="screen",
            emulate_tty=True,
            parameters=[
                config_file,
                {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
            ],
        ),
    ])
