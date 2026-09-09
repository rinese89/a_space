#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory("distance_control")
    default_config = os.path.join(share, "config", "distance_control.yaml")

    return LaunchDescription([
        DeclareLaunchArgument(
            "config_file",
            default_value=default_config,
            description="YAML de configuración de distance_control_node.",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
        ),
        Node(
            package="distance_control",
            executable="distance_control_node",
            name="distance_control_node",
            output="screen",
            emulate_tty=True,
            parameters=[
                LaunchConfiguration("config_file"),
                {
                    "use_sim_time": ParameterValue(
                        LaunchConfiguration("use_sim_time"),
                        value_type=bool,
                    )
                },
            ],
        ),
    ])
