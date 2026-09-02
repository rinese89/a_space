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
        package_share, "config", "flight_zone_server.yaml"
    )

    config_file = LaunchConfiguration("config_file")
    publish_period_s = LaunchConfiguration("publish_period_s")
    zone_topic = LaunchConfiguration("zone_topic")
    marker_topic = LaunchConfiguration("marker_topic")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription([
        DeclareLaunchArgument(
            "config_file",
            default_value=default_config,
            description="YAML global con todas las flight zones.",
        ),
        DeclareLaunchArgument(
            "publish_period_s",
            default_value="1.0",
            description="Periodo de republicación; 0.0 deja solo transient-local.",
        ),
        DeclareLaunchArgument(
            "zone_topic",
            default_value="/flight_zones",
        ),
        DeclareLaunchArgument(
            "marker_topic",
            default_value="/flight_zones/markers",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
        ),
        Node(
            package="flight_zone_global_manager",
            executable="flight_zone_server_node",
            name="flight_zone_server_node",
            output="screen",
            emulate_tty=True,
            parameters=[{
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                "config_file": config_file,
                "publish_period_s": ParameterValue(publish_period_s, value_type=float),
                "zone_topic": zone_topic,
                "marker_topic": marker_topic,
            }],
        ),
    ])
