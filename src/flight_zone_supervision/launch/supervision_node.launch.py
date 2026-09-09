#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _launch_setup(context):
    flight_zone_id = (
        LaunchConfiguration("flight_zone_id")
        .perform(context)
        .strip("/")
    )

    config_file = LaunchConfiguration(
        "config_file"
    ).perform(context)

    if not flight_zone_id:
        raise RuntimeError(
            "flight_zone_id must not be empty"
        )

    if "/" in flight_zone_id:
        raise RuntimeError(
            "flight_zone_id must be one ROS namespace segment"
        )

    return [
        Node(
            package="flight_zone_supervision",
            executable="supervision_node",
            namespace=flight_zone_id,
            name="supervision_node",
            output="screen",
            emulate_tty=True,
            parameters=[
                config_file,
                {
                    "flight_zone_id": flight_zone_id,
                    "use_sim_time": ParameterValue(
                        LaunchConfiguration("use_sim_time"),
                        value_type=bool,
                    ),
                },
            ],
        )
    ]


def generate_launch_description():
    share = get_package_share_directory(
        "flight_zone_supervision"
    )

    default_config = os.path.join(
        share,
        "config",
        "supervision_node.yaml",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "flight_zone_id",
                description=(
                    "Flight-zone namespace segment, "
                    "for example inspection_1."
                ),
            ),
            DeclareLaunchArgument(
                "config_file",
                default_value=default_config,
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
            ),
            OpaqueFunction(
                function=_launch_setup
            ),
        ]
    )
