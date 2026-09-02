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
    config_file,
) -> IncludeLaunchDescription:
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_file),
        launch_arguments={"config_file": config_file}.items(),
    )


def generate_launch_description() -> LaunchDescription:
    bringup_share = get_package_share_directory(
        "trajectory_system_bringup_pkg"
    )

    manager_share = get_package_share_directory(
        "static_trajectory_manager"
    )
    conflict_share = get_package_share_directory(
        "static_trajectory_conflict_manager"
    )
    server_share = get_package_share_directory(
        "trajectory_server_pkg"
    )

    manager_launch = os.path.join(
        manager_share,
        "launch",
        "static_trajectory_manager.launch.py",
    )
    conflict_launch = os.path.join(
        conflict_share,
        "launch",
        "static_trajectory_conflict_manager.launch.py",
    )
    server_launch = os.path.join(
        server_share,
        "launch",
        "trajectory_server_node.launch.py",
    )

    manager_default_config = os.path.join(
        bringup_share,
        "config",
        "static_trajectories.yaml",
    )
    conflict_default_config = os.path.join(
        bringup_share,
        "config",
        "static_trajectory_conflict_manager.yaml",
    )
    server_default_config = os.path.join(
        bringup_share,
        "config",
        "trajectory_server_node.yaml",
    )

    manager_config = LaunchConfiguration(
        "static_trajectories_config_file"
    )
    conflict_config = LaunchConfiguration(
        "conflict_manager_config_file"
    )
    server_config = LaunchConfiguration(
        "trajectory_server_config_file"
    )

    manager = _include_python_launch(
        manager_launch,
        manager_config,
    )
    conflict_manager = _include_python_launch(
        conflict_launch,
        conflict_config,
    )
    trajectory_server = _include_python_launch(
        server_launch,
        server_config,
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "static_trajectories_config_file",
            default_value=manager_default_config,
            description=(
                "Static trajectory definitions YAML stored by the "
                "trajectory-system bringup package."
            ),
        ),
        DeclareLaunchArgument(
            "conflict_manager_config_file",
            default_value=conflict_default_config,
            description=(
                "Static trajectory conflict-manager YAML stored by the "
                "trajectory-system bringup package."
            ),
        ),
        DeclareLaunchArgument(
            "trajectory_server_config_file",
            default_value=server_default_config,
            description=(
                "Trajectory-server YAML stored by the trajectory-system "
                "bringup package."
            ),
        ),
        LogInfo(
            msg=(
                "Launching trajectory system from external package launches: "
                "static_trajectory_manager -> "
                "static_trajectory_conflict_manager -> "
                "trajectory_server_pkg/trajectory_server_node"
            )
        ),
        manager,
        TimerAction(
            period=0.5,
            actions=[conflict_manager],
        ),
        TimerAction(
            period=1.0,
            actions=[trajectory_server],
        ),
    ])
