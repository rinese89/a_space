from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    package_share = get_package_share_directory(
        "supervision_trajectory_manager"
    )

    default_config = os.path.join(
        package_share,
        "config",
        "supervision_trajectory_manager.yaml",
    )

    config_file = LaunchConfiguration("config_file")

    return LaunchDescription([
        DeclareLaunchArgument(
            "config_file",
            default_value=default_config,
            description="Path to supervision_trajectory_manager YAML",
        ),
        Node(
            package="supervision_trajectory_manager",
            executable="supervision_trajectory_manager_node",
            name="supervision_trajectory_manager_node",
            output="screen",
            parameters=[config_file],
        ),
    ])
