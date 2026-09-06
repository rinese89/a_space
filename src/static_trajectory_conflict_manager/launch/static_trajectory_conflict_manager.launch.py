from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_config = PathJoinSubstitution([
        FindPackageShare("static_trajectory_conflict_manager"),
        "config",
        "static_trajectory_conflict_manager.yaml",
    ])

    config_file = LaunchConfiguration("config_file")

    return LaunchDescription([
        DeclareLaunchArgument(
            "config_file",
            default_value=default_config,
            description="Path to static trajectory conflict-manager YAML",
        ),
        Node(
            package="static_trajectory_conflict_manager",
            executable="static_trajectory_conflict_manager_node",
            name="static_trajectory_conflict_manager_node",
            output="screen",
            parameters=[config_file],
        ),
    ])
