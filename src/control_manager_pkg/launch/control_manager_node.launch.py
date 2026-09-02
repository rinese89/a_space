from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    flight_zone_id = LaunchConfiguration("flight_zone_id")
    uas_namespace = LaunchConfiguration("uas_namespace")
    config_file = LaunchConfiguration("config_file")

    namespace = [flight_zone_id, "/", uas_namespace]

    return LaunchDescription([
        DeclareLaunchArgument("flight_zone_id", default_value="inspection_1"),
        DeclareLaunchArgument("uas_namespace", default_value="ua_ins_1"),
        DeclareLaunchArgument(
            "config_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("control_manager_pkg"),
                "config",
                "control_manager_node.yaml",
            ]),
        ),
        Node(
            package="control_manager_pkg",
            executable="control_manager_node",
            name="control_manager_node",
            namespace=namespace,
            output="screen",
            parameters=[config_file],
        ),
    ])
