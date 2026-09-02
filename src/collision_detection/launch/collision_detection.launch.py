from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    package_share = get_package_share_directory('collision_detection')
    default_config = os.path.join(
        package_share,
        'config',
        'collision_detection.yaml')

    config_file = LaunchConfiguration('config_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=default_config,
            description='Path to collision-detection YAML',
        ),
        Node(
            package='collision_detection',
            executable='collision_detection_node',
            name='collision_detection_node',
            output='screen',
            parameters=[config_file],
        ),
    ])
