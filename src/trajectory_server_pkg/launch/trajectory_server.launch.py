from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    package_share = get_package_share_directory('trajectory_server_pkg')
    default_config = os.path.join(package_share, 'config', 'trajectory_server.yaml')
    config_file = LaunchConfiguration('config_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=default_config,
            description='Path to trajectory_server YAML',
        ),
        Node(
            package='trajectory_server_pkg',
            executable='trajectory_server_node',
            name='trajectory_server_node',
            output='screen',
            parameters=[config_file],
        ),
    ])
