from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    package_share = get_package_share_directory('a_space_virtual_net')
    default_config = os.path.join(
        package_share,
        'config',
        'a_space_virtual_net.yaml')

    config_file = LaunchConfiguration('config_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=default_config,
            description='Path to A-space virtual-net YAML',
        ),
        Node(
            package='a_space_virtual_net',
            executable='a_space_virtual_net_node',
            name='a_space_virtual_net_node',
            output='screen',
            parameters=[config_file],
        ),
    ])
