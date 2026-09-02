from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    package_share = get_package_share_directory('updated_flow_server')
    default_config = os.path.join(package_share, 'config', 'updated_flow_server.yaml')
    config_file = LaunchConfiguration('config_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=default_config,
            description='Path to updated_flow_server YAML',
        ),
        Node(
            package='updated_flow_server',
            executable='updated_flow_server_node',
            name='updated_flow_server_node',
            output='screen',
            parameters=[config_file],
        ),
    ])
