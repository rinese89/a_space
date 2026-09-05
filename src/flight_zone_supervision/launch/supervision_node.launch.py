from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    flight_zone_id = LaunchConfiguration('flight_zone_id')
    config_file = LaunchConfiguration('config_file')

    default_config = os.path.join(
        get_package_share_directory('flight_zone_supervision'),
        'config',
        'supervision_node.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'flight_zone_id',
            description='Flight-zone namespace, e.g. inspection_1'),
        DeclareLaunchArgument(
            'config_file',
            default_value=default_config,
            description='Path to supervision_node YAML'),
        Node(
            package='flight_zone_supervision',
            executable='supervision_node',
            namespace=flight_zone_id,
            name='supervision_node',
            output='screen',
            parameters=[
                config_file,
                {'flight_zone_id': flight_zone_id},
            ],
        ),
    ])
