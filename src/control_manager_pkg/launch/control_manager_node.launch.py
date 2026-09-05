from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    flight_zone_id = LaunchConfiguration('flight_zone_id')
    uas_namespace = LaunchConfiguration('uas_namespace')
    config_file = LaunchConfiguration('config_file')

    package_share = get_package_share_directory('control_manager_pkg')
    default_config = os.path.join(
        package_share,
        'config',
        'control_manager.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'flight_zone_id',
            description='Flight-zone namespace, e.g. inspection_1',
        ),
        DeclareLaunchArgument(
            'uas_namespace',
            description='UAS namespace inside the flight zone, e.g. ua_ins_1',
        ),
        DeclareLaunchArgument(
            'config_file',
            default_value=default_config,
            description='Path to control_manager YAML',
        ),
        Node(
            package='control_manager_pkg',
            executable='control_manager_node',
            name='control_manager_node',
            namespace=PathJoinSubstitution([
                flight_zone_id,
                uas_namespace,
            ]),
            output='screen',
            parameters=[config_file],
        ),
    ])
