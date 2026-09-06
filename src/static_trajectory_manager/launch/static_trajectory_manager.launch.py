from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_config = PathJoinSubstitution([
        FindPackageShare("static_trajectory_manager"),
        "config",
        "static_trajectories.yaml",
    ])

    config_file = LaunchConfiguration("config_file")
    flight_zones_topic = LaunchConfiguration("flight_zones_topic")
    requested_topic = LaunchConfiguration("requested_static_trajectories_topic")
    unvalidated_topic = LaunchConfiguration("unvalidated_trajectories_topic")
    latest_topic = LaunchConfiguration("latest_trajectories_topic")
    adjusted_topic = LaunchConfiguration("adjusted_trajectories_topic")
    markers_topic = LaunchConfiguration("requested_static_trajectories_markers_topic")
    validation_period_ms = LaunchConfiguration("validation_period_ms")

    return LaunchDescription([
        DeclareLaunchArgument(
            "config_file",
            default_value=default_config,
            description="YAML containing complete static trajectories",
        ),
        DeclareLaunchArgument(
            "flight_zones_topic",
            default_value="/flight_zones",
        ),
        DeclareLaunchArgument(
            "requested_static_trajectories_topic",
            default_value="/requested_static_trajectories",
        ),
        DeclareLaunchArgument(
            "unvalidated_trajectories_topic",
            default_value="/unvalidated_trajectories",
        ),
        DeclareLaunchArgument(
            "latest_trajectories_topic",
            default_value="/latest_trajectories",
        ),
        DeclareLaunchArgument(
            "adjusted_trajectories_topic",
            default_value="/adjusted_trajectories",
        ),
        DeclareLaunchArgument(
            "requested_static_trajectories_markers_topic",
            default_value="/requested_static_trajectories_markers",
        ),
        DeclareLaunchArgument(
            "validation_period_ms",
            default_value="1000",
        ),
        Node(
            package="static_trajectory_manager",
            executable="static_trajectory_manager_node",
            name="static_trajectory_manager_node",
            output="screen",
            parameters=[{
                "config_file": config_file,
                "flight_zones_topic": flight_zones_topic,
                "requested_static_trajectories_topic": requested_topic,
                "unvalidated_trajectories_topic": unvalidated_topic,
                "latest_trajectories_topic": latest_topic,
                "adjusted_trajectories_topic": adjusted_topic,
                "requested_static_trajectories_markers_topic": markers_topic,
                "validation_period_ms": validation_period_ms,
            }],
        ),
    ])
