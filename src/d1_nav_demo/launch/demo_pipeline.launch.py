from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    params_file = PathJoinSubstitution(
        [FindPackageShare("d1_nav_demo"), "config", "params.yaml"]
    )
    return LaunchDescription(
        [
            Node(
                package="d1_nav_demo",
                executable="bspline_trajectory_tracker_node",
                name="bspline_trajectory_tracker_node",
                output="screen",
                parameters=[params_file],
            ),
        ]
    )
