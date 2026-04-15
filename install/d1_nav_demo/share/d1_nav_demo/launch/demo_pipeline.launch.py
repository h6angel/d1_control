from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package="d1_nav_demo",
                executable="circle_trajectory_tracker_node",
                name="circle_trajectory_tracker_node",
                output="screen",
                parameters=["config/params.yaml"],
            ),
        ]
    )
