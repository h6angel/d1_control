from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package="d1_nav_demo",
                executable="fake_bspline_to_path_node",
                name="fake_bspline_to_path_node",
                output="screen",
                parameters=["config/params.yaml"],
            ),
            Node(
                package="d1_nav_demo",
                executable="path_tracker_node",
                name="path_tracker_node",
                output="screen",
                parameters=["config/params.yaml"],
            ),
            Node(
                package="d1_nav_demo",
                executable="d1_user_command_bridge_node",
                name="d1_user_command_bridge_node",
                output="screen",
                parameters=["config/params.yaml"],
            ),
        ]
    )
