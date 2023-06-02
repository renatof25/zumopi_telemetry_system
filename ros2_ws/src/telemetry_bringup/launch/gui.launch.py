from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    gui_node = Node(
        package="telemetry_pkg",
        executable="gui_node"
    )

    ld.add_action(gui_node)
    return ld
