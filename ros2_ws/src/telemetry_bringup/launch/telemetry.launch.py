from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    wifi_stats_node = Node(
        package="telemetry_pkg",
        executable="wifi_stats_node"
    )

    serial_interface_node = Node(
        package="telemetry_pkg",
        executable="serial_interface_node"
    )

    ups_reading_node = Node(
        package="telemetry_pkg",
        executable="ups_reading_node"
    )

    camera_node = Node(
        package="telemetry_pkg",
        executable="camera_node"
    )

    ld.add_action(wifi_stats_node)
    ld.add_action(serial_interface_node)
    ld.add_action(ups_reading_node)
    ld.add_action(camera_node)
    return ld
