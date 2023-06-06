from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    wifi_generator_node = Node(
        package="telemetry_pkg",
        executable="wifi_generator_node"
    )

    serial_interface_generator_node = Node(
        package="telemetry_pkg",
        executable="serial_interface_generator_node"
    )

    ups_generator_node = Node(
        package="telemetry_pkg",
        executable="ups_generator_node"
    )

    camera_generator_node = Node(
        package="telemetry_pkg",
        executable="camera_generator_node"
    )
    
    ld.add_action(wifi_generator_node)
    ld.add_action(serial_interface_generator_node)
    ld.add_action(ups_generator_node)
    ld.add_action(camera_generator_node)
    return ld