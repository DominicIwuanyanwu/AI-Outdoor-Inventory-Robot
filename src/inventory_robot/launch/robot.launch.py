from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='inventory_robot',
            executable='serial_bridge',
            name='serial_bridge',
            output='screen',
            emulate_tty=True,
        ),
        Node(
            package='inventory_robot',
            executable='obstacle_avoid',
            name='obstacle_avoid',
            output='screen',
            emulate_tty=True,
        ),
        Node(
            package='inventory_robot',
            executable='camera_scan_node',
            name='camera_scan_node',
            output='screen',
            emulate_tty=True,
        ),
    ])
