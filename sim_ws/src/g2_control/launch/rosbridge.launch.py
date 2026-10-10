"""Start only rosbridge_websocket and show its output in this terminal.

Run: ros2 launch ~/sim/sim_ws/src/g2_control/launch/rosbridge.launch.py
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="rosbridge_server",
            executable="rosbridge_websocket",
            name="rosbridge_websocket",
            output="screen",
        ),
    ])
