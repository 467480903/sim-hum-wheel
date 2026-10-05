"""Start only rosbridge_websocket for the browser console."""
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
