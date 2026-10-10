"""Start only the static web server and show its access log in this terminal.

Run: ros2 launch ~/sim/sim_ws/src/g2_control/launch/web.launch.py
"""
from launch import LaunchDescription
from launch.actions import ExecuteProcess

WEB_DIR = "/home/wxf/sim/web"


def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=["python3", "-u", "-m", "http.server", "8080", "--directory", WEB_DIR],
            name="web_server",
            output="screen",
        ),
    ])
