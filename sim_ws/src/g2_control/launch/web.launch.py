"""Start only the static web console server, with its access log in this terminal."""
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
