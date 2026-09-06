"""G2 仿真总启动: 控制栈 + rviz2 + rosbridge + nav2 + web 服务

用法: ros2 launch g2_control sim.launch.py
  web 页面: http://localhost:8080
  rosbridge: ws://localhost:9090
"""
import os

import ament_index_python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

DESC = ament_index_python.get_package_share_directory("g2_description")
CTRL = ament_index_python.get_package_share_directory("g2_control")
WEB_DIR = "/home/wxf/sim/web"


def generate_launch_description():
    ld = LaunchDescription()

    # 控制栈
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(CTRL, "launch", "control.launch.py"))))

    # rviz2
    ld.add_action(Node(
        package="rviz2", executable="rviz2", name="rviz2",
        arguments=["-d", os.path.join(DESC, "rviz", "sim.rviz")],
        output="screen",
    ))

    # rosbridge (web 交互)
    ld.add_action(Node(
        package="rosbridge_server", executable="rosbridge_websocket",
        name="rosbridge_websocket", output="screen",
    ))

    # ---------------- nav2 ----------------
    nav2_params = os.path.join(CTRL, "config", "nav2_params.yaml")
    map_yaml = os.path.join(DESC, "maps", "scene_map.yaml")

    nav_nodes = [
        Node(package="nav2_map_server", executable="map_server", name="map_server",
             output="screen",
             parameters=[nav2_params, {"yaml_filename": map_yaml}]),
        Node(package="nav2_amcl", executable="amcl", name="amcl", output="screen",
             parameters=[nav2_params]),
        Node(package="nav2_controller", executable="controller_server",
             name="controller_server", output="screen",
             parameters=[nav2_params],
             remappings=[("cmd_vel", "cmd_vel")]),
        Node(package="nav2_planner", executable="planner_server",
             name="planner_server", output="screen", parameters=[nav2_params]),
        Node(package="nav2_bt_navigator", executable="bt_navigator",
             name="bt_navigator", output="screen", parameters=[nav2_params]),
        Node(package="nav2_behaviors", executable="behavior_server",
             name="behavior_server", output="screen", parameters=[nav2_params]),
    ]
    for n in nav_nodes:
        ld.add_action(n)
    ld.add_action(Node(
        package="nav2_lifecycle_manager", executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[{"autostart": True,
                     "node_names": ["map_server", "amcl", "controller_server",
                                    "planner_server", "bt_navigator", "behavior_server"]}],
    ))

    # ---------------- web 服务 ----------------
    if os.path.isdir(WEB_DIR):
        from launch.actions import ExecuteProcess
        ld.add_action(ExecuteProcess(
            cmd=["python3", "-m", "http.server", "8080", "--directory", WEB_DIR],
            name="web_server", output="screen",
        ))
    return ld
