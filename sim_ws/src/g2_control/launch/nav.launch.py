"""导航栈单独启动 (控制栈已在运行时使用, 避免重复启动 controller_manager):

独立启动: ros2 launch ~/sim/sim_ws/src/g2_control/launch/nav.launch.py
包含: map_server + amcl + planner/controller/bt_navigator/behavior + lifecycle_manager
所有节点的输出都显示在当前终端。
"""
import os

import ament_index_python
from launch import LaunchDescription
from launch_ros.actions import Node

DESC = ament_index_python.get_package_share_directory("g2_description")
CTRL = ament_index_python.get_package_share_directory("g2_control")


def generate_launch_description():
    ld = LaunchDescription()
    nav2_params = os.path.join(CTRL, "config", "nav2_params.yaml")
    map_yaml = os.path.join(DESC, "maps", "scene_map.yaml")

    nodes = [
        Node(package="nav2_map_server", executable="map_server", name="map_server",
             output="screen",
             parameters=[nav2_params, {"yaml_filename": map_yaml}]),
        Node(package="nav2_amcl", executable="amcl", name="amcl", output="screen",
             parameters=[nav2_params]),
        Node(package="nav2_controller", executable="controller_server",
             name="controller_server", output="screen",
             parameters=[nav2_params]),
        Node(package="nav2_planner", executable="planner_server",
             name="planner_server", output="screen", parameters=[nav2_params]),
        Node(package="nav2_bt_navigator", executable="bt_navigator",
             name="bt_navigator", output="screen", parameters=[nav2_params]),
        Node(package="nav2_behaviors", executable="behavior_server",
             name="behavior_server", output="screen", parameters=[nav2_params]),
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
             name="lifecycle_manager_navigation", output="screen",
             parameters=[{"autostart": True,
                          "node_names": ["map_server", "amcl", "controller_server",
                                         "planner_server", "bt_navigator",
                                         "behavior_server"]}]),
    ]
    for n in nodes:
        ld.add_action(n)
    return ld
