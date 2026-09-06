"""SLAM 建图模式: 控制栈 + rviz2 + slam_toolbox (在线异步建图)

用法: ros2 launch g2_control slam.launch.py
建图完成后保存:
  ros2 run nav2_map_server map_saver_cli -f ~/map --occ 0.65 --free 0.25
"""
import os

import ament_index_python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

DESC = ament_index_python.get_package_share_directory("g2_description")
CTRL = ament_index_python.get_package_share_directory("g2_control")


def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(CTRL, "launch", "control.launch.py"))))

    ld.add_action(Node(
        package="rviz2", executable="rviz2", name="rviz2",
        arguments=["-d", os.path.join(DESC, "rviz", "sim.rviz")],
        output="screen",
    ))

    ld.add_action(Node(
        package="slam_toolbox", executable="async_slam_toolbox_node",
        name="slam_toolbox", output="screen",
        parameters=[os.path.join(CTRL, "config", "slam_params.yaml")],
    ))
    return ld
