"""G2 仿真控制栈启动 (无 rviz, 供测试/复用):
robot_state_publisher + controller_manager(mock 硬件) + 全部控制器
+ 舵轮运动学/里程计 + 虚拟激光 + 相对运动 + 导航中继

独立启动: ros2 launch ~/sim/sim_ws/src/g2_control/launch/control.launch.py
所有节点的输出都显示在当前终端。
"""
import os

import ament_index_python
from launch import LaunchDescription
from launch_ros.actions import Node

DESC = ament_index_python.get_package_share_directory("g2_description")
CTRL = ament_index_python.get_package_share_directory("g2_control")

URDF = os.path.join(DESC, "urdf", "model.urdf")
URDF_CTRL = os.path.join(DESC, "urdf", "model_control.urdf")
CONTROLLERS = os.path.join(CTRL, "config", "controllers.yaml")
SCENE = os.path.join(DESC, "config", "scene.yaml")

# 默认激活: 关节轨迹控制器 + 舵轮驱动
ACTIVE = [
    "joint_state_broadcaster",
    "left_arm_joint_controller",
    "right_arm_joint_controller",
    "head_joint_controller",
    "waist_joint_controller",
    "swerve_steering_controller",
    "swerve_drive_controller",
]
# 加载但不激活 (与上面的控制器共用硬件接口, 需切换)
INACTIVE = [
    "left_arm_position_controller",
    "right_arm_position_controller",
    "waist_position_controller",
    "chassis_joint_controller",
    "chassis_position_controller",
]


def generate_launch_description():
    with open(URDF, encoding="utf-8") as f:
        robot_desc = f.read()
    with open(URDF_CTRL, encoding="utf-8") as f:
        robot_desc_ctrl = f.read()

    ld = LaunchDescription()

    # 机器人描述 (TF)
    ld.add_action(Node(
        package="robot_state_publisher", executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{"robot_description": robot_desc}],
        output="screen",
    ))

    # 房间描述 (仅静态 TF + /room_description)
    with open(os.path.join(DESC, "urdf", "model_room.urdf"), encoding="utf-8") as f:
        room_desc = f.read()
    ld.add_action(Node(
        package="robot_state_publisher", executable="robot_state_publisher",
        name="room_state_publisher",
        parameters=[{"robot_description": room_desc}],
        remappings=[("robot_description", "room_description"),
                    ("joint_states", "room_joint_states")],
        output="screen",
    ))
    # map -> room_world 静态 TF, 使场景与导航地图同源
    ld.add_action(Node(
        package="tf2_ros", executable="static_transform_publisher",
        name="map_to_room_world",
        arguments=["0", "0", "0", "0", "0", "0", "map", "room_world"],
        output="screen",
    ))

    # controller_manager + mock 硬件
    # 注意: 不要设置 name="controller_manager"! launch 会生成 -r __node:= 全局重映射,
    # 导致所有控制器节点(use_global_arguments=true)也被重命名为 controller_manager,
    # 从而加载不到各自的参数。可执行文件内部节点名本来就是 controller_manager。
    ld.add_action(Node(
        package="controller_manager", executable="ros2_control_node",
        parameters=[{"robot_description": robot_desc_ctrl}, CONTROLLERS],
        output="screen",
        remappings=[("~/robot_description", "robot_description")],
    ))

    # 激活的控制器
    ld.add_action(Node(
        package="controller_manager", executable="spawner",
        arguments=ACTIVE + ["--controller-manager", "/controller_manager",
                            "--param-file", CONTROLLERS],
        output="screen",
    ))
    # 仅加载不激活
    ld.add_action(Node(
        package="controller_manager", executable="spawner",
        arguments=INACTIVE + ["--inactive", "--controller-manager", "/controller_manager",
                              "--param-file", CONTROLLERS],
        output="screen",
    ))

    # 舵轮运动学 + 里程计
    ld.add_action(Node(
        package="g2_control", executable="swerve_drive_node",
        name="swerve_drive_node", output="screen",
    ))
    # 虚拟激光 (SLAM 传感器)
    ld.add_action(Node(
        package="g2_control", executable="laser_sim_node",
        name="laser_sim_node", output="screen",
        parameters=[{"scene_yaml": SCENE}],
    ))
    # 底盘相对运动控制器
    ld.add_action(Node(
        package="g2_control", executable="chassis_relative_move_node",
        name="chassis_relative_move_node", output="screen",
    ))
    # 导航目标中继
    ld.add_action(Node(
        package="g2_control", executable="nav_goal_relay_node",
        name="nav_goal_relay_node", output="screen",
    ))
    return ld
