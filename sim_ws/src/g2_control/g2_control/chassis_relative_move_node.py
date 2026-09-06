#!/usr/bin/env python3
"""底盘相对运动控制器:

订阅 /chassis_relative_move (geometry_msgs/Vector3):
  x = 前进量 dx (m, 车体系), y = 左移量 dy (m), z = 旋转量 dtheta (rad)
基于里程计闭环, P 控制器生成 /cmd_vel, 到达目标后停车。
状态发布 /chassis_move_status (std_msgs/String): idle/moving/done
"""
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry
from std_msgs.msg import String


def wrap_pi(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class ChassisRelativeMoveNode(Node):

    def __init__(self):
        super().__init__("chassis_relative_move_node")
        self.declare_parameter("p_lin", 1.2)      # 线性 P 增益
        self.declare_parameter("p_ang", 2.0)      # 角度 P 增益
        self.declare_parameter("max_lin", 0.8)    # m/s
        self.declare_parameter("max_ang", 1.2)    # rad/s
        self.declare_parameter("tol_lin", 0.02)   # m
        self.declare_parameter("tol_ang", 0.03)   # rad
        self.p_lin = float(self.get_parameter("p_lin").value)
        self.p_ang = float(self.get_parameter("p_ang").value)
        self.max_lin = float(self.get_parameter("max_lin").value)
        self.max_ang = float(self.get_parameter("max_ang").value)
        self.tol_lin = float(self.get_parameter("tol_lin").value)
        self.tol_ang = float(self.get_parameter("tol_ang").value)

        self.pose = None           # (x, y, yaw) 最新里程计位姿
        self.target = None         # (x, y, yaw) 目标位姿 (odom 系)
        self.status = "idle"

        self.create_subscription(Odometry, "odom", self.on_odom, 10)
        self.create_subscription(Vector3, "chassis_relative_move", self.on_goal, 10)
        self.pub_cmd = self.create_publisher(Twist, "cmd_vel", 10)
        self.pub_status = self.create_publisher(String, "chassis_move_status", 10)
        self.timer = self.create_timer(0.05, self.loop)   # 20Hz

    def on_odom(self, msg: Odometry):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.pose = (msg.pose.pose.position.x,
                     msg.pose.pose.position.y, yaw)

    def on_goal(self, msg: Vector3):
        if self.pose is None:
            self.get_logger().warn("尚无里程计, 丢弃相对运动命令")
            return
        x, y, yaw = self.pose
        c, s = math.cos(yaw), math.sin(yaw)
        # 车体系位移 -> odom 系
        tx = x + msg.x * c - msg.y * s
        ty = y + msg.x * s + msg.y * c
        tyaw = wrap_pi(yaw + msg.z)
        self.target = (tx, ty, tyaw)
        self.set_status("moving")
        self.get_logger().info(
            f"相对运动目标: dx={msg.x:.3f} dy={msg.y:.3f} dth={msg.z:.3f} "
            f"-> odom ({tx:.3f},{ty:.3f},{tyaw:.3f})")

    def set_status(self, s):
        if s != self.status:
            self.status = s
            m = String()
            m.data = s
            self.pub_status.publish(m)

    def loop(self):
        if self.target is None or self.pose is None:
            return                    # 空闲: 不发布, 避免 与 nav2 的 cmd_vel 冲突
        cmd = Twist()
        x, y, yaw = self.pose
        tx, ty, tyaw = self.target
        ex = tx - x
        ey = ty - y
        eth = wrap_pi(tyaw - yaw)
        dist = math.hypot(ex, ey)

        if dist < self.tol_lin and abs(eth) < self.tol_ang:
            self.target = None
            self.set_status("done")
            self.pub_cmd.publish(cmd)   # 最终停车一次
            self.get_logger().info("相对运动完成")
            return

        # 误差 -> 车体系 (舵轮全向底盘: 平移与旋转可同时进行)
        c, s = math.cos(yaw), math.sin(yaw)
        exb = ex * c + ey * s
        eyb = -ex * s + ey * c

        cmd.linear.x = max(-self.max_lin, min(self.max_lin, self.p_lin * exb))
        cmd.linear.y = max(-self.max_lin, min(self.max_lin, self.p_lin * eyb))
        cmd.angular.z = max(-self.max_ang, min(self.max_ang, self.p_ang * eth))
        self.pub_cmd.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = ChassisRelativeMoveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
