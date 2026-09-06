#!/usr/bin/env python3
"""测试底盘相对运动控制器 (chassis_relative_move_node):
发 /chassis_relative_move (dx=0.5m, dyaw=0.5rad), 等待 /chassis_move_status=done,
校验 /odom 位移。前置: control.launch.py 已启动。
"""
import math
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
from std_msgs.msg import String


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class RelMoveTest(Node):

    def __init__(self):
        super().__init__("test_chassis_relative_move")
        self.odom = None
        self.yaw = 0.0
        self.status = "idle"
        self.pub_goal = self.create_publisher(Vector3, "chassis_relative_move", 10)
        self.create_subscription(Odometry, "odom", self.on_odom, 10)
        self.create_subscription(String, "chassis_move_status", self.on_status, 10)

    def on_odom(self, msg: Odometry):
        self.odom = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self.yaw = yaw_of(msg.pose.pose.orientation)

    def on_status(self, msg: String):
        self.status = msg.data


def main():
    rclpy.init()
    node = RelMoveTest()
    t0 = time.time()
    while (node.odom is None) and time.time() - t0 < 5.0:
        rclpy.spin_once(node, timeout_sec=0.1)
    if node.odom is None:
        print("FAIL: 未收到 /odom")
        sys.exit(1)

    x0, y0 = node.odom
    th0 = node.yaw
    g = Vector3()
    g.x = 0.5      # 前进 0.5m
    g.y = 0.0
    g.z = 0.5      # 旋转 0.5rad
    node.pub_goal.publish(g)
    node.get_logger().info("相对运动命令: dx=0.5m, dyaw=0.5rad")

    t0 = time.time()
    while time.time() - t0 < 30.0 and node.status != "done":
        rclpy.spin_once(node, timeout_sec=0.1)
    # 再转一会儿收尾状态
    for _ in range(10):
        rclpy.spin_once(node, timeout_sec=0.05)

    x1, y1 = node.odom
    dth = (node.yaw - th0 + math.pi) % (2 * math.pi) - math.pi
    # 初始 yaw≈th0, 前进 0.5m 主要沿初始朝向
    c, s = math.cos(th0), math.sin(th0)
    fwd = (x1 - x0) * c + (y1 - y0) * s
    node.get_logger().info(
        f"结果: 前进量={fwd:.3f} (期望0.5), dyaw={dth:.3f} (期望0.5), status={node.status}")

    ok = (node.status == "done" and abs(fwd - 0.5) < 0.1 and abs(dth - 0.5) < 0.1)
    print("PASS: 底盘相对运动" if ok else "FAIL: 底盘相对运动")
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
