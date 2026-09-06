#!/usr/bin/env python3
"""测试 4 舵轮底盘 (swerve_steering + swerve_drive 控制器, 经 swerve_drive_node):
1) 前进 0.3m/s x 3s  -> odom x ≈ +0.9
2) 左移 0.2m/s x 3s  -> odom y ≈ +0.6
3) 旋转 0.5rad/s x 2s -> yaw ≈ +1.0
校验 /odom 位移。前置: control.launch.py 已启动。
"""
import math
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class SwerveTest(Node):

    def __init__(self):
        super().__init__("test_swerve_chassis")
        self.odom = None
        self.yaw = 0.0
        self.pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.create_subscription(Odometry, "odom", self.on_odom, 10)

    def on_odom(self, msg: Odometry):
        self.odom = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self.yaw = yaw_of(msg.pose.pose.orientation)

    def drive(self, vx, vy, wz, dur):
        """以 20Hz 连续发布 cmd_vel (swerve_drive_node 0.5s 超时停车)"""
        t0 = time.time()
        while time.time() - t0 < dur:
            m = Twist()
            m.linear.x = float(vx)
            m.linear.y = float(vy)
            m.angular.z = float(wz)
            self.pub.publish(m)
            rclpy.spin_once(self, timeout_sec=0.05)
        # 停车
        self.pub.publish(Twist())
        for _ in range(20):
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_odom(self, timeout=5.0):
        t0 = time.time()
        while self.odom is None and time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)


def main():
    rclpy.init()
    node = SwerveTest()
    node.wait_odom()
    if node.odom is None:
        print("FAIL: 未收到 /odom")
        sys.exit(1)

    # 1. 前进
    x0, y0 = node.odom
    node.drive(0.3, 0.0, 0.0, 3.0)
    x1, y1 = node.odom
    dx = x1 - x0
    node.get_logger().info(f"前进 3s: dx={dx:.3f} (期望≈0.9), dy={y1 - y0:.3f}")
    ok1 = abs(dx - 0.9) < 0.2 and abs(y1 - y0) < 0.15

    # 2. 左移 (全向)
    x0, y0 = node.odom
    node.drive(0.0, 0.2, 0.0, 3.0)
    x1, y1 = node.odom
    dy = y1 - y0
    node.get_logger().info(f"左移 3s: dy={dy:.3f} (期望≈0.6), dx={x1 - x0:.3f}")
    ok2 = abs(dy - 0.6) < 0.2 and abs(x1 - x0) < 0.15

    # 3. 原地旋转
    th0 = node.yaw
    node.drive(0.0, 0.0, 0.5, 2.0)
    dth = node.yaw - th0
    dth = (dth + math.pi) % (2 * math.pi) - math.pi
    node.get_logger().info(f"旋转 2s: dyaw={dth:.3f} (期望≈1.0)")
    ok3 = abs(dth - 1.0) < 0.25

    ok = ok1 and ok2 and ok3
    print("PASS: 4 舵轮底盘 (前进/横移/旋转)" if ok else
          f"FAIL: 前进={ok1} 横移={ok2} 旋转={ok3}")
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
