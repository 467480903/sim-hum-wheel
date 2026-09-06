#!/usr/bin/env python3
"""测试 head_joint_controller (JTC):
发布头部轨迹 (左右摆头+点头), 校验 /joint_states 跟踪到位。
前置: control.launch.py 已启动。
"""
import math
import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

JOINTS = ["idx11_head_joint1", "idx12_head_joint2", "idx13_head_joint3"]
# 限位: [-1.5707,1.5707] [-0.349,0.349] [-0.5236,0.5236]
TOL = 0.05


class HeadTest(Node):

    def __init__(self):
        super().__init__("test_head_joint_controller")
        self.pos = {}
        self.pub = self.create_publisher(JointTrajectory,
                                         "head_joint_controller/joint_trajectory", 10)
        self.create_subscription(JointState, "joint_states", self.on_js, 10)

    def on_js(self, msg: JointState):
        for n, p in zip(msg.name, msg.position):
            self.pos[n] = p

    def send(self, points):
        t = JointTrajectory()
        t.joint_names = JOINTS
        for sec, pos in points:
            p = JointTrajectoryPoint()
            p.time_from_start.sec = int(sec)
            p.time_from_start.nanosec = int((sec - int(sec)) * 1e9)
            p.positions = [float(v) for v in pos]
            t.points.append(p)
        self.pub.publish(t)


def main():
    rclpy.init()
    node = HeadTest()
    # 等待 joint_states 与订阅者就绪
    for _ in range(50):
        rclpy.spin_once(node, timeout_sec=0.1)
        if all(j in node.pos for j in JOINTS):
            break
    node.get_logger().info("当前头部位置: " +
                           ", ".join(f"{node.pos[j]:.3f}" for j in JOINTS))

    node.send([(2.0, (0.4, 0.15, -0.2)),
               (4.0, (-0.4, -0.15, 0.2)),
               (6.0, (0.0, 0.0, 0.0))])
    node.get_logger().info("轨迹已发送: 摆头 -> 回中, 共 6s")

    import time

    def wait_target(target, deadline):
        t0 = time.time()
        while time.time() - t0 < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            cur = [node.pos.get(j, 99.0) for j in JOINTS]
            if all(abs(c - t) < TOL for c, t in zip(cur, target)):
                return cur, True
        return cur, False

    mid, ok_mid = wait_target((0.4, 0.15, -0.2), 3.8)
    node.get_logger().info("第一目标点: " + ", ".join(f"{v:.3f}" for v in mid) + f" 到达={ok_mid}")

    final, ok = wait_target((0.0, 0.0, 0.0), 8.0)
    node.get_logger().info("最终位置: " + ", ".join(f"{v:.3f}" for v in final))
    ok = ok_mid and ok
    print("PASS: 头部回中" if ok else "FAIL: 头部未回到目标")
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
