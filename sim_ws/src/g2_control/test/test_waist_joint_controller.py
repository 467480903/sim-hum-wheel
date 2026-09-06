#!/usr/bin/env python3
"""测试 waist_joint_controller (JTC, 5 关节):
腰部运动到目标位姿再回零, 校验 /joint_states 跟踪到位。
前置: control.launch.py 已启动。
"""
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

JOINTS = [f"idx0{i}_body_joint{i}" for i in range(1, 6)]
# 限位: [-1.0821,0] [0,2.6529] [-1.919,1.117] [-0.4363,0.4363] [-3.0456,3.0456]
POSE = [-0.30, 0.40, 0.25, 0.15, -0.20]
TOL = 0.05


class WaistTest(Node):

    def __init__(self):
        super().__init__("test_waist_joint_controller")
        self.pos = {}
        self.pub = self.create_publisher(JointTrajectory,
                                         "waist_joint_controller/joint_trajectory", 10)
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

    def wait_until(self, target, timeout):
        t0 = time.time()
        while time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            cur = [self.pos.get(j, 99.0) for j in JOINTS]
            if all(abs(c - t) < TOL for c, t in zip(cur, target)):
                return True
        return False


def main():
    rclpy.init()
    node = WaistTest()
    for _ in range(50):
        rclpy.spin_once(node, timeout_sec=0.1)
        if all(j in node.pos for j in JOINTS):
            break

    node.send([(3.0, POSE), (6.0, [0.0, 0.0, 0.0, 0.0, 0.0])])
    node.get_logger().info("腰部轨迹已发送: 弯腰+侧倾 -> 回零")

    ok1 = node.wait_until(POSE, 6.0)
    cur = [node.pos.get(j, 99.0) for j in JOINTS]
    node.get_logger().info("腰部到位: " + ", ".join(f"{v:.3f}" for v in cur))
    ok2 = node.wait_until([0.0] * 5, 6.0)
    node.get_logger().info(f"结果: 到位={ok1} 回零={ok2}")
    ok = ok1 and ok2
    print("PASS: 腰部 JTC" if ok else "FAIL: 腰部 JTC")
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
