#!/usr/bin/env python3
"""测试 left_arm_joint_controller / right_arm_joint_controller (JTC):
双臂同步运动到目标位姿再回零, 校验 /joint_states 跟踪到位。
前置: control.launch.py 已启动。
"""
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

L_JOINTS = [f"idx2{i}_arm_l_joint{i}" for i in range(1, 8)]
R_JOINTS = [f"idx6{i}_arm_r_joint{i}" for i in range(1, 8)]
# 目标位 (各关节均在限位内, j4 限位 [-2.4958, 1.0123])
POSE = [0.30, 0.30, -0.30, 0.50, 0.30, 0.30, 0.20]
TOL = 0.05


class ArmTest(Node):

    def __init__(self):
        super().__init__("test_arm_joint_controllers")
        self.pos = {}
        self.pub_l = self.create_publisher(JointTrajectory,
                                           "left_arm_joint_controller/joint_trajectory", 10)
        self.pub_r = self.create_publisher(JointTrajectory,
                                           "right_arm_joint_controller/joint_trajectory", 10)
        self.create_subscription(JointState, "joint_states", self.on_js, 10)

    def on_js(self, msg: JointState):
        for n, p in zip(msg.name, msg.position):
            self.pos[n] = p

    def send(self, pub, joints, points):
        t = JointTrajectory()
        t.joint_names = joints
        for sec, pos in points:
            p = JointTrajectoryPoint()
            p.time_from_start.sec = int(sec)
            p.time_from_start.nanosec = int((sec - int(sec)) * 1e9)
            p.positions = [float(v) for v in pos]
            t.points.append(p)
        pub.publish(t)

    def wait_ready(self):
        for _ in range(50):
            rclpy.spin_once(self, timeout_sec=0.1)
            if all(j in self.pos for j in L_JOINTS + R_JOINTS):
                return True
        return False

    def wait_until(self, joints, target, timeout):
        t0 = time.time()
        while time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            cur = [self.pos.get(j, 99.0) for j in joints]
            if all(abs(c - t) < TOL for c, t in zip(cur, target)):
                return True
        return False


def main():
    rclpy.init()
    node = ArmTest()
    if not node.wait_ready():
        print("FAIL: 未收到 /joint_states")
        sys.exit(1)

    node.send(node.pub_l, L_JOINTS, [(3.0, POSE), (6.0, [0.0] * 7)])
    node.send(node.pub_r, R_JOINTS, [(3.0, POSE), (6.0, [0.0] * 7)])
    node.get_logger().info("双臂轨迹已发送: 目标位 -> 回零")

    ok1 = node.wait_until(L_JOINTS, POSE, 5.0)
    cur_l = [node.pos.get(j, 99.0) for j in L_JOINTS]
    node.get_logger().info("左臂到位: " + ", ".join(f"{v:.3f}" for v in cur_l))
    ok2 = node.wait_until(R_JOINTS, POSE, 3.0)
    cur_r = [node.pos.get(j, 99.0) for j in R_JOINTS]
    node.get_logger().info("右臂到位: " + ", ".join(f"{v:.3f}" for v in cur_r))
    ok3 = node.wait_until(L_JOINTS, [0.0] * 7, 6.0)
    ok4 = node.wait_until(R_JOINTS, [0.0] * 7, 3.0)
    node.get_logger().info(f"结果: 左臂到位={ok1} 右臂到位={ok2} 左臂回零={ok3} 右臂回零={ok4}")
    ok = ok1 and ok2 and ok3 and ok4
    print("PASS: 双臂 JTC" if ok else "FAIL: 双臂 JTC")
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
