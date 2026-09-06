#!/usr/bin/env python3
"""测试 position 控制器 (JointGroupPositionController):
左臂/右臂/腰 position 控制器当前为 inactive (与 JTC 共用命令接口)。
流程: 切换 JTC -> position -> 发位置命令 -> 校验 -> 切回 JTC。
前置: control.launch.py 已启动。
"""
import sys
import time

import rclpy
from rclpy.node import Node
from controller_manager_msgs.srv import SwitchController
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

GROUPS = {
    "left_arm_position_controller": [f"idx2{i}_arm_l_joint{i}" for i in range(1, 8)],
    "right_arm_position_controller": [f"idx6{i}_arm_r_joint{i}" for i in range(1, 8)],
    "waist_position_controller": [f"idx0{i}_body_joint{i}" for i in range(1, 6)],
}
JTC = ["left_arm_joint_controller", "right_arm_joint_controller",
       "waist_joint_controller"]
POSE = {
    "left_arm_position_controller": [0.2, 0.2, -0.2, 0.4, 0.2, 0.2, 0.1],
    "right_arm_position_controller": [0.2, 0.2, -0.2, 0.4, 0.2, 0.2, 0.1],
    "waist_position_controller": [-0.2, 0.3, 0.2, 0.1, -0.1],
}
TOL = 0.05


class PosTest(Node):

    def __init__(self):
        super().__init__("test_position_controllers")
        self.pos = {}
        self.pubs = {c: self.create_publisher(Float64MultiArray, f"{c}/commands", 10)
                     for c in GROUPS}
        self.create_subscription(JointState, "joint_states", self.on_js, 10)
        self.cli = self.create_client(SwitchController, "controller_manager/switch_controller")

    def on_js(self, msg: JointState):
        for n, p in zip(msg.name, msg.position):
            self.pos[n] = p

    def switch(self, activate, deactivate, timeout=10.0):
        req = SwitchController.Request()
        req.activate_controllers = activate
        req.deactivate_controllers = deactivate
        req.strictness = SwitchController.Request.BEST_EFFORT
        self.cli.call_async(req)
        t0 = time.time()
        while time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        return True

    def send_and_wait(self, ctrl, target, timeout=6.0):
        m = Float64MultiArray()
        m.data = [float(v) for v in target]
        self.pubs[ctrl].publish(m)
        joints = GROUPS[ctrl]
        t0 = time.time()
        while time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            cur = [self.pos.get(j, 99.0) for j in joints]
            if all(abs(c - t) < TOL for c, t in zip(cur, target)):
                return True
        return False


def main():
    rclpy.init()
    node = PosTest()
    for _ in range(50):
        rclpy.spin_once(node, timeout_sec=0.1)
        if all(j in node.pos for g in GROUPS.values() for j in g):
            break

    node.get_logger().info("切换: JTC -> position 控制器")
    node.switch(list(GROUPS.keys()), JTC)

    results = {}
    for ctrl, target in POSE.items():
        results[ctrl] = node.send_and_wait(ctrl, target)
        cur = [node.pos.get(j, 99.0) for j in GROUPS[ctrl]]
        node.get_logger().info(f"{ctrl} 到位={results[ctrl]}: " +
                               ", ".join(f"{v:.3f}" for v in cur))

    # 回零
    for ctrl in GROUPS:
        node.send_and_wait(ctrl, [0.0] * len(GROUPS[ctrl]), 6.0)

    node.get_logger().info("切换回: position -> JTC")
    node.switch(JTC, list(GROUPS.keys()))

    ok = all(results.values())
    print("PASS: position 控制器" if ok else f"FAIL: {results}")
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
