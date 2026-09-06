#!/usr/bin/env python3
"""测试虚拟激光 (laser_sim_node):
校验 /scan 存在、frame 正确、数据条数/范围合理 (机器人位于 8x8 场景中心,
墙面距离 ~4m, 对角 ~5.66m)。前置: control.launch.py 已启动。
"""
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class LaserTest(Node):

    def __init__(self):
        super().__init__("test_laser_scan")
        self.scan = None
        self.create_subscription(LaserScan, "scan", self.on_scan,
                                 qos_profile_sensor_data)

    def on_scan(self, msg: LaserScan):
        self.scan = msg


def main():
    rclpy.init()
    node = LaserTest()
    t0 = time.time()
    while node.scan is None and time.time() - t0 < 5.0:
        rclpy.spin_once(node, timeout_sec=0.1)
    if node.scan is None:
        print("FAIL: 未收到 /scan")
        sys.exit(1)

    s = node.scan
    n = len(s.ranges)
    valid = [r for r in s.ranges if s.range_min < r < s.range_max and math.isfinite(r)]
    rmin = min(valid) if valid else -1.0
    rmax = max(valid) if valid else -1.0
    ang_span = math.degrees(s.angle_max - s.angle_min)
    node.get_logger().info(
        f"frame={s.header.frame_id} 线数={n} 角度范围={ang_span:.0f}° "
        f"有效={len(valid)} 最近={rmin:.2f}m 最远={rmax:.2f}m")

    # 场景设计: 8x8 场地, 四边各有 4m 墙 (角落有缺口, 穿缺口光线为 inf),
    # 左上角有 2x2 房间, 右下角有桌子 -> 有效光线约 70-80%
    ok = (n > 100 and len(valid) > n * 0.6 and
          1.0 < rmax < 7.5 and rmin > 0.1)
    print("PASS: 虚拟激光 /scan" if ok else
          f"FAIL: n={n} valid={len(valid)} rmin={rmin:.2f} rmax={rmax:.2f}")
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
