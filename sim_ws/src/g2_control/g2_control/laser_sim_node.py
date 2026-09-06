#!/usr/bin/env python3
"""虚拟 SLAM 激光雷达节点:

SLAM 虚拟硬件位于底盘前两轮中间向上 100mm (slam_sensor_link, URDF 中红球)。
本节点读取机器人实际位姿 (TF: odom -> slam_sensor_link), 对虚拟工业场景
(scene.yaml 中的墙体/桌子) 做二维光线投射, 发布 sensor_msgs/LaserScan,
供 slam_toolbox 建图 / AMCL 定位 / nav2 导航使用。
"""
import math
import os

import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener


class LaserSimNode(Node):

    def __init__(self):
        super().__init__("laser_sim_node")

        self.declare_parameter("scene_yaml", "")
        self.declare_parameter("frame_id", "slam_sensor_link")
        self.declare_parameter("scan_rate", 10.0)
        self.declare_parameter("samples", 360)
        self.declare_parameter("range_max", 12.0)
        self.declare_parameter("odom_frame", "odom")

        scene = self.get_parameter("scene_yaml").value
        if not scene:
            # 默认使用 g2_description 包内场景
            import ament_index_python
            pkg = ament_index_python.get_package_share_directory("g2_description")
            scene = os.path.join(pkg, "config", "scene.yaml")
        self.frame_id = self.get_parameter("frame_id").value
        self.odom_frame = self.get_parameter("odom_frame").value
        rate = float(self.get_parameter("scan_rate").value)
        self.n = int(self.get_parameter("samples").value)
        self.range_max = float(self.get_parameter("range_max").value)

        # 加载场景 -> 2D 线段 (仅保留高度高于激光平面的障碍)
        self.segments = []
        with open(scene, encoding="utf-8") as f:
            spec = yaml.safe_load(f)
        for b in spec["boxes"]:
            if b["height"] <= 0.31:      # 低于激光平面 (~0.307m) 的不遮挡
                continue
            cx, cy = b["center"]
            lx, ly = b["size"]
            x0, x1 = cx - lx / 2, cx + lx / 2
            y0, y1 = cy - ly / 2, cy + ly / 2
            pts = [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
            for i in range(4):
                self.segments.append((pts[i], pts[(i + 1) % 4]))
        self.get_logger().info(f"虚拟激光: {len(self.segments)} 线段来自 {scene}")

        self.tf_buf = Buffer()
        self.tf_listen = TransformListener(self.tf_buf, self)
        self.pub = self.create_publisher(LaserScan, "scan", qos_profile_sensor_data)
        self.timer = self.create_timer(1.0 / rate, self.tick)

    def tick(self):
        try:
            tf = self.tf_buf.lookup_transform(
                self.odom_frame, self.frame_id, rclpy.time.Time())
        except Exception:
            return
        t = tf.transform.translation
        q = tf.transform.rotation
        # 四元数 -> yaw
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        ox, oy = t.x, t.y

        msg = LaserScan()
        now = self.get_clock().now().to_msg()
        msg.header.stamp = now
        msg.header.frame_id = self.frame_id
        msg.angle_min = -math.pi
        msg.angle_max = math.pi - 2 * math.pi / self.n
        msg.angle_increment = 2 * math.pi / self.n
        msg.time_increment = 0.0
        msg.scan_time = 1.0 / 10.0
        msg.range_min = 0.05
        msg.range_max = self.range_max
        msg.ranges = [float("inf")] * self.n

        ca, sa = math.cos(yaw), math.sin(yaw)
        for i in range(self.n):
            a = msg.angle_min + i * msg.angle_increment
            # 激光方向 -> odom 系
            dx = ca * math.cos(a) - sa * math.sin(a)
            dy = sa * math.cos(a) + ca * math.sin(a)
            best = self.range_max
            for (p1, p2) in self.segments:
                d = self._ray_seg(ox, oy, dx, dy, p1, p2)
                if d is not None and d < best:
                    best = d
            msg.ranges[i] = best if best < self.range_max else float("inf")
        self.pub.publish(msg)

    @staticmethod
    def _ray_seg(ox, oy, dx, dy, p1, p2):
        """射线-线段求交, 返回距离或 None"""
        x1, y1 = p1
        x2, y2 = p2
        ex, ey = x2 - x1, y2 - y1
        denom = dx * ey - dy * ex
        if abs(denom) < 1e-12:
            return None
        # t: 沿射线参数, u: 沿线段参数 (0..1)
        t = ((x1 - ox) * ey - (y1 - oy) * ex) / denom
        u = ((x1 - ox) * dy - (y1 - oy) * dx) / denom
        if t >= 0.0 and 0.0 <= u <= 1.0:
            return t
        return None


def main(args=None):
    rclpy.init(args=args)
    node = LaserSimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
