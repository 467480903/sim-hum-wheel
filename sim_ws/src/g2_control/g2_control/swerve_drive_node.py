#!/usr/bin/env python3
"""4 舵轮底盘 运动学 + 里程计 节点 (基于业界通用 4WIS/swerve 控制方案):

逆运动学 (cmd_vel -> 每轮 转向角+轮速):
  轮 i 位于底盘坐标 (lx, ly), 车体速度 (vx, vy, wz):
    vx_i = vx - wz*ly
    vy_i = vy + wz*lx
    转向角_i = atan2(vy_i, vx_i)      # 转向关节连续 0-360, 取最短角差
    轮速_i  = hypot(vx_i, vy_i) / r   # 驱动轮无限位, r=轮半径(直径0.14/2)
  转向角按最大转向角速度限幅; 转向误差大时按 cos(err) 缩放轮速, 避免侧滑。
  零速时保持当前转向角 (避免抖动)。

正运动学/里程计 (轮速+转向角 -> /odom):
  每轮线速度方向 (cos t_i, sin t_i)*w_i*r, 联立 vx_i = vx - wz*ly, vy_i = vy + wz*lx
  4 轮 8 方程最小二乘解 (vx, vy, wz), 按轮直径积分得里程, 发布 /odom + TF odom->base_link
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from tf2_ros import TransformBroadcaster


def wrap_pi(a: float) -> float:
    """归一化角度到 [-pi, pi]"""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class SwerveDriveNode(Node):

    # 舵轮布置 (与 swerve_*_controller 关节顺序一致: 左前, 左后, 右前, 右后)
    WHEELS = [
        # (转向关节, 驱动关节, lx, ly)
        ("idx111_chassis_lwheel_front_joint1", "idx112_chassis_lwheel_front_joint2", 0.23, 0.218),
        ("idx121_chassis_lwheel_rear_joint1", "idx122_chassis_lwheel_rear_joint2", -0.23, 0.218),
        ("idx131_chassis_rwheel_front_joint1", "idx132_chassis_rwheel_front_joint2", 0.23, -0.218),
        ("idx141_chassis_rwheel_rear_joint1", "idx142_chassis_rwheel_rear_joint2", -0.23, -0.218),
    ]

    def __init__(self):
        super().__init__("swerve_drive_node")

        self.declare_parameter("wheel_radius", 0.07)          # 轮半径 m (直径 0.14)
        self.declare_parameter("max_steer_rate", 4.0)         # 转向角速度 rad/s (URDF 4.08)
        self.declare_parameter("max_wheel_speed", 20.0)       # 轮最大角速度 rad/s (URDF 20.9)
        self.declare_parameter("max_lin_vel", 1.0)            # 车体最大线速度 m/s
        self.declare_parameter("max_ang_vel", 1.5)            # 车体最大角速度 rad/s
        self.declare_parameter("cmd_timeout", 0.5)            # cmd_vel 超时 s
        self.declare_parameter("base_z", 0.04)                # base_link 离地高度 (轮子着地)
        self.declare_parameter("steer_ready_tol", 0.05)       # 转向到位阈值 rad (~3°), 到位后才驱动

        self.r = float(self.get_parameter("wheel_radius").value)
        self.max_steer_rate = float(self.get_parameter("max_steer_rate").value)
        self.max_wheel_speed = float(self.get_parameter("max_wheel_speed").value)
        self.max_lin = float(self.get_parameter("max_lin_vel").value)
        self.max_ang = float(self.get_parameter("max_ang_vel").value)
        self.cmd_timeout = float(self.get_parameter("cmd_timeout").value)
        self.base_z = float(self.get_parameter("base_z").value)
        self.steer_ready_tol = float(self.get_parameter("steer_ready_tol").value)

        # 当前状态
        self.steer_pos = [0.0] * 4            # 当前转向角 (来自 /joint_states)
        self.drive_vel = [0.0] * 4
        self.cmd = (0.0, 0.0, 0.0)            # 最新 cmd_vel (vx, vy, wz)
        self.last_cmd_t = self.get_clock().now()
        self.have_js = False

        # 里程计
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.last_js_t = None

        self.create_subscription(Twist, "cmd_vel", self.on_cmd_vel, 10)
        self.create_subscription(JointState, "joint_states", self.on_joint_states,
                                 qos_profile_sensor_data)
        self.pub_steer = self.create_publisher(
            Float64MultiArray, "swerve_steering_controller/commands", 10)
        self.pub_drive = self.create_publisher(
            Float64MultiArray, "swerve_drive_controller/commands", 10)
        self.pub_odom = self.create_publisher(Odometry, "odom", 20)
        self.tf_broad = TransformBroadcaster(self)

        self.timer = self.create_timer(0.02, self.control_loop)   # 50Hz
        self.get_logger().info(
            f"swerve_drive_node 启动: 4 舵轮, 轮半径 {self.r}m (直径 {2 * self.r}m)")

    # ---------------- cmd_vel -> 轮命令 (逆运动学) ----------------
    def on_cmd_vel(self, msg: Twist):
        vx = max(-self.max_lin, min(self.max_lin, msg.linear.x))
        vy = max(-self.max_lin, min(self.max_lin, msg.linear.y))
        wz = max(-self.max_ang, min(self.max_ang, msg.angular.z))
        self.cmd = (vx, vy, wz)
        self.last_cmd_t = self.get_clock().now()

    def control_loop(self):
        if not self.have_js:
            return
        now = self.get_clock().now()
        if (now - self.last_cmd_t).nanoseconds * 1e-9 > self.cmd_timeout:
            self.cmd = (0.0, 0.0, 0.0)   # 超时停车

        vx, vy, wz = self.cmd
        speed = math.hypot(vx, vy)
        moving = speed > 1e-3 or abs(wz) > 1e-3

        steer_cmd = list(self.steer_pos)    # 零速保持当前角
        drive_cmd = [0.0] * 4

        if moving:
            dt = 0.02
            max_step = self.max_steer_rate * dt
            for i, (_sj, _dj, lx, ly) in enumerate(self.WHEELS):
                vxi = vx - wz * ly
                vyi = vy + wz * lx
                wi = math.hypot(vxi, vyi)
                target = math.atan2(vyi, vxi)
                # 最短角差 (转向 0-360 连续, 不需要翻转轮速方向)
                err = wrap_pi(target - self.steer_pos[i])
                if abs(err) > self.steer_ready_tol:
                    # 尚未转向到位: 只做转向, 不驱动里程轮, 避免转向过程侧滑
                    steer_cmd[i] = self.steer_pos[i] + max(-max_step, min(max_step, err))
                    drive_cmd[i] = 0.0
                else:
                    # 转向到位: 锁定目标角 (连续等价形式), 再驱动里程轮
                    steer_cmd[i] = self.steer_pos[i] + err
                    w = wi / self.r
                    drive_cmd[i] = max(-self.max_wheel_speed,
                                       min(self.max_wheel_speed, w))

        msg_s = Float64MultiArray()
        msg_s.data = steer_cmd
        msg_d = Float64MultiArray()
        msg_d.data = drive_cmd
        self.pub_steer.publish(msg_s)
        self.pub_drive.publish(msg_d)

    # ---------------- 轮状态 -> 里程计 (正运动学) ----------------
    def on_joint_states(self, msg: JointState):
        idx = {n: i for i, n in enumerate(msg.name)}
        ok = all(w[0] in idx and w[1] in idx for w in self.WHEELS)
        if not ok:
            return
        try:
            if msg.velocity is None or len(msg.velocity) != len(msg.name):
                return
        except Exception:
            return

        steer = [msg.position[idx[w[0]]] for w in self.WHEELS]
        drive = [msg.velocity[idx[w[1]]] for w in self.WHEELS]
        self.steer_pos = steer          # 保持连续累积值 (continuous 关节)
        self.drive_vel = drive
        self.have_js = True

        # 时间
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if t == 0.0:
            t = self.get_clock().now().nanoseconds * 1e-9
        if self.last_js_t is None:
            self.last_js_t = t
            return
        dt = t - self.last_js_t
        if dt <= 0.0 or dt > 0.5:
            self.last_js_t = t
            return
        self.last_js_t = t

        # 每轮线速度向量 (车体系): v_i = w_i * r * (cos t_i, sin t_i)
        # 满足: vx_i = vx - wz*ly ; vy_i = vy + wz*lx  ->  最小二乘
        # A [vx, vy, wz]^T = b ; 每轮两行
        a_rows = []
        b_rows = []
        for i, (_sj, _dj, lx, ly) in enumerate(self.WHEELS):
            vxi = drive[i] * self.r * math.cos(steer[i])
            vyi = drive[i] * self.r * math.sin(steer[i])
            a_rows += [[1.0, 0.0, -ly], [0.0, 1.0, lx]]
            b_rows += [vxi, vyi]

        # 3x3 正规方程求解 (无需 numpy)
        n = 3
        ata = [[sum(a_rows[k][i] * a_rows[k][j] for k in range(8)) for j in range(n)] for i in range(n)]
        atb = [sum(a_rows[k][i] * b_rows[k] for k in range(8)) for i in range(n)]
        vx, vy, wz = self._solve3(ata, atb)

        # 积分 (车体系 -> 里程系)
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        self.x += (vx * c - vy * s) * dt
        self.y += (vx * s + vy * c) * dt
        self.yaw = wrap_pi(self.yaw + wz * dt)
        self._publish_odom(t, vx, vy, wz)

    @staticmethod
    def _solve3(a, b):
        """高斯消元解 3x3"""
        m = [row[:] + [b[i]] for i, row in enumerate(a)]
        for col in range(3):
            piv = max(range(col, 3), key=lambda r: abs(m[r][col]))
            if abs(m[piv][col]) < 1e-12:
                return 0.0, 0.0, 0.0
            m[col], m[piv] = m[piv], m[col]
            for r in range(3):
                if r != col:
                    f = m[r][col] / m[col][col]
                    for c2 in range(col, 4):
                        m[r][c2] -= f * m[col][c2]
        return (m[0][3] / m[0][0], m[1][3] / m[1][1], m[2][3] / m[2][2])

    def _publish_odom(self, t, vx, vy, wz):
        odom = Odometry()
        odom.header.stamp.sec = int(t)
        odom.header.stamp.nanosec = int((t - int(t)) * 1e9)
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = self.base_z
        odom.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.yaw / 2.0)
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz
        self.pub_odom.publish(odom)

        tf = TransformStamped()
        tf.header = odom.header
        tf.child_frame_id = "base_link"
        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.translation.z = self.base_z
        tf.transform.rotation = odom.pose.pose.orientation
        self.tf_broad.sendTransform(tf)


def main(args=None):
    rclpy.init(args=args)
    node = SwerveDriveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
