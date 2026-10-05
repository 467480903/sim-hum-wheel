#!/usr/bin/env python3
"""Inspect arm Cartesian control or command two 1 mm left-hand moves.

Run after sourcing env.sh. Modes: move (default), trace, watch, dry-run.
All positions and rotations are relative to base_link, matching the web view.
"""

import argparse
import math
import sys
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation


WORKSPACE = Path(__file__).resolve().parents[3]
ROBOT_URDF = WORKSPACE / "src/g2_description/urdf/model.urdf"
CONTROL_URDF = WORKSPACE / "src/g2_description/urdf/model_control.urdf"
ARM_JOINTS = {
    "left": [f"idx2{i}_arm_l_joint{i}" for i in range(1, 8)],
    "right": [f"idx6{i}_arm_r_joint{i}" for i in range(1, 8)],
}
END_LINK = {"left": "arm_l_end_link", "right": "arm_r_end_link"}
WAIST_JOINTS = [f"idx0{i}_body_joint{i}" for i in range(1, 6)]


def say(*parts):
    print(*parts, flush=True)


def vec(values, digits=6):
    return "[" + ", ".join(f"{v:.{digits}f}" for v in values) + "]"


class Kinematics:
    def __init__(self):
        root = ET.parse(ROBOT_URDF).getroot()
        self.by_child = {j.find("child").get("link"): j for j in root.findall("joint")}
        self.chains = {}
        self.limits = {}
        for side, end_link in END_LINK.items():
            chain = []
            link = end_link
            while link in self.by_child:
                joint = self.by_child[link]
                chain.append(joint)
                link = joint.find("parent").get("link")
            if link != "base_link":
                raise ValueError(f"{end_link} does not connect to base_link")
            self.chains[side] = chain[::-1]
        for joint in root.findall("joint"):
            limit = joint.find("limit")
            if limit is not None and "lower" in limit.attrib and "upper" in limit.attrib:
                self.limits[joint.get("name")] = (
                    float(limit.get("lower")), float(limit.get("upper")))

    @staticmethod
    def numbers(element, key, default="0 0 0"):
        return np.fromstring(element.get(key, default), sep=" ", dtype=float)

    def fk(self, side, state):
        transform = np.eye(4)
        for joint in self.chains[side]:
            origin = joint.find("origin")
            if origin is not None:
                step = np.eye(4)
                step[:3, :3] = Rotation.from_euler(
                    "xyz", self.numbers(origin, "rpy")).as_matrix()
                step[:3, 3] = self.numbers(origin, "xyz")
                transform = transform @ step
            if joint.get("type") in ("revolute", "continuous"):
                angle = float(state.get(joint.get("name"), 0.0))
                axis_element = joint.find("axis")
                axis = self.numbers(axis_element, "xyz", "1 0 0")
                step = np.eye(4)
                step[:3, :3] = Rotation.from_rotvec(axis * angle).as_matrix()
                transform = transform @ step
            elif joint.get("type") == "prismatic":
                axis_element = joint.find("axis")
                axis = self.numbers(axis_element, "xyz", "1 0 0")
                step = np.eye(4)
                step[:3, 3] = axis * float(state.get(joint.get("name"), 0.0))
                transform = transform @ step
        return transform[:3, 3].copy(), Rotation.from_matrix(transform[:3, :3])

    def ik(self, side, state, target_pos, target_rot):
        names = ARM_JOINTS[side]
        start = np.array([state[name] for name in names], dtype=float)
        lower = np.array([self.limits[name][0] for name in names])
        upper = np.array([self.limits[name][1] for name in names])
        start = np.clip(start, lower + 1e-8, upper - 1e-8)
        candidate = dict(state)

        def residual(values):
            candidate.update(zip(names, values))
            pos, rot = self.fk(side, candidate)
            position_error = pos - target_pos
            rotation_error = (target_rot * rot.inv()).as_rotvec()
            return np.r_[position_error, rotation_error, 1e-4 * (values - start)]

        result = least_squares(residual, start, bounds=(lower, upper),
                               diff_step=1e-5, max_nfev=250,
                               ftol=1e-12, xtol=1e-12, gtol=1e-12)
        candidate.update(zip(names, result.x))
        actual_pos, actual_rot = self.fk(side, candidate)
        pos_error = np.linalg.norm(actual_pos - target_pos)
        rot_error = np.linalg.norm((target_rot * actual_rot.inv()).as_rotvec())
        return result.x, result.nfev, pos_error, rot_error


def initial_joint_state():
    root = ET.parse(CONTROL_URDF).getroot()
    result = {}
    for joint in root.findall("ros2_control/joint"):
        position = joint.find("state_interface[@name='position']/param[@name='initial_value']")
        if position is not None:
            result[joint.get("name")] = float(position.text)
    return result


def show_pose(kin, side, state, prefix=""):
    pos, rot = kin.fk(side, state)
    say(f"{prefix}{side:5s} xyz(m)={vec(pos)}  "
        f"rpy(deg)={vec(rot.as_euler('xyz', degrees=True), 2)}  "
        f"quat(xyzw)={vec(rot.as_quat(), 4)}")
    return pos, rot


def solve_one(kin, side, state, axis, label):
    start_pos, start_rot = kin.fk(side, state)
    target = start_pos + axis * 0.001
    say(f"\n{label}: starting FK xyz={vec(start_pos)}")
    say(f"{label}: requested delta(m)={vec(axis * 0.001)} "
        f"target xyz={vec(target)}; hold orientation")
    values, evaluations, pos_error, rot_error = kin.ik(
        side, state, target, start_rot)
    names = ARM_JOINTS[side]
    say(f"{label}: IK evaluations={evaluations} "
        f"position residual={pos_error * 1000:.4f} mm "
        f"rotation residual={math.degrees(rot_error):.5f} deg")
    say(f"{label}: joint delta(rad)={vec(values - np.array([state[n] for n in names]))}")
    say(f"{label}: trajectory positions(rad)={vec(values)}")
    if pos_error > 0.00015 or rot_error > 0.002:
        raise RuntimeError(f"{label}: IK failed to reach the 1 mm target")
    return values, target, start_rot


def dry_run(kin):
    state = initial_joint_state()
    say("Using control URDF initial joint positions; no ROS command will be sent.")
    show_pose(kin, "left", state)
    show_pose(kin, "right", state)
    for axis, label in [(np.array([0., 0., 1.]), "UP 1 mm"),
                        (np.array([0., 1., 0.]), "LEFT 1 mm")]:
        values, _, _ = solve_one(kin, "left", state, axis, label)
        state.update(zip(ARM_JOINTS["left"], values))
        show_pose(kin, "left", state, prefix="predicted ")


def run_ros(mode, kin):
    import rclpy
    from control_msgs.msg import JointTrajectoryControllerState
    from controller_manager_msgs.srv import ListControllers
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

    class Diagnostic(Node):
        def __init__(self):
            super().__init__("g2_position_diagnostic")
            self.state = {}
            self.last_controller_print = {}
            self.seen_commands = 0
            self.create_subscription(JointState, "/joint_states", self.on_joint_state, 10)
            for side in ("left", "right"):
                topic = f"/{side}_arm_joint_controller"
                self.create_subscription(
                    JointTrajectory, topic + "/joint_trajectory",
                    lambda msg, s=side: self.on_trajectory(s, msg), 10)
                self.create_subscription(
                    JointTrajectoryControllerState, topic + "/controller_state",
                    lambda msg, s=side: self.on_controller_state(s, msg), 10)
            self.left_pub = self.create_publisher(
                JointTrajectory, "/left_arm_joint_controller/joint_trajectory", 10)
            self.manager = self.create_client(
                ListControllers, "/controller_manager/list_controllers")

        def on_joint_state(self, msg):
            self.state.update(zip(msg.name, msg.position))

        def on_trajectory(self, side, msg):
            self.seen_commands += 1
            say(f"\nRECEIVED {side} trajectory #{self.seen_commands}: "
                f"{len(msg.points)} point(s), names={msg.joint_names}")
            if not msg.points:
                return
            if not all(name in self.state for name in WAIST_JOINTS + ARM_JOINTS[side]):
                say("  FK/IK pending: /joint_states has not supplied every joint")
                return
            current_pos, current_rot = kin.fk(side, self.state)
            say(f"  current FK xyz={vec(current_pos)}")
            for index, point in enumerate(msg.points):
                if len(point.positions) != len(msg.joint_names):
                    say(f"  point {index}: invalid positions length")
                    continue
                requested = dict(self.state)
                requested.update(zip(msg.joint_names, point.positions))
                if not all(name in requested for name in ARM_JOINTS[side]):
                    say(f"  point {index}: missing arm joints")
                    continue
                target_pos, target_rot = kin.fk(side, requested)
                delta_rot = (target_rot * current_rot.inv()).as_rotvec()
                say(f"  point {index} at {point.time_from_start.sec + point.time_from_start.nanosec * 1e-9:.3f}s")
                say(f"    command q(rad)={vec(point.positions)}")
                say(f"    commanded FK xyz={vec(target_pos)}, "
                    f"delta(mm)={vec((target_pos - current_pos) * 1000, 3)}, "
                    f"rotation delta(deg)={vec(np.degrees(delta_rot), 3)}")
                values, evaluations, pos_error, rot_error = kin.ik(
                    side, self.state, target_pos, target_rot)
                say(f"    independent IK: {evaluations} evaluations, "
                    f"pos error={pos_error * 1000:.4f}mm, "
                    f"rot error={math.degrees(rot_error):.4f}deg, "
                    f"q={vec(values)}")

        def on_controller_state(self, side, msg):
            if mode == "watch":
                return
            now = time.monotonic()
            if now - self.last_controller_print.get(side, 0) < 1.0:
                return
            self.last_controller_print[side] = now
            if msg.actual.positions and msg.desired.positions:
                error = np.array(msg.desired.positions) - np.array(msg.actual.positions)
                say(f"CONTROLLER {side}: desired={vec(msg.desired.positions, 4)} "
                    f"actual={vec(msg.actual.positions, 4)} "
                    f"max joint error={np.max(np.abs(error)):.5f} rad")

    def spin_until(node, condition, seconds):
        deadline = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if condition():
                return True
        return False

    rclpy.init()
    node = Diagnostic()
    try:
        required = WAIST_JOINTS + ARM_JOINTS["left"] + ARM_JOINTS["right"]
        if not spin_until(node, lambda: all(n in node.state for n in required), 10):
            raise RuntimeError("no complete /joint_states within 10 s; start the control stack")

        if mode != "watch":
            if not node.manager.wait_for_service(timeout_sec=3):
                raise RuntimeError("/controller_manager/list_controllers is unavailable")
            future = node.manager.call_async(ListControllers.Request())
            if not spin_until(node, future.done, 5):
                raise RuntimeError("controller manager did not answer within 5 s")
            controllers = {item.name: item.state for item in future.result().controller}
            say("Controller states:", {name: controllers.get(name, "missing") for name in
                                      ("left_arm_joint_controller", "right_arm_joint_controller")})
            if mode == "move" and controllers.get("left_arm_joint_controller") != "active":
                raise RuntimeError("left_arm_joint_controller is not active")

        if mode == "watch":
            say("Watching left/right end poses relative to base_link. Ctrl-C to stop.")
            next_print = 0.0
            while rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.2)
                now = time.monotonic()
                if now >= next_print:
                    show_pose(kin, "left", node.state)
                    show_pose(kin, "right", node.state)
                    next_print = now + 0.5
        elif mode == "trace":
            say("Watching arm trajectory topics and controller states. "
                "Click a position button in the browser; Ctrl-C to stop.")
            while rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.5)
        else:
            # One subscription is our own command echo; the controller is the second.
            if not spin_until(node, lambda: node.left_pub.get_subscription_count() >= 2, 5):
                raise RuntimeError("left arm trajectory controller has no command subscription")
            say("Left arm controller subscriber found. Baseline:")
            show_pose(kin, "left", node.state)
            for axis, label in [(np.array([0., 0., 1.]), "UP 1 mm"),
                                (np.array([0., 1., 0.]), "LEFT 1 mm")]:
                before_pos, _ = kin.fk("left", node.state)
                values, target, target_rot = solve_one(kin, "left", node.state, axis, label)
                command = JointTrajectory()
                command.joint_names = ARM_JOINTS["left"]
                point = JointTrajectoryPoint()
                point.positions = values.tolist()
                point.time_from_start.sec = 2
                command.points.append(point)
                node.left_pub.publish(command)
                say(f"{label}: PUBLISHED to /left_arm_joint_controller/joint_trajectory")
                received_before = node.seen_commands

                def reached():
                    pos, rot = kin.fk("left", node.state)
                    return (np.linalg.norm(pos - target) < 0.00025 and
                            np.linalg.norm((target_rot * rot.inv()).as_rotvec()) < 0.003)

                arrived = spin_until(node, reached, 6)
                after_pos, _ = show_pose(kin, "left", node.state, prefix="observed ")
                say(f"{label}: observed delta(mm)={vec((after_pos - before_pos) * 1000, 3)} "
                    f"target error(mm)={np.linalg.norm(after_pos - target) * 1000:.3f} "
                    f"command echo={'yes' if node.seen_commands > received_before else 'no'}")
                if not arrived:
                    raise RuntimeError(f"{label}: controller did not reach target within 6 s")
            say("PASS: left hand moved up 1 mm, then left 1 mm")
    except KeyboardInterrupt:
        say("Stopped.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode", nargs="?", choices=("move", "trace", "watch", "dry-run"),
                        default="move")
    args = parser.parse_args()
    kin = Kinematics()
    try:
        if args.mode == "dry-run":
            dry_run(kin)
        else:
            run_ros(args.mode, kin)
    except (RuntimeError, ValueError) as error:
        say(f"FAIL: {error}")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
