#!/usr/bin/env python3
"""导航目标中继节点:

web 页面通过 rosbridge 发布 /nav_goal_pose (geometry_msgs/PoseStamped),
本节点调用 nav2 的 NavigateToPose action, 并把状态/反馈回传给 web:
  /nav_status   (std_msgs/String): accepted/executing/succeeded/canceled/failed
  /nav_feedback (geometry_msgs/PoseStamped): 当前位姿反馈
"""
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Empty, String

from nav2_msgs.action import NavigateToPose


class NavGoalRelayNode(Node):

    def __init__(self):
        super().__init__("nav_goal_relay_node")
        self.pub_status = self.create_publisher(String, "nav_status", 10)
        self.pub_feedback = self.create_publisher(PoseStamped, "nav_feedback", 10)
        self.action = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.goal_handle = None
        self.create_subscription(PoseStamped, "nav_goal_pose", self.on_goal, 10)
        self.create_subscription(Empty, "nav_cancel", self.on_cancel, 10)
        self.get_logger().info("导航中继就绪: /nav_goal_pose -> navigate_to_pose, /nav_cancel -> cancel")

    def send_status(self, s):
        self.get_logger().info(f"导航状态: {s}")
        m = String()
        m.data = s
        self.pub_status.publish(m)

    def on_goal(self, msg: PoseStamped):
        if not self.action.wait_for_server(timeout_sec=2.0):
            self.send_status("failed: action server 不可用")
            return
        goal = NavigateToPose.Goal()
        goal.pose = msg
        self.send_status("accepted")
        fut = self.action.send_goal_async(
            goal, feedback_callback=self.on_feedback)
        fut.add_done_callback(self.on_goal_response)

    def on_cancel(self, _msg: Empty):
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
            self.send_status("canceling")
        else:
            self.send_status("idle")

    def on_feedback(self, feedback_msg):
        fb = feedback_msg.feedback
        if hasattr(fb, "current_pose"):
            self.pub_feedback.publish(fb.current_pose)

    def on_goal_response(self, future):
        handle = future.result()
        if handle is None or not handle.accepted:
            self.goal_handle = None
            self.send_status("failed: 目标被拒绝")
            return
        self.goal_handle = handle
        handle.get_result_async().add_done_callback(self.on_result)

    def on_result(self, future):
        self.goal_handle = None
        res = future.result()
        # nav2: STATUS_SUCCEEDED=4, CANCELED=5, ABORTED=6
        status = {2: "executing", 4: "succeeded", 5: "canceled",
                  6: "failed"}.get(res.status, f"status={res.status}")
        self.send_status(status)


def main(args=None):
    rclpy.init(args=args)
    node = NavGoalRelayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
