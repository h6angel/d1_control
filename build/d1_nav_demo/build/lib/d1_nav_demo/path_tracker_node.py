import math
from typing import Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Path


def clamp(value: float, min_v: float, max_v: float) -> float:
    return max(min_v, min(max_v, value))


def yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny, cosy)


def angle_wrap(rad: float) -> float:
    return math.atan2(math.sin(rad), math.cos(rad))


class PathTrackerNode(Node):
    def __init__(self) -> None:
        super().__init__("path_tracker_node")

        self.declare_parameter("pose_topic", "/ov_msckf/poseimu")
        self.declare_parameter("path_topic", "/d1_demo/path")
        self.declare_parameter("cmd_vel_topic", "/d1_demo/cmd_vel_raw")
        self.declare_parameter("control_rate_hz", 20.0)

        self.declare_parameter("lookahead_m", 0.6)
        self.declare_parameter("goal_tolerance_m", 0.15)
        self.declare_parameter("heading_kp", 1.2)
        self.declare_parameter("max_linear_x", 0.35)
        self.declare_parameter("max_angular_z", 0.9)
        self.declare_parameter("pose_timeout_sec", 0.5)

        self.pose_topic = self.get_parameter("pose_topic").value
        self.path_topic = self.get_parameter("path_topic").value
        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        self.control_rate_hz = float(self.get_parameter("control_rate_hz").value)
        self.lookahead = float(self.get_parameter("lookahead_m").value)
        self.goal_tol = float(self.get_parameter("goal_tolerance_m").value)
        self.heading_kp = float(self.get_parameter("heading_kp").value)
        self.max_v = float(self.get_parameter("max_linear_x").value)
        self.max_w = float(self.get_parameter("max_angular_z").value)
        self.pose_timeout = float(self.get_parameter("pose_timeout_sec").value)

        self.pose_msg: Optional[PoseWithCovarianceStamped] = None
        self.path_msg: Optional[Path] = None
        self.last_pose_time_sec: Optional[float] = None

        self.create_subscription(PoseWithCovarianceStamped, self.pose_topic, self.pose_cb, 30)
        self.create_subscription(Path, self.path_topic, self.path_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 20)

        period = 1.0 / self.control_rate_hz if self.control_rate_hz > 0.0 else 0.05
        self.create_timer(period, self.control_loop)

    def pose_cb(self, msg: PoseWithCovarianceStamped) -> None:
        self.pose_msg = msg
        self.last_pose_time_sec = self.get_clock().now().nanoseconds * 1e-9

    def path_cb(self, msg: Path) -> None:
        self.path_msg = msg

    def publish_stop(self) -> None:
        self.cmd_pub.publish(Twist())

    def control_loop(self) -> None:
        if self.pose_msg is None or self.path_msg is None or len(self.path_msg.poses) == 0:
            self.publish_stop()
            return

        now_sec = self.get_clock().now().nanoseconds * 1e-9
        if self.last_pose_time_sec is None or (now_sec - self.last_pose_time_sec) > self.pose_timeout:
            self.get_logger().warn("Pose timeout, stop robot.")
            self.publish_stop()
            return

        cur_x = self.pose_msg.pose.pose.position.x
        cur_y = self.pose_msg.pose.pose.position.y
        q = self.pose_msg.pose.pose.orientation
        cur_yaw = yaw_from_quat(q.x, q.y, q.z, q.w)

        goal = self.path_msg.poses[-1].pose.position
        goal_dist = math.hypot(goal.x - cur_x, goal.y - cur_y)
        if goal_dist < self.goal_tol:
            self.publish_stop()
            return

        target = goal
        for pose_stamped in self.path_msg.poses:
            p = pose_stamped.pose.position
            if math.hypot(p.x - cur_x, p.y - cur_y) >= self.lookahead:
                target = p
                break

        heading = math.atan2(target.y - cur_y, target.x - cur_x)
        heading_err = angle_wrap(heading - cur_yaw)

        # When heading error is large, reduce linear speed for safety.
        v_scale = 1.0 - min(abs(heading_err), 1.2) / 1.2
        linear_x = clamp(self.max_v * v_scale, 0.0, self.max_v)
        angular_z = clamp(self.heading_kp * heading_err, -self.max_w, self.max_w)

        cmd = Twist()
        cmd.linear.x = linear_x
        cmd.angular.z = angular_z
        self.cmd_pub.publish(cmd)


def main() -> None:
    rclpy.init()
    node = PathTrackerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
