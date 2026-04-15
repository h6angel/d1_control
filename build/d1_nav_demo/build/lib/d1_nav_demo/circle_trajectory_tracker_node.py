"""
沿固定平面圆轨迹运动：订阅外部位姿，根据几何关系计算线速度 linear.x / 角速度 angular.z，
发布 geometry_msgs/Twist，供下游桥接为 D1 UserCommand。

不使用 B-spline / 离散 Path，轨迹由参数圆 (cx, cy, R) 解析给定。
"""
import math
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped, Twist


def clamp(value: float, min_v: float, max_v: float) -> float:
    return max(min_v, min(max_v, value))


def yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny, cosy)


def angle_wrap(rad: float) -> float:
    return math.atan2(math.sin(rad), math.cos(rad))


class CircleTrajectoryTrackerNode(Node):
    def __init__(self) -> None:
        super().__init__("circle_trajectory_tracker_node")

        self.declare_parameter("pose_topic", "/Tracker007/pose")
        self.declare_parameter("cmd_vel_topic", "/d1_demo/cmd_vel_raw")
        self.declare_parameter("control_rate_hz", 40.0)

        self.declare_parameter("circle_center_x", 1.2)
        self.declare_parameter("circle_center_y", 1.2)
        self.declare_parameter("circle_radius_m", 1.0)
        self.declare_parameter("clockwise", False)

        self.declare_parameter("nominal_linear_x", 0.5)
        self.declare_parameter("min_linear_x", 0.2)
        self.declare_parameter("max_linear_x", 1.2)
        self.declare_parameter("min_angular_z", 0.3)
        self.declare_parameter("max_angular_z", 1.5)
        self.declare_parameter("k_radius", 0.8)
        self.declare_parameter("k_heading", 1.2)
        self.declare_parameter("yaw_offset_rad", 0.0)
        self.declare_parameter("pose_timeout_sec", 0.5)

        self.pose_topic = self.get_parameter("pose_topic").value
        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        self.rate_hz = float(self.get_parameter("control_rate_hz").value)

        self.cx = float(self.get_parameter("circle_center_x").value)
        self.cy = float(self.get_parameter("circle_center_y").value)
        self.radius = float(self.get_parameter("circle_radius_m").value)
        if self.radius < 0.05:
            self.get_logger().error("circle_radius_m too small, clamping to 0.05")
            self.radius = 0.05

        self.clockwise = bool(self.get_parameter("clockwise").value)
        self.direction = -1.0 if self.clockwise else 1.0

        self.v_nom = float(self.get_parameter("nominal_linear_x").value)
        self.min_v = float(self.get_parameter("min_linear_x").value)
        self.max_v = float(self.get_parameter("max_linear_x").value)
        self.min_w = float(self.get_parameter("min_angular_z").value)
        self.max_w = float(self.get_parameter("max_angular_z").value)
        if self.min_w < 0.0:
            self.get_logger().warn("min_angular_z < 0, clamping to 0.0")
            self.min_w = 0.0
        if self.min_w > self.max_w:
            self.get_logger().warn("min_angular_z > max_angular_z, clamping to max_angular_z")
            self.min_w = self.max_w
        self.k_r = float(self.get_parameter("k_radius").value)
        self.k_psi = float(self.get_parameter("k_heading").value)
        self.yaw_offset = float(self.get_parameter("yaw_offset_rad").value)
        self.pose_timeout = float(self.get_parameter("pose_timeout_sec").value)

        self.pose_msg: Optional[PoseStamped] = None
        self.last_pose_time_sec: Optional[float] = None
        self.origin_x: Optional[float] = None
        self.origin_y: Optional[float] = None

        pose_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.create_subscription(PoseStamped, self.pose_topic, self.pose_cb, pose_qos)
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 20)

        period = 1.0 / self.rate_hz if self.rate_hz > 0.0 else 0.05
        self.create_timer(period, self.control_loop)

        self.get_logger().info(
            f"Circle track: center=({self.cx:.2f},{self.cy:.2f}), R={self.radius:.2f}m, "
            f"{'CW' if self.clockwise else 'CCW'}"
        )

    def pose_cb(self, msg: PoseStamped) -> None:
        self.pose_msg = msg
        if self.origin_x is None or self.origin_y is None:
            # 将第一帧作为机器人局部坐标原点，满足“起始位置为 0,0,0”假设
            self.origin_x = msg.pose.position.x
            self.origin_y = msg.pose.position.y
            self.get_logger().info(
                f"Set local origin from first pose: ({self.origin_x:.3f}, {self.origin_y:.3f})"
            )
        self.last_pose_time_sec = self.get_clock().now().nanoseconds * 1e-9

    def publish_stop(self) -> None:
        self.cmd_pub.publish(Twist())

    def control_loop(self) -> None:
        if self.pose_msg is None:
            self.publish_stop()
            return

        now_sec = self.get_clock().now().nanoseconds * 1e-9
        if self.last_pose_time_sec is None or (now_sec - self.last_pose_time_sec) > self.pose_timeout:
            self.get_logger().warn("Pose timeout, stop robot.")
            self.publish_stop()
            return

        if self.origin_x is None or self.origin_y is None:
            self.publish_stop()
            return

        x_world = self.pose_msg.pose.position.x
        y_world = self.pose_msg.pose.position.y
        x = x_world - self.origin_x
        y = y_world - self.origin_y
        q = self.pose_msg.pose.orientation
        yaw = angle_wrap(yaw_from_quat(q.x, q.y, q.z, q.w) + self.yaw_offset)

        dx = x - self.cx
        dy = y - self.cy
        r = math.hypot(dx, dy)
        if r < 1e-4:
            self.get_logger().warn("Robot on circle center, cannot define tangent; stop.")
            self.publish_stop()
            return

        # 目标航向：圆切向（逆时针为 atan2(dx,-dy)，顺时针取相反切向）
        if self.clockwise:
            psi_d = math.atan2(-dx, dy)
        else:
            psi_d = math.atan2(dx, -dy)

        e_r = r - self.radius
        e_psi = angle_wrap(psi_d - yaw)

        # 参考曲率角速度 v/R，加上半径误差与航向误差反馈
        # 给一个最小前进速度，避免在大航向误差下长期原地打转
        v_scale = max(0.0, math.cos(e_psi))
        if v_scale > 1e-3:
            v_cmd = clamp(self.v_nom * v_scale, self.min_v, self.max_v)
        else:
            v_cmd = self.min_v
        w_ff = self.direction * (v_cmd / self.radius)
        w_cmd = w_ff + self.k_r * e_r + self.k_psi * e_psi
        w_cmd = clamp(w_cmd, -self.max_w, self.max_w)
        if abs(w_cmd) > 1e-3 and abs(w_cmd) < self.min_w:
            w_cmd = math.copysign(self.min_w, w_cmd)

        cmd = Twist()
        cmd.linear.x = v_cmd
        cmd.angular.z = w_cmd
        self.cmd_pub.publish(cmd)


def main() -> None:
    rclpy.init()
    node = CircleTrajectoryTrackerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
