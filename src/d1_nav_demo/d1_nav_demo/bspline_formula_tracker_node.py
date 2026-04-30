import math
from dataclasses import dataclass
from typing import List, Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from traj_utils.msg import Bspline


Vec2 = Tuple[float, float]


def angle_wrap(rad: float) -> float:
    return math.atan2(math.sin(rad), math.cos(rad))


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny, cosy)


@dataclass
class ActiveBspline:
    traj_id: int
    start_time_sec: float
    knots: List[float]
    ctrl_xy: List[Vec2]


class BsplineFormulaTrackerNode(Node):
    def __init__(self) -> None:
        super().__init__("bspline_formula_tracker_node")

        # 话题按脚本内固定，不走参数文件：
        # - 订阅按原工程：/planning/bspline + /ov_msckf/poseimu
        # - 发布控制使用新话题，避免和原工程冲突
        self.bspline_topic = "/planning/bspline"
        self.pose_topic = "/ov_msckf/poseimu"
        self.cmd_vel_topic = "/d1_demo/cmd_vel_formula"

        self.declare_parameter("control_rate_hz", 30.0)
        self.declare_parameter("k_p", 1.2)
        self.declare_parameter("k_theta", 2.0)
        self.declare_parameter("phi_switch_dist_m", 0.08)
        self.declare_parameter("pose_timeout_sec", 0.5)
        self.declare_parameter("traj_timeout_sec", 1.0)

        self.control_rate_hz = float(self.get_parameter("control_rate_hz").value)
        self.k_p = float(self.get_parameter("k_p").value)
        self.k_theta = float(self.get_parameter("k_theta").value)
        self.phi_switch_dist = float(self.get_parameter("phi_switch_dist_m").value)
        self.pose_timeout_sec = float(self.get_parameter("pose_timeout_sec").value)
        self.traj_timeout_sec = float(self.get_parameter("traj_timeout_sec").value)

        self.cur_x: Optional[float] = None
        self.cur_y: Optional[float] = None
        self.cur_yaw: Optional[float] = None
        self.last_pose_time_sec: Optional[float] = None
        self.active_traj: Optional[ActiveBspline] = None

        self.create_subscription(Bspline, self.bspline_topic, self.bspline_cb, 20)
        self.create_subscription(PoseStamped, self.pose_topic, self.pose_stamped_cb, 50)
        self.create_subscription(PoseWithCovarianceStamped, self.pose_topic, self.pose_cov_cb, 50)
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 20)

        period = 1.0 / self.control_rate_hz if self.control_rate_hz > 0.0 else 0.05
        self.create_timer(period, self.control_loop)
        self.get_logger().info("Bspline formula tracker ready.")

    def pose_stamped_cb(self, msg: PoseStamped) -> None:
        p = msg.pose.position
        q = msg.pose.orientation
        self.cur_x = float(p.x)
        self.cur_y = float(p.y)
        self.cur_yaw = yaw_from_quat(q.x, q.y, q.z, q.w)
        self.last_pose_time_sec = self.get_clock().now().nanoseconds * 1e-9

    def pose_cov_cb(self, msg: PoseWithCovarianceStamped) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.cur_x = float(p.x)
        self.cur_y = float(p.y)
        self.cur_yaw = yaw_from_quat(q.x, q.y, q.z, q.w)
        self.last_pose_time_sec = self.get_clock().now().nanoseconds * 1e-9

    def bspline_cb(self, msg: Bspline) -> None:
        if int(msg.order) != 3:
            self.get_logger().warn("Only cubic B-spline(order=3) is supported.")
            return

        knots = [float(k) for k in msg.knots]
        ctrl_xy: List[Vec2] = [(float(p.x), float(p.y)) for p in msg.pos_pts]
        if len(ctrl_xy) < 4:
            self.get_logger().warn("Ignore bspline: need at least 4 control points.")
            return
        if len(knots) < len(ctrl_xy) + 4:
            self.get_logger().warn("Ignore bspline: knot size is invalid for cubic spline.")
            return

        start_t = float(msg.start_time.sec) + float(msg.start_time.nanosec) * 1e-9
        self.active_traj = ActiveBspline(
            traj_id=int(msg.traj_id),
            start_time_sec=start_t,
            knots=knots,
            ctrl_xy=ctrl_xy,
        )
        self.get_logger().info(f"Accepted cubic bspline traj_id={msg.traj_id}")

    def publish_stop(self) -> None:
        self.cmd_pub.publish(Twist())

    def find_segment_index(self, t: float, knots: List[float], n_ctrl: int) -> Optional[int]:
        seg_max = n_ctrl - 4
        if seg_max < 0:
            return None
        for i in range(seg_max + 1):
            if knots[i + 3] <= t < knots[i + 4]:
                return i
        if t >= knots[seg_max + 4]:
            return seg_max
        return None

    def eval_cubic_segment(self, t: float, knots: List[float], ctrl_xy: List[Vec2]) -> Optional[Tuple[Vec2, Vec2, Vec2]]:
        idx = self.find_segment_index(t, knots, len(ctrl_xy))
        if idx is None:
            return None

        ti = knots[idx + 3]
        dt = t - ti
        q0 = ctrl_xy[idx]
        q1 = ctrl_xy[idx + 1]
        q2 = ctrl_xy[idx + 2]
        q3 = ctrl_xy[idx + 3]

        # M * Q (按题目给定固定基矩阵)
        m0x = -q0[0] + 3.0 * q1[0] - 3.0 * q2[0] + q3[0]
        m1x = 3.0 * q0[0] - 6.0 * q1[0] + 3.0 * q2[0]
        m2x = -3.0 * q0[0] + 3.0 * q2[0]
        m3x = q0[0] + 4.0 * q1[0] + q2[0]

        m0y = -q0[1] + 3.0 * q1[1] - 3.0 * q2[1] + q3[1]
        m1y = 3.0 * q0[1] - 6.0 * q1[1] + 3.0 * q2[1]
        m2y = -3.0 * q0[1] + 3.0 * q2[1]
        m3y = q0[1] + 4.0 * q1[1] + q2[1]

        px = (dt * dt * dt * m0x + dt * dt * m1x + dt * m2x + m3x) / 6.0
        py = (dt * dt * dt * m0y + dt * dt * m1y + dt * m2y + m3y) / 6.0

        vx = (3.0 * dt * dt * m0x + 2.0 * dt * m1x + m2x) / 6.0
        vy = (3.0 * dt * dt * m0y + 2.0 * dt * m1y + m2y) / 6.0

        ax = (6.0 * dt * m0x + 2.0 * m1x) / 6.0
        ay = (6.0 * dt * m0y + 2.0 * m1y) / 6.0
        return (px, py), (vx, vy), (ax, ay)

    def control_loop(self) -> None:
        if self.active_traj is None or self.cur_x is None or self.cur_y is None or self.cur_yaw is None:
            self.publish_stop()
            return

        now_sec = self.get_clock().now().nanoseconds * 1e-9
        if self.last_pose_time_sec is None or (now_sec - self.last_pose_time_sec) > self.pose_timeout_sec:
            self.publish_stop()
            return

        t = now_sec - self.active_traj.start_time_sec
        end_time = self.active_traj.knots[len(self.active_traj.ctrl_xy)]
        if t < self.active_traj.knots[3]:
            self.publish_stop()
            return
        if t > end_time + self.traj_timeout_sec:
            self.publish_stop()
            return

        t_eval = clamp(t, self.active_traj.knots[3], end_time)
        state = self.eval_cubic_segment(t_eval, self.active_traj.knots, self.active_traj.ctrl_xy)
        if state is None:
            self.publish_stop()
            return

        (x_d, y_d), (vx, vy), (ax, ay) = state
        theta = self.cur_yaw

        ex_w = x_d - self.cur_x
        ey_w = y_d - self.cur_y
        pos_err = math.hypot(ex_w, ey_w)
        e_parallel = math.cos(theta) * ex_w + math.sin(theta) * ey_w

        v_parallel = math.cos(theta) * vx + math.sin(theta) * vy
        u1 = v_parallel + self.k_p * e_parallel

        denom = vx * vx + vy * vy
        theta_dot_d = 0.0 if denom < 1e-8 else (vx * ay - vy * ax) / denom
        # 目标点离机器人太近时，位置方向角会抖动；此时退化到速度切向角更稳定。
        if pos_err < self.phi_switch_dist and denom > 1e-8:
            phi = math.atan2(vy, vx)
        else:
            phi = math.atan2(ey_w, ex_w)
        alpha = angle_wrap(phi - theta)
        u2 = theta_dot_d + self.k_theta * alpha

        cmd = Twist()
        # 按给定公式直接输出控制量，不做额外限幅截断。
        cmd.linear.x = u1
        cmd.angular.z = u2
        self.cmd_pub.publish(cmd)


def main() -> None:
    rclpy.init()
    node = BsplineFormulaTrackerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
