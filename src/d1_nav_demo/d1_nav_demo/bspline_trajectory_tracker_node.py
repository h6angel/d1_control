import math
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from traj_utils.msg import Bspline


Vec2 = Tuple[float, float]


def clamp(value: float, min_v: float, max_v: float) -> float:
    return max(min_v, min(max_v, value))


def yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny, cosy)


def angle_wrap(rad: float) -> float:
    return math.atan2(math.sin(rad), math.cos(rad))


def find_span(u: float, knots: Sequence[float], degree: int, n_ctrl: int) -> int:
    n = n_ctrl - 1
    if u >= knots[n + 1]:
        return n
    if u <= knots[degree]:
        return degree

    low = degree
    high = n + 1
    mid = (low + high) // 2
    while u < knots[mid] or u >= knots[mid + 1]:
        if u < knots[mid]:
            high = mid
        else:
            low = mid
        mid = (low + high) // 2
    return mid


def de_boor_eval(u: float, degree: int, knots: Sequence[float], ctrl_pts: Sequence[Vec2]) -> Vec2:
    span = find_span(u, knots, degree, len(ctrl_pts))
    d = [ctrl_pts[span - degree + j] for j in range(degree + 1)]

    for r in range(1, degree + 1):
        for j in range(degree, r - 1, -1):
            i = span - degree + j
            den = knots[i + degree - r + 1] - knots[i]
            alpha = 0.0 if abs(den) < 1e-9 else (u - knots[i]) / den
            d0 = d[j - 1]
            d1 = d[j]
            d[j] = ((1.0 - alpha) * d0[0] + alpha * d1[0], (1.0 - alpha) * d0[1] + alpha * d1[1])
    return d[degree]


def derivative_curve(knots: Sequence[float], ctrl_pts: Sequence[Vec2], degree: int) -> Tuple[List[float], List[Vec2], int]:
    if degree <= 0 or len(ctrl_pts) < 2:
        return list(knots), [(0.0, 0.0)], 0

    out_pts: List[Vec2] = []
    for i in range(len(ctrl_pts) - 1):
        den = knots[i + degree + 1] - knots[i + 1]
        if abs(den) < 1e-9:
            out_pts.append((0.0, 0.0))
            continue
        scale = degree / den
        out_pts.append(((ctrl_pts[i + 1][0] - ctrl_pts[i][0]) * scale, (ctrl_pts[i + 1][1] - ctrl_pts[i][1]) * scale))
    return list(knots[1:-1]), out_pts, degree - 1


@dataclass
class ActiveTrajectory:
    traj_id: int
    start_time_sec: float
    degree: int
    knots: List[float]
    ctrl_xy: List[Vec2]
    u_min: float
    u_max: float
    knots_d1: List[float]
    ctrl_d1: List[Vec2]
    deg_d1: int
    knots_d2: List[float]
    ctrl_d2: List[Vec2]
    deg_d2: int


class BsplineTrajectoryTrackerNode(Node):
    def __init__(self) -> None:
        super().__init__("bspline_trajectory_tracker_node")

        self.declare_parameter("pose_topic", "/ov_msckf/poseimu")
        self.declare_parameter("bspline_topic", "/planning/bspline")
        self.declare_parameter("cmd_vel_topic", "/d1_demo/cmd_vel_raw")
        self.declare_parameter("control_rate_hz", 30.0)
        self.declare_parameter("pose_timeout_sec", 0.5)
        self.declare_parameter("traj_timeout_sec", 1.0)
        self.declare_parameter("k_x", 0.9)
        self.declare_parameter("k_y", 1.1)
        self.declare_parameter("k_heading", 0.9)
        self.declare_parameter("min_linear_x", 0.0)
        self.declare_parameter("max_linear_x", 0.18)
        self.declare_parameter("max_angular_z", 0.7)
        self.declare_parameter("max_linear_acc", 0.8)
        self.declare_parameter("max_angular_acc", 1.4)
        self.declare_parameter("min_speed_for_yaw", 0.01)
        self.declare_parameter("turning_speed_scale", 0.45)
        self.declare_parameter("low_speed_angular_cap", 0.35)
        self.declare_parameter("near_goal_dist_m", 0.05)
        self.declare_parameter("static_traj_speed_eps", 0.02)
        self.declare_parameter("k_static_linear", 0.8)
        self.declare_parameter("anchor_to_current_pose_on_new_traj", True)
        self.declare_parameter("anchor_search_samples", 80)
        self.declare_parameter("reject_degenerate_traj", True)
        self.declare_parameter("degenerate_ctrl_span_eps", 1e-3)
        self.declare_parameter("degenerate_far_dist_m", 0.2)

        self.pose_topic = self.get_parameter("pose_topic").value
        self.bspline_topic = self.get_parameter("bspline_topic").value
        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        self.control_rate_hz = float(self.get_parameter("control_rate_hz").value)
        self.pose_timeout_sec = float(self.get_parameter("pose_timeout_sec").value)
        self.traj_timeout_sec = float(self.get_parameter("traj_timeout_sec").value)
        self.k_x = float(self.get_parameter("k_x").value)
        self.k_y = float(self.get_parameter("k_y").value)
        self.k_heading = float(self.get_parameter("k_heading").value)
        self.min_v = float(self.get_parameter("min_linear_x").value)
        self.max_v = float(self.get_parameter("max_linear_x").value)
        self.max_w = float(self.get_parameter("max_angular_z").value)
        self.max_dv = float(self.get_parameter("max_linear_acc").value)
        self.max_dw = float(self.get_parameter("max_angular_acc").value)
        self.min_speed_for_yaw = float(self.get_parameter("min_speed_for_yaw").value)
        self.turning_speed_scale = float(self.get_parameter("turning_speed_scale").value)
        self.low_speed_angular_cap = float(self.get_parameter("low_speed_angular_cap").value)
        self.near_goal_dist = float(self.get_parameter("near_goal_dist_m").value)
        self.static_traj_speed_eps = float(self.get_parameter("static_traj_speed_eps").value)
        self.k_static_linear = float(self.get_parameter("k_static_linear").value)
        self.anchor_to_current_pose = bool(self.get_parameter("anchor_to_current_pose_on_new_traj").value)
        self.anchor_search_samples = int(self.get_parameter("anchor_search_samples").value)
        if self.anchor_search_samples < 8:
            self.anchor_search_samples = 8
        self.reject_degenerate_traj = bool(self.get_parameter("reject_degenerate_traj").value)
        self.degenerate_ctrl_span_eps = float(self.get_parameter("degenerate_ctrl_span_eps").value)
        self.degenerate_far_dist = float(self.get_parameter("degenerate_far_dist_m").value)

        self.cur_x: Optional[float] = None
        self.cur_y: Optional[float] = None
        self.cur_yaw: Optional[float] = None
        self.last_pose_time_sec: Optional[float] = None
        self.active_traj: Optional[ActiveTrajectory] = None
        self.last_v_cmd = 0.0
        self.last_w_cmd = 0.0
        self.last_static_log_traj_id: Optional[int] = None

        # 兼容两类常见位姿消息：/ov_msckf/poseimu 与 /Tracker*/pose。
        self.create_subscription(PoseWithCovarianceStamped, self.pose_topic, self.pose_cov_cb, 50)
        self.create_subscription(PoseStamped, self.pose_topic, self.pose_stamped_cb, 50)

        # 规划链路通常采用 BEST_EFFORT，使用显式 QoS 避免可靠性不匹配导致收不到轨迹。
        bspline_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.create_subscription(Bspline, self.bspline_topic, self.bspline_cb, bspline_qos)
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 20)

        period = 1.0 / self.control_rate_hz if self.control_rate_hz > 0.0 else 0.05
        self.dt = period
        self.create_timer(period, self.control_loop)

        self.get_logger().info("Bspline tracker ready.")

    def estimate_u_nearest_to_pose(self, knots: Sequence[float], ctrl_xy: Sequence[Vec2], degree: int, u_min: float, u_max: float) -> float:
        if self.cur_x is None or self.cur_y is None:
            return u_min
        if u_max <= u_min:
            return u_min

        best_u = u_min
        best_d2 = float("inf")
        samples = max(8, self.anchor_search_samples)
        for i in range(samples + 1):
            ratio = float(i) / float(samples)
            u = u_min + (u_max - u_min) * ratio
            px, py = de_boor_eval(u, degree, knots, ctrl_xy)
            d2 = (px - self.cur_x) * (px - self.cur_x) + (py - self.cur_y) * (py - self.cur_y)
            if d2 < best_d2:
                best_d2 = d2
                best_u = u
        return best_u

    def pose_cov_cb(self, msg: PoseWithCovarianceStamped) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.cur_x = float(p.x)
        self.cur_y = float(p.y)
        self.cur_yaw = yaw_from_quat(q.x, q.y, q.z, q.w)
        self.last_pose_time_sec = self.get_clock().now().nanoseconds * 1e-9

    def pose_stamped_cb(self, msg: PoseStamped) -> None:
        p = msg.pose.position
        q = msg.pose.orientation
        self.cur_x = float(p.x)
        self.cur_y = float(p.y)
        self.cur_yaw = yaw_from_quat(q.x, q.y, q.z, q.w)
        self.last_pose_time_sec = self.get_clock().now().nanoseconds * 1e-9

    def bspline_cb(self, msg: Bspline) -> None:
        if len(msg.pos_pts) < 4 or len(msg.knots) < 8:
            self.get_logger().warn("Ignore invalid bspline (too short).")
            return

        degree = int(msg.order)
        ctrl_xy: List[Vec2] = [(float(p.x), float(p.y)) for p in msg.pos_pts]
        knots = [float(k) for k in msg.knots]
        expected_knots = len(ctrl_xy) + degree + 1
        if len(knots) != expected_knots:
            self.get_logger().warn(
                f"Ignore bspline traj_id={msg.traj_id}: knot size mismatch ({len(knots)} != {expected_knots})."
            )
            return

        u_min = knots[degree]
        u_max = knots[len(ctrl_xy)]
        if u_max <= u_min:
            self.get_logger().warn("Ignore bspline: invalid knot domain.")
            return

        knots_d1, ctrl_d1, deg_d1 = derivative_curve(knots, ctrl_xy, degree)
        knots_d2, ctrl_d2, deg_d2 = derivative_curve(knots_d1, ctrl_d1, deg_d1)

        start_t_msg = float(msg.start_time.sec) + float(msg.start_time.nanosec) * 1e-9
        start_t = start_t_msg
        if self.anchor_to_current_pose and self.cur_x is not None and self.cur_y is not None:
            now_sec = self.get_clock().now().nanoseconds * 1e-9
            u_anchor = self.estimate_u_nearest_to_pose(knots, ctrl_xy, degree, u_min, u_max)
            # 将新轨迹“相位”锚定到当前位姿，避免上游 start_time 偏置导致从旧终点继续跟踪。
            start_t = now_sec - u_anchor

        span_x = max(p[0] for p in ctrl_xy) - min(p[0] for p in ctrl_xy)
        span_y = max(p[1] for p in ctrl_xy) - min(p[1] for p in ctrl_xy)
        span = max(span_x, span_y)
        if span < self.degenerate_ctrl_span_eps:
            px0, py0 = de_boor_eval(u_min, degree, knots, ctrl_xy)
            dist_to_traj = float("inf")
            if self.cur_x is not None and self.cur_y is not None:
                dist_to_traj = math.hypot(px0 - self.cur_x, py0 - self.cur_y)
            self.get_logger().warn(
                f"traj_id={msg.traj_id} looks degenerate (span={span:.6f}, dist_to_robot={dist_to_traj:.3f})."
            )
            # 上游偶发输出“单点轨迹”时，若机器人离该点还远，拒收该轨迹，保持上一条有效轨迹继续执行。
            if self.reject_degenerate_traj and dist_to_traj > self.degenerate_far_dist:
                self.get_logger().warn(
                    f"Reject degenerate traj_id={msg.traj_id}, keep previous active trajectory."
                )
                return

        self.active_traj = ActiveTrajectory(
            traj_id=int(msg.traj_id),
            start_time_sec=start_t,
            degree=degree,
            knots=knots,
            ctrl_xy=ctrl_xy,
            u_min=u_min,
            u_max=u_max,
            knots_d1=knots_d1,
            ctrl_d1=ctrl_d1,
            deg_d1=deg_d1,
            knots_d2=knots_d2,
            ctrl_d2=ctrl_d2,
            deg_d2=deg_d2,
        )
        self.get_logger().info(
            f"Accepted bspline traj_id={msg.traj_id}, ctrl={len(ctrl_xy)}, "
            f"anchor={'pose' if self.anchor_to_current_pose else 'msg_start_time'}"
        )

    def publish_stop(self) -> None:
        self.last_v_cmd = 0.0
        self.last_w_cmd = 0.0
        self.cmd_pub.publish(Twist())

    def control_loop(self) -> None:
        if self.cur_x is None or self.cur_y is None or self.cur_yaw is None or self.active_traj is None:
            self.publish_stop()
            return

        now_sec = self.get_clock().now().nanoseconds * 1e-9
        if self.last_pose_time_sec is None or (now_sec - self.last_pose_time_sec) > self.pose_timeout_sec:
            self.get_logger().warn("Pose timeout, stop robot.")
            self.publish_stop()
            return

        tau = now_sec - self.active_traj.start_time_sec
        if tau < self.active_traj.u_min:
            self.publish_stop()
            return

        if tau > self.active_traj.u_max + self.traj_timeout_sec:
            self.get_logger().warn("Trajectory expired, stop robot.")
            self.publish_stop()
            return

        u = clamp(tau, self.active_traj.u_min, self.active_traj.u_max)
        px_d, py_d = de_boor_eval(u, self.active_traj.degree, self.active_traj.knots, self.active_traj.ctrl_xy)
        vx_d, vy_d = de_boor_eval(u, self.active_traj.deg_d1, self.active_traj.knots_d1, self.active_traj.ctrl_d1)
        ax_d, ay_d = de_boor_eval(u, self.active_traj.deg_d2, self.active_traj.knots_d2, self.active_traj.ctrl_d2)

        cur_x = self.cur_x
        cur_y = self.cur_y
        yaw = self.cur_yaw

        dx = px_d - cur_x
        dy = py_d - cur_y
        ex = math.cos(yaw) * dx + math.sin(yaw) * dy
        ey = -math.sin(yaw) * dx + math.cos(yaw) * dy

        v_plan = math.hypot(vx_d, vy_d)
        dist_to_target = math.hypot(dx, dy)
        if v_plan > self.min_speed_for_yaw:
            psi_ref = math.atan2(vy_d, vx_d)
        else:
            psi_ref = math.atan2(dy, dx)
        e_psi = angle_wrap(psi_ref - yaw)

        denom = vx_d * vx_d + vy_d * vy_d
        w_ff = 0.0 if denom < 1e-6 else (vx_d * ay_d - vy_d * ax_d) / denom
        # 对“静止/退化”轨迹做兜底：近目标直接停，远目标按点跟踪，避免 v=0 且 w 饱和原地打转。
        if v_plan < self.static_traj_speed_eps:
            if dist_to_target < self.near_goal_dist:
                self.publish_stop()
                return
            if self.last_static_log_traj_id != self.active_traj.traj_id:
                self.get_logger().info(
                    f"traj_id={self.active_traj.traj_id} appears static (v_plan={v_plan:.4f}), fallback to point tracking."
                )
                self.last_static_log_traj_id = self.active_traj.traj_id
            v_raw = self.k_static_linear * dist_to_target * max(0.0, math.cos(e_psi))
            w_raw = self.k_heading * e_psi
        else:
            v_raw = v_plan * math.cos(e_psi) + self.k_x * ex
            w_raw = w_ff + self.k_heading * e_psi + self.k_y * ey
            # 大角度误差时自动降速，优先让机体姿态追上参考切向，减少弯道外飘。
            turn_scale = clamp(1.0 - self.turning_speed_scale * abs(e_psi) / math.pi, 0.25, 1.0)
            v_raw *= turn_scale

        v_limited = clamp(v_raw, self.min_v, self.max_v)
        w_limited = clamp(w_raw, -self.max_w, self.max_w)
        # 避免“线速度被压成 0 + 角速度饱和”导致长期原地打转：
        # 离目标还远时给一个小前进量，帮助尽快进入可跟踪姿态。
        if (
            dist_to_target > self.near_goal_dist
            and v_limited <= 1e-4
        ):
            v_limited = min(self.max_v, max(0.03, 0.15 * dist_to_target))
            # 低速救援阶段限制角速度，避免“几乎不走路 + 原地猛拧”。
            w_limited = clamp(w_limited, -self.low_speed_angular_cap, self.low_speed_angular_cap)

        dv_max = self.max_dv * self.dt
        dw_max = self.max_dw * self.dt
        v_cmd = clamp(v_limited, self.last_v_cmd - dv_max, self.last_v_cmd + dv_max)
        w_cmd = clamp(w_limited, self.last_w_cmd - dw_max, self.last_w_cmd + dw_max)

        cmd = Twist()
        cmd.linear.x = v_cmd
        cmd.angular.z = w_cmd
        self.cmd_pub.publish(cmd)

        self.last_v_cmd = v_cmd
        self.last_w_cmd = w_cmd


def main() -> None:
    rclpy.init()
    node = BsplineTrajectoryTrackerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
