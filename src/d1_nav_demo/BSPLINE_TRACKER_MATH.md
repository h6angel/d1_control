# B-spline 跟踪节点说明（简单版）

对应代码：`d1_nav_demo/bspline_trajectory_tracker_node.py`

这个节点只做一件事：  
把 `/planning/bspline` 的轨迹，变成 D1 底盘的 `cmd_vel`（`linear.x` + `angular.z`）。

---

## 1. 输入和输出

输入：

- 轨迹：`/planning/bspline`（`traj_utils/msg/Bspline`）
- 位姿：`pose_topic`（默认 `/ov_msckf/poseimu`）
  - 同时兼容 `PoseWithCovarianceStamped` 和 `PoseStamped`

输出：

- 速度指令：`/d1_demo/cmd_vel_raw`（`geometry_msgs/msg/Twist`）

---

## 2. 收到一条 B-spline 后，节点做了什么

1) **先校验消息**

- 控制点数量太少、knot 数不对，直接丢弃。
- 定义域不合法（`u_max <= u_min`），直接丢弃。

2) **只取平面信息**

- 控制点只用 `x,y`，忽略 `z`。

3) **预计算导数曲线**

- 基于原曲线生成一阶导曲线（速度）
- 再生成二阶导曲线（加速度）

4) **处理轨迹时间对齐**

- 默认开启 `anchor_to_current_pose_on_new_traj`。
- 节点会先在新轨迹上找一个离机器人当前位置最近的参数 `u_anchor`。
- 然后把轨迹起始时间改成：`start_time = now - u_anchor`。  
  作用：避免因为上游 `start_time` 偏差，机器人去跟“旧相位”。

5) **退化轨迹保护**

- 如果控制点跨度非常小（近似单点轨迹），会报警。
- 且机器人离该点还远时，可拒收这条轨迹（保留上一条有效轨迹）。

---

## 3. 控制循环每次怎么算

控制频率：`control_rate_hz`（默认 30Hz）。

每个周期流程：

1) **先做超时保护**

- 没位姿 / 没轨迹：停车
- 位姿超时（`pose_timeout_sec`）：停车
- 轨迹超时（`traj_timeout_sec`）：停车

2) **计算当前轨迹参数**

- `tau = now - start_time`
- `u = clamp(tau, u_min, u_max)`

3) **在同一个 u 上求期望状态**

- 位置：`(x_d, y_d)`
- 速度：`(vx_d, vy_d)`
- 加速度：`(ax_d, ay_d)`

以上都用 `de Boor` 做数值求值。

4) **算误差**

- 世界系：`dx = x_d - x`, `dy = y_d - y`
- 转机体系：
  - `ex = cos(yaw)*dx + sin(yaw)*dy`
  - `ey = -sin(yaw)*dx + cos(yaw)*dy`

5) **算目标朝向**

- 正常：沿速度方向 `atan2(vy_d, vx_d)`
- 速度太小时：改为朝目标点 `atan2(dy, dx)`
- 角误差 `e_psi = wrap(psi_ref - yaw)`

6) **算前馈 + 反馈控制**

- `v_plan = sqrt(vx_d^2 + vy_d^2)`
- `w_ff = (vx_d*ay_d - vy_d*ax_d) / (vx_d^2 + vy_d^2)`（分母太小则置 0）

常规控制：

- `v_raw = v_plan*cos(e_psi) + k_x*ex`
- `w_raw = w_ff + k_heading*e_psi + k_y*ey`

7) **静止轨迹兜底**

- 若 `v_plan` 很小：
  - 离目标很近：直接停车
  - 离目标还远：退化为点跟踪，避免原地打转

8) **限幅 + 斜率限制 + 发布**

- 速度限幅：`v_raw -> [min_linear_x, max_linear_x]`
- 角速度限幅：`w_raw -> [-max_angular_z, max_angular_z]`
- 再做加速度限制（防突变）：
  - `|delta_v| <= max_linear_acc * dt`
  - `|delta_w| <= max_angular_acc * dt`
- 发布 `Twist`。

---

## 4. 核心数学（够用版）

- B-spline 曲线：`C(u) = Σ N(i,p,u) * P(i)`（代码用 de Boor 算，不手推基函数）
- 参数时间关系：`u = t_now - t_start`
- 一阶导控制点：  
  `P1(i) = p / (U(i+p+1)-U(i+1)) * (P(i+1)-P(i))`
- 二阶导：对一阶导曲线再求一次导
- 非完整底盘控制：
  - `v = v_plan*cos(e_psi) + k_x*ex`
  - `w = w_ff + k_heading*e_psi + k_y*ey`

---

## 5. 关键参数（先调这几个）

- `k_heading`：转向积极程度
- `k_x`：前后误差纠正力度
- `k_y`：横向误差纠正力度
- `max_linear_x`、`max_angular_z`：速度上限
- `max_linear_acc`、`max_angular_acc`：平滑程度

---

## 6. 和代码一致性的说明

本文按当前 `bspline_trajectory_tracker_node.py` 实现整理，包含代码中的几个实际细节：

- 新轨迹“当前位置锚定”
- 退化轨迹拒收
- 静止轨迹点跟踪兜底

这些逻辑都已在上面写明，不是理论扩展项。
