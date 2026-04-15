# d1_nav_demo（ROS2）

本文档位于**工作区根目录**（与 `src/` 同级）。ROS2 包源码在 `src/d1_nav_demo/`。

当前项目实际运行链路只使用 `src/d1_nav_demo/d1_nav_demo/circle_trajectory_tracker_node.py` 的编译产物（`ros2 launch` 启动的就是这个节点）。
输入是动捕位姿，输出是 `Twist`，不再依赖 RealSense 话题，也不依赖 B-spline / Path 跟踪链路。

D1 机内控制话题在 Ubuntu 不可见，所以本包只负责发布 `Twist`；`Twist -> D1 机内控制` 的桥接仍在 D1 侧单独实现。

---

## 一、当前实际数据流

1. 动捕系统发布位姿到 `pose_topic`（消息类型：`geometry_msgs/PoseStamped`）
2. `circle_trajectory_tracker_node` 按给定圆轨迹和当前位姿计算控制量
3. 发布 `cmd_vel_topic`（消息类型：`geometry_msgs/Twist`，仅使用 `linear.x` 和 `angular.z`）
4. D1 侧脚本订阅该 `Twist`，转换为机内控制命令

---

## 二、构建与启动

```bash
cd /mnt/home/hywork/d1_nav_demo_ros2_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch d1_nav_demo demo_pipeline.launch.py
```

启动前请确认：
- Ubuntu 与 D1 的 `ROS_DOMAIN_ID` 一致
- `src/d1_nav_demo/config/params.yaml` 中 `pose_topic` 已改成你的动捕话题（例如 `/Tracker007/pose`）
- D1 侧桥接脚本订阅的话题与 `cmd_vel_topic` 一致

---

## 三、`circle_trajectory_tracker_node` 数学控制逻辑

### 1) 坐标与姿态处理

- 使用第一帧位姿作为局部原点：`x = x_world - x0`，`y = y_world - y0`
- 航向角由四元数转换得到，再加上可配偏置：`yaw = wrap(yaw_from_quat + yaw_offset_rad)`

这里 `wrap(.)` 表示角度归一化到 `(-pi, pi]`。

### 2) 圆轨迹几何量

给定目标圆参数 `(cx, cy, R)`：
- `dx = x - cx`
- `dy = y - cy`
- `r = sqrt(dx^2 + dy^2)`
- 半径误差：`e_r = r - R`

若 `r` 过小（机器人在圆心附近），切线方向不可定义，节点会直接停车保护。

### 3) 目标切向航向 `psi_d`

- 逆时针（`clockwise = false`）：
  - `psi_d = atan2(dx, -dy)`
- 顺时针（`clockwise = true`）：
  - `psi_d = atan2(-dx, dy)`

航向误差定义为：
- `e_psi = wrap(psi_d - yaw)`

### 4) 线速度 `v_cmd`

先根据航向误差做缩放：
- `v_scale = max(0, cos(e_psi))`

再得到线速度命令：
- 若 `v_scale > 1e-3`：
  - `v_cmd = clamp(v_nom * v_scale, min_v, max_v)`
- 否则：
  - `v_cmd = min_v`

这样在航向偏差较大时仍保留最小前进速度，避免长时间原地打转。

### 5) 角速度 `w_cmd`

前馈角速度（圆轨迹曲率项）：
- `w_ff = direction * (v_cmd / R)`
- 其中 `direction = +1`（逆时针）或 `-1`（顺时针）

总角速度控制律：
- `w_cmd = w_ff + k_radius * e_r + k_heading * e_psi`
- 再做限幅：`w_cmd = clamp(w_cmd, -max_w, max_w)`
- 若 `0 < |w_cmd| < min_w`，则提升到最小角速度：`w_cmd = sign(w_cmd) * min_w`

### 6) 安全保护

- 位姿超时（`pose_timeout_sec`）时发布零 `Twist` 停车
- 还未收到位姿或原点未初始化时发布零 `Twist`

---

## 四、当前常用参数（`src/d1_nav_demo/config/params.yaml`）

- 话题参数：`pose_topic`、`cmd_vel_topic`
- 轨迹参数：`circle_center_x`、`circle_center_y`、`circle_radius_m`、`clockwise`
- 速度参数：`nominal_linear_x`、`min_linear_x`、`max_linear_x`、`min_angular_z`、`max_angular_z`
- 反馈增益：`k_radius`、`k_heading`
- 其他：`yaw_offset_rad`、`control_rate_hz`、`pose_timeout_sec`
