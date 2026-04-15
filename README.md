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

## 三、`circle_trajectory_tracker_node` 数学控制逻辑（与代码一致）

下面记号与 `circle_trajectory_tracker_node.py` 中的变量一一对应，便于你对照实现或改参数。

### 0) 控制周期与输入输出

- 节点以固定频率 `control_rate_hz` 运行定时器；每次触发时读**最新**位姿并发布一次 `Twist`（离散时间实现，非连续时间控制器）。
- 输出：`cmd.linear.x = v_cmd`，`cmd.angular.z = w_cmd`；其它分量为 0。
- 位姿话题类型为 `PoseStamped`，控制只用平面位置 `(x_world, y_world)` 与航向 `yaw`（由四元数提取）。

### 1) 坐标系：世界坐标 → 以首帧为原点的局部平面

动捕给出的 `pose.position` 记为世界系下的 \((x_{\mathrm{world}}, y_{\mathrm{world}})\)。收到**第一帧**位姿时，记录：

- \(x_0 = x_{\mathrm{world}}\big|_{t=t_0}\)，\(y_0 = y_{\mathrm{world}}\big|_{t=t_0}\)

之后始终用局部坐标（等价于把起点平移到原点）：

- \(x = x_{\mathrm{world}} - x_0\)，\(y = y_{\mathrm{world}} - y_0\)

**含义**：圆心参数 `(circle_center_x, circle_center_y)` 与半径等几何量，都应理解为**在同一局部平面**里描述；若你期望圆心在动捕世界系的固定点，需要自行把该点换算到“首帧为原点”的坐标里，或改代码取消首帧归零。

### 2) 航向角：四元数 → yaw，再偏置与归一化

记位姿四元数为 \((q_x, q_y, q_z, q_w)\)（ROS 约定 \(q_w\) 为实部）。代码使用常见的 **绕 \(z\) 轴 yaw** 提取公式：

- \(\mathrm{siny} = 2(q_w q_z + q_x q_y)\)
- \(\mathrm{cosy} = 1 - 2(q_y^2 + q_z^2)\)
- \(\mathrm{yaw}_{\mathrm{raw}} = \mathrm{atan2}(\mathrm{siny}, \mathrm{cosy})\)

再加上参数 `yaw_offset_rad`（用于动捕坐标系与机器人前向不一致时的常值对齐）：

- \(\mathrm{yaw} = \mathrm{wrap}(\mathrm{yaw}_{\mathrm{raw}} + \mathrm{yaw\_offset\_rad})\)

其中 **wrap** 与代码中 `angle_wrap` 相同：

- \(\mathrm{wrap}(\theta) = \mathrm{atan2}(\sin\theta, \cos\theta)\)

结果落在 \((-\pi, \pi]\)，避免角度累加溢出。

### 3) 圆与误差：径向距离与半径误差

给定圆心 \((c_x, c_y)\)（参数 `circle_center_x/y`）与标称半径 \(R\)（`circle_radius_m`）：

- \(\Delta x = x - c_x\)，\(\Delta y = y - c_y\)
- \(r = \sqrt{\Delta x^2 + \Delta y^2}\)
- **半径误差**（正表示在外侧）：\(e_r = r - R\)

**半径下限（初始化时）**：若配置的 \(R < 0.05\,\mathrm{m}\)，代码会把 \(R\) 钳到 `0.05`，避免数值问题。

**奇异点**：若 \(r < 10^{-4}\)（几乎在圆心上），切向方向退化，代码发布零速度并返回（停车保护）。

### 4) 期望航向：沿圆的切向 `psi_d`

目标航向取为**过当前位置、沿约定转向绕行**的切线方向（在二维平面内）：

- **逆时针**（`clockwise = false`，`direction = +1`）：
  - \(\psi_d = \mathrm{atan2}(\Delta x,\,-\Delta y)\)
- **顺时针**（`clockwise = true`，`direction = -1`）：
  - \(\psi_d = \mathrm{atan2}(-\Delta x,\,\Delta y)\)

**航向误差**（期望朝向减去当前朝向，并归一化）：

- \(e_\psi = \mathrm{wrap}(\psi_d - \mathrm{yaw})\)

几何直观：\(e_\psi\) 接近 0 表示机体朝向与“该绕圈方向上的切向”一致；\(\cos(e_\psi)\) 接近 1 表示前向与切向同向分量最大。

### 5) 线速度 `v_cmd`：用 \(\cos(e_\psi)\) 缩放 + 最小前进速度

代码里 `clamp(a, lo, hi) = max(lo, min(hi, a))`。

先定义：

- \(s = \max(0,\,\cos(e_\psi))\)

再分两支：

- 若 \(s > 10^{-3}\)：
  - \(v_{\mathrm{cmd}} = \mathrm{clamp}(v_{\mathrm{nom}}\cdot s,\; v_{\min},\, v_{\max})\)
    - \(v_{\mathrm{nom}}\) ← `nominal_linear_x`
    - \(v_{\min}, v_{\max}\) ← `min_linear_x`, `max_linear_x`
- 否则（\(e_\psi\) 接近 \(\pm\pi/2\) 或更大，\(\cos\) 接近 0）：
  - \(v_{\mathrm{cmd}} = v_{\min}\)

**设计意图**：

- \(v_{\mathrm{nom}}\cos(e_\psi)\) 类似把标称切向速度投影到机体前向：航向差大时自动减小线速度，避免“横着冲”。
- 但在 \(\cos\) 很小时仍强制 \(v_{\min} > 0\)，避免车辆在大航向误差下**长期线速度过小、只靠原地转向**（代码注释里的“避免长期原地打转”）。

### 6) 角速度 `w_cmd`：曲率前馈 + 径向/航向反馈

**前馈**（匀速圆运动时 \(\omega \approx v/R\) 的经典关系，符号由顺/逆时针决定）：

- \(\omega_{\mathrm{ff}} = \mathrm{direction}\cdot \dfrac{v_{\mathrm{cmd}}}{R}\)
  - 逆时针：`direction = +1`
  - 顺时针：`direction = -1`

**反馈**（把半径误差与航向误差拉回到环上）：

- \(\omega_{\mathrm{cmd}} = \omega_{\mathrm{ff}} + k_r\, e_r + k_\psi\, e_\psi\)
  - \(k_r\) ← `k_radius`
  - \(k_\psi\) ← `k_heading`

**限幅与最小角速度**：

- \(\omega_{\mathrm{cmd}} \leftarrow \mathrm{clamp}(\omega_{\mathrm{cmd}},\,-\omega_{\max},\,\omega_{\max})\)（\(\omega_{\max}\) ← `max_angular_z`）
- 若 \(|\omega_{\mathrm{cmd}}| > 10^{-3}\) 且 \(|\omega_{\mathrm{cmd}}| < \omega_{\min}\)（\(\omega_{\min}\) ← `min_angular_z`），则把幅值抬到 \(\omega_{\min}\)，符号不变（代码为 `copysign`）。

**初始化对 `min_angular_z` 的修正**（与参数合法性有关）：

- 若 `min_angular_z < 0`，会先钳到 `0`。
- 若 `min_angular_z > max_angular_z`，会把 `min_angular_z` 钳到 `max_angular_z`。

### 7) 安全与边界条件汇总

| 条件 | 行为 |
|------|------|
| 尚未收到位姿 | 发布零 `Twist` |
| 首帧尚未记录原点（理论上与上一条同时出现） | 发布零 `Twist` |
| 距上次位姿时间戳超过 `pose_timeout_sec` | 发布零 `Twist`（超时停车） |
| \(r < 10^{-4}\)（在圆心附近） | 发布零 `Twist` |

### 8) 调参提示（简短）

- **跟圆更紧**：略增大 `k_heading`（注意振荡）或略增大 `k_radius`（径向拉回更强）。
- **更平滑**：略减小 `nominal_linear_x` 或减小 `k_heading` / `k_radius`。
- **航向与动捕前向不一致**：先调 `yaw_offset_rad`，再动增益。
- **圆心在世界系固定点**：记得换算到“首帧归零”后的局部坐标，或改代码逻辑。

---

## 四、与数学逻辑相关的参数一览

在 `src/d1_nav_demo/config/params.yaml` 里覆盖的项会**覆盖**代码默认值；未写出的参数将使用 `circle_trajectory_tracker_node.py` 里 `declare_parameter(..., default)` 的默认值。

- 话题：`pose_topic`、`cmd_vel_topic`
- 控制周期：`control_rate_hz`
- 圆轨迹：`circle_center_x`、`circle_center_y`、`circle_radius_m`、`clockwise`
- 线速度：`nominal_linear_x`、`min_linear_x`、`max_linear_x`
- 角速度：`min_angular_z`、`max_angular_z`
- 反馈增益：`k_radius`、`k_heading`
- 其它：`yaw_offset_rad`、`pose_timeout_sec`
