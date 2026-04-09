# d1_nav_demo（ROS2）

这是一个独立的 ROS2 最小演示包，用于把 OpenVINS 位姿和规划路径转换为 D1 机器人可执行的 `UserCommand` 控制指令。

当前版本中，规划输入先使用 **fake 占位路径**，后续你只需要替换一个节点即可接入真实 EgoPlanner 的 bspline 话题。

---

## 一、整体数据流（先看这个）

1. `OpenVINS` 发布位姿：`/ov_msckf/poseimu`
2. `fake_bspline_to_path_node` 读取占位路径：`/d1_demo/fake_bspline_path`
3. `path_tracker_node` 进行路径跟踪，发布：`/d1_demo/cmd_vel_raw`
4. `d1_user_command_bridge_node` 把 `Twist` 转成 `ddt_msgs/UserCommand`
5. 最终发布到 D1：`/d15019979/command/user_command`

---

## 二、文件结构与每个文件的作用

```text
d1_nav_demo/
  config/
    params.yaml
  d1_nav_demo/
    __init__.py
    fake_bspline_to_path_node.py
    path_tracker_node.py
    d1_user_command_bridge_node.py
  launch/
    demo_pipeline.launch.py
  resource/
    d1_nav_demo
  package.xml
  setup.py
  setup.cfg
  README.md
```

### 1) `package.xml`
- ROS2 包元信息与依赖声明文件。
- 关键依赖：
  - `rclpy`：Python ROS2 节点运行库
  - `geometry_msgs` / `nav_msgs` / `std_msgs`
  - `ddt_msgs`：D1 的 `UserCommand` 消息定义
  - `launch` / `launch_ros`：启动文件支持

### 2) `setup.py`
- Python 包安装与入口声明文件。
- `entry_points` 里注册了 3 个可执行节点：
  - `fake_bspline_to_path_node`
  - `path_tracker_node`
  - `d1_user_command_bridge_node`
- 没有这个文件或入口配置错误时，`ros2 run`/`ros2 launch` 会找不到节点。

### 3) `setup.cfg`
- 指定脚本安装目录给 ROS2 使用。
- 这是 `ament_python` 包的常见标准配置，避免脚本安装路径不一致问题。

### 4) `resource/d1_nav_demo`
- ROS2 `ament` 索引识别文件（标记这个包存在）。
- 文件内容可以为空，但文件必须存在。

### 5) `d1_nav_demo/__init__.py`
- Python 包初始化文件。
- 让 `d1_nav_demo/` 目录被 Python 当作模块导入。

### 6) `d1_nav_demo/fake_bspline_to_path_node.py`
- **占位节点（你后续会替换）**。
- 当前逻辑：
  - 订阅 `input_path_topic`（默认 `/d1_demo/fake_bspline_path`，类型 `nav_msgs/Path`）
  - 直接转发为标准路径到 `output_path_topic`（默认 `/d1_demo/path`）
- 作用：先把“路径跟踪 + D1 控制”链路跑通。
- 后续替换方法：把这个节点改成“订阅 EgoPlanner bspline 并采样为 `Path`”。

### 7) `d1_nav_demo/path_tracker_node.py`
- 路径跟踪控制核心节点。
- 输入：
  - `pose_topic`（默认 `/ov_msckf/poseimu`，`PoseWithCovarianceStamped`）
  - `path_topic`（默认 `/d1_demo/path`，`nav_msgs/Path`）
- 输出：
  - `cmd_vel_topic`（默认 `/d1_demo/cmd_vel_raw`，`geometry_msgs/Twist`）
- 控制特点：
  - 只使用 `linear.x` 和 `angular.z`
  - 使用前视点 + 航向误差控制
  - 位姿超时自动停车（安全保护）
  - 到达目标半径后自动停车

### 8) `d1_nav_demo/d1_user_command_bridge_node.py`
- 控制桥接节点：`Twist -> ddt_msgs/UserCommand`。
- 输入：`/d1_demo/cmd_vel_raw`
- 输出：`/<robot_ns>/command/user_command`（默认 `robot_ns=d15019979`）
- 关键行为：
  - `out.fsm_mode` 可配（默认 `loco`）
  - 仅映射 `twist.linear.x` 与 `twist.angular.z`
  - 其余速度分量强制置 0，符合你“仅 x + z 控制”的需求

### 9) `launch/demo_pipeline.launch.py`
- 一键启动 3 个节点：
  - `fake_bspline_to_path_node`
  - `path_tracker_node`
  - `d1_user_command_bridge_node`
- 所有参数统一从 `config/params.yaml` 加载。

### 10) `config/params.yaml`
- 集中参数配置文件（推荐只改这个，不改代码）。
- 主要包含：
  - 话题名映射
  - 控制参数（`lookahead_m`、`max_linear_x`、`max_angular_z` 等）
  - D1 命名空间和 `fsm_mode`

### 11) `README.md`
- 本说明文档。
- 记录工程结构、运行步骤、后续替换点。

---

## 三、如何构建与运行

```bash
cd /mnt/home/hywork/d1_nav_demo_ros2_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch d1_nav_demo demo_pipeline.launch.py
```

---

## 四、你后续必须做的替换（最重要）

当前 `fake_bspline_to_path_node.py` 只是占位。  
你拿到 EgoPlanner 的真实 bspline 话题与消息类型后，需要：

1. 在该节点里改成订阅真实 bspline 消息
2. 对 bspline 进行采样（离散出路径点）
3. 继续发布 `nav_msgs/Path` 到 `/d1_demo/path`

这样其余节点都不需要改，整条链路可直接复用。
