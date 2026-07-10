# 底盘驱动逻辑总结 & 仿真待补充项

## 一、当前驱动逻辑（已实现）

### 1.1 整体架构

```
ros2 topic /cmd_vel (Twist)
  → ros_gz_bridge (ROS_TO_GZ)
    → Gazebo native gz-sim-diff-drive-system plugin
      → left_track_joint + right_track_joint
        → left_virtual_drive_wheel_link + right_virtual_drive_wheel_link
```

**核心：使用 Gazebo 原生 DiffDrive 插件，不经过 ros2_control。**

### 1.2 驱动插件配置

文件：`urdf/gazebo_plugins.xacro`

```xml
<plugin filename="gz-sim-diff-drive-system" name="gz::sim::systems::DiffDrive">
  <left_joint>left_track_joint</left_joint>
  <right_joint>right_track_joint</right_joint>
  <wheel_separation>0.23</wheel_separation>
  <wheel_radius>0.04</wheel_radius>
  <odom_publish_frequency>50</odom_publish_frequency>
  <topic>/cmd_vel_guarded</topic>
  <odom_topic>/odom</odom_topic>
  <tf_topic>/tf</tf_topic>
  <frame_id>odom</frame_id>
  <child_frame_id>base_footprint</child_frame_id>
</plugin>
```

该插件**自行计算并发布** `/odom` 和 `odom → base_footprint` TF，不依赖 ros2_control。
ROS 侧第一版只桥接 `/odom`，再由 `odom_tf_broadcaster` 发布单一
`odom → base_footprint` TF。

### 1.3 关节结构

| 关节名 | 类型 | 父 Link | 子 Link | 作用 |
|--------|------|---------|---------|------|
| `base_footprint_to_chassis_joint` | fixed | base_footprint | chassis_link | 底盘抬升至中心高度 |
| `left_track_joint` | continuous | chassis_link | left_virtual_drive_wheel_link | 左侧驱动 |
| `right_track_joint` | continuous | chassis_link | right_virtual_drive_wheel_link | 右侧驱动 |
| `left_track_fixed_joint` | fixed | chassis_link | left_track_link | 左履带外观/碰撞 |
| `right_track_fixed_joint` | fixed | chassis_link | right_track_link | 右履带外观/碰撞 |
| `lidar_joint` | fixed | chassis_link | lidar_link | 激光雷达 |
| `d435_joint` | fixed | chassis_link | d435_link | RGB-D 相机 |
| `arm_joint_1~5` | revolute | 机械臂链 | 5-DOF 机械臂 |

### 1.4 传感器

| 传感器 | Gazebo 类型 | ROS Topic | 状态 |
|--------|------------|-----------|------|
| N10P 激光雷达 | gpu_lidar | `/scan` | 已实现 |
| D435 RGB | camera | `/camera/color/image_raw` | 已实现 |
| D435 Depth | depth_camera | `/camera/depth/image_raw` | 已实现 |
| IMU | 无 | 无 | **未实现** |

### 1.5 ros_gz_bridge

文件：`config/ros_gz_bridge.yaml`

桥接的 topic：
- `/clock` GZ→ROS
- `/cmd_vel` ROS→GZ
- `/cmd_vel_guarded` ROS→GZ
- `/odom` GZ→ROS
- `/scan` GZ→ROS
- `/camera/color/image_raw` GZ→ROS
- `/camera/color/camera_info` GZ→ROS
- `/camera/depth/image_raw` GZ→ROS
- `/camera/depth/camera_info` GZ→ROS

`/odom` bridge 已配置，待 Ubuntu/ROS 环境运行验收。`/tf` bridge 暂不启用，
避免和 `odom_tf_broadcaster` 重复发布 `odom → base_footprint`。

当前唯一 ROS TF 来源固定为：

```text
Gazebo DiffDrive -> Gazebo /odom -> ros_gz_bridge -> ROS /odom
-> odom_tf_broadcaster -> odom → base_footprint
```

### 1.6 ros2_control

文件：`config/ros2_controllers.yaml`

当前仅配置了：
- `joint_state_broadcaster` — 广播所有 joint 状态
- `arm_controller` — 机械臂 5 关节的 JointTrajectoryController

**没有** diff_drive_controller，底盘驱动不经过 ros2_control。

### 1.7 速度指令链路现状

```
ros2 topic pub /input_cmd_vel
  → scan_safety_guard_node
    → /cmd_vel_guarded
  → ros_gz_bridge
    → Gazebo DiffDrive plugin
      → 直接驱动左右 joint
```

安全守护启动已在 `sim_mapping_safety_guard.launch.py` 中配置，待运行验收。

---

## 二、当前已实现 vs 待补充总览

### 2.1 已实现

| 组件 | 文件 | 说明 |
|------|------|------|
| URDF 模型 | `urdf/*.xacro` | base_footprint, chassis, tracks, wheels, arm, sensors |
| Gazebo 原生 DiffDrive 插件 | `urdf/gazebo_plugins.xacro` | gz-sim-diff-drive-system，接收 `/cmd_vel_guarded`、控制左右 joint、内部发布 `/odom` |
| /cmd_vel_guarded ROS→GZ bridge | `config/ros_gz_bridge.yaml` | 安全守护后的速度指令传入 Gazebo，待运行验收 |
| /odom GZ→ROS bridge | `config/ros_gz_bridge.yaml` | 已配置，待运行验收 |
| odom_tf_broadcaster | `scripts/odom_tf_broadcaster.py` | 从 `/odom` 发布 `odom → base_footprint`，待运行验收 |
| 激光雷达 | `urdf/gazebo_plugins.xacro` | gpu_lidar, 360°, 10Hz → /scan |
| RGB-D 相机 | `urdf/gazebo_plugins.xacro` | D435 color + depth, 30Hz |
| ros2_control（仅机械臂） | `config/ros2_controllers.yaml` | joint_state_broadcaster + arm_controller（不涉及底盘） |
| Gazebo 启动 | `launch/gazebo.launch.py` | 启动 Gazebo + robot_state_publisher + bridge |
| RViz 启动 | `launch/display.launch.py` | 可视化 URDF 模型 |
| 空世界 | `worlds/empty_tracked_robot.sdf` | 20×20m 地面，物理/光照插件 |
| 模型尺寸文档 | `MODEL_MEASUREMENTS.md` | 所有尺寸参数记录 |

### 2.2 已配置/待补充

按优先级排列：

#### P0：SLAM 最小闭环

| # | 待补充项 | 说明 |
|---|---------|------|
| 1 | **`/odom` GZ→ROS bridge** | 已配置，待 Ubuntu/ROS 环境运行验收 |
| 2 | **`odom → base_footprint` TF** | `/tf` bridge 暂不启用；由 odom_tf_broadcaster 从 `/odom` 发布，待运行验收 |
| 3 | **`/cmd_vel_guarded` bridge** | 已配置，Gazebo 插件监听 `/cmd_vel_guarded`，待运行验收 |
| 4 | **2×2 米探索场景** | 新建 `worlds/frontier_exploration_2x2.sdf`，边界墙、隔断、箱体、蓝色物块 |
| 5 | **SLAM Toolbox 仿真集成** | launch + 参数已配置，待运行验收；第一版输入仅需 `/scan` + `/odom` + TF |

#### P1：安全 + 导航

| # | 待补充项 | 说明 |
|---|---------|------|
| 6 | **scan_safety_guard 仿真节点** | launch 已配置，复用实车 scan_safety_guard_node，待运行验收 |
| 7 | **Nav2 仿真集成** | 配置仿真用 nav2_params.yaml，启动 planner/controller/bt/recovery 节点 |
| 8 | **Frontier 探索节点** | 订阅 `/map`，发布目标点；含 fallback（预设 EXPLORE_ZONE_A/B） |

#### P2：增强（可选）

| # | 待补充项 | 说明 |
|---|---------|------|
| 9 | **IMU 传感器** | **可选增强项，不是 SLAM 硬前置**。后续需要模拟实车 EKF 或履带打滑时再添加 |
| 10 | **EKF (robot_localization)** | 仿真 odom 无噪声，第一阶段不需要；后续接入 IMU 后再启用 |

#### P3：RL 准备 + 完整集成

| # | 待补充项 | 说明 |
|---|---------|------|
| 11 | **RL 控制接口** | RL policy 节点发布 `/input_cmd_vel`（底盘）+ arm joint 目标（机械臂） |
| 12 | **机械臂 MoveIt2 仿真集成** | PlanningScene 安全禁区 + 机械臂运动规划 |
| 13 | **RL 训练环境包装** | Gym/Gymnasium 环境，订阅 sensor topics，发布 action，计算 reward |
| 14 | **Gazebo → RL 状态反馈** | 确保 RL 节点能获取 `/odom`, `/scan`, `/joint_states`, camera images |

#### P4：完善 + 展示

| # | 待补充项 | 说明 |
|---|---------|------|
| 15 | **CAD mesh 替换** | 用真实 CAD 导出的 STL/DAE 替换当前 primitive 几何体 |
| 16 | **多场景切换** | 空场景 / 探索场景 / 风险处置场景 的启动参数 |
| 17 | **风险物块检测仿真** | 在 Gazebo 场景中置入蓝色物块，D435 模拟检测 |
| 18 | **实车参数校准** | wheel_separation / wheel_radius 用实车测试数据校准 |

---

## 三、驱动方案：已冻结

**当前使用 Gazebo 原生 gz-sim-diff-drive-system，不再迁移到 ros2_control diff_drive_controller。**

| 对比维度 | 方案 A：Gazebo 原生 DiffDrive（当前，已冻结） | 方案 B：ros2_control diff_drive_controller（备选，当前不用） |
|----------|--------------------------------------|-------------------------------------------|
| 驱动方式 | Gazebo 插件直接控制 joint | ros2_control 通过 hardware_interface 控制 joint |
| `/odom` 来源 | 插件内部计算 | diff_drive_controller 根据 joint feedback 计算 |
| TF 发布 | 插件内部 | diff_drive_controller (`enable_odom_tf: true`) |
| 配置复杂度 | 低（已跑通） | 中（需额外 hardware_interface + controller 配置） |
| 当前状态 | **已实现** | **不启用** |

**禁止同时启用两个方案**：Gazebo DiffDrive 插件和 diff_drive_controller 会同时控制 `left_track_joint`/`right_track_joint`，造成双控制源冲突。`ros2_controllers.yaml` 仅用于机械臂，不涉及底盘驱动。

---

## 四、仿真最小闭环路径

要让仿真跑通这条链路：

```
遥控/Nav2/RL → /input_cmd_vel → safety_guard → /cmd_vel_guarded
  → ros_gz_bridge → Gazebo 底盘运动 → /odom + /scan
    → SLAM Toolbox → /map + map→odom TF
      → Nav2 Planner → /input_cmd_vel
```

**P0 最少需要补齐**：

1. `/odom` GZ→ROS bridge：已配置，待运行验收
2. `odom → base_footprint` TF：由 odom_tf_broadcaster 发布，待运行验收
3. `/cmd_vel_guarded` bridge 到 Gazebo 插件输入：已配置，待运行验收
4. 2×2 米探索场景 world 文件
5. SLAM Toolbox 仿真 launch：已配置，待运行验收（输入仅需 `/scan` + `/odom` + TF）

IMU 不是 SLAM 硬前置，第一阶段不接 IMU、不接 EKF 即可完成二维建图。
