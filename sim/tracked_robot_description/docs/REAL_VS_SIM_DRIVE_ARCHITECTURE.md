# 实车驱动逻辑总结 & 仿真待补充

## 一、实车遥控建图完整链路

### 1.1 直接遥控建图（无安全守护）

文件：`n10p_tank_mapping.launch.py`

```
┌─────────────────────────────────────────────────────────────────────┐
│ 键盘 (teleop_twist_keyboard)                                        │
│   │                                                                 │
│   │  geometry_msgs/Twist                                            │
│   │  topic: /cmd_vel                                                │
│   ▼                                                                 │
│  wheeltec_tank_base_safe.py  (Python, 50Hz)                        │
│   │  cmd_callback(): clamp + start/stop kick                       │
│   │  control_tick(): watchdog(0.25s超时归零) → 刹车脉冲 → 组帧     │
│   │                                                                 │
│   │  11-byte serial frame @ 115200 baud → /dev/base_controller     │
│   ▼                                                                 │
│  C30D 底盘主控                                                        │
│   │  解析帧 → PWM → 电机                                             │
│   │                                                                 │
│   │  24-byte serial frame ← 编码器速度 + IMU + 电池                  │
│   ▼                                                                 │
│  wheeltec_tank_base_safe.py  handle_frame()                         │
│   │  发布: /odom (里程计积分)                                        │
│   │  发布: odom → base_footprint TF                                 │
│   │  发布: /imu/data_raw (加速度+陀螺仪)                             │
│   ▼                                                                 │
│  EKF (robot_localization)                                           │
│   │  融合 /odom + /imu/data_raw → /odom_combined                    │
│   ▼                                                                 │
│  SLAM Toolbox                                                       │
│     订阅: /scan + /odom_combined                                    │
│     发布: /map + map → odom TF                                      │
└─────────────────────────────────────────────────────────────────────┘
```

### 1.2 带安全守护的遥控建图

文件：`n10p_tank_mapping_safety_guard.launch.py`

```
键盘 → /input_cmd_vel
         │
         ▼
scan_safety_guard_node  ← /scan
  ├─ clear: 透传
  ├─ warning: 前向限速 0.30 m/s（障碍物 < 1.60m）
  ├─ hard_stop: 强制归零 + latch 1.5s（障碍物 < 1.00m 或 紧急 < 0.45m）
  └─ stale_scan: 无扫描数据 > 0.6s → 归零
         │
         ▼ /cmd_vel_guarded
wheeltec_tank_base → serial → C30D → 电机
         │
         ▼
/odom + TF + /imu → EKF → SLAM
```

### 1.3 完整 Nav2 自主导航（带安全守护）

文件：`n10p_tank_nav2_guarded.launch.py`

```
Nav2 controller_server → /cmd_vel_nav
  → velocity_smoother → /cmd_vel_raw
    → scan_safety_guard_node ← /scan
      → /cmd_vel_guarded
        → wheeltec_tank_base → serial → C30D → 电机
```

### 1.4 关键参数（实车）

| 参数 | 值 | 说明 |
|------|-----|------|
| 控制频率 | 50 Hz | send_rate |
| 命令超时 | 0.25 s | cmd_timeout, 超时自动归零 |
| 最大线速度 | 0.45 m/s | max_linear |
| 最大角速度 | 0.80 rad/s | max_angular |
| 安全硬停距离 | 1.00 m | hard_stop_m |
| 安全减速距离 | 1.60 m | slow_down_m |
| 紧急停止距离 | 0.45 m | emergency_stop_m |
| 安全前向扇区 | ±35° | front_sector_deg |
| 硬停锁存 | 1.50 s | hard_stop_latch_s |
| 静态 TF (laser) | x=0.12, z=0.12 | base_footprint → laser |

---

## 二、仿真当前驱动链路

### 2.1 仿真现有架构

文件：`sim/tracked_robot_description/`

```
ros2 topic pub /input_cmd_vel  (或键盘 remap 到 /input_cmd_vel)
  │
  ▼
scan_safety_guard_node
  │
  ▼ /cmd_vel_guarded
ros_gz_bridge (ROS_TO_GZ)
  │
  ▼
Gazebo gz-sim-diff-drive-system 插件
  │  直接控制 left_track_joint + right_track_joint
  │  内部计算 /odom (只在 Gazebo 内部)
  │  内部发布 odom → base_footprint TF (只在 Gazebo 内部)
  ▼
Gazebo 物理引擎 → 关节转动 → 底盘运动

传感器 (Gazebo 内部):
  gpu_lidar    → /scan          (bridge GZ_TO_ROS ✓)
  camera       → /camera/color  (bridge GZ_TO_ROS ✓)
  depth_camera → /camera/depth  (bridge GZ_TO_ROS ✓)
```

### 2.2 仿真已有的 bridge

`config/ros_gz_bridge.yaml`:

| 方向 | ROS Topic | 状态 |
|------|-----------|------|
| GZ→ROS | `/clock` | ✓ |
| ROS→GZ | `/cmd_vel` | ✓ |
| ROS→GZ | `/cmd_vel_guarded` | 已配置，待运行验收 |
| GZ→ROS | `/scan` | ✓ |
| GZ→ROS | `/camera/color/image_raw`, `/camera_info` | ✓ |
| GZ→ROS | `/camera/depth/image_raw`, `/camera_info` | ✓ |
| GZ→ROS | `/odom` | 已配置，待运行验收 |
| GZ→ROS | `/tf` | 暂不启用，避免重复发布 |
| GZ→ROS | `/imu` | **缺失（无传感器）** |

---

## 三、仿真 vs 实车对比

| 组件 | 实车 | 仿真当前 | 差距 |
|------|------|----------|------|
| 底盘驱动 | Python → serial → C30D | Gazebo DiffDrive 插件 | 架构不同，但功能等效 |
| `/cmd_vel` 接收 | wheeltec_tank_base | Gazebo DiffDrive 插件 | 功能等效 |
| `/odom` 来源 | 编码器积分 (MCU→serial→ROS) | Gazebo 插件内部计算，bridge 已配置 | **待运行验收** |
| `odom→base_footprint` TF | wheeltec_tank_base 发布 | odom_tf_broadcaster 从 `/odom` 发布 | **待运行验收** |
| `/imu` | C30D 串口反馈 | 无 | **可选增强**（非 SLAM 硬前置，后续接 EKF 时再添加） |
| `/joint_states` | wheeltec_tank_base 不发布 | ros2_control joint_state_broadcaster | **需 bridge 或 ros2_control** |
| LiDAR `/scan` | lslidar_driver (真实雷达) | gpu_lidar 插件 | ✓ 已有 |
| D435 camera | RealSense 驱动 | camera + depth_camera 插件 | ✓ 已有 |
| scan_safety_guard | ✓ 在 mapping_safety_guard 中启动 | launch 已配置 | **待运行验收** |
| SLAM Toolbox | ✓ | launch + 参数已配置 | **待运行验收** |
| Nav2 | ✓ | ✗ | **需添加 launch + 参数** |
| EKF | ✓ 融合 odom+IMU | ✗ | 第一阶段不需要（仿真 odom 无噪声） |
| chassis_security_keepalive | ✓ 告知 C30D 受控 | ✗ | **仿真不需要**（无 C30D） |
| 世界 | 真实环境 | 20×20 空地 | **需建 2×2 探索场景** |
| 刹车脉冲 | stop_kick 逻辑 | 无 | **仿真不需要**（无惯性问题） |

---

## 四、仿真需要补充的内容

### P0：让 ROS 拿到里程计（SLAM 最小闭环的前提）

#### 1. 桥接 `/odom` 到 ROS

Gazebo DiffDrive 插件已在 Gazebo Transport 中发布 `/odom`，只需通过 `ros_gz_bridge` 转到 ROS：

```yaml
- ros_topic_name: "/odom"
  gz_topic_name: "/odom"
  ros_type_name: "nav_msgs/msg/Odometry"
  gz_type_name: "gz.msgs.Odometry"
  direction: GZ_TO_ROS
```

#### 2. 处理 `odom → base_footprint` TF

当前第一版不启用 `/tf` bridge，避免和手写 broadcaster 重复发布。TF 来源固定为：

```
只桥接 /odom
→ odom_tf_broadcaster
→ 从 nav_msgs/Odometry 读取 pose 和 header.stamp
→ 发布 odom → base_footprint
```

必须验收 `ros2 run tf2_ros tf2_echo odom base_footprint`，并用
`ros2 topic info /tf -v` 确认没有第二个同名 TF 发布者。

### P1：安全守护 + SLAM + Nav2

#### 3. 重映射 cmd_vel_guarded → Gazebo 插件

Gazebo DiffDrive 插件的输入已改为 `/cmd_vel_guarded`，并在
`ros_gz_bridge.yaml` 中配置 ROS→GZ bridge，待运行验收。

#### 4. 启动 scan_safety_guard

复用实车的 `scan_safety_guard_node`：

```python
Node(
    package="k1_sensor_event_adapter",
    executable="scan_safety_guard_node",
    name="scan_safety_guard_node",
    parameters=[{
        "scan_topic": "/scan",
        "input_cmd_topic": "/input_cmd_vel",
        "output_cmd_topic": "/cmd_vel_guarded",
    }],
)
```

#### 5. 启动 SLAM Toolbox

复用实车配置 `slam_toolbox_n10p_tank.yaml`：

```python
Node(
    package="slam_toolbox",
    executable="async_slam_toolbox_node",
    parameters=[slam_params, {"use_sim_time": True}],
)
```

第一版输入仅需：`/scan` + `/odom` + TF。不接 IMU、不接 EKF。

#### 6. 启动 Nav2

复用实车的 `nav2_n10p_tank_guarded.yaml`，适配仿真坐标系和 topic。

### P2：场景 + 集成 launch

#### 7. 2×2 米探索场景

新建 `worlds/frontier_exploration_2x2.sdf`：边界墙、短隔断、静态箱体、蓝色物块。

#### 8. 仿真集成 launch 文件

- `launch/sim_mapping.launch.py` — 对标 `n10p_tank_mapping.launch.py`
- `launch/sim_mapping_safety_guard.launch.py` — 对标 `n10p_tank_mapping_safety_guard.launch.py`
- `launch/sim_nav2.launch.py` — 对标 `n10p_tank_nav2_guarded.launch.py`

### P3：可选增强

| 组件 | 说明 |
|------|------|
| IMU 传感器 + bridge | **非 SLAM 硬前置**。后续需要模拟实车 EKF 或履带打滑时再添加 |
| EKF (robot_localization) | 仿真 odom 无噪声，第一阶段不需要 |

### 不需要补充的

| 组件 | 原因 |
|------|------|
| wheeltec_tank_base_safe.py | 仿真无 C30D 串口，Gazebo 插件替代 |
| chassis_security_keepalive | 仿真无 C30D |
| 刹车脉冲 stop_kick | 仿真无惯性问题 |
| 超声波 | 仿真有激光雷达已足够 |
| ros2_control diff_drive_controller | 避免与 Gazebo 原生插件双控制源冲突 |

---

## 五、仿真目标链路

补齐后，仿真的完整链路对标实车：

```
键盘 / Nav2 / RL
    │
    ▼ /input_cmd_vel
scan_safety_guard_node ← /scan (gpu_lidar → bridge → ROS)
    │
    ▼ /cmd_vel_guarded
    │
    │  ros_gz_bridge (ROS_TO_GZ)    ← 插件直接监听 /cmd_vel_guarded
    ▼
Gazebo DiffDrive 插件 → left_track_joint + right_track_joint
    │
    ▼
/odom + /scan  →  bridge (GZ_TO_ROS)  →  ROS
    │
    ▼
SLAM Toolbox → /map + map→odom TF   (第一阶段不接 IMU/EKF)
    │
    ▼
Nav2 Planner/Controller → /input_cmd_vel  (闭环)
```

**与实车的关键对齐点**：
1. 安全守护在 cmd_vel 链中处于同一位置
2. SLAM 和 Nav2 使用相同的配置文件
3. 上层（RL / 遥操 / 自主）切换 `/input_cmd_vel` 的发布源，不改底盘模型
