# 底盘里程计与 Nav2 自主探索参考文档

本文档总结底盘 URDF 建模、odom 坐标系、差速驱动、Nav2/Frontier 探索的完整技术方案，供软件组开发和现场演示参考。

---

## 1. 坐标系与 TF 树

### 1.1 坐标系层级

```
map
└── odom
    └── base_footprint
        └── base_link (chassis_link)
            ├── left_virtual_drive_wheel_link
            ├── right_virtual_drive_wheel_link
            ├── left_track_link (视觉)
            ├── right_track_link (视觉)
            └── laser_link
```

| 坐标系 | 说明 | 发布者 |
|--------|------|--------|
| `map` | 全局地图坐标系，原点固定 | SLAM Toolbox |
| `odom` | 局部里程计坐标系，相对起点固定 | Gazebo DiffDrive 插件 (通过 ros_gz_bridge) |
| `base_footprint` | 底盘驱/转中心在地面的投影，Z=0 | Gazebo DiffDrive 插件 (通过 ros_gz_bridge) |
| `base_link` | 底盘主体几何中心/质心高度 | 固定 joint |
| `laser_link` | N10P 激光雷达安装位置 | 固定 joint |

### 1.2 关键概念

**不要将 odom 设为底盘驱动中心**。odom 是相对世界起点固定的参考坐标系，底盘的驱动与旋转中心是 `base_footprint`。

- `map → odom`：由 **SLAM Toolbox** 发布（定位修正）
- `odom → base_footprint`：由 **Gazebo DiffDrive 插件** 发布，通过 ros_gz_bridge 桥接到 ROS（或通过 odom_tf_broadcaster 从 /odom 读取后发布）

### 1.3 base_footprint 位置

- X/Y：左右履带中心线中点
- Z：地面（Z=0）
- 作用：差速运动学中心、Nav2 机器人基准、底盘原地旋转中心

### 1.4 当前实现

当前项目中，`chassis.xacro` 已实现：

```xml
<link name="base_footprint"/>

<joint name="base_footprint_to_chassis_joint" type="fixed">
  <parent link="base_footprint"/>
  <child link="chassis_link"/>
  <origin xyz="0 0 ${chassis_center_z}" rpy="0 0 0"/>
</joint>
```

其中 `chassis_center_z = chassis_top_z - chassis_height / 2.0`，即底盘主体中心距地面高度。

---

## 2. 底盘驱动模型：两轮差速

### 2.1 设计原则

实车为履带底盘，Gazebo 中使用**两个虚拟等效驱动轮**近似差速运动学：

- **外观**：左右履带 STL/视觉体（固定在 chassis_link）
- **驱动**：左右各一个隐藏虚拟驱动轮（continuous joint）
- **碰撞**：履带区域简化碰撞体

### 2.2 当前项目实现

`chassis.xacro` 已实现两轮差速结构：

```xml
<!-- 左侧虚拟驱动轮 -->
<link name="left_virtual_drive_wheel_link">
  <collision>  <!-- 隐藏碰撞体，无视觉 -->
    <cylinder radius="0.04" length="0.04"/>  <!-- virtual_drive_radius = 履带高度/2 -->
  </collision>
</link>

<joint name="left_track_joint" type="continuous">
  <parent link="chassis_link"/>
  <child link="left_virtual_drive_wheel_link"/>
  <origin xyz="0 0.115 0" rpy="0 0 0"/>  <!-- track_y = +/-0.115 -->
  <axis xyz="0 1 0"/>
  <limit effort="40.0" velocity="30.0"/>
  <dynamics damping="0.2" friction="0.1"/>
</joint>
```

右侧对称，joint name 为 `right_track_joint`。

### 2.3 关键参数

| 参数 | 当前值 | 说明 |
|------|--------|------|
| `wheel_separation` | 0.23 m | 左右履带等效驱动中心距离（不是车体总宽） |
| `virtual_drive_radius` | 0.04 m | 等效轮半径 = 履带高度(0.08) / 2 |
| `track_center_distance` | 0.23 m | 左右履带中心距 |
| `chassis_length` | 0.27 m | 底盘长 |
| `chassis_width` | 0.27 m | 底盘宽 |

**轮距（wheel_separation）不准确会导致原地旋转和转弯出现系统误差**。建议实车行驶校准后通过 `wheel_radius_multiplier` 修正。

---

## 3. 底盘驱动：Gazebo 原生 DiffDrive 插件（当前方案）

当前仿真采用 **Gazebo 原生 gz-sim-diff-drive-system 插件**，已在 `urdf/gazebo_plugins.xacro` 中配置，负责接收 `/cmd_vel`、控制左右 joint、计算并发布 `/odom` 和 `odom → base_footprint` TF。

ros2_control diff_drive_controller **当前阶段不迁移**，避免 Gazebo 原生插件和 diff_drive_controller 同时控制 `left_track_joint`/`right_track_joint` 导致双控制源冲突。`ros2_controllers.yaml` 仅用于机械臂。

---

### 3.1 可选替代方案：ros2_control diff_drive_controller

以下配置仅在后续需要统一 ros2_control 管理底盘驱动时参考，**当前不启用**：

```yaml
diff_drive_controller:
  ros__parameters:
    left_wheel_names:
      - left_track_joint

    right_wheel_names:
      - right_track_joint

    wheels_per_side: 1

    wheel_separation: 0.23
    wheel_radius: 0.04

    odom_frame_id: odom
    base_frame_id: base_footprint
    enable_odom_tf: true

    open_loop: false          # 由仿真轮实际反馈计算 odom
    position_feedback: true

    publish_rate: 30.0
    cmd_vel_timeout: 0.5      # 命令超时自动停车
    use_stamped_vel: false
```

### 3.2 关键参数说明

| 参数 | 说明 |
|------|------|
| `open_loop: false` | 根据仿真轮实际转动的 joint state 反馈计算里程计，而非根据命令假算 |
| `enable_odom_tf: true` | 自动发布 `odom → base_footprint` TF |
| `cmd_vel_timeout` | 超过指定秒数未收到新命令，自动归零停车 |
| `wheels_per_side: 1` | 每侧只有 1 个驱动关节（当前设计） |

### 3.3 右轮反转问题

如果右轮在 Gazebo 中出现反向旋转：
1. 调整关节 axis：`<axis xyz="0 -1 0"/>`
2. 或在控制器/关节坐标系中统一方向
3. **不要同时修改多个地方**，避免方向混乱

---

## 4. 速度指令链路

### 4.1 完整链路

```
遥控 / Nav2 / RL
        ↓
/input_cmd_vel
        ↓
scan_safety_guard          ← /scan 输入
        ↓
/cmd_vel_guarded
        ↓
ros_gz_bridge (ROS_TO_GZ)  ← 插件直接监听 /cmd_vel_guarded
        ↓
Gazebo 原生 DiffDrive 插件
        ↓
left_track_joint + right_track_joint
        ↓
/odom (Gazebo → ros_gz_bridge → ROS)
        ↓
odom_tf_broadcaster
        ↓
odom → base_footprint TF
```

### 4.2 设计要点

- **安全门禁统一位置**：无论控制源是遥控、Nav2 还是 RL，都经过 `scan_safety_guard`
- **切换控制源不改底盘模型**：只改变 `/input_cmd_vel` 的发布者
- 当前项目 `ros2_controllers.yaml` 仅配置了 `joint_state_broadcaster` 和 `arm_controller`（用于机械臂），底盘驱动由 Gazebo 原生 DiffDrive 插件负责，两者不冲突

---

## 5. URDF/Xacro 最小模型清单

### 5.1 必选链路

```
[ ] base_footprint              ← 驱动中心地面投影
[ ] chassis_link (base_link)    ← 底盘主体
[ ] left_virtual_drive_wheel_link   ← 左侧虚拟驱动轮
[ ] right_virtual_drive_wheel_link  ← 右侧虚拟驱动轮
[ ] left_track_joint            ← 左侧连续关节
[ ] right_track_joint           ← 右侧连续关节
[ ] laser_link                  ← N10P 激光雷达
[ ] 左右履带 visual             ← 外观固定在 chassis_link
[ ] 简化车体 collision           ← 碰撞检测用
```

### 5.2 可选链路

```
[ ] 主动轮/从动轮 visual（已在 chassis.xacro 实现为 visible_wheel）
[ ] 承重轮 visual（已在 chassis.xacro 实现）
[ ] 减震支柱 visual（已在 chassis.xacro 实现）
[ ] D435 相机 link（已在 sensors.xacro 实现）
[ ] 五舵机机械臂（已在 arm_5dof.xacro 实现）
```

### 5.3 控制器与接口

```
[✓] Gazebo 原生 DiffDrive 插件 (gz-sim-diff-drive-system，待运行复验)
[✓] /odom bridge 已配置 (ros_gz_bridge，待运行验收)
[ ] /tf bridge 暂不启用，避免重复发布 odom→base_footprint
[✓] odom_tf_broadcaster 已实现 (使用 /odom.header.stamp，待运行验收)
[ ] odom → base_footprint TF 待运行验收
[✓] base_footprint → chassis_link TF (URDF fixed joint + robot_state_publisher)
[✓] chassis_link → laser_link TF (URDF fixed joint + robot_state_publisher)
[✓] ros2_control 机械臂控制器 (arm_controller)
```

---

## 6. Nav2 / Frontier 自主探索

### 6.1 探索流程

```
未知环境起点
  → SLAM 根据雷达/里程计增量生成 occupancy grid
  → 提取 frontier（已知空闲区与未知区的边界）
  → 聚类并打分 → 选择最优探索目标
  → Nav2 Planner Server（调用 A* 类全局规划插件）生成路径
  → Nav2 Controller Server 跟踪路径并局部避障
  → safety_guard 执行最终速度限制和紧急停车
  → 地图更新后重复
```

说明：Nav2 是完整导航框架，不是与 A*、RRT 并列的单一算法。当前底盘阶段不需要自己额外写 RRT，优先使用 Nav2 自带的规划器和控制器。

### 6.2 风险事件集成

```
D435 RGB-D → YOLOv8n 本地推理 → 风险事件
风险事件 + 深度 + 里程计 → 地图风险点
地图风险点 → MoveIt + RL 规划 → 机械臂处置候选 + 人工处置任务
Nav2 路径结果 + MoveIt/RL 规划结果 + 结构化风险点 → 本地 LLM 风险报告
```

---

## 7. 2×2 米 Gazebo 探索场景

### 7.1 场景规格

| 项目 | 规格 |
|------|------|
| 场地外边界 | 2.0 m × 2.0 m |
| 实际可移动区域 | 约 1.7 m × 1.7 m |
| 通道宽度 | 0.65 ~ 0.75 m |
| 静态箱体尺寸 | 0.20 ~ 0.25 m |
| 短隔断长度 | 0.60 ~ 0.80 m |
| 小车起点 | 靠近一个角落 |

### 7.2 建议布局

```
┌──────────────────┐
│        区域 B     │
│       ┌─────┐     │
│       │障碍 │     │
│       └─────┘     │
│   ┌──────          │
│   │ 隔断    区域 C │
│   │                │
│ 起点 A             │
└──────────────────┘
```

建议元素：四面边界墙、一个短隔断、一个静态箱体、一个蓝色堵塞物、两到三个可被逐步发现的小区域。

### 7.3 2×2 米特殊考虑

地图较小，可能原地旋转后大部分空间已被雷达扫描到，Frontier 数量很少。

**应对策略**：

1. **演示模式**：限制雷达有效探测距离或设置遮挡墙，让小车必须移动后才能看到后方区域
2. **稳定保底模式**：预设 `EXPLORE_ZONE_A` 和 `EXPLORE_ZONE_B` 两个候选区域；Frontier 正常时自动选择，Frontier 提取失败时使用未访问区域作为 fallback

### 7.4 现场演示流程（三步）

| 阶段 | 动作 | 预期效果 |
|------|------|----------|
| 1 | 小车原地缓慢旋转，N10P 扫描周围 | 生成初始局部地图 |
| 2 | Frontier 选择隔断后的未知区域，Nav2 前往 | 地图扩张 |
| 3 | 发现蓝色物块或风险标识，风险点落到地图 | 小车停稳 |

最终展示目标：初始未知 → 激光扫描建图 → 选择前沿目标 → 自动移动 → 地图范围扩大 → 风险点被记录。

---

## 8. 安全禁区处理

### 8.1 不放在 SolidWorks/URDF 中

人为设置的安全禁区**不应让机械同学建模导出**，推荐按场景分级处理：

| 场景 | 方案 |
|------|------|
| 只用于机械臂路径规划 | MoveIt 2 PlanningScene 添加虚拟碰撞体 |
| 需要在 Gazebo 中可见/碰撞 | 添加 Gazebo box 模型 |
| 只用于 RL 奖励和越界判断 | 直接在 RL 环境中按坐标判断 |

### 8.2 MoveIt PlanningScene 示例

```python
# 车体上方禁入区
collision_object.id = "chassis_keepout"
collision_object.header.frame_id = "base_link"

box.dimensions = [0.40, 0.30, 0.15]
pose.position.x = 0.0
pose.position.y = 0.0
pose.position.z = 0.08
```

优点：可修改尺寸/位置、可临时启/禁用、可根据底盘姿态动态更新、不影响真实物理模型。

### 8.3 RL 奖励判断示例

```python
if tool_z < minimum_safe_height:
    reward -= 10
    terminated = True

if point_inside_keepout(end_effector_position):
    reward -= 20
```

### 8.4 机械组交付物

机械组只需提供：
- 车体、相机、机械臂安装位置
- 真实结构外形和尺寸
- 各关节轴及限位
- 哪些区域在机械上建议禁止进入

软件侧根据这些尺寸建立虚拟安全区。

---

## 9. 验收标准

### 9.1 底盘单独验收（不接 SLAM/Nav2）

| # | 测试项 | 验收条件 |
|---|--------|----------|
| 1 | 正 linear.x | 小车直线前进 |
| 2 | 负 linear.x | 小车后退 |
| 3 | 正 angular.z | 小车原地左转 |
| 4 | 负 angular.z | 小车原地右转 |
| 5 | 超时停车 | 停止发送后 0.5 秒内归零 |
| 6 | /odom 话题 | 连续更新 |
| 7 | odom → base_footprint TF | 连续更新 |
| 8 | 里程计精度 | 原地转一圈后位置漂移不过大 |
| 9 | /scan 数据 | 墙体和障碍物方向正确 |
| 10 | safety_guard | 能拦截危险速度命令 |

### 9.2 与实车对应的验证

- 实际走 1 m，里程计显示值校准
- 通过 `wheel_radius_multiplier` 或调整 `wheel_radius` 修正
- 里程计漂移应能由 SLAM Toolbox 的 `map → odom` 修正补偿

---

## 10. 当前项目状态

| 组件 | 状态 |
|------|------|
| base_footprint | 已实现 |
| chassis_link (base_link) | 已实现 |
| 左右虚拟驱动轮 + continuous joint | 已实现 (`left_track_joint`, `right_track_joint`) |
| 左右履带 visual | 已实现 |
| 承重轮/减震支柱 visual | 已实现 |
| laser_link + N10P | 已实现 (sensors.xacro) |
| D435 RGB-D | 已实现 (sensors.xacro) |
| 五舵机机械臂 | 已实现 (arm_5dof.xacro) |
| Gazebo 原生 DiffDrive 插件 | **已实现，待运行复验**（gz-sim-diff-drive-system，gazebo_plugins.xacro） |
| /odom 话题 (ROS 侧) | **bridge 已配置，待运行验收** |
| /tf bridge | **暂不启用**（避免和 odom_tf_broadcaster 重复发布） |
| odom_tf_broadcaster | **已实现，待运行验收**（使用 `/odom.header.stamp`） |
| odom → base_footprint TF (ROS 侧) | **待运行验收** |
| scan_safety_guard | **仿真启动已配置，待运行验收** |
| 2×2 米探索场景 | **待创建** |
| Nav2 集成 | 实车已实现，仿真待联调 |
