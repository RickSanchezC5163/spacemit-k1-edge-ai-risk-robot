# K1 Edge AI Robot — 整车通信协议与逻辑总纲

**版本**: v1.0
**日期**: 2026-06-30
**范围**: 全车 ROS 2 协议、硬件链路、已验证子系统、Gazebo 迁移条件评估

---

## 目录

1. [系统拓扑总览](#1-系统拓扑总览)
2. [ROS-C30D：底盘串行协议](#2-ros-c30d底盘串行协议)
3. [ROS-N10P：激光雷达](#3-ros-n10p激光雷达)
4. [ROS-D435：深度相机](#4-ros-d435深度相机)
5. [ROS-机械臂：总线舵机](#5-ros-机械臂总线舵机)
6. [ROS-Light：GPIO37 灯光](#6-ros-lightgpio37-灯光)
7. [安全 Guard 链路](#7-安全-guard-链路)
8. [Guarded Policy：自动建图规划](#8-guarded-policy自动建图规划)
9. [已冻结验证的稳定协议](#9-已冻结验证的稳定协议)
10. [Gazebo 迁移条件评估](#10-gazebo-迁移条件评估)
11. [缺失项与待补充](#11-缺失项与待补充)

---

## 1. 系统拓扑总览

### 1.1 硬件拓扑

```
K1 Muse Pi Pro (RISC-V / ARM 伴生计算机)
├── [USB]    C30D 底盘控制器  →  /dev/base_controller @ 115200
├── [UART]   总线舵机控制器    →  /dev/ttyS1 @ 9600
├── [USB]    N10P 激光雷达     →  lslidar_driver_node → /scan
├── [USB]    D435 深度相机     →  astra_camera → /camera/camera/color|depth/...
├── [GPIO37] 照明灯            →  sysfs PWM
└── [ETH]    远程 SSH          →  soc@192.168.43.40
```

### 1.2 ROS 2 节点拓扑

```
/lslidar_driver_node          → /scan (LaserScan)
/astra_camera                 → /camera/camera/color/image_raw
                              → /camera/camera/depth/image_rect_raw
/scan_safety_guard_node       → /cmd_vel_guarded, /safety/front_obstacle
/wheeltec_tank_base_safe      → /odom, /robot_vel, /imu
/slam_toolbox                 → /map, /map_metadata
/guarded_auto_mapping_micro   → /input_cmd_vel (policy)
/gpio37_light_node            → /light/status
/adaptive_light_controller    → /light/brightness_cmd
/risk_light_bridge_node       → /light/brightness_cmd
```

### 1.3 核心 Topic 链路

```
感知层:
  /scan (N10P) ──→ scan_safety_guard_node
  /scan (N10P) ──→ slam_toolbox
  /scan (N10P) ──→ policy (scan_sector_snapshot)
  /camera/camera/color/image_raw (D435) ──→ policy (HOLD_CAPTURE)
  /camera/camera/depth/image_rect_raw (D435) ──→ policy

安全层:
  /input_cmd_vel ──→ scan_safety_guard_node ──→ /cmd_vel_guarded
  /safety/front_obstacle ──→ policy (front_p10, front_min)
  /chassis/stop_request ──→ wheeltec_tank_base_safe

执行层:
  /cmd_vel_guarded ──→ wheeltec_tank_base_safe ──→ C30D serial

反馈层:
  C30D serial ──→ wheeltec_tank_base_safe ──→ /odom
  C30D serial ──→ wheeltec_tank_base_safe ──→ /robot_vel
  wheeltec_tank_base_safe ──→ /rosout (diag log)
```

---

## 2. ROS-C30D：底盘串行协议

### 2.1 物理层

| 参数 | 值 |
|------|-----|
| 设备路径 | `/dev/base_controller` |
| 波特率 | 115200 |
| 数据位 | 8 |
| 校验 | BCC (XOR of bytes 0..N-2) |
| 帧头 | `0x7B` |
| 帧尾 | `0x7D` |

### 2.2 TX 帧 (11 bytes)：上位机 → C30D

| Byte | 字段 | 说明 |
|------|------|------|
| 0 | `0x7B` | 帧头 |
| 1 | auto_recharge | 自动回充标志 (保留) |
| 2 | security_ply | 底盘安全使能 |
| 3-4 | vx | 线速度 X, mm/s, signed 16-bit big-endian |
| 5-6 | vy | 线速度 Y (履带底盘恒为 0) |
| 7-8 | wz | 角速度 Z, mrad/s, signed 16-bit |
| 9 | BCC | bytes 0-8 异或校验 |
| 10 | `0x7D` | 帧尾 |

### 2.3 RX 帧 (24 bytes)：C30D → 上位机

| Byte | 字段 | 说明 |
|------|------|------|
| 0 | `0x7B` | 帧头 |
| 2-3 | vx | 反馈线速度 X, mm/s |
| 4-5 | vy | 反馈线速度 Y |
| 6-7 | wz | 反馈角速度 Z, mrad/s |
| 8-13 | accel_x/y/z | IMU 加速度原始值 |
| 14-19 | gyro_x/y/z | IMU 陀螺仪原始值 |
| 20-21 | voltage | 电池电压, mV |
| 22 | BCC | bytes 0-21 异或校验 |
| 23 | `0x7D` | 帧尾 |

### 2.4 安全帧 (底盘安全使能)

- 传统模式: `byte[2] = 0x01` (使能) / `0x00` (禁用)
- 新模式: `byte[2] = 0xB1` (使能) / `0xB0` (禁用)
- 启动时两种帧依次发送

### 2.5 wheeltec_tank_base_safe.py 节点

**订阅**:
| Topic | 类型 | 说明 |
|-------|------|------|
| `/cmd_vel` (remapped → `/cmd_vel_guarded`) | Twist | 速度指令 |
| `/chassis/stop_request` | String (JSON) | 紧急停止请求 |

**发布**:
| Topic | 类型 | 说明 |
|-------|------|------|
| `/odom` | Odometry | 航位推算里程计 |
| `/imu` | Imu | IMU 数据 |
| `/battery_voltage` | Float32 | 电池电压 |
| `/robot_vel` | Vector3 | 原始反馈速度 |

**驱动特性**:
- 启动冲击 (start_kick): 从零启动时额外脉冲突破静摩擦
- 停止冲击 (stop_kick): 收到 STOP_REQUEST 时反向制动脉冲 + brake_duration 零速保持
- 巡航限速: `cruise_linear_limit=0.08 m/s`, `cruise_angular_limit=0.20 rad/s`
- 制动时长: `brake_duration=1.0s`
- 指令超时: `cmd_timeout=0.25s`
- 看门狗: 指令超时后自动发送零速

### 2.6 已验证状态

```
✓ C30D 串行协议已冻结 (0x7B/0x7D 帧格式)
✓ wheeltec_tank_base_safe.py 驱动稳定
✓ stop_kick + brake_hold 制动链验证通过
✓ base_zero_ok 三条件检测 (guarded_cmd + robot_vel + diag) 验证通过
✓ /rosout diag 解析正则已验证
✓ odom 航位推算 + TF 发布正常
```

---

## 3. ROS-N10P：激光雷达

### 3.1 物理参数

| 参数 | 值 |
|------|-----|
| 型号 | LSLiDAR N10 Plus |
| 安装位置 | 居中, 距地 13cm, 中心距底盘前边缘 6cm |
| 驱动包 | `lslidar_driver` (ROS 2) |
| Topic | `/scan` (sensor_msgs/LaserScan) |
| 角度范围 | 360° |
| 分辨率 | ~0.18° (约 2000 points/rev) |
| 最大量程 | 12m (slam_toolbox 配置) / 25m (硬件标称) |
| 最小量程 | 0.05m |
| 频率 | ~10 Hz |

### 3.2 扇区分析 (policy 内)

`scan_sector_snapshot()` 将 360° 扫描划分为 5 个扇区：

| 扇区 | 角度范围 | 宽度 | 统计量 |
|------|---------|------|--------|
| front | -15° ~ +15° | 30° | count, min, p10 |
| left | +15° ~ +75° | 60° | count, min, p10 |
| right | -75° ~ -15° | 60° | count, min, p10 |
| left45 | +30° ~ +60° | 30° | count, min, p10 |
| right45 | -60° ~ -30° | 30° | count, min, p10 |

### 3.3 SLAM 配置

| 参数 | 值 |
|------|-----|
| 包 | `slam_toolbox` |
| 模式 | async (online_async) |
| 地图分辨率 | 0.05 m |
| 扫描匹配 | 启用, 20 帧缓冲区 |
| 回环检测 | 启用, 最大搜索距离 3.0m |
| 地图更新间隔 | 2.0s |

### 3.4 已验证状态

```
✓ /scan topic 稳定发布
✓ slam_toolbox 在线建图正常
✓ scan_sector_snapshot 5 扇区统计已验证
✓ front_p10 / front_min 阈值策略已验证 (P4 系列)
✓ 扇区 count 密度可用于探索奖励 (RL 准备)
```

---

## 4. ROS-D435：深度相机

### 4.1 物理参数

| 参数 | 值 |
|------|-----|
| 型号 | Intel RealSense D435 |
| 安装位置 | 居中, 距地 11cm, 距底盘前边缘 3cm |
| 驱动包 | `astra_camera` (ROS 2) |

### 4.2 Topics

| Topic | 类型 | 分辨率 | 编码 | 频率 |
|-------|------|--------|------|------|
| `/camera/camera/color/image_raw` | Image | 640×480 | rgb8 | ~30 Hz |
| `/camera/camera/depth/image_rect_raw` | Image | 640×480 | 16UC1 | ~29 Hz |
| `/camera/camera/color/camera_info` | CameraInfo | — | — | — |
| `/camera/camera/depth/camera_info` | CameraInfo | — | — | — |

### 4.3 HOLD_CAPTURE 动作原语

**验证状态**: 10/10 succeeded (P4-X2)

动作流程:
```
收到 HOLD_CAPTURE
→ 确认 base_zero_ok
→ 读取最近 RGB 帧 → 保存 rgb.png
→ 读取最近 depth 帧 → 保存 depth_raw.npy + depth_vis.png
→ 读取 camera_info → 保存 camera_info.json
→ 读取当前 odom → 保存 odom.json
→ 生成 capture_meta.json
→ 若存在 bbox + depth → 生成 risk_point.json
→ 返回 ActionResult {status: "succeeded", base_zero_ok_before: true}
```

关键约束:
- `publishes_cmd_vel = false` (不发送任何速度指令)
- `requires_base_zero = true` (要求底盘已停稳)
- 证据文件: rgb.png + depth_raw.npy + camera_info.json + odom.json + capture_meta.json

### 4.4 已验证状态

```
✓ D435 topic 审计通过 (P4-X0)
✓ 单次 capture 验证通过 (P4-X1)
✓ HOLD_CAPTURE 连续 10/10 成功 (P4-X2)
✓ RGB/depth/camera_info/odom/meta 证据链完整
✓ depth_raw.npy 可加载 (480×640, uint16)
✓ published_cmd_vel = false 始终成立
```

---

## 5. ROS-机械臂：总线舵机

### 5.1 硬件参数

| 参数 | 值 |
|------|-----|
| 型号 | Lobot 总线舵机 ×5 |
| 通信 | UART, 9600 baud, 8N1 |
| 帧协议 | `0x55 0x55 <len> <cmd> <params...>` |
| 舵机 ID | 1=底座 yaw, 2=肩部, 3=肘部, 4=腕部, 5=夹爪 |
| 脉冲范围 | 0-1000 (bus servo mode) |
| 时间范围 | 0-30000 ms |

### 5.2 关键指令

| 指令 | 代码 | 帧格式 |
|------|------|--------|
| SERVO_MOVE (单舵机) | 3 | `55 55 08 03 01 <time_lo> <time_hi> <id> <pulse_lo> <pulse_hi>` |
| SERVO_MOVE (多舵机) | 3 | `55 55 <3n+5> 03 <n> <time> [<id> <pulse>]×n` |
| ACTION_GROUP_RUN | 6 | `55 55 05 06 <group_id> <count_lo> <count_hi>` |
| ACTION_GROUP_STOP | 7 | `55 55 02 07` |
| GET_BATTERY_VOLTAGE | 15 | 协议已定义，示例缺失 |

### 5.3 机械臂运动学

```
ID1 (yaw, z轴) → L1=19cm 竖臂 (z向)
  → ID2 (shoulder, y轴) → L2=4cm 连杆 (x向)
    → ID3 (elbow, y轴) → L3=19cm 竖臂 (z向)
      → ID4 (wrist, y轴) → L4=5.5cm 腕部 + 6cm 手指
        → ID5 (gripper, z轴开合)
```

- 臂基座: x=-0.005m, z=0.13m (相对底盘中心)
- 最大水平伸展: ~45cm
- 最大垂直伸展: ~55cm

### 5.4 安全架构 (7 层)

| 层 | 名称 | 说明 |
|----|------|------|
| L1 | 相位门 | 全局 AND 阶段门: phase 不能绕过 global gate |
| L2 | 协议验证 | ID 范围 [1,5], pulse [0,1000], time [0,30000]ms |
| L3 | 关节限位 | soft limit (警告) + hard limit (阻止) |
| L4 | 步长限制 | max_step = 300 × (time_ms/1000), 时间缩放 |
| L5 | 工作空间 | FK 计算末端位置, 检查安全区 + 禁区 |
| L6 | 机器人安全 | base_zero 必须为 true, 禁止行驶时动臂 |
| L7 | 紧急停止 | E-stop + heartbeat 超时 → 自动回 home |

### 5.5 安全门关键修复

```python
# 修复前: phase gate 可绕过全局 gate
serial_write_allowed = phase_gate.serial_write_allowed  # BUG

# 修复后: 全局 AND 阶段
serial_write_allowed = global_gate.serial_write_allowed AND phase_gate.serial_write_allowed
```

**当前状态**:
```
global serial_write_allowed = false
global hardware_access_allowed = false
→ 任何 phase 都无法执行硬件操作
→ build_move_frame() 始终返回 frame_built_for_review_only=true
→ 0 字节实际串行数据发送
```

### 5.6 相位门序列

| 阶段 | 硬件 | 串行 | 接触 | 状态 |
|------|------|------|------|------|
| Arm-B1 | 否 | 否 | 否 | ✓ dry-run 通过 |
| Arm-B2 | 是 | 是 | 否 | 待标定 |
| Arm-B3 | 是 | 是 | 否 | 待 B2 |
| Arm-C | 是 | 是 | 否 | 待 B3 |
| Arm-D | 是 | 是 | 仅泡沫 | 待 C |
| Arm-E | 是 | 是 | 是 | 待 D |

### 5.7 已验证状态

```
✓ Lobot 总线协议审查完成 (读 SDK, 未接硬件)
✓ 安全配置文件 arm_safety_config.json 完成
✓ 安全模块 arm_safety.py 完成 (7 层安全验证)
✓ 全局 AND 阶段安全门修复验证通过
✓ Arm-B1 dry-run 计划生成验证通过 (B2:10/10, B3:9/9)
✓ 候选动作序列 arm_b_no_load_sample_v0_candidate.json 已记录
✓ safe_idle_home 姿态已记录 (ID1=510, ID2=771, ID3=426, ID4=503, ID5=497)
✗ 未执行任何硬件控制 (0 bytes 串行发送)
✗ 最终标定数据待确认 (home_pulse, soft_limit, 方向)
```

---

## 6. ROS-Light：GPIO37 灯光

### 6.1 物理参数

| 参数 | 值 |
|------|-----|
| 控制引脚 | GPIO37 |
| 驱动方式 | sysfs PWM |
| PWM 频率 | 50 Hz |
| 脉宽范围 | 1100-1900 μs |
| 亮度范围 | 0-100% (Int32 topic) |

### 6.2 ROS 2 节点

**gpio37_light_node**: 基础 PWM 控制
- 订阅 `/light/brightness_cmd` (Int32, 0-100)
- 发布 `/light/status` (Int32)
- 支持 dry_run 模式

**adaptive_light_controller_node**: 自适应亮度
- 订阅 `/camera/camera/color/image_raw`
- 计算图像 luma 均值 → 查表确定亮度
- 步进斜坡过渡 (step_limit per update)

**risk_light_bridge_node**: 风险灯光桥接
- 订阅 `/risk/recommended_action` (String)
- 收到 `turn_on_light_and_recheck` → 亮度 5%, 持续 8s

### 6.3 已验证状态

```
✓ GPIO37 PWM 驱动可用
✓ 三节点灯光系统设计完整
✓ dry_run 模式安全
✓ 与风险检测链路桥接
✗ 完整低照度场景端到端测试待做
```

---

## 7. 安全 Guard 链路

### 7.1 scan_safety_guard_node

**功能**: 拦截 `/input_cmd_vel`，基于 LiDAR 前向扇区 (35°) 检查是否安全，发布过滤后的 `/cmd_vel_guarded`。

**关键阈值** (当前运行配置):

| 参数 | 值 | 说明 |
|------|-----|------|
| `hard_stop_m` | 0.30 m | front_p10 ≤ 此值 → 零速 |
| `emergency_stop_m` | 0.20 m | front_min ≤ 此值 → 零速 |
| `slow_down_m` | 0.80 m | 低于此值 → warning 状态 |
| `soft_max_linear` | 0.30 m/s | warning 状态下最大线速度 |
| `clear_max_linear` | 0.30 m/s | clear 状态下最大线速度 |
| `approach_stop_m` | 0.80 m | 动态停止评估区 |
| `front_sector_deg` | 35.0° | 前向检测半角 |

**状态机**: `clear → warning → hard_stop`
- hard_stop 触发后锁存 1.5s
- 恢复需 front_p10 > slow_clear_m (1.75m)

### 7.2 完整防护链

```
策略层:
  max_consecutive_fast_arc ≤ 2    (防止连续绕圈)
  max_total_forward_m ≤ 1.0/1.5   (防止跑太远)
  front_block_reason()            (动作前检查)
  base_zero_ok                    (动作后验证)

Guard 层:
  scan_safety_guard_node          (激光安全屏障)
  /chassis/stop_request           (紧急停止请求)

底盘层:
  wheeltec_tank_base_safe         (stop_kick + brake_hold)
  cmd_timeout = 0.25s             (指令超时 → 零速)
  C30D 串行看门狗                 (下位机自主停止)
```

### 7.3 已验证状态

```
✓ 安全 Guard 链三层防护全部验证 (P4-Y2 压力测试)
✓ hard_stop 触发 + 锁存 + 恢复逻辑正确
✓ stop_kick + brake_hold 制动链正确
✓ max_consecutive_fast_arc 拦停生效
✓ max_total_forward_m 限制生效
✓ emergency_stop 独立于 hard_stop
```

---

## 8. Guarded Policy：自动建图规划

### 8.1 策略框架

**文件**: `tools/guarded_auto_mapping_micro.py` (约 4300 行)

**模式**: `guarded-policy-run` (多步有界自动探图)

### 8.2 状态输入 (PolicyState)

| 字段 | 来源 | 说明 |
|------|------|------|
| `front_p10` | `/safety/front_obstacle` | 前向 35° 扇区 P10 距离 |
| `front_min` | `/safety/front_obstacle` | 前向扇区最近点 |
| `left_p10` | scan_sector_snapshot | 左侧 60° 扇区 P10 |
| `right_p10` | scan_sector_snapshot | 右侧 60° 扇区 P10 |
| `odom` | `/odom` | 里程计位姿 (x, y, yaw) |
| `base_zero_ok` | zero_status_snapshot | 底盘三条件零速确认 |
| `map_metadata` | `/map_metadata` | 地图尺寸/分辨率 |
| `last_action` | 内部状态 | 上一步动作 |
| `consecutive_fast_arc` | 内部计数 | 连续 fast arc 次数 |

### 8.3 动作空间 (离散原语)

| 动作 | 参数 | 说明 |
|------|------|------|
| `FORWARD_0P05` | 0.05m | 极近距微动 |
| `FORWARD_0P10` | 0.10m | 中距前进 |
| `FORWARD_0P15` | 0.15m | 开阔前进 |
| `ARC_FAST_LEFT` | ω=+0.80 rad/s, t=1.0s | 快速左弧 (~25°) |
| `ARC_FAST_RIGHT` | ω=-0.80 rad/s, t=1.0s | 快速右弧 (~28°) |
| `HOLD_CAPTURE` | — | 停稳拍照保存 |
| `STOP_SAFE` | — | 安全终止 |

### 8.4 决策逻辑 (select_policy_action)

**interaction_mode 阈值** (当前配置):

| front_p10 范围 | 选定动作 |
|---------------|---------|
| < 0.20m (front_min) | HARD_STOP |
| < 0.30m | HOLD_AND_CAPTURE |
| 0.30-0.40m | HOLD_SAVE_OBSERVE |
| 0.40-0.60m | ARC30_OR_FORWARD_0P05 |
| 0.60-0.80m | ARC30_OR_FORWARD_0P10 |
| ≥ 0.80m | FORWARD_0P15_OR_ARC30 |

### 8.5 已验证的多步运行

| 测试 | 步数 | 停止原因 | 结果 |
|------|------|---------|------|
| P4-W Speed | 2 (bounded) | target_overshot | base_zero_ok=true, map saved |
| P4-Y2 | 3 (7-step stress) | max_consecutive_fast_arc_reached | 保险生效, 累计前进 0.124m |

### 8.6 已验证状态

```
✓ FORWARD_0P15 动作原语可靠 (G1/G2 标定)
✓ ARC_FAST 动作原语可靠 (~3.9s/次, G1 标定)
✓ pipelined_critical save_map 异步保存正常
✓ compact console 模式正常
✓ 事件驱动 zero_hold 自适应退出
✓ base_zero_ok 三条件检测稳定
✓ 安全 Guard 三层防护全部生效
✗ 端到端 "移动→发现风险→拍照→报告" 未集成测试
```

---

## 9. 已冻结验证的稳定协议

### 9.1 C30D 底盘协议 (已冻结)

```
物理层: /dev/base_controller @ 115200, 8N1
帧格式: 0x7B ... BCC 0x7D (TX=11B, RX=24B)
速度单位: mm/s (TX/RX vx/vy), mrad/s (TX/RX wz)
已验证: 串行帧、stop_kick、brake_hold、航位推算 odom、/rosout diag 解析
```

### 9.2 N10P 激光雷达 (已冻结)

```
Topic: /scan (LaserScan), 360°, ~10Hz
扇区分析: 5 扇区 (front 30° + left/right 60° + left45/right45 30°)
每扇区: count, min, p10
已验证: /scan 稳定性, slam_toolbox 建图, 扇区统计
```

### 9.3 D435 深度相机 (已冻结)

```
Topics: color 640x480 rgb8 @30Hz, depth 640x480 16UC1 @29Hz
HOLD_CAPTURE: 10/10 成功, published_cmd_vel=false, base_zero_ok_before=true
证据文件: rgb.png + depth_raw.npy + camera_info.json + odom.json + capture_meta.json
已验证: topic 审计, 单次 capture, 连续 10 次 capture
```

### 9.4 机械臂总线舵机 (协议已冻结，硬件未执行)

```
物理层: UART @ 9600, 8N1
帧协议: 0x55 0x55 <len> <cmd> <params> (无校验和)
CMD_SERVO_MOVE=3: pulse [0,1000], time [0,30000]ms
安全门: 全局 AND 阶段, 7 层验证, build_move_frame 始终标记 for_review_only
已验证: 协议审计, 安全配置, dry-run 计划生成
待执行: 标定 (home_pulse, soft_limit, 方向), B2 单舵机小角度
```

### 9.5 GPIO37 灯光 (已冻结)

```
控制: sysfs PWM, GPIO37, 50Hz, 1100-1900μs
亮度: 0-100% (Int32 topic)
三节点: gpio37_light_node + adaptive_light_controller + risk_light_bridge
已验证: PWM 驱动, dry_run 模式
```

### 9.6 安全 Guard 链路 (已冻结)

```
scan_safety_guard_node: hard_stop=0.30m, emergency_stop=0.20m
wheeltec_tank_base_safe: stop_kick + brake_hold + 看门狗
策略层: max_consecutive_fast_arc≤2, max_total_forward≤1.0m
已验证: 三层防护, P4-Y2 压力测试保险生效
```

### 9.7 EdgeRobot 任务协议 (已冻结)

**文件**: `src/edge_robot_protocol.py`

| 协议对象 | 关键字段 | 验证状态 |
|---------|---------|---------|
| PolicyState | state_id, base_zero_ok, odom, front_p10 | ✓ P4-X/Arm-A/LLM-A 一致 |
| PolicyAction | action_id, action_type, requires_base_zero | ✓ 三种 action type 跨场景验证 |
| ActionResult | status, base_zero_ok_before, evidence_paths | ✓ 10/10 连续验证通过 |
| CaptureMeta | capture_id, rgb/depth/camera_info paths | ✓ P4-X 证据完整 |
| RiskPoint | bbox, depth_m, camera_point_xyz_m | ✓ mock 生成正常 |
| EpisodeReport | episode_id, policy_state, actions, action_results | ✓ P4-X + Arm-A + LLM-A 格式一致 |

---

## 10. Gazebo 迁移条件评估

### 10.1 已具备的条件

| 条件 | 状态 | 说明 |
|------|------|------|
| 完整 Gazebo 模型 | ✓ | 底盘+履带+N10P+D435+5-DOF 臂, URDF/Xacro 完整 |
| 传感器仿真插件 | ✓ | GPU lidar (720 rays, /scan), RGB camera (640x480), depth camera |
| ros_gz_bridge 配置 | ✓ | /clock, /cmd_vel, /scan, /camera/* 全部桥接 |
| 运动控制仿真 | ✓ | DiffDrive skid-steer plugin, ros2_control joint_trajectory_controller |
| 安全 Guard 仿真 | ✓ | scan_safety_guard_node 可直接运行, 无硬件依赖 |
| 策略动作原语对标 | ✓ | Gazebo 模型支持 FORWARD/ARC (差速驱动), HOLD_CAPTURE (相机仿真) |
| 协议接口统一 | ✓ | PolicyState/PolicyAction/ActionResult 与真机共享 TypedDict 定义 |
| 物理尺寸对标 | ✓ | MODEL_MEASUREMENTS.md 记录实车尺寸, 已录入模型 |
| 真机 baseline 数据 | ✓ | P4 系列有完整 step 级别的 state→action→result 记录 |
| 服务器硬件 | ✓ | 8×4090, 可并行多 Gazebo 实例 |
| 机械臂仿真模型 | ✓ | 5 revolute joints, mimic gripper, 关节限位 |
| 灯光仿真 | △ | Gazebo 中可仿真但非必要 (RL 训练不依赖灯光) |

### 10.2 待补充的条件

| 条件 | 优先级 | 说明 |
|------|--------|------|
| **端到端集成测试** | **高** | 真机上跑一次 "移动→发现风险→HOLD_CAPTURE→生成报告" 的完整流程 |
| **Gazebo world 场景** | 高 | 当前仅 empty world, 需建管廊/窄道/障碍物/风险纹理场景 |
| **风险检测仿真** | 高 | Gazebo 中放置裂缝/锈蚀纹理, D435 模拟相机可拍到, mock detector 触发 |
| **履带参数实测** | 中 | 履带宽/高/驱动轮直径/左右中心距 (当前占位值) |
| **舵机角度范围实测** | 中 | ID1-ID5 的 pulse 方向/范围/软限位 (当前占位值) |
| **RL 训练脚本** | 中 | 离散动作 Gym wrapper, 奖励函数, 训练配置 |
| **sim-to-real 验证流程** | 中 | 真机/Gazebo 协议一致性测试脚本 |
| **Ubuntu 24.04 环境** | 中 | 服务器需装 ROS 2 Jazzy + Gazebo Harmonic (模型包已有安装脚本) |
| **RISC-V 推理部署** | 低 | 竞赛展示用, 不阻塞 RL 训练 |
| **LLM 在线推理** | 低 | 当前 LLM-A 已证明确定性报告生成可行, 在线版可后接 |

### 10.3 评估结论

**已具备迁移 Gazebo RL 的基本条件，但建议先完成端到端真机集成测试。**

理由:
1. 当前每个子系统都独立验证通过 (底盘/建图/D435/机械臂/灯光), 但从未串联跑过一次完整的 "自主探图 + 风险发现 + 证据保存" 流程
2. 这次集成测试会暴露协议层的连接问题 (PolicyState → PolicyAction → ActionResult 在真实多步 run 中是否都准确), 修复成本远低于到 Gazebo 训练完才发现
3. Gazebo 模型和协议接口已经就绪, 不会因为晚一周上线而阻塞整体进度
4. 端到端集成测试的录像/日志本身就是竞赛材料的一部分

**建议顺序**: 端到端真机集成 (1 天) → Gazebo 场景搭建 (2 天) → RL 训练 (3-5 天)

---

## 11. 缺失项与待补充

### 11.1 数据集

| 缺失项 | 说明 | 优先级 |
|--------|------|--------|
| 风险图像数据集 | 裂缝/锈蚀/堵塞实拍图像 (用于训练 YOLO 或微调) | 中 |
| 多场景 lidar 数据 | 管廊/涵洞/窄道/分叉口的 /scan + /odom 记录 | 中 |
| 端到端 episode 记录 | 完整 "探图→风险→拍照→日志" 的 episode 数据 | 高 |
| 机械臂操作演示数据 | 移除障碍物的动作序列视频/数据 | 低 |

### 11.2 仿真地图 (Gazebo World)

| 缺失项 | 说明 | 优先级 |
|--------|------|--------|
| 管廊场景 | T 型分叉, 窄道 0.8m, 墙壁纹理 | 高 |
| 障碍物模型 | 可移除的砖块/碎片 SDF 模型 | 高 |
| 风险纹理 | 管道裂缝/锈蚀纹理贴图 (用于 D435 仿真检测) | 高 |
| 低照度场景 | 暗光/无光区域 (验证 lidar-only 鲁棒性) | 中 |
| 设备舱场景 | 起点场景, 狭窄入口 | 中 |
| 多场景随机化 | 障碍物位置随机、走廊宽度随机化 (RL 训练用) | 中 |

### 11.3 软件基础设施

| 缺失项 | 说明 | 优先级 |
|--------|------|--------|
| **端到端集成测试脚本** | 串联底盘+D435+建图+LLM 的完整 run | **最高** |
| Gazebo RL Gym wrapper | 将 Gazebo 封装为标准 Gym env | 高 |
| 奖励函数定义 | 基于 PolicyState/ActionResult 的奖励计算 | 高 |
| sim-to-real 协议验证 | 比较真机和 Gazebo 的 ActionResult 差异 | 中 |
| 服务器训练脚本 | 多实例并行训练 + 周期性评估 | 中 |
| 机械臂硬件标定 | 实测 home_pulse/soft_limit/方向 | 中 |

### 11.4 竞赛材料

| 缺失项 | 说明 | 优先级 |
|--------|------|--------|
| Gazebo 演示录屏 | 画中画: Gazebo+RVIZ+终端 | 低 (最后做) |
| 架构图 | 感知→决策→执行→安全 分层框图 | 低 |
| 实物展示准备 | K1 真机 + 海报 | 低 |
| LLM 巡检报告示例 | 基于真实 episode 生成的完整报告 | 中 |

---

## 附录 A：文件索引

### 核心协议/配置
- `src/edge_robot_protocol.py` — 任务协议 TypedDict
- `src/arm_safety.py` — 机械臂 7 层安全验证
- `configs/arm_safety_config.json` — 机械臂安全边界
- `configs/arm_b_no_load_sample_v0_candidate.json` — 候选动作序列
- `configs/risk_classes.yaml` — 风险类别定义
- `configs/sop_knowledge_base.json` — SOP 知识库

### 核心工具
- `tools/guarded_auto_mapping_micro.py` — 自动建图策略主程序 (4300 行)
- `tools/generate_arm_b_no_load_dry_run_plan.py` — 机械臂 dry-run 生成器
- `tools/generate_llm_a_risk_report.py` — LLM 报告生成器
- `tools/d435_capture_once.py` — D435 单次 capture
- `tools/d435_topic_audit.py` — D435 topic 审计
- `tools/validate_episode_report_schema.py` — 协议 schema 验证
- `tools/send_c30d_serial_zero.py` — C30D 串行零速帧

### 仿真模型
- `sim/tracked_robot_description/urdf/tracked_robot.urdf.xacro` — 主模型
- `sim/tracked_robot_description/urdf/chassis.xacro` — 底盘
- `sim/tracked_robot_description/urdf/arm_5dof.xacro` — 机械臂
- `sim/tracked_robot_description/urdf/sensors.xacro` — 传感器
- `sim/tracked_robot_description/urdf/gazebo_plugins.xacro` — Gazebo 插件
- `sim/tracked_robot_description/launch/gazebo.launch.py` — Gazebo 启动
- `sim/tracked_robot_description/MODEL_MEASUREMENTS.md` — 实车尺寸记录

### 关键文档
- `docs/k1_current_operations_handoff_20260629.md` — K1 当前操作状态
- `docs/k1_status_and_8day_aggressive_plan_20260630.md` — 8 天计划
- `docs/arm_bus_servo_protocol_audit_20260630.md` — 总线舵机协议审查
- `docs/p4_guarded_policy_executable_modes_20260629.md` — P4 策略模式
- `docs/p4x_d435_hold_capture_validation_20260629.md` — P4-X 验证
- `docs/evidence_manifest_20260629.md` — 证据清单

---

## 附录 B：竞赛叙事对齐检查

| 竞赛声称 | 当前实现 | 差距 |
|---------|---------|------|
| GPS 拒止 | odom 坐标系 (无 GPS 依赖) | ✓ 已实现 |
| 通信受限 | 全本地闭环 (无云 API) | ✓ 已实现 |
| 低照度 | LiDAR 不受光照影响 + 自适应灯光 | △ 待低照度端到端测试 |
| 遮挡干扰 | 安全 Guard + 多扇区感知 | ✓ 已实现 |
| 离线认知 | 本地策略决策 + 本地 LLM-A 报告生成 | ✓ 已实现 |
| 风险闭环 | 风险检测 → 策略动作 → 证据保存 | △ 待端到端集成 |
| 多源信息融合 | LiDAR + D435 + Odom + IMU | ✓ 已实现 |
| RISC-V 轻量化部署 | 当前在 x86/ARM 伴生计算机运行, RISC-V 部署待验证 | △ 待移植 |
| 管廊/涵洞/设备舱场景 | Gazebo 模型已建, 待场景 world 构建 | △ 待构建 |
| 抢险救灾任务 | 机械臂移除障碍物 (设计完成, 硬件待标定) | △ 待 Arm-B2+ |
