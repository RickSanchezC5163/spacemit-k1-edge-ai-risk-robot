# K1 整车当前关键路径操作手册

更新时间：2026-06-29  
当前定位：P4 已完成到有界多步 guarded autonomous mapping 雏形。下一步建议进入 P4-Z lite 协议层与 P4-X D435 HOLD_CAPTURE。

本文用于新对话交接：说明如何 SSH 到 SOC、如何 source 环境、如何启动 ROS / 雷达 / D435 / guarded mapping、当前验证过的移动语义和策略参数，以及哪些路径和参数名已经验证。

---

## 1. 仓库与机器路径

Windows 本机仓库：

```text
K:\risc-vCar\edge-ai-robot-k1
```

K1 端主仓库：

```text
/home/soc/edge-ai-robot-k1
```

K1 ROS 工作区：

```text
/home/soc/edge-ai-robot-k1/ros2_ws
```

N10P 雷达工作区：

```text
/home/soc/lslidar_ws
```

D435 RealSense 工作区：

```text
/home/soc/realsense_ws
```

ROS 系统安装：

```text
/opt/ros/humble
```

D435 librealsense 约定安装位置：

```text
/opt/ext/librealsense/librealsense-2.56.4
```

STM32 固件源码本机位置：

```text
K:\risc-vCar\ros相关\Mini小车_D版STM32源码_2025.01.13(默认GMR编码器)
```

历史参考 ROS 相关源码：

```text
K:\risc-vCar\ros相关\src
```

当前分支：

```text
codex/mapping-mvp-test-tools-20260626
```

最近关键提交：

```text
58399be Validate seven step guarded stress stop
81ddfb7 Validate five step guarded exploration
b7152f4 Add bounded policy forward limit
aae3294 Validate fast arc branch mixed run
922da6d Validate policy arc fast step
```

最新 bundle 备份：

```text
K:\risc-vCar\edge-ai-robot-k1\edge-ai-robot-k1-p4-y2-7step-guarded-stress-58399be.bundle
```

---

## 2. SSH 到 SOC

PowerShell / Windows Terminal：

```powershell
ssh soc@192.168.43.40
```

如果连接超时：

```powershell
ping 192.168.43.40
ssh -v soc@192.168.43.40
```

常见原因：

```text
K1 未接入同一热点
IP 改变
K1 未开机或 Wi-Fi 未连接
SSH 服务未起来
```

进入 K1 后先切主仓库：

```bash
cd /home/soc/edge-ai-robot-k1
```

---

## 3. 每次 ROS 操作前必须 source

底盘 + 雷达 + mapping / policy 推荐 source：

```bash
source /opt/ros/humble/setup.bash
source /home/soc/lslidar_ws/install/setup.bash
source /home/soc/edge-ai-robot-k1/ros2_ws/install/setup.bash
```

D435 单独测试时加 source：

```bash
source /opt/ros/humble/setup.bash
source /home/soc/realsense_ws/install/setup.bash
source /home/soc/edge-ai-robot-k1/ros2_ws/install/setup.bash
```

清理 FastDDS 共享内存：

```bash
rm -f /dev/shm/fastrtps* /dev/shm/fastdds* 2>/dev/null || true
```

构建 K1 ROS 工作区：

```bash
cd /home/soc/edge-ai-robot-k1/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

---

## 4. 设备与话题概览

当前硬件：

```text
K1 Muse Pi Pro + Bianbu LXQT v2.3.3
N10P 雷达：/scan, frame_id=laser
C30D Tank 底盘：/dev/base_controller
D435：RGB/depth 已验证，下一步做 HOLD_CAPTURE
总线舵机：1-4 已调通，2/3 有机械碰撞风险
灯光：PWM7 / GPIO37，已验证亮灭与亮度调节
```

关键 ROS 话题：

```text
/scan
/odom
/map
/map_metadata
/tf
/tf_static
/input_cmd_vel
/cmd_vel_guarded
/safety/front_obstacle
/chassis/stop_request
```

当前安全运动链路：

```text
/input_cmd_vel
-> scan_safety_guard_node
-> /cmd_vel_guarded
-> wheeltec_tank_base_safe.py
-> C30D
```

禁止自动脚本绕过 guard 直接发 `/cmd_vel_guarded` 或底盘串口。

---

## 5. 启动 guarded mapping stack

当前已验证 launch：

```text
ros2_ws/src/turn_on_wheeltec_robot/launch/n10p_tank_mapping_safety_guard.launch.py
```

后台启动推荐命令：

```bash
cd /home/soc/edge-ai-robot-k1
rm -f /dev/shm/fastrtps* /dev/shm/fastdds* 2>/dev/null || true

nohup bash -lc '
source /opt/ros/humble/setup.bash
source /home/soc/lslidar_ws/install/setup.bash
source /home/soc/edge-ai-robot-k1/ros2_ws/install/setup.bash
ros2 launch turn_on_wheeltec_robot n10p_tank_mapping_safety_guard.launch.py \
  hard_stop_m:=0.30 \
  emergency_stop_m:=0.20 \
  slow_down_m:=0.80 \
  approach_stop_m:=0.80 \
  min_effective_forward:=0.08 \
  clear_max_linear:=0.30 \
  soft_max_linear:=0.30
' > logs/p4_active.launch.log 2>&1 &
```

该 launch 会启动：

```text
lslidar_driver_node
wheeltec_tank_base_safe.py
base_footprint -> laser static TF
slam_toolbox
scan_safety_guard_node
```

启动后检查：

```bash
source /opt/ros/humble/setup.bash
source /home/soc/lslidar_ws/install/setup.bash
source /home/soc/edge-ai-robot-k1/ros2_ws/install/setup.bash

ros2 topic list | sort
timeout 6s ros2 topic echo /safety/front_obstacle --once
timeout 6s ros2 topic echo /odom --once
timeout 8s ros2 topic echo /map_metadata --once
```

期望：

```text
/scan 存在
/odom 存在，静止时 linear.x=0, angular.z=0
/map_metadata 可读
/safety/front_obstacle 包含 front_min_range_m 和 front_p10_range_m
```

停止 ROS 进程：

```bash
pkill -f '[n]10p_tank_mapping_safety_guard.launch.py' || true
sleep 2
pkill -f '[a]sync_slam_toolbox_node' || true
pkill -f '[l]slidar_driver_node' || true
pkill -f '[w]heeltec_tank_base_safe.py' || true
pkill -f '[s]can_safety_guard_node' || true
sleep 2
pgrep -af 'n10p_tank_mapping_safety_guard|slam_toolbox|lslidar_driver|wheeltec_tank_base_safe|scan_safety_guard|guarded_auto_mapping_micro' || true
```

---

## 6. N10P 雷达操作

安装/工作区：

```text
/home/soc/lslidar_ws
```

已使用包：

```text
lslidar_driver
```

mapping launch 内部会引用：

```text
lslidar_driver/launch/lsn10p_launch.py
```

检查雷达串口：

```bash
ls -l /dev/wheeltec_lidar
ls -l /dev/ttyUSB* /dev/ttyACM* /dev/ttyCH343USB* 2>/dev/null || true
```

只检查 `/scan`：

```bash
source /opt/ros/humble/setup.bash
source /home/soc/lslidar_ws/install/setup.bash
ros2 launch lslidar_driver lsn10p_launch.py
```

另一个窗口：

```bash
source /opt/ros/humble/setup.bash
source /home/soc/lslidar_ws/install/setup.bash
ros2 topic echo /scan --once
ros2 topic hz /scan
```

正常情况：

```text
topic: /scan
frame_id: laser
频率约 10Hz
```

---

## 7. D435 RealSense 操作

文档：

```text
docs/d435_realsense.md
tools/d435_static_check.sh
```

安装位置：

```text
/home/soc/realsense_ws
/opt/ext/librealsense/librealsense-2.56.4
```

ROS 包：

```text
realsense2_camera
```

直接启动 D435：

```bash
source /opt/ros/humble/setup.bash
source /home/soc/realsense_ws/install/setup.bash

ros2 launch realsense2_camera rs_launch.py \
  depth_module.depth_profile:=640,480,30 \
  depth_module.infra_profile:=640,480,30 \
  rgb_camera.color_profile:=640,480,30 \
  pointcloud.enable:=false
```

检查 D435 话题：

```bash
ros2 topic list | sort | grep -E 'camera|depth|color|points'
ros2 topic hz /camera/camera/color/image_raw
ros2 topic hz /camera/camera/depth/image_rect_raw
timeout 5s ros2 topic echo --once /camera/camera/color/camera_info
```

如果需要点云：

```bash
ros2 topic hz /camera/camera/depth/color/points
```

但当前建议：

```text
先关闭 pointcloud.enable
先做 RGB/depth/meta 证据链
再考虑点云
```

静态检查脚本：

```bash
cd /home/soc/edge-ai-robot-k1
source /opt/ros/humble/setup.bash
source /home/soc/realsense_ws/install/setup.bash
source /home/soc/edge-ai-robot-k1/ros2_ws/install/setup.bash

ENABLE_POINTCLOUD=0 bash tools/d435_static_check.sh
```

该脚本会通过：

```text
ros2_ws/src/k1_system_bringup/launch/non_arm_bringup.launch.py
```

启动 D435，并记录日志到：

```text
logs/tests/
```

D435 下一步目标：

```text
HOLD_CAPTURE
-> 保存 RGB
-> 保存 depth
-> 保存 odom
-> 保存 camera_info
-> 输出 capture_meta.json
-> mock risk detector 输出 risk_point.json
```

---

## 8. SLAM 地图保存

推荐用 slam_toolbox 服务，不优先用 map_saver_cli：

```bash
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
"{name: {data: '/home/soc/edge-ai-robot-k1/maps/manual_snapshot_YYYYMMDD_HHMMSS'}}"
```

policy 工具自动保存路径：

```text
/home/soc/edge-ai-robot-k1/maps/
```

本机拉回后路径：

```text
K:\risc-vCar\edge-ai-robot-k1\maps
```

---

## 9. 当前移动语义

现在不再按 `速度 x 时间 = 位移` 设计动作。

正确抽象：

```text
cmd_vel 只是油门
/odom 才是尺子
每一步动作后必须停稳并验证 base_zero_ok
```

### 9.1 forward-staged

语义：

```text
小步 odom 闭环前进
远处较快，接近目标变慢，到动态刹车线提前停
```

已验证速度档：

```text
forward_fast_speed = 0.15
forward_mid_speed  = 0.12
forward_slow_speed = 0.10
forward_brake_coef_s = 1.05
forward_static_brake_margin_m = 0.02
forward_brake_margin_m = 0.03
forward_timeout_s = 5.0
```

policy 中动作名：

```text
FORWARD_0P05 -> target 0.05m, front_gate 0.40m
FORWARD_0P10 -> target 0.10m, front_gate 0.60m
FORWARD_0P15 -> target 0.15m, front_gate 0.80m 或 mapping 模式 1.20m
```

### 9.2 ARC30_PRECISE

语义：

```text
离散角度闭环
步内开环 arc-step
每步后停稳读 odom yaw
进入目标区间后停止
```

参数：

```text
arc_yaw_target_deg = 30
arc_yaw_tolerance_deg = 6
arc_step_linear = 0.10
arc_step_angular = 0.50
arc_step_duration_s = 1.0
arc_max_steps = 4
zero_hold_s = 4.0 或 5.0
```

用途：

```text
实验记录、可解释验证、慢但稳
```

### 9.3 ARC_FAST

语义：

```text
快速单脉冲弧线动作
不追求精确 30 度
依靠每步重新读取 front_p10/front_min 进行行为闭环
```

当前验证参数：

```text
policy_arc_mode = fast
policy_arc_fast_linear = 0.12
policy_arc_fast_angular = 0.80
policy_arc_fast_duration_s = 1.0
policy_max_consecutive_fast_arc = 2
```

预期：

```text
多数情况下 yaw 约 20-35 度
可能出现 14-16 度，不视为单步失败
必须看下一步 front_p10 是否改善
```

已验证结果：

```text
G2-A: ARC_FAST_LEFT yaw +21.05deg
G2-B: 3-step 主循环 25.081s -> 17.660s
P4-Y1: 5-step 成功，FORWARD -> ARC_FAST -> FORWARD
P4-Y2: 连续 2 次 ARC_FAST 后未稳定打开空间，HOLD_MAX_FAST_ARC 拦停
```

### 9.4 HOLD / HOLD_CAPTURE

语义：

```text
不运动
zero_hold
检查 base_zero_ok
保存地图或 capture placeholder
```

当前 `HOLD_AND_CAPTURE` 还只是 placeholder，下一步 P4-X 要接 D435 实拍。

---

## 10. Policy profile 与阈值

代码路径：

```text
tools/guarded_auto_mapping_micro.py
```

### 10.1 mapping_safe_mode

阈值：

```text
front_p10 < 0.50:
  HARD_STOP

0.50 <= front_p10 < 0.80:
  HOLD_AND_SAVE

0.80 <= front_p10 < 1.20:
  ARC30_PREFERRED

front_p10 >= 1.20:
  FORWARD_ALLOWED
```

### 10.2 interaction_mode

阈值：

```text
front_min < 0.20:
  HARD_STOP

front_p10 < 0.30:
  HOLD_AND_CAPTURE

0.30 <= front_p10 < 0.40:
  HOLD_SAVE_OBSERVE

0.40 <= front_p10 < 0.60:
  ARC30_OR_FORWARD_0P05

0.60 <= front_p10 < 0.80:
  ARC30_OR_FORWARD_0P10

front_p10 >= 0.80:
  FORWARD_0P15_OR_ARC30
```

常用 policy 选择参数：

```text
POLICY_CLOSE_ACTION=arc30|forward
POLICY_MID_ACTION=arc30|forward
POLICY_NORMAL_ACTION=forward|arc30
POLICY_ARC_DIRECTION=auto|left|right
POLICY_ARC_MODE=precise|fast
```

当前推荐：

```text
POLICY_ARC_MODE=fast
POLICY_CLOSE_ACTION=arc30
POLICY_MID_ACTION=arc30
POLICY_NORMAL_ACTION=forward
POLICY_ARC_DIRECTION=auto
```

---

## 11. 运行 policy dry-run

dry-run 不发布 `/input_cmd_vel`，用于看策略会选什么动作。

```bash
cd /home/soc/edge-ai-robot-k1
source /opt/ros/humble/setup.bash
source /home/soc/lslidar_ws/install/setup.bash
source /home/soc/edge-ai-robot-k1/ros2_ws/install/setup.bash

POLICY_ARC_MODE=fast \
POLICY_MID_ACTION=arc30 \
POLICY_CLOSE_ACTION=arc30 \
POLICY_NORMAL_ACTION=forward \
POLICY_MAX_STEPS=5 \
POLICY_MAX_RUNTIME_S=120 \
POLICY_MAX_TOTAL_FORWARD_M=1.0 \
POLICY_MAX_CONSECUTIVE_FAST_ARC=2 \
ZERO_HOLD_S=4.0 \
ZERO_MIN_HOLD_S=0.8 \
ZERO_POLL_S=0.1 \
ZERO_CONFIRM_SAMPLES=3 \
SAVE_POLICY=pipelined_critical \
SAVE_EVERY_N=2 \
MAX_PENDING_SAVES=1 \
CONSOLE_MODE=compact \
bash tools/p4w_guarded_policy_branch_mixed.sh dry-run
```

---

## 12. 运行 5-step 有界探图

先确认现场安全、有人看管、能随时断电或抬车。

```bash
cd /home/soc/edge-ai-robot-k1
source /opt/ros/humble/setup.bash
source /home/soc/lslidar_ws/install/setup.bash
source /home/soc/edge-ai-robot-k1/ros2_ws/install/setup.bash

POLICY_ARC_MODE=fast \
POLICY_MID_ACTION=arc30 \
POLICY_CLOSE_ACTION=arc30 \
POLICY_NORMAL_ACTION=forward \
POLICY_MAX_STEPS=5 \
POLICY_MAX_RUNTIME_S=120 \
POLICY_MAX_TOTAL_FORWARD_M=1.0 \
POLICY_MAX_CONSECUTIVE_FAST_ARC=2 \
ZERO_HOLD_S=4.0 \
ZERO_MIN_HOLD_S=0.8 \
ZERO_POLL_S=0.1 \
ZERO_CONFIRM_SAMPLES=3 \
SAVE_POLICY=pipelined_critical \
SAVE_EVERY_N=2 \
MAX_PENDING_SAVES=1 \
CONSOLE_MODE=compact \
bash tools/p4w_guarded_policy_branch_mixed.sh run
```

P4-Y1 已验证结果：

```text
FORWARD -> ARC_FAST_LEFT -> FORWARD -> FORWARD -> FORWARD
base_zero_ok=true
final_map_saved=true
cumulative_positive_forward_m=0.7515m
```

---

## 13. 运行 7-step 压力测试

此项不是自由探索，是压力测试。

```bash
POLICY_ARC_MODE=fast \
POLICY_MID_ACTION=arc30 \
POLICY_CLOSE_ACTION=arc30 \
POLICY_NORMAL_ACTION=forward \
POLICY_MAX_STEPS=7 \
POLICY_MAX_RUNTIME_S=180 \
POLICY_MAX_TOTAL_FORWARD_M=1.0 \
POLICY_MAX_CONSECUTIVE_FAST_ARC=2 \
ZERO_HOLD_S=4.0 \
ZERO_MIN_HOLD_S=0.8 \
ZERO_POLL_S=0.1 \
ZERO_CONFIRM_SAMPLES=3 \
SAVE_POLICY=pipelined_critical \
SAVE_EVERY_N=2 \
MAX_PENDING_SAVES=1 \
CONSOLE_MODE=compact \
bash tools/p4w_guarded_policy_branch_mixed.sh run
```

P4-Y2 已验证结果：

```text
ARC_FAST_RIGHT -> ARC_FAST_RIGHT -> HOLD_MAX_FAST_ARC
stop=max_consecutive_fast_arc_reached
base_zero_ok=true
final_map_saved=true
```

这个结果说明连续 fast arc 保险有效。

---

## 14. 保存策略参数

推荐：

```text
SAVE_POLICY=pipelined_critical
SAVE_EVERY_N=2
MAX_PENDING_SAVES=1
CONSOLE_MODE=compact
```

含义：

```text
普通步骤：异步/流水线保存，避免阻塞主循环
critical 事件：同步保存
run end：同步保存 final map
每步 JSON checkpoint 始终记录
```

critical 事件包括：

```text
HARD_STOP
HOLD_AND_CAPTURE
HOLD_SAVE_OBSERVE
HOLD_AND_SAVE
HOLD_MAX_FAST_ARC
max_consecutive_fast_arc_reached
```

---

## 15. 停稳参数

当前推荐：

```text
ZERO_HOLD_S=4.0
ZERO_MIN_HOLD_S=0.8
ZERO_POLL_S=0.1
ZERO_CONFIRM_SAMPLES=3
```

语义：

```text
先等最短 0.8s
再每 0.1s 检查一次 base_zero
连续 3 次确认 cmd/serial/fb/odom 归零后进入下一步
最长等到 4.0s
```

不要继续压低到 1s 以内。

---

## 16. 关键源码路径

底盘安全节点：

```text
ros2_ws/src/turn_on_wheeltec_robot/turn_on_wheeltec_robot/wheeltec_tank_base_safe.py
```

scan safety guard：

```text
ros2_ws/src/k1_sensor_event_adapter/k1_sensor_event_adapter/scan_safety_guard_node.py
```

guarded mapping launch：

```text
ros2_ws/src/turn_on_wheeltec_robot/launch/n10p_tank_mapping_safety_guard.launch.py
```

基础 mapping launch：

```text
ros2_ws/src/turn_on_wheeltec_robot/launch/n10p_tank_mapping.launch.py
```

D435 / non-arm launch：

```text
ros2_ws/src/k1_system_bringup/launch/non_arm_bringup.launch.py
```

policy / motion primitive 工具：

```text
tools/guarded_auto_mapping_micro.py
```

P4-W/P4-Y helper：

```text
tools/p4w_guarded_policy_branch_mixed.sh
```

D435 static check：

```text
tools/d435_static_check.sh
```

---

## 17. 关键文档与证据

P4 policy / fast arc / Y1 / Y2 主文档：

```text
docs/p4_guarded_policy_executable_modes_20260629.md
```

D435 文档：

```text
docs/d435_realsense.md
```

Bring-up 命令：

```text
docs/bringup_commands.md
```

P4-Y1 report：

```text
logs/policy_p4w_run_branch_mixed_20260629_182847.json
```

P4-Y2 report：

```text
logs/policy_p4w_run_branch_mixed_20260629_183731.json
```

P4-Y2 final marked map：

```text
maps/policy_p4w_branch_mixed_20260629_183731_final_marked.png
```

---

## 18. 禁止事项

当前不要做：

```text
不启动官方 RRT
不启动 AMCL 主定位
不跑无人看管 Nav2
不绕过 scan_safety_guard
不直接写 /cmd_vel_guarded
不让自动脚本直接控制串口
不把 P4-Y 叫完整自由探索
不继续扩 10-step
```

底盘测试必须：

```text
人在现场
确认地面安全
随时准备抬车或断电
每次 run 后清理 ROS 进程
```

---

## 19. 下一步建议

不要继续扩步。下一步进入：

```text
P4-Z lite：协议层
P4-X：HOLD_AND_CAPTURE 接 D435 实拍
```

建议顺序：

```text
1. 定义 episode_report.json / PolicyState / PolicyAction / ActionResult / RiskPoint
2. 在 HOLD_AND_CAPTURE 中保存 RGB/depth/odom/camera_info/meta
3. mock risk detector 输出 risk_point.json
4. LLM-A 从 episode_report.json 生成巡检报告
5. 机械臂固定动作链作为半主线
6. Gazebo / RL 只做加分项
```

一句话状态：

```text
P4 已经证明安全可控移动底座成立；后续应从“能移动”转向“能产生任务证据和报告”。
```
