# K1 实机 RRT + Nav2 探图准备流程 - 2026-07-17

本文档用于把 Ubuntu 仿真中已经跑通的 RRT frontier 探图流程迁移到 K1 实机。当前阶段只接入底盘里程计、二维雷达建图和 D435 YOLO 风险识别，不接机械臂自动动作。

## 1. 控制链路

手动建图链路：

```text
keyboard teleop -> /input_cmd_vel
scan_safety_guard_node(/scan)
-> /cmd_vel_guarded
-> tank base
-> /odom + /map
```

Nav2/RRT 探图链路：

```text
RRT frontier explorer
-> Nav2 NavigateToPose
-> /cmd_vel_raw
scan_safety_guard_node(/scan)
-> /cmd_vel_guarded
-> tank base
-> SLAM /map update
```

风险识别链路：

```text
D435 RGB-D
-> YOLOv8n SpaceMIT EP
-> /perception/risk_alarm
-> RRT stop-on-risk
-> risk events + map risk point
```

`tools/sim_rrt_frontier_explorer.py` 虽然名字里带 `sim`，但实现只依赖 `/map`、TF `map -> base_footprint`、`/goal_pose` 和 Nav2 action，因此可以直接用于实机。本文档先保留文件名，避免临场改动扩大风险。

## 2. K1 端脚本

新增脚本：

```bash
/home/soc/edge-ai-robot-k1/tools/start_real_k1_rrt_nav2_mapping.sh
```

所有命令都在 K1 上执行：

```bash
cd /home/soc/edge-ai-robot-k1
bash tools/start_real_k1_rrt_nav2_mapping.sh help
```

脚本会自动 source ROS2 和本项目工作空间，并把当前 run 目录写入：

```text
/home/soc/edge-ai-robot-k1/.current_real_k1_rrt_nav2_run_dir
```

`manual-map` 和 `nav2-slam` 会新建 run 目录；后续 `d435`、`yolo-ep`、`rrt-preview`、`rrt-run`、`ui`、`save-map` 在不显式传 `run_dir` 时会复用这个目录。

从 Windows 部署到 K1：

```powershell
Set-Location K:\risc-vCar\edge-ai-robot-k1
scp tools\start_real_k1_rrt_nav2_mapping.sh soc@192.168.43.40:/home/soc/edge-ai-robot-k1/tools/
scp tools\win_start_real_k1_rrt_nav2_mapping.ps1 soc@192.168.43.40:/home/soc/edge-ai-robot-k1/tools/
scp docs\real_k1_rrt_nav2_mapping_bringup_20260717.md soc@192.168.43.40:/home/soc/edge-ai-robot-k1/docs/
scp tools\sim_rrt_frontier_explorer.py soc@192.168.43.40:/home/soc/edge-ai-robot-k1/tools/
scp ros2_ws\src\turn_on_wheeltec_robot\launch\n10p_tank_nav2_slam.launch.py soc@192.168.43.40:/home/soc/edge-ai-robot-k1/ros2_ws/src/turn_on_wheeltec_robot/launch/
```

K1 上检查：

```bash
cd /home/soc/edge-ai-robot-k1
bash -n tools/start_real_k1_rrt_nav2_mapping.sh
python3 -m py_compile tools/sim_rrt_frontier_explorer.py ros2_ws/src/turn_on_wheeltec_robot/launch/n10p_tank_nav2_slam.launch.py
```

## 3. 建议启动顺序

### 3.1 清理旧进程

```bash
cd /home/soc/edge-ai-robot-k1
bash tools/start_real_k1_rrt_nav2_mapping.sh clean
```

### 3.2 先恢复底盘手动操作

窗口 A：启动手动建图安全链路。

```bash
cd /home/soc/edge-ai-robot-k1
bash tools/start_real_k1_rrt_nav2_mapping.sh manual-map
```

窗口 B：键盘遥控。

```bash
cd /home/soc/edge-ai-robot-k1
bash tools/start_real_k1_rrt_nav2_mapping.sh teleop-manual
```

按键说明仍然按 `teleop_twist_keyboard` 默认规则：

```text
i 前进
, 后退
j 原地左转
l 原地右转
k 停止
q/z 同时增减速度
w/x 只增减线速度
e/c 只增减角速度
```

当前小场景安全边界：

```text
<= 0.10 m: stop
0.20-0.30 m: slow
>= 0.30 m: clear speed cap
```

### 3.3 切到 Nav2 + RRT 探图

先关闭窗口 A/B，或者执行：

```bash
cd /home/soc/edge-ai-robot-k1
bash tools/start_real_k1_rrt_nav2_mapping.sh zero
bash tools/start_real_k1_rrt_nav2_mapping.sh clean
```

窗口 A：启动 Nav2-SLAM 安全链路。

```bash
cd /home/soc/edge-ai-robot-k1
bash tools/start_real_k1_rrt_nav2_mapping.sh nav2-slam
```

窗口 B：预检查。

```bash
cd /home/soc/edge-ai-robot-k1
bash tools/start_real_k1_rrt_nav2_mapping.sh preflight
```

窗口 C：先只发布 RRT frontier 目标，不让 Nav2 自动执行。预览目标发布在 `/rrt_preview_goal_pose`，用于 RViz 里看目标点是否合理。

```bash
cd /home/soc/edge-ai-robot-k1
bash tools/start_real_k1_rrt_nav2_mapping.sh rrt-preview
```

确认目标点合理后，再运行真实 Nav2 action：

```bash
cd /home/soc/edge-ai-robot-k1
bash tools/start_real_k1_rrt_nav2_mapping.sh rrt-run
```

`rrt-run` 只发 Nav2 目标，不直接发布 `Twist`。底盘速度仍走：

```text
Nav2 -> /cmd_vel_raw -> scan_safety_guard_node -> /cmd_vel_guarded
```

如果需要人工接管 Nav2-SLAM 模式，可开键盘窗口：

```bash
cd /home/soc/edge-ai-robot-k1
bash tools/start_real_k1_rrt_nav2_mapping.sh teleop-nav2
```

注意：`teleop-nav2` 发布到 `/cmd_vel_raw`，用于 Nav2-SLAM 模式；`teleop-manual` 发布到 `/input_cmd_vel`，用于纯手动建图模式。

## 4. 接入 D435 YOLO 风险识别

先启动 D435 ROS 驱动：

```bash
cd /home/soc/edge-ai-robot-k1
bash tools/start_real_k1_rrt_nav2_mapping.sh d435
```

另开窗口：

```bash
cd /home/soc/edge-ai-robot-k1
bash tools/start_real_k1_rrt_nav2_mapping.sh yolo-ep
```

默认风险自动落图边界：

```text
crack: confidence >= 0.29, 0.60m <= depth <= 0.80m
blockage: confidence >= 0.23, 0.35m <= depth <= 0.75m
```

如需临时改边界：

```bash
export AUTO_RISK_GATES='crack:0.29:0.60:0.80,blockage:0.23:0.35:0.75'
bash tools/start_real_k1_rrt_nav2_mapping.sh yolo-ep
```

RRT 已兼容真实 `/perception/risk_alarm` 的字段：

```text
alarm=true
event_id + class_name
active_risk_id + state
detection_count > 0
```

因此实机 YOLO 报警后，RRT 会停止继续探图，便于后续切换到风险抵近或人工处置流程。

## 5. 本地 UI 和语音

风险 UI：

```bash
cd /home/soc/edge-ai-robot-k1
bash tools/start_real_k1_rrt_nav2_mapping.sh ui
```

Windows 访问：

```text
http://192.168.43.40:8765/yolo_monitor.html
```

这里的 UI 服务目录是当前 run 目录下的 `yolo_risk/`，和 `yolo-ep` 写入的 `alarm_state.json`、`latest_overlay.png` 保持一致。

SYN6288 语音桥：

```bash
cd /home/soc/edge-ai-robot-k1
bash tools/start_real_k1_rrt_nav2_mapping.sh voice
```

手动播报 cue：

```bash
source /opt/ros/humble/setup.bash
source /home/soc/edge-ai-robot-k1/ros2_ws/install/setup.bash
ros2 topic pub --once /prelim_demo/voice_cue std_msgs/msg/String "{data: 'blockage_detected'}"
ros2 topic pub --once /prelim_demo/voice_cue std_msgs/msg/String "{data: 'approach_reached'}"
ros2 topic pub --once /prelim_demo/voice_cue std_msgs/msg/String "{data: 'arm_clear_start'}"
ros2 topic pub --once /prelim_demo/voice_cue std_msgs/msg/String "{data: 'clear_done'}"
ros2 topic pub --once /prelim_demo/voice_cue std_msgs/msg/String "{data: 'report_ready'}"
```

## 6. 保存地图

```bash
cd /home/soc/edge-ai-robot-k1
bash tools/start_real_k1_rrt_nav2_mapping.sh save-map
```

地图默认保存到当前 run 目录：

```text
outputs/real_k1_rrt_nav2_mapping_*/maps/
```

## 7. Windows 端 SSH 示例

已经提供 Windows 启动器：

```powershell
Set-Location K:\risc-vCar\edge-ai-robot-k1
powershell -ExecutionPolicy Bypass -File tools\win_start_real_k1_rrt_nav2_mapping.ps1 -Mode nav2-preview -CleanFirst
```

模式说明：

```text
manual       手动安全建图 + 键盘遥控
nav2-preview Nav2-SLAM + D435 + YOLO + RRT 只发布 /rrt_preview_goal_pose
nav2-run     Nav2-SLAM + D435 + YOLO + RRT 发送 Nav2 action
nav2-preview-2m  2m x 2m 小场景 RRT 只发布 /rrt_preview_goal_pose
nav2-run-2m      2m x 2m 小场景 RRT 发送 Nav2 action
```

Windows PowerShell 中不要直接把未加引号的 `&&` 放在 `ssh` 后面。用下面这种形式：

```powershell
$K1 = "soc@192.168.43.40"
ssh -t $K1 "bash -lc 'cd /home/soc/edge-ai-robot-k1; bash tools/start_real_k1_rrt_nav2_mapping.sh nav2-slam'"
```

开多个窗口时：

```powershell
$K1 = "soc@192.168.43.40"
$Base = "cd /home/soc/edge-ai-robot-k1; bash tools/start_real_k1_rrt_nav2_mapping.sh"

Start-Process powershell -ArgumentList @("-NoExit", "-Command", "ssh -t $K1 `"bash -lc '$Base nav2-slam'`"")
Start-Sleep -Seconds 10
Start-Process powershell -ArgumentList @("-NoExit", "-Command", "ssh -t $K1 `"bash -lc '$Base d435'`"")
Start-Sleep -Seconds 6
Start-Process powershell -ArgumentList @("-NoExit", "-Command", "ssh -t $K1 `"bash -lc '$Base yolo-ep'`"")
Start-Process powershell -ArgumentList @("-NoExit", "-Command", "ssh -t $K1 `"bash -lc '$Base rrt-preview'`"")
Start-Process powershell -ArgumentList @("-NoExit", "-Command", "ssh -t $K1 `"bash -lc '$Base ui'`"")
Start-Process "http://192.168.43.40:8765/yolo_monitor.html"
```

第一次实机建议只跑到 `rrt-preview`，确认 RViz 中目标点和障碍膨胀合理后，再换成 `rrt-run`。
