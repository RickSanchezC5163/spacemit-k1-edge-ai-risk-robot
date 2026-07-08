# 初赛遥控建图风险闭环 - 2026-07-03

## 目标

先完成一个可录制的初赛代码闭环：

```text
遥控低速建图
-> D435 本地 YOLO 风险识别
-> 风险报警
-> bbox + depth + odom 近似风险点落图
-> 生成手动机械臂 no-load 响应候选
-> 操作者遥控决定什么时候、在哪里执行 no-load 动作
-> 生成风险报告和展示面板
```

不做 RL 控制真车，不做自动清障，不做机械臂自主触发。

## 新增脚本

```bash
python3 tools/run_prelim_remote_mapping_yolo_arm_demo.py
```

脚本职责：

- 订阅 D435 RGB/depth/camera_info 和 `/odom`
- 运行本地 YOLO ONNX 推理
- 按类别 + 地图网格去重保存风险事件
- 发布 `/perception/mock_event` 给现有 risk engine
- 发布 `/prelim_demo/alarm` 和 `/prelim_demo/risk_event`
- 写入 `risk_events.jsonl`、`risk_event_index.json`、`risk_map_points.json`
- 生成 `risk_map_snapshot.png`、`risk_control_report.md`、`dashboard.html`
- 为高风险事件生成 `manual_arm_response_candidate.json`

脚本不会发布 `cmd_vel`，不会启动底盘运动，也不会自动启动机械臂。

## 推荐启动顺序

### Terminal 1 - 遥控建图安全链

```bash
cd ~/edge-ai-robot-k1
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
source ~/lslidar_ws/install/setup.bash

ros2 launch turn_on_wheeltec_robot n10p_tank_mapping_safety_guard.launch.py \
  hard_stop_m:=0.30 \
  emergency_stop_m:=0.20 \
  slow_down_m:=0.80 \
  approach_stop_m:=0.80 \
  min_effective_forward:=0.08 \
  clear_max_linear:=0.30 \
  soft_max_linear:=0.30
```

遥控输入必须走：

```text
/input_cmd_vel -> scan_safety_guard_node -> /cmd_vel_guarded -> Tank base
```

### Terminal 2 - D435

```bash
source /opt/ros/humble/setup.bash
ros2 launch realsense2_camera rs_launch.py \
  depth_module.depth_profile:=640,480,30 \
  depth_module.infra_profile:=640,480,30 \
  rgb_camera.color_profile:=640,480,30
```

### Terminal 3 - 风险引擎和日志

```bash
cd ~/edge-ai-robot-k1
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash

ros2 run k1_risk_engine risk_engine_node
```

可选日志：

```bash
ros2 run k1_event_logger event_logger_node --ros-args -p log_dir:=logs/prelim_demo_events
```

### Terminal 4 - YOLO 风险闭环

```bash
cd ~/edge-ai-robot-k1
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash

python3 tools/run_prelim_remote_mapping_yolo_arm_demo.py \
  --provider spacemit \
  --model models/risk_vision/yolov8n_320_q_truncated_balanced.onnx \
  --imgsz 320 \
  --conf 0.20 \
  --output-dir outputs/prelim_remote_mapping_yolo_arm_demo_v1/live_001
```

输出目录会持续更新：

- `dashboard.html`
- `alarm_state.json`
- `risk_events.jsonl`
- `risk_event_index.json`
- `risk_map_points.json`
- `risk_map_snapshot.png`
- `risk_control_report.md`
- `episode_report.json`
- `captures/<risk_event_id>/...`

## 机械臂 no-load 响应策略

机械臂响应由操作者遥控决定，不由 YOLO 自动触发。

触发流程：

1. YOLO 识别到风险并报警。
2. 脚本生成 `manual_arm_response_candidate.json`。
3. 操作者根据 UI/地图风险点，遥控底盘到安全响应位置。
4. 操作者确认底盘静止、工作空间安全、急停可用。
5. 操作者手动执行 candidate 文件中的 dry-run 或 hardware 命令。

候选文件位置示例：

```text
outputs/prelim_remote_mapping_yolo_arm_demo_v1/live_001/
  captures/<risk_event_id>/manual_arm_response_candidate/manual_arm_response_candidate.json
```

先执行 dry-run：

```bash
python3 tools/run_arm_b3_no_load_sample_sequence.py \
  --output-dir outputs/prelim_remote_mapping_yolo_arm_demo_v1/live_001/manual_arm_dryrun_001 \
  --serial-port /dev/arm_bus \
  --baudrate 9600
```

只有在操作者确认位置、空间、安全门后，才执行硬件 no-load：

```bash
python3 tools/run_arm_b3_no_load_sample_sequence.py \
  --output-dir outputs/prelim_remote_mapping_yolo_arm_demo_v1/live_001/manual_arm_hw_001 \
  --serial-port /dev/arm_bus \
  --baudrate 9600 \
  --enable-hardware-write \
  --confirm-no-load-sample-sequence
```

展示口径：

- 可以说：风险报警后，操作者遥控到安全位置，手动触发机械臂 no-load 响应并回到 6b。
- 不要说：YOLO 自动控制机械臂清障。
- 不要说：机械臂已经完成真实清障、抓取、接触或搬运。

## 视频段落对应

0:30-1:30 真机段建议录制：

```text
遥控低速建图
-> D435 画面 YOLO bbox
-> alarm_state.json/dashboard.html 报警
-> risk_map_snapshot.png 出现风险点
-> manual_arm_response_candidate.json 生成
```

2:00-2:30 机械臂段建议单独录制：

```text
操作者已经遥控到选定位置
-> 手动执行 Arm-B3 no-load
-> 机械臂完成固定动作
-> 回到 safe_idle_home_like_6b
```

## Claim Boundary

- 遥控建图，不是自动探索。
- YOLO 是本地视觉模型推理，但当前只声明打印风险图卡/演示场景识别。
- 风险点是近似落图，不声明高精度坐标。
- 机械臂是人工触发 no-load 响应，不声明自主清障。
- RL 不控制真车；视频中 RL 只作为仿真验证材料。
