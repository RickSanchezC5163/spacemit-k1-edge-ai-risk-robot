# K1 边缘智能巡检排障小车项目汇报

日期：2026-07-06  
仓库：`K:\risc-vCar\edge-ai-robot-k1`  
当前分支：`codex/mapping-mvp-test-tools-20260626`

## 1. 项目定位

本项目面向 GPS 拒止、通信受限、低照度、遮挡干扰等复杂受限空间场景，基于 SpacemiT K1 Muse Pi Pro / Bianbu / ROS2 Humble 构建一套边缘智能巡检排障小车系统。系统以 K1 作为车载计算核心，融合 N10P 激光雷达、D435 RGB-D 相机、C30D Tank 履带底盘、PWM 补光灯和总线舵机机械臂，实现低速安全建图、局部风险感知、风险事件记录、近似风险点落图和操作员确认后的 no-load 机械臂响应。

当前初赛推荐展示路径已经收敛为：

```text
远程低速 guarded SLAM 建图
-> D435 本地 YOLO 风险识别
-> 置信度/深度门控风险报警
-> bbox + depth + odom 近似风险点落图
-> 操作员选择手动机械臂 no-load 响应
-> 生成确定性风险报告 / dashboard
```

当前不把 RL 用于真车控制，不声称自动探索、自动清障、自动抓取或全自主机械臂处置。RL/Gazebo 相关内容作为仿真和策略设计补充材料。

## 2. 硬件与运行环境

### 2.1 车载端

- 主控平台：SpacemiT K1 Muse Pi Pro
- 系统：Bianbu LXQT v2.3.3
- 架构：RISC-V `riscv64`
- ROS：ROS2 Humble
- 车载用户：`soc`
- 车载仓库路径：`/home/soc/edge-ai-robot-k1`
- ROS 工作区：`/home/soc/edge-ai-robot-k1/ros2_ws`
- N10P 雷达工作区：`/home/soc/lslidar_ws`

常用环境加载：

```bash
source /opt/ros/humble/setup.bash
source ~/edge-ai-robot-k1/ros2_ws/install/setup.bash
source ~/lslidar_ws/install/setup.bash
```

### 2.2 主要硬件链路

| 模块 | 当前状态 | 关键接口 |
| --- | --- | --- |
| C30D Tank 履带底盘 | 已刷 ROS 固件，支持 Python 安全底盘节点 | `/dev/base_controller`, `/odom`, `/cmd_vel_guarded` |
| N10P 激光雷达 | ROS2 驱动已接入，发布 LaserScan | `/scan`, `frame_id=laser` |
| D435 RGB-D 相机 | 已验证 RGB/depth 采集和 YOLO 风险识别 | `/camera/color/image_raw`, depth topics |
| PWM 补光灯 | 已实现亮灭、亮度调节和自适应补光逻辑 | PWM7 / GPIO37 light tools |
| 总线舵机机械臂 | 已完成协议验证、no-load 动作和候选响应文件 | `/dev/arm_bus`, no-load only |

## 3. 软件架构

### 3.1 ROS2 包

仓库内 ROS2 包位于 `ros2_ws/src`：

| 包 | 职责 |
| --- | --- |
| `turn_on_wheeltec_robot` | WHEELTEC / C30D / Tank 底盘、雷达、SLAM、Nav2 guarded launch |
| `k1_sensor_event_adapter` | 雷达/相机事件适配、scan safety guard、低照度适配 |
| `k1_light_control` | GPIO/PWM 灯控、自适应补光、风险到灯光桥接 |
| `k1_risk_engine` | 规则化风险分级和风险事件输出 |
| `k1_event_logger` | 风险事件日志记录 |
| `k1_system_bringup` | 非机械臂基础 bringup |
| `wheeltec_robot_msg` | WHEELTEC 消息接口 |
| `serial_ros2` | 串口通信依赖 |

### 3.2 底盘安全链路

底盘不允许裸接 Nav2 或自动策略，当前统一通过安全链：

```text
/input_cmd_vel 或 Nav2 /cmd_vel_raw
-> scan_safety_guard_node
-> /cmd_vel_guarded
-> wheeltec_tank_base_safe.py
-> C30D
```

底盘 Python 节点能力：

- 连续发送 C30D ROS 速度帧
- 周期性发送 security keepalive
- 解析 C30D 回包
- 发布 `/odom`
- 发布 `odom -> base_footprint` TF
- 支持 STOP_REQUEST
- 支持停车反打和零速保持

### 3.3 建图链路

当前可用建图 launch：

```bash
ros2 launch turn_on_wheeltec_robot n10p_tank_mapping_safety_guard.launch.py \
  hard_stop_m:=0.30 \
  emergency_stop_m:=0.20 \
  slow_down_m:=0.80 \
  approach_stop_m:=0.80 \
  min_effective_forward:=0.08 \
  clear_max_linear:=0.30 \
  soft_max_linear:=0.30
```

推荐遥控输入：

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r /cmd_vel:=/input_cmd_vel
```

Tank 控制注意事项：

- `i` / `,`：前进 / 后退
- `j` / `l`：原地左转 / 右转
- `u` / `o`：弧线转向
- 不使用大写 `J` / `L`，Tank 不支持横移
- 速度保持在 guarded mapping 范围内，不盲目增大

### 3.4 Nav2 guarded 小目标链路

当前已完成 Nav2 guarded 小目标和 through-poses 小范围测试。Nav2 输出不直接给底盘：

```text
Nav2 /cmd_vel_raw
-> scan_safety_guard_node
-> /cmd_vel_guarded
-> wheeltec_tank_base_safe.py
```

相关 launch：

```bash
ros2 launch turn_on_wheeltec_robot n10p_tank_nav2_guarded.launch.py \
  use_amcl:=false \
  use_static_map_to_odom:=true
```

已验证工具：

- `tools/nav2_guarded_small_goal.py`
- `tools/nav2_guarded_through_poses.py`
- `tools/nav2_cancel_and_zero.py`

已验证模式：

- `line_2`
- `line_3`
- `micro_l`

结论：可作为受控导航验证，不作为 RRT/自动探索展示。

## 4. 当前已完成能力

### 4.1 低速安全建图

已完成 N10P + Tank odom + slam_toolbox 的低速建图链路。可保存地图并生成 PNG 预览。

关键地图证据位于：

- `maps/manual_mapping_snapshot_20260628_161624.*`
- `maps/manual_mapping_snapshot_20260628_163900.*`
- `maps/manual_mapping_snapshot_20260628_165000.*`
- `maps/manual_mapping_snapshot_20260628_170300.*`
- `maps/guarded_auto_micro_20260629_180401_final.*`

P4 guarded auto-mapping 冻结证据：

```text
evidence/p4_guarded_auto_mapping_20260629/
```

该证据包说明系统已完成一段短的 guarded 自动建图序列：

```text
F0.15 -> L30_arc -> F0.10 -> R30_arc -> F0.10
```

结果摘要：

- 保存地图数：6
- 最小 `front_p10`：0.50 m
- 最终 `front_p10`：1.771 m
- 累计前进：0.5978 m
- 横向变化：0.0943 m
- yaw 变化：-4.52 deg
- 地图尺寸：`128x222 -> 128x240`
- 每段动作结束均满足 `base_zero_ok=true`

### 4.2 D435 + YOLO 本地风险识别

当前推荐模型：

```text
models/risk_vision/yolov8n_480x640_q_truncated6_balanced_blockage03.onnx
provider: SpaceMITExecutionProvider
input: [1, 3, 480, 640]
camera: D435 640x480 at 15 FPS
```

相关工具：

- `tools/run_k1_yolo_ep_cli_light5.sh`
- `tools/run_k1_d435_yolo_realtime_display.py`
- `tools/run_prelim_remote_mapping_yolo_arm_demo.py`
- `tools/review_risk_detection_labels.py`
- `tools/benchmark_k1_d435_yolo_stream_vs_file.py`（当前为未提交新增文件）

当前风险识别定位：能在 K1 本地运行 D435 YOLO 风险识别，并生成 bbox、置信度、深度门控和风险事件输出。当前不声称泛化准确率，只作为受限演示场景中的本地视觉风险感知。

### 4.3 风险事件与地图点

`run_prelim_remote_mapping_yolo_arm_demo.py` 负责：

- 订阅 D435 RGB/depth/camera_info 和 `/odom`
- 运行本地 YOLO ONNX 推理
- 按类别和地图网格去重风险事件
- 发布 `/perception/mock_event`
- 发布 `/prelim_demo/alarm`
- 发布 `/prelim_demo/risk_event`
- 生成风险点、截图、报告和 dashboard

典型输出：

```text
dashboard.html
alarm_state.json
risk_events.jsonl
risk_event_index.json
risk_map_points.json
risk_map_snapshot.png
risk_control_report.md
episode_report.json
captures/<risk_event_id>/overlay.png
manual_arm_response_candidates/<risk_event_id>/manual_arm_response_candidate.json
```

### 4.4 低照度自适应补光

已完成：

- D435 RGB 亮度采样
- low_light 风险事件
- 风险到补光指令桥接
- PWM 灯光亮度控制
- 默认低功率策略，避免长时间高亮耗电

相关 ROS 节点：

- `adaptive_light_controller_node`
- `gpio37_light_node`
- `risk_light_bridge_node`

### 4.5 总线舵机机械臂 no-load 响应

已完成：

- 总线舵机协议审计
- 串口 dry-run
- 单舵机 no-load 测试
- 多舵机 no-load 样例动作
- map-gated no-load 候选动作

关键文档：

- `docs/arm_bus_servo_protocol_audit_20260630.md`
- `docs/arm_b2_b3_no_load_validation_20260630.md`
- `docs/arm_c1_map_gated_no_load_validation_20260630.md`

机械臂展示边界：

- 操作员确认后手动执行
- no-load
- 不接触、不抓取、不清障
- 不由 YOLO 自动触发

### 4.6 报告与本地认知层

当前已具备确定性报告生成工具：

- `tools/generate_llm_a_risk_report.py`
- `tools/run_local_llm_summary.py`
- `docs/llm_a_episode_report_input_contract_20260629.md`
- `docs/local_llm_report_interface_20260701.md`

当前定位：以结构化 episode report 为输入，生成风险摘要和处置建议。后续可替换为本地 LLM，但当前展示可先使用确定性模板，保证稳定。

## 5. 初赛演示推荐流程

### 5.1 推荐主线

```text
1. 启动 guarded mapping：N10P + Tank odom + slam_toolbox + scan guard
2. 启动 D435 640x480 RGB-D
3. 遥控小车低速建图，保存地图
4. 运行 K1 本地 YOLO 风险识别
5. 生成风险报警、风险点落图和 dashboard
6. 操作员确认风险点和安全位置
7. 手动执行机械臂 no-load 响应
8. 生成最终风险报告
```

### 5.2 K1 端启动命令

Guarded mapping：

```bash
cd /home/soc/edge-ai-robot-k1
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

D435：

```bash
source /opt/ros/humble/setup.bash
ros2 launch realsense2_camera rs_launch.py \
  depth_module.depth_profile:=640,480,30 \
  depth_module.infra_profile:=640,480,30 \
  rgb_camera.color_profile:=640,480,30
```

YOLO 风险闭环：

```bash
cd /home/soc/edge-ai-robot-k1
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash

sudo env PYTHONUNBUFFERED=1 python3 tools/run_prelim_remote_mapping_yolo_arm_demo.py \
  --provider spacemit \
  --model models/risk_vision/yolov8n_480x640_q_truncated6_balanced_blockage03.onnx \
  --imgsz 640 \
  --conf 0.15 \
  --iou 0.45 \
  --max-det 10 \
  --min-depth-m 0.20 \
  --max-depth-m 1.20 \
  --output-dir outputs/prelim_remote_mapping_yolo_arm_demo_v1/live_001
```

保存地图：

```bash
mkdir -p /home/soc/edge-ai-robot-k1/maps/prelim_remote_mapping
ros2 run nav2_map_server map_saver_cli \
  -f /home/soc/edge-ai-robot-k1/maps/prelim_remote_mapping/map_<run_id>
```

## 6. 关键证据路径

### 6.1 地图证据

- `maps/prelim_remote_mapping/map_20260703_095806.yaml`
- `maps/prelim_remote_mapping/map_20260703_095806_preview.png`
- `maps/prelim_remote_mapping/map_risk_live_20260703_103217.yaml`
- `maps/prelim_remote_mapping/map_risk_live_20260703_103217_preview.png`
- `evidence/p4_guarded_auto_mapping_20260629/maps/`

### 6.2 风险识别和报告输出

- `outputs/k1_pull/prelim_remote_mapping_yolo_arm_demo_v1/live_cpu_480_20260703_101632/`
- `outputs/k1_pull/prelim_remote_mapping_yolo_arm_demo_v1/live_cpu_480_20260703_101632_final/`
- `outputs/prelim_yolo_map_dryrun_v1/`

### 6.3 机械臂 no-load 证据

- `outputs/arm_b3_no_load_sample_sequence_v1/`
- `outputs/arm_c1_map_gated_no_load_once_v1/`
- `docs/arm_c1_map_gated_no_load_validation_20260630.md`

### 6.4 Step7-E2 备用演示

- `outputs/step7e2_guarded_motion_red_rule_flow_v1/e2_guarded_red_rule_arm_hw_fastdemo_002/`
- `outputs/step7e2_guarded_motion_red_rule_flow_v1/e2_guarded_red_rule_arm_hw_fastdemo_010/`
- `docs/step7e2_fastdemo_reproduction_20260630.md`

## 7. 当前 Git 状态

最新提交摘要：

```text
6aeb1e8 Update README.md
fdbb18c Add prelim remote mapping YOLO risk demo workflow
0463f2d feat(sim): refine tracked robot visual model
bf3f636 feat(vision): stabilize k1 d435 yolo ep realtime flow
4af3b02 feat(vision): add k1 yolo deployment and xquant calibration
8123418 docs(competition): add final component design plan
531bad5 feat(vision): add d435 risk detection backends and benchmark skeleton
```

当前工作区有一个未跟踪文件：

```text
tools/benchmark_k1_d435_yolo_stream_vs_file.py
```

如果要提交汇报文档，应单独 add 本文件，避免顺手提交未确认 benchmark 脚本。

## 8. 当前能力边界

可以明确展示：

- K1 端运行 ROS2 Humble 机器人系统
- N10P 雷达建图
- C30D Tank 底盘低速安全运动
- scan safety guard 保护链
- D435 RGB-D 输入
- K1 本地 YOLO 风险识别
- 风险报警、事件日志、风险点近似落图
- PWM 自适应补光
- 操作员确认后的机械臂 no-load 响应
- 确定性风险报告和 dashboard

不建议声称：

- 完全自主探索
- RRT 已上真车稳定运行
- Nav2 可无人看管导航
- YOLO 泛化检测准确率已充分验证
- 机械臂自动清障、抓取、搬运或接触式处置
- RL 控制真实车辆
- 本地 LLM 已稳定闭环控制机器人

## 9. 评奖叙事重点

建议将作品亮点表述为：

1. **RISC-V 边缘端闭环**：K1 上完成传感输入、风险识别、事件记录、报告生成和执行候选，不依赖云端。
2. **多源融合**：N10P 提供空间距离和建图，D435 提供 RGB-D 风险识别，odom 用于近似风险点落图。
3. **安全优先的运动链路**：底盘控制不裸接 Nav2，所有速度经过 scan safety guard 和 C30D 停车保护。
4. **低照度适应**：通过 D435 亮度采样和 PWM 补光，展示受限环境下的自适应感知。
5. **可解释输出**：所有风险事件有 bbox、depth、odom/map 近似位置、截图、JSONL 和报告。
6. **工程证据完整**：仓库中保留地图、日志、运行结果、验证文档和可复现实验脚本。

## 10. 后续计划

短期优先级：

1. 固化初赛视频脚本，按 README 当前主线录制。
2. 清理输出目录，挑选 3-5 个最能说明系统能力的证据文件。
3. 对 D435 YOLO 风险识别做少量人工复核，避免误检截图进入正式材料。
4. 将 `benchmark_k1_d435_yolo_stream_vs_file.py` 确认为可提交或移出工作区。
5. 准备 PPT：系统架构、硬件链路、K1 本地推理、guarded mapping、风险落图、no-load 响应、局限与后续。

中期增强：

1. 改进 D435 数据集和 YOLO 类别定义。
2. 增加更稳的风险点投影和地图坐标校验。
3. 将确定性报告生成替换或增强为本地 LLM 总结。
4. 完善 Gazebo / RL 材料，但保持“仿真辅助，不控真车”的边界。
5. 继续验证 guarded Nav2 小目标，不直接启用裸 RRT。

