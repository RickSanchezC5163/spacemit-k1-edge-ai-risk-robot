# K1 Edge AI Risk Inspection Robot

本仓库为进迭时空 K1 MUSE Pi Pro 边缘 AI 应用赛道的开源提交版，主体任务是构建一套面向 GPS 拒止、通信受限场景的端侧风险探测机器人系统。系统在 K1 本地完成遥控建图、D435 RGB-D 感知、YOLOv8n 风险识别、风险点地图落图、本地 LLM 报告生成，并预留机械臂安全处置与 RRT/MoveIt/RL 策略验证接口。

## 功能链路

1. 遥控建图：`/input_cmd_vel` 经安全守护模块过滤后发布到 `/cmd_vel_guarded`，同步生成 `/map`。
2. 安全守护：读取 `/scan` 前向距离，按急停、慢速、放行三档限制底盘速度。
3. 本地视觉：K1 端 D435 图像输入，ONNX Runtime SpaceMIT EP 执行 YOLOv8n 量化模型推理。
4. 风险空间化：将 bbox、depth、camera info、odom 融合成地图坐标风险点，支持同类近距离合并。
5. 可视化展示：浏览器 dashboard 显示 YOLO overlay、infer_fps、front_min、odom、risk map 和报警信息。
6. 报告生成：本地 LLM 根据风险点、处置规则和任务上下文生成巡检总结报告。
7. 机械臂接口：按风险类型生成处置动作候选，当前以 no-load 安全响应和 MoveIt/RRT 规划接口为主。

## 目录结构

```text
.
├── ros2_ws/src/              # ROS2 节点、launch、底盘安全守护、传感器适配
├── tools/                    # K1 推理、风险落图、dashboard、报告生成、演示脚本
├── src/                      # 风险协议、机械臂安全校验等通用代码
├── configs/                  # 风险类别、动作语义、本地 LLM、机械臂安全配置
├── schemas/                  # 风险点、检测结果、动作候选、episode report JSON schema
├── rl/                       # 语义动作空间与仿真训练/评估脚本
├── models/risk_vision/       # 已量化 YOLOv8n ONNX 示例模型与量化报告
├── maps/                     # 遥控建图与风险地图样例
├── evidence/                 # 端到端验证记录样例
├── docs/                     # 设计文档、报告、硬件图片、部署记录
└── demo/                     # 演示视频占位/样例
```

## K1 端快速启动

```bash
cd /home/soc/edge-ai-robot-k1
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
```

启动遥控建图与安全守护：

```bash
ros2 launch turn_on_wheeltec_robot n10p_tank_mapping_safety_guard.launch.py \
  hard_stop_m:=0.10 \
  emergency_stop_m:=0.10 \
  slow_down_m:=0.30 \
  approach_stop_m:=0.20 \
  min_effective_forward:=0.05 \
  clear_max_linear:=0.30 \
  soft_max_linear:=0.10
```

启动 D435 YOLO 风险识别和风险落图：

```bash
sudo env PYTHONUNBUFFERED=1 python3 tools/run_prelim_remote_mapping_yolo_arm_demo.py \
  --provider spacemit \
  --model models/risk_vision/yolov8n_480x640_q_truncated6_balanced_blockage03.onnx \
  --imgsz 640 --conf 0.15 --iou 0.45 --max-det 10 \
  --min-depth-m 0.20 --max-depth-m 1.20 \
  --auto-risk-gates crack:0.29:0.60:0.80,blockage:0.23:0.35:0.75 \
  --dedup-map-grid-m 0.20 \
  --output-dir outputs/prelim_remote_mapping_yolo_arm_demo_v1/live_demo
```

启动本地 dashboard：

```bash
cd outputs/prelim_remote_mapping_yolo_arm_demo_v1/live_demo
python3 -m http.server 8765 --bind 0.0.0.0
```

浏览器访问：

```text
http://<K1_IP>:8765/dashboard.html
http://<K1_IP>:8765/yolo_monitor.html
```

## 关键代码入口

- `tools/run_prelim_remote_mapping_yolo_arm_demo.py`：D435 YOLO、深度融合、风险事件、落图、dashboard 状态写出。
- `ros2_ws/src/k1_sensor_event_adapter/k1_sensor_event_adapter/scan_safety_guard_node.py`：扫描雷达前向距离安全守护。
- `tools/start_prelim_noarm_ep_k1.sh`：K1 端 SpaceMIT EP 演示启动脚本。
- `tools/finalize_prelim_demo_k1.sh`：演示结束、地图保存、报告收尾脚本。
- `tools/run_local_llm_summary.py`：本地 LLM 风险报告生成。
- `src/arm_safety.py`：机械臂 no-load 与动作安全校验。
- `rl/train_semantic_ppo.py`、`rl/eval_semantic_policy.py`：仿真策略训练与评估入口。

## 模型与数据

仓库包含一个已量化的 YOLOv8n ONNX 示例模型：

```text
models/risk_vision/yolov8n_480x640_q_truncated6_balanced_blockage03.onnx
```

训练数据、原始采集数据和大规模运行输出未纳入 Git 仓库。相关采集、标注、量化与阈值调整流程见：

```text
docs/risk_vision_model_completion_path_20260707.md
docs/k1_yolov8n_onnx_deployment_20260702.md
docs/k1_xquant_yolov8_truncated_quantization_20260702.md
```

## 开源说明

本仓库按 MIT License 开源。硬件照片、项目报告和演示视频样例放在 `docs/` 与 `demo/` 下，最终提交前可将正式视频链接补入 `SUBMISSION.md`。
