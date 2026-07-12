<div align="center">

# K1 边缘 AI 风险探测机器人

### 基于进迭时空 K1 的 SLAM-Frontier 自主探测、YOLO 风险识别、风险落图、MoveIt+RL 响应与本地 LLM 报告

[![License: PolyForm Noncommercial 1.0.0](https://img.shields.io/badge/License-PolyForm%20Noncommercial%201.0.0-orange.svg)](LICENSE)
[![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/)
[![SpaceMIT K1](https://img.shields.io/badge/SpaceMIT-K1%20MUSE%20Pi%20Pro-purple)](https://www.spacemit.com/)
[![ONNX Runtime](https://img.shields.io/badge/ONNX%20Runtime-SpaceMIT%20EP-orange)](https://onnxruntime.ai/)
[![SpaceMIT EP](https://img.shields.io/badge/SpaceMIT-Execution%20Provider-orange)](https://bianbu.spacemit.com/)
[![YOLOv8n](https://img.shields.io/badge/YOLOv8n-Risk%20Vision-red)](https://github.com/ultralytics/ultralytics)
[![Local LLM](https://img.shields.io/badge/LLM-Local%20Inference-black)](https://github.com/ggml-org/llama.cpp)

[项目报告](docs/report/spacemit_k1_edge_ai_robot_report.docx) |
[演示视频](demo/demo_clip_20260708_220330.mp4) |
[自主建图录屏](demo/recordings/) |
[部署说明](docs/k1_yolov8n_onnx_deployment_20260702.md) |
[模型目录](models/risk_vision/) |
[提交材料索引](SUBMISSION.md)

**语言 / Language**：中文 | [English](README.en.md)

</div>

## 项目简介

本仓库为进迭时空 K1 MUSE Pi Pro 边缘 AI 应用赛道的公开源码提交版。项目面向 GPS 拒止、通信受限、云端不可依赖的巡检场景，构建一套在端侧完成建图、感知、风险定位、处置规划和报告生成的移动机器人系统。

当前系统已经集成：

- 基于 ROS2、雷达、里程计和安全守护的遥控二维建图。
- Ubuntu ROS2 Humble + Gazebo + RViz 下的 SLAM-Frontier/RRT/Nav2 自主建图仿真。
- SolidWorks/SW2URDF 机械臂导入包，作为后续 MoveIt 与 leakage 处置仿真的独立模型。
- Intel RealSense D435 RGB-D 输入和 K1 本地 YOLOv8n ONNX 推理。
- 面向 `crack`、`corrosion`、`blockage`、`leakage` 的风险识别。
- 结合 confidence 与 depth 的自动报警和落图门限。
- 将 `bbox + depth + odom` 转换为地图坐标风险点。
- 浏览器 dashboard 展示 YOLO overlay、`infer_fps`、`front_min`、odom、报警和风险地图。
- SLAM-Frontier 自主探测与 RRT/A*/Nav2 路径生成接口。
- MoveIt 机械臂仿真配置、`link4_tip_link` 末端 TCP 标定与地面任务到达验证。
- MoveIt+RL 风格的机械臂处置动作候选与 no-load 安全响应。
- 本地 LLM CLI 根据结构化风险点生成最终风险报告。

系统闭环如下：

```text
未知环境起点 -> SLAM 根据雷达/里程计增量生成 occupancy grid
SLAM map -> 提取 frontier：已知空闲区 与 未知区 的边界
frontier -> 聚类并打分 -> 选择最优探索目标
探索目标 -> RRT/A*/Nav2 生成可行路径 -> 底盘安全守护执行
底盘执行 -> 地图更新后重复
D435 RGB-D -> YOLOv8n 本地推理 -> 风险事件
风险事件 + 深度 + 里程计 -> 地图风险点
地图风险点 -> MoveIt+RL 规划 -> 机械臂处置候选 + 人工处置任务
RRT/A*/Nav2 路径结果 + MoveIt+RL 规划结果 + 结构化风险点 -> 本地 LLM 风险报告
```

## 更新记录

- **2026-07-12**：完成机械臂 MoveIt 地面任务 TCP 到达验证；以 `link4_tip_link` 作为第四截末端 TCP，1000 个保守地面工作区目标全部规划并到达，最大 TCP 误差约 7.18 mm，平均约 2.52 mm。记录见 `docs/mechanical_arm_moveit_tcp_reach_20260712.md`，本地输出位于 `outputs/moveit_arm_visual_ground_tcp_direct_marker_1000/`。
- **2026-07-11**：完成 Ubuntu Humble 仿真自主建图录屏基线：Gazebo 履带底盘、N10P 雷达、D435、`slam_toolbox`、RRT frontier、Nav2、安全守护和 RViz 轨迹显示。
- **2026-07-08**：整理 GitHub 公开源码提交仓库，补充代码、报告、模型、样例地图、evidence 和演示材料。
- **2026-07-07**：根据实机场景完成 confidence + depth 联合门限调整。
- **2026-07-06**：完成演示链路：SLAM-Frontier 自主探测、D435 YOLO、风险地图、dashboard 和 LLM 报告。
- **2026-07-03**：完成 K1 D435 YOLO 部署流程，并验证 SpaceMIT Execution Provider 推理路径。
- **2026-06-30**：完成机械臂 no-load 安全响应和地图门控动作接口。

## 核心亮点

### K1 端侧 AI 推理

风险视觉模型在 K1 本地运行，通过 ONNX Runtime 的 SpaceMIT Execution Provider 执行量化 YOLOv8n 模型，不依赖云端 API。演示模型位于：

```text
models/risk_vision/yolov8n_480x640_q_truncated6_balanced_blockage03.onnx
```

### SLAM-Frontier 自主建图仿真

7 月 11 日完成的 Ubuntu 仿真链路用于展示自主探图能力：

```text
Gazebo tracked base + N10P lidar + D435-style camera
-> slam_toolbox occupancy grid
-> RRT frontier selection
-> Nav2 path following
-> scan safety guard
-> RViz map / laser / trajectory visualization
```

对应材料：

- 进度记录：`docs/autonomous_mapping_progress_20260711.md`
- 仿真包：`sim/tracked_robot_description/`
- 录屏：`demo/recordings/`
- 机械臂导入审计：`docs/mechanical_arm_1_import_audit_20260711.md`

现场记录指标：

- 风险视觉模型 mAP：**0.949**。
- D435 实时推理在 SpaceMIT EP 下约 **9-11 FPS**。
- dashboard 示例延迟约 **108 ms**。
- 风险识别、风险落图和报告生成均在本地链路中完成。

### 风险空间化

系统将通过门限的检测框转换为地图风险点，并将风险帧、风险 JSON、风险地图、dashboard 状态和最终报告保存在同一个 run 目录中，方便回放和复核。

实机场景中采用的自动落图门限：

```text
crack：    confidence >= 0.29，0.60 m <= depth <= 0.80 m
blockage： confidence >= 0.23，0.35 m <= depth <= 0.75 m
```

同类近邻合并用于避免连续帧重复保存同一物理风险点。

### 本地 LLM 报告

LLM 报告不是开放式聊天，而是从结构化风险点生成处置清单。最终报告面向人工巡检人员回答：

- 风险位于地图的哪个近似位置；
- 风险属于哪一类；
- 检测置信度和风险含义是什么；
- 人工需要执行什么处置动作。

风险处置映射示例：

| 标签 | 中文含义 | 处置动作 |
| --- | --- | --- |
| `crack` | 破损 / 裂缝 | 表面清理、裂缝修补、密封、复查 |
| `corrosion` | 锈蚀 / 腐蚀 | 除锈、防腐处理、管壁复查 |
| `blockage` | 障碍物 / 堵塞 | 移除障碍物、清理通道、复查通行 |
| `leakage` | 渗漏 / 漏液 | 查漏、封堵、干燥、复查 |

## 系统流程

```mermaid
flowchart LR
    A["未知环境起点"] --> B["SLAM occupancy grid"]
    B --> C["Frontier 提取/聚类/打分"]
    C --> D["RRT/A*/Nav2 可行路径"]
    D --> E["雷达安全守护"]
    E --> F["/cmd_vel_guarded"]
    F --> G["履带底盘执行"]
    G --> B
    B --> H["/map + /odom"]

    I["RealSense D435 RGB-D"] --> J["K1 端 YOLOv8n ONNX"]
    J --> K["风险检测"]
    K --> L["深度 + 里程计投影"]
    H --> L
    L --> M["风险地图"]
    M --> N["MoveIt+RL 机械臂规划"]
    N --> O["机械臂处置候选 + 人工处置任务"]
    D -- "路径结果" --> P["本地 LLM 风险报告"]
    N -- "规划结果" --> P
    M -- "结构化风险点" --> P
    K --> Q["Dashboard / 报警"]
    M --> Q
```

## 仓库特性

- **ROS2 建图与启动**：履带底盘、雷达、里程计、SLAM 和地图保存。
- **安全守护控制**：基于前向距离的遥控速度过滤与急停/慢速/放行逻辑。
- **D435 本地感知**：RGB-D 采集、YOLO overlay、深度门限和风险事件保存。
- **风险事件归档**：保存 overlay 帧、原始 RGB 帧、检测 JSON 和运行指标。
- **风险地图渲染**：风险点投影到地图坐标，并支持同类近邻合并。
- **Dashboard UI**：通过 K1 本地 HTTP 服务在 K1 屏幕或 Windows 浏览器查看。
- **机械臂响应接口**：no-load 安全验证、动作空间映射和处置候选。
- **本地报告生成**：CLI 形式展示 LLM token 流式输出和最终处置清单。
- **SLAM-Frontier/RRT/A*/Nav2 与 RL 扩展接口**：保留探索目标、路径生成、语义动作空间、primitive registry、训练和评估脚本。

## 快速开始

### 1. 准备 K1 环境

```bash
cd /home/soc/edge-ai-robot-k1
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
```

### 2. 启动 SLAM-Frontier 自主探测与安全守护

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

### 3. 启动 D435 YOLO 风险闭环

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

### 4. 打开 dashboard

```bash
cd outputs/prelim_remote_mapping_yolo_arm_demo_v1/live_demo
python3 -m http.server 8765 --bind 0.0.0.0
```

浏览器访问：

```text
http://<K1_IP>:8765/dashboard.html
http://<K1_IP>:8765/yolo_monitor.html
```

### 5. 结束并整理结果

```bash
bash tools/finalize_prelim_demo_k1.sh <run_dir>
```

运行目录中会保存地图文件、风险帧、风险 JSON、dashboard 页面、风险位置图和最终 LLM 报告。

## 主要代码入口

| 模块 | 文件 |
| --- | --- |
| D435 YOLO + 风险地图集成演示 | `tools/run_prelim_remote_mapping_yolo_arm_demo.py` |
| K1 SpaceMIT EP 启动脚本 | `tools/start_prelim_noarm_ep_k1.sh` |
| 演示收尾脚本 | `tools/finalize_prelim_demo_k1.sh` |
| 底盘安全守护 | `ros2_ws/src/k1_sensor_event_adapter/k1_sensor_event_adapter/scan_safety_guard_node.py` |
| 本地 LLM 报告 | `tools/run_local_llm_summary.py` |
| 风险地图总结 | `tools/run_risk_map_summary.py` |
| 机械臂安全 | `src/arm_safety.py` |
| 动作原语配置 | `configs/primitive_registry.yaml` |
| RL 语义策略 | `rl/train_semantic_ppo.py`、`rl/eval_semantic_policy.py` |

## 模型与数据

仓库包含一个可用于复现实机演示的轻量化模型文件：

```text
models/risk_vision/yolov8n_480x640_q_truncated6_balanced_blockage03.onnx
```

大规模原始数据集、长视频、ROS bag 和临时运行输出未纳入仓库。数据采集、模型训练、量化和阈值调整过程见：

- [风险视觉模型完成路径](docs/risk_vision_model_completion_path_20260707.md)
- [K1 YOLOv8n ONNX 部署说明](docs/k1_yolov8n_onnx_deployment_20260702.md)
- [XQuant YOLOv8 量化说明](docs/k1_xquant_yolov8_truncated_quantization_20260702.md)

## 项目结构

```text
.
├── ros2_ws/src/              # ROS2 节点、launch、底盘安全守护、传感器适配
├── tools/                    # K1 推理、风险落图、dashboard、报告生成、演示脚本
├── src/                      # 风险协议、机械臂安全校验等通用代码
├── configs/                  # 风险类别、动作语义、本地 LLM、机械臂安全配置
├── schemas/                  # 风险点、检测结果、动作候选、报告 JSON schema
├── rl/                       # 语义动作空间与仿真训练/评估脚本
├── models/risk_vision/       # 已量化 YOLOv8n ONNX 示例模型与量化报告
├── maps/                     # 遥控建图与风险地图样例
├── evidence/                 # 端到端验证记录样例
├── docs/                     # 设计文档、项目报告、硬件图片、部署记录
└── demo/                     # 演示视频样例或最终视频链接说明
```

## 文档索引

- [提交材料索引](SUBMISSION.md)
- [公开源码范围说明](docs/OPEN_SOURCE_SCOPE.md)
- [最终项目报告](docs/report/spacemit_k1_edge_ai_robot_report.docx)
- [遥控建图 + YOLO + 机械臂演示设计](docs/prelim_remote_mapping_yolo_arm_demo_20260703.md)
- [系统协议与整体逻辑](docs/k1_full_system_protocol_and_logic_20260630.md)
- [本地 LLM 报告接口](docs/local_llm_report_interface_20260701.md)
- [风险地图总结接口](docs/risk_map_summary_interface_20260702.md)
- [机械臂 SW2URDF 导入审计](docs/mechanical_arm_1_import_audit_20260711.md)

## 许可协议

本仓库采用 [PolyForm Noncommercial License 1.0.0](LICENSE) 授权。
允许学习、研究、测试、教育和非商用展示使用；未经额外书面授权，
不得用于商业用途、商业产品集成、商业部署或商业转授权。

## 引用

如果本项目对你的边缘 AI 机器人项目有帮助，可按如下形式引用：

```bibtex
@misc{k1_edge_ai_risk_robot_2026,
  title  = {K1 Edge AI Risk Inspection Robot},
  author = {K1 Edge AI Risk Inspection Robot Contributors},
  year   = {2026},
  note   = {SpaceMIT K1 MUSE Pi Pro edge AI risk inspection system}
}
```
