# 进迭时空赛题提交材料索引

## 仓库链接

待上传 GitHub 后填写：

```text
https://github.com/<your-account>/spacemit-k1-edge-ai-risk-robot
```

## 许可说明

本仓库采用 `PolyForm Noncommercial License 1.0.0`。源码公开用于学习、
研究、测试、教学和非商用展示；未经额外书面授权，不允许商业使用、
商业集成、商业部署或商业转授权。

## 赛题要求对应

| 要求 | 仓库位置 |
| --- | --- |
| 主体任务代码开源 | `ros2_ws/src/`、`tools/`、`src/` |
| K1 本地 AI 推理 | `tools/run_prelim_remote_mapping_yolo_arm_demo.py`、`models/risk_vision/` |
| 端侧模型部署 | `models/risk_vision/*.onnx`、`docs/k1_yolov8n_onnx_deployment_20260702.md` |
| 感知输入到执行输出完整链路 | `README.md`、`docs/prelim_remote_mapping_yolo_arm_demo_20260703.md` |
| 风险点地图化 | `tools/project_risk_point_to_map.py`、`tools/run_risk_map_summary.py` |
| 本地 LLM 报告 | `tools/run_local_llm_summary.py`、`configs/local_llm_config.yaml` |
| 机械臂安全响应 | `src/arm_safety.py`、`configs/arm_safety_config.json` |
| RRT/MoveIt/RL 验证接口 | `sim/tracked_robot_description/`、`tools/gz_sync_model_pose_from_odom.py`、`tools/ros2_twist_relay.py`、`rl/`、`configs/rl_action_space.yaml`、`configs/primitive_registry.yaml` |
| 项目报告 | `docs/report/spacemit_k1_edge_ai_robot_report.docx` |
| 设计文件/硬件图片 | `docs/hardware/`、`firmware/` |
| 演示视频 | `demo/demo_clip_20260708_220330.mp4`、`demo/recordings/` 或最终网盘/GitHub Release 链接 |

## 2026-07-11 自主建图补充材料

Ubuntu ROS2 Humble 仿真链路已经跑通：

```text
Gazebo 履带底盘 + N10P 雷达 + D435
-> slam_toolbox 建图
-> RRT frontier 选点
-> Nav2 导航
-> 雷达安全守护
-> RViz 地图/点云/轨迹显示
```

材料位置：

- 进度说明：`docs/autonomous_mapping_progress_20260711.md`
- 仿真包：`sim/tracked_robot_description/`
- 录屏文件：
  - `demo/recordings/录屏 2026年07月11日 00时59分50秒.webm`
  - `demo/recordings/录屏 2026年07月11日 01时19分55秒.webm`
  - `demo/recordings/录屏 2026年07月11日 01时42分10秒.webm`

## 建议提交邮件内容

```text
作品名称：K1 边缘 AI 风险探测机器人
赛道：题二：边缘 AI 应用
代码仓库：https://github.com/<your-account>/spacemit-k1-edge-ai-risk-robot
演示视频：<填写视频链接或 GitHub Release 链接>
项目报告：仓库 docs/report/spacemit_k1_edge_ai_robot_report.docx
主要说明：项目在 K1 MUSE Pi Pro 本地完成 D435 视觉输入、YOLOv8n 量化模型推理、风险点地图化、本地 LLM 报告生成和机械臂安全响应链路。
```

## 上传前检查

- `git status` 中不应出现 `outputs/`、`datasets/`、`*.bag`、`*.db3`、大体积临时视频。
- 正式演示视频建议放 GitHub Release、网盘或比赛指定平台，仓库内只保留压缩样例。
- 若 GitHub 提示文件超过 100 MB，不要强行提交，改用 Release 或外链。
- 提交前确认 README 中的 K1 IP、模型路径、演示命令与最终设备一致。
