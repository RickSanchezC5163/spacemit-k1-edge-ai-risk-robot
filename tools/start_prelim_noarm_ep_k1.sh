#!/usr/bin/env bash
set -eo pipefail

RUN_DIR="${1:-/home/soc/edge-ai-robot-k1/outputs/prelim_remote_mapping_yolo_arm_demo_v1/live_20260706_noarm_194538}"

cd /home/soc/edge-ai-robot-k1

set +u
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
set -u

mkdir -p "$RUN_DIR"

echo "[ep] stopping previous prelim risk runner"
pkill -INT -f 'run_prelim_remote_mapping_yolo_arm_demo.py' 2>/dev/null || true
sleep 2
pkill -f 'run_prelim_remote_mapping_yolo_arm_demo.py' 2>/dev/null || true

echo "[ep] waiting for SpaceMIT EP resources to settle"
sleep 8

echo "[ep] starting no-arm prelim risk runner with SpaceMITExecutionProvider"
echo "[ep] run_dir=$RUN_DIR"

exec python3 tools/run_prelim_remote_mapping_yolo_arm_demo.py \
  --provider spacemit \
  --model models/risk_vision/yolov8n_480x640_q_truncated6_balanced_blockage03.onnx \
  --imgsz 640 \
  --conf 0.15 \
  --iou 0.45 \
  --max-det 10 \
  --min-depth-m 0.20 \
  --max-depth-m 1.20 \
  --auto-risk-gates crack:0.29:0.60:0.80,blockage:0.23:0.35:0.75 \
  --dedup-map-grid-m 0.20 \
  --arm-response-mode disabled \
  --output-dir "$RUN_DIR"
