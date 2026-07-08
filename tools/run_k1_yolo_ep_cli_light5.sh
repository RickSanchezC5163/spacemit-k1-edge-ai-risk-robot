#!/usr/bin/env bash
set -Eeuo pipefail

ROOT="${EDGE_AI_ROBOT_ROOT:-/home/soc/edge-ai-robot-k1}"
MODEL="$ROOT/models/risk_vision/yolov8n_480x640_q_truncated6_balanced_blockage03.onnx"
RUN_ID="${1:-cli_ep_480x640_truncated6_light5}"
OUT_DIR="$ROOT/outputs/k1_d435_yolo_realtime_v1/$RUN_ID"
LOG_DIR="$ROOT/logs"
LOG_FILE="$LOG_DIR/k1_yolo_${RUN_ID}.log"

mkdir -p "$OUT_DIR" "$LOG_DIR"

echo "[k1-yolo] root=$ROOT"
echo "[k1-yolo] model=$MODEL"
echo "[k1-yolo] output=$OUT_DIR"
echo "[k1-yolo] log=$LOG_FILE"
echo "[k1-yolo] This script does not start ROS, publish cmd_vel, or control the chassis/arm."

if [ ! -f "$MODEL" ]; then
  echo "[k1-yolo] ERROR: model not found: $MODEL" >&2
  exit 1
fi

echo "[k1-yolo] Enabling GPIO37 fill light at 5%."
sudo python3 /home/soc/tools/gpio37_light_smooth.py set 5 --start 0 --ramp 2 --hold -1

echo "[k1-yolo] Cleaning old YOLO/watch processes."
sudo pkill -f 'run_k1_d435_yolo_realtime_display.py' || true
pkill -f 'watch_k1_yolo_cli_log' || true
sleep 5

echo "[k1-yolo] Starting SpaceMIT EP CLI realtime inference. Ctrl-C to stop."
cd "$ROOT"
sudo env PYTHONUNBUFFERED=1 python3 tools/run_k1_d435_yolo_realtime_display.py \
  --provider spacemit \
  --model "$MODEL" \
  --width 640 --height 480 --fps 15 --imgsz 640 \
  --conf 0.15 --iou 0.45 --max-det 10 \
  --warmup-frames 90 \
  --cli-realtime \
  --cli-print-period-s 0.5 \
  --output-dir "$OUT_DIR" 2>&1 | tee "$LOG_FILE"
