#!/usr/bin/env bash
set -euo pipefail

PORT="${1:-/dev/ttyUSB0}"
BAUD="${2:-9600}"

cd /home/soc/edge-ai-robot-k1
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash

echo "[voice] starting SYN6288 voice bridge port=${PORT} baud=${BAUD}"
python3 tools/prelim_voice_event_bridge.py \
  --port "${PORT}" \
  --baud "${BAUD}" \
  --alarm-topic /perception/risk_alarm \
  --cue-topic /prelim_demo/voice_cue \
  --volume 12 \
  --background-volume 15 \
  --speed 5 \
  --cooldown-s 4.0 \
  --say-startup
