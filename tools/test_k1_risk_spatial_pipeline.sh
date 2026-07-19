#!/usr/bin/env bash
set -euo pipefail

# Stationary test only: synthetic odom/TF + D435 + YOLO, with no cmd_vel publisher.
REPO_DIR="${REPO_DIR:-/home/soc/edge-ai-robot-k1}"
RUN_S="${1:-18}"
OUTPUT_DIR="${2:-/tmp/k1_risk_spatial_pipeline_test}"
REALSENSE_PREFIX="${REALSENSE_PREFIX:-/home/soc/.local/realsense2-2.55.1}"

cd "${REPO_DIR}"
set +u
source /opt/ros/humble/setup.bash
[[ -f ros2_ws/install/setup.bash ]] && source ros2_ws/install/setup.bash
set -u
export PYTHONPATH="${REALSENSE_PREFIX}/lib/python3.12/site-packages:${REPO_DIR}:${PYTHONPATH:-}"
export LD_LIBRARY_PATH="${REALSENSE_PREFIX}/lib:${LD_LIBRARY_PATH:-}"

rm -rf "${OUTPUT_DIR}"
mkdir -p "${OUTPUT_DIR}"
PIDS=()
cleanup() {
  for pid in "${PIDS[@]}"; do
    kill -INT -- "-${pid}" 2>/dev/null || kill -INT "${pid}" 2>/dev/null || true
  done
  sleep 1
  for pid in "${PIDS[@]}"; do
    kill -- "-${pid}" 2>/dev/null || kill "${pid}" 2>/dev/null || true
  done
}
trap cleanup EXIT

setsid ros2 run tf2_ros static_transform_publisher \
  0 0 0 0 0 0 map odom >"${OUTPUT_DIR}/static_tf.log" 2>&1 &
PIDS+=("$!")
setsid ros2 topic pub -r 10 /odom nav_msgs/msg/Odometry \
  "{header: {frame_id: odom}, child_frame_id: base_footprint, pose: {pose: {orientation: {w: 1.0}}}}" \
  >"${OUTPUT_DIR}/odom_pub.log" 2>&1 &
PIDS+=("$!")
sleep 2

setsid python3 tools/run_prelim_remote_mapping_yolo_arm_demo.py \
  --provider spacemit \
  --frame-source realsense \
  --model models/risk_vision/yolov8n_480x640_q_truncated6_balanced_blockage03.onnx \
  --imgsz 640 \
  --conf 0.10 \
  --inference-period-s 1.0 \
  --pose-cache-duration-s 3.0 \
  --pose-sample-hz 10.0 \
  --pose-max-age-s 0.20 \
  --risk-fusion-distance-m 0.25 \
  --risk-fusion-time-s 2.0 \
  --risk-fusion-required 2 \
  --risk-fusion-window 3 \
  --map-write-policy approach_confirmed \
  --arm-response-mode disabled \
  --no-visuals \
  --output-dir "${OUTPUT_DIR}" \
  >"${OUTPUT_DIR}/node.log" 2>&1 &
NODE_PID="$!"
PIDS+=("${NODE_PID}")
sleep "${RUN_S}"

ps -p "${NODE_PID}" -o pid=,%cpu=,rss=,etime= || true
kill -INT -- "-${NODE_PID}" 2>/dev/null || kill -INT "${NODE_PID}" 2>/dev/null || true
sleep 2
[[ -f "${OUTPUT_DIR}/alarm_state.json" ]] && cat "${OUTPUT_DIR}/alarm_state.json"
[[ -f "${OUTPUT_DIR}/mission_state.json" ]] && cat "${OUTPUT_DIR}/mission_state.json"
