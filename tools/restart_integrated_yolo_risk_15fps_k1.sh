#!/usr/bin/env bash
set -euo pipefail

REPO_DIR="${REPO_DIR:-/home/soc/edge-ai-robot-k1}"
RUN_DIR="${1:-}"
D435_NICE="${K1_D435_NICE:-8}"
YOLO_NICE="${K1_YOLO_NICE:-10}"
UI_NICE="${K1_UI_NICE:-15}"
FRAME_SOURCE="${K1_YOLO_FRAME_SOURCE:-ros}"
REALSENSE_PREFIX="${REALSENSE_PREFIX:-/home/soc/.local/realsense2-2.55.1}"

if [[ -z "${RUN_DIR}" ]]; then
  if [[ -s "${REPO_DIR}/.current_real_k1_rrt_nav2_run_dir" ]]; then
    RUN_DIR="$(cat "${REPO_DIR}/.current_real_k1_rrt_nav2_run_dir")"
  else
    RUN_DIR="${REPO_DIR}/outputs/real_k1_rrt_nav2_mapping_$(date +%Y%m%d_%H%M%S)"
  fi
fi

mkdir -p "${RUN_DIR}"
echo "${RUN_DIR}" > "${REPO_DIR}/.current_real_k1_rrt_nav2_run_dir"

cd "${REPO_DIR}"
set +u
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
set -u

echo "[integrated-yolo] run_dir=${RUN_DIR}"
echo "[integrated-yolo] stopping previous YOLO and D435 ROS users"
pkill -INT -f 'run_prelim_remote_mapping_yolo_arm_demo.py' 2>/dev/null || true
pkill -INT -f 'realsense2_camera.*rs_launch.py' 2>/dev/null || true
pkill -INT -f 'realsense2_camera_node' 2>/dev/null || true
pkill -INT -f 'http.server 8765' 2>/dev/null || true
sleep 3
pkill -f 'run_prelim_remote_mapping_yolo_arm_demo.py' 2>/dev/null || true
pkill -f 'realsense2_camera.*rs_launch.py' 2>/dev/null || true
pkill -f 'realsense2_camera_node' 2>/dev/null || true
pkill -KILL -f 'http.server 8765' 2>/dev/null || true

echo "[integrated-yolo] waiting for EP/USB resources"
sleep 8

if [[ "${FRAME_SOURCE}" == "realsense" ]]; then
  echo "[integrated-yolo] starting direct D435 SpaceMIT EP risk mapper"
  mkdir -p "${RUN_DIR}/yolo_risk"
  ln -sfn dashboard.html "${RUN_DIR}/yolo_risk/yolo_monitor.html"
  nohup env \
    PYTHONUNBUFFERED=1 \
    PYTHONPATH="${REALSENSE_PREFIX}/lib/python3.12/site-packages:${PYTHONPATH:-}" \
    LD_LIBRARY_PATH="${REALSENSE_PREFIX}/lib:${LD_LIBRARY_PATH:-}" \
    nice -n "${YOLO_NICE}" python3 tools/run_prelim_remote_mapping_yolo_arm_demo.py \
      --provider spacemit \
      --frame-source realsense \
      --realsense-width 640 \
      --realsense-height 480 \
      --realsense-fps 15 \
      --realsense-frame-slots 3 \
      --model models/risk_vision/yolov8n_480x640_q_truncated6_balanced_blockage03.onnx \
      --imgsz 640 \
      --conf 0.15 \
      --iou 0.45 \
      --max-det 10 \
      --inference-period-s "${K1_YOLO_INFERENCE_PERIOD_S:-0.10}" \
      --passive-close-confirm-distance-m 0.40 \
      --opencv-num-threads 1 \
      --ort-intra-op-threads 1 \
      --ort-inter-op-threads 1 \
      --min-depth-m 0.20 \
      --max-depth-m 1.20 \
      --map-frame map \
      --odom-frame odom \
      --tf-lookup-timeout-s 0.05 \
      --pose-cache-duration-s 3.0 \
      --pose-sample-hz 10.0 \
      --pose-max-age-s 0.20 \
      --alarm-topic /perception/risk_alarm \
      --auto-risk-gates 'crack:0.60:0.20:1.20,corrosion:0.60:0.20:1.20,leakage:0.60:0.20:1.20,blockage:0.60:0.20:1.20' \
      --approach-risk-gates 'crack:0.15:0.20:1.20,corrosion:0.15:0.20:1.20,leakage:0.15:0.20:1.20,blockage:0.15:0.20:1.20' \
      --dedup-map-grid-m 0.20 \
      --risk-fusion-distance-m 0.25 \
      --risk-fusion-time-s 2.0 \
      --risk-fusion-required 2 \
      --risk-fusion-window 3 \
      --arm-response-mode disabled \
      --map-write-policy approach_confirmed \
      --output-dir "${RUN_DIR}/yolo_risk" \
      > "${RUN_DIR}/yolo_direct_fast.log" 2>&1 < /dev/null &
  echo "$!" > "${RUN_DIR}/yolo.pid"
else
  echo "[integrated-yolo] starting D435 ROS at 640x480x15"
  nohup nice -n "${D435_NICE}" ros2 launch realsense2_camera rs_launch.py \
    depth_module.depth_profile:=640,480,15 \
    depth_module.infra_profile:=640,480,15 \
    rgb_camera.color_profile:=640,480,15 \
    > "${RUN_DIR}/d435_15fps.log" 2>&1 < /dev/null &

  sleep 8

  echo "[integrated-yolo] starting integrated SpaceMIT EP risk mapper"
  nohup nice -n "${YOLO_NICE}" bash tools/start_real_k1_rrt_nav2_mapping.sh yolo-ep "${RUN_DIR}" \
    > "${RUN_DIR}/yolo_ep_retry.log" 2>&1 < /dev/null &
  echo "$!" > "${RUN_DIR}/yolo.pid"
fi

nohup nice -n "${UI_NICE}" python3 -m http.server 8765 --bind 0.0.0.0 \
  --directory "${RUN_DIR}/yolo_risk" \
  > "${RUN_DIR}/http_8765.log" 2>&1 < /dev/null &
echo "$!" > "${RUN_DIR}/http_8765.pid"

echo "[integrated-yolo] started"
echo "[integrated-yolo] UI: http://192.168.43.40:8765/yolo_monitor.html"
