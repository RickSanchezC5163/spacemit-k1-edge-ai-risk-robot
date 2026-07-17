#!/usr/bin/env bash
set -euo pipefail

REPO_DIR="${REPO_DIR:-/home/soc/edge-ai-robot-k1}"
RUN_DIR="${1:-}"

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
sleep 3
pkill -f 'run_prelim_remote_mapping_yolo_arm_demo.py' 2>/dev/null || true
pkill -f 'realsense2_camera.*rs_launch.py' 2>/dev/null || true

echo "[integrated-yolo] waiting for EP/USB resources"
sleep 8

echo "[integrated-yolo] starting D435 ROS at 640x480x15"
nohup ros2 launch realsense2_camera rs_launch.py \
  depth_module.depth_profile:=640,480,15 \
  depth_module.infra_profile:=640,480,15 \
  rgb_camera.color_profile:=640,480,15 \
  > "${RUN_DIR}/d435_15fps.log" 2>&1 < /dev/null &

sleep 8

echo "[integrated-yolo] starting integrated SpaceMIT EP risk mapper"
nohup bash tools/start_real_k1_rrt_nav2_mapping.sh yolo-ep "${RUN_DIR}" \
  > "${RUN_DIR}/yolo_ep_retry.log" 2>&1 < /dev/null &

echo "[integrated-yolo] started"
echo "[integrated-yolo] UI: http://192.168.43.40:8765/yolo_monitor.html"
