#!/usr/bin/env bash
set -Eeuo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_WS="$ROOT/ros2_ws"
LOG_DIR="$ROOT/logs/tests"
STAMP="$(date +%Y%m%d_%H%M%S)"
LAUNCH_LOG="$LOG_DIR/d435_static_launch_$STAMP.log"
TOPICS_LOG="$LOG_DIR/d435_static_topics_$STAMP.txt"
ENABLE_POINTCLOUD="${ENABLE_POINTCLOUD:-0}"

source_setup() {
  local setup_file="$1"
  set +u
  # shellcheck disable=SC1090
  source "$setup_file"
  set -u
}

cleanup() {
  if [ -n "${LAUNCH_PID:-}" ] && kill -0 "$LAUNCH_PID" 2>/dev/null; then
    kill "$LAUNCH_PID" 2>/dev/null || true
    wait "$LAUNCH_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT

mkdir -p "$LOG_DIR"

echo "D435 static check"
echo "root: $ROOT"
echo "launch log: $LAUNCH_LOG"
echo "enable pointcloud: $ENABLE_POINTCLOUD"

source_setup /opt/ros/humble/setup.bash
source_setup "$ROS_WS/install/setup.bash"

cd "$ROOT"
ros2 launch k1_system_bringup non_arm_bringup.launch.py \
  use_base:=false \
  use_lidar:=false \
  use_camera:=true \
  camera_enable_depth:=true \
  enable_pointcloud:="$ENABLE_POINTCLOUD" \
  use_light:=true \
  light_dry_run:=true \
  use_risk_engine:=false \
  use_event_logger:=false >"$LAUNCH_LOG" 2>&1 &
LAUNCH_PID=$!

sleep 12

echo "=== camera topics ==="
ros2 topic list | sort | tee "$TOPICS_LOG" | grep -E 'camera|points|depth|color' || true

echo "=== color hz ==="
timeout 10 ros2 topic hz /camera/camera/color/image_raw || true

echo "=== depth hz ==="
timeout 10 ros2 topic hz /camera/camera/depth/image_rect_raw || true

echo "=== pointcloud hz ==="
if [ "$ENABLE_POINTCLOUD" = "1" ] || [ "$ENABLE_POINTCLOUD" = "true" ]; then
  timeout 10 ros2 topic hz /camera/camera/depth/color/points || true
else
  echo "SKIP: set ENABLE_POINTCLOUD=1 to test D435 point cloud."
fi

echo "=== camera info ==="
timeout 5 ros2 topic echo --once /camera/camera/color/camera_info || true

echo "=== launch log tail ==="
tail -n 120 "$LAUNCH_LOG"
