#!/usr/bin/env bash
set -Eeuo pipefail

ROOT="${EDGE_AI_ROBOT_ROOT:-$HOME/edge-ai-robot-k1}"
ROS_WS="$ROOT/ros2_ws"
FAIL_COUNT=0
WARN_COUNT=0

pass() { echo "PASS: $*"; }
warn() { WARN_COUNT=$((WARN_COUNT + 1)); echo "WARN: $*"; }
fail() { FAIL_COUNT=$((FAIL_COUNT + 1)); echo "FAIL: $*"; }

source_setup() {
  local setup_file="$1"
  set +u
  # shellcheck disable=SC1090
  source "$setup_file"
  set -u
}

list_serial_devices() {
  local devices=()
  shopt -s nullglob
  devices=(/dev/ttyUSB* /dev/ttyACM* /dev/ttyCH343USB*)
  shopt -u nullglob

  if [ "${#devices[@]}" -eq 0 ]; then
    return 1
  fi

  ls -l "${devices[@]}"
}

echo "Mapping MVP preflight check"
echo "root: $ROOT"
echo "This script does not publish /cmd_vel."

echo "[1/6] ROS2 environment"
if [ -f /opt/ros/humble/setup.bash ]; then
  source_setup /opt/ros/humble/setup.bash
  pass "found /opt/ros/humble/setup.bash"
else
  fail "/opt/ros/humble/setup.bash not found"
fi

if [ -f "$ROS_WS/install/setup.bash" ]; then
  source_setup "$ROS_WS/install/setup.bash"
  pass "found $ROS_WS/install/setup.bash"
else
  warn "$ROS_WS/install/setup.bash not found; run colcon build first"
fi

if command -v ros2 >/dev/null 2>&1; then
  pass "ros2 command is available"
else
  fail "ros2 command is not available"
fi

echo "[2/6] Base controller"
if [ -e /dev/base_controller ]; then
  ls -l /dev/base_controller
  pass "/dev/base_controller exists"
else
  fail "/dev/base_controller not found"
fi

echo "[3/6] Lidar serial"
if [ -e /dev/wheeltec_lidar ]; then
  ls -l /dev/wheeltec_lidar
  pass "/dev/wheeltec_lidar exists"
else
  warn "/dev/wheeltec_lidar not found; checking tty candidates"
fi

if list_serial_devices; then
  pass "serial tty candidates are present"
else
  fail "no ttyUSB/ttyACM/ttyCH343USB serial devices found"
fi

echo "[4/6] Residual process check"
PROCESS_PATTERN='ros2|realsense|realsense2_camera|slam_toolbox|map_saver|lslidar|wheeltec_robot|wheeltec_tank_base|rviz2'
if pgrep -af "$PROCESS_PATTERN" 2>/dev/null | grep -v 'ros2-daemon' >/tmp/mapping_preflight_processes.txt; then
  cat /tmp/mapping_preflight_processes.txt
  warn "residual ROS/sensor/SLAM-related processes found; stop them before mapping"
else
  pass "no residual ROS/sensor/SLAM-related processes found"
fi

echo "[5/6] Maps directory"
if mkdir -p "$ROOT/maps"; then
  pass "maps directory ready: $ROOT/maps"
else
  fail "failed to create maps directory: $ROOT/maps"
fi

echo "[6/6] Safety reminder"
pass "preflight script never publishes motion commands"
echo "Before ground motion: lift/guard the chassis, then run tools/send_safe_zero_cmd.py."

echo "Summary: $FAIL_COUNT FAIL, $WARN_COUNT WARN"
if [ "$FAIL_COUNT" -gt 0 ]; then
  exit 1
fi
