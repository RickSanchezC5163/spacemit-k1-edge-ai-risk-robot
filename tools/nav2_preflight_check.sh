#!/usr/bin/env bash
set -u

FAILURES=0
WARNINGS=0

pass() {
  printf 'PASS: %s\n' "$1"
}

warn() {
  WARNINGS=$((WARNINGS + 1))
  printf 'WARN: %s\n' "$1"
}

fail() {
  FAILURES=$((FAILURES + 1))
  printf 'FAIL: %s\n' "$1"
}

check_pkg() {
  if ros2 pkg prefix "$1" >/dev/null 2>&1; then
    pass "ROS package found: $1"
  else
    fail "ROS package missing: $1"
  fi
}

if ! command -v ros2 >/dev/null 2>&1; then
  fail "ros2 command not found. Source /opt/ros/humble/setup.bash first."
else
  pass "ros2 command found"
fi

if [ -z "${AMENT_PREFIX_PATH:-}" ]; then
  warn "AMENT_PREFIX_PATH is empty; ROS overlays may not be sourced."
else
  pass "AMENT_PREFIX_PATH is set"
fi

for pkg in \
  nav2_bringup \
  nav2_amcl \
  nav2_map_server \
  nav2_bt_navigator \
  nav2_controller \
  nav2_planner \
  nav2_behaviors \
  nav2_velocity_smoother \
  nav2_waypoint_follower \
  nav2_navfn_planner \
  dwb_core \
  slam_toolbox \
  tf2_tools
do
  check_pkg "$pkg"
done

if [ -e /dev/base_controller ]; then
  pass "/dev/base_controller exists"
else
  fail "/dev/base_controller missing"
fi

if [ -e /dev/wheeltec_lidar ] || [ -e /dev/lslidar ] || [ -e /dev/ttyUSB0 ] || [ -e /dev/ttyACM0 ]; then
  pass "A likely lidar serial device exists"
else
  warn "No obvious lidar serial device found"
fi

CONFIG="$HOME/edge-ai-robot-k1/ros2_ws/src/turn_on_wheeltec_robot/config/nav2_n10p_tank_guarded_map.yaml"
LAUNCH="$HOME/edge-ai-robot-k1/ros2_ws/src/turn_on_wheeltec_robot/launch/n10p_tank_nav2_guarded.launch.py"
MAP="$HOME/edge-ai-robot-k1/maps/mapping_fixed_odom_20260628_085623.yaml"

if [ -f "$CONFIG" ]; then
  pass "Nav2 guarded map config exists"
else
  warn "Nav2 guarded map config not found at $CONFIG"
fi

if [ -f "$LAUNCH" ]; then
  pass "Nav2 guarded static-map launch exists"
else
  warn "Nav2 guarded static-map launch not found at $LAUNCH"
fi

if [ -f "$MAP" ]; then
  pass "Default Nav2 test map exists"
else
  warn "Default Nav2 test map not found at $MAP"
fi

PROCESS_PATTERN='nav2|bt_navigator|controller_server|planner_server|velocity_smoother|slam_toolbox|wheeltec_robot|lslidar'
if pgrep -af "$PROCESS_PATTERN" >/tmp/nav2_preflight_processes.txt 2>/dev/null; then
  warn "Possible residual navigation/mapping processes:"
  sed 's/^/  /' /tmp/nav2_preflight_processes.txt
else
  pass "No obvious residual Nav2/mapping processes"
fi
rm -f /tmp/nav2_preflight_processes.txt

printf '\nSummary: %d failure(s), %d warning(s)\n' "$FAILURES" "$WARNINGS"

if [ "$FAILURES" -gt 0 ]; then
  exit 1
fi

exit 0
