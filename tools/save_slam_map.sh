#!/usr/bin/env bash
set -Eeuo pipefail

ROOT="${EDGE_AI_ROBOT_ROOT:-$HOME/edge-ai-robot-k1}"
MAP_DIR="$ROOT/maps"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUTPUT_PREFIX="${1:-$MAP_DIR/test_map_$STAMP}"

source_setup() {
  local setup_file="$1"
  set +u
  # shellcheck disable=SC1090
  source "$setup_file"
  set -u
}

mkdir -p "$MAP_DIR"

if [ -f /opt/ros/humble/setup.bash ]; then
  source_setup /opt/ros/humble/setup.bash
else
  echo "FAIL: /opt/ros/humble/setup.bash not found" >&2
  exit 1
fi

if [ -f "$ROOT/ros2_ws/install/setup.bash" ]; then
  source_setup "$ROOT/ros2_ws/install/setup.bash"
fi

if ! command -v ros2 >/dev/null 2>&1; then
  echo "FAIL: ros2 command not found after sourcing ROS environment" >&2
  exit 1
fi

if ! ros2 pkg executables nav2_map_server 2>/dev/null | grep -q 'map_saver_cli'; then
  cat >&2 <<'EOF'
FAIL: nav2_map_server map_saver_cli is not available.
Install/check the ROS 2 Nav2 map server package, for example:
  sudo apt install ros-humble-nav2-map-server
EOF
  exit 1
fi

echo "Saving map to prefix: $OUTPUT_PREFIX"
ros2 run nav2_map_server map_saver_cli -f "$OUTPUT_PREFIX"

