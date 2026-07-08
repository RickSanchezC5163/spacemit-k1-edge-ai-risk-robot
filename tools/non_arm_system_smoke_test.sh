#!/usr/bin/env bash
set -Eeuo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_WS="$ROOT/ros2_ws"
RUN_BRINGUP="${RUN_BRINGUP:-0}"

pass() { echo "PASS: $*"; }
warn() { echo "WARN: $*"; }
fail() { echo "FAIL: $*"; }

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

echo "K1 non-arm system smoke test"
echo "root: $ROOT"

echo "[1/8] Checking current directory"
if [ -d "$ROS_WS/src" ] && [ -d "$ROOT/tools" ]; then
  pass "repository layout looks valid"
else
  fail "run this script from inside the edge-ai-robot-k1 repository"
fi

echo "[2/8] Checking ROS2 environment"
if [ -f /opt/ros/humble/setup.bash ]; then
  source_setup /opt/ros/humble/setup.bash
  pass "found /opt/ros/humble/setup.bash"
else
  fail "/opt/ros/humble/setup.bash not found"
fi

if [ -f "$ROS_WS/install/setup.bash" ]; then
  source_setup "$ROS_WS/install/setup.bash"
  pass "found ros2_ws/install/setup.bash"
else
  warn "$ROS_WS/install/setup.bash not found; run colcon build first"
fi

echo "[3/8] Checking base controller"
if [ -e /dev/base_controller ]; then
  ls -l /dev/base_controller
  pass "base controller device exists"
else
  warn "/dev/base_controller not present"
fi

echo "[4/8] Checking serial devices"
if list_serial_devices; then
  pass "serial device candidate exists"
else
  warn "no ttyUSB/ttyACM/ttyCH343USB serial device found"
fi

echo "[5/8] Checking USB devices and D435 hints"
if command -v lsusb >/dev/null 2>&1; then
  lsusb || true
  if lsusb | grep -Ei "RealSense|Intel" >/dev/null; then
    pass "RealSense/Intel-like USB device detected"
  else
    warn "no RealSense/Intel-like USB device detected"
  fi
else
  warn "lsusb command not found"
fi

echo "[6/8] Confirming safety constraints"
pass "this script does not publish /cmd_vel"
pass "this script does not start arm or servo actions"

echo "[7/8] Suggested manual bring-up command"
cat <<'EOF'
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
ros2 launch k1_system_bringup non_arm_bringup.launch.py \
  use_base:=false \
  use_lidar:=false \
  use_camera:=false \
  use_light:=true \
  light_dry_run:=true \
  use_risk_engine:=true \
  use_event_logger:=true
EOF

echo "[8/8] Suggested manual mock event command"
cat <<'EOF'
python3 tools/publish_mock_event.py --type soft_obstacle --distance 0.8 --confidence 0.9 --count 1
ros2 topic echo --once /risk/current_level
ros2 topic echo --once /risk/recommended_action
EOF

if [ "$RUN_BRINGUP" = "1" ]; then
  echo "RUN_BRINGUP=1 set; starting static non-arm bring-up with base/lidar/camera disabled."
  cd "$ROOT"
  ros2 launch k1_system_bringup non_arm_bringup.launch.py \
    use_base:=false \
    use_lidar:=false \
    use_camera:=false \
    use_light:=true \
    light_dry_run:=true \
    use_risk_engine:=true \
    use_event_logger:=true
else
  echo "Set RUN_BRINGUP=1 to launch the static non-arm stack from this script."
fi
