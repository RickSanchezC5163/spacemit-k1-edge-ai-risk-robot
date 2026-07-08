#!/usr/bin/env bash
set -Eeo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_WS="$ROOT/ros2_ws"

pass() { echo "PASS: $*"; }
warn() { echo "WARN: $*"; }
info() { echo "INFO: $*"; }

topic_exists() {
  ros2 topic list 2>/dev/null | grep -Fx "$1" >/dev/null 2>&1
}

sample_hz() {
  local topic="$1"
  local seconds="${2:-5}"
  if topic_exists "$topic"; then
    info "Sampling $topic for about ${seconds}s"
    timeout "${seconds}s" ros2 topic hz "$topic" || true
  else
    warn "$topic is not present"
  fi
}

echo "K1 real-sensor non-arm safety test"
echo "root: $ROOT"

if [ -f /opt/ros/humble/setup.bash ]; then
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
  pass "found /opt/ros/humble/setup.bash"
else
  warn "/opt/ros/humble/setup.bash not found"
fi

if [ -f "$ROS_WS/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "$ROS_WS/install/setup.bash"
  pass "found $ROS_WS/install/setup.bash"
else
  warn "$ROS_WS/install/setup.bash not found; run colcon build first"
fi

pass "this script does not publish /cmd_vel"
pass "this script does not start arm or servo actions"
pass "this script does not enable real lamp output; use light_dry_run:=true"

echo
echo "== Current topics =="
ros2 topic list || true

echo
echo "== Sensor topic rate checks =="
sample_hz /scan 5
sample_hz /camera/color/image_raw 5

echo
echo "== Risk topic one-shot checks =="
for topic in /risk/current_event /risk/current_level /risk/recommended_action; do
  if topic_exists "$topic"; then
    info "Reading one message from $topic"
    timeout 5s ros2 topic echo --once "$topic" || true
  else
    warn "$topic is not present"
  fi
done

echo
echo "== Event logs =="
if [ -d "$ROOT/logs/events" ]; then
  ls -lh "$ROOT/logs/events" || true
  tail -n 10 "$ROOT"/logs/events/*.jsonl 2>/dev/null || true
else
  warn "$ROOT/logs/events does not exist yet"
fi

echo
echo "== Suggested real sensor adapter bring-up =="
cat <<'EOF'
cd ~/edge-ai-robot-k1
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
ros2 launch k1_system_bringup non_arm_bringup.launch.py \
  use_base:=false \
  use_lidar:=true \
  use_camera:=true \
  use_light:=true \
  light_dry_run:=true \
  use_risk_engine:=true \
  use_event_logger:=true \
  use_scan_event_adapter:=true \
  use_camera_low_light_adapter:=true
EOF

echo
echo "Done. Review WARN lines before enabling any moving subsystem."
