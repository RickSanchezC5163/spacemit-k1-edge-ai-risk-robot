#!/usr/bin/env bash
set -Eeuo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_WS="$ROOT/ros2_ws"
LOG_DIR="$ROOT/logs/tests"
STAMP="$(date +%Y%m%d_%H%M%S)"
LAUNCH_LOG="$LOG_DIR/non_arm_static_launch_$STAMP.log"
TOPICS_LOG="$LOG_DIR/non_arm_static_topics_$STAMP.txt"
LEVEL_LOG="$LOG_DIR/non_arm_static_risk_level_$STAMP.txt"
ACTION_LOG="$LOG_DIR/non_arm_static_risk_action_$STAMP.txt"
LIGHT_LOG="$LOG_DIR/non_arm_static_light_status_$STAMP.txt"

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

echo "K1 non-arm static stack check"
echo "root: $ROOT"
echo "launch log: $LAUNCH_LOG"

source_setup /opt/ros/humble/setup.bash
source_setup "$ROS_WS/install/setup.bash"

cd "$ROOT"
ros2 launch k1_system_bringup non_arm_bringup.launch.py \
  use_base:=false \
  use_lidar:=false \
  use_camera:=false \
  use_light:=true \
  light_dry_run:=true \
  use_risk_engine:=true \
  use_event_logger:=true >"$LAUNCH_LOG" 2>&1 &
LAUNCH_PID=$!

sleep 8

echo "=== topics ==="
ros2 topic list | sort | tee "$TOPICS_LOG"

echo "=== waiting subscribers ==="
timeout 12 ros2 topic echo --once /risk/current_level >"$LEVEL_LOG" 2>&1 &
LEVEL_PID=$!
timeout 12 ros2 topic echo --once /risk/recommended_action >"$ACTION_LOG" 2>&1 &
ACTION_PID=$!
timeout 8 ros2 topic echo --once /light/status >"$LIGHT_LOG" 2>&1 &
LIGHT_PID=$!

sleep 2

echo "=== publishing mock event ==="
python3 tools/publish_mock_event.py \
  --type soft_obstacle \
  --distance 0.8 \
  --confidence 0.9 \
  --count 5 \
  --interval 0.5

wait "$LEVEL_PID" || true
wait "$ACTION_PID" || true
wait "$LIGHT_PID" || true

echo "=== risk level ==="
cat "$LEVEL_LOG"

echo "=== risk action ==="
cat "$ACTION_LOG"

echo "=== light status ==="
cat "$LIGHT_LOG"

echo "=== latest event log ==="
LATEST_EVENT_LOG="$(find logs/events -type f -name 'events_*.jsonl' -printf '%T@ %p\n' 2>/dev/null | sort -n | tail -1 | cut -d' ' -f2-)"
if [ -n "$LATEST_EVENT_LOG" ]; then
  echo "$LATEST_EVENT_LOG"
  tail -n 3 "$LATEST_EVENT_LOG"
else
  echo "WARN: no event log found"
fi

echo "=== launch log tail ==="
tail -n 80 "$LAUNCH_LOG"
