#!/usr/bin/env bash
set -Eeo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT_DIR="$ROOT/logs"
OUT="$OUT_DIR/topic_health_$(date +%Y%m%d_%H%M%S).txt"
TOPICS=(
  "/scan"
  "/camera/color/image_raw"
  "/camera/depth/image_raw"
  "/risk/current_event"
  "/light/status"
)

mkdir -p "$OUT_DIR"

topic_exists() {
  ros2 topic list 2>/dev/null | grep -Fx "$1" >/dev/null 2>&1
}

sample_topic() {
  local topic="$1"
  echo
  echo "== $topic =="
  if topic_exists "$topic"; then
    timeout 5s ros2 topic hz "$topic" || true
  else
    echo "WARN: $topic is not present"
  fi
}

{
  echo "K1 topic health check"
  echo "timestamp: $(date --iso-8601=seconds)"
  echo "root: $ROOT"
  echo

  if [ -f /opt/ros/humble/setup.bash ]; then
    # shellcheck disable=SC1091
    source /opt/ros/humble/setup.bash
    echo "PASS: sourced /opt/ros/humble/setup.bash"
  else
    echo "WARN: /opt/ros/humble/setup.bash not found"
  fi

  if [ -f "$ROOT/ros2_ws/install/setup.bash" ]; then
    # shellcheck disable=SC1091
    source "$ROOT/ros2_ws/install/setup.bash"
    echo "PASS: sourced ros2_ws/install/setup.bash"
  else
    echo "WARN: ros2_ws/install/setup.bash not found"
  fi

  echo
  echo "== topic list =="
  ros2 topic list || true

  for topic in "${TOPICS[@]}"; do
    sample_topic "$topic"
  done
} | tee "$OUT"

echo "Saved topic health report to $OUT"
