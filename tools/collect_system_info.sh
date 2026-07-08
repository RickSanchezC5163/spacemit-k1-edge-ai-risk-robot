#!/usr/bin/env bash
set -Eeuo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
mkdir -p "$ROOT/logs"
OUT="$ROOT/logs/system_info_$(date +%Y%m%d_%H%M%S).txt"

{
  echo "== uname -a =="
  uname -a || true
  echo
  echo "== /etc/os-release =="
  cat /etc/os-release || true
  echo
  echo "== whoami / pwd =="
  whoami || true
  pwd || true
  echo
  echo "== Python =="
  python3 --version || true
  echo
  echo "== ROS2 =="
  if command -v ros2 >/dev/null 2>&1; then
    ros2 --help | head -40 || true
  else
    echo "ros2 command not found"
  fi
  echo
  echo "== lsusb =="
  lsusb || true
  echo
  echo "== serial devices =="
  ls -l /dev/base_controller /dev/wheeltec_lidar /dev/ttyUSB* /dev/ttyACM* /dev/ttyCH343USB* 2>/dev/null || true
  echo
  echo "== disk =="
  df -h || true
  echo
  echo "== memory =="
  free -h || true
  echo
  echo "== top processes =="
  ps -eo pid,ppid,comm,%cpu,%mem --sort=-%cpu | head -25 || true
} | tee "$OUT"

echo "Saved system info to $OUT"
