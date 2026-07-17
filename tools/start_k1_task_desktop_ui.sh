#!/usr/bin/env bash
set -euo pipefail

REPO_DIR="${REPO_DIR:-/home/soc/edge-ai-robot-k1}"
PORT="${K1_TASK_DASHBOARD_PORT:-8780}"
URL="http://127.0.0.1:${PORT}/"
LOG_DIR="${REPO_DIR}/outputs/k1_task_desktop_ui"
START_BASE_STANDBY="${K1_TASK_START_BASE_STANDBY:-1}"
export K1_BATTERY_VOLTAGE_OFFSET_V="${K1_BATTERY_VOLTAGE_OFFSET_V:-0.83}"
export K1_BATTERY_WARN_V="${K1_BATTERY_WARN_V:-11.10}"

mkdir -p "${LOG_DIR}"
cd "${REPO_DIR}"

set +u
source /opt/ros/humble/setup.bash 2>/dev/null || true
source ros2_ws/install/setup.bash 2>/dev/null || true
set -u

base_stack_running() {
  ps -ef \
    | grep -E 'wheeltec_tank_base_safe.py|tank_base_safe.launch.py|n10p_tank_mapping_safety_guard.launch.py|n10p_tank_nav2_slam.launch.py' \
    | grep -v grep \
    | grep -q .
}

if [[ "${START_BASE_STANDBY}" == "1" ]]; then
  if ! base_stack_running; then
    setsid -f ros2 launch turn_on_wheeltec_robot tank_base_safe.launch.py \
      serial_port:=/dev/base_controller \
      cmd_vel_topic:=/k1_boot_zero_cmd \
      send_security_enable_on_start:=true \
      security_ply:=1 \
      send_rate:=20.0 \
      cmd_timeout:=0.25 \
      max_linear:=0.005 \
      max_angular:=0.03 \
      cruise_linear_limit:=0.005 \
      cruise_angular_limit:=0.03 \
      brake_duration:=0.30 \
      publish_tf:=true \
      > "${LOG_DIR}/base_standby.log" 2>&1 < /dev/null
  fi
fi

pkill -f "k1_task_desktop_dashboard.py.*--port ${PORT}" 2>/dev/null || true
sleep 0.5
setsid -f python3 tools/k1_task_desktop_dashboard.py \
  --repo-dir "${REPO_DIR}" \
  --host 0.0.0.0 \
  --port "${PORT}" \
  > "${LOG_DIR}/server.log" 2>&1 < /dev/null

for _ in $(seq 1 20); do
  if python3 - "${PORT}" <<'PY'
import http.client
import sys
port = int(sys.argv[1])
try:
    conn = http.client.HTTPConnection("127.0.0.1", port, timeout=0.5)
    conn.request("GET", "/api/status")
    ok = conn.getresponse().status == 200
    conn.close()
    raise SystemExit(0 if ok else 1)
except Exception:
    raise SystemExit(1)
PY
  then
    break
  fi
  sleep 0.3
done

export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/run/user/$(id -u)}"
export WAYLAND_DISPLAY="${WAYLAND_DISPLAY:-wayland-0}"
export DISPLAY="${DISPLAY:-:0}"

if command -v chromium-browser >/dev/null 2>&1; then
  setsid -f chromium-browser \
    --new-window \
    --start-maximized \
    --noerrdialogs \
    --disable-infobars \
    --ozone-platform=wayland \
    "${URL}" \
    > "${LOG_DIR}/chromium.log" 2>&1 < /dev/null || true
elif command -v xdg-open >/dev/null 2>&1; then
  setsid -f xdg-open "${URL}" > "${LOG_DIR}/xdg-open.log" 2>&1 < /dev/null || true
fi

echo "${URL}" > "${LOG_DIR}/url.txt"
echo "[k1-ui] ${URL}"
