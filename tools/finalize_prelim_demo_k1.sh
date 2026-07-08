#!/usr/bin/env bash
set -euo pipefail

RUN_DIR="${1:-/home/soc/edge-ai-robot-k1/outputs/prelim_remote_mapping_yolo_arm_demo_v1/live_20260706_smallmap}"
PORT="${PRELIM_DEMO_UI_PORT:-8765}"
URL="http://127.0.0.1:${PORT}/dashboard.html"

cd /home/soc/edge-ai-robot-k1
set +u
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
set -u

mkdir -p "$RUN_DIR"

echo "[finalize] publishing zero cmd_vel through /input_cmd_vel"
timeout 2s ros2 topic pub -r 10 /input_cmd_vel geometry_msgs/msg/Twist '{}' >/tmp/prelim_demo_zero_stop.log 2>&1 || true

echo "[finalize] asking live risk runner to finalize, if present"
if pgrep -f "run_prelim_remote_mapping_yolo_arm_demo.py.*${RUN_DIR}" >/dev/null 2>&1; then
  pkill -INT -f "run_prelim_remote_mapping_yolo_arm_demo.py.*${RUN_DIR}" || true
  sleep 3
fi

if pgrep -f "run_prelim_remote_mapping_yolo_arm_demo.py.*${RUN_DIR}" >/dev/null 2>&1; then
  echo "[finalize] runner still alive after SIGINT; leaving it running"
fi

echo "[finalize] report files"
for path in \
  "$RUN_DIR/dashboard.html" \
  "$RUN_DIR/risk_control_report.md" \
  "$RUN_DIR/episode_report.json" \
  "$RUN_DIR/risk_event_index.json" \
  "$RUN_DIR/risk_map_points.json"; do
  if [ -f "$path" ]; then
    ls -lh "$path"
  else
    echo "missing: $path"
  fi
done

echo "[finalize] starting dashboard server on 0.0.0.0:${PORT}"
pkill -f "python3 -m http.server ${PORT}" >/dev/null 2>&1 || true
cd "$RUN_DIR"
setsid -f python3 -m http.server "$PORT" --bind 0.0.0.0 </dev/null >/tmp/prelim_demo_ui_${PORT}.log 2>&1
sleep 1

if command -v python3 >/dev/null 2>&1; then
  python3 - <<PY
import urllib.request
url = "${URL}"
with urllib.request.urlopen(url, timeout=3) as response:
    print(f"[finalize] dashboard_http_status={response.status} url={url}")
PY
fi

echo "[finalize] opening dashboard on K1 display"
if command -v chromium-browser >/dev/null 2>&1; then
  DISPLAY="${DISPLAY:-:0}" XAUTHORITY="${XAUTHORITY:-/home/soc/.Xauthority}" \
    setsid -f chromium-browser --new-window --start-maximized "$URL" \
    </dev/null >/tmp/prelim_demo_chromium.log 2>&1 || true
elif command -v xdg-open >/dev/null 2>&1; then
  DISPLAY="${DISPLAY:-:0}" XAUTHORITY="${XAUTHORITY:-/home/soc/.Xauthority}" \
    setsid -f xdg-open "$URL" </dev/null >/tmp/prelim_demo_xdg_open.log 2>&1 || true
else
  echo "[finalize] no graphical browser found; open $URL on the K1 display manually"
fi

echo "[finalize] Windows URL: http://192.168.43.40:${PORT}/dashboard.html"
