#!/usr/bin/env bash
set -euo pipefail

REPO_DIR="${REPO_DIR:-/home/soc/edge-ai-robot-k1}"
ACTION="${1:-status}"
RUNTIME_S="${K1_AUTONOMOUS_RUNTIME_S:-240}"
STATE_DIR="${REPO_DIR}/outputs/k1_autonomous_inspection"
STATUS_FILE="${STATE_DIR}/status.json"
CURRENT_RUN_FILE="${REPO_DIR}/.current_k1_autonomous_inspection_run"
REPORTS_ROOT="${REPO_DIR}/outputs/k1_autonomous_reports"
STARTER="${REPO_DIR}/tools/start_real_k1_rrt_nav2_mapping.sh"

mkdir -p "${STATE_DIR}"
cd "${REPO_DIR}"

write_status() {
  python3 - "${STATUS_FILE}" "$1" "$2" "${3:-}" <<'PY'
import json
import os
import sys
from datetime import datetime

path, phase, detail, run_dir = sys.argv[1:]
payload = {
    "schema_version": "k1_autonomous_inspection_status_v1",
    "phase": phase,
    "detail": detail,
    "run_dir": run_dir or None,
    "updated_at": datetime.now().astimezone().isoformat(timespec="seconds"),
    "mapping_mode": "SLAM + Frontier/RRT + Nav2 autonomous exploration",
}
temporary = path + ".tmp"
with open(temporary, "w", encoding="utf-8") as stream:
    json.dump(payload, stream, ensure_ascii=False, indent=2)
    stream.write("\n")
os.replace(temporary, path)
PY
}

light_off() {
  python3 tools/k1_pwm7_light.py off >/dev/null 2>&1 || true
}

safe_stop() {
  pkill -INT -f 'sim_rrt_frontier_explorer.py.*--send-nav2-action' 2>/dev/null || true
  bash "${STARTER}" zero >/dev/null 2>&1 || true
  pkill -INT -f 'adaptive_light_controller_node' 2>/dev/null || true
  pkill -INT -f 'pwm7_light_node' 2>/dev/null || true
  light_off
}

wait_for_map() {
  local deadline=$((SECONDS + 90))
  while (( SECONDS < deadline )); do
    if bash -lc "source /opt/ros/humble/setup.bash && source '${REPO_DIR}/ros2_ws/install/setup.bash' && timeout 3s ros2 topic echo /map --once >/dev/null 2>&1"; then
      return 0
    fi
    sleep 1
  done
  echo "SLAM /map did not become ready" >&2
  return 1
}

finalize_run() {
  local run_dir="$1"
  write_status "finalizing" "正在停止自主探索、保存地图并冻结风险证据" "${run_dir}"
  safe_stop
  sleep 2
  bash "${STARTER}" save-map "${run_dir}"

  pkill -INT -f 'run_prelim_remote_mapping_yolo_arm_demo.py' 2>/dev/null || true
  pkill -INT -f 'run_real_k1_risk_approach_from_event.py' 2>/dev/null || true
  sleep 4
  light_off
  bash "${STARTER}" clean >/dev/null 2>&1 || true

  write_status "reporting" "地图与风险证据已保存，正在生成本地 Qwen/HTML/PDF 报告" "${run_dir}"
  python3 tools/generate_k1_autonomous_report_bundle.py \
    --run-dir "${run_dir}" \
    --reports-root "${REPORTS_ROOT}"
  if ! python3 - "${REPORTS_ROOT}/${run_dir##*/}/report.json" <<'PY'
import json
import sys
payload = json.load(open(sys.argv[1], encoding="utf-8"))
raise SystemExit(0 if payload["autonomy"]["autonomous_mapping_supported"] is True else 1)
PY
  then
    write_status "failed" "报告已生成，但本轮自主建图证据不完整" "${run_dir}"
    return 9
  fi

  pkill -f 'python3 -m http.server 8780' 2>/dev/null || true
  nohup python3 -m http.server 8780 --bind 0.0.0.0 --directory "${REPORTS_ROOT}" \
    >"${STATE_DIR}/report_http.log" 2>&1 < /dev/null &
  write_status "complete" "自主建图、风险识别、自动补光、地图保存和本地报告全部完成" "${run_dir}"
}

case "${ACTION}" in
  run)
    if ! [[ "${RUNTIME_S}" =~ ^[0-9]+$ ]] || (( RUNTIME_S < 30 )); then
      echo "K1_AUTONOMOUS_RUNTIME_S must be an integer >= 30" >&2
      exit 2
    fi
    RUN_DIR="${REPO_DIR}/outputs/k1_autonomous_$(date +%Y%m%d_%H%M%S)"
    mkdir -p "${RUN_DIR}"
    printf '%s\n' "${RUN_DIR}" > "${CURRENT_RUN_FILE}"
    trap 'safe_stop' EXIT INT TERM
    write_status "starting" "正在启动SLAM、Nav2、D435、YOLO和PWM7补光" "${RUN_DIR}"
    if ! python3 tools/k1_pwm7_light.py status >/dev/null 2>&1; then
      write_status "failed" "PWM7硬件灯控未就绪，请先安装并验证k1-light-mode" "${RUN_DIR}"
      exit 8
    fi
    light_off
    bash "${STARTER}" clean >/dev/null 2>&1 || true

    nohup bash "${STARTER}" nav2-slam "${RUN_DIR}" \
      >"${RUN_DIR}/nav2_slam_guard.log" 2>&1 < /dev/null &
    echo "$!" > "${RUN_DIR}/nav2_slam.pid"
    wait_for_map

    nohup env YOLO_FRAME_SOURCE=ros YOLO_INFERENCE_PERIOD_S="${YOLO_INFERENCE_PERIOD_S:-0.35}" \
      bash "${STARTER}" risk-approach "${RUN_DIR}" \
      >"${RUN_DIR}/risk_stack_start.log" 2>&1 < /dev/null &
    sleep "${K1_AUTONOMOUS_VISION_START_WAIT_S:-12}"

    nohup bash -lc "source /opt/ros/humble/setup.bash; source '${REPO_DIR}/ros2_ws/install/setup.bash'; ros2 run k1_light_control pwm7_light_node --ros-args -p brightness_topic:=/light/brightness_cmd" \
      >"${RUN_DIR}/pwm7_light.log" 2>&1 < /dev/null &
    echo "$!" > "${RUN_DIR}/pwm7_light.pid"
    nohup bash -lc "source /opt/ros/humble/setup.bash; source '${REPO_DIR}/ros2_ws/install/setup.bash'; ros2 run k1_light_control adaptive_light_controller_node --ros-args -p image_topic:=/camera/camera/color/image_raw -p brightness_topic:=/light/brightness_cmd -p max_brightness:=25 -p stable_frames:=3 -p image_timeout_s:=2.0" \
      >"${RUN_DIR}/adaptive_light.log" 2>&1 < /dev/null &
    echo "$!" > "${RUN_DIR}/adaptive_light.pid"

    nohup env RRT_RUNTIME_S="${RUNTIME_S}" RRT_MAX_GOALS="${RRT_MAX_GOALS:-1000000}" \
      bash "${STARTER}" rrt-run-2m-unlimited "${RUN_DIR}" \
      >"${RUN_DIR}/rrt_unlimited.log" 2>&1 < /dev/null &
    RRT_PID=$!
    echo "${RRT_PID}" > "${RUN_DIR}/rrt_unlimited.pid"
    write_status "running" "机器人正在通过实时SLAM地图自主选择Frontier/RRT目标" "${RUN_DIR}"

    deadline=$((SECONDS + RUNTIME_S))
    while (( SECONDS < deadline )) && kill -0 "${RRT_PID}" 2>/dev/null; do
      sleep 2
    done
    if ! kill -0 "${RRT_PID}" 2>/dev/null; then
      set +e
      wait "${RRT_PID}"
      rrt_rc=$?
      set -e
      if [[ "${rrt_rc}" -ne 0 ]]; then
        write_status "failed" "Frontier/RRT自主探索进程异常退出（rc=${rrt_rc}）" "${RUN_DIR}"
        exit "${rrt_rc}"
      fi
    fi
    finalize_run "${RUN_DIR}"
    trap - EXIT INT TERM
    ;;
  finalize)
    [[ -s "${CURRENT_RUN_FILE}" ]] || { echo "no autonomous run is recorded" >&2; exit 3; }
    finalize_run "$(cat "${CURRENT_RUN_FILE}")"
    ;;
  stop)
    safe_stop
    write_status "stopped" "自主任务已安全停止，PWM7补光已关闭" "$(cat "${CURRENT_RUN_FILE}" 2>/dev/null || true)"
    ;;
  status)
    if [[ -s "${STATUS_FILE}" ]]; then
      cat "${STATUS_FILE}"
    else
      write_status "idle" "等待启动自主巡检" ""
      cat "${STATUS_FILE}"
    fi
    ;;
  *)
    echo "usage: $0 {run|finalize|stop|status}" >&2
    exit 2
    ;;
esac
