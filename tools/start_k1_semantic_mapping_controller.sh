#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="${REPO_DIR:-$(cd "${SCRIPT_DIR}/.." && pwd)}"
PORT="${K1_SEMANTIC_CONTROLLER_PORT:-8769}"
RUNTIME_DIR="${K1_SEMANTIC_CONTROLLER_RUNTIME_DIR:-${REPO_DIR}/outputs/semantic_mapping_controller_current}"
PID_FILE="${RUNTIME_DIR}/server.pid"
SERVER_LOG="${RUNTIME_DIR}/server.log"
EVENT_LOG="${RUNTIME_DIR}/motion_events.jsonl"
SLAM_PID_FILE="${RUNTIME_DIR}/slam_toolbox.pid"
SLAM_LOG="${RUNTIME_DIR}/slam_toolbox.log"
SLAM_PARAMS="${K1_SEMANTIC_SLAM_PARAMS:-${REPO_DIR}/ros2_ws/src/turn_on_wheeltec_robot/config/slam_toolbox_n10p_tank.yaml}"
LIDAR_SETUP="${K1_LIDAR_SETUP:-/home/soc/lslidar_ws/install/setup.bash}"
LIDAR_PID_FILE="${RUNTIME_DIR}/lidar.pid"
LIDAR_LOG="${RUNTIME_DIR}/lidar.log"
LASER_TF_PID_FILE="${RUNTIME_DIR}/base_to_laser_tf.pid"
LASER_TF_LOG="${RUNTIME_DIR}/base_to_laser_tf.log"
BASE_PID_FILE="${RUNTIME_DIR}/base.pid"
BASE_LOG="${RUNTIME_DIR}/base.log"

mkdir -p "${RUNTIME_DIR}"

is_running() {
  [[ -f "${PID_FILE}" ]] || return 1
  local pid
  pid="$(tr -cd '0-9' < "${PID_FILE}")"
  [[ -n "${pid}" ]] || return 1
  kill -0 "${pid}" 2>/dev/null
}

pid_file_running() {
  local file="$1"
  [[ -f "${file}" ]] || return 1
  local pid
  pid="$(tr -cd '0-9' < "${file}")"
  [[ -n "${pid}" ]] || return 1
  kill -0 "${pid}" 2>/dev/null
}

source_ros() {
  cd "${REPO_DIR}"
  source /opt/ros/humble/setup.bash
  if [[ -f "${LIDAR_SETUP}" ]]; then
    source "${LIDAR_SETUP}"
  fi
  if [[ -f "${REPO_DIR}/install/setup.bash" ]]; then
    source "${REPO_DIR}/install/setup.bash"
  fi
  if [[ -f "${REPO_DIR}/ros2_ws/install/setup.bash" ]]; then
    source "${REPO_DIR}/ros2_ws/install/setup.bash"
  fi
}

scan_has_publisher() {
  local count
  count="$(ros2 topic info /scan 2>/dev/null | sed -n 's/^Publisher count: //p')"
  [[ "${count:-0}" -gt 0 ]]
}

raw_cmd_has_subscriber() {
  local count
  count="$(ros2 topic info /cmd_vel_raw 2>/dev/null | sed -n 's/^Subscription count: //p')"
  [[ "${count:-0}" -gt 0 ]]
}

audit_raw_cmd_publishers() {
  local attempt info count names
  for attempt in 1 2 3; do
    if info="$(ros2 topic info -v /cmd_vel_raw 2>&1)"; then
      count="$(sed -n 's/^Publisher count: //p' <<<"${info}" | head -n 1)"
      names="$(awk '
        /^Publisher count:/ { publishers = 1; next }
        /^Subscription count:/ { publishers = 0 }
        publishers && /^[[:space:]]*Node name:/ {
          sub(/^[[:space:]]*Node name:[[:space:]]*/, "")
          print
        }
      ' <<<"${info}" | paste -sd, -)"
      if [[ "${count}" =~ ^[0-9]+$ ]]; then
        echo "/cmd_vel_raw publisher audit attempt ${attempt}/3: count=${count} nodes=${names:-<none>}"
        if [[ "${count}" -eq 0 ]]; then
          return 0
        fi
      else
        echo "/cmd_vel_raw publisher audit attempt ${attempt}/3: publisher count unavailable" >&2
      fi
    else
      echo "/cmd_vel_raw publisher audit attempt ${attempt}/3 failed: ${info}" >&2
    fi
    [[ "${attempt}" -eq 3 ]] || sleep 1
  done
  echo "refusing to start: /cmd_vel_raw publisher uniqueness was not established" >&2
  return 1
}

start_base() {
  source_ros
  if raw_cmd_has_subscriber; then
    echo "/cmd_vel_raw already has a base subscriber; reusing external base"
    return
  fi

  pkill -INT -f 'tank_base_safe.launch.py.*cmd_vel_topic:=/k1_boot_zero_cmd' 2>/dev/null || true
  sleep 1
  if ! raw_cmd_has_subscriber; then
    pkill -INT -f wheeltec_tank_base_safe.py 2>/dev/null || true
    sleep 1
  fi

  nohup ros2 launch turn_on_wheeltec_robot tank_base_safe.launch.py \
    serial_port:=/dev/base_controller \
    cmd_vel_topic:=/cmd_vel_raw \
    send_security_enable_on_start:=true security_ply:=1 \
    send_rate:=50.0 cmd_timeout:=0.25 \
    max_linear:=0.45 max_angular:=0.80 \
    cruise_linear_limit:=0.45 cruise_angular_limit:=0.80 \
    brake_duration:=1.0 start_kick_duration:=0.0 stop_kick_duration:=0.0 \
    stop_kick_match_cmd:=false stop_kick_match_duration:=false \
    stop_kick_speed_gain:=1.5 stop_kick_duration_mode:=fixed \
    stop_kick_duration_ratio:=1.0 stop_kick_impulse_ratio:=1.0 \
    stop_kick_duration_offset:=0.0 stop_kick_max_duration:=1.0 \
    stop_kick_min_duration:=0.12 stop_kick_until_stopped:=false \
    stop_kick_velocity_epsilon:=0.02 publish_tf:=true \
    >"${BASE_LOG}" 2>&1 &
  local pid=$!
  printf '%s\n' "${pid}" > "${BASE_PID_FILE}"
  for _ in $(seq 1 40); do
    if raw_cmd_has_subscriber; then
      echo "calibrated base started pid=${pid} log=${BASE_LOG}"
      return
    fi
    sleep 0.25
  done
  echo "base failed to subscribe /cmd_vel_raw; inspect ${BASE_LOG}" >&2
  return 1
}

stop_base() {
  if pid_file_running "${BASE_PID_FILE}"; then
    local pid
    pid="$(cat "${BASE_PID_FILE}")"
    kill -INT "${pid}" 2>/dev/null || true
    for _ in $(seq 1 30); do
      kill -0 "${pid}" 2>/dev/null || break
      sleep 0.1
    done
    echo "calibrated base stopped"
  fi
  rm -f "${BASE_PID_FILE}"
}

start_lidar() {
  source_ros
  if scan_has_publisher; then
    echo "scan publisher already exists; reusing external lidar"
  else
    nohup ros2 launch lslidar_driver lsn10p_launch.py \
      >"${LIDAR_LOG}" 2>&1 &
    local pid=$!
    printf '%s\n' "${pid}" > "${LIDAR_PID_FILE}"
    for _ in $(seq 1 40); do
      if scan_has_publisher; then
        echo "N10P lidar started pid=${pid} log=${LIDAR_LOG}"
        break
      fi
      sleep 0.25
    done
    if ! scan_has_publisher; then
      echo "N10P lidar failed to publish /scan; inspect ${LIDAR_LOG}" >&2
      return 1
    fi
  fi

  if ! ros2 node list 2>/dev/null | grep -qx '/semantic_base_to_laser_tf'; then
    nohup ros2 run tf2_ros static_transform_publisher \
      0.12 0.0 0.12 0.0 0.0 0.0 base_footprint laser \
      --ros-args -r __node:=semantic_base_to_laser_tf \
      >"${LASER_TF_LOG}" 2>&1 &
    printf '%s\n' "$!" > "${LASER_TF_PID_FILE}"
  fi
}

stop_lidar() {
  local file pid label
  for file in "${LIDAR_PID_FILE}" "${LASER_TF_PID_FILE}"; do
    if pid_file_running "${file}"; then
      pid="$(cat "${file}")"
      label="$(basename "${file}" .pid)"
      kill -INT "${pid}" 2>/dev/null || true
      for _ in $(seq 1 20); do
        kill -0 "${pid}" 2>/dev/null || break
        sleep 0.1
      done
      echo "${label} stopped"
    fi
    rm -f "${file}"
  done
}

start_server() {
  if is_running; then
    echo "already running pid=$(cat "${PID_FILE}") url=http://0.0.0.0:${PORT}/"
    return
  fi
  source_ros
  audit_raw_cmd_publishers
  nohup python3 tools/k1_semantic_mapping_server.py \
    --port "${PORT}" \
    --log-jsonl "${EVENT_LOG}" \
    >"${SERVER_LOG}" 2>&1 &
  local pid=$!
  printf '%s\n' "${pid}" > "${PID_FILE}"
  for _ in $(seq 1 30); do
    if curl -fsS "http://127.0.0.1:${PORT}/api/state" >/dev/null 2>&1; then
      echo "started pid=${pid} url=http://0.0.0.0:${PORT}/ log=${SERVER_LOG}"
      return
    fi
    sleep 0.2
  done
  echo "server failed to become ready; inspect ${SERVER_LOG}" >&2
  return 1
}

start_slam() {
  source_ros
  if ros2 node list 2>/dev/null | grep -qx '/slam_toolbox'; then
    echo "slam_toolbox already exists; reusing external SLAM node"
    return
  fi
  if pid_file_running "${SLAM_PID_FILE}"; then
    echo "slam_toolbox already running pid=$(cat "${SLAM_PID_FILE}")"
    return
  fi
  nohup ros2 run slam_toolbox async_slam_toolbox_node \
    --ros-args --params-file "${SLAM_PARAMS}" \
    >"${SLAM_LOG}" 2>&1 &
  local pid=$!
  printf '%s\n' "${pid}" > "${SLAM_PID_FILE}"
  for _ in $(seq 1 40); do
    if ros2 node list 2>/dev/null | grep -qx '/slam_toolbox'; then
      echo "slam_toolbox started pid=${pid} log=${SLAM_LOG}"
      return
    fi
    sleep 0.25
  done
  echo "slam_toolbox failed to become ready; inspect ${SLAM_LOG}" >&2
  return 1
}

stop_slam() {
  if pid_file_running "${SLAM_PID_FILE}"; then
    local pid
    local children
    local targets
    pid="$(cat "${SLAM_PID_FILE}")"
    children="$(pgrep -P "${pid}" 2>/dev/null || true)"
    targets="${children} ${pid}"
    # ros2 run keeps the actual slam_toolbox node as a child process.
    # Stop both so a new mapping round cannot silently reuse the old map.
    kill -INT ${targets} 2>/dev/null || true
    for _ in $(seq 1 30); do
      local alive=0
      local target
      for target in ${targets}; do
        if kill -0 "${target}" 2>/dev/null; then
          alive=1
        fi
      done
      [[ "${alive}" -eq 0 ]] && break
      sleep 0.1
    done
    kill -TERM ${targets} 2>/dev/null || true
    echo "slam_toolbox stopped"
  else
    echo "no controller-owned slam_toolbox process"
  fi
  rm -f "${SLAM_PID_FILE}"
}

start_mapping() {
  local host_ip
  start_base
  start_lidar
  start_slam
  start_server
  host_ip="$(hostname -I 2>/dev/null | awk '{print $1}')"
  echo "local controller: http://127.0.0.1:${PORT}/"
  if [[ -n "${host_ip}" ]]; then
    echo "network controller: http://${host_ip}:${PORT}/"
  fi
}

stop_mapping() {
  stop_server
  stop_slam
  stop_lidar
  stop_base
}

stop_server() {
  curl -fsS -X POST -H 'Content-Type: application/json' -d '{}' \
    "http://127.0.0.1:${PORT}/api/stop" >/dev/null 2>&1 || true
  if is_running; then
    local pid
    pid="$(cat "${PID_FILE}")"
    kill -INT "${pid}" 2>/dev/null || true
    for _ in $(seq 1 20); do
      kill -0 "${pid}" 2>/dev/null || break
      sleep 0.1
    done
  fi
  rm -f "${PID_FILE}"
  echo "stopped"
}

case "${1:-start}" in
  start) start_server ;;
  stop) stop_server ;;
  restart) stop_server; start_server ;;
  mapping-start) start_mapping ;;
  mapping-stop) stop_mapping ;;
  mapping-restart) stop_mapping; start_mapping ;;
  status)
    if is_running; then
      echo "running pid=$(cat "${PID_FILE}") url=http://0.0.0.0:${PORT}/"
      curl -fsS "http://127.0.0.1:${PORT}/api/state"
      echo
    else
      echo "stopped"
      exit 1
    fi
    ;;
  *) echo "usage: $0 {start|stop|restart|status|mapping-start|mapping-stop|mapping-restart}" >&2; exit 2 ;;
esac
