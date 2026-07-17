#!/usr/bin/env bash
set -euo pipefail

REPO_DIR="${REPO_DIR:-/home/soc/edge-ai-robot-k1}"
COMMAND="${1:-help}"
RUN_DIR="${2:-}"
CURRENT_RUN_FILE="${REPO_DIR}/.current_real_k1_rrt_nav2_run_dir"

if [[ -z "${RUN_DIR}" ]]; then
  case "${COMMAND}" in
    manual-map|nav2-slam)
      RUN_DIR="${REPO_DIR}/outputs/real_k1_rrt_nav2_mapping_$(date +%Y%m%d_%H%M%S)"
      ;;
    *)
      if [[ -s "${CURRENT_RUN_FILE}" ]]; then
        RUN_DIR="$(cat "${CURRENT_RUN_FILE}")"
      else
        RUN_DIR="${REPO_DIR}/outputs/real_k1_rrt_nav2_mapping_$(date +%Y%m%d_%H%M%S)"
      fi
      ;;
  esac
fi

source_ros() {
  cd "${REPO_DIR}"
  set +u
  source /opt/ros/humble/setup.bash
  if [[ -f /home/soc/lslidar_ws/install/setup.bash ]]; then
    source /home/soc/lslidar_ws/install/setup.bash
  fi
  source ros2_ws/install/setup.bash
  set -u
}

ensure_run_dir() {
  mkdir -p "${RUN_DIR}"
  echo "${RUN_DIR}" > "${CURRENT_RUN_FILE}"
  echo "[real-k1] run_dir=${RUN_DIR}"
}

usage() {
  cat <<'EOF'
Usage:
  bash tools/start_real_k1_rrt_nav2_mapping.sh <command> [run_dir]

Commands:
  preflight       Check core ROS topics and TF after a stack is running.
  clean           Stop mapping, Nav2, RRT, teleop, YOLO and local UI helpers.
  manual-map      Start guarded SLAM mapping. Teleop publishes to /input_cmd_vel.
  teleop-manual   Keyboard control for manual-map mode: /cmd_vel -> /input_cmd_vel.
  nav2-slam       Start guarded SLAM + Nav2. Nav2 publishes to /cmd_vel_raw.
  teleop-nav2     Keyboard override for nav2-slam mode: /cmd_vel -> /cmd_vel_raw.
  rrt-preview     Publish RRT frontier /rrt_preview_goal_pose only; does not send Nav2 action.
  rrt-run         Send RRT frontier goals through Nav2 NavigateToPose.
  rrt-preview-2m  2m x 2m scene preset: compact frontier /rrt_preview_goal_pose preview.
  rrt-run-2m      2m x 2m scene preset: compact frontier Nav2 action run.
  rrt-run-2m-unlimited
                  2m x 2m long-running guarded RRT/Nav2 mapping run without YOLO.
  d435            Start RealSense D435 ROS driver for RGB-D topics.
  yolo-ep         Start D435 YOLO risk bridge with SpaceMIT EP, no arm action.
  voice           Start SYN6288 serial voice bridge.
  ui              Serve current run directory on port 8765.
  save-map        Save /map into run_dir/maps.
  zero            Publish zero Twist through both /input_cmd_vel and /cmd_vel_raw.

Examples:
  bash tools/start_real_k1_rrt_nav2_mapping.sh nav2-slam
  bash tools/start_real_k1_rrt_nav2_mapping.sh rrt-preview /home/soc/edge-ai-robot-k1/outputs/test_run
  bash tools/start_real_k1_rrt_nav2_mapping.sh yolo-ep /home/soc/edge-ai-robot-k1/outputs/test_run
EOF
}

case "${COMMAND}" in
  help|-h|--help)
    usage
    ;;

  clean)
    pkill -INT -f 'sim_rrt_frontier_explorer.py' 2>/dev/null || true
    pkill -INT -f 'run_prelim_remote_mapping_yolo_arm_demo.py' 2>/dev/null || true
    pkill -f 'teleop_twist_keyboard' 2>/dev/null || true
    pkill -f 'http.server 8765' 2>/dev/null || true
    pkill -f 'tank_base_safe.launch.py' 2>/dev/null || true
    pkill -f 'wheeltec_tank_base_safe.py' 2>/dev/null || true
    pkill -f 'n10p_tank_nav2_slam.launch.py' 2>/dev/null || true
    pkill -f 'n10p_tank_mapping.launch.py' 2>/dev/null || true
    pkill -f 'n10p_tank_mapping_safety_guard.launch.py' 2>/dev/null || true
    pkill -f 'scan_safety_guard_node' 2>/dev/null || true
    pkill -f '/opt/ros/humble/lib/nav2_' 2>/dev/null || true
    pkill -f '/opt/ros/humble/lib/slam_toolbox' 2>/dev/null || true
    pkill -f 'lslidar_driver' 2>/dev/null || true
    pkill -f 'realsense2_camera.*rs_launch.py' 2>/dev/null || true
    sleep 2
    pkill -f 'sim_rrt_frontier_explorer.py' 2>/dev/null || true
    pkill -f 'run_prelim_remote_mapping_yolo_arm_demo.py' 2>/dev/null || true
    echo "[real-k1] cleaned"
    ;;

  preflight)
    source_ros
    echo "[real-k1] topics"
    ros2 topic list | grep -E '^/(scan|odom|map|tf|tf_static|cmd_vel_raw|cmd_vel_guarded|input_cmd_vel|perception/risk_alarm|safety/front_obstacle|camera/camera/color/image_raw|camera/camera/depth/image_rect_raw|camera/camera/color/camera_info)$' || true
    echo "[real-k1] /scan hz"
    timeout 5s ros2 topic hz /scan || true
    echo "[real-k1] /odom once"
    timeout 5s ros2 topic echo /odom --once || true
    echo "[real-k1] map->base_footprint tf"
    timeout 5s ros2 run tf2_ros tf2_echo map base_footprint || true
    ;;

  manual-map)
    source_ros
    ensure_run_dir
    exec ros2 launch turn_on_wheeltec_robot n10p_tank_mapping_safety_guard.launch.py \
      front_collision_corridor_half_width_m:="${FRONT_COLLISION_CORRIDOR_HALF_WIDTH_M:-0.26}" \
      front_collision_min_x_m:="${FRONT_COLLISION_MIN_X_M:-0.02}" \
      micro_adjust_sector_deg:="${MICRO_ADJUST_SECTOR_DEG:-45.0}" \
      micro_adjust_trigger_m:="${MICRO_ADJUST_TRIGGER_M:-0.22}" \
      micro_adjust_clear_m:="${MICRO_ADJUST_CLEAR_M:-0.30}" \
      micro_adjust_direction_deadband_m:="${MICRO_ADJUST_DIRECTION_DEADBAND_M:-0.03}" \
      micro_adjust_direction_latch_s:="${MICRO_ADJUST_DIRECTION_LATCH_S:-1.50}" \
      enable_escape_reverse:="${ENABLE_ESCAPE_REVERSE:-true}" \
      escape_reverse_trigger_m:="${ESCAPE_REVERSE_TRIGGER_M:-0.16}" \
      escape_reverse_clear_m:="${ESCAPE_REVERSE_CLEAR_M:-0.24}" \
      escape_reverse_linear_x:="${ESCAPE_REVERSE_LINEAR_X:--0.08}" \
      escape_reverse_angular_z:="${ESCAPE_REVERSE_ANGULAR_Z:-0.20}" \
      escape_reverse_max_s:="${ESCAPE_REVERSE_MAX_S:-0.80}" \
      escape_reverse_cooldown_s:="${ESCAPE_REVERSE_COOLDOWN_S:-0.40}" \
      hard_stop_m:=0.10 \
      emergency_stop_m:=0.10 \
      slow_down_m:=0.30 \
      approach_stop_m:=0.20 \
      min_effective_forward:=0.05 \
      clear_max_linear:=0.30 \
      soft_max_linear:=0.10
    ;;

  teleop-manual)
    source_ros
    exec ros2 run teleop_twist_keyboard teleop_twist_keyboard \
      --ros-args -r /cmd_vel:=/input_cmd_vel
    ;;

  nav2-slam)
    source_ros
    ensure_run_dir
    exec ros2 launch turn_on_wheeltec_robot n10p_tank_nav2_slam.launch.py \
      front_collision_corridor_half_width_m:="${FRONT_COLLISION_CORRIDOR_HALF_WIDTH_M:-0.26}" \
      front_collision_min_x_m:="${FRONT_COLLISION_MIN_X_M:-0.02}" \
      micro_adjust_sector_deg:="${MICRO_ADJUST_SECTOR_DEG:-45.0}" \
      micro_adjust_trigger_m:="${MICRO_ADJUST_TRIGGER_M:-0.22}" \
      micro_adjust_clear_m:="${MICRO_ADJUST_CLEAR_M:-0.30}" \
      micro_adjust_direction_deadband_m:="${MICRO_ADJUST_DIRECTION_DEADBAND_M:-0.03}" \
      micro_adjust_direction_latch_s:="${MICRO_ADJUST_DIRECTION_LATCH_S:-1.50}" \
      enable_escape_reverse:="${ENABLE_ESCAPE_REVERSE:-true}" \
      escape_reverse_trigger_m:="${ESCAPE_REVERSE_TRIGGER_M:-0.16}" \
      escape_reverse_clear_m:="${ESCAPE_REVERSE_CLEAR_M:-0.24}" \
      escape_reverse_linear_x:="${ESCAPE_REVERSE_LINEAR_X:--0.08}" \
      escape_reverse_angular_z:="${ESCAPE_REVERSE_ANGULAR_Z:-0.20}" \
      escape_reverse_max_s:="${ESCAPE_REVERSE_MAX_S:-0.80}" \
      escape_reverse_cooldown_s:="${ESCAPE_REVERSE_COOLDOWN_S:-0.40}" \
      hard_stop_m:=0.10 \
      emergency_stop_m:=0.10 \
      slow_down_m:=0.30 \
      approach_stop_m:=0.20 \
      min_effective_forward:=0.05 \
      clear_max_linear:=0.30 \
      soft_max_linear:=0.10
    ;;

  teleop-nav2)
    source_ros
    exec ros2 run teleop_twist_keyboard teleop_twist_keyboard \
      --ros-args -r /cmd_vel:=/cmd_vel_raw
    ;;

  rrt-preview)
    source_ros
    ensure_run_dir
    exec python3 tools/sim_rrt_frontier_explorer.py \
      --map-topic /map \
      --goal-topic /rrt_preview_goal_pose \
      --stop-on-risk-topic /perception/risk_alarm \
      --map-frame map \
      --base-frame base_footprint \
      --runtime-s "${RRT_RUNTIME_S:-180}" \
      --max-goals "${RRT_MAX_GOALS:-10}" \
      --sample-radius-m "${RRT_SAMPLE_RADIUS_M:-2.5}" \
      --min-goal-distance-m "${RRT_MIN_GOAL_DISTANCE_M:-0.45}" \
      --inflation-m "${RRT_INFLATION_M:-0.25}" \
      --frontier-standoff-m "${RRT_FRONTIER_STANDOFF_M:-0.35}" \
      --frontier-backoffs-m "${RRT_FRONTIER_BACKOFFS_M:-0.10,0.18,0.25,0.35}" \
      --goal-clearance-check-m "${RRT_GOAL_CLEARANCE_CHECK_M:-0.50}" \
      --rejected-goal-separation-m "${RRT_REJECTED_GOAL_SEPARATION_M:-0.25}" \
      --frontier-mode "${RRT_FRONTIER_MODE:-hybrid}" \
      --wfd-max-cells "${RRT_WFD_MAX_CELLS:-12000}" \
      --min-frontier-cluster-cells "${RRT_MIN_FRONTIER_CLUSTER_CELLS:-2}" \
      --frontier-cluster-candidate-limit "${RRT_FRONTIER_CLUSTER_CANDIDATE_LIMIT:-8}" \
      --frontier-distance-weight "${RRT_FRONTIER_DISTANCE_WEIGHT:-1.0}" \
      --frontier-size-weight "${RRT_FRONTIER_SIZE_WEIGHT:-0.05}" \
      --goal-result-timeout-s "${RRT_GOAL_TIMEOUT_S:-35}" \
      --goal-send-timeout-s "${RRT_GOAL_SEND_TIMEOUT_S:-8}" \
      --goal-progress-timeout-s "${RRT_GOAL_PROGRESS_TIMEOUT_S:-12}" \
      --goal-progress-grace-s "${RRT_GOAL_PROGRESS_GRACE_S:-5}" \
      --goal-progress-epsilon-m "${RRT_GOAL_PROGRESS_EPSILON_M:-0.03}" \
      --failure-backoff-after "${RRT_FAILURE_BACKOFF_AFTER:-8}" \
      --failure-backoff-s "${RRT_FAILURE_BACKOFF_S:-5}" \
      --start-free-search-m "${RRT_START_FREE_SEARCH_M:-0.35}" \
      --report "${RUN_DIR}/rrt_frontier_preview_report.json"
    ;;

  rrt-run)
    source_ros
    ensure_run_dir
    exec python3 tools/sim_rrt_frontier_explorer.py \
      --map-topic /map \
      --goal-topic /rrt_preview_goal_pose \
      --stop-on-risk-topic /perception/risk_alarm \
      --map-frame map \
      --base-frame base_footprint \
      --runtime-s "${RRT_RUNTIME_S:-180}" \
      --max-goals "${RRT_MAX_GOALS:-10}" \
      --sample-radius-m "${RRT_SAMPLE_RADIUS_M:-2.5}" \
      --min-goal-distance-m "${RRT_MIN_GOAL_DISTANCE_M:-0.45}" \
      --inflation-m "${RRT_INFLATION_M:-0.25}" \
      --frontier-standoff-m "${RRT_FRONTIER_STANDOFF_M:-0.35}" \
      --frontier-backoffs-m "${RRT_FRONTIER_BACKOFFS_M:-0.10,0.18,0.25,0.35}" \
      --goal-clearance-check-m "${RRT_GOAL_CLEARANCE_CHECK_M:-0.50}" \
      --rejected-goal-separation-m "${RRT_REJECTED_GOAL_SEPARATION_M:-0.25}" \
      --frontier-mode "${RRT_FRONTIER_MODE:-hybrid}" \
      --wfd-max-cells "${RRT_WFD_MAX_CELLS:-12000}" \
      --min-frontier-cluster-cells "${RRT_MIN_FRONTIER_CLUSTER_CELLS:-2}" \
      --frontier-cluster-candidate-limit "${RRT_FRONTIER_CLUSTER_CANDIDATE_LIMIT:-8}" \
      --frontier-distance-weight "${RRT_FRONTIER_DISTANCE_WEIGHT:-1.0}" \
      --frontier-size-weight "${RRT_FRONTIER_SIZE_WEIGHT:-0.05}" \
      --goal-result-timeout-s "${RRT_GOAL_TIMEOUT_S:-35}" \
      --goal-send-timeout-s "${RRT_GOAL_SEND_TIMEOUT_S:-8}" \
      --goal-progress-timeout-s "${RRT_GOAL_PROGRESS_TIMEOUT_S:-12}" \
      --goal-progress-grace-s "${RRT_GOAL_PROGRESS_GRACE_S:-5}" \
      --goal-progress-epsilon-m "${RRT_GOAL_PROGRESS_EPSILON_M:-0.03}" \
      --failure-backoff-after "${RRT_FAILURE_BACKOFF_AFTER:-8}" \
      --failure-backoff-s "${RRT_FAILURE_BACKOFF_S:-5}" \
      --start-free-search-m "${RRT_START_FREE_SEARCH_M:-0.35}" \
      --send-nav2-action \
      --report "${RUN_DIR}/rrt_frontier_nav2_report.json"
    ;;

  rrt-preview-2m)
    source_ros
    ensure_run_dir
    exec python3 tools/sim_rrt_frontier_explorer.py \
      --map-topic /map \
      --goal-topic /rrt_preview_goal_pose \
      --stop-on-risk-topic /perception/risk_alarm \
      --map-frame map \
      --base-frame base_footprint \
      --runtime-s "${RRT_RUNTIME_S:-120}" \
      --max-goals "${RRT_MAX_GOALS:-6}" \
      --sample-radius-m "${RRT_SAMPLE_RADIUS_M:-1.00}" \
      --min-goal-distance-m "${RRT_MIN_GOAL_DISTANCE_M:-0.20}" \
      --inflation-m "${RRT_INFLATION_M:-0.12}" \
      --frontier-standoff-m "${RRT_FRONTIER_STANDOFF_M:-0.10}" \
      --frontier-backoffs-m "${RRT_FRONTIER_BACKOFFS_M:-0.10,0.18,0.25,0.35}" \
      --goal-clearance-check-m "${RRT_GOAL_CLEARANCE_CHECK_M:-0.50}" \
      --goal-separation-m "${RRT_GOAL_SEPARATION_M:-0.12}" \
      --map-edge-margin-m "${RRT_MAP_EDGE_MARGIN_M:-0.15}" \
      --rejected-goal-separation-m "${RRT_REJECTED_GOAL_SEPARATION_M:-0.25}" \
      --frontier-mode "${RRT_FRONTIER_MODE:-hybrid}" \
      --wfd-max-cells "${RRT_WFD_MAX_CELLS:-12000}" \
      --min-frontier-cluster-cells "${RRT_MIN_FRONTIER_CLUSTER_CELLS:-2}" \
      --frontier-cluster-candidate-limit "${RRT_FRONTIER_CLUSTER_CANDIDATE_LIMIT:-8}" \
      --frontier-distance-weight "${RRT_FRONTIER_DISTANCE_WEIGHT:-1.0}" \
      --frontier-size-weight "${RRT_FRONTIER_SIZE_WEIGHT:-0.05}" \
      --goal-result-timeout-s "${RRT_GOAL_TIMEOUT_S:-25}" \
      --goal-send-timeout-s "${RRT_GOAL_SEND_TIMEOUT_S:-8}" \
      --goal-progress-timeout-s "${RRT_GOAL_PROGRESS_TIMEOUT_S:-12}" \
      --goal-progress-grace-s "${RRT_GOAL_PROGRESS_GRACE_S:-5}" \
      --goal-progress-epsilon-m "${RRT_GOAL_PROGRESS_EPSILON_M:-0.03}" \
      --failure-backoff-after "${RRT_FAILURE_BACKOFF_AFTER:-8}" \
      --failure-backoff-s "${RRT_FAILURE_BACKOFF_S:-5}" \
      --start-free-search-m "${RRT_START_FREE_SEARCH_M:-0.35}" \
      --report "${RUN_DIR}/rrt_frontier_preview_2m_report.json"
    ;;

  rrt-run-2m)
    source_ros
    ensure_run_dir
    exec python3 tools/sim_rrt_frontier_explorer.py \
      --map-topic /map \
      --goal-topic /rrt_preview_goal_pose \
      --stop-on-risk-topic /perception/risk_alarm \
      --map-frame map \
      --base-frame base_footprint \
      --runtime-s "${RRT_RUNTIME_S:-120}" \
      --max-goals "${RRT_MAX_GOALS:-4}" \
      --sample-radius-m "${RRT_SAMPLE_RADIUS_M:-1.00}" \
      --min-goal-distance-m "${RRT_MIN_GOAL_DISTANCE_M:-0.20}" \
      --inflation-m "${RRT_INFLATION_M:-0.12}" \
      --frontier-standoff-m "${RRT_FRONTIER_STANDOFF_M:-0.10}" \
      --frontier-backoffs-m "${RRT_FRONTIER_BACKOFFS_M:-0.10,0.18,0.25,0.35}" \
      --goal-clearance-check-m "${RRT_GOAL_CLEARANCE_CHECK_M:-0.50}" \
      --goal-separation-m "${RRT_GOAL_SEPARATION_M:-0.12}" \
      --map-edge-margin-m "${RRT_MAP_EDGE_MARGIN_M:-0.15}" \
      --rejected-goal-separation-m "${RRT_REJECTED_GOAL_SEPARATION_M:-0.25}" \
      --frontier-mode "${RRT_FRONTIER_MODE:-hybrid}" \
      --wfd-max-cells "${RRT_WFD_MAX_CELLS:-12000}" \
      --min-frontier-cluster-cells "${RRT_MIN_FRONTIER_CLUSTER_CELLS:-2}" \
      --frontier-cluster-candidate-limit "${RRT_FRONTIER_CLUSTER_CANDIDATE_LIMIT:-8}" \
      --frontier-distance-weight "${RRT_FRONTIER_DISTANCE_WEIGHT:-1.0}" \
      --frontier-size-weight "${RRT_FRONTIER_SIZE_WEIGHT:-0.05}" \
      --goal-result-timeout-s "${RRT_GOAL_TIMEOUT_S:-25}" \
      --goal-send-timeout-s "${RRT_GOAL_SEND_TIMEOUT_S:-8}" \
      --goal-progress-timeout-s "${RRT_GOAL_PROGRESS_TIMEOUT_S:-12}" \
      --goal-progress-grace-s "${RRT_GOAL_PROGRESS_GRACE_S:-5}" \
      --goal-progress-epsilon-m "${RRT_GOAL_PROGRESS_EPSILON_M:-0.03}" \
      --failure-backoff-after "${RRT_FAILURE_BACKOFF_AFTER:-8}" \
      --failure-backoff-s "${RRT_FAILURE_BACKOFF_S:-5}" \
      --start-free-search-m "${RRT_START_FREE_SEARCH_M:-0.35}" \
      --send-nav2-action \
      --report "${RUN_DIR}/rrt_frontier_nav2_2m_report.json"
    ;;

  rrt-run-2m-unlimited)
    source_ros
    ensure_run_dir
    exec python3 tools/sim_rrt_frontier_explorer.py \
      --map-topic /map \
      --goal-topic /rrt_preview_goal_pose \
      --stop-on-risk-topic '' \
      --map-frame map \
      --base-frame base_footprint \
      --runtime-s "${RRT_RUNTIME_S:-86400}" \
      --max-goals "${RRT_MAX_GOALS:-1000000}" \
      --sample-radius-m "${RRT_SAMPLE_RADIUS_M:-1.00}" \
      --min-goal-distance-m "${RRT_MIN_GOAL_DISTANCE_M:-0.20}" \
      --inflation-m "${RRT_INFLATION_M:-0.12}" \
      --frontier-standoff-m "${RRT_FRONTIER_STANDOFF_M:-0.10}" \
      --frontier-backoffs-m "${RRT_FRONTIER_BACKOFFS_M:-0.10,0.18,0.25,0.35}" \
      --goal-clearance-check-m "${RRT_GOAL_CLEARANCE_CHECK_M:-0.50}" \
      --goal-separation-m "${RRT_GOAL_SEPARATION_M:-0.12}" \
      --recent-goal-memory "${RRT_RECENT_GOAL_MEMORY:-8}" \
      --rejected-goal-memory "${RRT_REJECTED_GOAL_MEMORY:-60}" \
      --rejected-goal-separation-m "${RRT_REJECTED_GOAL_SEPARATION_M:-0.25}" \
      --free-roam-when-no-frontier \
      --free-roam-min-distance-m "${RRT_FREE_ROAM_MIN_DISTANCE_M:-0.20}" \
      --map-edge-margin-m "${RRT_MAP_EDGE_MARGIN_M:-0.15}" \
      --frontier-mode "${RRT_FRONTIER_MODE:-hybrid}" \
      --wfd-max-cells "${RRT_WFD_MAX_CELLS:-12000}" \
      --min-frontier-cluster-cells "${RRT_MIN_FRONTIER_CLUSTER_CELLS:-2}" \
      --frontier-cluster-candidate-limit "${RRT_FRONTIER_CLUSTER_CANDIDATE_LIMIT:-8}" \
      --frontier-distance-weight "${RRT_FRONTIER_DISTANCE_WEIGHT:-1.0}" \
      --frontier-size-weight "${RRT_FRONTIER_SIZE_WEIGHT:-0.05}" \
      --goal-result-timeout-s "${RRT_GOAL_TIMEOUT_S:-25}" \
      --goal-send-timeout-s "${RRT_GOAL_SEND_TIMEOUT_S:-8}" \
      --goal-progress-timeout-s "${RRT_GOAL_PROGRESS_TIMEOUT_S:-12}" \
      --goal-progress-grace-s "${RRT_GOAL_PROGRESS_GRACE_S:-5}" \
      --goal-progress-epsilon-m "${RRT_GOAL_PROGRESS_EPSILON_M:-0.03}" \
      --failure-backoff-after "${RRT_FAILURE_BACKOFF_AFTER:-8}" \
      --failure-backoff-s "${RRT_FAILURE_BACKOFF_S:-5}" \
      --start-free-search-m "${RRT_START_FREE_SEARCH_M:-0.35}" \
      --send-nav2-action \
      --report "${RUN_DIR}/rrt_frontier_nav2_2m_unlimited_report.json"
    ;;

  d435)
    source_ros
    ensure_run_dir
    exec ros2 launch realsense2_camera rs_launch.py \
      depth_module.depth_profile:=640,480,30 \
      depth_module.infra_profile:=640,480,30 \
      rgb_camera.color_profile:=640,480,30
    ;;

  yolo-ep)
    source_ros
    ensure_run_dir
    pkill -INT -f 'run_prelim_remote_mapping_yolo_arm_demo.py' 2>/dev/null || true
    sleep 2
    pkill -f 'run_prelim_remote_mapping_yolo_arm_demo.py' 2>/dev/null || true
    sleep 8
    exec python3 tools/run_prelim_remote_mapping_yolo_arm_demo.py \
      --provider spacemit \
      --model models/risk_vision/yolov8n_480x640_q_truncated6_balanced_blockage03.onnx \
      --imgsz 640 \
      --conf "${YOLO_CONF:-0.15}" \
      --iou "${YOLO_IOU:-0.45}" \
      --max-det "${YOLO_MAX_DET:-10}" \
      --min-depth-m "${YOLO_MIN_DEPTH_M:-0.20}" \
      --max-depth-m "${YOLO_MAX_DEPTH_M:-1.20}" \
      --alarm-topic /perception/risk_alarm \
      --auto-risk-gates "${AUTO_RISK_GATES:-crack:0.29:0.60:0.80,blockage:0.23:0.35:0.75}" \
      --dedup-map-grid-m "${RISK_DEDUP_MAP_GRID_M:-0.20}" \
      --arm-response-mode disabled \
      --output-dir "${RUN_DIR}/yolo_risk"
    ;;

  voice)
    source_ros
    ensure_run_dir
    exec bash tools/start_prelim_syn6288_voice_k1.sh "${SYN6288_PORT:-/dev/ttyUSB0}" "${SYN6288_BAUD:-9600}"
    ;;

  ui)
    ensure_run_dir
    UI_DIR="${RUN_DIR}/yolo_risk"
    mkdir -p "${UI_DIR}"
    cp "${REPO_DIR}/tools/prelim_yolo_monitor.html" "${UI_DIR}/yolo_monitor.html" 2>/dev/null || true
    exec python3 -m http.server 8765 --bind 0.0.0.0 --directory "${UI_DIR}"
    ;;

  save-map)
    source_ros
    ensure_run_dir
    mkdir -p "${RUN_DIR}/maps"
    ros2 run nav2_map_server map_saver_cli -f "${RUN_DIR}/maps/map_$(date +%Y%m%d_%H%M%S)"
    ;;

  zero)
    source_ros
    timeout 2s ros2 topic pub -r 10 /input_cmd_vel geometry_msgs/msg/Twist '{}' >/tmp/real_k1_zero_input.log 2>&1 || true
    timeout 2s ros2 topic pub -r 10 /cmd_vel_raw geometry_msgs/msg/Twist '{}' >/tmp/real_k1_zero_raw.log 2>&1 || true
    echo "[real-k1] zero Twist published"
    ;;

  *)
    echo "Unknown command: ${COMMAND}" >&2
    usage
    exit 2
    ;;
esac
