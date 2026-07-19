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
  export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"
  export RCUTILS_LOGGING_BUFFERED_STREAM="${RCUTILS_LOGGING_BUFFERED_STREAM:-1}"
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
  risk-approach   Start headless D435 YOLO risk detection and event-driven Nav2 approach.
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
    pkill -INT -f 'run_real_k1_risk_approach_from_event.py' 2>/dev/null || true
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
    pkill -f 'static_transform_publisher.*base_footprint.*laser' 2>/dev/null || true
    pkill -f 'realsense2_camera.*rs_launch.py' 2>/dev/null || true
    pkill -TERM -f '[k]1_task_desktop_dashboard.py' 2>/dev/null || true
    pkill -TERM -f '[c]hrom.*8780' 2>/dev/null || true
    sleep 2
    pkill -f 'sim_rrt_frontier_explorer.py' 2>/dev/null || true
    pkill -f 'run_prelim_remote_mapping_yolo_arm_demo.py' 2>/dev/null || true
    pkill -f 'run_real_k1_risk_approach_from_event.py' 2>/dev/null || true
    pkill -KILL -f '[k]1_task_desktop_dashboard.py' 2>/dev/null || true
    pkill -KILL -f '[c]hrom.*8780' 2>/dev/null || true
    source_ros >/dev/null 2>&1 || true
    ros2 daemon stop >/dev/null 2>&1 || true
    find /dev/shm -maxdepth 1 \( -name 'fastrtps_*' -o -name 'sem.fastrtps_*' \) -user "$(id -un)" -delete 2>/dev/null || true
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
      front_collision_corridor_half_width_m:="${FRONT_COLLISION_CORRIDOR_HALF_WIDTH_M:-0.20}" \
      front_collision_min_x_m:="${FRONT_COLLISION_MIN_X_M:-0.12}" \
      micro_adjust_sector_deg:="${MICRO_ADJUST_SECTOR_DEG:-45.0}" \
      micro_adjust_trigger_m:="${MICRO_ADJUST_TRIGGER_M:-0.16}" \
      micro_adjust_clear_m:="${MICRO_ADJUST_CLEAR_M:-0.22}" \
      micro_adjust_direction_deadband_m:="${MICRO_ADJUST_DIRECTION_DEADBAND_M:-0.03}" \
      micro_adjust_direction_latch_s:="${MICRO_ADJUST_DIRECTION_LATCH_S:-1.50}" \
      enable_spin_escape:="${ENABLE_SPIN_ESCAPE:-true}" \
      spin_escape_turn_changes:="${SPIN_ESCAPE_TURN_CHANGES:-3}" \
      spin_escape_degrees:="${SPIN_ESCAPE_DEGREES:-180.0}" \
      spin_escape_angular_z:="${SPIN_ESCAPE_ANGULAR_Z:-0.35}" \
      spin_escape_cooldown_s:="${SPIN_ESCAPE_COOLDOWN_S:-3.0}" \
      enable_micro_adjust_stuck_spin_escape:="${ENABLE_MICRO_ADJUST_STUCK_SPIN_ESCAPE:-true}" \
      micro_adjust_stuck_spin_min_s:="${MICRO_ADJUST_STUCK_SPIN_MIN_S:-6.0}" \
      micro_adjust_stuck_spin_front_blocked_m:="${MICRO_ADJUST_STUCK_SPIN_FRONT_BLOCKED_M:-0.30}" \
      micro_adjust_stuck_spin_clear_m:="${MICRO_ADJUST_STUCK_SPIN_CLEAR_M:-0.40}" \
      micro_adjust_stuck_spin_cmd_angular_mps:="${MICRO_ADJUST_STUCK_SPIN_CMD_ANGULAR_MPS:-0.05}" \
      enable_corridor_stuck_spin_escape:="${ENABLE_CORRIDOR_STUCK_SPIN_ESCAPE:-true}" \
      corridor_stuck_spin_trigger_m:="${CORRIDOR_STUCK_SPIN_TRIGGER_M:-0.18}" \
      corridor_stuck_spin_clear_m:="${CORRIDOR_STUCK_SPIN_CLEAR_M:-0.24}" \
      corridor_stuck_spin_min_s:="${CORRIDOR_STUCK_SPIN_MIN_S:-3.0}" \
      corridor_stuck_spin_cmd_angular_mps:="${CORRIDOR_STUCK_SPIN_CMD_ANGULAR_MPS:-0.06}" \
      corridor_stuck_spin_front_blocked_m:="${CORRIDOR_STUCK_SPIN_FRONT_BLOCKED_M:-0.30}" \
      corridor_stuck_spin_front_sector_deg:="${CORRIDOR_STUCK_SPIN_FRONT_SECTOR_DEG:-20.0}" \
      corridor_stuck_spin_require_sides:="${CORRIDOR_STUCK_SPIN_REQUIRE_SIDES:-true}" \
      corridor_stuck_spin_side_blocked_m:="${CORRIDOR_STUCK_SPIN_SIDE_BLOCKED_M:-0.32}" \
      enable_corridor_trial:="${ENABLE_CORRIDOR_TRIAL:-true}" \
      corridor_trial_center_half_width_m:="${CORRIDOR_TRIAL_CENTER_HALF_WIDTH_M:-0.10}" \
      corridor_trial_enter_front_p10_m:="${CORRIDOR_TRIAL_ENTER_FRONT_P10_M:-0.40}" \
      corridor_trial_keep_front_p10_m:="${CORRIDOR_TRIAL_KEEP_FRONT_P10_M:-0.34}" \
      corridor_trial_side_near_m:="${CORRIDOR_TRIAL_SIDE_NEAR_M:-0.40}" \
      corridor_trial_enter_stable_s:="${CORRIDOR_TRIAL_ENTER_STABLE_S:-0.50}" \
      corridor_trial_exit_stable_s:="${CORRIDOR_TRIAL_EXIT_STABLE_S:-0.80}" \
      corridor_trial_forward_intent_mps:="${CORRIDOR_TRIAL_FORWARD_INTENT_MPS:-0.04}" \
      corridor_trial_max_linear_mps:="${CORRIDOR_TRIAL_MAX_LINEAR_MPS:-0.24}" \
      corridor_trial_max_angular_radps:="${CORRIDOR_TRIAL_MAX_ANGULAR_RADPS:-0.16}" \
      corridor_trial_wall_turn_limit_radps:="${CORRIDOR_TRIAL_WALL_TURN_LIMIT_RADPS:-0.06}" \
      corridor_trial_progress_window_s:="${CORRIDOR_TRIAL_PROGRESS_WINDOW_S:-2.50}" \
      corridor_trial_min_forward_progress_m:="${CORRIDOR_TRIAL_MIN_FORWARD_PROGRESS_M:-0.04}" \
      corridor_trial_blocked_front_p10_m:="${CORRIDOR_TRIAL_BLOCKED_FRONT_P10_M:-0.30}" \
      corridor_trial_blocked_stable_s:="${CORRIDOR_TRIAL_BLOCKED_STABLE_S:-0.80}" \
      corridor_trial_stop_s:="${CORRIDOR_TRIAL_STOP_S:-0.40}" \
      corridor_trial_rear_clear_m:="${CORRIDOR_TRIAL_REAR_CLEAR_M:-0.18}" \
      corridor_trial_max_recoveries:="${CORRIDOR_TRIAL_MAX_RECOVERIES:-2}" \
      corridor_trial_odom_timeout_s:="${CORRIDOR_TRIAL_ODOM_TIMEOUT_S:-0.60}" \
      enable_escape_reverse:="${ENABLE_ESCAPE_REVERSE:-true}" \
      escape_reverse_trigger_m:="${ESCAPE_REVERSE_TRIGGER_M:-0.12}" \
      escape_reverse_clear_m:="${ESCAPE_REVERSE_CLEAR_M:-0.18}" \
      escape_reverse_linear_x:="${ESCAPE_REVERSE_LINEAR_X:--0.14}" \
      escape_reverse_angular_z:="${ESCAPE_REVERSE_ANGULAR_Z:-0.20}" \
      escape_reverse_max_s:="${ESCAPE_REVERSE_MAX_S:-0.80}" \
      escape_reverse_cooldown_s:="${ESCAPE_REVERSE_COOLDOWN_S:-0.40}" \
      hard_stop_m:="${HARD_STOP_M:-0.10}" \
      emergency_stop_m:="${EMERGENCY_STOP_M:-0.10}" \
      slow_down_m:="${SLOW_DOWN_M:-0.30}" \
      approach_stop_m:="${APPROACH_STOP_M:-0.20}" \
      min_effective_forward:="${MIN_EFFECTIVE_FORWARD:-0.12}" \
      clear_max_linear:="${CLEAR_MAX_LINEAR:-0.24}" \
      soft_max_linear:="${SOFT_MAX_LINEAR:-0.12}"
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
      front_collision_corridor_half_width_m:="${FRONT_COLLISION_CORRIDOR_HALF_WIDTH_M:-0.20}" \
      front_collision_min_x_m:="${FRONT_COLLISION_MIN_X_M:-0.12}" \
      micro_adjust_sector_deg:="${MICRO_ADJUST_SECTOR_DEG:-45.0}" \
      micro_adjust_trigger_m:="${MICRO_ADJUST_TRIGGER_M:-0.16}" \
      micro_adjust_clear_m:="${MICRO_ADJUST_CLEAR_M:-0.22}" \
      micro_adjust_direction_deadband_m:="${MICRO_ADJUST_DIRECTION_DEADBAND_M:-0.03}" \
      micro_adjust_direction_latch_s:="${MICRO_ADJUST_DIRECTION_LATCH_S:-1.50}" \
      enable_spin_escape:="${ENABLE_SPIN_ESCAPE:-true}" \
      spin_escape_turn_changes:="${SPIN_ESCAPE_TURN_CHANGES:-3}" \
      spin_escape_degrees:="${SPIN_ESCAPE_DEGREES:-180.0}" \
      spin_escape_angular_z:="${SPIN_ESCAPE_ANGULAR_Z:-0.35}" \
      spin_escape_cooldown_s:="${SPIN_ESCAPE_COOLDOWN_S:-3.0}" \
      enable_micro_adjust_stuck_spin_escape:="${ENABLE_MICRO_ADJUST_STUCK_SPIN_ESCAPE:-true}" \
      micro_adjust_stuck_spin_min_s:="${MICRO_ADJUST_STUCK_SPIN_MIN_S:-6.0}" \
      micro_adjust_stuck_spin_front_blocked_m:="${MICRO_ADJUST_STUCK_SPIN_FRONT_BLOCKED_M:-0.30}" \
      micro_adjust_stuck_spin_clear_m:="${MICRO_ADJUST_STUCK_SPIN_CLEAR_M:-0.40}" \
      micro_adjust_stuck_spin_cmd_angular_mps:="${MICRO_ADJUST_STUCK_SPIN_CMD_ANGULAR_MPS:-0.05}" \
      enable_corridor_stuck_spin_escape:="${ENABLE_CORRIDOR_STUCK_SPIN_ESCAPE:-true}" \
      corridor_stuck_spin_trigger_m:="${CORRIDOR_STUCK_SPIN_TRIGGER_M:-0.18}" \
      corridor_stuck_spin_clear_m:="${CORRIDOR_STUCK_SPIN_CLEAR_M:-0.24}" \
      corridor_stuck_spin_min_s:="${CORRIDOR_STUCK_SPIN_MIN_S:-3.0}" \
      corridor_stuck_spin_cmd_angular_mps:="${CORRIDOR_STUCK_SPIN_CMD_ANGULAR_MPS:-0.06}" \
      corridor_stuck_spin_front_blocked_m:="${CORRIDOR_STUCK_SPIN_FRONT_BLOCKED_M:-0.30}" \
      corridor_stuck_spin_front_sector_deg:="${CORRIDOR_STUCK_SPIN_FRONT_SECTOR_DEG:-20.0}" \
      corridor_stuck_spin_require_sides:="${CORRIDOR_STUCK_SPIN_REQUIRE_SIDES:-true}" \
      corridor_stuck_spin_side_blocked_m:="${CORRIDOR_STUCK_SPIN_SIDE_BLOCKED_M:-0.32}" \
      enable_corridor_trial:="${ENABLE_CORRIDOR_TRIAL:-true}" \
      corridor_trial_center_half_width_m:="${CORRIDOR_TRIAL_CENTER_HALF_WIDTH_M:-0.10}" \
      corridor_trial_enter_front_p10_m:="${CORRIDOR_TRIAL_ENTER_FRONT_P10_M:-0.40}" \
      corridor_trial_keep_front_p10_m:="${CORRIDOR_TRIAL_KEEP_FRONT_P10_M:-0.34}" \
      corridor_trial_side_near_m:="${CORRIDOR_TRIAL_SIDE_NEAR_M:-0.40}" \
      corridor_trial_enter_stable_s:="${CORRIDOR_TRIAL_ENTER_STABLE_S:-0.50}" \
      corridor_trial_exit_stable_s:="${CORRIDOR_TRIAL_EXIT_STABLE_S:-0.80}" \
      corridor_trial_forward_intent_mps:="${CORRIDOR_TRIAL_FORWARD_INTENT_MPS:-0.04}" \
      corridor_trial_max_linear_mps:="${CORRIDOR_TRIAL_MAX_LINEAR_MPS:-0.24}" \
      corridor_trial_max_angular_radps:="${CORRIDOR_TRIAL_MAX_ANGULAR_RADPS:-0.16}" \
      corridor_trial_wall_turn_limit_radps:="${CORRIDOR_TRIAL_WALL_TURN_LIMIT_RADPS:-0.06}" \
      corridor_trial_progress_window_s:="${CORRIDOR_TRIAL_PROGRESS_WINDOW_S:-2.50}" \
      corridor_trial_min_forward_progress_m:="${CORRIDOR_TRIAL_MIN_FORWARD_PROGRESS_M:-0.04}" \
      corridor_trial_blocked_front_p10_m:="${CORRIDOR_TRIAL_BLOCKED_FRONT_P10_M:-0.30}" \
      corridor_trial_blocked_stable_s:="${CORRIDOR_TRIAL_BLOCKED_STABLE_S:-0.80}" \
      corridor_trial_stop_s:="${CORRIDOR_TRIAL_STOP_S:-0.40}" \
      corridor_trial_rear_clear_m:="${CORRIDOR_TRIAL_REAR_CLEAR_M:-0.18}" \
      corridor_trial_max_recoveries:="${CORRIDOR_TRIAL_MAX_RECOVERIES:-2}" \
      corridor_trial_odom_timeout_s:="${CORRIDOR_TRIAL_ODOM_TIMEOUT_S:-0.60}" \
      enable_escape_reverse:="${ENABLE_ESCAPE_REVERSE:-true}" \
      escape_reverse_trigger_m:="${ESCAPE_REVERSE_TRIGGER_M:-0.12}" \
      escape_reverse_clear_m:="${ESCAPE_REVERSE_CLEAR_M:-0.18}" \
      escape_reverse_linear_x:="${ESCAPE_REVERSE_LINEAR_X:--0.14}" \
      escape_reverse_angular_z:="${ESCAPE_REVERSE_ANGULAR_Z:-0.20}" \
      escape_reverse_max_s:="${ESCAPE_REVERSE_MAX_S:-0.80}" \
      escape_reverse_cooldown_s:="${ESCAPE_REVERSE_COOLDOWN_S:-0.40}" \
      hard_stop_m:="${HARD_STOP_M:-0.10}" \
      emergency_stop_m:="${EMERGENCY_STOP_M:-0.10}" \
      slow_down_m:="${SLOW_DOWN_M:-0.30}" \
      approach_stop_m:="${APPROACH_STOP_M:-0.20}" \
      min_effective_forward:="${MIN_EFFECTIVE_FORWARD:-0.12}" \
      clear_max_linear:="${CLEAR_MAX_LINEAR:-0.24}" \
      soft_max_linear:="${SOFT_MAX_LINEAR:-0.12}"
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
      --adaptive-sample-radius \
      --adaptive-tight-sample-radius-m "${RRT_ADAPTIVE_TIGHT_SAMPLE_RADIUS_M:-0.60}" \
      --adaptive-wide-sample-radius-m "${RRT_ADAPTIVE_WIDE_SAMPLE_RADIUS_M:-1.00}" \
      --adaptive-wide-clearance-m "${RRT_ADAPTIVE_WIDE_CLEARANCE_M:-0.55}" \
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
      --frontier-distance-weight "${RRT_FRONTIER_DISTANCE_WEIGHT:-0.25}" \
      --frontier-size-weight "${RRT_FRONTIER_SIZE_WEIGHT:-1.0}" \
      --frontier-unknown-weight "${RRT_FRONTIER_UNKNOWN_WEIGHT:-0.35}" \
      --frontier-unknown-gain-radius-m "${RRT_FRONTIER_UNKNOWN_GAIN_RADIUS_M:-0.35}" \
      --free-roam-unknown-weight "${RRT_FREE_ROAM_UNKNOWN_WEIGHT:-0.30}" \
      --free-roam-distance-weight "${RRT_FREE_ROAM_DISTANCE_WEIGHT:-0.20}" \
      --free-roam-unknown-gain-radius-m "${RRT_FREE_ROAM_UNKNOWN_GAIN_RADIUS_M:-0.40}" \
      --goal-result-timeout-s "${RRT_GOAL_TIMEOUT_S:-35}" \
      --goal-send-timeout-s "${RRT_GOAL_SEND_TIMEOUT_S:-8}" \
      --goal-progress-timeout-s "${RRT_GOAL_PROGRESS_TIMEOUT_S:-12}" \
      --goal-progress-grace-s "${RRT_GOAL_PROGRESS_GRACE_S:-5}" \
      --goal-progress-epsilon-m "${RRT_GOAL_PROGRESS_EPSILON_M:-0.03}" \
      --physical-stuck-cmd-topic "${RRT_PHYSICAL_STUCK_CMD_TOPIC:-/cmd_vel_guarded}" \
      --physical-stuck-odom-topic "${RRT_PHYSICAL_STUCK_ODOM_TOPIC:-/odom}" \
      --physical-stuck-cmd-linear-mps "${RRT_PHYSICAL_STUCK_CMD_LINEAR_MPS:-0.10}" \
      --physical-stuck-odom-linear-mps "${RRT_PHYSICAL_STUCK_ODOM_LINEAR_MPS:-0.02}" \
      --physical-stuck-s "${RRT_PHYSICAL_STUCK_S:-2.0}" \
      --physical-stuck-escape-cmd-topic "${RRT_PHYSICAL_STUCK_ESCAPE_CMD_TOPIC:-/cmd_vel_raw}" \
      --physical-stuck-escape-reverse-mps "${RRT_PHYSICAL_STUCK_ESCAPE_REVERSE_MPS:-0.14}" \
      --physical-stuck-escape-angular-radps "${RRT_PHYSICAL_STUCK_ESCAPE_ANGULAR_RADPS:-0.25}" \
      --physical-stuck-escape-reverse-s "${RRT_PHYSICAL_STUCK_ESCAPE_REVERSE_S:-0.80}" \
      --physical-stuck-escape-turn-s "${RRT_PHYSICAL_STUCK_ESCAPE_TURN_S:-0.80}" \
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
      --adaptive-sample-radius \
      --adaptive-tight-sample-radius-m "${RRT_ADAPTIVE_TIGHT_SAMPLE_RADIUS_M:-0.60}" \
      --adaptive-wide-sample-radius-m "${RRT_ADAPTIVE_WIDE_SAMPLE_RADIUS_M:-1.00}" \
      --adaptive-wide-clearance-m "${RRT_ADAPTIVE_WIDE_CLEARANCE_M:-0.55}" \
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
      --frontier-distance-weight "${RRT_FRONTIER_DISTANCE_WEIGHT:-0.25}" \
      --frontier-size-weight "${RRT_FRONTIER_SIZE_WEIGHT:-1.0}" \
      --frontier-unknown-weight "${RRT_FRONTIER_UNKNOWN_WEIGHT:-0.35}" \
      --frontier-unknown-gain-radius-m "${RRT_FRONTIER_UNKNOWN_GAIN_RADIUS_M:-0.35}" \
      --free-roam-unknown-weight "${RRT_FREE_ROAM_UNKNOWN_WEIGHT:-0.30}" \
      --free-roam-distance-weight "${RRT_FREE_ROAM_DISTANCE_WEIGHT:-0.20}" \
      --free-roam-unknown-gain-radius-m "${RRT_FREE_ROAM_UNKNOWN_GAIN_RADIUS_M:-0.40}" \
      --goal-result-timeout-s "${RRT_GOAL_TIMEOUT_S:-35}" \
      --goal-send-timeout-s "${RRT_GOAL_SEND_TIMEOUT_S:-8}" \
      --goal-progress-timeout-s "${RRT_GOAL_PROGRESS_TIMEOUT_S:-12}" \
      --goal-progress-grace-s "${RRT_GOAL_PROGRESS_GRACE_S:-5}" \
      --goal-progress-epsilon-m "${RRT_GOAL_PROGRESS_EPSILON_M:-0.03}" \
      --physical-stuck-cmd-topic "${RRT_PHYSICAL_STUCK_CMD_TOPIC:-/cmd_vel_guarded}" \
      --physical-stuck-odom-topic "${RRT_PHYSICAL_STUCK_ODOM_TOPIC:-/odom}" \
      --physical-stuck-cmd-linear-mps "${RRT_PHYSICAL_STUCK_CMD_LINEAR_MPS:-0.10}" \
      --physical-stuck-odom-linear-mps "${RRT_PHYSICAL_STUCK_ODOM_LINEAR_MPS:-0.02}" \
      --physical-stuck-s "${RRT_PHYSICAL_STUCK_S:-2.0}" \
      --physical-stuck-escape-cmd-topic "${RRT_PHYSICAL_STUCK_ESCAPE_CMD_TOPIC:-/cmd_vel_raw}" \
      --physical-stuck-escape-reverse-mps "${RRT_PHYSICAL_STUCK_ESCAPE_REVERSE_MPS:-0.14}" \
      --physical-stuck-escape-angular-radps "${RRT_PHYSICAL_STUCK_ESCAPE_ANGULAR_RADPS:-0.25}" \
      --physical-stuck-escape-reverse-s "${RRT_PHYSICAL_STUCK_ESCAPE_REVERSE_S:-0.80}" \
      --physical-stuck-escape-turn-s "${RRT_PHYSICAL_STUCK_ESCAPE_TURN_S:-0.80}" \
      --failure-backoff-after "${RRT_FAILURE_BACKOFF_AFTER:-8}" \
      --failure-backoff-s "${RRT_FAILURE_BACKOFF_S:-5}" \
      --replan-sleep-s "${RRT_REPLAN_SLEEP_S:-2.5}" \
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
      --sample-radius-m "${RRT_SAMPLE_RADIUS_M:-0.50}" \
      --adaptive-sample-radius \
      --adaptive-tight-sample-radius-m "${RRT_ADAPTIVE_TIGHT_SAMPLE_RADIUS_M:-0.50}" \
      --adaptive-wide-sample-radius-m "${RRT_ADAPTIVE_WIDE_SAMPLE_RADIUS_M:-0.50}" \
      --adaptive-wide-clearance-m "${RRT_ADAPTIVE_WIDE_CLEARANCE_M:-0.15}" \
      --min-goal-distance-m "${RRT_MIN_GOAL_DISTANCE_M:-0.30}" \
      --inflation-m "${RRT_INFLATION_M:-0.12}" \
      --frontier-standoff-m "${RRT_FRONTIER_STANDOFF_M:-0.10}" \
      --frontier-backoffs-m "${RRT_FRONTIER_BACKOFFS_M:-0.10,0.18,0.25,0.35}" \
      --goal-clearance-check-m "${RRT_GOAL_CLEARANCE_CHECK_M:-0.45}" \
      --min-goal-clearance-m "${RRT_MIN_GOAL_CLEARANCE_M:-0.24}" \
      --goal-separation-m "${RRT_GOAL_SEPARATION_M:-0.12}" \
      --map-edge-margin-m "${RRT_MAP_EDGE_MARGIN_M:-0.10}" \
      --rejected-goal-separation-m "${RRT_REJECTED_GOAL_SEPARATION_M:-0.25}" \
      --frontier-mode "${RRT_FRONTIER_MODE:-hybrid}" \
      --wfd-max-cells "${RRT_WFD_MAX_CELLS:-12000}" \
      --min-frontier-cluster-cells "${RRT_MIN_FRONTIER_CLUSTER_CELLS:-2}" \
      --frontier-cluster-candidate-limit "${RRT_FRONTIER_CLUSTER_CANDIDATE_LIMIT:-8}" \
      --frontier-distance-weight "${RRT_FRONTIER_DISTANCE_WEIGHT:-1.0}" \
      --frontier-size-weight "${RRT_FRONTIER_SIZE_WEIGHT:-0.05}" \
      --frontier-unknown-weight "${RRT_FRONTIER_UNKNOWN_WEIGHT:-0.0}" \
      --frontier-unknown-gain-radius-m "${RRT_FRONTIER_UNKNOWN_GAIN_RADIUS_M:-0.35}" \
      --free-roam-unknown-weight "${RRT_FREE_ROAM_UNKNOWN_WEIGHT:-0.0}" \
      --free-roam-distance-weight "${RRT_FREE_ROAM_DISTANCE_WEIGHT:-0.20}" \
      --free-roam-unknown-gain-radius-m "${RRT_FREE_ROAM_UNKNOWN_GAIN_RADIUS_M:-0.40}" \
      --goal-result-timeout-s "${RRT_GOAL_TIMEOUT_S:-45}" \
      --goal-send-timeout-s "${RRT_GOAL_SEND_TIMEOUT_S:-8}" \
      --goal-progress-timeout-s "${RRT_GOAL_PROGRESS_TIMEOUT_S:-12}" \
      --goal-progress-grace-s "${RRT_GOAL_PROGRESS_GRACE_S:-5}" \
      --goal-progress-epsilon-m "${RRT_GOAL_PROGRESS_EPSILON_M:-0.03}" \
      --physical-stuck-cmd-angular-radps "${RRT_PHYSICAL_STUCK_CMD_ANGULAR_RADPS:-0.20}" \
      --physical-stuck-odom-angular-radps "${RRT_PHYSICAL_STUCK_ODOM_ANGULAR_RADPS:-0.05}" \
      --failure-backoff-after "${RRT_FAILURE_BACKOFF_AFTER:-8}" \
      --failure-backoff-s "${RRT_FAILURE_BACKOFF_S:-5}" \
      --start-free-search-m "${RRT_START_FREE_SEARCH_M:-0.45}" \
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
      --sample-radius-m "${RRT_SAMPLE_RADIUS_M:-0.50}" \
      --adaptive-sample-radius \
      --adaptive-tight-sample-radius-m "${RRT_ADAPTIVE_TIGHT_SAMPLE_RADIUS_M:-0.50}" \
      --adaptive-wide-sample-radius-m "${RRT_ADAPTIVE_WIDE_SAMPLE_RADIUS_M:-0.50}" \
      --adaptive-wide-clearance-m "${RRT_ADAPTIVE_WIDE_CLEARANCE_M:-0.15}" \
      --min-goal-distance-m "${RRT_MIN_GOAL_DISTANCE_M:-0.30}" \
      --inflation-m "${RRT_INFLATION_M:-0.12}" \
      --frontier-standoff-m "${RRT_FRONTIER_STANDOFF_M:-0.10}" \
      --frontier-backoffs-m "${RRT_FRONTIER_BACKOFFS_M:-0.10,0.18,0.25,0.35}" \
      --goal-clearance-check-m "${RRT_GOAL_CLEARANCE_CHECK_M:-0.45}" \
      --min-goal-clearance-m "${RRT_MIN_GOAL_CLEARANCE_M:-0.24}" \
      --goal-separation-m "${RRT_GOAL_SEPARATION_M:-0.12}" \
      --map-edge-margin-m "${RRT_MAP_EDGE_MARGIN_M:-0.10}" \
      --rejected-goal-separation-m "${RRT_REJECTED_GOAL_SEPARATION_M:-0.25}" \
      --frontier-mode "${RRT_FRONTIER_MODE:-hybrid}" \
      --wfd-max-cells "${RRT_WFD_MAX_CELLS:-12000}" \
      --min-frontier-cluster-cells "${RRT_MIN_FRONTIER_CLUSTER_CELLS:-2}" \
      --frontier-cluster-candidate-limit "${RRT_FRONTIER_CLUSTER_CANDIDATE_LIMIT:-8}" \
      --frontier-distance-weight "${RRT_FRONTIER_DISTANCE_WEIGHT:-1.0}" \
      --frontier-size-weight "${RRT_FRONTIER_SIZE_WEIGHT:-0.05}" \
      --frontier-unknown-weight "${RRT_FRONTIER_UNKNOWN_WEIGHT:-0.0}" \
      --frontier-unknown-gain-radius-m "${RRT_FRONTIER_UNKNOWN_GAIN_RADIUS_M:-0.35}" \
      --free-roam-unknown-weight "${RRT_FREE_ROAM_UNKNOWN_WEIGHT:-0.0}" \
      --free-roam-distance-weight "${RRT_FREE_ROAM_DISTANCE_WEIGHT:-0.20}" \
      --free-roam-unknown-gain-radius-m "${RRT_FREE_ROAM_UNKNOWN_GAIN_RADIUS_M:-0.40}" \
      --goal-result-timeout-s "${RRT_GOAL_TIMEOUT_S:-45}" \
      --goal-send-timeout-s "${RRT_GOAL_SEND_TIMEOUT_S:-8}" \
      --goal-progress-timeout-s "${RRT_GOAL_PROGRESS_TIMEOUT_S:-12}" \
      --goal-progress-grace-s "${RRT_GOAL_PROGRESS_GRACE_S:-5}" \
      --goal-progress-epsilon-m "${RRT_GOAL_PROGRESS_EPSILON_M:-0.03}" \
      --failure-backoff-after "${RRT_FAILURE_BACKOFF_AFTER:-8}" \
      --failure-backoff-s "${RRT_FAILURE_BACKOFF_S:-5}" \
      --replan-sleep-s "${RRT_REPLAN_SLEEP_S:-2.5}" \
      --start-free-search-m "${RRT_START_FREE_SEARCH_M:-0.45}" \
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
      --sample-radius-m "${RRT_SAMPLE_RADIUS_M:-0.50}" \
      --adaptive-sample-radius \
      --adaptive-tight-sample-radius-m "${RRT_ADAPTIVE_TIGHT_SAMPLE_RADIUS_M:-0.50}" \
      --adaptive-wide-sample-radius-m "${RRT_ADAPTIVE_WIDE_SAMPLE_RADIUS_M:-0.50}" \
      --adaptive-wide-clearance-m "${RRT_ADAPTIVE_WIDE_CLEARANCE_M:-0.15}" \
      --min-goal-distance-m "${RRT_MIN_GOAL_DISTANCE_M:-0.30}" \
      --inflation-m "${RRT_INFLATION_M:-0.12}" \
      --frontier-standoff-m "${RRT_FRONTIER_STANDOFF_M:-0.10}" \
      --frontier-backoffs-m "${RRT_FRONTIER_BACKOFFS_M:-0.10,0.18,0.25,0.35}" \
      --goal-clearance-check-m "${RRT_GOAL_CLEARANCE_CHECK_M:-0.45}" \
      --min-goal-clearance-m "${RRT_MIN_GOAL_CLEARANCE_M:-0.24}" \
      --goal-separation-m "${RRT_GOAL_SEPARATION_M:-0.12}" \
      --recent-goal-memory "${RRT_RECENT_GOAL_MEMORY:-8}" \
      --recent-goal-cooldown-s "${RRT_RECENT_GOAL_COOLDOWN_S:-30}" \
      --rejected-goal-memory "${RRT_REJECTED_GOAL_MEMORY:-60}" \
      --rejected-goal-cooldown-s "${RRT_REJECTED_GOAL_COOLDOWN_S:-30}" \
      --rejected-goal-separation-m "${RRT_REJECTED_GOAL_SEPARATION_M:-0.25}" \
      --free-roam-when-no-frontier \
      --free-roam-min-distance-m "${RRT_FREE_ROAM_MIN_DISTANCE_M:-0.15}" \
      --map-edge-margin-m "${RRT_MAP_EDGE_MARGIN_M:-0.10}" \
      --frontier-mode "${RRT_FRONTIER_MODE:-hybrid}" \
      --wfd-max-cells "${RRT_WFD_MAX_CELLS:-12000}" \
      --min-frontier-cluster-cells "${RRT_MIN_FRONTIER_CLUSTER_CELLS:-2}" \
      --frontier-cluster-candidate-limit "${RRT_FRONTIER_CLUSTER_CANDIDATE_LIMIT:-8}" \
      --frontier-distance-weight "${RRT_FRONTIER_DISTANCE_WEIGHT:-1.0}" \
      --frontier-size-weight "${RRT_FRONTIER_SIZE_WEIGHT:-0.05}" \
      --frontier-unknown-weight "${RRT_FRONTIER_UNKNOWN_WEIGHT:-0.0}" \
      --frontier-unknown-gain-radius-m "${RRT_FRONTIER_UNKNOWN_GAIN_RADIUS_M:-0.35}" \
      --free-roam-unknown-weight "${RRT_FREE_ROAM_UNKNOWN_WEIGHT:-0.0}" \
      --free-roam-distance-weight "${RRT_FREE_ROAM_DISTANCE_WEIGHT:-0.20}" \
      --free-roam-unknown-gain-radius-m "${RRT_FREE_ROAM_UNKNOWN_GAIN_RADIUS_M:-0.40}" \
      --goal-result-timeout-s "${RRT_GOAL_TIMEOUT_S:-45}" \
      --goal-send-timeout-s "${RRT_GOAL_SEND_TIMEOUT_S:-8}" \
      --goal-progress-timeout-s "${RRT_GOAL_PROGRESS_TIMEOUT_S:-30}" \
      --goal-progress-grace-s "${RRT_GOAL_PROGRESS_GRACE_S:-5}" \
      --goal-progress-epsilon-m "${RRT_GOAL_PROGRESS_EPSILON_M:-0.03}" \
      --physical-stuck-cmd-angular-radps "${RRT_PHYSICAL_STUCK_CMD_ANGULAR_RADPS:-0.20}" \
      --physical-stuck-odom-angular-radps "${RRT_PHYSICAL_STUCK_ODOM_ANGULAR_RADPS:-0.05}" \
      --failure-backoff-after "${RRT_FAILURE_BACKOFF_AFTER:-8}" \
      --failure-backoff-s "${RRT_FAILURE_BACKOFF_S:-5}" \
      --replan-sleep-s "${RRT_REPLAN_SLEEP_S:-2.5}" \
      --no-frontier-retry-s "${RRT_NO_FRONTIER_RETRY_S:-10}" \
      --start-free-search-m "${RRT_START_FREE_SEARCH_M:-0.45}" \
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
      --passive-close-confirm-distance-m "${RISK_PASSIVE_CLOSE_CONFIRM_DISTANCE_M:-0.40}" \
      --min-depth-m "${YOLO_MIN_DEPTH_M:-0.20}" \
      --max-depth-m "${YOLO_MAX_DEPTH_M:-1.20}" \
      --map-frame "${RISK_MAP_FRAME:-map}" \
      --odom-frame "${RISK_ODOM_FRAME:-odom}" \
      --tf-lookup-timeout-s "${RISK_TF_LOOKUP_TIMEOUT_S:-0.05}" \
      --pose-cache-duration-s "${RISK_POSE_CACHE_DURATION_S:-3.0}" \
      --pose-sample-hz "${RISK_POSE_SAMPLE_HZ:-10.0}" \
      --pose-max-age-s "${RISK_POSE_MAX_AGE_S:-0.20}" \
      --alarm-topic /perception/risk_alarm \
      --auto-risk-gates "${AUTO_RISK_GATES:-crack:0.60:0.20:1.20,corrosion:0.60:0.20:1.20,leakage:0.60:0.20:1.20,blockage:0.60:0.20:1.20}" \
      --approach-risk-gates "${APPROACH_RISK_GATES:-crack:0.15:0.20:1.20,corrosion:0.15:0.20:1.20,leakage:0.15:0.20:1.20,blockage:0.15:0.20:1.20}" \
      --dedup-map-grid-m "${RISK_DEDUP_MAP_GRID_M:-0.20}" \
      --risk-fusion-distance-m "${RISK_FUSION_DISTANCE_M:-0.25}" \
      --risk-fusion-time-s "${RISK_FUSION_TIME_S:-2.0}" \
      --risk-fusion-required "${RISK_FUSION_REQUIRED:-2}" \
      --risk-fusion-window "${RISK_FUSION_WINDOW:-3}" \
      --arm-response-mode disabled \
      --map-write-policy "${RISK_MAP_WRITE_POLICY:-approach_confirmed}" \
      --output-dir "${RUN_DIR}/yolo_risk"
    ;;

  risk-approach)
    source_ros
    ensure_run_dir
    YOLO_FRAME_SOURCE="${YOLO_FRAME_SOURCE:-realsense}"
    YOLO_PROVIDER="${YOLO_PROVIDER:-spacemit}"
    if [[ -z "${YOLO_MODEL:-}" ]]; then
      if [[ "${YOLO_PROVIDER}" == "cpu" ]]; then
        YOLO_MODEL="models/risk_vision/yolov8n_480x640_fp32_blockage03.onnx"
      else
        YOLO_MODEL="models/risk_vision/yolov8n_480x640_q_truncated6_balanced_blockage03.onnx"
      fi
    fi
    REALSENSE_PREFIX="${REALSENSE_PREFIX:-/home/soc/.local/realsense2-2.55.1}"
    REALSENSE_PYTHONPATH="${REALSENSE_PYTHONPATH:-${REALSENSE_PREFIX}/lib/python3.12/site-packages}"
    pkill -INT -f 'run_prelim_remote_mapping_yolo_arm_demo.py' 2>/dev/null || true
    pkill -INT -f 'run_real_k1_risk_approach_from_event.py' 2>/dev/null || true
    pkill -INT -f 'realsense2_camera.*rs_launch.py' 2>/dev/null || true
    pkill -INT -f 'realsense2_camera_node' 2>/dev/null || true
    sleep 3
    pkill -f 'run_prelim_remote_mapping_yolo_arm_demo.py' 2>/dev/null || true
    pkill -f 'run_real_k1_risk_approach_from_event.py' 2>/dev/null || true
    pkill -f 'realsense2_camera.*rs_launch.py' 2>/dev/null || true
    pkill -f 'realsense2_camera_node' 2>/dev/null || true

    if [[ "${YOLO_FRAME_SOURCE}" == "ros" ]]; then
      echo "[risk-approach] starting D435 ROS at 640x480x15"
      nohup ros2 launch realsense2_camera rs_launch.py \
        depth_module.depth_profile:=640,480,15 \
        depth_module.infra_profile:=640,480,15 \
        rgb_camera.color_profile:=640,480,15 \
        > "${RUN_DIR}/d435_15fps.log" 2>&1 < /dev/null &
      echo "$!" > "${RUN_DIR}/d435_15fps.pid"
      echo "[risk-approach] waiting for D435 topics"
      sleep "${D435_START_WAIT_S:-8}"
    else
      echo "[risk-approach] D435 is read directly by YOLO; raw image DDS is disabled"
    fi

    echo "[risk-approach] starting headless YOLO risk mapper provider=${YOLO_PROVIDER} model=${YOLO_MODEL}"
    nohup env \
      PYTHONUNBUFFERED=1 \
      PYTHONPATH="${REALSENSE_PYTHONPATH}:${PYTHONPATH:-}" \
      LD_LIBRARY_PATH="${REALSENSE_PREFIX}/lib:${LD_LIBRARY_PATH:-}" \
      python3 tools/run_prelim_remote_mapping_yolo_arm_demo.py \
      --provider "${YOLO_PROVIDER}" \
      --frame-source "${YOLO_FRAME_SOURCE}" \
      --realsense-width "${D435_WIDTH:-640}" \
      --realsense-height "${D435_HEIGHT:-480}" \
      --realsense-fps "${D435_FPS:-15}" \
      --realsense-frame-slots "${D435_FRAME_SLOTS:-3}" \
      --model "${YOLO_MODEL}" \
      --imgsz "${YOLO_IMGSZ:-640}" \
      --conf "${YOLO_CONF:-0.15}" \
      --iou "${YOLO_IOU:-0.45}" \
      --max-det "${YOLO_MAX_DET:-10}" \
      --inference-period-s "${YOLO_INFERENCE_PERIOD_S:-1.0}" \
      --passive-close-confirm-distance-m "${RISK_PASSIVE_CLOSE_CONFIRM_DISTANCE_M:-0.40}" \
      --opencv-num-threads "${YOLO_OPENCV_NUM_THREADS:-1}" \
      --ort-intra-op-threads "${YOLO_ORT_INTRA_OP_THREADS:-1}" \
      --ort-inter-op-threads "${YOLO_ORT_INTER_OP_THREADS:-1}" \
      --min-depth-m "${YOLO_MIN_DEPTH_M:-0.20}" \
      --max-depth-m "${YOLO_MAX_DEPTH_M:-1.20}" \
      --map-frame "${RISK_MAP_FRAME:-map}" \
      --odom-frame "${RISK_ODOM_FRAME:-odom}" \
      --tf-lookup-timeout-s "${RISK_TF_LOOKUP_TIMEOUT_S:-0.05}" \
      --pose-cache-duration-s "${RISK_POSE_CACHE_DURATION_S:-3.0}" \
      --pose-sample-hz "${RISK_POSE_SAMPLE_HZ:-10.0}" \
      --pose-max-age-s "${RISK_POSE_MAX_AGE_S:-0.20}" \
      --event-topic /perception/mock_event \
      --demo-event-topic /prelim_demo/risk_event \
      --alarm-topic /perception/risk_alarm \
      --auto-risk-gates "${AUTO_RISK_GATES:-crack:0.60:0.20:1.20,corrosion:0.60:0.20:1.20,leakage:0.60:0.20:1.20,blockage:0.60:0.20:1.20}" \
      --approach-risk-gates "${APPROACH_RISK_GATES:-crack:0.15:0.20:1.20,corrosion:0.15:0.20:1.20,leakage:0.15:0.20:1.20,blockage:0.15:0.20:1.20}" \
      --dedup-map-grid-m "${RISK_DEDUP_MAP_GRID_M:-0.20}" \
      --max-risk-candidates "${RISK_MAX_CANDIDATES:-64}" \
      --risk-candidate-ttl-s "${RISK_CANDIDATE_TTL_S:-60}" \
      --risk-index-flush-period-s "${RISK_INDEX_FLUSH_PERIOD_S:-5}" \
      --risk-fusion-distance-m "${RISK_FUSION_DISTANCE_M:-0.25}" \
      --risk-fusion-time-s "${RISK_FUSION_TIME_S:-2.0}" \
      --risk-fusion-required "${RISK_FUSION_REQUIRED:-2}" \
      --risk-fusion-window "${RISK_FUSION_WINDOW:-3}" \
      --arm-response-mode disabled \
      --map-write-policy "${RISK_MAP_WRITE_POLICY:-approach_confirmed}" \
      --no-visuals \
      --output-dir "${RUN_DIR}/yolo_risk" \
      > "${RUN_DIR}/yolo_risk_headless.log" 2>&1 < /dev/null &
    echo "$!" > "${RUN_DIR}/yolo_risk_headless.pid"

    echo "[risk-approach] starting event-driven Nav2 approach"
    nohup env PYTHONUNBUFFERED=1 python3 tools/run_real_k1_risk_approach_from_event.py \
      --event-topic /prelim_demo/risk_event \
      --status-topic /risk/approach_status \
      --odom-topic /odom \
      --goal-frame "${RISK_APPROACH_GOAL_FRAME:-odom}" \
      --stand-off-m "${RISK_APPROACH_STAND_OFF_M:-0.40}" \
      --final-stand-off-m "${RISK_FINAL_STAND_OFF_M:-0.20}" \
      --final-creep-cmd-topic "${RISK_FINAL_CREEP_CMD_TOPIC:-/cmd_vel_raw}" \
      --final-creep-linear-x "${RISK_FINAL_CREEP_LINEAR_X:-0.04}" \
      --final-creep-max-s "${RISK_FINAL_CREEP_MAX_S:-5.0}" \
      --already-near-margin-m "${RISK_APPROACH_NEAR_MARGIN_M:-0.03}" \
      --passive-close-confirm-distance-m "${RISK_PASSIVE_CLOSE_CONFIRM_DISTANCE_M:-0.40}" \
      --min-confidence "${RISK_APPROACH_MIN_CONFIDENCE:-0.20}" \
      --confirm-map-confidence "${RISK_CONFIRM_MAP_CONFIDENCE:-0.60}" \
      --max-events "${RISK_APPROACH_MAX_EVENTS:-1000000}" \
      --nav2-wait-s "${RISK_APPROACH_NAV2_WAIT_S:-2.0}" \
      --interrupt-rrt-on-event \
      --resume-rrt-after-event \
      --arm-simulation-mode "${RISK_ARM_SIMULATION_MODE:-semantic_only}" \
      --arm-simulation-s "${RISK_ARM_SIMULATION_S:-1.0}" \
      --resume-rrt-runtime-s "${RRT_RUNTIME_S:-86400}" \
      --resume-rrt-max-goals "${RRT_MAX_GOALS:-1000000}" \
      --resume-rrt-min-goal-clearance-m "${RRT_MIN_GOAL_CLEARANCE_M:-0.24}" \
      --resume-rrt-map-edge-margin-m "${RRT_MAP_EDGE_MARGIN_M:-0.10}" \
      --resume-rrt-min-goal-distance-m "${RRT_MIN_GOAL_DISTANCE_M:-0.30}" \
      --resume-rrt-free-roam-min-distance-m "${RRT_FREE_ROAM_MIN_DISTANCE_M:-0.15}" \
      --resume-rrt-start-free-search-m "${RRT_START_FREE_SEARCH_M:-0.45}" \
      --resume-rrt-replan-sleep-s "${RRT_REPLAN_SLEEP_S:-2.5}" \
      --close-confirm-usb-device "${RISK_CLOSE_USB_DEVICE:-/dev/video26}" \
      --close-confirm-width "${RISK_CLOSE_USB_WIDTH:-640}" \
      --close-confirm-height "${RISK_CLOSE_USB_HEIGHT:-480}" \
      --close-confirm-warmup-frames "${RISK_CLOSE_USB_WARMUP_FRAMES:-3}" \
      --close-confirm-capture-timeout-s "${RISK_CLOSE_USB_CAPTURE_TIMEOUT_S:-20}" \
      --close-confirm-infer-timeout-s "${RISK_CLOSE_USB_INFER_TIMEOUT_S:-45}" \
      --close-confirm-conf "${RISK_CLOSE_USB_CONF:-0.15}" \
      --close-confirm-model "${RISK_CLOSE_USB_MODEL:-models/risk_vision/yolov8n_480x640_q_truncated6_balanced_blockage03.onnx}" \
      --close-confirm-providers "${RISK_CLOSE_USB_PROVIDERS:-SpaceMITExecutionProvider,CPUExecutionProvider}" \
      --output-dir "${RUN_DIR}/risk_approach" \
      > "${RUN_DIR}/risk_approach.log" 2>&1 < /dev/null &
    echo "$!" > "${RUN_DIR}/risk_approach.pid"

    echo "[risk-approach] started"
    echo "[risk-approach] logs:"
    if [[ "${YOLO_FRAME_SOURCE}" == "ros" ]]; then
      echo "  ${RUN_DIR}/d435_15fps.log"
    fi
    echo "  ${RUN_DIR}/yolo_risk_headless.log"
    echo "  ${RUN_DIR}/risk_approach.log"
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
