#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  tools/p4w_guarded_policy_branch_mixed.sh dry-run
  tools/p4w_guarded_policy_branch_mixed.sh run

Runs the P4-W guarded policy branch-mixed test on K1.

This script does not launch ROS nodes. Start the guarded mapping stack first.
It keeps all motion routed through /input_cmd_vel and scan_safety_guard_node.

Environment overrides:
  POLICY_ARC_DIRECTION=auto|left|right       default: auto
  POLICY_ARC_MODE=precise|fast               default: precise
  POLICY_MAX_CONSECUTIVE_FAST_ARC=1..3       default: 2
  POLICY_ARC_FAST_LINEAR=0..0.30             default: 0.12
  POLICY_ARC_FAST_ANGULAR=0..0.80            default: 0.80
  POLICY_ARC_FAST_DURATION_S=0.5..1.5        default: 1.0
  POLICY_CLOSE_ACTION=arc30|forward          default: arc30
  POLICY_MID_ACTION=arc30|forward            default: arc30
  POLICY_NORMAL_ACTION=forward|arc30         default: forward
  POLICY_MAX_STEPS=1..7                      default: 3
  POLICY_MAX_RUNTIME_S=10..180               default: 120
  POLICY_MAX_TOTAL_FORWARD_M=0.10..3.0       default: 1.0
  POLICY_DURATION_S=1..300                   default: 15
  POLICY_SAMPLES=0..300                      default: 8
  ZERO_HOLD_S=1..8                           max zero wait, default: 5.0
  ZERO_MIN_HOLD_S=0.2..ZERO_HOLD_S           default: 0.8
  ZERO_POLL_S=0.05..0.5                      default: 0.1
  ZERO_CONFIRM_SAMPLES=1..10                 default: 3
  SAVE_POLICY=every_step|every_n_steps|critical_or_end|pipelined_critical
                                             default: every_step
  SAVE_EVERY_N=1..10                         default: 2
  MAX_PENDING_SAVES=1                        default: 1
  CONSOLE_MODE=full|compact                  default: full
EOF
}

MODE="${1:-}"
if [[ "${MODE}" != "dry-run" && "${MODE}" != "run" ]]; then
  usage >&2
  exit 2
fi

REPO="${REPO:-${HOME}/edge-ai-robot-k1}"
cd "${REPO}"

set +u
source /opt/ros/humble/setup.bash
source "${HOME}/lslidar_ws/install/setup.bash"
source "${HOME}/edge-ai-robot-k1/ros2_ws/install/setup.bash"
set -u

mkdir -p logs maps
TS="$(date +%Y%m%d_%H%M%S)"

COMMON_ARGS=(
  --behavior-profile interaction_mode
  --policy-arc-direction "${POLICY_ARC_DIRECTION:-auto}"
  --policy-arc-mode "${POLICY_ARC_MODE:-precise}"
  --policy-max-consecutive-fast-arc "${POLICY_MAX_CONSECUTIVE_FAST_ARC:-2}"
  --policy-arc-fast-linear "${POLICY_ARC_FAST_LINEAR:-0.12}"
  --policy-arc-fast-angular "${POLICY_ARC_FAST_ANGULAR:-0.80}"
  --policy-arc-fast-duration-s "${POLICY_ARC_FAST_DURATION_S:-1.0}"
  --policy-close-action "${POLICY_CLOSE_ACTION:-arc30}"
  --policy-mid-action "${POLICY_MID_ACTION:-arc30}"
  --policy-normal-action "${POLICY_NORMAL_ACTION:-forward}"
  --policy-max-total-forward-m "${POLICY_MAX_TOTAL_FORWARD_M:-1.0}"
  --forward-fast-speed "${FORWARD_FAST_SPEED:-0.15}"
  --forward-mid-speed "${FORWARD_MID_SPEED:-0.12}"
  --forward-slow-speed "${FORWARD_SLOW_SPEED:-0.10}"
  --forward-brake-coef-s "${FORWARD_BRAKE_COEF_S:-1.05}"
  --forward-static-brake-margin-m "${FORWARD_STATIC_BRAKE_MARGIN_M:-0.02}"
  --forward-brake-margin-m "${FORWARD_BRAKE_MARGIN_M:-0.03}"
  --forward-timeout-s "${FORWARD_TIMEOUT_S:-5.0}"
  --zero-hold-s "${ZERO_HOLD_S:-5.0}"
  --zero-min-hold-s "${ZERO_MIN_HOLD_S:-0.8}"
  --zero-poll-s "${ZERO_POLL_S:-0.1}"
  --zero-confirm-samples "${ZERO_CONFIRM_SAMPLES:-3}"
  --save-policy "${SAVE_POLICY:-every_step}"
  --save-every-n "${SAVE_EVERY_N:-2}"
  --max-pending-saves "${MAX_PENDING_SAVES:-1}"
  --console-mode "${CONSOLE_MODE:-full}"
  --save-map-retries "${SAVE_MAP_RETRIES:-2}"
  --save-map-retry-delay-s "${SAVE_MAP_RETRY_DELAY_S:-2.0}"
  --confirm YES
)

if [[ "${MODE}" == "dry-run" ]]; then
  REPORT="${REPO}/logs/policy_p4w_precheck_branch_mixed_${TS}.json"
  RUN_LOG="${REPORT%.json}.run.log"
  python3 tools/guarded_auto_mapping_micro.py \
    --mode guarded-policy-dry-run \
    --policy-duration-s "${POLICY_DURATION_S:-15}" \
    --policy-samples "${POLICY_SAMPLES:-8}" \
    --policy-sample-period-s "${POLICY_SAMPLE_PERIOD_S:-1.0}" \
    --report "${REPORT}" \
    "${COMMON_ARGS[@]}" 2>&1 | tee "${RUN_LOG}"
else
  REPORT="${REPO}/logs/policy_p4w_run_branch_mixed_${TS}.json"
  RUN_LOG="${REPORT%.json}.run.log"
  MAP_PREFIX="${REPO}/maps/policy_p4w_branch_mixed_${TS}"
  python3 tools/guarded_auto_mapping_micro.py \
    --mode guarded-policy-run \
    --policy-max-steps "${POLICY_MAX_STEPS:-3}" \
    --policy-max-runtime-s "${POLICY_MAX_RUNTIME_S:-120}" \
    --map-prefix "${MAP_PREFIX}" \
    --report "${REPORT}" \
    "${COMMON_ARGS[@]}" 2>&1 | tee "${RUN_LOG}"
fi

echo "P4W_REPORT=${REPORT}"
echo "P4W_RUN_LOG=${RUN_LOG}"
