# Step7-E2 Guarded Motion Red-Rule Flow - 2026-06-30

## Goal

Step7-E2 validates a live guarded-motion red-rule evidence chain:

```text
guarded micro-motion
-> base_zero after motion
-> D435 live capture
-> deterministic HSV red-color rule
-> depth risk_point
-> approximate Map-A0 projection
-> Arm-C0 dry-run by default
-> LLM-A deterministic report
```

## Runner

```text
tools/run_step7e2_guarded_motion_red_rule_flow.py
```

The runner delegates chassis motion to the existing P4-W/P4-Y guarded policy
runner. It does not publish directly to `/cmd_vel_guarded` and does not write
the chassis serial port.

Required chassis command path:

```text
/input_cmd_vel -> scan_safety_guard_node -> /cmd_vel_guarded
```

The D435 red-rule subflow is delegated to:

```text
tools/run_step7e1_red_rule_stationary_flow.py
```

## Dry-Run Command

```bash
python3 tools/run_step7e2_guarded_motion_red_rule_flow.py \
  --policy-steps 5 \
  --enable-guarded-motion \
  --confirm-guarded-micro-motion \
  --confirm-n10p-safety \
  --confirm-no-direct-cmd-vel \
  --dry-run-arm
```

Expected output root:

```text
outputs/step7e2_guarded_motion_red_rule_flow_v1/
```

## Hardware Arm Gate

Arm hardware is disabled by default. A single Arm-C1 no-load hardware response
requires all of these flags:

```text
--enable-arm-hardware
--confirm-map-gated-no-load
--confirm-no-contact
--confirm-base-zero-live
--confirm-no-cmd-vel
```

Step7-E2 should be frozen in dry-run form before any hardware extension.

## Fastdemo Hardware Baseline

The current teacher-facing reference run is:

```text
outputs/step7e2_guarded_motion_red_rule_flow_v1/e2_guarded_red_rule_arm_hw_fastdemo_002/
```

Result summary:

- `status=succeeded`
- `guarded_motion_executed=true`
- `motion_command_path=/input_cmd_vel -> scan_safety_guard_node -> /cmd_vel_guarded`
- `direct_cmd_vel_bypass=false`
- `policy_executed_count=2`
- `policy_sequence_stop_reason=max_consecutive_fast_arc_reached`
- `cumulative_positive_forward_m=0.118`
- `base_zero_ok_after_motion=true`
- `demo_fast_reuse_policy_base_zero=true`
- `red_object_detected=true`
- `bbox_xywh=[93,250,275,117]`
- `depth_median_m=0.561`
- `risk_map_points=1`
- `projected=1`
- `arm_execution_status=succeeded`
- `hardware_executed=true`
- `serial_bytes_written=180`
- `published_cmd_vel_during_capture=false`
- `published_cmd_vel_during_arm=false`
- `contact_allowed=false`
- `obstacle_removed=false`
- `errors=[]`
- operator confirmed final return to `6b` with no abnormal issue observed

Fastdemo uses `--demo-fast-reuse-policy-base-zero` to reuse the immediately
preceding guarded policy final base-zero evidence. This is a latency reduction
for video demonstration only; it does not bypass the chassis safety guard and
does not publish `/cmd_vel_guarded` directly.

Reproduction details are frozen in:

```text
docs/step7e2_fastdemo_reproduction_20260630.md
```

## Required Evidence

- `episode_report.json`
- `step7e2_report.md`
- `errors.json`
- `guarded_motion/`
- `red_rule_after_motion/episode_report.json`
- `red_rule_after_motion/d435_red_rule_capture/episode_report.json`
- `red_rule_after_motion/map_projection/risk_map_points.json`
- `red_rule_after_motion/arm_candidate/episode_report.json`
- `red_rule_after_motion/arm_execution/episode_report.json`
- `llm_a_report/risk_report.md`

## Success Criteria

- `guarded_motion_executed=true`
- `base_zero_ok_after_motion=true`
- `red_object_detected=true`
- `risk_trigger_source=D435_red_color_rule`
- `d435_live_capture_executed=true`
- `risk_point_generated=true`
- `risk_map_points>=1`
- `projected>=1`
- `arm_candidate_selected=true`
- `hardware_executed=false` in dry-run mode
- `serial_bytes_written=0` in dry-run mode
- `published_cmd_vel_during_capture=false`
- `published_cmd_vel_during_arm=false`
- `errors.json=[]`
- LLM-A deterministic report generated

## Claim Boundary

Allowed:

- guarded micro-motion through the existing P4/N10P safety chain
- D435 deterministic red-color rule trigger after base-zero
- depth median and approximate risk point projection
- Arm-C0 dry-run no-load response by default
- deterministic LLM-A report generation without online API

Disallowed:

- direct `/cmd_vel_guarded` publish
- direct chassis serial control
- autonomous navigation success
- path planning success
- high-precision SLAM or high-precision risk coordinates
- trained visual model inference or visual detection accuracy
- grasping, contact, payload handling, or obstacle removal
- LLM control of the robot
