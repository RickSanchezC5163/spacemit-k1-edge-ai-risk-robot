# Step7-E0 Event-Triggered Capture Arm Flow

## Goal

Step7-E0 validates a live event-triggered evidence chain:

```text
guarded micro-motion
-> N10P front_p10/front_min event
-> base_zero confirmation
-> D435 HOLD_CAPTURE
-> deterministic mock risk point
-> approximate Map-A0 projection
-> Arm-C0/Arm-C1 no-load dry-run by default
-> LLM-A deterministic report
```

This stage is intended to close the gap left by Step7-D: D435 HOLD_CAPTURE is no
longer simply appended after motion. It is gated by an explicit N10P front range
event extracted from the guarded policy report.

## Runner

```text
tools/run_step7e_event_triggered_capture_arm_flow.py
```

The runner delegates chassis motion to the existing P4-W/P4-Y guarded policy
runner. It does not publish directly to `/cmd_vel_guarded` and does not write
the chassis serial port.

Required chassis command path:

```text
/input_cmd_vel -> scan_safety_guard_node -> /cmd_vel_guarded
```

## Event Rule

The trigger source is N10P front range evidence from the guarded policy report:

```text
front_p10 < 0.80m
or
front_min < 0.70m
```

The runner writes:

- `event_triggered`
- `event_source=N10P_front_p10`
- `trigger_reason`
- `trigger_front_p10_m`
- `trigger_front_min_m`
- `trigger_step_id`
- `risk_trigger_source=N10P_front_p10_event`

If no event is found, the runner writes `failed_safe`, records the error, and
skips D435 HOLD_CAPTURE. It must not silently capture.

## Dry-Run Command

```bash
python3 tools/run_step7e_event_triggered_capture_arm_flow.py \
  --policy-steps 5 \
  --enable-guarded-motion \
  --confirm-guarded-micro-motion \
  --confirm-n10p-safety \
  --confirm-no-direct-cmd-vel \
  --dry-run-arm
```

Expected output root:

```text
outputs/step7e_event_triggered_capture_arm_flow_v1/e0_n10p_trigger_dryrun_001/
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

Step7-E0 should be frozen in dry-run form before any hardware extension.

## Required Evidence

- `episode_report.json`
- `event_trigger.json`
- `step7e_report.md`
- `errors.json`
- `guarded_motion/`
- `event_triggered_step7c_flow/d435_hold_capture/episode_report.json`
- `event_triggered_step7c_flow/map_projection/risk_map_points.json`
- `event_triggered_step7c_flow/arm_candidate/episode_report.json`
- `event_triggered_step7c_flow/arm_execution/episode_report.json`
- `llm_a_report/risk_report.md`

## Success Criteria

- `event_triggered=true`
- `base_zero_ok_before_capture=true`
- `d435_live_capture_executed=true`
- `risk_point_generated=true`
- `risk_map_points>=1`
- `projected>=1`
- `arm_candidate_selected=true`
- `errors.json=[]`
- LLM-A deterministic report generated

## Claim Boundary

Allowed:

- N10P/front_p10 event-triggered D435 HOLD_CAPTURE
- deterministic mock risk output
- approximate Map-A0 risk point projection
- Arm no-load dry-run response by default
- one explicit Arm-C1 no-load hardware response only in a separate confirmed run
- deterministic LLM-A report generation without online API

Disallowed:

- direct `/cmd_vel_guarded` publish
- direct chassis serial control
- autonomous navigation success
- path planning success
- high-precision SLAM or high-precision risk coordinates
- real visual detection accuracy
- grasping, contact, payload handling, or obstacle removal
- LLM control of the robot
