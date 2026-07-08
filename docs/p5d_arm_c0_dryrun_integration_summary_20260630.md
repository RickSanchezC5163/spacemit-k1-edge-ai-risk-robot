# P5-D / Arm-C0 Dry-Run Integration Summary

Date: 2026-06-30

## Scope

This document freezes the current dry-run evidence chain:

```text
P4-X D435 evidence
  -> Map-A0 offline risk point projection
  -> Arm-C0 map-gated arm no-load action candidates
  -> LLM-A deterministic risk report
```

No ROS process was started for Map-A0, Arm-C0, or this summary step. No
`cmd_vel` was published. No serial port was opened. No mechanical-arm hardware
was controlled.

## Evidence Inputs

P4-X frozen evidence:

- `outputs/p4x_d435_hold_capture_v1/episode_report.json`
- 10/10 `HOLD_CAPTURE` succeeded.
- `base_zero_ok_before=true` for P4-X captures.
- `published_cmd_vel=false`.
- RGB/depth/camera_info/odom/meta/risk_point evidence chain is complete.

Map-A0 evidence:

- `outputs/map_a_risk_point_projection_v1/offline_p4x/risk_map_points.json`
- `outputs/map_a_risk_point_projection_v1/offline_p4x/risk_map_points.csv`
- `outputs/map_a_risk_point_projection_v1/offline_p4x/risk_map_snapshot.png`
- `outputs/map_a_risk_point_projection_v1/offline_p4x/projection_report.md`
- `outputs/map_a_risk_point_projection_v1/offline_p4x/errors.json`

Arm-C0 evidence:

- `outputs/arm_c0_map_to_arm_dryrun_v1/offline_p4x/map_gated_arm_candidates.json`
- `outputs/arm_c0_map_to_arm_dryrun_v1/offline_p4x/map_gated_arm_candidates.csv`
- `outputs/arm_c0_map_to_arm_dryrun_v1/offline_p4x/arm_c0_dryrun_report.md`
- `outputs/arm_c0_map_to_arm_dryrun_v1/offline_p4x/episode_report.json`
- `outputs/arm_c0_map_to_arm_dryrun_v1/offline_p4x/errors.json`

LLM-A evidence:

- `outputs/llm_a_risk_report_v1/arm_c0_map_to_arm_dryrun/risk_report.md`
- `outputs/llm_a_risk_report_v1/arm_c0_map_to_arm_dryrun/risk_report.json`
- `outputs/llm_a_risk_report_v1/arm_c0_map_to_arm_dryrun/claim_boundary.md`
- `outputs/llm_a_risk_report_v1/arm_c0_map_to_arm_dryrun/README.md`

## Map-A0 Result

Map-A0 converted P4-X mock risk points into approximate local odom/map points:

- `risk_map_points=10`
- `projected=10`
- `missing_required_field=0`
- `errors.json=[]`
- `projection_mode=approximate_static_camera_offset`
- `tf_validated=false`
- `slam_used=false`
- `navigation_used=false`

The projection is intentionally approximate. It does not claim TF validation,
SLAM, autonomous navigation, path planning, or absolute high-precision risk
point coordinates.

## Arm-C0 Result

Arm-C0 consumed Map-A0 output and generated map-gated no-load action
candidates:

- `candidates=10`
- `succeeded_dry_run=10`
- `blocked=0`
- all candidates selected `ARM_SAMPLE_NO_LOAD`
- all candidates selected `arm_b3_8_step_safety_adjusted_no_load_sample`
- all candidates were classified as `front/far`
- `hardware_executed=false`
- `serial_port_opened=false`
- `serial_bytes_written=0`
- `published_cmd_vel=false`
- `contact_allowed=false`
- `obstacle_removed=false`
- `base_zero_required=true`
- `base_zero_checked=false`

`base_zero_checked=false` is correct for Arm-C0 because this stage is offline
candidate generation only. It must not be interpreted as a permission to move
the real arm.

## LLM-A Result

LLM-A generated a deterministic report from Arm-C0 `episode_report.json`:

- `episode_kind=arm_c0_map_to_arm_dryrun`
- `status=PASS_DRY_RUN`
- `action_count=10`
- `llm_used=false`
- `online_api_used=false`
- `local_model_used=false`

LLM-A is a rule-based report generator in this stage. It does not control the
robot and does not provide real online or local large-model reasoning.

## Integrated Claim

The integrated dry-run chain supports this claim:

```text
The system can transform stationary D435 risk evidence into approximate local
risk map points, derive map-gated no-load mechanical-arm action candidates, and
generate a deterministic audit report, all without starting ROS, publishing
cmd_vel, opening serial ports, or executing arm hardware.
```

## Claim Boundary

This stage may claim:

- safe stationary P4-X visual evidence exists as the upstream evidence source
- Map-A0 offline risk point projection completed
- Arm-C0 map-to-arm no-load action candidate dry-run completed
- LLM-A deterministic report generation completed
- no hardware execution occurred during Map-A0 / Arm-C0 / LLM-A

This stage must not claim:

- real mechanical-arm action based on the map
- obstacle removal
- grasping
- contact
- payload handling
- SLAM
- autonomous navigation
- path planning
- absolute high-precision risk point coordinates
- LLM control of the robot

## Readiness For Arm-C1

Arm-C1 is not part of this evidence chain. Before any Arm-C1 hardware step, the
system must separately verify:

- live `base_zero_ok_before_arm=true`
- no `cmd_vel` is published during the arm step
- serial write is explicitly enabled only for the Arm-C1 script invocation
- the selected candidate is from Arm-C0 and remains `succeeded_dry_run`
- only the validated no-load Arm-B3 sequence is allowed
- contact remains disallowed
- obstacle removal remains disallowed
- the arm returns to `6b` safe idle/home

## Next Step

Design Arm-C1 as a separate hardware-gated no-load validation. Do not merge it
with Arm-C0 dry-run reporting, and do not let map output or LLM-A directly drive
hardware.
