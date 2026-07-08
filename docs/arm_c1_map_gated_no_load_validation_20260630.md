# Arm-C1-H Map-Gated No-Load Validation

Date: 2026-06-30

Status: PASS. One supervised Arm-C1-H hardware run was executed. No second
candidate was executed and no repeat run was performed.

## Summary

Arm-C1-H validates the integrated chain:

```text
Map-A0 risk_map_point
-> Arm-C0 map-gated arm candidate
-> fresh K1 live base-zero evidence
-> Arm-C1 gate
-> validated Arm-B3 8-step no-load sequence
-> return to 6b
```

This stage only claims map-gated no-load integrated validation. It does not
claim grasping, contact, payload handling, obstacle removal, autonomous
navigation, path planning, SLAM accuracy, ROS arm executor validation, or LLM
control.

## Evidence Directories

- `outputs/arm_c1_base_zero_evidence_v1/live_precheck_003/`
- `outputs/arm_c1_map_gated_no_load_once_v1/dryrun_candidate_001_with_live_precheck_003/`
- `outputs/arm_c1_map_gated_no_load_once_v1/hw_candidate_001_with_live_precheck_003/`
- `outputs/llm_a_risk_report_v1/arm_c1_map_gated_no_load_once/`

## Fresh Base-Zero Evidence

Source:

```text
outputs/arm_c1_base_zero_evidence_v1/live_precheck_003/base_zero_evidence.json
```

Result:

- `evidence_type=live_base_zero_observation`
- `valid_for_arm_c1_hardware=true`
- `base_zero_ok_before_arm=true`
- `published_cmd_vel=false`
- `serial_port_opened=false`
- `serial_bytes_written=0`
- `errors.json=[]`

The live evidence was generated with the previously validated guarded stack
running on K1. It was used immediately for the dry-run and the single hardware
run.

## Dry-Run Gate Check

Source:

```text
outputs/arm_c1_map_gated_no_load_once_v1/dryrun_candidate_001_with_live_precheck_003/
```

Result:

- `status=succeeded_dry_run`
- `candidate_id=arm_c0_candidate_001`
- `base_zero_ok_before_arm=true`
- `hardware_executed=false`
- `serial_port_opened=false`
- `serial_bytes_written=0`
- `published_cmd_vel=false`
- `contact_allowed=false`
- `obstacle_removed=false`
- `errors.json=[]`

The dry-run confirmed candidate and base-zero gates before hardware execution.

## Hardware Run

Source:

```text
outputs/arm_c1_map_gated_no_load_once_v1/hw_candidate_001_with_live_precheck_003/
```

Result:

- `status=succeeded`
- `candidate_id=arm_c0_candidate_001`
- `selected_action=ARM_SAMPLE_NO_LOAD`
- `selected_sequence=arm_b3_8_step_safety_adjusted_no_load_sample`
- `step_count=8`
- `step_success_count=8`
- `base_zero_ok_before_arm=true`
- `hardware_executed=true`
- `serial_port_opened=true`
- `serial_bytes_written=180`
- `published_cmd_vel=false`
- `contact_allowed=false`
- `obstacle_removed=false`
- `errors.json=[]`

Operator physical confirmation:

- `physical_actuation_observed=true`
- `returned_to_6b_observed=true`
- `final_pose_observed=6b`
- `physical_issue_observed=false`
- `contact_observed=false`
- `abnormal_sound_observed=false`
- `binding_or_stall_observed=false`
- `visible_overheating_observed=false`
- `operator_notes="Operator confirmed: no abnormal issue observed; final pose returned to 6b."`

## LLM-A Report

Deterministic LLM-A report output:

```text
outputs/llm_a_risk_report_v1/arm_c1_map_gated_no_load_once/
```

Result:

- `episode_kind=arm_c1_map_gated_no_load_once`
- `status=PASS`
- `llm_used=false`
- `online_api_used=false`
- `local_model_used=false`

## Cleanup

The guarded stack started for live base-zero evidence was stopped after the
hardware run. No matching residual guarded stack or arm-run process remained.

## Claim Boundary

Allowed claim:

```text
Arm-C1-H completed one supervised map-gated no-load integrated validation:
after fresh live base-zero evidence passed, the system consumed one valid
Arm-C0 candidate and executed the already validated Arm-B3 8-step no-load
sequence once, with no cmd_vel publish, no contact, no obstacle removal, and
final return to 6b observed.
```

Disallowed claims:

- no grasping claim
- no contact claim
- no payload claim
- no obstacle removal claim
- no autonomous navigation claim
- no path planning claim
- no SLAM/high-precision map claim
- no ROS arm executor claim
- no LLM control claim
- no claim that live base-zero evidence can be reused after its freshness window

## Next Recommended Step

Freeze Arm-C1-H evidence. Do not proceed to contact, grasping, load handling, or
obstacle-removal tests until a separate Arm-D safety plan and hardware boundary
document are written.
