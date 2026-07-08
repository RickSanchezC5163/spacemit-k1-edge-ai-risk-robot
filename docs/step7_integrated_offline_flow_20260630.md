# Step7 Integrated Offline Flow

Date: 2026-06-30

## Scope

Step7-A is an offline integrated rule-flow rehearsal:

```text
P4-Y2 guarded policy stress-stop and final map evidence
-> P4-X stationary D435 evidence
-> Map-A0 approximate risk map points
-> D435 simulated trigger rule
-> Arm-C0 simulated no-load arm trigger rule
-> episode_report.json
-> deterministic LLM-A report export
```

This stage does not start ROS, publish `cmd_vel`, open serial ports, or control
mechanical-arm hardware. It does not create new D435 captures. It consumes
existing evidence and verifies that the rule chain can be represented in one
auditable `episode_report.json`.

## Inputs

- `logs/policy_p4w_run_branch_mixed_20260629_183731.json`
- `maps/policy_p4w_branch_mixed_20260629_183731_final_marked.png`
- `docs/p4_guarded_policy_executable_modes_20260629.md`
- `edge-ai-robot-k1-p4-y2-7step-guarded-stress-58399be.bundle`
- `outputs/p4x_d435_hold_capture_v1/episode_report.json`
- `outputs/map_a_risk_point_projection_v1/offline_p4x/risk_map_points.json`
- `outputs/arm_c0_map_to_arm_dryrun_v1/offline_p4x/map_gated_arm_candidates.json`

## Outputs

Default output directory:

```text
outputs/step7_integrated_offline_flow_v1/offline_p4x_arm_c0/
```

Files:

- `episode_report.json`
- `step7_flow_summary.json`
- `step7_trigger_trace.csv`
- `step7_integrated_report.md`
- `errors.json`
- `README.md`

LLM-A report directory:

```text
outputs/llm_a_risk_report_v1/step7_integrated_offline_flow/
```

## Planning And Mapping Rule

Step7-A consumes P4-Y2 as the upstream guarded policy and map-save evidence
source. P4-Y2 is not treated as "7 steps completed"; it is treated as a useful
guarded stress-test result where the safety policy stopped early:

- requested policy max steps: `7`
- actual step count: `3`
- executed motion steps: `2`
- stop reason: `max_consecutive_fast_arc_reached`
- Step1: `ARC_FAST_RIGHT`, yaw delta about `-21.26 deg`
- Step2: `ARC_FAST_RIGHT`, yaw delta about `-27.07 deg`
- final `base_zero_ok=true`
- `final_map_saved=true`
- critical map save executed
- cumulative positive forward motion: about `0.1242 m`, below `1.0 m`
- hard stop was not triggered

Step7-A also consumes Map-A0 output as the risk-point mapping evidence source:

- `projection_mode=approximate_static_camera_offset`
- `tf_validated=false`
- `slam_used=false`
- `navigation_used=false`

This Step7 offline runner does not run SLAM, Nav2, Gazebo, or a live planner.
The output therefore must keep:

- `path_planning_executed=false`
- `autonomous_navigation_executed=false`
- `map_built_in_this_step=false`

## D435 Simulated Trigger Rule

For each Map-A0 risk point, Step7-A checks:

- `projection_status == "projected"`
- existing P4-X visual evidence paths are present
- `depth_median_m <= depth_trigger_m`
- upstream P4-X `base_zero_ok_before=true`
- upstream P4-X `published_cmd_vel=false`

If all conditions pass, the D435 rule trigger is marked
`succeeded_dry_run`. No new RGB/depth capture is created.

## Arm Simulated Trigger Rule

For each D435 trigger, Step7-A checks the matching Arm-C0 candidate:

- candidate status is `succeeded_dry_run`
- selected action is `ARM_SAMPLE_NO_LOAD`
- selected sequence is `arm_b3_8_step_safety_adjusted_no_load_sample`
- `validated_no_load_action=true`
- `contact_allowed=false`
- `obstacle_removed=false`

If all conditions pass, the arm rule trigger is marked `succeeded_dry_run`.
No hardware execution is allowed in Step7-A:

- `hardware_executed=false`
- `serial_port_opened=false`
- `serial_bytes_written=0`
- `published_cmd_vel=false`

## Claim Boundary

Allowed claims:

- Step7-A offline integrated rule-flow validation completed.
- Existing P4-Y2 guarded policy stress-stop evidence was consumed as planning/mapping safety evidence.
- Existing P4-X stationary D435 evidence was consumed as the visual trigger source.
- Existing Map-A0 approximate risk map points were consumed as mapping evidence.
- Arm-C0 no-load candidates were consumed as simulated arm trigger outputs.
- Deterministic LLM-A can summarize the resulting `episode_report.json`.

Disallowed claims:

- no new live D435 capture claim
- no ROS started claim
- no `cmd_vel` publish claim
- no real mechanical-arm motion claim for Step7-A
- no grasping, contact, payload handling, or obstacle-removal claim
- no full autonomous navigation, path-planning success, or high-precision SLAM claim
- no LLM-control claim

## Next Step

Step7-B should be a separate simulation or K1 live guarded validation. It must
define live ROS process gates, map/navigation scope, fresh base-zero evidence,
and explicit no-contact arm gates before any hardware motion is allowed.
