# Evidence Manifest

Date: 2026-06-29

This manifest records local evidence directories and claim boundaries. The full
`outputs/` tree is intentionally ignored by git and should be packaged
separately when needed.

## Evidence Overview

- `outputs/p4x_d435_hold_capture_v1/`
- `outputs/arm_a_mock_remove_obstacle_v1/`
- `outputs/arm_b1_send_home_once_v1/`
- `outputs/arm_b2_single_servo_no_load_v1/`
- `outputs/arm_b3_no_load_sample_sequence_v1/`
- `outputs/map_a_risk_point_projection_v1/offline_p4x/`
- `outputs/arm_c0_map_to_arm_dryrun_v1/offline_p4x/`
- `outputs/arm_c1_base_zero_evidence_v1/offline_from_p4x/`
- `outputs/arm_c1_base_zero_evidence_v1/live_precheck_001/`
- `outputs/arm_c1_base_zero_evidence_v1/live_precheck_002/`
- `outputs/arm_c1_base_zero_evidence_v1/live_precheck_003/`
- `outputs/arm_c1_map_gated_no_load_once_v1/dryrun_candidate_001/`
- `outputs/arm_c1_map_gated_no_load_once_v1/dryrun_candidate_001_with_offline_base_zero/`
- `outputs/arm_c1_map_gated_no_load_once_v1/dryrun_candidate_001_with_k1_live_base_zero_precheck_002/`
- `outputs/arm_c1_map_gated_no_load_once_v1/dryrun_candidate_001_with_live_precheck_003/`
- `outputs/arm_c1_map_gated_no_load_once_v1/hw_candidate_001_with_live_precheck_003/`
- `outputs/step7_integrated_offline_flow_v1/offline_p4x_arm_c0/`
- `outputs/step7b_live_stationary_flow_v1/step7b_live_stationary_20260630_130029/`
- `outputs/step7e2_guarded_motion_red_rule_flow_v1/e2_guarded_red_rule_negative_002/`
- `outputs/step7e2_guarded_motion_red_rule_flow_v1/e2_guarded_red_rule_arm_hw_fastdemo_002/`
- `outputs/llm_a_risk_report_v1/p4x/`
- `outputs/llm_a_risk_report_v1/arm_a/`
- `outputs/llm_a_risk_report_v1/arm_b3_hw_sequence_002/`
- `outputs/llm_a_risk_report_v1/arm_c0_map_to_arm_dryrun/`
- `outputs/llm_a_risk_report_v1/arm_c1_map_gated_no_load_once/`
- `outputs/llm_a_risk_report_v1/step7_integrated_offline_flow/`

Doc-level summaries:

- `docs/p5d_arm_c0_dryrun_integration_summary_20260630.md`
- `docs/arm_c1_base_zero_evidence_20260630.md`
- `docs/arm_c1_live_base_zero_precheck_20260630.md`
- `docs/arm_c1_map_gated_no_load_validation_20260630.md`
- `docs/arm_c1_hardware_gate_script_design_20260630.md`
- `docs/step7_integrated_offline_flow_20260630.md`
- `docs/step7b_live_stationary_flow_20260630.md`
- `docs/step7e2_guarded_motion_red_rule_flow_20260630.md`
- `docs/step7e2_fastdemo_reproduction_20260630.md`

## P4-X Evidence

Stage: P4-X D435 HOLD_CAPTURE.

Core files:

- `outputs/p4x_d435_hold_capture_v1/episode_report.json`
- `outputs/p4x_d435_hold_capture_v1/captures/<capture_id>/rgb.png`
- `outputs/p4x_d435_hold_capture_v1/captures/<capture_id>/depth_raw.npy`
- `outputs/p4x_d435_hold_capture_v1/captures/<capture_id>/depth_vis.png`
- `outputs/p4x_d435_hold_capture_v1/captures/<capture_id>/camera_info.json`
- `outputs/p4x_d435_hold_capture_v1/captures/<capture_id>/odom.json`
- `outputs/p4x_d435_hold_capture_v1/captures/<capture_id>/capture_meta.json`
- `outputs/p4x_d435_hold_capture_v1/captures/<capture_id>/risk_point.json`
- `outputs/p4x_d435_hold_capture_v1/P4X_FREEZE.md`
- `outputs/p4x_d435_hold_capture_v1/p4x_d435_hold_capture_summary.md`
- `outputs/p4x_d435_hold_capture_v1/claim_boundary_p4x.md`

Validation conclusion:

- 10/10 succeeded.
- 0 failed_safe.
- `base_zero_ok_before=True`.
- `published_cmd_vel=false`.
- RGB/depth/camera_info/odom/meta/risk_point evidence chain is complete.

Claim boundary:

- Only safe stationary visual evidence capture is claimed.
- Visual detection accuracy is not claimed.
- Real mechanical-arm operation is not claimed.
- Autonomous semantic reasoning is not claimed.
- P4-X3 new fields are code-ready only and are not claimed hardware-validated.

## Arm-A Evidence

Stage: Arm-A mock remove obstacle.

Core files:

- `outputs/arm_a_mock_remove_obstacle_v1/episode_report.json`
- `outputs/arm_a_mock_remove_obstacle_v1/arm_a_mock_status.csv`
- `outputs/arm_a_mock_remove_obstacle_v1/errors.json`
- `outputs/arm_a_mock_remove_obstacle_v1/README.md`
- `outputs/arm_a_mock_remove_obstacle_v1/actions/<action_id>/action_result.json`

Validation conclusion:

- 10/10 succeeded.
- 0 failed_safe.
- all `base_zero_ok_before=True`.
- all `published_cmd_vel=false`.
- all `mock=True`.
- all `obstacle_removed=True`.

Claim boundary:

- Only the mock action chain is claimed.
- Real mechanical-arm control is not claimed.
- Physical obstacle removal is not claimed.
- Grasping, pushing, contact, or obstacle interaction success is not claimed.

## LLM-A Evidence

Stage: LLM-A deterministic risk report baseline.

Core files:

- `outputs/llm_a_risk_report_v1/p4x/risk_report.md`
- `outputs/llm_a_risk_report_v1/p4x/risk_report.json`
- `outputs/llm_a_risk_report_v1/p4x/claim_boundary.md`
- `outputs/llm_a_risk_report_v1/arm_a/risk_report.md`
- `outputs/llm_a_risk_report_v1/arm_a/risk_report.json`
- `outputs/llm_a_risk_report_v1/arm_a/claim_boundary.md`
- `outputs/llm_a_risk_report_v1/arm_b3_hw_sequence_002/risk_report.md`
- `outputs/llm_a_risk_report_v1/arm_b3_hw_sequence_002/risk_report.json`
- `outputs/llm_a_risk_report_v1/arm_b3_hw_sequence_002/claim_boundary.md`
- `outputs/llm_a_risk_report_v1/arm_c0_map_to_arm_dryrun/risk_report.md`
- `outputs/llm_a_risk_report_v1/arm_c0_map_to_arm_dryrun/risk_report.json`
- `outputs/llm_a_risk_report_v1/arm_c0_map_to_arm_dryrun/claim_boundary.md`
- `outputs/llm_a_risk_report_v1/arm_c1_map_gated_no_load_once/risk_report.md`
- `outputs/llm_a_risk_report_v1/arm_c1_map_gated_no_load_once/risk_report.json`
- `outputs/llm_a_risk_report_v1/arm_c1_map_gated_no_load_once/claim_boundary.md`
- `outputs/llm_a_risk_report_v1/step7_integrated_offline_flow/risk_report.md`
- `outputs/llm_a_risk_report_v1/step7_integrated_offline_flow/risk_report.json`
- `outputs/llm_a_risk_report_v1/step7_integrated_offline_flow/claim_boundary.md`
- `outputs/step7e2_guarded_motion_red_rule_flow_v1/e2_guarded_red_rule_negative_002/llm_a_report/risk_report.md`
- `outputs/step7e2_guarded_motion_red_rule_flow_v1/e2_guarded_red_rule_negative_002/llm_a_report/risk_report.json`
- `outputs/step7e2_guarded_motion_red_rule_flow_v1/e2_guarded_red_rule_negative_002/llm_a_report/claim_boundary.md`
- `outputs/step7e2_guarded_motion_red_rule_flow_v1/e2_guarded_red_rule_arm_hw_fastdemo_002/llm_a_report/risk_report.md`
- `outputs/step7e2_guarded_motion_red_rule_flow_v1/e2_guarded_red_rule_arm_hw_fastdemo_002/llm_a_report/risk_report.json`
- `outputs/step7e2_guarded_motion_red_rule_flow_v1/e2_guarded_red_rule_arm_hw_fastdemo_002/llm_a_report/claim_boundary.md`

Validation conclusion:

- P4-X report PASS.
- Arm-A report PASS.
- Arm-B3 report PASS after `arm_b3_hw_sequence_002` generation.
- Arm-C0 report PASS_DRY_RUN for map-to-arm no-load action candidate generation.
- Arm-C1-H report PASS for one map-gated no-load integrated validation.
- Step7-A report PASS_DRY_RUN for offline planning/mapping, D435 trigger, and arm trigger rule-flow integration.
- Step7-B0 report PASS for live stationary base-zero, D435 capture, map projection, Arm-C0 dry-run, and local LLM-A report.
- Step7-E2 negative-control report PASS for guarded motion plus no-red D435 rule validation.
- Step7-E2 fastdemo report PASS for guarded motion, D435 red-rule trigger, approximate projection, and one Arm-C1 no-load hardware response.
- deterministic baseline.
- `llm_used=false`.
- `online_api_used=false`.
- `local_model_used=false`.

Claim boundary:

- Real LLM reasoning is not claimed.
- Local large-model deployment is not claimed.
- Online API calls are not claimed.
- Only rule-based `episode_report.json` to risk report generation is claimed.

## Map-A Evidence

Stage: Map-A0 offline risk point projection.

Core directory:

- `outputs/map_a_risk_point_projection_v1/offline_p4x/`

Core files:

- `outputs/map_a_risk_point_projection_v1/offline_p4x/risk_map_points.json`
- `outputs/map_a_risk_point_projection_v1/offline_p4x/risk_map_points.csv`
- `outputs/map_a_risk_point_projection_v1/offline_p4x/risk_map_snapshot.png`
- `outputs/map_a_risk_point_projection_v1/offline_p4x/projection_report.md`
- `outputs/map_a_risk_point_projection_v1/offline_p4x/errors.json`

Validation conclusion:

- `risk_map_points=10`.
- `projected=10`.
- `missing_required_field=0`.
- `errors.json=[]`.
- `projection_mode=approximate_static_camera_offset`.
- `tf_validated=false`.
- `slam_used=false`.
- `navigation_used=false`.

Claim boundary:

- Only offline risk point projection is claimed.
- SLAM is not claimed.
- Autonomous navigation is not claimed.
- Path planning is not claimed.
- Absolute high-precision risk point coordinates are not claimed.
- Mechanical-arm autonomous handling based on map output is not claimed.

## Arm-C0 Evidence

Stage: map-to-arm action candidate dry-run.

Core directory:

- `outputs/arm_c0_map_to_arm_dryrun_v1/offline_p4x/`

Core files:

- `outputs/arm_c0_map_to_arm_dryrun_v1/offline_p4x/map_gated_arm_candidates.json`
- `outputs/arm_c0_map_to_arm_dryrun_v1/offline_p4x/map_gated_arm_candidates.csv`
- `outputs/arm_c0_map_to_arm_dryrun_v1/offline_p4x/arm_c0_dryrun_report.md`
- `outputs/arm_c0_map_to_arm_dryrun_v1/offline_p4x/episode_report.json`
- `outputs/arm_c0_map_to_arm_dryrun_v1/offline_p4x/errors.json`
- `docs/p5d_arm_c0_dryrun_integration_summary_20260630.md`
- `docs/arm_c1_hardware_gate_script_design_20260630.md`

Validation conclusion:

- `candidates=10`.
- `succeeded_dry_run=10`.
- `blocked=0`.
- `hardware_executed=false`.
- `serial_port_opened=false`.
- `serial_bytes_written=0`.
- `published_cmd_vel=false`.
- `contact_allowed=false`.
- `obstacle_removed=false`.
- `base_zero_checked=false`.

Claim boundary:

- Only map risk point to arm no-load action candidate dry-run mapping is claimed.
- Real mechanical-arm action is not claimed.
- Obstacle removal is not claimed.
- Grasping, contact, or payload handling is not claimed.
- SLAM, autonomous navigation, and path planning are not claimed.
- LLM control of the robot is not claimed.
- `base_zero_checked=false` is the dry-run stage state; Arm-C1 hardware execution must re-check live `base_zero_ok_before_arm=true`.

## Arm-C1 Gate Script Evidence

Stage: map-gated no-load hardware gate script dry-run.

Core script:

- `tools/run_arm_c1_map_gated_no_load_once.py`
- `tools/generate_arm_c1_base_zero_evidence.py`

Base-zero evidence directory:

- `outputs/arm_c1_base_zero_evidence_v1/offline_from_p4x/`
- `outputs/arm_c1_base_zero_evidence_v1/live_precheck_001/`
- `outputs/arm_c1_base_zero_evidence_v1/live_precheck_002/`
- `outputs/arm_c1_base_zero_evidence_v1/live_precheck_003/`

Base-zero evidence files:

- `outputs/arm_c1_base_zero_evidence_v1/offline_from_p4x/base_zero_evidence.json`
- `outputs/arm_c1_base_zero_evidence_v1/offline_from_p4x/errors.json`
- `outputs/arm_c1_base_zero_evidence_v1/offline_from_p4x/README.md`
- `outputs/arm_c1_base_zero_evidence_v1/live_precheck_001/base_zero_evidence.json`
- `outputs/arm_c1_base_zero_evidence_v1/live_precheck_001/errors.json`
- `outputs/arm_c1_base_zero_evidence_v1/live_precheck_001/README.md`
- `outputs/arm_c1_base_zero_evidence_v1/live_precheck_002/base_zero_evidence.json`
- `outputs/arm_c1_base_zero_evidence_v1/live_precheck_002/errors.json`
- `outputs/arm_c1_base_zero_evidence_v1/live_precheck_002/README.md`
- `outputs/arm_c1_base_zero_evidence_v1/live_precheck_003/base_zero_evidence.json`
- `outputs/arm_c1_base_zero_evidence_v1/live_precheck_003/errors.json`
- `outputs/arm_c1_base_zero_evidence_v1/live_precheck_003/README.md`
- `docs/arm_c1_live_base_zero_precheck_20260630.md`
- `docs/arm_c1_map_gated_no_load_validation_20260630.md`

Base-zero evidence validation conclusion:

- `evidence_type=offline_episode_report_snapshot`.
- `base_zero_ok_before_arm=true`.
- `published_cmd_vel=false`.
- `valid_for_arm_c1_hardware=false`.
- `cmd_vel_published_by_this_script=false`.
- `serial_port_opened=false`.
- `serial_bytes_written=0`.
- Offline evidence is dry-run/documentation input only.
- K1 live precheck `live_precheck_001` correctly failed safe with
  `valid_for_arm_c1_hardware=false`, `base_zero_ok_before_arm=false`,
  `confirm_count=0`, and all required freshness checks false.
- K1 live precheck `live_precheck_002` produced
  `evidence_type=live_base_zero_observation`.
- K1 live precheck `live_precheck_002` produced
  `valid_for_arm_c1_hardware=true`.
- K1 live precheck `live_precheck_002` produced
  `base_zero_ok_before_arm=true` and `published_cmd_vel=false`.
- K1 live precheck `live_precheck_003` produced fresh
  `valid_for_arm_c1_hardware=true` evidence for Arm-C1-H.
- K1 live precheck `live_precheck_003` produced
  `base_zero_ok_before_arm=true` and `published_cmd_vel=false`.

Core directory:

- `outputs/arm_c1_map_gated_no_load_once_v1/dryrun_candidate_001/`
- `outputs/arm_c1_map_gated_no_load_once_v1/dryrun_candidate_001_with_offline_base_zero/`
- `outputs/arm_c1_map_gated_no_load_once_v1/dryrun_candidate_001_with_k1_live_base_zero_precheck_002/`
- `outputs/arm_c1_map_gated_no_load_once_v1/dryrun_candidate_001_with_live_precheck_003/`
- `outputs/arm_c1_map_gated_no_load_once_v1/hw_candidate_001_with_live_precheck_003/`

Core files:

- `outputs/arm_c1_map_gated_no_load_once_v1/dryrun_candidate_001/episode_report.json`
- `outputs/arm_c1_map_gated_no_load_once_v1/dryrun_candidate_001/action_result.json`
- `outputs/arm_c1_map_gated_no_load_once_v1/dryrun_candidate_001/arm_c1_status.json`
- `outputs/arm_c1_map_gated_no_load_once_v1/dryrun_candidate_001/selected_candidate.json`
- `outputs/arm_c1_map_gated_no_load_once_v1/dryrun_candidate_001/sent_frame_hex.txt`
- `outputs/arm_c1_map_gated_no_load_once_v1/dryrun_candidate_001/physical_actuation_confirmation.json`
- `outputs/arm_c1_map_gated_no_load_once_v1/dryrun_candidate_001/errors.json`
- `outputs/arm_c1_map_gated_no_load_once_v1/dryrun_candidate_001_with_offline_base_zero/episode_report.json`
- `outputs/arm_c1_map_gated_no_load_once_v1/dryrun_candidate_001_with_offline_base_zero/arm_c1_status.json`
- `outputs/arm_c1_map_gated_no_load_once_v1/dryrun_candidate_001_with_offline_base_zero/errors.json`
- `outputs/arm_c1_map_gated_no_load_once_v1/dryrun_candidate_001_with_k1_live_base_zero_precheck_002/episode_report.json`
- `outputs/arm_c1_map_gated_no_load_once_v1/dryrun_candidate_001_with_k1_live_base_zero_precheck_002/arm_c1_status.json`
- `outputs/arm_c1_map_gated_no_load_once_v1/dryrun_candidate_001_with_k1_live_base_zero_precheck_002/errors.json`
- `outputs/arm_c1_map_gated_no_load_once_v1/dryrun_candidate_001_with_live_precheck_003/episode_report.json`
- `outputs/arm_c1_map_gated_no_load_once_v1/dryrun_candidate_001_with_live_precheck_003/arm_c1_status.json`
- `outputs/arm_c1_map_gated_no_load_once_v1/dryrun_candidate_001_with_live_precheck_003/errors.json`
- `outputs/arm_c1_map_gated_no_load_once_v1/hw_candidate_001_with_live_precheck_003/episode_report.json`
- `outputs/arm_c1_map_gated_no_load_once_v1/hw_candidate_001_with_live_precheck_003/action_result.json`
- `outputs/arm_c1_map_gated_no_load_once_v1/hw_candidate_001_with_live_precheck_003/arm_c1_status.json`
- `outputs/arm_c1_map_gated_no_load_once_v1/hw_candidate_001_with_live_precheck_003/physical_actuation_confirmation.json`
- `outputs/arm_c1_map_gated_no_load_once_v1/hw_candidate_001_with_live_precheck_003/errors.json`

Dry-run validation conclusion:

- `dryrun_candidate_001_with_live_precheck_003` reports `status=succeeded_dry_run`.
- `candidate_id=arm_c0_candidate_001`.
- `candidate_gate_passed=true`.
- `base_zero_ok_before_arm=true`.
- `hardware_executed=false`.
- `serial_port_opened=false`.
- `serial_bytes_written=0`.
- `published_cmd_vel=false`.
- `contact_allowed=false`.
- `obstacle_removed=false`.
- `errors.json=[]`.

Arm-C1-H validation conclusion:

- `hw_candidate_001_with_live_precheck_003` completed one supervised hardware
  no-load run.
- Arm-C1-H `status=succeeded`.
- Arm-C1-H `candidate_id=arm_c0_candidate_001`.
- Arm-C1-H `selected_action=ARM_SAMPLE_NO_LOAD`.
- Arm-C1-H `selected_sequence=arm_b3_8_step_safety_adjusted_no_load_sample`.
- Arm-C1-H `step_count=8` and `step_success_count=8`.
- Arm-C1-H `base_zero_ok_before_arm=true`.
- Arm-C1-H `hardware_executed=true`.
- Arm-C1-H `serial_port_opened=true`.
- Arm-C1-H `serial_bytes_written=180`.
- Arm-C1-H `published_cmd_vel=false`.
- Arm-C1-H `contact_allowed=false`.
- Arm-C1-H `obstacle_removed=false`.
- Arm-C1-H `errors.json=[]`.
- Operator confirmed physical actuation, final return to `6b`, and no observed
  physical issue.

Claim boundary:

- Only one Arm-C1-H map-gated no-load integrated validation is claimed.
- The run selected a validated no-load sequence after fresh live base-zero
  evidence passed.
- Contact is not claimed.
- Grasping is not claimed.
- Payload handling is not claimed.
- Real obstacle removal is not claimed.
- Autonomous navigation, path planning, SLAM/high-precision mapping, ROS arm
  executor validation, and LLM control are not claimed.
- Live base-zero evidence is freshness-bounded and must be regenerated before
  future hardware runs.

## Arm-B Evidence

Stage: real bus-servo mechanical-arm no-load validation.

Core directories:

- `outputs/arm_b1_send_home_once_v1/`
- `outputs/arm_b2_single_servo_no_load_v1/`
- `outputs/arm_b3_no_load_sample_sequence_v1/`

Arm-B1 core files:

- `outputs/arm_b1_send_home_once_v1/episode_report.json`
- `outputs/arm_b1_send_home_once_v1/action_result.json`
- `outputs/arm_b1_send_home_once_v1/arm_b1_status.json`
- `outputs/arm_b1_send_home_once_v1/physical_observation.json`
- `outputs/arm_b1_send_home_once_v1/physical_observation_after_switch_on.json`
- `outputs/arm_b1_send_home_once_v1/physical_actuation_confirmation.json`
- `outputs/arm_b1_send_home_once_v1/errors.json`

Arm-B2 core files:

- `outputs/arm_b2_single_servo_no_load_v1/hw_id5_360/episode_report.json`
- `outputs/arm_b2_single_servo_no_load_v1/hw_id5_360/physical_actuation_confirmation.json`
- `outputs/arm_b2_single_servo_no_load_v1/hw_id4_470/episode_report.json`
- `outputs/arm_b2_single_servo_no_load_v1/hw_id4_470/physical_actuation_confirmation.json`
- `outputs/arm_b2_single_servo_no_load_v1/hw_id3_526/episode_report.json`
- `outputs/arm_b2_single_servo_no_load_v1/hw_id3_526/physical_actuation_confirmation.json`
- `outputs/arm_b2_single_servo_no_load_v1/hw_id2_671/episode_report.json`
- `outputs/arm_b2_single_servo_no_load_v1/hw_id2_671/physical_actuation_confirmation.json`
- `outputs/arm_b2_single_servo_no_load_v1/hw_id1_610_id2_671_support_repeat1/episode_report.json`
- `outputs/arm_b2_single_servo_no_load_v1/hw_id1_610_id2_671_support_repeat1/physical_actuation_confirmation.json`

Arm-B3 accepted core files:

- `outputs/arm_b3_no_load_sample_sequence_v1/hw_sequence_002/episode_report.json`
- `outputs/arm_b3_no_load_sample_sequence_v1/hw_sequence_002/action_results.json`
- `outputs/arm_b3_no_load_sample_sequence_v1/hw_sequence_002/arm_b3_status.json`
- `outputs/arm_b3_no_load_sample_sequence_v1/hw_sequence_002/physical_actuation_confirmation.json`
- `outputs/arm_b3_no_load_sample_sequence_v1/hw_sequence_002/errors.json`

Validation conclusion:

- Arm-B0: K1 serial audit + dry-run safety gate validation PASS.
- Arm-B1: `6b` safe idle/home single-frame return PASS.
- Arm-B2: ID5, ID4, ID3, ID2, and ID1-with-ID2-support no-load checks PASS.
- Arm-B3: `hw_sequence_002` full 8-step safety-adjusted no-load sample sequence PASS.
- Arm-B3: 8/8 steps succeeded.
- Arm-B3: `controller_response_observed=true`.
- Arm-B3: `battery_mv=11304`.
- Arm-B3: `published_cmd_vel=false`.
- Arm-B3: `contact_allowed=false`.
- Arm-B3: `obstacle_removed=false`.
- Arm-B3: `errors=[]`.
- Operator confirmation: no issue observed and final return to `6b`.

Claim boundary:

- Only K1-to-bus-servo-controller serial control and no-load mechanical-arm execution are claimed.
- Grasping is not claimed.
- Contact is not claimed.
- Payload handling is not claimed.
- Real obstacle removal is not claimed.
- Autonomous execution is not claimed.
- ROS arm executor validation is not claimed.

## Step7-A Integrated Offline Flow Evidence

Stage: offline planning/mapping, D435 trigger, arm trigger, and LLM-A report flow.

Core script:

- `tools/run_step7_integrated_offline_flow.py`

Core directory:

- `outputs/step7_integrated_offline_flow_v1/offline_p4x_arm_c0/`

Core files:

- `logs/policy_p4w_run_branch_mixed_20260629_183731.json`
- `maps/policy_p4w_branch_mixed_20260629_183731_final_marked.png`
- `docs/p4_guarded_policy_executable_modes_20260629.md`
- `edge-ai-robot-k1-p4-y2-7step-guarded-stress-58399be.bundle`
- `outputs/step7_integrated_offline_flow_v1/offline_p4x_arm_c0/episode_report.json`
- `outputs/step7_integrated_offline_flow_v1/offline_p4x_arm_c0/step7_flow_summary.json`
- `outputs/step7_integrated_offline_flow_v1/offline_p4x_arm_c0/step7_trigger_trace.csv`
- `outputs/step7_integrated_offline_flow_v1/offline_p4x_arm_c0/step7_integrated_report.md`
- `outputs/step7_integrated_offline_flow_v1/offline_p4x_arm_c0/errors.json`
- `outputs/step7_integrated_offline_flow_v1/offline_p4x_arm_c0/README.md`
- `outputs/llm_a_risk_report_v1/step7_integrated_offline_flow/risk_report.md`
- `outputs/llm_a_risk_report_v1/step7_integrated_offline_flow/risk_report.json`
- `outputs/llm_a_risk_report_v1/step7_integrated_offline_flow/claim_boundary.md`
- `docs/step7_integrated_offline_flow_20260630.md`

Validation conclusion:

- `status=succeeded_dry_run`.
- P4-Y2 is consumed as upstream guarded policy / final map evidence.
- P4-Y2 requested 7 max policy steps but stopped at step 3 by
  `max_consecutive_fast_arc_reached`.
- P4-Y2 executed two `ARC_FAST_RIGHT` motions with yaw deltas about `-21.26 deg`
  and `-27.07 deg`.
- P4-Y2 `base_zero_ok=true`.
- P4-Y2 `final_map_saved=true`.
- P4-Y2 cumulative positive forward motion was about `0.1242 m`, below `1.0 m`.
- `risk_map_points=10`.
- `d435_succeeded_dry_run=10`.
- `d435_blocked=0`.
- `arm_succeeded_dry_run=10`.
- `arm_blocked=0`.
- `errors.json=[]`.
- `ros_started=false`.
- `published_cmd_vel=false`.
- `hardware_executed=false`.
- `serial_port_opened=false`.
- `serial_bytes_written=0`.
- `contact_allowed=false`.
- `obstacle_removed=false`.
- LLM-A report status is `PASS_DRY_RUN`.

Claim boundary:

- Only Step7-A offline integrated rule-flow validation is claimed.
- Existing P4-Y2 guarded policy stress-stop evidence is consumed as planning/mapping safety evidence.
- Existing P4-X stationary D435 evidence is consumed as the visual trigger source.
- Existing Map-A0 approximate risk map points are consumed as mapping evidence.
- Arm-C0 no-load candidates are consumed as simulated mechanical-arm trigger outputs.
- New live D435 capture is not claimed.
- ROS startup is not claimed.
- Real mechanical-arm motion from this Step7 offline run is not claimed.
- Grasping, contact, payload handling, and obstacle removal are not claimed.
- Full autonomous navigation, path planning success, and high-precision SLAM are not claimed.
- LLM control of the robot is not claimed.

## Step7-B0 Live Stationary Flow Evidence

Stage: live stationary base-zero, D435 capture, Map-A0 projection, Arm-C0 dry-run,
and deterministic LLM-A report.

Core script:

- `tools/run_step7b_live_stationary_flow.py`

Core directory:

- `outputs/step7b_live_stationary_flow_v1/step7b_live_stationary_20260630_130029/`

Core files:

- `outputs/step7b_live_stationary_flow_v1/step7b_live_stationary_20260630_130029/base_zero_live/base_zero_evidence.json`
- `outputs/step7b_live_stationary_flow_v1/step7b_live_stationary_20260630_130029/p4x_live_hold_capture/episode_report.json`
- `outputs/step7b_live_stationary_flow_v1/step7b_live_stationary_20260630_130029/map_a0_live_projection/risk_map_points.json`
- `outputs/step7b_live_stationary_flow_v1/step7b_live_stationary_20260630_130029/arm_c0_live_dryrun/episode_report.json`
- `outputs/step7b_live_stationary_flow_v1/step7b_live_stationary_20260630_130029/episode_report.json`
- `outputs/step7b_live_stationary_flow_v1/step7b_live_stationary_20260630_130029/step7b_live_report.md`
- `outputs/step7b_live_stationary_flow_v1/step7b_live_stationary_20260630_130029/llm_a_report/risk_report.md`
- `outputs/step7b_live_stationary_flow_v1/step7b_live_stationary_20260630_130029/errors.json`
- `docs/step7b_live_stationary_flow_20260630.md`

Validation conclusion:

- `status=succeeded`.
- `base_zero_ok_before_capture=true`.
- `d435_live_capture_executed=true`.
- `risk_map_points=1`.
- `projected=1`.
- `arm_c0_candidates=1`.
- `arm_c0_succeeded_dry_run=1`.
- `arm_c0_blocked=0`.
- `arm_c1_hardware_executed=false`.
- `hardware_executed=false`.
- `serial_port_opened=false`.
- `serial_bytes_written=0`.
- `published_cmd_vel=false`.
- `contact_allowed=false`.
- `obstacle_removed=false`.
- top-level `errors.json=[]`.
- LLM-A report status is `PASS`.
- Schema validation returned `ok=true`; the only warning is the expected
  `succeeded_dry_run` Arm-C0 action status.
- `map_a0_live_projection/errors.json` contains a plotting warning because K1
  does not have `matplotlib`; JSON/CSV/report projection evidence was still
  generated.

Claim boundary:

- Live stationary base-zero gate and D435 HOLD_CAPTURE are claimed.
- Map-A0 projection from the live risk point is claimed.
- Arm-C0 map-gated no-load candidate generation is claimed as dry-run only.
- Chassis motion during Step7-B0 is not claimed.
- Autonomous navigation or path planning success is not claimed.
- SLAM/high-precision mapping is not claimed.
- Grasping, contact, payload handling, and obstacle removal are not claimed.
- LLM control of the robot is not claimed.
- Arm-C1-H hardware execution is not claimed for this run.

## Step7-C0 Guarded D435 Mock-Risk Arm No-Load Dry-Run Evidence

Stage: guarded live base-zero, D435 live HOLD_CAPTURE, deterministic mock risk
trigger, Map-A0 live projection, Arm-C0 candidate generation, Arm-C1 no-load
dry-run gate, and deterministic LLM-A report.

Core script:

- `tools/run_step7c_guarded_d435_mockrisk_arm_noload.py`

Core directory:

- `outputs/step7c_guarded_d435_mockrisk_arm_noload_v1/dryrun_001/`

Core files:

- `outputs/step7c_guarded_d435_mockrisk_arm_noload_v1/dryrun_001/base_zero_live/base_zero_evidence.json`
- `outputs/step7c_guarded_d435_mockrisk_arm_noload_v1/dryrun_001/d435_hold_capture/episode_report.json`
- `outputs/step7c_guarded_d435_mockrisk_arm_noload_v1/dryrun_001/mock_risk/mock_risk_summary.json`
- `outputs/step7c_guarded_d435_mockrisk_arm_noload_v1/dryrun_001/map_projection/risk_map_points.json`
- `outputs/step7c_guarded_d435_mockrisk_arm_noload_v1/dryrun_001/arm_candidate/episode_report.json`
- `outputs/step7c_guarded_d435_mockrisk_arm_noload_v1/dryrun_001/arm_execution/episode_report.json`
- `outputs/step7c_guarded_d435_mockrisk_arm_noload_v1/dryrun_001/episode_report.json`
- `outputs/step7c_guarded_d435_mockrisk_arm_noload_v1/dryrun_001/step7c_report.md`
- `outputs/step7c_guarded_d435_mockrisk_arm_noload_v1/dryrun_001/llm_a_report/risk_report.md`
- `outputs/step7c_guarded_d435_mockrisk_arm_noload_v1/dryrun_001/errors.json`
- `docs/step7c_guarded_d435_mockrisk_arm_noload_20260630.md`

Validation conclusion:

- `status=succeeded`.
- `arm_mode=dry_run`.
- `base_zero_ok_before_capture=true`.
- `base_zero_ok_before_arm=true`.
- `d435_live_capture_executed=true`.
- `risk_point_generated=true`.
- `mock_risk_triggered=true`.
- `risk_map_points=1`.
- `projected=1`.
- `arm_candidate_selected=true`.
- `arm_c0_candidates=1`.
- `arm_c0_succeeded_dry_run=1`.
- `arm_c0_blocked=0`.
- `selected_sequence=arm_b3_8_step_safety_adjusted_no_load_sample`.
- `arm_execution_status=succeeded_dry_run`.
- `hardware_executed=false`.
- `serial_port_opened=false`.
- `serial_bytes_written=0`.
- `published_cmd_vel=false`.
- `contact_allowed=false`.
- `obstacle_removed=false`.
- top-level `errors.json=[]`.
- LLM-A report status is `PASS`.
- Schema validation returned `ok=true`; the warnings are the expected
  `succeeded_dry_run` statuses for Arm-C0 and Arm-C1 dry-run steps.

Claim boundary:

- Guarded stationary live integration is claimed.
- D435 evidence capture and deterministic mock anomaly trigger are claimed.
- Approximate Map-A0 risk point projection is claimed.
- Map-gated no-load arm response is claimed only as dry-run for this evidence.
- Real mechanical-arm hardware execution is not claimed for Step7-C0.
- Real visual detection accuracy is not claimed.
- Grasping, contact, payload handling, and obstacle removal are not claimed.
- Autonomous navigation, path planning, SLAM/high-precision mapping, and LLM
  control of the robot are not claimed.

## Step7-C1 Guarded D435 Mock-Risk Arm No-Load Hardware Evidence

Stage: guarded live base-zero, D435 live HOLD_CAPTURE, deterministic mock risk
trigger, Map-A0 live projection, Arm-C0 candidate generation, one Arm-C1
hardware no-load execution, and deterministic LLM-A report.

Core script:

- `tools/run_step7c_guarded_d435_mockrisk_arm_noload.py`

Core directory:

- `outputs/step7c_guarded_d435_mockrisk_arm_noload_v1/hw_001/`

Core files:

- `outputs/step7c_guarded_d435_mockrisk_arm_noload_v1/hw_001/base_zero_live/base_zero_evidence.json`
- `outputs/step7c_guarded_d435_mockrisk_arm_noload_v1/hw_001/d435_hold_capture/episode_report.json`
- `outputs/step7c_guarded_d435_mockrisk_arm_noload_v1/hw_001/mock_risk/mock_risk_summary.json`
- `outputs/step7c_guarded_d435_mockrisk_arm_noload_v1/hw_001/map_projection/risk_map_points.json`
- `outputs/step7c_guarded_d435_mockrisk_arm_noload_v1/hw_001/arm_candidate/episode_report.json`
- `outputs/step7c_guarded_d435_mockrisk_arm_noload_v1/hw_001/arm_execution/episode_report.json`
- `outputs/step7c_guarded_d435_mockrisk_arm_noload_v1/hw_001/arm_execution/physical_actuation_confirmation.json`
- `outputs/step7c_guarded_d435_mockrisk_arm_noload_v1/hw_001/episode_report.json`
- `outputs/step7c_guarded_d435_mockrisk_arm_noload_v1/hw_001/step7c_report.md`
- `outputs/step7c_guarded_d435_mockrisk_arm_noload_v1/hw_001/llm_a_report/risk_report.md`
- `outputs/step7c_guarded_d435_mockrisk_arm_noload_v1/hw_001/errors.json`
- `docs/step7c_guarded_d435_mockrisk_arm_noload_20260630.md`

Validation conclusion:

- `status=succeeded`.
- `arm_mode=hardware_once`.
- `base_zero_ok_before_capture=true`.
- `base_zero_ok_before_arm=true`.
- `d435_live_capture_executed=true`.
- `risk_point_generated=true`.
- `mock_risk_triggered=true`.
- `risk_map_points=1`.
- `projected=1`.
- `arm_candidate_selected=true`.
- `arm_c0_candidates=1`.
- `arm_c0_succeeded_dry_run=1`.
- `arm_c0_blocked=0`.
- `selected_sequence=arm_b3_8_step_safety_adjusted_no_load_sample`.
- `arm_execution_status=succeeded`.
- `hardware_executed=true`.
- `serial_port_opened=true`.
- `serial_bytes_written=180`.
- `published_cmd_vel=false`.
- `contact_allowed=false`.
- `obstacle_removed=false`.
- top-level `errors.json=[]`.
- LLM-A report status is `PASS`.
- Schema validation returned `ok=true`; the warning is the expected
  `succeeded_dry_run` status for the Arm-C0 candidate step.
- Physical observation confirmation has been completed from operator observation:
  `physical_actuation_observed=true`, `returned_to_6b_observed=true`,
  `physical_issue_observed=false`, `contact_observed=false`.

Claim boundary:

- Guarded stationary live integration is claimed.
- D435 evidence capture and deterministic mock anomaly trigger are claimed.
- Approximate Map-A0 risk point projection is claimed.
- One map-gated no-load arm hardware response is claimed only at the script
  evidence level: `hardware_executed=true`, `serial_bytes_written=180`,
  `published_cmd_vel=false`, `contact_allowed=false`, `obstacle_removed=false`.
- Field observation confirms final 6b and no visible abnormal issue for this
  single no-load run.
- Real visual detection accuracy is not claimed.
- Grasping, contact, payload handling, and obstacle removal are not claimed.
- Autonomous navigation, path planning, SLAM/high-precision mapping, and LLM
  control of the robot are not claimed.

## Step7-E2 Guarded Motion Red-Rule Evidence

Stage: guarded micro-motion, D435 deterministic HSV red-rule trigger, Map-A0
projection, Arm-C1 no-load hardware response, and deterministic LLM-A report.

Core scripts:

- `tools/d435_red_rule_detector.py`
- `tools/run_step7e1_red_rule_stationary_flow.py`
- `tools/run_step7e2_guarded_motion_red_rule_flow.py`
- `tools/generate_llm_a_risk_report.py`

Negative-control directory:

- `outputs/step7e2_guarded_motion_red_rule_flow_v1/e2_guarded_red_rule_negative_002/`

Negative-control conclusion:

- No red target was present.
- `status=succeeded`.
- `guarded_motion_executed=true`.
- `base_zero_ok_before_capture=true`.
- `d435_live_capture_executed=true`.
- `negative_control_expected=true`.
- `negative_control_pass=true`.
- `red_object_detected=false`.
- `red_mask_pixels=0`.
- `risk_point_generated=false`.
- `risk_map_points=0`.
- `projected=0`.
- `arm_execution_status=skipped_negative_control`.
- `hardware_executed=false`.
- `serial_bytes_written=0`.
- `errors=[]`.
- LLM-A report status is `PASS`.

Fastdemo positive hardware directory:

- `outputs/step7e2_guarded_motion_red_rule_flow_v1/e2_guarded_red_rule_arm_hw_fastdemo_002/`

Fastdemo positive hardware conclusion:

- `status=succeeded`.
- `guarded_motion_executed=true`.
- Required motion path was
  `/input_cmd_vel -> scan_safety_guard_node -> /cmd_vel_guarded`.
- `direct_cmd_vel_bypass=false`.
- `policy_executed_count=2`.
- `policy_sequence_stop_reason=max_consecutive_fast_arc_reached`.
- `cumulative_positive_forward_m=0.118`.
- `base_zero_ok_after_motion=true`.
- `demo_fast_reuse_policy_base_zero=true`.
- `red_object_detected=true`.
- `bbox_xywh=[93,250,275,117]`.
- `depth_median_m=0.561`.
- `risk_map_points=1`.
- `projected=1`.
- `arm_execution_status=succeeded`.
- `hardware_executed=true`.
- `serial_bytes_written=180`.
- `published_cmd_vel_during_capture=false`.
- `published_cmd_vel_during_arm=false`.
- `contact_allowed=false`.
- `obstacle_removed=false`.
- `errors=[]`.
- Operator confirmation completed:
  `final_pose_observed=6b`, `physical_issue_observed=false`.
- LLM-A report status is `PASS`.

Core files:

- `outputs/step7e2_guarded_motion_red_rule_flow_v1/e2_guarded_red_rule_negative_002/episode_report.json`
- `outputs/step7e2_guarded_motion_red_rule_flow_v1/e2_guarded_red_rule_negative_002/step7e2_negative_acceptance_check.json`
- `outputs/step7e2_guarded_motion_red_rule_flow_v1/e2_guarded_red_rule_arm_hw_fastdemo_002/episode_report.json`
- `outputs/step7e2_guarded_motion_red_rule_flow_v1/e2_guarded_red_rule_arm_hw_fastdemo_002/step7e2_fastdemo_positive_arm_hw_acceptance_check.json`
- `outputs/step7e2_guarded_motion_red_rule_flow_v1/e2_guarded_red_rule_arm_hw_fastdemo_002/red_rule_after_motion/d435_red_rule_capture/captures/<capture_id>/red_object_overlay.png`
- `outputs/step7e2_guarded_motion_red_rule_flow_v1/e2_guarded_red_rule_arm_hw_fastdemo_002/llm_a_report/risk_report.md`
- `outputs/step7e2_guarded_motion_red_rule_flow_v1/e2_guarded_red_rule_arm_hw_fastdemo_002/llm_a_report/risk_report.json`
- `docs/step7e2_guarded_motion_red_rule_flow_20260630.md`
- `docs/step7e2_fastdemo_reproduction_20260630.md`

Claim boundary:

- Only guarded motion through the existing P4/N10P safety chain is claimed.
- Only deterministic D435 HSV red-rule trigger is claimed; trained-model
  visual recognition accuracy is not claimed.
- Only approximate Map-A0 projection is claimed; high-precision SLAM is not
  claimed.
- Only one Arm-C1 no-load hardware response is claimed for `fastdemo_002`.
- Grasping, contact, payload handling, and obstacle clearing are not claimed.
- Autonomous navigation and path planning are not claimed.
- LLM control of the robot is not claimed.

Git note:

- `outputs/` remains ignored by git.
- Git commits should record evidence paths, summaries, and claim boundaries
  only; the actual evidence directory should be packaged separately if needed.

## Git / Packaging Note

- `outputs/` is ignored by `.gitignore`.
- Evidence directories remain local by default.
- Do not add the full `outputs/` tree to git.
- For competition submission or archival, package the evidence directories separately.
- Docs record only the evidence manifest and validation summary.

## Next Recommended Steps

- P4-X3: validate header/frame/depth-ratio fields on real ROS/D435 data streams.
- Arm-B: after mechanical repair is complete, prepare no-load dry-run safety tests.
- Arm-B is frozen at no-load validation.
- Arm-C/D/E: do not proceed until Arm-B evidence is reviewed and a new safety plan is written.
- Map-A: offline projection from D435 `risk_point` plus odom into risk-map evidence.
