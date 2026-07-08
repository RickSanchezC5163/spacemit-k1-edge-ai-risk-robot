# LLM-A Episode Report Input Contract

Date: 2026-06-29

Contract version: `p4x_d435_hold_capture_v1`

Primary input:

```text
outputs/p4x_d435_hold_capture_v1/episode_report.json
```

## Consumer Role

LLM-A is a report-generation consumer. It may summarize evidence and produce a human-readable inspection report.

LLM-A must not issue control commands, infer robot motion commands, operate an arm, or enter the control loop.

## Top-Level Schema

The existing `episode_report.json` contains:

- `episode_id`
- `started_at`
- `ended_at`
- `protocol_version`
- `policy_state`
- `actions`
- `action_results`
- `captures`
- `risk_points`
- `summary`
- `errors`
- `output_root`

## Join Keys

Use these joins:

- `actions[].action_id` joins `action_results[].action_id`.
- `captures[].action_id` joins `actions[].action_id`.
- `captures[].capture_id` joins `action_results[].capture_id`.
- `risk_points[].capture_id` joins `captures[].capture_id`.

## Required Summary Checks

Before generating a positive report, LLM-A should check:

- `summary.succeeded >= summary.min_successes`
- `summary.acceptance_10_runs_9_success == true`
- `summary.published_cmd_vel == false`
- `errors` is empty or explicitly summarized

For each successful capture, LLM-A should check:

- `action_results[].action_type == "HOLD_CAPTURE"`
- `action_results[].status == "succeeded"`
- `action_results[].base_zero_ok_before == true`
- `action_results[].published_cmd_vel == false`
- `action_results[].capture_meta_path` is present
- `action_results[].risk_point_path` is present

If any `ActionResult.status == "failed_safe"`, LLM-A must report it as a safe failure, not hide it.

## Capture Fields

Each `captures[]` record may include:

- `capture_id`
- `action_id`
- `timestamp`
- `sequence`
- `topics.rgb`
- `topics.depth`
- `topics.camera_info`
- `topics.odom`
- `paths.rgb`
- `paths.depth_raw`
- `paths.depth_vis`
- `paths.camera_info`
- `paths.odom`
- `paths.capture_meta`
- `rgb.encoding`, `rgb.height`, `rgb.width`
- `depth.encoding`, `depth.height`, `depth.width`
- `depth.dtype`
- `depth.depth_scale_m`
- `depth.valid_count`
- `depth.vis_min_m`
- `depth.vis_max_m`
- `camera_info.k`
- `odom`

Path note: some paths inside the report are K1 absolute paths under `/home/soc/edge-ai-robot-k1`. The local mirrored evidence root is `K:\risc-vCar\edge-ai-robot-k1\outputs\p4x_d435_hold_capture_v1`.

## Risk Point Fields

Each `risk_points[]` record may include:

- `risk_point_id`
- `capture_id`
- `label`
- `bbox_xywh`
- `depth_median_m`
- `camera_point_xyz_m`
- `confidence`
- `evidence_paths`
- `generated_by`
- `timestamp`
- `notes`

Units:

- `depth_raw.npy` is raw `uint16` depth.
- `depth.depth_scale_m=0.001` converts raw depth units to meters.
- `risk_points[].depth_median_m` is meters.
- `risk_points[].camera_point_xyz_m` is meters.

## Required Claim Boundary

LLM-A may claim:

- D435 RGB/depth/camera_info topics were readable.
- Stationary capture saved RGB/depth/camera_info/odom/meta files.
- HOLD_CAPTURE ran only after `base_zero_ok_before=True`.
- HOLD_CAPTURE did not publish `cmd_vel`.
- Mock risk point evidence was generated and linked.

LLM-A must not claim:

- Real visual detection accuracy.
- Object recognition.
- Hazard classification accuracy.
- Arm manipulation.
- Autonomous semantic reasoning.
- LLM control-loop authority.
- Any P4-V base safety chain modification.

## Known Missing Fields

The frozen P4-X0/X1/X2 evidence does not include:

- Per-capture RGB ROS header timestamp.
- Per-capture depth ROS header timestamp.
- `valid_depth_ratio`.
- `bbox_valid_depth_ratio`.

LLM-A should mention these as limitations or P4-X3 additions when relevant. It must not fabricate these values.
