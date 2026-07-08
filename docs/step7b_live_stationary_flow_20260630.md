# Step7-B0 Live Stationary Flow

Date: 2026-06-30

## Scope

Step7-B0 upgrades Step7 from offline evidence composition to a live stationary
integration run:

```text
guarded stack already running
-> live base_zero evidence
-> live D435 HOLD_CAPTURE
-> mock_risk_detector risk_point
-> Map-A0 live projection
-> Arm-C0 map-gated no-load candidate dry-run
-> deterministic LLM-A report
```

The runner does not publish `cmd_vel`. By default it does not open serial ports
and does not control the mechanical arm.

## Preconditions

- K1 is online.
- The guarded stack is running and exposes `/odom`, `/cmd_vel_guarded`,
  `/robot_vel`, and base diagnostics through `/rosout`.
- D435 topics are running:
  - `/camera/camera/color/image_raw`
  - `/camera/camera/depth/image_rect_raw`
  - `/camera/camera/color/camera_info`
- The robot is stationary before capture.

## Command

```bash
cd /home/soc/edge-ai-robot-k1
python3 tools/run_step7b_live_stationary_flow.py
```

Optional Arm-C1-H no-load once is disabled by default. It requires both explicit
flags:

```bash
python3 tools/run_step7b_live_stationary_flow.py \
  --enable-arm-c1-hardware-once \
  --confirm-arm-c1-hardware-once
```

Do not use the optional hardware flags until Step7-B0 live stationary evidence
has been reviewed.

## Output

Default output root:

```text
outputs/step7b_live_stationary_flow_v1/<run_id>/
```

Files:

- `base_zero_live/base_zero_evidence.json`
- `p4x_live_hold_capture/episode_report.json`
- `map_a0_live_projection/risk_map_points.json`
- `arm_c0_live_dryrun/episode_report.json`
- `episode_report.json`
- `step7b_live_report.md`
- `llm_a_report/risk_report.md`
- `errors.json`

## Claim Boundary

Allowed claims:

- live stationary base-zero gate was checked
- live D435 HOLD_CAPTURE was executed after base-zero
- live D435 evidence was projected by Map-A0
- Arm-C0 generated map-gated no-load candidates in dry-run mode
- deterministic LLM-A report was exported

Disallowed claims:

- chassis motion during Step7-B0
- autonomous navigation or path planning success
- SLAM/high-precision mapping
- grasping, contact, payload handling, or obstacle removal
- LLM control of the robot
- Arm-C1-H hardware execution unless the source report explicitly says
  `arm_c1_hardware_executed=true`
