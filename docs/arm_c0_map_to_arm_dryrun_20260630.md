# Arm-C0 Map-To-Arm Candidate Dry-Run

Date: 2026-06-30

## Scope

Arm-C0 consumes Map-A0 offline risk map points and generates dry-run mechanical
arm no-load action candidates.

Input:

```text
outputs/map_a_risk_point_projection_v1/offline_p4x/risk_map_points.json
```

Output:

```text
outputs/arm_c0_map_to_arm_dryrun_v1/offline_p4x/
```

This stage is candidate generation only. It does not start ROS, publish
`cmd_vel`, open a serial port, control the mechanical arm, or run Arm-B3
hardware actions.

## Tool

```text
tools/generate_arm_c0_map_to_arm_dryrun.py
```

Usage:

```powershell
python tools\generate_arm_c0_map_to_arm_dryrun.py --risk-map-points outputs\map_a_risk_point_projection_v1\offline_p4x\risk_map_points.json --output-dir outputs\arm_c0_map_to_arm_dryrun_v1\offline_p4x
```

Generated files:

- `map_gated_arm_candidates.json`
- `map_gated_arm_candidates.csv`
- `arm_c0_dryrun_report.md`
- `episode_report.json`
- `README.md`
- `errors.json`

## Candidate Rules

For each Map-A0 risk map point:

- `projection_status="projected"` can generate `status="succeeded_dry_run"`.
- Any non-projected point is marked `status="blocked"`.
- `tf_validated=false` is allowed for dry-run only and is marked
  `projection_precision="approximate"`.
- `selected_action="ARM_SAMPLE_NO_LOAD"`.
- `selected_sequence="arm_b3_8_step_safety_adjusted_no_load_sample"`.
- `validated_no_load_action=true` only for unblocked candidates.

Zone classification is intentionally coarse:

- `front`: `x > 0` and `|y| <= 0.25`
- `left`: `y > 0.25`
- `right`: `y < -0.25`
- `near`: distance `<= 0.5 m`
- `far`: distance `> 0.5 m`

The classifier uses `base_point_xyz_m` first. If that is unavailable, it falls
back to `odom_point_xy_m - robot_odom_pose`.

## Safety Boundary

Every candidate records:

- `hardware_executed=false`
- `serial_port_opened=false`
- `serial_bytes_written=0`
- `contact_allowed=false`
- `obstacle_removed=false`
- `base_zero_required=true`
- `base_zero_checked=false`
- `published_cmd_vel=false`

Because Arm-C0 is offline, it does not claim a live `base_zero_ok_before=true`
measurement. A later Arm-C1 hardware step must perform that check before any
real no-load action.

## Claim Boundary

This stage only claims map risk point to arm no-load action candidate dry-run
mapping.

It must not claim:

- real mechanical-arm action
- obstacle removal
- grasping
- contact
- payload handling
- SLAM
- autonomous navigation
- path planning
- LLM control of the robot

## Next Recommended Step

Arm-C1 should remain a separate explicit hardware validation. It may only be
attempted after a live base-zero gate is checked, and it must still use a
validated no-load sequence with no contact and no obstacle-removal claim.
