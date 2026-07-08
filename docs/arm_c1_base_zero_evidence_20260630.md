# Arm-C1 Base-Zero Evidence

Date: 2026-06-30

Status: read-only evidence generator implemented. Offline validation completed.
No ROS process was started for this validation step. No serial port was opened.
No mechanical-arm command was sent.

## Purpose

Arm-C1 real no-load execution must not rely on verbal confirmation that the
base is stopped. It requires an auditable `base_zero_evidence.json` input.

The generator supports two evidence modes:

- `offline_episode_report_snapshot`: extracts base-zero evidence from an
  existing `episode_report.json`. This is valid for dry-run and documentation
  only.
- `live_base_zero_observation`: future K1 read-only ROS observation mode. This
  is the only evidence type that may satisfy the Arm-C1 hardware gate.

## Script

```text
tools/generate_arm_c1_base_zero_evidence.py
```

Offline command used for validation:

```powershell
python tools\generate_arm_c1_base_zero_evidence.py --from-episode-report outputs\p4x_d435_hold_capture_v1\episode_report.json --output-dir outputs\arm_c1_base_zero_evidence_v1\offline_from_p4x
```

Future live K1 command shape:

```bash
python3 tools/generate_arm_c1_base_zero_evidence.py \
  --ros-live \
  --output-dir outputs/arm_c1_base_zero_evidence_v1/live_arm_c1_precheck
```

The live mode creates a read-only subscriber node. It does not create a
`cmd_vel` publisher, does not open a serial port, and does not control the
mechanical arm.

## Evidence Fields

`base_zero_evidence.json` records:

- `evidence_type`
- `source_mode`
- `valid_for_arm_c1_hardware`
- `base_zero_checked_live`
- `base_zero_ok_before_arm`
- `published_cmd_vel`
- `base_zero`
- `odom`
- `cmd_vel_published_by_this_script=false`
- `serial_port_opened=false`
- `serial_bytes_written=0`

Offline P4-X evidence keeps:

- `base_zero_ok_before_arm=true`
- `published_cmd_vel=false`
- `evidence_type=offline_episode_report_snapshot`
- `valid_for_arm_c1_hardware=false`

That offline evidence can be referenced by Arm-C1 dry-run reports, but it must
not unlock hardware execution.

## Arm-C1 Gate Requirement

`tools/run_arm_c1_map_gated_no_load_once.py` now requires the following before
real hardware execution:

- `--base-zero-evidence` is present and loadable
- `evidence_type=live_base_zero_observation`
- `valid_for_arm_c1_hardware=true`
- `base_zero_ok_before_arm=true`
- `published_cmd_vel=false`
- evidence age is within `--base-zero-max-age-s` (default 60 seconds)
- all explicit hardware confirmation flags are present

If any condition fails, Arm-C1 must write `failed_safe` and must not open the
serial port for writes.

## Claim Boundary

This step only adds auditable Arm-C1 base-zero precondition evidence.

It does not claim:

- Arm-C1 real hardware execution
- map-gated mechanical-arm motion
- obstacle removal
- grasping
- contact
- payload handling
- autonomous navigation
- SLAM
- LLM control of the robot

## Next Recommended Step

Before any Arm-C1 hardware attempt, generate live base-zero evidence on K1
immediately before the action, then run Arm-C1 with the explicit hardware flags
and the fresh live evidence path.
