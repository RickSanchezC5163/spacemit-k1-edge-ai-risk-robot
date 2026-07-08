# Arm-C1 Live Base-Zero Precheck

Date: 2026-06-30

Status: frozen as Arm-C1 pre-hardware safety evidence. No Arm-C1 real hardware
execution was performed in this stage.

## Summary

This stage validates the Arm-C1 precondition path:

```text
Map-A0 risk_map_point
-> Arm-C0 map-gated no-load candidate
-> K1 live base-zero evidence
-> Arm-C1 dry-run gate consumption
```

The result is a clean pre-hardware readiness milestone. It proves that a fresh
K1 live guarded-stack observation can generate auditable
`base_zero_ok_before_arm=true` evidence and that the Arm-C1 dry-run gate can
consume that evidence without opening the arm serial port.

## Evidence Directories

- `outputs/arm_c1_base_zero_evidence_v1/live_precheck_001/`
- `outputs/arm_c1_base_zero_evidence_v1/live_precheck_002/`
- `outputs/arm_c1_map_gated_no_load_once_v1/dryrun_candidate_001_with_k1_live_base_zero_precheck_002/`

## Live Precheck 001

Purpose: verify failed-safe behavior when the ROS guarded stack is not running.

Result:

- `evidence_type=live_base_zero_observation`
- `valid_for_arm_c1_hardware=false`
- `base_zero_ok_before_arm=false`
- `published_cmd_vel=false`
- `timed_out=true`
- freshness:
  - `odom_fresh=false`
  - `guarded_cmd_fresh=false`
  - `robot_vel_fresh=false`
  - `diag_fresh=false`
- `confirm_count=0`
- `required_confirmations=3`

Conclusion: PASS as failed-safe behavior. The gate did not infer base-zero when
required live ROS evidence was missing.

## Live Precheck 002

Purpose: verify live base-zero evidence generation with the previously validated
guarded stack running on K1.

Guarded stack source:

```text
turn_on_wheeltec_robot n10p_tank_mapping_safety_guard.launch.py
```

Result:

- `evidence_type=live_base_zero_observation`
- `valid_for_arm_c1_hardware=true`
- `base_zero_ok_before_arm=true`
- `published_cmd_vel=false`
- `timed_out=false`
- freshness:
  - `odom_fresh=true`
  - `guarded_cmd_fresh=true`
  - `robot_vel_fresh=true`
  - `diag_fresh=true`
- `confirm_count=3`
- `required_confirmations=3`
- `policy_zero_basis=odom+guarded_cmd+robot_vel+base_diag`

Conclusion: PASS. K1 produced valid live base-zero evidence under the guarded
stack.

## Arm-C1 Dry-Run Consumption

The live evidence from `live_precheck_002` was consumed by the Arm-C1 gate script
in dry-run mode:

```text
outputs/arm_c1_map_gated_no_load_once_v1/dryrun_candidate_001_with_k1_live_base_zero_precheck_002/
```

Result:

- `status=succeeded_dry_run`
- `candidate_id=arm_c0_candidate_001`
- `base_zero_ok_before_arm=true`
- `published_cmd_vel=false`
- `hardware_executed=false`
- `serial_port_opened=false`
- `serial_bytes_written=0`
- `contact_allowed=false`
- `obstacle_removed=false`
- `errors.json=[]`

This confirms the evidence contract and gate wiring, not real Arm-C1 hardware
execution.

## Freshness Rule

Arm-C1 hardware execution requires fresh live evidence. The current gate default
is:

```text
--base-zero-max-age-s 60
```

Therefore `live_precheck_002` is historical evidence and must not be reused for
a future hardware run. A new live base-zero evidence file must be generated
immediately before any Arm-C1-H attempt.

## Operational Notes

- No mechanical-arm command was sent in this stage.
- No arm serial port was opened in this stage.
- The guarded stack was stopped after live evidence collection.
- A later ROS battery voltage read reported `/battery_voltage=10.788 V`.

## Claim Boundary

Current allowed claim:

```text
Arm-C1 pre-hardware live base-zero safety evidence has been validated: K1
produced live guarded-stack evidence with base_zero_ok_before_arm=true, and that
evidence was successfully consumed by the map-gated Arm-C1 no-load dry-run gate.
```

Do not claim:

- Arm-C1 real hardware execution passed
- mechanical arm acted based on the map
- obstacle removal
- grasping
- contact
- payload handling
- ROS arm executor validation
- LLM control of the robot
- live evidence can be reused indefinitely

## Next Recommended Step

Freeze this stage before Arm-C1-H. The next hardware attempt should be exactly
one supervised run:

```text
fresh live_base_zero_evidence <= 60s
-> one valid Arm-C0 candidate
-> one validated Arm-B3 no-load sequence
-> return to 6b
-> episode_report + LLM-A report
```

Do not run multiple candidates or contact any obstacle in Arm-C1-H.
