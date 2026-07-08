# Arm-C1 Hardware Gate Script Design

Date: 2026-06-30

Status: gate script implemented for dry-run and future explicitly confirmed
hardware use. No Arm-C1 real hardware execution has been performed in this
step.

## Purpose

Arm-C1 will be a future explicitly confirmed hardware step that consumes one
approved Arm-C0 candidate and executes only a validated no-load sequence after
live safety gates pass.

Arm-C1 is not autonomous obstacle removal. It is not grasping, contact, payload
handling, or clearing. It is a map-gated no-load response validation.

## Proposed Script

Script name:

```text
tools/run_arm_c1_map_gated_no_load_once.py
```

The script should execute at most one candidate per invocation.

Required inputs for future hardware execution:

```text
--arm-c0-episode-report outputs/arm_c0_map_to_arm_dryrun_v1/offline_p4x/episode_report.json
--candidate-id arm_c0_candidate_001
--serial-port /dev/arm_bus
--output-dir outputs/arm_c1_map_gated_no_load_once_v1/<run_id>
--enable-hardware-write
--confirm-map-gated-no-load
--confirm-no-contact
--confirm-base-zero-live
--confirm-no-cmd-vel
--base-zero-evidence <live_base_zero_evidence.json>
--base-zero-max-age-s 60
```

The default mode is dry-run. Hardware writes remain disabled unless all
explicit confirmation flags are present and `--base-zero-evidence` proves live
`base_zero_ok_before_arm=true`.

`tools/generate_arm_c1_base_zero_evidence.py` now generates the required
evidence shape. Offline extraction from an existing P4-X `episode_report.json`
is valid for dry-run documentation only. Real Arm-C1 hardware execution requires
fresh live evidence with:

```text
evidence_type=live_base_zero_observation
valid_for_arm_c1_hardware=true
base_zero_ok_before_arm=true
published_cmd_vel=false
```

The Arm-C1 gate rejects offline evidence for hardware even if the offline
snapshot shows `base_zero_ok_before_arm=true`.

Dry-run validation command:

```powershell
python tools\run_arm_c1_map_gated_no_load_once.py --arm-c0-episode-report outputs\arm_c0_map_to_arm_dryrun_v1\offline_p4x\episode_report.json --candidate-id arm_c0_candidate_001 --output-dir outputs\arm_c1_map_gated_no_load_once_v1\dryrun_candidate_001
```

Dry-run result:

- `status=succeeded_dry_run`
- `hardware_executed=false`
- `serial_port_opened=false`
- `serial_bytes_written=0`
- `published_cmd_vel=false`
- `contact_allowed=false`
- `obstacle_removed=false`
- `errors.json=[]`

## Required Preconditions

Physical preconditions:

- mechanical arm is no-load
- no object is held by the gripper
- no obstacle is within contact distance
- operator is watching the arm
- emergency power-off is reachable
- arm starts near `6b` safe idle/home
- bus-servo controller power is on
- external servo power is stable

Software preconditions:

- no ROS arm executor is running
- no unrelated servo control process is running
- no wheel/base motion process is issuing movement
- serial device is present
- current repository code matches the expected Arm-B/Arm-C0 version
- Arm-C0 candidate exists and has `status=succeeded_dry_run`

## Mandatory Safety Gates

The script must check and record:

- `base_zero_ok_before_arm=true`
- `published_cmd_vel_before=false`
- `published_cmd_vel_during_arm=false`
- `candidate.status=succeeded_dry_run`
- `candidate.selected_action=ARM_SAMPLE_NO_LOAD`
- `candidate.selected_sequence=arm_b3_8_step_safety_adjusted_no_load_sample`
- `candidate.validated_no_load_action=true`
- `candidate.contact_allowed=false`
- `candidate.obstacle_removed=false`
- `candidate.hardware_executed=false` in the source Arm-C0 record
- `serial_write_allowed_effective=true` only after explicit Arm-C1 confirmation
- `--base-zero-evidence` exists and reports `base_zero_ok_before_arm=true`
- `--base-zero-evidence` has `evidence_type=live_base_zero_observation`
- `--base-zero-evidence` has `valid_for_arm_c1_hardware=true`
- `--base-zero-evidence` is fresh within `--base-zero-max-age-s`
- `--confirm-no-cmd-vel` is present for hardware execution

If any gate fails, the script must not open the serial port for writes and must
write:

```text
status=failed_safe
hardware_executed=false
serial_bytes_written=0
```

## Allowed Action

Only this no-load action is allowed:

```text
ARM_SAMPLE_NO_LOAD
arm_b3_8_step_safety_adjusted_no_load_sample
```

The action must return to:

```text
6b safe_idle_home_like
#1 P510 #2 P771 #3 P426 #4 P503 #5 P497
```

No contact, grasping, load, or obstacle interaction is allowed.

## Explicitly Forbidden

Arm-C1 must not:

- start Gazebo or RL
- start autonomous navigation
- claim SLAM
- publish movement `cmd_vel`
- run arbitrary Arm-B/B3 sequences
- select arm actions through LLM output
- use LLM in the control loop
- execute contact, grasping, payload, or obstacle removal
- run if `base_zero_ok_before_arm` is not true
- continue after any failed safety gate

## Proposed Outputs

Future evidence directory:

```text
outputs/arm_c1_map_gated_no_load_once_v1/<run_id>/
```

Required files:

- `episode_report.json`
- `action_result.json`
- `arm_c1_status.json`
- `selected_candidate.json`
- `sent_frame_hex.txt`
- `physical_actuation_confirmation.json`
- `errors.json`
- `README.md`

Dry-run evidence already generated:

```text
outputs/arm_c1_map_gated_no_load_once_v1/dryrun_candidate_001/
```

Offline base-zero evidence for dry-run/reporting:

```text
outputs/arm_c1_base_zero_evidence_v1/offline_from_p4x/
```

## Acceptance Criteria

For the first Arm-C1 run:

- exactly one candidate is selected
- live `base_zero_ok_before_arm=true`
- no base `cmd_vel` is published during the arm step
- serial port opens only after explicit confirmations
- selected no-load sequence executes once
- arm returns to `6b`
- `contact_allowed=false`
- `obstacle_removed=false`
- operator confirms no contact, no abnormal sound, no binding, and no visible overheating
- `episode_report.json` is generated
- LLM-A report can be generated afterward from the Arm-C1 episode report

Current dry-run acceptance:

- exactly one candidate selected: `arm_c0_candidate_001`
- candidate gate passed
- no serial port opened
- no serial bytes written
- no hardware execution
- no contact and no obstacle removal claim

## Claim Boundary

Even after a successful Arm-C1 run, the allowed claim should remain:

```text
Map-gated no-load mechanical-arm response executed once after live base-zero
safety confirmation.
```

It must still not claim:

- real obstacle removal
- grasping
- payload handling
- contact manipulation
- autonomous semantic reasoning
- LLM control
- SLAM or autonomous navigation
