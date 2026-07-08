# Arm-B2 / Arm-B3 No-Load Validation

Date: 2026-06-30

## Conclusion

Arm-B3 is frozen as a hardware milestone. K1 has validated the real serial control path to the bus-servo controller and completed a full mechanical-arm sample sequence under no-load conditions. The final accepted run executed an 8-step safety-adjusted sample sequence, returned to `safe_idle_home_like_6b`, and did not start ROS or publish `cmd_vel`.

This stage does not claim grasping, contact, payload handling, obstacle removal, autonomous execution, or a ROS arm executor.

## Milestone Status

```text
Arm-B0  K1 serial device audit + dry-run safety gate      PASS
Arm-B1  6b safe idle/home single-frame return             PASS
Arm-B2  single-servo / support-chain no-load validation   PASS
Arm-B3  full 8-step no-load sample sequence               PASS
```

## Arm-B2 Results

Arm-B2 was executed as isolated no-load checks. Each run required `base_zero_ok_before=true`, did not publish `cmd_vel`, did not allow contact, and did not allow obstacle removal.

| item | target | evidence directory | result |
| --- | --- | --- | --- |
| ID5 gripper | `497 -> 360 -> 497` | `outputs/arm_b2_single_servo_no_load_v1/hw_id5_360/` | PASS |
| ID4 wrist | `503 -> 470 -> 503` | `outputs/arm_b2_single_servo_no_load_v1/hw_id4_470/` | PASS |
| ID3 elbow | `426 -> 526 -> 426` | `outputs/arm_b2_single_servo_no_load_v1/hw_id3_526/` | PASS |
| ID2 shoulder | `771 -> 671 -> 771` | `outputs/arm_b2_single_servo_no_load_v1/hw_id2_671/` | PASS |
| ID1 yaw with ID2 support | `ID2 771 -> 671`, `ID1 510 -> 610 -> 510`, return `6b` | `outputs/arm_b2_single_servo_no_load_v1/hw_id1_610_id2_671_support_repeat1/` | PASS |

Observed boundaries:

- ID2 did not exceed `771`.
- ID1 yaw was executed only after ID2 moved to the validated support pulse `671`, satisfying `ID2 >= 600`.
- No B2 run used obstacle contact or payload.
- Each accepted run has operator confirmation with `physical_issue_observed=false`.

## Arm-B3 Result

Accepted run:

```text
outputs/arm_b3_no_load_sample_sequence_v1/hw_sequence_002/
```

Result summary:

```text
status=succeeded
dry_run=false
hardware_executed=true
step_count=8
step_success_count=8
step_ok=[true, true, true, true, true, true, true, true]
controller_response_observed=true
battery_mv=11304
published_cmd_vel=false
contact_allowed=false
obstacle_removed=false
errors=[]
```

Step trace:

| step | name | result |
| ---: | --- | --- |
| 1 | `step_1_safe_flat_start` | succeeded |
| 2 | `step_2_mid_retract` | succeeded |
| 3 | `step_3a_pre_reach` | succeeded |
| 4 | `step_3b_reach_no_load` | succeeded |
| 5 | `step_4_pre_gripper` | succeeded |
| 6 | `step_5_gripper_open_no_object` | succeeded |
| 7 | `step_6a_return_mid` | succeeded |
| 8 | `step_6b_return_home_like` | succeeded |

Operator confirmation for `hw_sequence_002`:

```text
physical_actuation_observed=true
physical_issue_observed=false
motion_quality=acceptable
returned_home=true
object_contact_observed=false
```

## Evidence Files

Arm-B1:

```text
outputs/arm_b1_send_home_once_v1/episode_report.json
outputs/arm_b1_send_home_once_v1/action_result.json
outputs/arm_b1_send_home_once_v1/physical_actuation_confirmation.json
```

Arm-B2:

```text
outputs/arm_b2_single_servo_no_load_v1/hw_id5_360/episode_report.json
outputs/arm_b2_single_servo_no_load_v1/hw_id5_360/physical_actuation_confirmation.json
outputs/arm_b2_single_servo_no_load_v1/hw_id4_470/episode_report.json
outputs/arm_b2_single_servo_no_load_v1/hw_id4_470/physical_actuation_confirmation.json
outputs/arm_b2_single_servo_no_load_v1/hw_id3_526/episode_report.json
outputs/arm_b2_single_servo_no_load_v1/hw_id3_526/physical_actuation_confirmation.json
outputs/arm_b2_single_servo_no_load_v1/hw_id2_671/episode_report.json
outputs/arm_b2_single_servo_no_load_v1/hw_id2_671/physical_actuation_confirmation.json
outputs/arm_b2_single_servo_no_load_v1/hw_id1_610_id2_671_support_repeat1/episode_report.json
outputs/arm_b2_single_servo_no_load_v1/hw_id1_610_id2_671_support_repeat1/physical_actuation_confirmation.json
```

Arm-B3:

```text
outputs/arm_b3_no_load_sample_sequence_v1/hw_sequence_002/episode_report.json
outputs/arm_b3_no_load_sample_sequence_v1/hw_sequence_002/action_results.json
outputs/arm_b3_no_load_sample_sequence_v1/hw_sequence_002/arm_b3_status.json
outputs/arm_b3_no_load_sample_sequence_v1/hw_sequence_002/physical_actuation_confirmation.json
outputs/arm_b3_no_load_sample_sequence_v1/hw_sequence_002/errors.json
```

## Claim Boundary

Allowed claim:

```text
The system has validated the real K1 serial control path to the bus-servo controller and completed a full mechanical-arm sample sequence under no-load conditions, with final return to safe idle/home.
```

Disallowed claims:

- Do not claim grasping.
- Do not claim contact.
- Do not claim payload handling.
- Do not claim obstacle removal.
- Do not claim autonomous execution.
- Do not claim a ROS arm executor has been validated.
- Do not claim Arm-C/D/E is complete.

## Stop Condition

Mechanical-arm hardware action is stopped after Arm-B3. Do not run:

- `hw_sequence_003`
- foam contact
- obstacle contact
- ROS arm executor integration
- autonomous arm/base coordination
- real obstacle removal

The next recommended non-arm task is Map-A: offline projection from D435 `risk_point` plus odom into risk-map evidence.
