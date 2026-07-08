# P4 Guarded Policy Executable Modes

Date: `2026-06-29`

This note documents the first executable guarded policy modes. It is not a run
record; no robot motion has been executed for these modes yet.

## Modes

### guarded-policy-dry-run

Purpose:

- observe current guarded mapping state
- select a would-be action from the active behavior profile
- do not publish `/input_cmd_vel`
- do not save maps
- exit after a fixed duration or sample count

Example:

```bash
python3 /home/soc/edge-ai-robot-k1/tools/guarded_auto_mapping_micro.py \
  --mode guarded-policy-dry-run \
  --behavior-profile interaction_mode \
  --policy-duration-s 30 \
  --confirm YES
```

### guarded-policy-step

Purpose:

- read one state snapshot
- select one action from the active behavior profile
- execute at most one primitive
- zero-hold, verify `base_zero_ok`, save one map, and exit

This mode does not loop and must not be treated as free exploration.

Example:

```bash
python3 /home/soc/edge-ai-robot-k1/tools/guarded_auto_mapping_micro.py \
  --mode guarded-policy-step \
  --behavior-profile interaction_mode \
  --policy-arc-direction auto \
  --confirm YES
```

## Profiles

`mapping_safe_mode`:

```text
front_p10 < 0.50m        -> HARD_STOP
0.50m <= front_p10 < 0.80m -> HOLD_AND_SAVE
0.80m <= front_p10 < 1.20m -> ARC30_PREFERRED
front_p10 >= 1.20m      -> FORWARD_ALLOWED
```

`interaction_mode`:

```text
front_min < 0.20m        -> HARD_STOP
front_p10 < 0.30m        -> HOLD_AND_CAPTURE
0.30m <= front_p10 < 0.40m -> HOLD_SAVE_OBSERVE
0.40m <= front_p10 < 0.60m -> ARC30_OR_FORWARD_0P05
0.60m <= front_p10 < 0.80m -> ARC30_OR_FORWARD_0P10
front_p10 >= 0.80m      -> FORWARD_0P15_OR_ARC30
```

## Reference Adaptation

The implementation checked existing WHEELTEC local avoidance code under:

```text
K:\risc-vCar\ros相关\src\wheeltec_multi\src\multi_avoidance.cpp
K:\risc-vCar\ros相关\src\wheeltec_jetracer_ros2\wheeltec_jetracer\wheeltec_jetracer\laser_detect.py
```

The useful ideas were:

- avoid relying on a single raw range sample
- derive local left/right/front context from laser sectors
- turn toward the side with more clearance

The policy adapts those ideas conservatively:

- `front_p10` and `front_min` from `/safety/front_obstacle` remain the authority
  for action selection and hard stops.
- `/scan` sector statistics are used only for reporting and automatic arc
  direction choice.
- `--policy-arc-direction auto` compares left/right sector p10 and chooses the
  clearer side.
- If side scan data is unavailable, auto direction falls back to left and logs
  the fallback reason.

The policy does not copy the old continuous velocity mixer. It still executes
only the validated spatial primitives:

```text
forward-staged
arc-yaw-closed
hold/save/capture-placeholder
```

## Logging

Every dry-run sample records:

```text
profile
front_min
front_p10
selected_action
would_execute_action
arc_direction
arc_direction_reason
executed=false
base_zero_ok
odom_before/after
map_saved=false
stop_reason
scan_sectors
```

Every step run records:

```text
profile
front_min
front_p10
selected_action
action_reason
execution_action
arc_direction
arc_direction_reason
executed
base_zero_ok
odom_before/after
map_saved
stop_reason
action_record
```

## Safety Boundary

These modes still obey the P4 constraints:

- publish motion only to `/input_cmd_vel`
- do not bypass `scan_safety_guard_node`
- do not start RRT
- do not start AMCL
- do not send Nav2 goals
- do not run long loops
- `guarded-policy-step` executes at most one primitive

## P4-S Dry-Run Verification

Date: 2026-06-29

The first dry-run found a readiness bug: policy selection required
`map_fresh=true`, so a stable but not recently updated `/map_metadata` sample
caused `NOT_READY/stale_map` even though `/scan`, `/odom`, guard status and
base diagnostics were healthy. The policy now requires that map metadata has
been observed at least once, but it no longer requires map metadata freshness
for every dry-run decision.

Fixed dry-run commands used the actual CLI names:

```bash
python3 /home/soc/edge-ai-robot-k1/tools/guarded_auto_mapping_micro.py \
  --mode guarded-policy-dry-run \
  --behavior-profile interaction_mode \
  --policy-arc-direction auto \
  --policy-duration-s 30 \
  --report /home/soc/edge-ai-robot-k1/logs/policy_dry_interaction_fixed_20260629_012831.json \
  --confirm YES

python3 /home/soc/edge-ai-robot-k1/tools/guarded_auto_mapping_micro.py \
  --mode guarded-policy-dry-run \
  --behavior-profile mapping_safe_mode \
  --policy-arc-direction auto \
  --policy-duration-s 30 \
  --report /home/soc/edge-ai-robot-k1/logs/policy_dry_mapping_fixed_20260629_012948.json \
  --confirm YES
```

Results:

| Profile | Samples | Front p10 range | Selected action | Mismatches | Motion published | Maps saved | base_zero_bad |
| --- | ---: | ---: | --- | ---: | ---: | ---: | ---: |
| `interaction_mode` | 14 | 1.751-1.771 m | `FORWARD_0P15_OR_ARC30` | 0 | 0 | 0 | 0 |
| `mapping_safe_mode` | 14 | 1.748-1.769 m | `FORWARD_ALLOWED` | 0 | 0 | 0 | 0 |

This matched the configured threshold bands for an open front field near
1.75 m. No `/input_cmd_vel` was published in dry-run mode, and the remote ROS
processes were stopped after the reports were copied back.

## P4-T1-B Hold Observe Step

Date: 2026-06-29

Goal: verify that `interaction_mode` does not move when the front obstacle is
in the observation band.

Precheck:

```text
report: logs/policy_t1_precheck_interaction_20260629_014149.json
profile: interaction_mode
samples: 4
final stable band: 0.30m <= front_p10 < 0.40m
selected_action: HOLD_SAVE_OBSERVE
executed: false
base_zero_ok: true
```

Step result:

```text
report: logs/policy_t1b_step_hold_observe_20260629_014241.json
front_min: 0.370 m
front_p10: 0.377 m
selected_action: HOLD_SAVE_OBSERVE
execution_action: HOLD
executed: false
base_zero_ok: true
odom_delta: dx=0.000 m, dy=0.000 m, dyaw=0.00 deg
map_saved: true
map_prefix: maps/policy_t1b_hold_observe_20260629_014241_policy_step
stop_reason: none
```

Artifacts:

```text
logs/policy_t1b_step_hold_observe_20260629_014241.json
logs/policy_t1b_step_hold_observe_20260629_014241.run.log
maps/policy_t1b_hold_observe_20260629_014241_policy_step.{pgm,yaml,png}
maps/policy_t1b_hold_observe_20260629_014241_policy_step_marked.png
```

The result satisfies the P4-T1-B acceptance criteria: the policy selected a
near-object hold/observe action, executed no motion primitive, reported zero
odom change, verified base zero, saved a map snapshot, and the remote ROS
processes were cleaned up after the run.

## P4-T1-A Hold Capture Step

Date: 2026-06-29

Goal: verify that `interaction_mode` enters the placeholder capture task state
without moving when the front obstacle is in the 0.20-0.30 m task-interaction
band.

Precheck:

```text
report: logs/policy_t1a_precheck_interaction_20260629_014811.json
profile: interaction_mode
samples: 4
early band: 0.30m <= front_p10 < 0.40m -> HOLD_SAVE_OBSERVE
final stable band: front_p10 < 0.30m with front_min > 0.20m
selected_action: HOLD_AND_CAPTURE
executed: false
base_zero_ok: true
```

Step result:

```text
report: logs/policy_t1a_step_hold_capture_20260629_014851.json
front_min: 0.281 m
front_p10: 0.284 m
selected_action: HOLD_AND_CAPTURE
execution_action: HOLD
executed: false
base_zero_ok: true
odom_delta: dx=0.000 m, dy=0.000 m, dyaw=0.00 deg
map_saved: true
capture_event: placeholder_capture
capture_reason: front_p10 < 0.30m
map_prefix: maps/policy_t1a_hold_capture_20260629_014851_policy_step
stop_reason: none
```

Artifacts:

```text
logs/policy_t1a_precheck_interaction_20260629_014811.json
logs/policy_t1a_precheck_interaction_20260629_014811.run.log
logs/policy_t1a_step_hold_capture_20260629_014851.json
logs/policy_t1a_step_hold_capture_20260629_014851.run.log
maps/policy_t1a_hold_capture_20260629_014851_policy_step.{pgm,yaml,png}
maps/policy_t1a_hold_capture_20260629_014851_policy_step_marked.png
```

The result satisfies the P4-T1-A acceptance criteria: the policy selected the
capture hold state, executed no motion primitive, reported zero odom change,
verified base zero, saved a map snapshot, and emitted a placeholder capture
event containing odom, `front_p10`, map file prefix and capture reason.

## P4-T2-A Mapping Forward Step

Date: 2026-06-29

Goal: verify that `mapping_safe_mode` selects and executes exactly one forward
primitive in an open front field.

Dry-run precheck:

```text
report: logs/policy_t2a_precheck_mapping_20260629_015630.json
profile: mapping_safe_mode
samples: 4
front_p10: 1.471-1.472 m
selected_action: FORWARD_ALLOWED
would_execute_action: FORWARD_0P15
executed: false
base_zero_ok: true
```

Step result:

```text
report: logs/policy_t2a_step_mapping_forward_20260629_020507.json
profile: mapping_safe_mode
front_min: 1.465 m
front_p10: 1.470 m
selected_action: FORWARD_ALLOWED
execution_action: FORWARD_0P15
executed: true
base_zero_ok: true
target_forward: 0.150 m
actual_final_forward: 0.1563 m
overshoot: 0.0063 m
delta_yaw: -3.05 deg
front_p10_after: 1.313 m
stop_reason: none
action_stop_reason: brake_margin_dynamic
```

The motion part passed: only one staged forward primitive ran, it published
through `/input_cmd_vel`, the guard passed it, the final odom delta was within
the expected 0.15 m step size, and the base returned to zero.

Map save note:

```text
step map_save response: slam_toolbox.srv.SaveMap_Response(result=255)
slam_toolbox constant: RESULT_UNDEFINED_FAILURE=255
initial step map files: not generated
manual retry response: slam_toolbox.srv.SaveMap_Response(result=0)
retry map_prefix: maps/policy_t2a_mapping_forward_20260629_020507_policy_step_retry
```

The test exposed a reporting bug in `tools/guarded_auto_mapping_micro.py`:
`save_map()` treated any service response as success. It now checks
`result_code == 0`, records the result code, and marks non-zero responses as
failed. The K1 copy was updated and both local and K1 `py_compile` passed after
the fix.

Artifacts:

```text
logs/policy_t2a_precheck_mapping_20260629_015630.json
logs/policy_t2a_precheck_mapping_20260629_015630.run.log
logs/policy_t2_precheck_20260629_015601.launch.log
logs/policy_t2a_step_20260629_020439.launch.log
logs/policy_t2a_step_mapping_forward_20260629_020507.json
logs/policy_t2a_step_mapping_forward_20260629_020507.run.log
maps/policy_t2a_mapping_forward_20260629_020507_policy_step_retry.{pgm,yaml,png}
maps/policy_t2a_mapping_forward_20260629_020507_policy_step_retry_marked.png
```

Status: T2-A motion control passed. The map artifact exists via manual
slam_toolbox retry, and the discovered save-result bug is fixed for the next
policy-step run. This should not be counted as a clean automatic map-save pass.

## Save Map Retry Fix Verification

Date: 2026-06-29

The map saving helper was updated to:

- record every save attempt
- require `SaveMap_Response.result == 0`
- verify that both `.pgm` and `.yaml` files exist
- retry non-zero result codes or missing files
- expose `result_code`, `attempt_count`, per-attempt file status and errors

A no-motion `save-map-only` mode was added for validating this chain without
publishing `/input_cmd_vel`.

Save-only verification:

```text
report: logs/save_map_only_retry_20260629_021556.json
mode: save-map-only
published_input_cmd_vel: false
executed: false
odom_delta: dx=0.000 m, dy=0.000 m, dyaw=0.00 deg
base_zero_ok: true
map_saved: true
attempt_count: 1
result_code: 0
files_verified: true
map_prefix: maps/save_map_only_retry_20260629_021556_save_only
```

Clean T2-A rerun after the fix:

```text
report: logs/policy_t2a_step_mapping_forward_clean_20260629_021643.json
profile: mapping_safe_mode
front_min: 1.311 m
front_p10: 1.313 m
selected_action: FORWARD_ALLOWED
execution_action: FORWARD_0P15
executed: true
base_zero_ok: true
target_forward: 0.150 m
actual_final_forward: 0.1747 m
overshoot: 0.0247 m
delta_yaw: 0.47 deg
front_p10_after: 1.133 m
map_saved: true
attempt_count: 1
result_code: 0
files_verified: true
map_prefix: maps/policy_t2a_mapping_forward_clean_20260629_021643_policy_step
stop_reason: none
action_stop_reason: brake_margin_dynamic
```

Artifacts:

```text
logs/save_map_retry_test_20260629_021520.launch.log
logs/save_map_only_retry_20260629_021556.json
logs/save_map_only_retry_20260629_021556.run.log
logs/policy_t2a_step_mapping_forward_clean_20260629_021643.json
logs/policy_t2a_step_mapping_forward_clean_20260629_021643.run.log
maps/save_map_only_retry_20260629_021556_save_only.{pgm,yaml,png}
maps/save_map_only_retry_20260629_021556_save_only_marked.png
maps/policy_t2a_mapping_forward_clean_20260629_021643_policy_step.{pgm,yaml,png}
maps/policy_t2a_mapping_forward_clean_20260629_021643_policy_step_marked.png
```

Status: the clean T2-A rerun passed as a complete single-action policy step:
one forward primitive, base returned to zero, front distance remained above the
hard safety zone, and automatic map saving succeeded with verified files.

## P4-T2-B Interaction Single Forward Step

Date: 2026-06-29

Goal: verify `interaction_mode` can select and execute exactly one safe
movement primitive in open space, still using `/input_cmd_vel` through
`scan_safety_guard_node`.

Dry-run precheck:

```text
report: logs/policy_t2b_precheck_interaction_20260629_022204.json
profile: interaction_mode
samples: 4
front_min: 1.839-1.846 m
front_p10: 1.845-1.846 m
selected_action: FORWARD_0P15_OR_ARC30
would_execute_action: FORWARD_0P15
executed: false
base_zero_ok: true
```

Step result:

```text
report: logs/policy_t2b_step_interaction_forward_20260629_022250.json
profile: interaction_mode
front_min: 1.831 m
front_p10: 1.846 m
selected_action: FORWARD_0P15_OR_ARC30
execution_action: FORWARD_0P15
executed: true
base_zero_ok: true
target_forward: 0.150 m
actual_final_forward: 0.1930 m
overshoot: 0.0430 m
lateral_delta: -0.0032 m
delta_yaw: -1.30 deg
front_p10_after: 2.028 m
map_saved: true
attempt_count: 1
result_code: 0
files_verified: true
map_size_after: 179 x 226
stop_reason: none
action_stop_reason: brake_margin_dynamic
```

The single action passed the executable policy-step contract: one forward
primitive was selected, one forward primitive was executed, the robot stopped
cleanly, `base_zero_ok=true`, and the map save succeeded on the first verified
attempt.

The final forward distance was 0.193 m for a 0.150 m target. This is still
inside the currently accepted open-space interaction step behavior, but it
should be treated as a margin requirement before any bounded multi-step run:
the next test area should leave at least 0.30-0.40 m extra forward clearance,
or the policy should use a smaller target/brake margin if a tighter approach is
needed.

Artifacts:

```text
logs/policy_t2b_step_20260629_022119.launch.log
logs/policy_t2b_precheck_interaction_20260629_022204.json
logs/policy_t2b_precheck_interaction_20260629_022204.run.log
logs/policy_t2b_step_interaction_forward_20260629_022250.json
logs/policy_t2b_step_interaction_forward_20260629_022250.run.log
maps/policy_t2b_interaction_forward_20260629_022250_policy_step.{pgm,yaml,png}
maps/policy_t2b_interaction_forward_20260629_022250_policy_step_marked.png
```

Remote cleanup: launch PID 8731 and child nodes 8746, 8747, 8749 and 8799 were
stopped, FastDDS shared-memory files were cleared, and no matching guarded
mapping/SLAM/laser/base processes remained.

Status: P4-T2-B passed as the interaction-mode open-space single-action step.
Together with T1-A/T1-B and the clean T2-A rerun, the policy layer has now
shown both near-field hold behavior and one-step open-space motion behavior.

## P4-U Bounded Policy Run

Date: 2026-06-29

`guarded-policy-run` was added as the first bounded executable policy loop. It
is deliberately not a free exploration mode:

- hard limited by `--policy-max-steps`, validated to 1-3
- hard limited by `--policy-max-runtime-s`
- every step re-runs the policy precheck
- every step executes at most one selected primitive
- every step saves a map after successful stop
- any `HOLD`, `HARD_STOP`, `NOT_READY`, base-zero failure or map-save failure
  stops the sequence

Local and K1 `py_compile` passed before running it on the robot.

Dry-run precheck:

```text
report: logs/policy_p4u_precheck_interaction_20260629_023231.json
profile: interaction_mode
samples: 3
front_p10: 1.971-1.989 m
selected_action: FORWARD_0P15_OR_ARC30
would_execute_action: FORWARD_0P15
executed: false
base_zero_ok: true
```

Bounded run:

```text
report: logs/policy_p4u_run_interaction_20260629_023329.json
profile: interaction_mode
max_steps: 3
max_runtime_s: 120
sequence_stop_reason: max_steps_reached
step_count: 3
executed_count: 3
base_zero_ok: true
saved_maps: 3/3
map_save_result_codes: 0, 0, 0
```

Step summary:

```text
step 1:
  selected_action: FORWARD_0P15_OR_ARC30
  execution_action: FORWARD_0P15
  front_p10: 2.022 -> 1.828 m
  odom_forward: 0.1678 m
  yaw_delta: -0.28 deg
  base_zero_ok: true
  map_saved: true

step 2:
  selected_action: FORWARD_0P15_OR_ARC30
  execution_action: FORWARD_0P15
  front_p10: 1.866 -> 1.696 m
  odom_forward: 0.1632 m
  yaw_delta: -1.42 deg
  base_zero_ok: true
  map_saved: true

step 3:
  selected_action: FORWARD_0P15_OR_ARC30
  execution_action: FORWARD_0P15
  front_p10: 1.719 -> 1.545 m
  odom_forward: 0.1722 m
  yaw_delta: +1.30 deg
  base_zero_ok: true
  map_saved: true
```

Sequence odom:

```text
total_forward_delta: 0.5033 m
total_lateral_delta: 0.0011 m
total_yaw_delta: -0.40 deg
```

Map sizes:

```text
maps/policy_p4u_interaction_20260629_023329_01_policy: 179 x 224
maps/policy_p4u_interaction_20260629_023329_02_policy: 179 x 226
maps/policy_p4u_interaction_20260629_023329_03_policy: 182 x 227
```

Artifacts:

```text
logs/policy_p4u_bounded_20260629_023135.launch.log
logs/policy_p4u_precheck_interaction_20260629_023231.json
logs/policy_p4u_precheck_interaction_20260629_023231.run.log
logs/policy_p4u_run_interaction_20260629_023329.json
logs/policy_p4u_run_interaction_20260629_023329.run.log
maps/policy_p4u_interaction_20260629_023329_01_policy.{pgm,yaml,png}
maps/policy_p4u_interaction_20260629_023329_01_policy_marked.png
maps/policy_p4u_interaction_20260629_023329_02_policy.{pgm,yaml,png}
maps/policy_p4u_interaction_20260629_023329_02_policy_marked.png
maps/policy_p4u_interaction_20260629_023329_03_policy.{pgm,yaml,png}
maps/policy_p4u_interaction_20260629_023329_03_policy_marked.png
```

Remote cleanup: launch PID 9543 and child nodes 9558, 9559, 9560, 9561 and
9610 were stopped, FastDDS shared-memory files were cleared, and no matching
guarded mapping/SLAM/laser/base processes remained.

Status: P4-U passed. This is the first verified bounded autonomous policy run:
the system performed three guarded policy decisions, three guarded forward
primitives, three stop-and-zero checks, and three verified map saves without
RRT, AMCL, Nav2 goals or any direct bypass around the scan guard.

## P4-V Policy Branch Tests

Date: 2026-06-29

P4-V tested the interaction-mode policy branches that P4-U did not cover:

- mid-range arc step
- near-range hold in bounded-run mode
- mid-range arc followed by a second policy redecision

The stack was still the guarded mapping stack. Motion commands went only to
`/input_cmd_vel`, through `scan_safety_guard_node`, then to
`/cmd_vel_guarded`. No RRT, AMCL, Nav2 goals or direct chassis command path was
used.

For these interaction-mode tests the guarded launch used the interaction
front-distance guard values:

```text
hard_stop_m: 0.30
emergency_stop_m: 0.20
slow_down_m: 0.80
approach_stop_m: 0.80
```

This kept the scan guard active while avoiding the earlier mapping-mode
`hard_stop_m=1.00` behavior, which correctly blocked arc tests around
`front_p10 ~= 0.60-0.80 m`.

### V1: ARC30 single step

Precheck:

```text
report: logs/policy_p4v_v1_precheck_interaction_guard_20260629_024202.json
profile: interaction_mode
front_p10: 0.675-0.677 m
front_state: warning
selected_action: ARC30_OR_FORWARD_0P10
would_execute_action: ARC30_RIGHT
base_zero_ok: true
```

Step result:

```text
report: logs/policy_p4v_v1_step_arc_interaction_20260629_024252.json
selected_action: ARC30_OR_FORWARD_0P10
execution_action: ARC30_RIGHT
executed: true
base_zero_ok: true
map_saved: true
front_p10: 0.676 -> 1.313 m
odom_forward: 0.0919 m
odom_lateral: -0.0348 m
yaw_delta: -32.88 deg
arc_steps: 3
arc_stop_reason: target_band_reached
```

Status: V1 passed. In the 0.60-0.80 m band the policy selected and executed one
arc primitive, stopped cleanly, and saved a map.

### V2: HOLD bounded run

Precheck:

```text
report: logs/policy_p4v_v2_precheck_hold_20260629_024609.json
profile: interaction_mode
front_p10: 0.342-0.343 m
selected_action: HOLD_SAVE_OBSERVE
would_execute_action: HOLD
base_zero_ok: true
```

Bounded-run result:

```text
report: logs/policy_p4v_v2_run_hold_20260629_024641.json
selected_action: HOLD_SAVE_OBSERVE
execution_action: HOLD
step_count: 1
executed_count: 0
sequence_stop_reason: hold_action_HOLD_SAVE_OBSERVE
base_zero_ok: true
map_saved: true
front_p10: 0.342 -> 0.341 m
odom_forward: 0.0000 m
yaw_delta: 0.00 deg
```

Status: V2 passed. Bounded-run mode did not turn a near-range hold decision into
motion. It saved the map and exited after the hold action.

### V3: Mid-range bounded run and front-block redecision

The first V3 run deliberately tested the 0.50-0.70 m interaction band:

```text
report: logs/policy_p4v_v3_run_arc_20260629_024915.json
selected_action: ARC30_OR_FORWARD_0P10
execution_action: ARC30_LEFT
step_count: 1
executed_count: 1
base_zero_ok: true
map_saved: true
front_p10: 0.606 -> 0.573 m
odom_forward: 0.0390 m
yaw_delta: +9.19 deg
sequence_stop_reason:
  guarded_policy_run_01_arc30_left_2: front_blocked: front_p10 0.571 < 0.60
```

This was safe, but too conservative for bounded policy behavior. A primitive
that is stopped by its own front gate should not automatically terminate the
whole bounded sequence if the base is zeroed and the map save succeeded. It
should return to the outer policy layer for one more state read and decision.

Implementation change:

```text
tools/guarded_auto_mapping_micro.py
run_guarded_policy_run:
  continue after front_blocked only when:
    - the stop reason is a front_blocked gate
    - base_zero_ok remains true
    - map_save is successful
  still stop immediately on:
    - base_zero_failed
    - map_save_failed
    - NOT_READY
    - HOLD
    - hard policy stop reasons
```

After this change V3b was rerun:

```text
precheck: logs/policy_p4v_v3b_precheck_after_patch_20260629_025216.json
run: logs/policy_p4v_v3b_run_after_patch_20260629_025248.json
profile: interaction_mode
sequence_stop_reason: max_steps_reached
step_count: 2
executed_count: 2
base_zero_ok: true
saved_maps: 2/2
total_forward_delta: 0.2696 m
total_lateral_delta: 0.1359 m
total_yaw_delta: +35.51 deg
```

Step summary:

```text
step 1:
  selected_action: ARC30_OR_FORWARD_0P05
  execution_action: ARC30_LEFT
  front_p10: 0.569 -> 1.106 m
  odom_forward: 0.1050 m
  odom_lateral: 0.0289 m
  yaw_delta: +31.28 deg
  arc_steps: 2
  arc_stop_reason: target_band_reached
  base_zero_ok: true
  map_saved: true

step 2:
  selected_action: FORWARD_0P15_OR_ARC30
  execution_action: FORWARD_0P15
  front_p10: 1.107 -> 0.821 m
  odom_forward: 0.1963 m
  odom_lateral: 0.0060 m
  yaw_delta: +4.22 deg
  forward_stop_reason: brake_margin_dynamic
  base_zero_ok: true
  map_saved: true
```

Map sizes:

```text
maps/policy_p4v_v1_arc_interaction_20260629_024252_policy_step: 178 x 176
maps/policy_p4v_v2_hold_20260629_024641_01_policy: 188 x 176
maps/policy_p4v_v3_arc_20260629_024915_01_policy: 188 x 176
maps/policy_p4v_v3b_after_patch_20260629_025248_01_policy: 188 x 181
maps/policy_p4v_v3b_after_patch_20260629_025248_02_policy: 188 x 181
```

Artifacts:

```text
logs/policy_p4v_interaction_guard_20260629_024123.launch.log
logs/policy_p4v_v1_precheck_interaction_guard_20260629_024202.json
logs/policy_p4v_v1_precheck_interaction_guard_20260629_024202.run.log
logs/policy_p4v_v1_step_arc_interaction_20260629_024252.json
logs/policy_p4v_v1_step_arc_interaction_20260629_024252.run.log
logs/policy_p4v_v2_precheck_hold_20260629_024609.json
logs/policy_p4v_v2_precheck_hold_20260629_024609.run.log
logs/policy_p4v_v2_run_hold_20260629_024641.json
logs/policy_p4v_v2_run_hold_20260629_024641.run.log
logs/policy_p4v_v3_precheck_arc_run_20260629_024843.json
logs/policy_p4v_v3_precheck_arc_run_20260629_024843.run.log
logs/policy_p4v_v3_run_arc_20260629_024915.json
logs/policy_p4v_v3_run_arc_20260629_024915.run.log
logs/policy_p4v_v3b_precheck_after_patch_20260629_025216.json
logs/policy_p4v_v3b_precheck_after_patch_20260629_025216.run.log
logs/policy_p4v_v3b_run_after_patch_20260629_025248.json
logs/policy_p4v_v3b_run_after_patch_20260629_025248.run.log
maps/policy_p4v_v1_arc_interaction_20260629_024252_policy_step.{pgm,yaml,png}
maps/policy_p4v_v1_arc_interaction_20260629_024252_policy_step_marked.png
maps/policy_p4v_v2_hold_20260629_024641_01_policy.{pgm,yaml,png}
maps/policy_p4v_v2_hold_20260629_024641_01_policy_marked.png
maps/policy_p4v_v3_arc_20260629_024915_01_policy.{pgm,yaml,png}
maps/policy_p4v_v3_arc_20260629_024915_01_policy_marked.png
maps/policy_p4v_v3b_after_patch_20260629_025248_01_policy.{pgm,yaml,png}
maps/policy_p4v_v3b_after_patch_20260629_025248_01_policy_marked.png
maps/policy_p4v_v3b_after_patch_20260629_025248_02_policy.{pgm,yaml,png}
maps/policy_p4v_v3b_after_patch_20260629_025248_02_policy_marked.png
```

Remote cleanup: launch PID 10671 and child nodes 10686, 10687, 10688, 10689
and 10737 were stopped, FastDDS shared-memory files were cleared, and no
matching guarded mapping/SLAM/laser/base processes remained.

Status: P4-V passed. The policy layer now has executable evidence for open
space forward, near-range hold, mid-range arc, and bounded step-to-step
redecision after spatial state changes. It is still bounded and supervised; the
next step should be a very small `max_steps=3` branch-mixed run, not free
exploration.

## P4-W Branch-Mixed Runner

Date: 2026-06-29

P4-W is prepared as a bounded branch-mixed policy run. It is still not free
exploration:

- dry-run first
- then at most 3 movement decisions
- close and mid interaction bands prefer `arc30`
- open band prefers `forward 0.15m`
- every executed step still goes through `/input_cmd_vel` and the scan guard
- every executed step must stop, zero, save map, and return to the policy layer

Helper script:

```text
tools/p4w_guarded_policy_branch_mixed.sh
```

The script does not start ROS nodes. Start the guarded mapping stack first with
the interaction-mode guard thresholds used in P4-V:

```text
hard_stop_m:=0.30
emergency_stop_m:=0.20
slow_down_m:=0.80
approach_stop_m:=0.80
```

Dry-run command:

```bash
tools/p4w_guarded_policy_branch_mixed.sh dry-run
```

Bounded-run command:

```bash
tools/p4w_guarded_policy_branch_mixed.sh run
```

Default policy choices:

```text
POLICY_CLOSE_ACTION=arc30
POLICY_MID_ACTION=arc30
POLICY_NORMAL_ACTION=forward
POLICY_ARC_DIRECTION=auto
POLICY_MAX_STEPS=3
POLICY_MAX_RUNTIME_S=120
ZERO_HOLD_S=5.0
```

Expected acceptance:

```text
step_count <= 3
at least one ARC30 action when front_p10 is in the 0.40-0.80m band
FORWARD_0P15 only when front_p10 >= 0.80m
HOLD if front_p10 enters the 0.30-0.40m band
base_zero_ok=true for every step
map_saved=true for every step
front_min never below 0.20m
no RRT, AMCL, Nav2 goal, or guard bypass
```

## P4-W Branch-Mixed Run Result

Date: 2026-06-29

The guarded stack was launched with the same interaction-mode guard thresholds
used in P4-V:

```text
launch_log: logs/policy_p4w_branch_mixed_20260629_143119.launch.log
hard_stop_m: 0.30
emergency_stop_m: 0.20
slow_down_m: 0.80
approach_stop_m: 0.80
```

Dry-run:

```text
report: logs/policy_p4w_precheck_branch_mixed_20260629_143316.json
sample_count: 8
action_counts: FORWARD_0P15_OR_ARC30 = 8
would_execute_action: FORWARD_0P15
front_p10: 1.013 -> 0.819 m
executed: false
base_zero_ok: true
```

The dry-run did not publish motion. It showed the robot was just above the
`front_p10 >= 0.80m` open-band threshold, so the first bounded-run action was
expected to be `FORWARD_0P15`.

Bounded run:

```text
report: logs/policy_p4w_run_branch_mixed_20260629_143407.json
profile: interaction_mode
sequence_stop_reason: max_steps_reached
step_count: 3
executed_count: 3
base_zero_ok: true
saved_maps: 3/3
total_forward_delta: 0.3362 m
total_lateral_delta: -0.0978 m
total_yaw_delta: -30.53 deg
```

Step summary:

```text
step 1:
  selected_action: FORWARD_0P15_OR_ARC30
  execution_action: FORWARD_0P15
  front_p10: 0.818 -> 0.695 m
  odom_forward: 0.1198 m
  yaw_delta: +2.61 deg
  stop_reason: front_blocked: front_p10 0.794 < 0.80
  sequence_continue_after_front_block: true
  base_zero_ok: true
  map_saved: true

step 2:
  selected_action: ARC30_OR_FORWARD_0P10
  execution_action: ARC30_RIGHT
  front_p10: 0.695 -> 1.409 m
  odom_forward: 0.0734 m
  odom_lateral: -0.0241 m
  yaw_delta: -31.59 deg
  arc_steps: 2
  arc_stop_reason: target_band_reached
  base_zero_ok: true
  map_saved: true

step 3:
  selected_action: FORWARD_0P15_OR_ARC30
  execution_action: FORWARD_0P15
  front_p10: 1.410 -> 1.249 m
  odom_forward: 0.1634 m
  yaw_delta: -1.56 deg
  forward_stop_reason: brake_margin_dynamic
  base_zero_ok: true
  map_saved: true
```

Map sizes:

```text
maps/policy_p4w_branch_mixed_20260629_143407_01_policy: 84 x 224
maps/policy_p4w_branch_mixed_20260629_143407_02_policy: 112 x 225
maps/policy_p4w_branch_mixed_20260629_143407_03_policy: 113 x 225
```

Artifacts:

```text
logs/policy_p4w_branch_mixed_20260629_143119.launch.log
logs/policy_p4w_precheck_branch_mixed_20260629_143316.json
logs/policy_p4w_precheck_branch_mixed_20260629_143316.run.log
logs/policy_p4w_run_branch_mixed_20260629_143407.json
logs/policy_p4w_run_branch_mixed_20260629_143407.run.log
maps/policy_p4w_branch_mixed_20260629_143407_01_policy.{pgm,yaml,png}
maps/policy_p4w_branch_mixed_20260629_143407_01_policy_marked.png
maps/policy_p4w_branch_mixed_20260629_143407_02_policy.{pgm,yaml,png}
maps/policy_p4w_branch_mixed_20260629_143407_02_policy_marked.png
maps/policy_p4w_branch_mixed_20260629_143407_03_policy.{pgm,yaml,png}
maps/policy_p4w_branch_mixed_20260629_143407_03_policy_marked.png
```

Remote cleanup: launch PID 1997 and child nodes 2011, 2012, 2014 and 2062 were
stopped, FastDDS shared-memory files were cleared, and no matching guarded
mapping/SLAM/laser/base processes remained.

Status: P4-W passed. This is the first verified branch-mixed bounded policy
run: the policy chose open-space forward, then mid-range arc, then open-space
forward again, with a map save and base-zero check after every step.

## P4-W-Speed-A Zero Hold 3s

Date: 2026-06-29

P4-W-Speed-A repeated the same branch-mixed bounded policy run while changing
only one timing parameter:

```text
ZERO_HOLD_S: 5.0 -> 3.0
policy profile: interaction_mode
policy thresholds: unchanged
policy action preferences: unchanged
max_steps: 3
```

Dry-run:

```text
report: logs/policy_p4w_precheck_branch_mixed_20260629_145324.json
sample_count: 6
action_counts: FORWARD_0P15_OR_ARC30 = 6
would_execute_action: FORWARD_0P15
front_p10: 0.873 -> 0.872 m
executed: false
base_zero_ok: true
```

Bounded run:

```text
launch_log: logs/policy_p4w_speed_a_20260629_145220.launch.log
report: logs/policy_p4w_run_branch_mixed_20260629_145424.json
sequence_stop_reason: max_steps_reached
step_count: 3
executed_count: 3
base_zero_ok: true
saved_maps: 3/3
total_forward_delta: 0.3762 m
total_lateral_delta: -0.1172 m
total_yaw_delta: -32.57 deg
```

Step summary:

```text
step 1:
  selected_action: FORWARD_0P15_OR_ARC30
  execution_action: FORWARD_0P15
  front_p10: 0.873 -> 0.730 m
  odom_forward: 0.1544 m
  yaw_delta: -1.44 deg
  stop_reason: front_blocked: front_p10 0.789 < 0.80
  sequence_continue_after_front_block: true
  base_zero_ok: true
  map_saved: true

step 2:
  selected_action: ARC30_OR_FORWARD_0P10
  execution_action: ARC30_RIGHT
  front_p10: 0.725 -> 1.461 m
  odom_forward: 0.0537 m
  yaw_delta: -27.66 deg
  arc_steps: 2
  arc_stop_reason: target_band_reached
  base_zero_ok: true
  map_saved: true

step 3:
  selected_action: FORWARD_0P15_OR_ARC30
  execution_action: FORWARD_0P15
  front_p10: 1.458 -> 1.272 m
  odom_forward: 0.1975 m
  yaw_delta: -3.47 deg
  forward_stop_reason: brake_margin_dynamic
  base_zero_ok: true
  map_saved: true
```

Timing comparison against the previous P4-W run:

```text
P4-W zero_hold=5.0:
  step 1 elapsed: 15.380 s
  step 2 elapsed: 36.797 s
  step 3 elapsed: 51.640 s

P4-W-Speed-A zero_hold=3.0:
  step 1 elapsed: 14.344 s
  step 2 elapsed: 31.827 s
  step 3 elapsed: 45.641 s
```

The 3s zero-hold run reduced the 3-step elapsed time by about 6.0 seconds while
keeping all stop/zero and map-save checks green.

Map sizes:

```text
maps/policy_p4w_branch_mixed_20260629_145424_01_policy: 77 x 224
maps/policy_p4w_branch_mixed_20260629_145424_02_policy: 113 x 224
maps/policy_p4w_branch_mixed_20260629_145424_03_policy: 114 x 225
```

Artifacts:

```text
logs/policy_p4w_speed_a_20260629_145220.launch.log
logs/policy_p4w_precheck_branch_mixed_20260629_145324.json
logs/policy_p4w_precheck_branch_mixed_20260629_145324.run.log
logs/policy_p4w_run_branch_mixed_20260629_145424.json
logs/policy_p4w_run_branch_mixed_20260629_145424.run.log
maps/policy_p4w_branch_mixed_20260629_145424_01_policy.{pgm,yaml,png}
maps/policy_p4w_branch_mixed_20260629_145424_01_policy_marked.png
maps/policy_p4w_branch_mixed_20260629_145424_02_policy.{pgm,yaml,png}
maps/policy_p4w_branch_mixed_20260629_145424_02_policy_marked.png
maps/policy_p4w_branch_mixed_20260629_145424_03_policy.{pgm,yaml,png}
maps/policy_p4w_branch_mixed_20260629_145424_03_policy_marked.png
```

Remote cleanup: launch PID 2931 and child nodes 2945, 2946, 2948 and 2996 were
stopped, FastDDS shared-memory files were cleared, and no matching guarded
mapping/SLAM/laser/base processes remained.

Status: P4-W-Speed-A passed. `zero_hold_s=3.0` is acceptable for the current
bounded branch-mixed policy run. The next speed test can try `2.5s`, but that
requires lowering the script validation floor from 3.0 to 2.0 and should be
committed as a separate change.

## P4-W-Speed-B: one-second zero-hold probe

Goal: check whether the guarded policy runner can safely shorten the per-action
settle period further, without changing thresholds, action preferences, guard
routing or the verified motion primitives.

Code change:

```text
tools/guarded_auto_mapping_micro.py:
  --zero-hold-s validation floor changed from 3.0s to 1.0s

tools/p4w_guarded_policy_branch_mixed.sh:
  usage text updated to allow ZERO_HOLD_S=1..8
```

Configuration:

```text
ZERO_HOLD_S: 3.0 -> 1.0
policy profile: interaction_mode
policy thresholds: unchanged
policy action preferences: unchanged
max_steps: 3
```

Dry-run:

```text
report: logs/policy_p4w_precheck_branch_mixed_20260629_150737.json
sample_count: 5
action_counts: ARC30_OR_FORWARD_0P10 = 5
front_p10: 0.767 -> 0.764 m
front_min: 0.755 -> 0.757 m
executed: false
```

Bounded run:

```text
launch_log: logs/policy_p4w_speed_b_20260629_150637.launch.log
report: logs/policy_p4w_run_branch_mixed_20260629_150824.json
sequence_stop_reason: target_overshot
step_count: 1
executed_count: 1
base_zero_ok: true
saved_maps: 1/1
total_forward_delta: 0.1033 m
total_lateral_delta: -0.0422 m
total_yaw_delta: -38.20 deg
elapsed: 17.20 s
```

Step summary:

```text
step 1:
  selected_action: ARC30_OR_FORWARD_0P10
  execution_action: ARC30_RIGHT
  front_p10: 0.762 -> 1.560 m
  odom_forward: 0.1033 m
  odom_lateral: -0.0422 m
  yaw_delta: -38.20 deg
  arc_steps: 3
  arc_stop_reason: target_overshot
  base_zero_ok: true
  map_saved: true

arc substeps:
  step 1: cumulative_yaw=-8.20 deg,  forward=0.0168 m, lateral=-0.0011 m, front_p10_after=0.752 m
  step 2: cumulative_yaw=-23.88 deg, forward=0.0566 m, lateral=-0.0134 m, front_p10_after=0.780 m
  step 3: cumulative_yaw=-38.20 deg, forward=0.1033 m, lateral=-0.0422 m, front_p10_after=1.541 m
```

Map size:

```text
maps/policy_p4w_branch_mixed_20260629_150824_01_policy: 223 x 112
```

Artifacts:

```text
logs/policy_p4w_speed_b_20260629_150637.launch.log
logs/policy_p4w_precheck_branch_mixed_20260629_150737.json
logs/policy_p4w_precheck_branch_mixed_20260629_150737.run.log
logs/policy_p4w_run_branch_mixed_20260629_150824.json
logs/policy_p4w_run_branch_mixed_20260629_150824.run.log
maps/policy_p4w_branch_mixed_20260629_150824_01_policy.{pgm,yaml,png}
maps/policy_p4w_branch_mixed_20260629_150824_01_policy_marked.png
```

Remote cleanup: launch PID 3743 and child nodes 3757, 3758, 3760 and 3810 were
stopped, FastDDS shared-memory files were cleared, and no matching guarded
mapping/SLAM/laser/base processes remained.

Status: P4-W-Speed-B is mechanically safe but behaviorally too aggressive for
the current arc policy default. `zero_hold_s=1.0` still produced
`base_zero_ok=true` and a verified map save, so the stop/zero chain did not
fail. However, the arc yaw accumulated past the target band to `-38.20 deg`,
causing `target_overshot` after one policy step. Keep `3.0s` as the practical
default for `arc-yaw-closed` policy runs, and treat `1.0s` as a probe or as a
future action-specific setting after separate forward-only and arc-only tests.

## P4-W-Speed-C: 1.5-second zero-hold probe

Goal: test an intermediate settle period after the 1s probe showed clean
mechanical zeroing but arc target overshoot.

Configuration:

```text
ZERO_HOLD_S: 1.5
policy profile: interaction_mode
policy thresholds: unchanged
policy action preferences: unchanged
max_steps: 3
```

Dry-run:

```text
report: logs/policy_p4w_precheck_branch_mixed_20260629_151912.json
sample_count: 5
action_counts: FORWARD_0P15_OR_ARC30 = 5
front_p10: 1.035 -> 1.038 m
front_min: 0.995 -> 0.995 m
executed: false
```

Bounded run:

```text
launch_log: logs/policy_p4w_speed_c_20260629_151834.launch.log
report: logs/policy_p4w_run_branch_mixed_20260629_152000.json
sequence_stop_reason: max_steps_reached
step_count: 3
executed_count: 3
final_base_zero_ok: true
saved_maps: 2/2 attempted saves
total_forward_delta: 0.3363 m
total_lateral_delta: -0.0303 m
total_yaw_delta: -31.93 deg
```

Step summary:

```text
step 1:
  execution_action: FORWARD_0P15
  front_p10: 1.038 -> 0.847 m
  odom_forward: 0.2001 m
  yaw_delta: -4.98 deg
  stop_reason: brake_margin_dynamic
  base_zero_ok: true
  map_saved: true

step 2:
  execution_action: FORWARD_0P15
  front_p10: 0.844 -> 0.749 m
  odom_forward: 0.0885 m
  yaw_delta: -1.72 deg
  stop_reason: front_blocked: front_p10 0.786 < 0.80
  base_zero_ok: false
  map_saved: false
  sequence_continue_after_front_block: true

step 3:
  execution_action: ARC30_RIGHT
  front_p10: 0.751 -> 0.784 m
  odom_forward: 0.0495 m
  yaw_delta: -25.22 deg
  arc_steps: 2
  arc_stop_reason: target_band_reached
  base_zero_ok: true
  map_saved: true
```

Observed issue:

The 1.5s run exposed a policy-run fail-safe bug. Step 2 had
`base_zero_ok=false` after a front-blocked forward segment, but the run still
continued because the `front_blocked` continuation path did not require
post-action base-zero success. This was not acceptable for bounded autonomous
policy execution.

Patch applied after this run:

```text
tools/guarded_auto_mapping_micro.py:
  if an action record exists and base_zero_ok is false:
    append or set stop_reason=base_zero_failed
    force continue_after_front_block=false
```

Map sizes:

```text
maps/policy_p4w_branch_mixed_20260629_152000_01_policy: 218 x 182
maps/policy_p4w_branch_mixed_20260629_152000_03_policy: 221 x 182
```

Artifacts:

```text
logs/policy_p4w_speed_c_20260629_151834.launch.log
logs/policy_p4w_precheck_branch_mixed_20260629_151912.json
logs/policy_p4w_precheck_branch_mixed_20260629_151912.run.log
logs/policy_p4w_run_branch_mixed_20260629_152000.json
logs/policy_p4w_run_branch_mixed_20260629_152000.run.log
maps/policy_p4w_branch_mixed_20260629_152000_01_policy.{pgm,yaml,png}
maps/policy_p4w_branch_mixed_20260629_152000_01_policy_marked.png
maps/policy_p4w_branch_mixed_20260629_152000_03_policy.{pgm,yaml,png}
maps/policy_p4w_branch_mixed_20260629_152000_03_policy_marked.png
```

Status: P4-W-Speed-C is useful evidence but should not be counted as a clean
policy-run pass because it was executed before the base-zero fail-safe patch.
It suggests that 1.5s can be enough for some arc steps, but it is not safe as a
global policy default.

## P4-W-Speed-D: two-second zero-hold probe after fail-safe patch

Goal: test a middle ground after patching the policy-run continuation logic.

Configuration:

```text
ZERO_HOLD_S: 2.0
policy profile: interaction_mode
policy thresholds: unchanged
policy action preferences: unchanged
max_steps: 3
```

Dry-run:

```text
report: logs/policy_p4w_precheck_branch_mixed_20260629_152650.json
sample_count: 5
action_counts: FORWARD_0P15_OR_ARC30 = 5
front_p10: 0.855 -> 0.863 m
front_min: 0.823 -> 0.838 m
executed: false
```

Bounded run:

```text
launch_log: logs/policy_p4w_speed_d_20260629_152850.launch.log
report: logs/policy_p4w_run_branch_mixed_20260629_152737.json
sequence_stop_reason: target_overshot
step_count: 2
executed_count: 2
base_zero_ok: true
saved_maps: 2/2
total_forward_delta: 0.2175 m
total_lateral_delta: -0.0408 m
total_yaw_delta: -41.62 deg
```

Step summary:

```text
step 1:
  execution_action: FORWARD_0P15
  front_p10: 0.866 -> 0.738 m
  odom_forward: 0.1287 m
  yaw_delta: -2.39 deg
  stop_reason: front_blocked: front_p10 0.797 < 0.80
  sequence_continue_after_front_block: true
  base_zero_ok: true
  map_saved: true

step 2:
  execution_action: ARC30_RIGHT
  front_p10: 0.735 -> 1.579 m
  odom_forward: 0.0905 m
  yaw_delta: -39.23 deg
  arc_steps: 3
  arc_stop_reason: target_overshot
  base_zero_ok: true
  map_saved: true
```

Map sizes:

```text
maps/policy_p4w_branch_mixed_20260629_152737_01_policy: 109 x 115
maps/policy_p4w_branch_mixed_20260629_152737_02_policy: 221 x 116
```

Artifacts:

```text
logs/policy_p4w_speed_d_20260629_152850.launch.log
logs/policy_p4w_precheck_branch_mixed_20260629_152650.json
logs/policy_p4w_precheck_branch_mixed_20260629_152650.run.log
logs/policy_p4w_run_branch_mixed_20260629_152737.json
logs/policy_p4w_run_branch_mixed_20260629_152737.run.log
maps/policy_p4w_branch_mixed_20260629_152737_01_policy.{pgm,yaml,png}
maps/policy_p4w_branch_mixed_20260629_152737_01_policy_marked.png
maps/policy_p4w_branch_mixed_20260629_152737_02_policy.{pgm,yaml,png}
maps/policy_p4w_branch_mixed_20260629_152737_02_policy_marked.png
```

Remote cleanup: launch PID 5291 and child nodes 5305, 5306, 5308 and 5356 were
stopped, FastDDS shared-memory files were cleared, and no matching guarded
mapping/SLAM/laser/base processes remained.

Status: P4-W-Speed-D validates the patched fail-safe behavior and shows that
`zero_hold_s=2.0` is sufficient for base-zero and verified map saves in this
run. It still produced an arc target overshoot (`-39.23 deg`), so it should not
replace `3.0s` as the global policy default for mixed forward/arc runs. The
better next optimization is action-specific settling: keep arc steps at about
`3.0s`, and test forward-only segments at `2.0s`.

## P4-W-Speed-RootCause implementation

Goal: stop tuning `zero_hold_s` blindly and expose where each policy step spends
time.

Implemented changes:

```text
tools/guarded_auto_mapping_micro.py:
  zero_hold() is now event-driven:
    publish zero for --zero-min-hold-s
    poll base zero every --zero-poll-s
    require --zero-confirm-samples consecutive base_zero_ok samples
    stop early when confirmed or time out at --zero-hold-s

  base_zero_status(wait_s=0.0) supports immediate checks so policy loops do not
  add an unconditional 0.8s delay after an event-driven zero wait.

  action records now include timing_breakdown:
    state_wait_time_s
    decision_time_s
    motion_execution_time_s
    stop_kick_time_s
    base_zero_wait_time_s
    adaptive_zero_extra_wait_s
    map_save_time_s
    postcheck_time_s
    loop_overhead_time_s
    total_time_s / step_total_time_s

  guarded-policy-step and guarded-policy-run now include step-level
  timing_breakdown, with the nested action timing preserved under
  action_timing_breakdown.
```

New parameters:

```text
--zero-hold-s              max event-driven zero wait, still accepted as before
--zero-min-hold-s          minimum zero publish window before polling
--zero-poll-s              base-zero poll interval
--zero-confirm-samples     consecutive base_zero_ok samples required
```

`tools/p4w_guarded_policy_branch_mixed.sh` now forwards:

```text
ZERO_HOLD_S
ZERO_MIN_HOLD_S
ZERO_POLL_S
ZERO_CONFIRM_SAMPLES
```

Expected next validation:

```text
P4-W-Speed-RootCause dry-run:
  confirm records contain timing_breakdown without publishing /input_cmd_vel

P4-W-Speed-RootCause run:
  same branch-mixed max_steps=3
  ZERO_HOLD_S=3.0
  ZERO_MIN_HOLD_S=0.8
  ZERO_POLL_S=0.1
  ZERO_CONFIRM_SAMPLES=3
```

This implementation has not yet been motion-tested. The next vehicle run should
compare per-step `base_zero_wait_time_s`, `motion_execution_time_s`,
`map_save_time_s` and `step_total_time_s` against P4-W-Speed-A/D before changing
arc or forward behavior further.

## P4-W-Speed-RootCause validation

Date: 2026-06-29

Launch:

```text
logs/p4w_speed_rootcause_20260629_161237.launch.log

hard_stop_m:=0.30
emergency_stop_m:=0.20
slow_down_m:=0.80
approach_stop_m:=0.80
min_effective_forward:=0.08
```

Dry-run:

```text
logs/policy_p4w_precheck_branch_mixed_20260629_161158.json
```

Result:

```text
sample_count: 8
action_counts: FORWARD_0P15_OR_ARC30 x 8
would_execute_action: FORWARD_0P15
base_zero_ok: true for all samples
published_input_cmd_vel: false
sample step_total_time_s: about 0.40s
```

Bounded run:

```text
logs/policy_p4w_run_branch_mixed_20260629_161237.json
logs/policy_p4w_run_branch_mixed_20260629_161237.run.log
```

Summary:

```text
step_count: 2
executed_count: 2
sequence_stop_reason: target_overshot
base_zero_ok: true
final odom delta: x=0.2813m, y=-0.0395m, yaw=-44.27deg
```

Map outputs:

```text
maps/policy_p4w_branch_mixed_20260629_161237_01_policy.{pgm,yaml,png}
maps/policy_p4w_branch_mixed_20260629_161237_01_policy_marked.png
maps/policy_p4w_branch_mixed_20260629_161237_02_policy.{pgm,yaml,png}
maps/policy_p4w_branch_mixed_20260629_161237_02_policy_marked.png
```

Map sizes:

```text
01_policy: 120 x 109
02_policy: 225 x 110
```

Timing breakdown:

| Step | Action | Stop reason | Motion wall | Motion exec | Base zero wait | Map save | Step total |
| --- | --- | --- | ---: | ---: | ---: | ---: | ---: |
| 1 | `FORWARD_0P15` | none | 5.384s | 2.478s | 2.093s | 5.273s | 11.482s |
| 2 | `ARC30_RIGHT` | `target_overshot` | 11.627s | 3.029s | 6.120s | 4.996s | 17.446s |

Arc sub-step timing:

| Arc step | Yaw after settle | Cumulative yaw | Base zero wait | Result |
| --- | ---: | ---: | ---: | --- |
| 1 | -6.50deg | -6.50deg | 2.089s | below target |
| 2 | -14.89deg | -21.39deg | 2.072s | below target |
| 3 | -20.01deg | -41.40deg | 1.959s | overshot |

Findings:

- Event-driven zero wait worked: every action reached `base_zero_ok=true`, with
  three consecutive confirmations and no zero-wait timeout.
- Fixed `zero_hold_s=3.0` is now only a maximum. In this run, forward and arc
  sub-steps released after about `1.96-2.09s`.
- The largest measured blockers are map saving and arc internals:
  - each synchronous `slam_toolbox/save_map` call cost about `5s`
  - `ARC30_RIGHT` spent `6.12s` just waiting for base zero across three
    internal arc steps
- The arc policy remains behaviorally conservative but slow. It also overshot
  the 30deg target on the third sub-step, so this run is a safe RootCause run,
  not a clean 3-step branch-mixed navigation pass.
- The console run log is very large because each step prints full sample arrays.
  The shell wall time is therefore higher than the reported step timing. A
  future speed pass should print compact step summaries and keep full samples in
  the JSON report only.

Next optimization order:

1. Add `save-policy` so normal demos save on hold/end or every 2 steps, while
   debug mode can keep every-step synchronous saves.
2. Add compact console output for policy runs.
3. Add `arc30_fast` or one-shot arc for exploration, while keeping current
   `arc-yaw-closed` for precise experiments.
4. Keep event-driven zero wait as the default stop validation mechanism.

## P4-Speed-F implementation

Goal: remove synchronous map saving and full JSON console printing from the
normal policy loop without weakening evidence capture.

Implemented changes:

```text
tools/guarded_auto_mapping_micro.py:
  --save-policy every_step
  --save-policy every_n_steps
  --save-policy critical_or_end
  --save-policy pipelined_critical
  --save-every-n N
  --max-pending-saves 1
  --console-mode full|compact
```

Save policy semantics:

```text
every_step:
  old behavior; every policy step synchronously calls save_map.

every_n_steps:
  normal movement steps start an async save only every N steps.

critical_or_end:
  normal movement steps only write JSON checkpoints; critical events and run end
  synchronously save maps.

pipelined_critical:
  normal movement steps start at most one async save. If a save is already
  pending, the step records save_skipped_pending and does not block. Critical
  events wait for any pending save, then synchronously save the current map.
```

Critical events are:

```text
HARD_STOP
HOLD_AND_CAPTURE
HOLD_SAVE_OBSERVE
HOLD_AND_SAVE
any hold/capture placeholder path
```

Run-end behavior:

```text
base_zero_ok=true:
  wait for pending save
  synchronously save final map as *_final

base_zero_ok=false:
  skip final save and record final_map_save.status=skipped_base_not_zero
```

Each policy step now also writes a lightweight checkpoint file:

```text
<report>.checkpoints.jsonl
```

The full report JSON is still preserved. `--console-mode compact` only changes
terminal output:

```text
STEP 1 FORWARD_0P15 executed=true stop=none fwd=0.20m yaw=-2.8deg zero=ok map=pending elapsed=...
RUN stop=target_overshot steps=2 executed=2 zero=ok final_map=saved ...
RESULT_SUMMARY ... report=/home/soc/edge-ai-robot-k1/logs/...
```

The P4-W helper script now forwards:

```text
SAVE_POLICY
SAVE_EVERY_N
MAX_PENDING_SAVES
CONSOLE_MODE
```

Recommended P4-Speed-F validation command:

```bash
ZERO_HOLD_S=3.0 \
ZERO_MIN_HOLD_S=0.8 \
ZERO_POLL_S=0.1 \
ZERO_CONFIRM_SAMPLES=3 \
POLICY_MAX_STEPS=3 \
SAVE_POLICY=pipelined_critical \
CONSOLE_MODE=compact \
tools/p4w_guarded_policy_branch_mixed.sh run
```

This implementation has only been statically checked so far. It was synced to
K1 and passed:

```text
python3 -m py_compile tools/guarded_auto_mapping_micro.py
bash -n tools/p4w_guarded_policy_branch_mixed.sh
python3 tools/guarded_auto_mapping_micro.py --help
```

Next run should compare `map_save_time_s`, `step_total_time_s`,
`final_map_save.elapsed_with_pending_wait_s`, checkpoint creation, and terminal
log length against P4-W-Speed-RootCause.

## P4-Speed-F validation

Validation date: 2026-06-29.

Guarded stack:

```bash
ros2 launch turn_on_wheeltec_robot n10p_tank_mapping_safety_guard.launch.py \
  hard_stop_m:=0.30 \
  emergency_stop_m:=0.20 \
  slow_down_m:=0.80 \
  approach_stop_m:=0.80 \
  min_effective_forward:=0.08 \
  clear_max_linear:=0.30 \
  soft_max_linear:=0.30
```

Dry-run with compact console passed first:

```text
report: /home/soc/edge-ai-robot-k1/logs/policy_p4w_precheck_branch_mixed_20260629_165106.json
samples: 5/5
selected_action: FORWARD_0P15_OR_ARC30
execution_action: FORWARD_0P15
front_p10: about 0.95 m
base_zero_bad: 0
motion_executed: false
```

First motion attempt used the proposed `ZERO_HOLD_S=3.0`. It was intentionally
kept as a negative data point:

```text
report: /home/soc/edge-ai-robot-k1/logs/policy_p4w_run_branch_mixed_20260629_165154.json
sequence_stop_reason: base_zero_failed
step_count: 1
executed_count: 1
action: FORWARD_0P15
step_stop_reason: base_zero_failed
step_base_zero_ok: false
run_end_base_zero_ok: true
final_map_saved: true
```

Conclusion: `3.0s` is too short for this chassis/pose in at least one forward
case. The run did not show a save-policy failure; it showed the adaptive zero
wait still needs a longer max window for reliable stop validation.

Successful P4-Speed-F run:

```bash
ZERO_HOLD_S=4.0 \
ZERO_MIN_HOLD_S=0.8 \
ZERO_POLL_S=0.1 \
ZERO_CONFIRM_SAMPLES=3 \
POLICY_MAX_STEPS=3 \
POLICY_MAX_RUNTIME_S=120 \
SAVE_POLICY=pipelined_critical \
CONSOLE_MODE=compact \
bash tools/p4w_guarded_policy_branch_mixed.sh run
```

Result:

```text
report: /home/soc/edge-ai-robot-k1/logs/policy_p4w_run_branch_mixed_20260629_165413.json
run_log: /home/soc/edge-ai-robot-k1/logs/policy_p4w_run_branch_mixed_20260629_165413.run.log
sequence_stop_reason: max_steps_reached
step_count: 3
executed_count: 3
base_zero_ok: true
final_map_saved: true
final_map: /home/soc/edge-ai-robot-k1/maps/policy_p4w_branch_mixed_20260629_165413_final
final_map_save_elapsed_with_pending_wait_s: 8.337
odom_forward_delta_m: 0.1955
odom_lateral_delta_m: -0.0838
odom_delta_yaw_deg: -57.91
```

Step summary:

```text
step  action        stop_reason                         base_zero  odom_forward  odom_yaw
1     FORWARD_0P15  front_blocked: front_p10 0.799<0.80 true       0.0690 m      -0.91 deg
2     ARC30_RIGHT   none                                true       0.0491 m      -25.00 deg
3     ARC30_RIGHT   none                                true       0.1016 m      -32.00 deg
```

Timing summary:

```text
step  motion_wall_s  motion_exec_s  base_zero_wait_s  map_save_time_s  step_total_s
1     6.550          2.107          3.631             0.001            7.378
2     8.038          2.014          4.366             0.001            8.854
3     8.005          2.025          4.301             0.001            8.832
```

The important speed result is `map_save_time_s ~= 0.001s` in each motion step.
Map saving no longer blocks the policy loop on normal steps.

Saved map evidence:

```text
normal async save 01: ok, result_code=0, elapsed=4.017s, pgm/yaml verified
normal async save 02: ok, result_code=0, elapsed=4.243s, pgm/yaml verified
normal async save 03: ok, result_code=0, elapsed=4.752s, pgm/yaml verified
final sync save:    ok, result_code=0, pgm/yaml verified
```

The final map and marked visualization were pulled to the Windows repo:

```text
maps/policy_p4w_branch_mixed_20260629_165413_final.pgm
maps/policy_p4w_branch_mixed_20260629_165413_final.yaml
maps/policy_p4w_branch_mixed_20260629_165413_final.png
maps/policy_p4w_branch_mixed_20260629_165413_final_marked.png
```

Console mode note:

The 165413 run still printed full `ARC_YAW_CLOSED_STEP` JSON inside the nested
arc primitive. After the run, `arc-yaw-closed` step and direction output were
patched to respect `--console-mode compact` as `ARC_STEP` and `ARC_DIRECTION`
summaries. That final console-only patch passed local and K1 `py_compile`; it
was not motion-retested in this validation run.

P4-Speed-F conclusion:

```text
pipelined_critical save-policy: validated
critical/final sync map evidence: validated
normal-step async map evidence: validated
compact policy summary output: validated
arc nested compact output: patched after validation, static checked
recommended zero max window: keep 4.0s for now
```

Next optimization should not reduce zero wait blindly. The next high-value speed
work is `arc30_fast` as a separate primitive while keeping the validated
`arc-yaw-closed` path for evidence-grade tests.

## P4-Speed-G0 arc-fast calibration implementation

Goal: add a fast arc candidate for exploration speed tests without replacing
the validated precise arc primitive.

Important boundary:

```text
ARC30_PRECISE:
  current arc-yaw-closed multi-step odom-yaw primitive
  remains the evidence-grade behavior

ARC_FAST:
  new candidate primitive
  single pulse, one stop validation, no policy integration yet
```

New mode:

```bash
python3 tools/guarded_auto_mapping_micro.py \
  --mode arc-fast-calib \
  --arc-fast-direction both \
  --arc-fast-front-p10-min-m 0.40 \
  --zero-hold-s 4.0 \
  --zero-min-hold-s 0.8 \
  --zero-poll-s 0.1 \
  --zero-confirm-samples 3 \
  --console-mode compact \
  --report /home/soc/edge-ai-robot-k1/logs/arc_fast_calib_$(date +%Y%m%d_%H%M%S).json \
  --confirm YES
```

Default calibration matrix:

```text
linear=0.10, angular=0.50, duration=1.0s
linear=0.10, angular=0.55, duration=1.0s
linear=0.10, angular=0.60, duration=1.0s
linear=0.10, angular=0.55, duration=1.2s
```

Each case runs left and right once when `--arc-fast-direction both` is used.
All motion still publishes only to `/input_cmd_vel`, so the chain remains:

```text
/input_cmd_vel -> scan_safety_guard_node -> /cmd_vel_guarded -> wheeltec_tank_base_safe.py -> C30D
```

Each record includes:

```text
yaw_delta_after_settle_deg
abs_yaw_delta_deg
forward_drift_m
lateral_drift_m
front_p10_before / front_p10_after
front_improved
front_min_safe
front_not_danger
base_zero_ok
elapsed_s
candidate_ok
```

Candidate acceptance is intentionally recorded but does not stop the sweep:

```text
20 deg <= abs_yaw_delta_deg <= 35 deg
base_zero_ok=true
front_min_after >= 0.20m
front_p10_after >= 0.30m or front_p10 improved
```

The sweep stops early only on:

```text
front_blocked by arc-fast front gate
base_zero_failed
front_min_after < 0.20m
```

Status: implementation only. It passed local syntax checks and should be synced
to K1 before the first supervised P4-Speed-G0 run.

## P4-Speed-G0 arc-fast calibration result

Validation date: 2026-06-29.

Run:

```bash
python3 tools/guarded_auto_mapping_micro.py \
  --mode arc-fast-calib \
  --arc-fast-direction both \
  --arc-fast-front-p10-min-m 0.40 \
  --zero-hold-s 4.0 \
  --zero-min-hold-s 0.8 \
  --zero-poll-s 0.1 \
  --zero-confirm-samples 3 \
  --console-mode compact \
  --report /home/soc/edge-ai-robot-k1/logs/arc_fast_calib_20260629_172447.json \
  --confirm YES
```

Result:

```text
report: /home/soc/edge-ai-robot-k1/logs/arc_fast_calib_20260629_172447.json
run_log: /home/soc/edge-ai-robot-k1/logs/arc_fast_calib_20260629_172447.run.log
sequence_stop_reason: arc_fast_calib_04_lin0p10_wz0p55_1p2s_right: front_blocked: front_p10 0.350 < 0.40
records: 8
candidate_ok: 0
fastest_ok: none
base_zero_ok: true for all executed motion segments
```

Summary:

```text
case                    direction  yaw_delta  forward_drift  front_p10       elapsed
lin0p10_wz0p50_1p0s     left       +10.41deg  0.0363m        0.765 -> 0.718  3.885s
lin0p10_wz0p50_1p0s     right      -5.12deg   0.0555m        0.720 -> 0.667  4.004s
lin0p10_wz0p55_1p0s     left       +8.55deg   0.0529m        0.669 -> 0.605  4.003s
lin0p10_wz0p55_1p0s     right      -7.09deg   0.0554m        0.610 -> 0.553  3.906s
lin0p10_wz0p60_1p0s     left       +8.95deg   0.0492m        0.554 -> 0.480  3.776s
lin0p10_wz0p60_1p0s     right      -8.79deg   0.0537m        0.489 -> 0.433  3.914s
lin0p10_wz0p55_1p2s     left       +12.33deg  0.0673m        0.432 -> 0.354  4.236s
lin0p10_wz0p55_1p2s     right      blocked    0.0000m        0.350 -> 0.356  1.916s
```

Interpretation:

```text
The tested single-pulse arc_fast candidates are mechanically safe but too weak.
They produce only about 5-12deg per pulse, below the 20-35deg target band.
They are faster than precise arc substeps, but not enough to replace ARC30.
The full sweep also accumulates forward motion and reduced front_p10 from 0.765m
to 0.350m, which triggered the intended front gate.
```

Conclusion:

```text
Do not connect these candidates to policy.
Keep ARC30_PRECISE as the active policy arc.
Next G1 should test stronger or longer fast candidates in a more open pose, but
as a shorter matrix, not another full 8-segment close-range sweep.
```

Recommended G1 candidates:

```text
linear=0.10, angular=0.70, duration=1.2s
linear=0.10, angular=0.80, duration=1.2s
linear=0.12, angular=0.70, duration=1.2s
linear=0.12, angular=0.80, duration=1.0s
```

Run left/right as separate supervised passes if front_p10 starts below 0.80m.

## P4-Speed-G1 arc-fast profile implementation

After G0 showed the 0.50-0.60 rad/s fast arcs were too weak, the calibration
mode was extended with a selectable profile:

```bash
--arc-fast-profile g0   # original weak single-pulse matrix
--arc-fast-profile g1   # stronger single-pulse matrix, still calibration only
```

G1 matrix:

```text
linear=0.10, angular=0.70, duration=1.2s
linear=0.10, angular=0.80, duration=1.2s
linear=0.12, angular=0.70, duration=1.2s
linear=0.12, angular=0.80, duration=1.0s
```

Recommended first run:

```bash
python3 tools/guarded_auto_mapping_micro.py \
  --mode arc-fast-calib \
  --arc-fast-profile g1 \
  --arc-fast-direction left \
  --arc-fast-front-p10-min-m 0.40 \
  --zero-hold-s 4.0 \
  --zero-min-hold-s 0.8 \
  --zero-poll-s 0.1 \
  --zero-confirm-samples 3 \
  --console-mode compact \
  --report /home/soc/edge-ai-robot-k1/logs/arc_fast_g1_left_$(date +%Y%m%d_%H%M%S).json \
  --confirm YES
```

Do not run both directions in one pass unless the starting `front_p10` is clearly
open. The G0 full sweep consumed about 0.4m of front clearance.

## P4-Speed-G1 arc-fast profile result

Validation date: 2026-06-29.

Stack:

```text
n10p_tank_mapping_safety_guard.launch.py
hard_stop_m:=0.30
emergency_stop_m:=0.20
slow_down_m:=0.80
approach_stop_m:=0.80
min_effective_forward:=0.08
clear_max_linear:=0.30
soft_max_linear:=0.30
```

Starting condition before G1-left:

```text
front_min: about 1.31m
front_p10: about 1.32m
base diag: cmd=(0,0), serial=(0,0), feedback=(0,0)
```

G1-left:

```bash
python3 tools/guarded_auto_mapping_micro.py \
  --mode arc-fast-calib \
  --arc-fast-profile g1 \
  --arc-fast-direction left \
  --arc-fast-front-p10-min-m 0.40 \
  --zero-hold-s 4.0 \
  --zero-min-hold-s 0.8 \
  --zero-poll-s 0.1 \
  --zero-confirm-samples 3 \
  --console-mode compact \
  --report /home/soc/edge-ai-robot-k1/logs/arc_fast_g1_left_20260629_173441.json \
  --confirm YES
```

G1-right:

```bash
python3 tools/guarded_auto_mapping_micro.py \
  --mode arc-fast-calib \
  --arc-fast-profile g1 \
  --arc-fast-direction right \
  --arc-fast-front-p10-min-m 0.40 \
  --zero-hold-s 4.0 \
  --zero-min-hold-s 0.8 \
  --zero-poll-s 0.1 \
  --zero-confirm-samples 3 \
  --console-mode compact \
  --report /home/soc/edge-ai-robot-k1/logs/arc_fast_g1_right_20260629_173534.json \
  --confirm YES
```

G1-left result:

```text
sequence_stop_reason: none
records: 4
candidate_ok: 4
fastest_ok: lin0p12_wz0p80_1p0s:left
```

```text
case                    yaw_delta  forward_drift  lateral_drift  front_p10       elapsed
lin0p10_wz0p70_1p2s     +20.43deg  0.0408m        0.0069m        1.321 -> 1.377  3.997s
lin0p10_wz0p80_1p2s     +28.79deg  0.0562m        0.0137m        1.375 -> 2.130  4.118s
lin0p12_wz0p70_1p2s     +24.74deg  0.0769m        0.0193m        2.121 -> 1.027  4.097s
lin0p12_wz0p80_1p0s     +25.49deg  0.0500m        0.0107m        1.029 -> 0.905  3.899s
```

G1-right result:

```text
sequence_stop_reason: none
records: 4
candidate_ok: 3
fastest_ok: lin0p12_wz0p80_1p0s:right
```

```text
case                    yaw_delta  forward_drift  lateral_drift  front_p10       elapsed
lin0p10_wz0p70_1p2s     -19.85deg  0.0715m       -0.0121m        0.909 -> 0.844  4.002s
lin0p10_wz0p80_1p2s     -30.02deg  0.0583m       -0.0149m        0.831 -> 1.887  4.104s
lin0p12_wz0p70_1p2s     -25.02deg  0.0770m       -0.0194m        1.896 -> 1.184  3.998s
lin0p12_wz0p80_1p0s     -28.35deg  0.0553m       -0.0133m        1.180 -> 1.054  3.917s
```

Interpretation:

```text
G1 produces usable fast arc behavior.
The best symmetric candidate is:
  linear.x = 0.12
  angular.z = +/-0.80
  duration = 1.0s

Left:  +25.49deg, forward 0.0500m, elapsed 3.899s
Right: -28.35deg, forward 0.0553m, elapsed 3.917s
base_zero_ok=true in both directions
front_p10 stayed above the 0.40m arc-fast gate
```

Speed comparison:

```text
ARC30_PRECISE in P4-Speed-F:
  about 8.83s per policy arc step

ARC_FAST G1 best candidate:
  about 3.90s per arc

Expected gain:
  save about 4.9s per arc action
  two-arc branch-mixed run can drop by about 9-10s
```

Conclusion:

```text
ARC_FAST candidate selected:
  linear=0.12
  angular_abs=0.80
  duration=1.0s

Do not delete ARC30_PRECISE.
Next step is G2: add --policy-arc-mode precise|fast, where fast uses this
candidate and precise keeps the current arc-yaw-closed primitive.
```

## P4-Speed-G2 policy arc-fast implementation

Goal: make policy runs able to choose the G1 fast arc primitive without
replacing the evidence-grade precise arc.

Implemented controls:

```text
--policy-arc-mode precise|fast
--policy-max-consecutive-fast-arc 2
--policy-arc-fast-linear 0.12
--policy-arc-fast-angular 0.80
--policy-arc-fast-duration-s 1.0
```

Default remains:

```text
--policy-arc-mode precise
```

Policy behavior:

```text
policy-arc-mode=precise:
  selected arc actions execute motion="arc30"
  execution_action=ARC30_LEFT or ARC30_RIGHT
  uses current arc-yaw-closed primitive and target-band checks

policy-arc-mode=fast:
  selected arc actions execute motion="arc_fast"
  execution_action=ARC_FAST_LEFT or ARC_FAST_RIGHT
  uses one calibrated pulse:
    linear.x=0.12
    angular.z=+/-0.80
    duration=1.0s
  does not require target_band_reached
  still checks front gate, base_zero_ok and front_min_after >= 0.20m
```

The implementation keeps `arc_fast` as an independent policy motion type. It is
not hidden inside the old `arc30` structure.

Fast-arc run guard:

```text
max consecutive fast arcs: 2 by default
on max_consecutive_fast_arc_reached:
  execute hold
  do not publish more motion
  treat stop reason as critical
  save critical/final evidence
  stop the bounded run
```

Front-gate handling:

```text
turn_threshold_record now accepts a local front_gate_m override.
Precise arc and fast arc use local gates instead of mutating global args.
Forward still uses the existing staged-forward gate path.
```

The P4-W branch-mixed helper now forwards:

```text
POLICY_ARC_MODE
POLICY_MAX_CONSECUTIVE_FAST_ARC
POLICY_ARC_FAST_LINEAR
POLICY_ARC_FAST_ANGULAR
POLICY_ARC_FAST_DURATION_S
```

G2-A recommended step test:

```bash
python3 tools/guarded_auto_mapping_micro.py \
  --mode guarded-policy-step \
  --behavior-profile interaction_mode \
  --policy-arc-mode fast \
  --policy-mid-action arc30 \
  --policy-arc-direction auto \
  --save-policy pipelined_critical \
  --console-mode compact \
  --report /home/soc/edge-ai-robot-k1/logs/policy_g2a_step_arc_fast_$(date +%Y%m%d_%H%M%S).json \
  --confirm YES
```

Place the robot so `front_p10` is about `0.50-0.80m`; the selected action should
be `ARC30_OR_FORWARD_0P10` and the execution action should be `ARC_FAST_LEFT` or
`ARC_FAST_RIGHT`.

G2-B recommended branch-mixed comparison:

```bash
POLICY_ARC_MODE=fast \
POLICY_MID_ACTION=arc30 \
POLICY_CLOSE_ACTION=arc30 \
POLICY_NORMAL_ACTION=forward \
POLICY_MAX_STEPS=3 \
ZERO_HOLD_S=4.0 \
ZERO_MIN_HOLD_S=0.8 \
ZERO_POLL_S=0.1 \
ZERO_CONFIRM_SAMPLES=3 \
SAVE_POLICY=pipelined_critical \
CONSOLE_MODE=compact \
bash tools/p4w_guarded_policy_branch_mixed.sh run
```

Expected timing:

```text
P4-Speed-F precise arc:
  about 8.8s per arc action

G2 fast arc:
  about 3.9-4.5s per arc action
```

Status: G2-A step validated. It passed local and K1 static checks before the
live step test.

## P4-Speed-G2-A policy arc-fast step result

Validation date: 2026-06-29.

Starting condition:

```text
front_min: 0.952m
front_p10: 0.993m
guard state: clear
odom twist before step: linear.x=0.0, angular.z=0.0
```

Because the robot was in open space (`front_p10 >= 0.80m`), this step used
`--policy-normal-action arc30` to force the single policy action through the
arc branch. That means:

```text
selected_action: FORWARD_0P15_OR_ARC30
execution_action: ARC_FAST_LEFT
policy_arc_mode: fast
```

Run command:

```bash
python3 tools/guarded_auto_mapping_micro.py \
  --mode guarded-policy-step \
  --behavior-profile interaction_mode \
  --policy-arc-mode fast \
  --policy-normal-action arc30 \
  --policy-arc-direction auto \
  --save-policy pipelined_critical \
  --console-mode compact \
  --zero-hold-s 4.0 \
  --zero-min-hold-s 0.8 \
  --zero-poll-s 0.1 \
  --zero-confirm-samples 3 \
  --report /home/soc/edge-ai-robot-k1/logs/policy_g2a_step_arc_fast_20260629_180358.json \
  --confirm YES
```

Compact console result:

```text
STEP ARC_FAST_LEFT executed=true stop=none fwd=0.0527m yaw=21.05deg zero=ok map=saved final_map=saved elapsed=15.126s
RESULT_SUMMARY stop=none executed=True base_zero=True map_saved=True
```

Action record:

```text
kind: guarded_policy_arc_fast
arc_mode: fast
direction: left
front_gate_m: 0.80
front_p10_before: 0.988m
front_p10_after: 0.921m
front_min_after: 0.892m
front_min_safe: true
abs_yaw_delta_deg: 21.05
fast_expected_yaw_band_deg: 20.0-35.0
fast_yaw_in_expected_band: true
base_zero_ok: true
stop_reason: duration_elapsed
```

Timing:

```text
motion_execution_time_s: 1.011
stop_kick_time_s: 0.800
base_zero_wait_time_s: 2.074
motion_wall_time_s: 3.902
map_save_time_s in main step: 0.001
final_map_save_time_s: 9.334
step_total_time_s: 15.126
```

The motion-only comparison is the important G2-A metric:

```text
ARC30_PRECISE in P4-Speed-F: about 8.8s per arc action
ARC_FAST in G2-A:             3.902s motion wall time
Gain:                         about 4.9s saved per arc action
```

Save evidence:

```text
normal async map save: ok, result_code=0, pgm/yaml verified
final sync map save:  ok, result_code=0, pgm/yaml verified
```

Pulled local artifacts:

```text
logs/policy_g2a_step_arc_fast_20260629_180358.json
logs/policy_g2a_step_arc_fast_20260629_180358.run.log
logs/policy_g2a_step_arc_fast_20260629_180358.checkpoints.jsonl
logs/p4_speed_g2_arc_fast_active.launch.log

maps/guarded_auto_micro_20260629_180401_policy_step.{pgm,yaml,png}
maps/guarded_auto_micro_20260629_180401_policy_step_marked.png
maps/guarded_auto_micro_20260629_180401_final.{pgm,yaml,png}
maps/guarded_auto_micro_20260629_180401_final_marked.png
```

Conclusion:

```text
G2-A passed.
policy loop can execute arc_fast as a distinct motion type.
The selected fast pulse landed inside the expected 20-35deg yaw band.
Base zero confirmation passed after the motion.
Front clearance stayed safely above the 0.20m hard floor.
Map save and final save both produced verified pgm/yaml files.
```

## P4-Speed-G2-B fast branch-mixed result

Validation date: 2026-06-29.

Goal: compare a 3-step branch-mixed run using `policy_arc_mode=fast` against
the P4-Speed-F precise-arc baseline.

Starting condition:

```text
front_min: 0.910m
front_p10: 0.931m
odom twist: linear.x=0.0, angular.z=0.0
map_metadata: readable, 136x111 at 0.05m resolution
```

Run command:

```bash
POLICY_ARC_MODE=fast \
POLICY_MID_ACTION=arc30 \
POLICY_CLOSE_ACTION=arc30 \
POLICY_NORMAL_ACTION=forward \
POLICY_MAX_STEPS=3 \
POLICY_MAX_RUNTIME_S=120 \
ZERO_HOLD_S=4.0 \
ZERO_MIN_HOLD_S=0.8 \
ZERO_POLL_S=0.1 \
ZERO_CONFIRM_SAMPLES=3 \
SAVE_POLICY=pipelined_critical \
SAVE_EVERY_N=2 \
MAX_PENDING_SAVES=1 \
CONSOLE_MODE=compact \
bash tools/p4w_guarded_policy_branch_mixed.sh run
```

Compact console result:

```text
STEP 1 FORWARD_0P15 executed=true stop=front_blocked: front_p10 0.795 < 0.80 fwd=0.1856m yaw=0.25deg zero=ok map=pending elapsed=7.512s
STEP 2 ARC_FAST_LEFT executed=true stop=none fwd=0.0318m yaw=16.01deg zero=ok map=pending elapsed=5.385s
STEP 3 ARC_FAST_LEFT executed=true stop=none fwd=0.0555m yaw=28.51deg zero=ok map=skipped_pending elapsed=4.749s
RUN stop=max_steps_reached steps=3 executed=3 zero=ok final_map=saved fwd=0.2669m yaw=44.77deg
```

Per-step records:

```text
step  selected_action          execution_action  front_p10_start  result
1     FORWARD_0P15_OR_ARC30    FORWARD_0P15      0.870m           front_blocked at 0.795m, base_zero_ok=true
2     ARC30_OR_FORWARD_0P10    ARC_FAST_LEFT     0.682m           yaw=16.01deg, front_p10_after=0.658m, base_zero_ok=true
3     ARC30_OR_FORWARD_0P10    ARC_FAST_LEFT     0.657m           yaw=28.51deg, front_p10_after=1.215m, base_zero_ok=true
```

The second fast arc undershot the G1 20-35deg expected yaw band:

```text
step 2 fast_yaw_in_expected_band: false
step 3 fast_yaw_in_expected_band: true
```

This is acceptable for the current policy design because `ARC_FAST` is a
single-pulse exploration primitive, not a precise yaw controller. The safety
property comes from stop-and-reobserve after each step:

```text
front_min_after step 2: 0.656m
front_min_after step 3: 1.199m
postcheck front_p10:   1.210m
hard stop:             not triggered
base_zero_ok:          true
```

Timing comparison against P4-Speed-F:

```text
metric                                  P4-Speed-F precise   G2-B fast       delta
main loop elapsed at last step           25.081s             17.660s        -7.421s
final save wait                          8.337s              5.237s         -3.100s
main loop + final save evidence window   33.418s             22.897s        -10.521s

arc step 1 step_total                    8.854s              5.385s         -3.469s
arc step 2 step_total                    8.832s              4.749s         -4.083s
arc step 1 motion_wall                   8.038s              4.570s         -3.468s
arc step 2 motion_wall                   8.005s              3.940s         -4.065s
```

Map save behavior:

```text
01 normal async save: ok, result_code=0, pgm/yaml verified
02 normal async save: ok, result_code=0, pgm/yaml verified
03 normal async save: skipped_pending
final sync save:      ok, result_code=0, pgm/yaml verified
```

Pulled local artifacts:

```text
logs/policy_p4w_run_branch_mixed_20260629_181158.json
logs/policy_p4w_run_branch_mixed_20260629_181158.run.log
logs/policy_p4w_run_branch_mixed_20260629_181158.checkpoints.jsonl
logs/p4_speed_g2b_arc_fast.launch.log

maps/policy_p4w_branch_mixed_20260629_181158_01_policy.{pgm,yaml,png}
maps/policy_p4w_branch_mixed_20260629_181158_01_policy_marked.png
maps/policy_p4w_branch_mixed_20260629_181158_02_policy.{pgm,yaml,png}
maps/policy_p4w_branch_mixed_20260629_181158_02_policy_marked.png
maps/policy_p4w_branch_mixed_20260629_181158_final.{pgm,yaml,png}
maps/policy_p4w_branch_mixed_20260629_181158_final_marked.png
```

Conclusion:

```text
G2-B passed.
Fast arc reduced 3-step main-loop time by about 7.4s versus the precise-arc F run.
Final evidence window dropped by about 10.5s.
The bounded policy loop remained guarded, stopped after max_steps=3, and ended with base_zero_ok=true.
The result supports using ARC_FAST for demo/exploration mode while keeping ARC30_PRECISE for evidence-grade yaw tests.
```

## P4-Y bounded multi-step exploration guardrails

Before opening 5-step / 7-step runs, the policy runner was extended with two
guardrails:

```text
--policy-max-steps now allows 1..7
--policy-max-total-forward-m defaults to 1.0m
```

The total-forward guard accumulates each step's positive local forward motion.
It does not subtract reverse/side effects or use net displacement to hide
previous movement. When the limit is reached, the current step is allowed to
finish its zero/base check and map evidence path, then the run stops before the
next step.

The branch-mixed helper forwards:

```text
POLICY_MAX_TOTAL_FORWARD_M
```

The existing `POLICY_MAX_CONSECUTIVE_FAST_ARC=2` guard remains active. A 5-step
run therefore has these limits:

```text
max_steps <= 5
max_runtime_s <= 120
cumulative_positive_forward_m <= 1.0m
max_consecutive_fast_arc <= 2
hard stop still uses the scan safety guard
all motion still publishes only to /input_cmd_vel
```

## P4-Y1 5-step fast bounded exploration result

Validation date: 2026-06-29.

Goal: verify multi-step guarded exploration beyond the 3-step G2-B run without
calling it full free exploration.

Starting condition:

```text
front_min: 0.872m
front_p10: 0.899m
odom twist: linear.x=0.0, angular.z=0.0
map_metadata: readable, 137x111 at 0.05m resolution
```

Run command:

```bash
POLICY_ARC_MODE=fast \
POLICY_MID_ACTION=arc30 \
POLICY_CLOSE_ACTION=arc30 \
POLICY_NORMAL_ACTION=forward \
POLICY_MAX_STEPS=5 \
POLICY_MAX_RUNTIME_S=120 \
POLICY_MAX_TOTAL_FORWARD_M=1.0 \
POLICY_MAX_CONSECUTIVE_FAST_ARC=2 \
ZERO_HOLD_S=4.0 \
ZERO_MIN_HOLD_S=0.8 \
ZERO_POLL_S=0.1 \
ZERO_CONFIRM_SAMPLES=3 \
SAVE_POLICY=pipelined_critical \
SAVE_EVERY_N=2 \
MAX_PENDING_SAVES=1 \
CONSOLE_MODE=compact \
bash tools/p4w_guarded_policy_branch_mixed.sh run
```

Compact console result:

```text
STEP 1 FORWARD_0P15 executed=true stop=none fwd=0.2046m yaw=3.95deg zero=ok map=pending elapsed=8.104s
STEP 2 ARC_FAST_LEFT executed=true stop=none fwd=0.0291m yaw=14.6deg zero=ok map=skipped_pending elapsed=4.654s
STEP 3 FORWARD_0P15 executed=true stop=none fwd=0.178m yaw=7.06deg zero=ok map=pending elapsed=8.054s
STEP 4 FORWARD_0P15 executed=true stop=none fwd=0.1678m yaw=1.19deg zero=ok map=pending elapsed=8.813s
STEP 5 FORWARD_0P15 executed=true stop=none fwd=0.172m yaw=1.24deg zero=ok map=pending elapsed=8.752s
RUN stop=max_steps_reached steps=5 executed=5 zero=ok final_map=saved fwd=0.6998m yaw=28.04deg
```

Per-step behavior:

```text
step  selected_action          execution_action  front_p10_start  front_p10_after  cumulative_positive_forward
1     FORWARD_0P15_OR_ARC30    FORWARD_0P15      0.903m           0.711m           0.2046m
2     ARC30_OR_FORWARD_0P10    ARC_FAST_LEFT     0.710m           1.158m           0.2337m
3     FORWARD_0P15_OR_ARC30    FORWARD_0P15      1.156m           2.064m           0.4117m
4     FORWARD_0P15_OR_ARC30    FORWARD_0P15      2.073m           1.887m           0.5795m
5     FORWARD_0P15_OR_ARC30    FORWARD_0P15      1.892m           1.708m           0.7515m
```

The important behavior transition:

```text
Step 1 forward moved the robot into the 0.60-0.80m zone.
Step 2 selected ARC_FAST_LEFT.
After the fast arc, front_p10 increased from 0.711m to 1.162m.
Step 3 automatically switched back to FORWARD_0P15.
```

Safety and limits:

```text
sequence_stop_reason: max_steps_reached
step_count: 5
executed_count: 5
base_zero_ok: true
final_map_saved: true
hard stop: not triggered
postcheck front_min: 1.705m
postcheck front_p10: 1.710m
cumulative_positive_forward_m: 0.7515m
policy_max_total_forward_m: 1.0m
max_consecutive_fast_arc: 2, not exceeded
```

The step 2 fast arc undershot the G1 20-35deg expected yaw band:

```text
step 2 yaw: 14.6deg
fast_yaw_in_expected_band: false
```

This did not break the policy behavior because the post-step reobserve opened
the front sector and caused the next decision to switch back to forward.

Timing:

```text
main-loop elapsed at step 5: 38.406s
final save wait:            10.118s
evidence window total:      about 48.524s
```

Map save behavior:

```text
01 normal async save: ok, result_code=0, pgm/yaml verified
02 normal async save: skipped_pending
03 normal async save: ok, result_code=0, pgm/yaml verified
04 normal async save: ok, result_code=0, pgm/yaml verified
05 normal async save: ok, result_code=0, pgm/yaml verified
final sync save:      ok, result_code=0, pgm/yaml verified
```

Pulled local artifacts:

```text
logs/policy_p4w_run_branch_mixed_20260629_182847.json
logs/policy_p4w_run_branch_mixed_20260629_182847.run.log
logs/policy_p4w_run_branch_mixed_20260629_182847.checkpoints.jsonl
logs/p4_y1_5step_fast.launch.log

maps/policy_p4w_branch_mixed_20260629_182847_01_policy.{pgm,yaml,png}
maps/policy_p4w_branch_mixed_20260629_182847_01_policy_marked.png
maps/policy_p4w_branch_mixed_20260629_182847_03_policy.{pgm,yaml,png}
maps/policy_p4w_branch_mixed_20260629_182847_03_policy_marked.png
maps/policy_p4w_branch_mixed_20260629_182847_04_policy.{pgm,yaml,png}
maps/policy_p4w_branch_mixed_20260629_182847_04_policy_marked.png
maps/policy_p4w_branch_mixed_20260629_182847_05_policy.{pgm,yaml,png}
maps/policy_p4w_branch_mixed_20260629_182847_05_policy_marked.png
maps/policy_p4w_branch_mixed_20260629_182847_final.{pgm,yaml,png}
maps/policy_p4w_branch_mixed_20260629_182847_final_marked.png
```

Conclusion:

```text
P4-Y1 passed.
The system completed a 5-step guarded multi-step exploration run.
It switched from forward to ARC_FAST when front_p10 dropped into the mid zone,
then switched back to forward after the fast arc opened the front sector.
The run stayed under the 1.0m cumulative positive forward limit and ended with
base_zero_ok=true and final_map_saved=true.
```

## P4-Y2 7-step bounded stress test result

Validation date: 2026-06-29.

Goal: run a 7-step bounded stress test with the same 1.0m total-forward limit
and the existing `max_consecutive_fast_arc=2` guard. This was not intended as a
full free-exploration run.

Starting condition:

```text
front_min: 0.614m
front_p10: 0.623m
front_state: warning
front_action: pass
odom twist: linear.x=0.0, angular.z=0.0
map_metadata: readable, 101x115 at 0.05m resolution
```

Because the robot started in the `0.60-0.80m` mid zone, the first policy action
was expected to be an arc rather than forward.

Run command:

```bash
POLICY_ARC_MODE=fast \
POLICY_MID_ACTION=arc30 \
POLICY_CLOSE_ACTION=arc30 \
POLICY_NORMAL_ACTION=forward \
POLICY_MAX_STEPS=7 \
POLICY_MAX_RUNTIME_S=180 \
POLICY_MAX_TOTAL_FORWARD_M=1.0 \
POLICY_MAX_CONSECUTIVE_FAST_ARC=2 \
ZERO_HOLD_S=4.0 \
ZERO_MIN_HOLD_S=0.8 \
ZERO_POLL_S=0.1 \
ZERO_CONFIRM_SAMPLES=3 \
SAVE_POLICY=pipelined_critical \
SAVE_EVERY_N=2 \
MAX_PENDING_SAVES=1 \
CONSOLE_MODE=compact \
bash tools/p4w_guarded_policy_branch_mixed.sh run
```

Compact console result:

```text
STEP 1 ARC_FAST_RIGHT executed=true stop=none fwd=0.042m yaw=-21.26deg zero=ok map=pending elapsed=4.963s
STEP 2 ARC_FAST_RIGHT executed=true stop=none fwd=0.0822m yaw=-27.07deg zero=ok map=skipped_pending elapsed=5.197s
STEP 3 HOLD_MAX_FAST_ARC executed=false stop=max_consecutive_fast_arc_reached fwd=0.0m yaw=0.0deg zero=ok map=saved elapsed=8.204s
RUN stop=max_consecutive_fast_arc_reached steps=3 executed=2 zero=ok final_map=saved fwd=0.1113m yaw=-48.33deg
```

Per-step behavior:

```text
step  selected_action        execution_action   front_p10_start  front_p10_after  result
1     ARC30_OR_FORWARD_0P10  ARC_FAST_RIGHT     0.620m           0.708m           yaw=-21.26deg, base_zero_ok=true
2     ARC30_OR_FORWARD_0P10  ARC_FAST_RIGHT     0.705m           0.662m           yaw=-27.07deg, base_zero_ok=true
3     ARC30_OR_FORWARD_0P10  HOLD_MAX_FAST_ARC  0.675m           0.670m           no motion, critical save
```

Both fast arcs landed inside the expected 20-35deg yaw band:

```text
step 1 fast_yaw_in_expected_band: true
step 2 fast_yaw_in_expected_band: true
```

However, the second arc did not improve front clearance:

```text
step 1 front_p10: 0.621m -> 1.035m in action record, postcheck p10=0.708m
step 2 front_p10: 0.701m -> 0.667m in action record, postcheck p10=0.662m
```

The `max_consecutive_fast_arc=2` guard therefore stopped the run before a third
fast arc. This is the expected safety behavior for Y2.

Safety and limits:

```text
sequence_stop_reason: max_consecutive_fast_arc_reached
step_count: 3
executed_count: 2
base_zero_ok: true
final_map_saved: true
hard stop: not triggered
postcheck front_min: 0.636m
postcheck front_p10: 0.664m
cumulative_positive_forward_m: 0.1242m
policy_max_total_forward_m: 1.0m
max_consecutive_fast_arc: 2, reached and enforced
```

Map save behavior:

```text
01 normal async save: ok, result_code=0, pgm/yaml verified
02 normal async save: skipped_pending
03 critical sync save: ok, result_code=0, pgm/yaml verified
final sync save:       ok, result_code=0, pgm/yaml verified
```

Pulled local artifacts:

```text
logs/policy_p4w_run_branch_mixed_20260629_183731.json
logs/policy_p4w_run_branch_mixed_20260629_183731.run.log
logs/policy_p4w_run_branch_mixed_20260629_183731.checkpoints.jsonl
logs/p4_y2_7step_fast.launch.log

maps/policy_p4w_branch_mixed_20260629_183731_01_policy.{pgm,yaml,png}
maps/policy_p4w_branch_mixed_20260629_183731_01_policy_marked.png
maps/policy_p4w_branch_mixed_20260629_183731_03_policy.{pgm,yaml,png}
maps/policy_p4w_branch_mixed_20260629_183731_03_policy_marked.png
maps/policy_p4w_branch_mixed_20260629_183731_final.{pgm,yaml,png}
maps/policy_p4w_branch_mixed_20260629_183731_final_marked.png
```

Conclusion:

```text
P4-Y2 passed as a bounded stress test.
It did not reach 7 executed motion steps because the consecutive-fast-arc guard
correctly stopped the run after two ARC_FAST actions failed to produce a stable
forward-clear transition.
The stop path produced a critical map save, final map save, and base_zero_ok=true.
This validates the pressure-test safety behavior rather than extending toward
unbounded exploration.
```
