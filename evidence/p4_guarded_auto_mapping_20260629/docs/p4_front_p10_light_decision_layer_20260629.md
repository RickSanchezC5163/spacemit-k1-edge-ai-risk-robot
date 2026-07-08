# P4 Light Decision Layer Draft

Date: `2026-06-29`

This is a design note only. It should not be treated as a request to run a new
robot test.

## Goal

Upgrade P4 from a fixed S-shaped sequence to a small deterministic controller
that chooses one guarded primitive at a time:

```text
forward-staged
arc-yaw-closed left
arc-yaw-closed right
stop/save
```

The controller remains conservative:

- no RRT
- no AMCL main localization
- no global exploration planner
- no long unattended run
- no direct publish to `/cmd_vel_guarded`

## Inputs

Required fresh inputs before every primitive:

```text
/scan
/odom
/map_metadata
/safety/front_obstacle
/robot_vel or base diagnostics
```

The key decision signal is `front_p10`, not `front_min`. `front_min` is still
logged as a hard warning signal, but `front_p10` is less sensitive to a single
bad scan point.

## Behavior Profiles

Do not use a single distance threshold set for every task. P4 now has two
different operating intents:

```text
mapping_safe_mode:
  conservative map expansion and motion primitive validation

interaction_mode:
  approach, observe, photograph, recognize, and prepare for future arm work
```

The old `front_p10 >= 1.20m` forward threshold belongs only to
`mapping_safe_mode`. It is too conservative for interaction tasks, where the
robot must be able to approach an object before camera recognition or arm
operation.

## Mapping Safe Thresholds

These thresholds are retained for wide-space guarded mapping tests:

```text
front_hard_stop_p10_m = 0.50
front_arc_min_p10_m   = 0.80
front_forward_p10_m   = 1.20
```

Interpretation:

| Condition | Decision |
| --- | --- |
| stale scan/odom/map/status | stop and mark `not_ready` |
| base not zero before action | stop and mark `base_not_zero` |
| `front_p10 < 0.50m` | stop, save map, mark `blocked` |
| `front_p10 < 0.80m` | stop, save map, mark `too_close_for_arc` |
| `front_p10 < 1.20m` | prefer `arc-yaw-closed` |
| `front_p10 >= 1.20m` | allow `forward-staged` |

## Interaction Thresholds

Use this profile for close-range perception and future arm tasks:

```text
front_hard_stop_min_m       = 0.20
front_hold_p10_m            = 0.30
front_observe_p10_m         = 0.40
front_short_action_p10_m    = 0.60
front_normal_action_p10_m   = 0.80
```

Interpretation:

| Condition | Decision |
| --- | --- |
| stale scan/odom/map/status | stop and mark `not_ready` |
| base not zero before action | stop and mark `base_not_zero` |
| `front_min < 0.20m` | `HARD_STOP` |
| `front_p10 < 0.30m` | `HOLD_AND_PERCEIVE`; no chassis motion |
| `0.30m <= front_p10 < 0.40m` | `HOLD_SAVE_OBSERVE`; save map, camera/recognition only |
| `0.40m <= front_p10 < 0.60m` | `ARC30_OR_VERY_SHORT_FORWARD` |
| `0.60m <= front_p10 < 0.80m` | `LOCAL_AVOIDANCE`; arc30 or forward `0.10m` |
| `front_p10 >= 0.80m` | `EXPLORE`; allow normal guarded primitive selection |

`front_min < 0.20m` is an emergency stop line, not the normal behavior
boundary. The normal stop/observe band starts earlier at `front_p10 < 0.30m`
or `front_p10 < 0.40m`.

The P4-R run used `front_p10_min_m=0.50` as a supervised hard gate. That value
should be treated as a field-test gate, not as the only decision boundary for
future interaction behavior.

## Primitive Selection

First version should be intentionally simple and profile-driven:

```text
CHECK_READY
  if not ready: STOP_SAVE

CHECK_PROFILE
  choose mapping_safe_mode or interaction_mode thresholds

CHECK_FRONT_MAPPING_SAFE
  if front_p10 < 0.50: STOP_SAVE
  if front_p10 < 0.80: STOP_SAVE
  if front_p10 < 1.20: ARC
  else: FORWARD_0P15

CHECK_FRONT_INTERACTION
  if front_min < 0.20: HARD_STOP
  if front_p10 < 0.30: HOLD_AND_PERCEIVE
  if front_p10 < 0.40: HOLD_SAVE_OBSERVE
  if front_p10 < 0.60: ARC_OR_FORWARD_0P05_TO_0P10
  if front_p10 < 0.80: ARC_OR_FORWARD_0P10
  else: NORMAL_PRIMITIVE

FORWARD
  run forward-staged with target chosen by profile and front_p10 band
  stop, settle, verify base_zero_ok
  save map

ARC
  choose left/right by policy
  run arc-yaw-closed target 24-30deg
  stop, settle, verify base_zero_ok
  save map

LOOP_LIMIT
  stop after max_steps or max_duration_s
```

## Interaction Action Limits

The validated `arc-yaw-closed` primitive is not a pure in-place rotation. In
P4-R and related tests, it can move forward about `0.05-0.15m` while changing
yaw. Treat it as a spatial arc maneuver.

Use these limits in `interaction_mode`:

| front_p10 band | Allowed chassis action |
| --- | --- |
| `<0.30m` | no chassis motion |
| `0.30-0.40m` | no chassis motion; observe only |
| `0.40-0.60m` | arc30 if needed, or forward `0.05-0.10m` only |
| `0.60-0.80m` | arc30 or forward `0.10m` |
| `>=0.80m` | forward `0.15m`, arc30, or short guarded combinations |

Do not run the full S-run just because `front_p10 >= 0.40m`. The `0.40-0.80m`
range is for short local actions only.

## Direction Policy

Do not add wall following or frontier search yet. The first direction policy can
be one of these:

1. Alternating: left, right, left, right.
2. Bias recovery: if the last turn was left, try right next.
3. Manual seed: user starts with `--prefer-arc left` or `right`.

The first implementation should use alternating plus a maximum step count.

## Initial Parameters

```text
behavior_profile = mapping_safe | interaction

max_decision_steps = 5
max_duration_s = 90
save_every_step = true
zero_hold_s = 5.0

mapping_safe:
  front_hard_stop_p10_m = 0.50
  front_arc_min_p10_m = 0.80
  front_forward_p10_m = 1.20

interaction:
  front_hard_stop_min_m = 0.20
  front_hold_p10_m = 0.30
  front_observe_p10_m = 0.40
  front_short_action_p10_m = 0.60
  front_normal_action_p10_m = 0.80

forward_target_m = 0.05, 0.10, or 0.15 by profile band
forward_fast/mid/slow = 0.15/0.12/0.10m/s
forward_brake = 0.03 + odom_vx*1.05 + 0.02

arc_yaw_target_deg = 30
arc_yaw_tolerance_deg = 6
arc_yaw_overshoot_epsilon_deg = 1.5
arc_step_linear = 0.10
arc_step_angular = 0.50
arc_step_duration_s = 1.0
arc_max_steps = 4
```

## Required Logging

Every decision cycle must record:

```text
decision_index
behavior_profile
chosen_action
threshold_band
front_p10_start/end
front_min_start/end
odom_start/end
actual_forward
actual_lateral
actual_yaw
base_zero_ok
stop_reason
map_width/map_height
map_file
```

## Acceptance Criteria

The first decision-layer test should pass only if:

- every primitive uses `/input_cmd_vel`
- every primitive ends with `base_zero_ok=true`
- `front_min` never falls below `0.20m` in `interaction_mode`
- `front_p10` never violates the active profile's hard stop gate
- maps save after every decision
- total runtime stays under `90s`
- final report shows no stale input and no unhandled stop reason

## First Interaction Test

The first near-object behavior-selector test should not include arm motion.
Limit it to:

```text
if front_p10 >= 0.40m:
  run one arc30 or one very-short forward action
  save map
  hold/perceive
else:
  hold/perceive only
```

This validates close-range behavior selection before adding camera recognition
or mechanical arm actions.

## Non-goals

This layer is not:

- official RRT exploration
- Nav2 full autonomy
- AMCL-based navigation
- frontier exploration
- a global path planner

It is a small guarded behavior selector on top of already validated P4
primitives.
