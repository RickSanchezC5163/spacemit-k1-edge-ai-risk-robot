# Primitive Action Semantics - 2026-07-01

This document freezes the high-level primitive names used by Step7, Gazebo,
RL, and future hardware-gated runners.

## Chassis

- `HOLD`
- `FORWARD_0P15`
- `ARC_FAST_LEFT`
- `ARC_FAST_RIGHT`
- `SAVE_MAP`
- `STOP_SAFE`

Real chassis execution must remain:

```text
/input_cmd_vel -> scan_safety_guard_node -> /cmd_vel_guarded
```

RL must never output low-level `cmd_vel`, direct `/cmd_vel_guarded`, or chassis
serial bytes.

## D435

- `HOLD_CAPTURE`
- `D435_CAPTURE`

Both require `base_zero=true` and must not publish `cmd_vel`.

## Risk Vision

- `RISK_DETECT_HSV_RED_RULE`
- `RISK_DETECT_LOCAL_MODEL`
- `RISK_CLASSIFY_PRINTED_RISK`

HSV is a deterministic baseline. Competition AI claims require a local model
backend and K1 benchmark data.

## Risk Map

- `RISK_POINT_FROM_BBOX_DEPTH`
- `RISK_PROJECT_TO_MAP`
- `RISK_MAP_SUMMARY`
- `RISK_VISUALIZE_MAP`

Projection is approximate unless calibrated TF is validated.

## Arm

- `ARM_HOME_6B`
- `ARM_NO_LOAD_RESPONSE`
- `ARM_CLEAR_CANDIDATE_DRYRUN`
- `ARM_CLEAR_NO_LOAD_TRAJECTORY`
- `ARM_SOFT_CONTACT_TEST`
- `ARM_CONTROLLED_CLEAR`

All arm primitives require base-zero. Current validated hardware claim is
no-load only. Contact, grasping, payload handling, and obstacle removal remain
future gated stages.

## Report

- `REPORT_DETERMINISTIC`
- `REPORT_LOCAL_LLM`
- `REPORT_EXPORT_BUNDLE`

Deterministic reporting is not a local LLM claim.
