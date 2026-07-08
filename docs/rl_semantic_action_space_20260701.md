# RL Semantic Action Space - 2026-07-01

RL is allowed to output only high-level actions:

```text
0 HOLD
1 FORWARD_0P15
2 ARC_FAST_LEFT
3 ARC_FAST_RIGHT
4 HOLD_CAPTURE
5 ARM_NO_LOAD_RESPONSE
6 STOP_SAFE
```

Optional disabled actions:

```text
7 SAVE_MAP
8 ARM_CLEAR_CANDIDATE_DRYRUN
```

RL must not output:

- raw `cmd_vel`
- direct `/cmd_vel_guarded`
- chassis serial bytes
- arm servo pulse frames

Real-vehicle execution must consume `action_candidate.json` and pass through
existing P4/Step7 safety gates.

Observation fields are aligned to the real vehicle `scan_sector_snapshot()`
naming:

```text
front_min
front_p10
left_p10
right_p10
odom_x
odom_y
odom_yaw
map_progress
risk_detected
risk_confidence
risk_class_id
risk_distance_m
base_zero
arm_ready
capture_recent
steps_since_capture
consecutive_fast_arc
total_forward_m
```

`consecutive_fast_arc` and `total_forward_m` are included because the real P4/Y
guarded policy uses those bounds to stop risky repeated arcs or excessive
forward motion. Reward terms are encoded in `configs/rl_action_space.yaml`.

Current status: `rl/envs/semantic_guarded_nav_env.py` is a dependency-free
mock environment for action-space bring-up, not a hardware policy.
