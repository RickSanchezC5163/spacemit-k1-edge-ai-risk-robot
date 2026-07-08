# P4 Guarded Auto Mapping Evidence

Date: `2026-06-29`

This package freezes the P4-R result as displayable evidence. It contains only
already captured data and does not require rerunning the robot.

## Claim

The K1 robot completed a short guarded auto-mapping sequence using:

```text
/input_cmd_vel
-> scan_safety_guard_node
-> /cmd_vel_guarded
-> wheeltec_tank_base_safe.py
-> C30D
```

The run did not use RRT, AMCL, or long autonomous exploration.

## Validated Sequence

```text
F0.15 -> L30_arc -> F0.10 -> R30_arc -> F0.10
```

Result:

```text
sequence_stop_reason=null
saved_maps=6
front_p10_min_m=0.50
final_front_p10=1.771m
sequence_forward=0.5978m
sequence_lateral=0.0943m
sequence_yaw=-4.52deg
map_size=128x222 -> 128x240
```

Every executed movement segment ended with `base_zero_ok=true`.

## Segment Summary

| Segment | Result | Odom result | Final zero |
| --- | --- | --- | --- |
| `forward-staged 0.15m` | completed | `forward=0.1901m`, yaw `+2.09deg` | true |
| `arc-yaw-closed left 30deg` | completed | `2 steps`, yaw `+24.30deg`, forward `0.0478m` | true |
| `forward-staged 0.10m` | completed | `forward=0.1278m`, yaw `+1.43deg` | true |
| `arc-yaw-closed right 30deg` | completed | `3 steps`, yaw `-28.35deg`, forward `0.1014m` | true |
| `forward-staged 0.10m` | completed | `forward=0.1449m`, yaw `-3.98deg` | true |

## Display Images

Use the zoomed final image for presentation:

```text
maps/spatial_s_p4r_20260629_001054_05_forward_0p10_after_right_marked_zoom.png
```

All stage maps are also included under `maps/`.

## Raw Evidence

```text
logs/spatial_s_p4r_20260629_001054.json
logs/spatial_s_p4r_20260629_001054.run.log
docs/guarded_auto_mapping_micro_20260628.md
```

## Interpretation

This proves the system has moved beyond fixed one-shot motion tests. It can now
chain odom-closed forward primitives and discrete arc-yaw primitives under the
scan safety guard, save maps between segments, and return the base to zero at
each stop.

This is still not full autonomous exploration. The next reasonable upgrade is a
small deterministic decision layer that chooses between `forward-staged`,
`arc-yaw-closed`, or stop based on `front_p10` and readiness checks.
