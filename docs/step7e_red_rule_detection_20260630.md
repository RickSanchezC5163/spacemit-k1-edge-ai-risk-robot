# Step7-E Red-Rule Detection Precheck

## Summary

This precheck validates that a red object placed in front of the D435 can be
detected by a deterministic color rule and paired with depth evidence.

Evidence directory:

```text
outputs/step7e_red_object_visual_check_v1/live_001/captures/red_object_live_001/
```

## Result

- red_object_detected: `true`
- detection_mode: `hsv_rule_based_red_color`
- model_used: `false`
- accuracy_claimed: `false`
- bbox_xywh: `[253, 159, 231, 199]`
- depth_median_m: `0.57`
- bbox_valid_depth_ratio: `0.878`
- camera_point_xyz_m: `{x: 0.032, y: 0.029, z: 0.57}`
- red_mask_ratio: `0.075758`
- component_count: `3`

## Evidence Files

- `rgb.png`
- `depth_raw.npy`
- `depth_vis.png`
- `camera_info.json`
- `odom.json`
- `capture_meta.json`
- `red_object_rule_detection.json`
- `red_object_mask.png`
- `red_object_overlay.png`
- `README_red_object_check.md`

## Interpretation

This result proves that the current D435 scene contains a visible red region
that can be extracted by a fixed HSV color rule and assigned an approximate
depth/camera-frame point from `depth_raw.npy`.

This is suitable as a Step7-E1 trigger source:

```text
risk_trigger_source = D435_red_color_rule
```

It replaces pure mock risk triggering for the next stationary validation step,
but it is still not a trained model and does not claim robust recognition
accuracy.

## Claim Boundary

Allowed:

- fixed-rule red color visibility check
- depth median extraction inside the detected red bbox
- approximate camera-frame point from D435 intrinsics
- use as a deterministic Step7-E1 trigger source

Disallowed:

- trained model inference claim
- visual detection accuracy claim
- general red-object recognition robustness claim
- grasping, contact, load, clearing, or obstacle-removal claim
- autonomous navigation or path-planning claim
- LLM control claim
