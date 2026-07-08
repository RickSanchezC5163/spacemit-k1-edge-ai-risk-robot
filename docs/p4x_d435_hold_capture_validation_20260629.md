# P4-X D435 HOLD_CAPTURE Validation

Date: 2026-06-29

Local evidence root:

```text
K:\risc-vCar\edge-ai-robot-k1\outputs\p4x_d435_hold_capture_v1
```

Remote run root on K1:

```text
/home/soc/edge-ai-robot-k1/outputs/p4x_d435_hold_capture_v1
```

## Objective

Validate D435 as a stationary visual evidence source for the current policy action / episode report protocol. Phase 1 does not do real recognition, arm manipulation, model training, Gazebo, RL, or LLM-in-the-loop control.

## Result

P4-X0 / P4-X1 / P4-X2 passed and are frozen.

P4-X validates safe stationary visual evidence capture. It does not modify the P4-V base safety chain.

## P4-X0 Topic Audit

Result: PASS.

| Topic | Shape | Encoding | Rate | Status |
| --- | --- | --- | --- | --- |
| `/camera/camera/color/image_raw` | 640x480 | `rgb8` | about 30 Hz | readable |
| `/camera/camera/depth/image_rect_raw` | 640x480 | `16UC1` | about 29 Hz | readable |
| `/camera/camera/color/camera_info` | 640x480 | `plumb_bob` | about 31.6 Hz | readable |

Evidence:

- `outputs/p4x_d435_hold_capture_v1/d435_topic_audit.json`
- `outputs/p4x_d435_hold_capture_v1/d435_topic_audit.md`

## P4-X1 Capture Once

Result: PASS.

Saved files:

- `rgb.png`
- `depth_raw.npy`
- `depth_vis.png`
- `camera_info.json`
- `odom.json`
- `capture_meta.json`
- `risk_point.json`

Evidence directory:

```text
outputs/p4x_d435_hold_capture_v1/captures/p4x_capture_once_001
```

## P4-X2 HOLD_CAPTURE Validation

Result: PASS.

- 10/10 succeeded.
- 0 `failed_safe`.
- Every validation capture has `base_zero_ok_before=True`.
- `acceptance_10_runs_9_success=true`.
- `published_cmd_vel=false`.
- `errors.json=[]`.
- Total capture directories: 11, including 1 capture_once and 10 HOLD_CAPTURE validation captures.

Evidence:

- `outputs/p4x_d435_hold_capture_v1/p4x_hold_capture_status.csv`
- `outputs/p4x_d435_hold_capture_v1/episode_report.json`
- `outputs/p4x_d435_hold_capture_v1/errors.json`

## Evidence Chain

The frozen chain for each successful HOLD_CAPTURE is:

```text
PolicyAction(HOLD_CAPTURE)
-> base_zero_ok_before=True
-> D435 RGB/depth/camera_info + odom capture
-> capture_meta.json
-> mock risk_point.json
-> ActionResult(status=succeeded)
-> episode_report.json
```

HOLD_CAPTURE did not publish `cmd_vel`.

## Depth And Metadata Audit

The first HOLD_CAPTURE `depth_raw.npy` loads successfully as:

```text
shape: 480x640
dtype: uint16
```

Depth audit:

- `depth_raw.npy` is `uint16` raw depth.
- `capture_meta.depth.encoding=16UC1`.
- `capture_meta.depth.depth_scale_m=0.001`.
- `risk_point.depth_median_m` is stored in meters.
- `risk_point.camera_point_xyz_m` is stored in meters.
- Depth unit conversion is explicit in metadata and risk notes.

Timestamp audit:

- `capture_meta.timestamp` is present.
- `camera_info.json.header.stamp` is present.
- `odom.json.header.stamp` is present.
- `risk_point.timestamp` is present.
- Per-capture RGB/depth ROS header timestamps are not present in `capture_meta.json`.

Ratio audit:

- `capture_meta.depth.valid_count` is present.
- `valid_depth_ratio` is not present.
- `risk_point.notes` includes `valid_depth_samples=...`.
- `bbox_valid_depth_ratio` is not present.

## P4-X3 Recommendations

Do not rewrite P4-X0/X1/X2 evidence. In P4-X3, add:

- RGB and depth ROS header timestamps to `capture_meta.json`.
- `valid_depth_ratio`.
- `bbox_valid_depth_ratio`.
- Structured `bbox_valid_depth_samples`.

## Claim Boundary

This validation does not claim visual detection accuracy.
This validation does not claim arm manipulation.
This validation does not claim autonomous semantic reasoning.
This validation does not claim model training or RL progress.
This validation does not modify the P4-V base safety chain.
