# K1 HOLD_CAPTURE Mapping Schema - 2026-07-02

## Purpose

This schema defines the capture record expected from K1/ROS-side
`HOLD_CAPTURE` when the capture is intended for final risk-map projection.

Windows dataset captures are training examples. K1 `HOLD_CAPTURE` records are
mapping evidence.

## Required Capture Fields

```json
{
  "rgb_path": "...",
  "depth_path": "...",
  "camera_info_path": "...",
  "odom_path": "...",
  "map_pose_path": "...",
  "pose_available": true,
  "used_for_mapping": true
}
```

Recommended additional fields:

```json
{
  "capture_id": "...",
  "captured_at": "...",
  "base_zero_ok_before_capture": true,
  "published_cmd_vel_during_capture": false,
  "depth_available": true,
  "camera_frame_id": "...",
  "depth_frame_id": "...",
  "rgb_header_stamp": "...",
  "depth_header_stamp": "...",
  "depth_scale_m": 0.001,
  "risk_trigger_source": "D435_local_model_or_rule",
  "used_for_training": false
}
```

## Mapping Inputs

Final risk-point projection should use:

```text
bbox + depth + camera_info + odom/map pose
```

The projection chain is:

```text
detector bbox
-> bbox median depth
-> camera_point_xyz_m using camera_info
-> base_link via camera/base extrinsics or TF
-> odom/map pose
-> risk_map_points.json
```

## Field Boundary

`manual_distance_m` from Windows dataset collection must not be used here. It
does not represent calibrated camera geometry and has no odom/map pose.

`center_depth_m` from Windows dataset collection must not be used as a map
coordinate either. It is only an approximate center-ROI distance check for
capture quality; final risk projection must use the detected risk bbox median
depth, camera intrinsics, and odom/map pose.

Projection is approximate unless camera calibration and TF are validated.
`used_for_mapping=true` requires pose evidence.
