# Risk Map Summary Interface - 2026-07-02

## Coordinate Source

Risk map coordinates must be derived from:

```text
bbox + depth + camera_info + odom/map pose
```

They must not be derived from `manual_distance_m` or Windows dataset
`center_depth_m` alone.

## Expected Flow

```text
D435 RGB
-> local detector bbox
-> D435 depth median inside bbox
-> camera_info intrinsics
-> camera_point_xyz_m
-> camera/base extrinsics or TF
-> odom/map pose
-> risk_map_summary.json / risk_map_points.json
```

## Windows Dataset Difference

Windows-side dataset captures may include:

```json
{
  "manual_distance_m": 0.8,
  "center_depth_m": 0.79,
  "pose_available": false,
  "used_for_training": true,
  "used_for_mapping": false
}
```

That distance is only a coverage tag for data collection. It should help answer
questions such as "do we have enough 0.5m, 0.8m, and 1.2m examples?" It is not
a detector label and is not a map-coordinate source.

`center_depth_m` is only a quick center-ROI D435 distance check for capture
quality. It is not a replacement for the detected risk bbox median depth.

## K1 Mapping Requirement

K1/ROS mapping captures must include:

```json
{
  "depth_available": true,
  "pose_available": true,
  "used_for_mapping": true,
  "odom_path": "...",
  "map_pose_path": "..."
}
```

If pose evidence is missing, `projection_status` should be marked missing or
approximate. Do not silently substitute manual distance.

## Claim Boundary

- `manual_distance_m` is not map evidence.
- `center_depth_m` is not map evidence by itself.
- Risk map projection is approximate unless TF/camera calibration is validated.
- Do not claim high-precision SLAM or absolute risk-point accuracy from the
  lightweight projection pipeline alone.
