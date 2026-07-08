# Map-A0 Offline Risk Point Projection

Date: 2026-06-30

## Scope

Map-A0 projects P4-X D435 mock risk points into an approximate local odom/map
view using only an existing `episode_report.json`.

Input:

```text
outputs/p4x_d435_hold_capture_v1/episode_report.json
```

Output:

```text
outputs/map_a_risk_point_projection_v1/offline_p4x/
```

This stage is offline only. It does not start ROS, publish `cmd_vel`, access
serial ports, control the mechanical arm, or modify frozen P4-X evidence.

## Tool

```text
tools/project_risk_point_to_map.py
```

Usage:

```powershell
python tools\project_risk_point_to_map.py --episode-report outputs\p4x_d435_hold_capture_v1\episode_report.json --output-dir outputs\map_a_risk_point_projection_v1\offline_p4x
```

Generated files:

- `risk_map_points.json`
- `risk_map_points.csv`
- `risk_map_snapshot.png`
- `projection_report.md`
- `README.md`
- `errors.json`

## Projection Method

The projection is intentionally approximate:

- `projection_mode="approximate_static_camera_offset"`
- `tf_validated=false`
- `slam_used=false`
- `navigation_used=false`
- `camera_offset_base_m=[0.15, 0.0, 0.20]`
- `camera_yaw_offset_rad=0.0`

Existing P4-X risk points store D435 optical-frame points where `z` is depth.
Without TF, the tool uses this explicit approximation:

```text
base_x = camera_z + offset_x
base_y = -camera_x + offset_y
base_z = -camera_y + offset_z
```

Then the base-frame `x/y` point is rotated by the odom yaw and translated by
the robot odom `x/y`.

If `camera_point_xyz_m`, `odom`, or explicit `odom.pose.yaw_rad` is missing,
the point is marked:

```text
projection_status="missing_required_field"
```

The error is recorded in `errors.json`, and report generation continues.

## Claim Boundary

This stage may claim only offline risk point projection from existing P4-X
evidence.

It must not claim:

- SLAM
- autonomous navigation
- path planning
- absolute high-precision risk point coordinates
- mechanical-arm autonomous handling based on map output

## Next Recommended Step

Arm-C0 / P5-D should consume `risk_map_points.json` in dry-run mode and produce
a map-gated no-load arm action candidate with:

- no serial writes
- no hardware execution
- no contact
- no obstacle removal claim
