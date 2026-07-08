# D435 Dataset Collection Workflow - 2026-07-01

## Purpose

This workflow is for Windows-side D435 dataset collection after the camera is
mounted in its final vehicle position. It is used to collect RGB/depth examples
for local risk-model training and coverage review.

It is not the final map-positioning evidence path.

## Windows Collection Tool

Run on Windows with the D435 connected directly to the laptop:

```powershell
python tools\capture_d435_dataset_win.py --output-dir outputs\d435_dataset_win_v1
```

Optional initial fields:

```powershell
python tools\capture_d435_dataset_win.py `
  --class-name crack `
  --print-id crack_a4_001 `
  --manual-distance-m 0.8 `
  --angle front `
  --light normal `
  --output-dir outputs\d435_dataset_win_v1
```

`--distance-m` is accepted as a legacy alias, but all outputs use
`manual_distance_m`.

The UI shows the live RGB frame only. It also displays a rough center-distance
reading computed from the D435 depth median in a small center ROI. The live UI
does not render a depth preview; the saved `depth_vis.png` is a false-color
depth visualization for later review.

## Output Structure

```text
outputs/d435_dataset_win_v1/
  capture_manifest.csv
  captures/<capture_id>/
    rgb.png
    depth_raw.npy
    depth_vis.png       # false-color depth visualization
    camera_info.json
    meta.json
```

`capture_manifest.csv` columns:

```text
capture_id,class_name,print_id,manual_distance_m,angle,light,rgb_path,depth_path,depth_vis_path,camera_info_path,meta_path,depth_available,pose_available,used_for_training,used_for_mapping,center_depth_m,center_depth_valid_ratio,center_depth_roi_xywh,center_distance_band,note
```

`meta.json` includes:

```json
{
  "manual_distance_m": 0.8,
  "depth_available": true,
  "depth_scale_m": 0.001,
  "center_depth_m": 0.79,
  "center_depth_valid_ratio": 0.98,
  "center_depth_roi_xywh": [280, 210, 80, 60],
  "center_distance_band": "near",
  "pose_available": false,
  "used_for_training": true,
  "used_for_mapping": false,
  "distance_source": "manual_for_dataset_coverage"
}
```

## Field Semantics

`manual_distance_m` is manually entered by the operator, such as `0.5`, `0.8`,
or `1.2` meters. It is used only for dataset coverage statistics, for example
checking whether the training set covers near, medium, and far views.

YOLO training does not use `manual_distance_m` as a label. The detector should
learn image-space classes and bounding boxes, not a manual distance field.

Final risk-point map positioning must not use `manual_distance_m`.

`center_depth_m` is computed from the median valid D435 depth value inside a
small center ROI. It is a quick capture-quality and scene-distance reference
for the operator, not a ground-truth distance label and not a final map
projection input.

`center_distance_band` is a rough operator-facing bucket derived from
`center_depth_m`: `very_close` is below 0.4 m, `near` is 0.4-0.8 m, `medium` is
0.8-1.2 m, and `far` is 1.2 m or above. It is for capture review only.

## Final Mapping Evidence Path

The final map position of a risk point must come from:

```text
YOLO bbox
+ D435 depth median
+ camera_info intrinsics
+ odom/map pose
+ camera/base approximate extrinsics or TF
```

Windows dataset collection has `pose_available=false` and
`used_for_mapping=false` because it does not record K1 odom/map pose.

K1/ROS final demonstration captures must record depth, camera_info, and
odom/map pose so risk-point projection can be traced.

## Claim Boundary

- Windows D435 dataset captures are training-set evidence only.
- Manual distance is dataset metadata for coverage review.
- Center depth is an approximate D435 quality check.
- Saved depth visualization is false-color review evidence, not a model label.
- Manual distance is not a map-positioning input.
- Center depth and center distance band alone are not map-positioning inputs.
- Risk projection is approximate unless TF/camera calibration is validated.
- Do not claim final map accuracy from Windows-only captures.
