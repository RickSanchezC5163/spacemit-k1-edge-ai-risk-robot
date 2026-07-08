# P4-X3 Evidence Fields Patch

Date: 2026-06-29

Status: recommended follow-up patch for future captures only.

## Boundary

This patch does not rewrite or backfill frozen P4-X0/X1/X2 evidence under:

```text
outputs/p4x_d435_hold_capture_v1/
```

It does not change the P4-X freeze conclusion. P4-X remains a validation of safe stationary visual evidence capture, not visual detection accuracy, arm manipulation, or autonomous semantic reasoning.

## Future Capture Fields

Future `capture_meta.json` records written by `tools/d435_capture_once.py` include:

- `rgb_header_stamp`
- `depth_header_stamp`
- `rgb_frame_id`
- `depth_frame_id`
- `depth_encoding`
- `depth_scale_m`
- `valid_depth_ratio`

The nested `rgb` and `depth` metadata also include header stamp and frame details for easier local inspection.

## Future Risk Point Fields

Future `risk_point.json` records written by `tools/mock_risk_detector.py` include:

- `depth_scale_m`
- `bbox_valid_depth_samples`
- `bbox_valid_depth_ratio`

Depth fields remain in meters after applying `depth_scale_m`. The risk point remains mock evidence from a fixed/configured bbox and does not claim detector accuracy.

## Compatibility

Older P4-X0/X1/X2 evidence may lack these fields. Consumers should treat them as optional and must not fabricate missing values.
