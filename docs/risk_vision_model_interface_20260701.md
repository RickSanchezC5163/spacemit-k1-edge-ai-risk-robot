# Risk Vision Model Interface - 2026-07-01

Unified function:

```python
detect_risk(
    rgb_path: str,
    depth_path: str | None = None,
    camera_info_path: str | None = None,
    backend: str = "hsv_red_rule",
    output_dir: str | None = None,
) -> dict
```

Configured backends:

- `hsv_red_rule`
- `yolov8n_onnx_cpu`
- `opencv_dnn_onnx`
- `ncnn_int8`
- `printed_risk_classifier_cpu`
- `stub_local_model`

The current stable backend is `hsv_red_rule`; it is a baseline only. The
competition path should replace it with a K1-local lightweight model for A4
printed risk-image detection. Required output fields include backend,
latency/FPS, model metadata, detections, bbox, depth, and camera point.

`RISK_DETECT_LOCAL_MODEL.real_enabled=false` until a local model file is
deployed and benchmarked on K1. Simulator/mock use may still exercise the same
interface through configured fallback behavior.

Every backend reports `runtime_backend`, `precision`,
`acceleration_target`, `model_name`, and `model_size_mb`.

Timing fields are intentionally separated:

- `load_latency_ms`: model/session load smoke time, if measured
- `inference_latency_ms`: actual per-image inference time, if inference ran
- `inference_fps`: FPS derived from actual inference time
- `latency_ms` / `fps`: compatibility aliases for actual inference only, never
  for model loading

If the requested model file is unavailable, the interface must not crash. It
must mark `backend_available=false`, `fallback_used=true`, and avoid claiming
model accuracy.

D435 depth localization is handled by
`src/primitives/d435_depth_localization.py`. It computes bbox median depth,
valid depth ratio, and approximate `camera_point_xyz_m` using camera intrinsics.
It must return `depth_status=missing/invalid` when evidence is absent rather
than backfilling old data.

`detect_risk(...)` applies this localization as a post-detection step whenever
both `depth_path` and `camera_info_path` are provided. Each detection receives a
`depth_localization` object, and the top-level report records
`depth_localization_attempted`, `depth_localization_applied_count`, and
`depth_localization_status`.

K1 deployment path:

- `opencv_dnn_onnx`: CPU fallback candidate
- `yolov8n_onnx_cpu`: target YOLO ONNX CPU backend
- `ncnn_int8`: quantized RVV optimization candidate

All K1 local model claims require K1-side latency/FPS/CPU/memory benchmark
evidence. Do not claim NPU or RVV acceleration without measured backend proof.

For skeleton backends such as `opencv_dnn_onnx`, `yolov8n_onnx_cpu`, and
`ncnn_int8`, `backend_available=true` only means the loader prerequisites were
present. `inference_ready=false` and `inference_executed=false` remain explicit
until preprocessing, forward pass, and postprocess are implemented and tested.

Claim boundary:

- HSV rule is not an AI model.
- Local model claims require K1 benchmark data.
- Online API use is forbidden.
- Server-side inference must not be described as K1 local inference.
