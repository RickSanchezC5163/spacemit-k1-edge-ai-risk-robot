# K1 D435 Risk Vision Deployment Plan - 2026-07-01

## Goal

Prepare the D435 RGB/depth risk-vision chain for K1-local inference while
preserving the existing Step7 safety boundaries.

Target chain:

```text
D435 RGB
-> local risk detector backend
-> bbox
-> D435 depth median inside bbox
-> camera_point_xyz_m
-> approximate map projection
-> action candidate / report
```

This plan does not start ROS, control the chassis, control the arm, open serial
ports, or use online APIs.

## Backend Plan

The unified interface is `detect_risk(...)` in
`src/primitives/risk_vision_primitives.py`.

Configured backends:

- `hsv_red_rule`: deterministic baseline, `model_used=false`
- `opencv_dnn_onnx`: K1 CPU fallback candidate, `model_used=true`
- `yolov8n_onnx_cpu`: target YOLO ONNX CPU backend, `model_used=true`
- `ncnn_int8`: quantized ncnn/RVV optimization candidate, `model_used=true`
- `printed_risk_classifier_cpu`: printed A4 risk classifier candidate
- `stub_local_model`: interface test only, `model_used=false`

All backends must report:

```text
online_api_used=false
runtime_backend
precision
acceleration_target
model_name
model_size_mb
latency_ms
fps
claim_boundary
```

## D435 Depth Localization

`localize_bbox_with_depth(...)` in
`src/primitives/d435_depth_localization.py` computes:

- bbox median valid depth in meters
- valid depth ratio inside bbox
- approximate camera-frame point using `fx/fy/cx/cy`

It must return `depth_status=missing/invalid` instead of backfilling evidence
when depth or camera info is absent.

This is approximate localization only. Do not claim high-precision 3D position
unless TF and camera calibration are validated.

## K1 Deployment Notes

K1 is a RISC-V AI CPU/RVV-capable platform. The intended optimization path is:

1. CPU fallback: `opencv_dnn_onnx`
2. YOLO ONNX CPU runtime: `yolov8n_onnx_cpu`
3. quantized deployment candidate: `ncnn_int8`
4. only after real measurement: discuss RVV/backend acceleration

Do not claim NPU acceleration unless a measured K1 backend proves it.

Do not use server-side inference as a substitute for K1 local inference.

## Benchmark Requirements

Run on the target platform and record:

- latency average / p50 / p95
- load latency, separated from inference latency
- FPS
- CPU and memory, when available
- model name and size
- runtime backend
- precision
- acceleration target
- image count

Current benchmark entry point:

```powershell
python tools\benchmark_risk_vision_backend.py --backend hsv_red_rule --image-dir assets\risk_print_set_v1\samples --output-dir outputs\risk_vision_benchmark_v1\hsv
python tools\benchmark_risk_vision_backend.py --backend stub_local_model --image-dir assets\risk_print_set_v1\samples --output-dir outputs\risk_vision_benchmark_v1\stub
```

Future model benchmark:

```powershell
python tools\benchmark_risk_vision_backend.py --backend opencv_dnn_onnx --model models\risk_vision\yolov8n.onnx --image-dir assets\risk_print_set_v1\samples --output-dir outputs\risk_vision_benchmark_v1\onnx
```

When a backend only performs model-load smoke validation, benchmark output must
store that time as `load_latency_ms`; `inference_latency_ms`, `inference_fps`,
`latency_ms`, and `fps` must stay empty/zero so reviewers do not mistake model
loading for detector throughput.

## Claim Boundary

Allowed current claims:

- D435 RGB/depth risk-vision interface is prepared.
- HSV red-rule remains a deterministic baseline.
- Local model backends have configuration and skeleton adapters.
- Benchmark outputs can be generated without ROS or hardware control.

Not allowed:

- HSV is an AI model.
- a model has been deployed on K1 without model files and benchmark evidence.
- server inference is K1 local inference.
- NPU/RVV acceleration exists without K1 measurement.
- bbox depth localization is high-precision 3D localization.
