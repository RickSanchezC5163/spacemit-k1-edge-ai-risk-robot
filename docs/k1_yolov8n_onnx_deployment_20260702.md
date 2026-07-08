# K1 YOLOv8n ONNX Deployment - 2026-07-02

## Summary

YOLOv8n risk-print detection was deployed to the K1 repository and validated
with local offline ONNX inference on K1. This deployment used the official
SpacemiT/Bianbu ONNX Runtime packages, not PyPI `onnxruntime`.

No ROS process was started. No `cmd_vel` was published. No chassis or arm
hardware was controlled.

## Deployed Files

K1 root:

```text
/home/soc/edge-ai-robot-k1/
```

Files:

```text
models/risk_vision/yolov8n.onnx
models/risk_vision/yolov8n_int8.onnx
models/risk_vision/model_report.json
configs/risk_detection_backends.yaml
tools/run_yolo_inference_once.py
tools/export_risk_detection_for_policy.py
src/primitives/d435_depth_localization.py
```

## Runtime Packages

Installed from the SpacemiT/Bianbu apt repository:

```text
spacemit-onnxruntime 2.0.3-bpo1
python3-spacemit-ort 2.0.3-bpo1
spacemit-tcm 3.0.0-bpo1+2
```

Python import check:

```text
onnxruntime 1.24.0+spacemit.a3
providers ['CPUExecutionProvider']
```

## Smoke Input

Offline D435 sample copied to K1:

```text
outputs/k1_yolo_deploy_smoke_input/crack_sample/
  rgb.png
  depth_raw.npy
  camera_info.json
```

## Validation Commands

FP32:

```bash
cd /home/soc/edge-ai-robot-k1
python3 tools/run_yolo_inference_once.py \
  --runtime onnxruntime \
  --image outputs/k1_yolo_deploy_smoke_input/crack_sample/rgb.png \
  --depth outputs/k1_yolo_deploy_smoke_input/crack_sample/depth_raw.npy \
  --camera-info outputs/k1_yolo_deploy_smoke_input/crack_sample/camera_info.json \
  --model models/risk_vision/yolov8n.onnx \
  --output-dir outputs/yolo_inference_k1_v1/crack_sample \
  --conf 0.25
```

INT8 dynamic quantization candidate:

```bash
cd /home/soc/edge-ai-robot-k1
python3 tools/run_yolo_inference_once.py \
  --runtime onnxruntime \
  --image outputs/k1_yolo_deploy_smoke_input/crack_sample/rgb.png \
  --model models/risk_vision/yolov8n_int8.onnx \
  --output-dir outputs/yolo_inference_k1_v1/crack_sample_int8 \
  --conf 0.25
```

## Results

FP32:

```text
backend=yolov8n_onnx_cpu
precision=fp32
detections=1
risk_class_name=crack
risk_confidence=0.8434
risk_distance_m=0.742
online_api_used=false
latency_ms=1759.659
fps=0.568
schema=risk_detection.schema.json PASS
```

INT8 dynamic:

```text
backend=yolov8n_onnx_cpu
precision=int8
detections=1
risk_class_name=crack
risk_confidence=0.8469
online_api_used=false
latency_ms=844.648
fps=1.184
schema=risk_detection.schema.json PASS
```

## Evidence

Mirrored local evidence:

```text
outputs/yolo_inference_k1_v1/crack_sample/risk_detection.json
outputs/yolo_inference_k1_v1/crack_sample/policy_risk_fields.json
outputs/yolo_inference_k1_v1/crack_sample/overlay.png
outputs/yolo_inference_k1_v1/crack_sample_int8/risk_detection.json
outputs/yolo_inference_k1_v1/crack_sample_int8/policy_risk_fields.json
outputs/yolo_inference_k1_v1/crack_sample_int8/overlay.png
```

## Notes

- PyPI `onnxruntime` has no matching wheel for this K1 `riscv64` environment.
- K1 OpenCV DNN can load the ONNX model but `forward()` did not return within a
  60 second timeout, so it is not considered the active deployment runtime.
- SpacemiT ONNX Runtime is the active K1 local inference backend.
- INT8 dynamic ONNX is faster in this single smoke test, but it still needs a
  broader K1 benchmark before performance claims.

## Claim Boundary

- This validates K1-local offline ONNX inference on one D435-captured printed
  risk sample.
- It does not start ROS or control robot hardware.
- It does not claim real-world defect detection accuracy.
- It does not claim high-precision 3D localization.
- It does not claim final real-time performance for the full live D435 flow.
- K1 live camera inference and multi-sample benchmark remain separate steps.
