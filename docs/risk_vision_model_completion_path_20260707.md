# Risk Vision Model Completion Path

Date: 2026-07-07

This document summarizes the completed path for the local risk-vision model:
D435 data collection, YOLOv8 training, ONNX export, xquant quantization,
manual truncate-point adjustment, K1 SpaceMIT EP validation, and the current
deployment boundary.

## 1. Target

Build a local K1 risk-vision model for printed risk-card detection.

Supported classes:

```text
0 crack
1 corrosion
2 leakage
3 blockage
```

Primary runtime target:

```text
K1 MUSE Pi Pro
D435 RGB live stream
YOLOv8n ONNX
SpaceMITExecutionProvider + CPUExecutionProvider
no cloud API
```

## 2. Dataset Path

Dataset root:

```text
datasets/risk_print_yolo_v1/
```

Dataset layout:

```text
datasets/risk_print_yolo_v1/
  captures_raw/
  images/
    train/
    val/
  labels/
    train/
    val/
  capture_manifest.csv
  data.yaml
```

`data.yaml`:

```yaml
path: K:/risc-vCar/edge-ai-robot-k1/datasets/risk_print_yolo_v1
train: images/train
val: images/val
names:
  0: crack
  1: corrosion
  2: leakage
  3: blockage
```

Important field boundary:

- `manual_distance_m` from Windows collection is only dataset coverage metadata.
- YOLO training does not use distance labels.
- Final risk-map localization does not use manual distance.
- Final localization uses bbox + D435 depth + camera_info + odom/map pose.

## 3. Training Path

### First training run

Run:

```text
datasets/risk_print_yolo_v1/runs/detect/risk_print_yolov8n_e50/
```

Source model:

```text
yolov8n.pt
```

Training:

```powershell
yolo train model=yolov8n.pt data=K:\risc-vCar\edge-ai-robot-k1\datasets\risk_print_yolo_v1\data.yaml epochs=50 imgsz=640
```

The first run proved the training chain, but `blockage` was weaker than the
other classes, so extra raw samples were collected and labeled.

### Blockage03 refinement run

Run:

```text
datasets/risk_print_yolo_v1/runs/detect/risk_print_yolov8n_blockage03_e50/
```

Source model:

```text
datasets/risk_print_yolo_v1/runs/detect/risk_print_yolov8n_e50/weights/best.pt
```

Training command pattern:

```powershell
yolo train ^
  model=K:\risc-vCar\edge-ai-robot-k1\datasets\risk_print_yolo_v1\runs\detect\risk_print_yolov8n_e50\weights\best.pt ^
  data=K:\risc-vCar\edge-ai-robot-k1\datasets\risk_print_yolo_v1\data.yaml ^
  epochs=50 imgsz=640 device=0 workers=0 ^
  project=K:\risc-vCar\edge-ai-robot-k1\datasets\risk_print_yolo_v1\runs\detect ^
  name=risk_print_yolov8n_blockage03_e50 exist_ok=True
```

Final `results.csv` last epoch:

```text
precision(B): 0.97762
recall(B):    0.91767
mAP50(B):     0.96000
mAP50-95(B):  0.69510
```

Current training artifact:

```text
datasets/risk_print_yolo_v1/runs/detect/risk_print_yolov8n_blockage03_e50/weights/best.pt
```

## 4. ONNX Export Path

The K1 camera stream is 640x480. ONNX uses NCHW shape, so the deployed
rectangular model is:

```text
[1, 3, 480, 640]
```

This is not reversed. It means:

```text
height = 480
width  = 640
```

FP32 ONNX artifact:

```text
models/risk_vision/yolov8n_480x640_fp32_blockage03.onnx
```

Deployment scripts were updated to read the ONNX input shape directly instead
of assuming square `imgsz x imgsz`.

Updated tools:

```text
tools/run_yolo_inference_once.py
tools/run_d435_yolo_realtime_win.py
tools/run_k1_d435_yolo_realtime_display.py
```

## 5. xquant Quantization Path

Quantization tool:

```text
xquant
```

Quantization is performed on x86, not on K1.

Calibration config directory:

```text
models/risk_vision/xquant_yolov8n_480x640/
```

Balanced calibration list:

```text
models/risk_vision/xquant_yolov8n_480x640/calib_list_balanced.txt
```

Calibration summary:

```json
{
  "total_images": 207,
  "calibration_step": 128,
  "first_step_primary_class_counts": {
    "crack": 33,
    "corrosion": 33,
    "leakage": 32,
    "blockage": 30
  },
  "all_primary_class_counts": {
    "crack": 63,
    "corrosion": 64,
    "leakage": 50,
    "blockage": 30
  }
}
```

The calibration list was balanced because the earlier calibration order was
class-skewed and hurt weaker classes, especially `blockage`.

## 6. Manual Truncate-Point Adjustment

### Original issue

The first xquant truncated model only protected the YOLOv8 bbox branch reshape
outputs:

```text
/model.22/Reshape_output_0
/model.22/Reshape_1_output_0
/model.22/Reshape_2_output_0
```

This produced a model that loaded and ran on K1 SpaceMIT EP, but the output
quality was abnormal in real camera scenes:

```text
Many background regions were detected as blockage with high confidence.
```

The issue was not a simple D435 color-channel flip. The same K1 captured RGB
frame tested on CPU did not show the same behavior. The problem was consistent
with quantization error in the class branch.

### Netron manual inspection

The model was opened in Netron to inspect YOLOv8 head outputs. The class branch
reshape outputs were identified manually:

```text
Reshape_3 output: PPQ_Operation_717
  source: /model.22/cv3.0/cv3.0.2/Conv

Reshape_4 output: PPQ_Operation_785
  source: /model.22/cv3.1/cv3.1.2/Conv

Reshape_5 output: PPQ_Operation_853
  source: /model.22/cv3.2/cv3.2.2/Conv
```

These correspond to the YOLOv8 class prediction branches at three scales.

### Final 6-point truncate list

Final config:

```text
models/risk_vision/xquant_yolov8n_480x640/yolov8n_480x640_xquant_config_truncated6_balanced_blockage03.json
```

Final truncate points:

```json
[
  "/model.22/Reshape_output_0",
  "/model.22/Reshape_1_output_0",
  "/model.22/Reshape_2_output_0",
  "/model.22/Reshape_3_output_0",
  "/model.22/Reshape_4_output_0",
  "/model.22/Reshape_5_output_0"
]
```

Interpretation:

```text
Reshape 0/1/2: bbox branch protection
Reshape 3/4/5: class branch protection
```

The script supports this mode:

```powershell
python tools\run_xquant_yolov8_truncated.py ^
  --config models\risk_vision\xquant_yolov8n_480x640\yolov8n_480x640_xquant_config_truncated6_balanced_blockage03.json ^
  --output models\risk_vision\yolov8n_480x640_q_truncated6_balanced_blockage03.onnx ^
  --truncate-mode cv2_cv3
```

Final quantized artifact:

```text
models/risk_vision/yolov8n_480x640_q_truncated6_balanced_blockage03.onnx
```

Model size:

```text
3,349,961 bytes
```

## 7. K1 Deployment Path

K1 runtime observed:

```text
onnxruntime: 1.24.0+spacemit.a3
providers: SpaceMITExecutionProvider + CPUExecutionProvider
```

Recommended command-line realtime launch:

```bash
cd /home/soc/edge-ai-robot-k1

sudo env PYTHONUNBUFFERED=1 python3 tools/run_k1_d435_yolo_realtime_display.py \
  --provider spacemit \
  --model models/risk_vision/yolov8n_480x640_q_truncated6_balanced_blockage03.onnx \
  --width 640 --height 480 --fps 15 --imgsz 640 \
  --conf 0.15 --iou 0.45 --max-det 10 \
  --warmup-frames 90 \
  --cli-realtime \
  --cli-print-period-s 0.5 \
  --output-dir outputs/k1_d435_yolo_realtime_v1/cli_ep_480x640_truncated6_light5
```

Expected startup evidence:

```text
Using ONNX Runtime providers: ['SpaceMITExecutionProvider', 'CPUExecutionProvider']
Active ONNX Runtime providers: ['SpaceMITExecutionProvider', 'CPUExecutionProvider']
Model input shape: [1, 3, 480, 640]
```

Stable snapshot evidence examples were saved under:

```text
outputs/k1_d435_yolo_realtime_v1/stable_snapshot_ep_480x640_truncated6_light5_*/
```

These are evidence outputs and are not committed to Git.

### Manual field adjustment after K1 dry-run

The model weights were not changed during the final K1 field dry-run. The
manual adjustment was applied around the model as runtime acceptance gates and
risk-map/report correction rules.

The raw detector is still launched with a low candidate threshold so that the
operator can see borderline detections:

```text
--conf 0.15
```

Automatic alarm/risk-map insertion is then gated by class-specific confidence
and D435 depth. The current preliminary-demo boundary is:

```text
crack:
  confidence >= 0.29
  0.60 m <= depth <= 0.80 m

blockage:
  confidence >= 0.23
  0.35 m <= depth <= 0.75 m
```

Command-line form:

```bash
--auto-risk-gates crack:0.29:0.60:0.80,blockage:0.23:0.35:0.75
```

Reasoning:

- `--conf 0.15` keeps the live overlay useful for observation.
- The class-specific gate decides which detections are allowed to trigger alarm
  and map insertion.
- Confidence alone was not stable enough for the small demo scene.
- D435 depth was added to reject detections outside the expected handling range.
- `corrosion` and `leakage` are still kept as detectable classes, but their
  final demo inclusion was handled through event review/reporting rather than a
  frozen automatic alarm gate.

Manual review process:

1. Run K1 D435 YOLO and save each risk-event evidence packet:
   `overlay.png`, `rgb.png`, `depth_raw.npy`, `camera_info.json`,
   `odom.json`, and `risk_detection.json`.
2. Review the overlay/RGB evidence frame by frame.
3. Mark which detections are visually correct and which are background/geometry
   false positives.
4. Tune the acceptance boundary using both confidence and depth.
5. Keep the tuned gate in the demo command, not inside the model weights.

For the final preliminary demo risk map, the `corrosion`/rust point was manually
reviewed and included as one final risk point with display confidence:

```text
R2 corrosion confidence_display = 0.5780
```

This correction belongs to the risk-map/report layer. It should not be described
as retraining, improved model accuracy, or an automatic corrosion alarm gate.

Lighting decision from manual sampling:

- D435 auto exposure handled the bright/dark placement well enough for most
  printed risk-card detections.
- Dynamic supplemental-light control was not needed.
- Fixed supplemental light can still be used for dark pipe-cavity `leakage`
  shots if the live overlay becomes unstable.

## 8. Stream-vs-File Runtime Finding

The original idea of periodically saving images and then reading file paths for
inference was tested and is not the preferred runtime architecture.

Benchmark output:

```text
outputs/k1_d435_yolo_realtime_v1/stream_vs_file_benchmark_20260705_001/
```

Result summary on K1:

```text
stream_in_memory:
  inference_avg: 96.477 ms
  e2e_avg:       298.841 ms
  e2e_fps:       3.346

png_file_roundtrip:
  inference_avg: 94.463 ms
  png_write_avg: 55.960 ms
  png_read_avg:  21.220 ms
  e2e_avg:       372.680 ms
  e2e_fps:       2.683
```

Conclusion:

```text
D435 live frame stream -> in-memory YOLO inference
```

is the correct runtime path. Full RGB/depth/camera_info/pose evidence should be
saved only when a risk event is triggered.

Current bottleneck:

```text
Python postprocess/NMS is slower than ONNX inference in the current script.
```

So future optimization should target postprocess/NMS, not just SpaceMIT EP.

## 9. Final Runtime Architecture

Realtime detection path:

```text
D435 live RGB frame in memory
  -> YOLO ONNX / SpaceMIT EP
  -> bbox, class, confidence
  -> risk_detection event
```

Evidence path after event trigger:

```text
risk event
  -> save rgb.png
  -> save depth_raw.npy / depth_vis.png
  -> save camera_info.json
  -> save odom/map pose
  -> save risk_detection.json
  -> save risk_point.json
  -> episode_report.json
  -> LLM-A deterministic report
```

Mapping position source:

```text
YOLO bbox
+ D435 depth median
+ camera_info intrinsics
+ odom/map pose
+ camera/base approximate extrinsic or TF
```

Manual dataset distance is not used for final map projection.

## 10. Current Recommended Model

Recommended K1 demo model:

```text
models/risk_vision/yolov8n_480x640_q_truncated6_balanced_blockage03.onnx
```

Recommended K1 runtime:

```text
SpaceMITExecutionProvider + CPUExecutionProvider
D435 640x480 YUY2
no supplemental light by default
fixed supplemental light only when dark leakage scenes require it
90-frame warmup before evidence capture or benchmark
candidate detector threshold: --conf 0.15
auto alarm/map gates:
  crack    confidence >= 0.29, 0.60 m <= depth <= 0.80 m
  blockage confidence >= 0.23, 0.35 m <= depth <= 0.75 m
```

Fallback model:

```text
models/risk_vision/yolov8n_480x640_fp32_blockage03.onnx
```

Use fallback only for CPU/accuracy comparison or quantization regression
diagnosis.

## 11. Claim Boundary

Allowed claims:

- The model is trained on the project D435-captured printed risk-card dataset.
- The model runs locally on K1 with ONNX Runtime and SpaceMIT EP.
- The final K1 demo model uses xquant INT8 quantization with manually verified
  YOLOv8 head truncate points.
- The preliminary demo uses manually reviewed confidence+depth gates for
  automatic alarm/risk-map insertion.
- The runtime avoids periodic PNG file roundtrips and uses live frame inference.
- Evidence is saved only after risk-event trigger for audit/report generation.

Not allowed claims:

- Do not claim real-world crack/corrosion/leakage/blockage detection accuracy.
- Do not claim the printed-card dataset generalizes to real industrial defects.
- Do not claim manual dataset distance is used for final map localization.
- Do not claim high-precision SLAM or high-precision 3D localization from this
  model alone.
- Do not claim LLM controls the robot.
- Do not claim autonomous obstacle removal from the vision model alone.
- Do not claim the manually included final-demo `corrosion` point is an
  automatically learned model improvement.

## 12. Next Recommended Work

1. Optimize postprocess/NMS on K1.
2. Compare 480x640 truncated6 against a 320x320 deployment variant.
3. Add a compact benchmark table to the final project README.
4. Preserve Netron screenshots or node notes for the six truncate points.
5. Integrate the realtime risk event into Step7 live flow using in-memory frames,
   with evidence saved only on trigger.
6. Collect more true `corrosion` and dark `leakage` frames, then retrain or
   recalibrate instead of relying on manual report-layer correction.
