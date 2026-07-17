# K1 YOLO Runtime Optimization Plan - 2026-07-17

## Current Finding

The CPU no-visuals path is stable but too slow for real-time field scanning:

```text
CPU ONNX Runtime, no per-frame visuals:
mean infer_fps ~= 0.13 FPS
mean latency ~= 7.8 s/frame
```

Therefore, disabling overlay/dashboard writes is useful for stability and data logging, but it does not solve the core CPU inference bottleneck.

The SpaceMIT EP failure was traced to `tcm buffer acquire failed`, matching the known `spacemit-ort` runtime issue. The 2.0.5 package fixed the minimal EP smoke test. The next practical work should prioritize EP 2.0.5 calibration before spending time on CPU-only real-time operation.

## Runtime Priority

```text
Priority 1: SpaceMIT EP 2.0.5 + class confidence recalibration
Priority 2: ORT CPU thread/taskset benchmark as correctness fallback
Priority 3: V4L2 latest-frame asynchronous pipeline for perceived real-time UI
Priority 4: ncnn FP32 RVV / C++ backend if CPU must approach 3 FPS
```

Do not use CPU no-visuals as the real-time main line unless the benchmark proves it can reach the required rate. Based on current measurement, it is a low-frequency high-confidence logging fallback.

## K1 Bring-Up After Power-On

Copy the updated scripts if they are not already on K1:

```powershell
scp K:\risc-vCar\edge-ai-robot-k1\tools\run_prelim_remote_mapping_yolo_arm_demo.py soc@192.168.43.40:/home/soc/edge-ai-robot-k1/tools/run_prelim_remote_mapping_yolo_arm_demo.py
scp K:\risc-vCar\edge-ai-robot-k1\tools\benchmark_k1_yolo_cpu_ort_threads.py soc@192.168.43.40:/home/soc/edge-ai-robot-k1/tools/benchmark_k1_yolo_cpu_ort_threads.py
scp K:\risc-vCar\edge-ai-robot-k1\spacemit-ort.riscv64.2.0.5.tar.gz soc@192.168.43.40:/home/soc/edge-ai-robot-k1/third_party/spacemit-ort.riscv64.2.0.5.tar.gz
```

Install or unpack SpaceMIT ORT 2.0.5 into a persistent location:

```bash
cd /home/soc/edge-ai-robot-k1
mkdir -p third_party/spacemit_ort_205
tar -xzf third_party/spacemit-ort.riscv64.2.0.5.tar.gz -C third_party/spacemit_ort_205
python3 -m pip install --user --break-system-packages --force-reinstall --no-deps \
  third_party/spacemit_ort_205/spacemit-ort.riscv64.2.0.5/python/spacemit_ort-2.0.5-py3-none-linux_riscv64.whl
```

Verify EP provider registration:

```bash
cd /home/soc/edge-ai-robot-k1
export LD_LIBRARY_PATH=/home/soc/edge-ai-robot-k1/third_party/spacemit_ort_205/spacemit-ort.riscv64.2.0.5/lib:${LD_LIBRARY_PATH:-}
python3 - <<'PY'
import importlib.metadata as m
import onnxruntime as ort
print("ort", ort.__version__, ort.__file__)
print("providers before", ort.get_available_providers())
print("spacemit-ort", m.version("spacemit-ort"))
import spacemit_ort
print("spacemit_ort", spacemit_ort.__file__)
print("providers after", ort.get_available_providers())
PY
```

Expected:

```text
providers after ['SpaceMITExecutionProvider', 'CPUExecutionProvider']
```

## Test 1 - EP 2.0.5 Detection Gate Calibration

Run the mapping stack and D435 as usual, then start YOLO with EP 2.0.5. Do not reuse the old CPU gate `0.60` as the EP gate. Test lower gates first:

```text
*:0.15:0.20:1.20
*:0.20:0.20:1.20
*:0.25:0.20:1.20
*:0.30:0.20:1.20
```

Example:

```bash
cd /home/soc/edge-ai-robot-k1
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
export LD_LIBRARY_PATH=/home/soc/edge-ai-robot-k1/third_party/spacemit_ort_205/spacemit-ort.riscv64.2.0.5/lib:${LD_LIBRARY_PATH:-}

RUN=$(cat .current_real_k1_rrt_nav2_run_dir)
PYTHONUNBUFFERED=1 python3 tools/run_prelim_remote_mapping_yolo_arm_demo.py \
  --provider spacemit \
  --model models/risk_vision/yolov8n_480x640_q_truncated6_balanced_blockage03.onnx \
  --imgsz 640 --conf 0.15 --iou 0.45 --max-det 10 \
  --timer-period-s 0.10 --inference-period-s 0.20 \
  --min-depth-m 0.20 --max-depth-m 1.20 \
  --alarm-topic /perception/risk_alarm \
  --auto-risk-gates '*:0.20:0.20:1.20' \
  --dedup-map-grid-m 0.20 \
  --arm-response-mode disabled \
  --output-dir "$RUN/yolo_risk_ep205_gate020"
```

Acceptance criteria:

```text
No tcm buffer acquire failed
latest_overlay.png keeps refreshing
alarm_state.json updates continuously
6 placed risk points are observed across one route
false positives are tolerable or explainable
```

## Test 2 - CPU ORT Thread Matrix

Use a fixed frame first. This answers whether CPU has any chance of approaching 3 FPS independent of camera and ROS timing.

```bash
cd /home/soc/edge-ai-robot-k1
taskset -c 3-7 env \
  OMP_NUM_THREADS=1 \
  OPENBLAS_NUM_THREADS=1 \
  MKL_NUM_THREADS=1 \
  NUMEXPR_NUM_THREADS=1 \
  python3 tools/benchmark_k1_yolo_cpu_ort_threads.py \
    --model models/risk_vision/yolov8n_480x640_q_truncated6_balanced_blockage03.onnx \
    --threads 2,3,4,5,6,7 \
    --inter-op-threads 1 \
    --allow-spinning 1 \
    --opencv-num-threads 1 \
    --warmup 20 \
    --frames 80 \
    --output-dir outputs/k1_yolo_cpu_ort_thread_benchmark
```

If D435 V4L2 is busy, pass an existing captured RGB image:

```bash
python3 tools/benchmark_k1_yolo_cpu_ort_threads.py \
  --image outputs/real_k1_rrt_nav2_mapping_20260717_161040/yolo_risk_cpu_novis_conf06/captures/<event_id>/rgb.png \
  --threads 2,3,4,5,6,7 \
  --warmup 20 \
  --frames 80
```

Acceptance criteria:

```text
>= 3 FPS median e2e_compute: CPU line can be considered
< 1 FPS median e2e_compute: CPU line remains fallback only
```

## Test 3 - CPU No-Visuals Logging Fallback

If EP output confidence is unacceptable, use CPU no-visuals only for low-frequency risk logging:

```bash
cd /home/soc/edge-ai-robot-k1
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
RUN=$(cat .current_real_k1_rrt_nav2_run_dir)

taskset -c 3-7 env \
  OMP_NUM_THREADS=1 \
  OPENBLAS_NUM_THREADS=1 \
  MKL_NUM_THREADS=1 \
  NUMEXPR_NUM_THREADS=1 \
  PYTHONUNBUFFERED=1 \
  python3 tools/run_prelim_remote_mapping_yolo_arm_demo.py \
    --provider cpu \
    --model models/risk_vision/yolov8n_480x640_q_truncated6_balanced_blockage03.onnx \
    --imgsz 640 --conf 0.15 --iou 0.45 --max-det 10 \
    --timer-period-s 0.10 --inference-period-s 0.20 \
    --opencv-num-threads 1 \
    --ort-graph-optimization-level all \
    --ort-execution-mode sequential \
    --ort-intra-op-threads 5 \
    --ort-inter-op-threads 1 \
    --ort-allow-spinning 1 \
    --min-depth-m 0.20 --max-depth-m 1.20 \
    --alarm-topic /perception/risk_alarm \
    --auto-risk-gates '*:0.60:0.20:1.20' \
    --dedup-map-grid-m 0.20 \
    --arm-response-mode disabled \
    --no-visuals \
    --output-dir "$RUN/yolo_risk_cpu_novis_conf06"
```

Expected based on current data:

```text
~0.1-0.2 FPS, stable, correct confidence, not real-time
```

## Decision Rule

```text
If EP 2.0.5 with calibrated gate detects all 6 points:
  use EP for the live demo.

If EP 2.0.5 still misses too much but CPU benchmark is below 1 FPS:
  use EP for video/live display and CPU only for post-route evidence confirmation.

If CPU benchmark approaches 3 FPS:
  build CPU no-visuals/async latest-frame as a real fallback.

If neither EP nor CPU meets the target:
  move to ncnn FP32 RVV or C++ ORT, not more Python-level tuning.
```

