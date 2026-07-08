# K1 xquant YOLOv8 截断量化验证

日期：2026-07-02

## 目标

修复 YOLOv8 320x320 ONNX 使用 SpaceMIT xquant 截断量化时的 dtype/shape 丢失问题，并验证生成模型在 K1 上的推理表现。

## 修复点

xquant 1.2.1 的 `format_onnx_model()` 会清空 ONNX `value_info`，导致 `truncate_var_names` 指向的 YOLOv8 中间张量在 onnx_graphsurgeon 导出时缺少 dtype/shape。

新增脚本：

```powershell
python tools\run_xquant_yolov8_truncated.py --output models\risk_vision\yolov8n_320_q_truncated.onnx
```

脚本在进程内 patch xquant pipeline：

```text
format_onnx_model()
  -> onnx.shape_inference.infer_shapes(..., data_prop=True)
  -> truncate_onnx_model()
```

不修改 conda/site-packages 内的 xquant 安装包。

## 截断节点

```text
/model.22/Reshape_output_0    float32 [1, 64, 1600]
/model.22/Reshape_1_output_0  float32 [1, 64, 400]
/model.22/Reshape_2_output_0  float32 [1, 64, 100]
```

## 生成结果

```text
models/risk_vision/yolov8n_320_q_truncated.onnx
size: 4,389,063 bytes
nodes: 727
output: output0
```

本机验证集样本：

```text
model: yolov8n_320_q_truncated.onnx
runtime: onnxruntime CPU
detection: corrosion
confidence: 0.9513
latency: 26.116 ms
fps: 38.291
```

## K1 验证

### FP32 320 baseline, SpaceMIT EP

```text
output_dir: outputs/k1_d435_yolo_realtime_v1/headless_320_fp32_spacemit_recheck_001/
detected_frame_count: 24/30
avg_latency_ms: 270.783
avg_infer_fps: 4.028
```

### xquant truncated, CPU EP

```text
output_dir: outputs/k1_d435_yolo_realtime_v1/headless_320_q_truncated_cpu_001/
detected_frame_count: 18/20
avg_latency_ms: 976.005
avg_infer_fps: 1.025
confidence examples: 0.4090 - 0.6382
```

### xquant truncated, SpaceMIT EP

```text
output_dir: outputs/k1_d435_yolo_realtime_v1/headless_320_q_truncated_spacemit_001/
detected_frame_count: 0/60 at conf=0.25
avg_latency_ms: 65.420
avg_infer_fps: 15.533
```

Low confidence diagnostic:

```text
output_dir: outputs/k1_d435_yolo_realtime_v1/headless_320_q_truncated_spacemit_lowconf_001/
conf: 0.001
detected_frame_count: 30/30
avg_latency_ms: 67.585
avg_infer_fps: 15.206
observed issue: confidence compressed to about 0.001-0.015 with large edge-biased boxes
```

## 结论

1. xquant 截断量化生成链路已修复，能够稳定产出 ONNX，并通过 ONNX checker。
2. 生成的截断量化模型在 CPU EP 上检测语义正常。
3. 同一截断量化模型在 K1 SpaceMIT EP 上速度明显提升到约 15 FPS，但检测输出质量异常，不能作为当前展示模型。
4. 当前实机展示仍建议使用 `yolov8n_320_fp32.onnx + SpaceMIT EP`，约 4 FPS，检测稳定。

## Balanced Calibration Update

补充检查发现原始校准列表前 128 张图类别严重偏斜：

```text
crack: 63
corrosion: 64
leakage: 1
blockage: 0
```

这会影响 leakage/blockage 等弱类在量化后的分类头输出。已新增类别均衡校准列表：

```text
models/risk_vision/xquant_yolov8n_320/calib_list_balanced.txt
```

前 128 张校准图分布：

```text
crack: 37
corrosion: 37
leakage: 40
blockage: 17
```

新增配置：

```text
models/risk_vision/xquant_yolov8n_320/yolov8n_320_xquant_config_balanced.json
```

输出模型：

```text
models/risk_vision/yolov8n_320_q_truncated_balanced.onnx
```

本机 blockage 验证集中，旧 `q_truncated` 在一张 blockage 图上误识别为 leakage；balanced 版恢复为 blockage。K1 SpaceMIT EP 当前场景验证：

```text
output_dir: outputs/k1_d435_yolo_realtime_v1/headless_320_q_truncated_balanced_spacemit_current_001/
detected_frame_count: 53/60
avg_latency_ms: 66.445
avg_infer_fps: 15.327
class in current scene: leakage
```

当前推荐：

```text
Use yolov8n_320_q_truncated_balanced.onnx + SpaceMIT EP + supplemental light for realtime K1 CLI/display demos.
Keep yolov8n_320_fp32.onnx + SpaceMIT EP as the accuracy fallback.
```

## Claim Boundary

- 本阶段只声明 xquant 截断量化生成链路修复。
- balanced xquant 截断量化模型可作为补光条件下的 K1 SpaceMIT EP 实时展示候选。
- 不声明该量化模型在弱光、全部类别或真实缺陷场景下稳定。
- 不声明真实缺陷检测准确率。
- 不启动 ROS，不发布 cmd_vel，不打开串口，不控制底盘或机械臂。
