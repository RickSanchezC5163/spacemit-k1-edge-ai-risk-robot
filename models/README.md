# Models

本目录只保留可复现演示所需的轻量化模型样例，避免提交训练数据和中间工作目录。

当前包含：

```text
risk_vision/yolov8n_480x640_q_truncated6_balanced_blockage03.onnx
```

该模型用于 K1 端 D435 风险识别演示，配套量化和部署记录见：

```text
docs/k1_yolov8n_onnx_deployment_20260702.md
docs/k1_xquant_yolov8_truncated_quantization_20260702.md
docs/risk_vision_model_completion_path_20260707.md
```
