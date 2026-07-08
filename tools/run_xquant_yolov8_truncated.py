#!/usr/bin/env python3
"""Run SpaceMIT xquant YOLOv8 truncation with a local shape-inference patch.

The xquant 1.2.1 pipeline clears ONNX value_info in format_onnx_model().
For YOLOv8 truncation targets, onnx_graphsurgeon then sees tensors without
dtype/shape and refuses to export the truncated graph. This wrapper keeps the
installed xquant package untouched and patches the in-process pipeline to run
ONNX shape inference immediately after xquant formatting.

This script only converts model files. It does not start ROS, publish cmd_vel,
open serial devices, or control robot hardware.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any


DEFAULT_CONFIG = "models/risk_vision/xquant_yolov8n_320/yolov8n_320_xquant_config_v121.json"
DEFAULT_OUTPUT = "models/risk_vision/yolov8n_320_q_truncated.onnx"
TRUNCATE_TARGETS_CV2 = [
    "/model.22/Reshape_output_0",
    "/model.22/Reshape_1_output_0",
    "/model.22/Reshape_2_output_0",
]
TRUNCATE_TARGETS_CV2_CV3 = [
    *TRUNCATE_TARGETS_CV2,
    "/model.22/Reshape_3_output_0",
    "/model.22/Reshape_4_output_0",
    "/model.22/Reshape_5_output_0",
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default=DEFAULT_CONFIG, help="xquant config JSON.")
    parser.add_argument("--output", default=DEFAULT_OUTPUT, help="Output quantized ONNX path.")
    parser.add_argument("--calibration-step", type=int, default=None, help="Override calibration_step for quick tests.")
    parser.add_argument("--calibration-device", default=None, help="Override calibration_device, e.g. cuda or cpu.")
    parser.add_argument("--skip-onnxsim", action="store_true", help="Override config to skip onnxsim.")
    parser.add_argument(
        "--truncate-mode",
        choices=["cv2", "cv2_cv3", "config"],
        default="cv2",
        help="cv2 keeps the original bbox branch targets; cv2_cv3 also truncates class branch Reshapes.",
    )
    return parser.parse_args()


def load_config(path: Path, args: argparse.Namespace) -> dict[str, Any]:
    data = json.loads(path.read_text(encoding="utf-8"))
    data.setdefault("quantization_parameters", {})
    if args.truncate_mode == "cv2":
        data["quantization_parameters"]["truncate_var_names"] = TRUNCATE_TARGETS_CV2
    elif args.truncate_mode == "cv2_cv3":
        data["quantization_parameters"]["truncate_var_names"] = TRUNCATE_TARGETS_CV2_CV3
    if args.calibration_step is not None:
        data.setdefault("calibration_parameters", {})["calibration_step"] = args.calibration_step
    if args.calibration_device is not None:
        data.setdefault("calibration_parameters", {})["calibration_device"] = args.calibration_device
    if args.skip_onnxsim:
        data.setdefault("model_parameters", {})["skip_onnxsim"] = True
    return data


def patch_xquant_shape_inference() -> None:
    import onnx
    import xquant.xquant_pipeline as pipeline

    original_format = pipeline.format_onnx_model

    def patched_format_onnx_model(model, *args, **kwargs):  # type: ignore[no-untyped-def]
        formatted = original_format(model, *args, **kwargs)
        return onnx.shape_inference.infer_shapes(formatted, data_prop=True)

    pipeline.format_onnx_model = patched_format_onnx_model


def main() -> int:
    args = parse_args()
    config_path = Path(args.config)
    output_path = Path(args.output)
    if not config_path.exists():
        raise SystemExit(f"xquant config not found: {config_path}")

    config = load_config(config_path, args)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    patch_xquant_shape_inference()

    from xquant import quantize_onnx_model

    quantize_onnx_model(config, output_path=str(output_path))

    import onnx

    model = onnx.load(str(output_path))
    onnx.checker.check_model(model)
    print(
        json.dumps(
            {
                "status": "ok",
                "output": str(output_path),
                "size_bytes": output_path.stat().st_size,
                "nodes": len(model.graph.node),
                "outputs": [out.name for out in model.graph.output],
                "ros_started": False,
                "cmd_vel_published": False,
                "serial_port_opened": False,
            },
            ensure_ascii=False,
            indent=2,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
