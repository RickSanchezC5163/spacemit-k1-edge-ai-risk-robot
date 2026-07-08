"""YOLO/ONNX backend skeletons for local risk vision.

These helpers are intentionally conservative: missing dependencies or model
files return backend-unavailable metadata instead of fabricated detections.
"""

from __future__ import annotations

import time
from pathlib import Path
from typing import Any, Dict, Sequence


def _model_size_mb(path: Path) -> float:
    try:
        return round(path.stat().st_size / (1024.0 * 1024.0), 3)
    except OSError:
        return 0.0


def unavailable_result(
    backend: str,
    runtime_backend: str,
    precision: str,
    acceleration_target: str,
    model_path: str | None,
    reason: str,
    input_size: Sequence[int] = (640, 640),
) -> Dict[str, Any]:
    return {
        "backend": backend,
        "backend_available": False,
        "inference_ready": False,
        "inference_executed": False,
        "runtime_backend": runtime_backend,
        "precision": precision,
        "acceleration_target": acceleration_target,
        "model_name": str(model_path) if model_path else "none",
        "model_size_mb": 0.0,
        "input_size": list(input_size),
        "load_latency_ms": None,
        "inference_latency_ms": None,
        "inference_fps": None,
        "latency_ms": None,
        "fps": None,
        "detections": [],
        "reason": reason,
        "claim_boundary": [
            "Backend skeleton did not execute model inference.",
            "Do not claim K1 local model performance without a measured benchmark.",
        ],
    }


def run_opencv_dnn_onnx(
    rgb_path: str,
    model_path: str | None,
    input_size: Sequence[int] = (640, 640),
) -> Dict[str, Any]:
    backend = "opencv_dnn_onnx"
    runtime_backend = "opencv_dnn"
    precision = "fp32"
    acceleration_target = "cpu"
    if not model_path:
        return unavailable_result(backend, runtime_backend, precision, acceleration_target, model_path, "model_path missing", input_size)
    model = Path(model_path)
    if not model.exists():
        return unavailable_result(backend, runtime_backend, precision, acceleration_target, model_path, "model file missing", input_size)
    try:
        import cv2  # type: ignore
    except Exception as exc:
        return unavailable_result(backend, runtime_backend, precision, acceleration_target, model_path, f"cv2 unavailable: {exc}", input_size)
    if not Path(rgb_path).exists():
        return unavailable_result(backend, runtime_backend, precision, acceleration_target, model_path, "rgb_path missing", input_size)

    start = time.perf_counter()
    try:
        cv2.dnn.readNetFromONNX(str(model))
    except Exception as exc:
        return unavailable_result(backend, runtime_backend, precision, acceleration_target, model_path, f"ONNX load failed: {exc}", input_size)
    elapsed_ms = max(0.001, (time.perf_counter() - start) * 1000.0)
    return {
        "backend": backend,
        "backend_available": True,
        "inference_ready": False,
        "inference_executed": False,
        "runtime_backend": runtime_backend,
        "precision": precision,
        "acceleration_target": acceleration_target,
        "model_name": str(model_path),
        "model_size_mb": _model_size_mb(model),
        "input_size": list(input_size),
        "load_latency_ms": round(elapsed_ms, 3),
        "inference_latency_ms": None,
        "inference_fps": None,
        "latency_ms": None,
        "fps": None,
        "detections": [],
        "reason": "model load smoke only; inference not executed",
        "claim_boundary": [
            "OpenCV DNN model load succeeded, but detection postprocess is not implemented here.",
            "Do not claim detection accuracy from this skeleton.",
        ],
    }


def run_yolov8n_onnx_cpu(
    rgb_path: str,
    model_path: str | None,
    input_size: Sequence[int] = (640, 640),
) -> Dict[str, Any]:
    backend = "yolov8n_onnx_cpu"
    runtime_backend = "onnxruntime"
    precision = "fp32"
    acceleration_target = "cpu"
    if not model_path:
        return unavailable_result(backend, runtime_backend, precision, acceleration_target, model_path, "model_path missing", input_size)
    model = Path(model_path)
    if not model.exists():
        return unavailable_result(backend, runtime_backend, precision, acceleration_target, model_path, "model file missing", input_size)
    try:
        import onnxruntime as ort  # type: ignore
    except Exception as exc:
        return unavailable_result(backend, runtime_backend, precision, acceleration_target, model_path, f"onnxruntime unavailable: {exc}", input_size)
    if not Path(rgb_path).exists():
        return unavailable_result(backend, runtime_backend, precision, acceleration_target, model_path, "rgb_path missing", input_size)

    start = time.perf_counter()
    try:
        ort.InferenceSession(str(model), providers=["CPUExecutionProvider"])
    except Exception as exc:
        return unavailable_result(backend, runtime_backend, precision, acceleration_target, model_path, f"ONNX session load failed: {exc}", input_size)
    elapsed_ms = max(0.001, (time.perf_counter() - start) * 1000.0)
    return {
        "backend": backend,
        "backend_available": True,
        "inference_ready": False,
        "inference_executed": False,
        "runtime_backend": runtime_backend,
        "precision": precision,
        "acceleration_target": acceleration_target,
        "model_name": str(model_path),
        "model_size_mb": _model_size_mb(model),
        "input_size": list(input_size),
        "load_latency_ms": round(elapsed_ms, 3),
        "inference_latency_ms": None,
        "inference_fps": None,
        "latency_ms": None,
        "fps": None,
        "detections": [],
        "reason": "session load smoke only; inference not executed",
        "claim_boundary": [
            "ONNX Runtime session load succeeded, but detection postprocess is not implemented here.",
            "Do not claim K1 YOLO inference until measured on K1.",
        ],
    }
