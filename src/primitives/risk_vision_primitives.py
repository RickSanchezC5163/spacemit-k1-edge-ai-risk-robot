"""Risk vision backend interface.

The HSV red-rule backend is kept as a deterministic baseline. Local model
backends are interface placeholders until real K1 model files and benchmarks
exist. This module is offline/file-based and does not use ROS, serial ports, or
online APIs.
"""

from __future__ import annotations

import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence

from .d435_depth_localization import localize_bbox_with_depth
from .ncnn_backend import run_ncnn_int8
from .schemas import now_iso, read_yaml, write_json
from .yolo_backend import run_opencv_dnn_onnx, run_yolov8n_onnx_cpu


ROOT = Path(__file__).resolve().parents[2]
DEFAULT_BACKENDS = ROOT / "configs" / "risk_detection_backends.yaml"


def _load_rgb(path: Path) -> tuple[int, int, list[tuple[int, int, int]]]:
    from PIL import Image

    img = Image.open(path).convert("RGB")
    pixels = list(img.getdata())
    return img.width, img.height, pixels


def _red_bbox_from_rgb(path: Path) -> tuple[list[int] | None, int, int]:
    width, height, pixels = _load_rgb(path)
    xs: List[int] = []
    ys: List[int] = []
    for idx, (r, g, b) in enumerate(pixels):
        # Conservative red rule: strong red channel and clear separation.
        if r >= 110 and r >= g * 1.45 and r >= b * 1.45 and (r - max(g, b)) >= 35:
            y, x = divmod(idx, width)
            xs.append(x)
            ys.append(y)
    if not xs:
        return None, width, height
    x0, x1 = min(xs), max(xs)
    y0, y1 = min(ys), max(ys)
    return [x0, y0, x1 - x0 + 1, y1 - y0 + 1], width, height


def _path_exists(model_path: Any) -> bool:
    if not model_path:
        return False
    if isinstance(model_path, dict):
        values = [value for value in model_path.values() if value]
        return bool(values) and all((ROOT / str(value)).exists() for value in values)
    return (ROOT / str(model_path)).exists()


def _resolve_model_path(model_path: Any) -> Any:
    if isinstance(model_path, dict):
        return {key: str(ROOT / str(value)) for key, value in model_path.items() if value}
    if isinstance(model_path, str):
        return str(ROOT / model_path)
    return model_path


def _model_size_mb(model_path: Any) -> float:
    try:
        if isinstance(model_path, dict):
            total = sum((ROOT / str(value)).stat().st_size for value in model_path.values() if value)
            return round(total / (1024.0 * 1024.0), 3)
        if isinstance(model_path, str):
            return round((ROOT / model_path).stat().st_size / (1024.0 * 1024.0), 3)
    except OSError:
        return 0.0
    return 0.0


def _backend_info(config: Dict[str, Any], backend: str) -> Dict[str, Any]:
    return dict(((config.get("backends") or {}).get(backend) or {}))


def _backend_enabled(info: Dict[str, Any]) -> bool:
    return info.get("enabled", info.get("available")) is True


def _model_backend_available(info: Dict[str, Any], allow_disabled_backend: bool = False) -> bool:
    if not allow_disabled_backend and not _backend_enabled(info):
        return False
    if info.get("model_used") is not True:
        return info.get("available", True) is True
    return _path_exists(info.get("model_path"))


def _stub_detection(input_size: Sequence[int] = (640, 480), class_name: str = "printed_risk") -> Dict[str, Any]:
    width, height = int(input_size[0]), int(input_size[1])
    bbox = [int(width * 0.38), int(height * 0.32), int(width * 0.24), int(height * 0.22)]
    return {
        "class_name": class_name,
        "confidence": 0.5,
        "bbox_xywh": bbox,
        "depth_median_m": None,
        "bbox_valid_depth_ratio": None,
        "camera_point_xyz_m": None,
        "detection_source": "stub",
    }


def _apply_depth_localization(
    detections: List[Dict[str, Any]],
    depth_path: str | None,
    camera_info_path: str | None,
) -> Dict[str, Any]:
    status = {
        "depth_localization_attempted": bool(depth_path and camera_info_path),
        "depth_localization_applied_count": 0,
        "depth_localization_status": "not_requested",
    }
    if not depth_path or not camera_info_path:
        return status
    status["depth_localization_status"] = "no_detections" if not detections else "attempted"
    for detection in detections:
        bbox = detection.get("bbox_xywh")
        if not isinstance(bbox, list) or len(bbox) < 4:
            continue
        localized = localize_bbox_with_depth(bbox, depth_path, camera_info_path)
        detection["depth_localization"] = localized
        if localized.get("depth_status") == "valid":
            detection["depth_median_m"] = localized.get("depth_median_m")
            detection["bbox_valid_depth_ratio"] = localized.get("bbox_valid_depth_ratio")
            detection["camera_point_xyz_m"] = localized.get("camera_point_xyz_m")
            status["depth_localization_applied_count"] += 1
    if status["depth_localization_applied_count"]:
        status["depth_localization_status"] = "valid"
    elif detections:
        status["depth_localization_status"] = "missing_or_invalid"
    return status


def _run_model_backend(backend: str, rgb_path: str, info: Dict[str, Any]) -> Dict[str, Any]:
    model_path = _resolve_model_path(info.get("model_path"))
    input_size = info.get("input_size") or [640, 640]
    if backend == "opencv_dnn_onnx":
        return run_opencv_dnn_onnx(rgb_path, model_path, input_size)
    if backend == "yolov8n_onnx_cpu":
        return run_yolov8n_onnx_cpu(rgb_path, model_path, input_size)
    if backend == "ncnn_int8":
        return run_ncnn_int8(rgb_path, model_path, input_size)
    return {
        "backend": backend,
        "backend_available": False,
        "inference_executed": False,
        "detections": [],
        "reason": f"backend {backend} has no adapter",
    }


def _claim_boundary_for(info: Dict[str, Any], fallback_used: bool, requested_backend: str, backend: str) -> List[str]:
    claims = list(info.get("claim_boundary") or [])
    claims.extend(
        [
            "No online API is used.",
            "Do not claim NPU acceleration without measured K1 backend evidence.",
        ]
    )
    if fallback_used:
        claims.append(
            f"Requested backend {requested_backend} was unavailable; fallback {backend} is not proof of real AI model inference."
        )
    return claims


def detect_risk(
    rgb_path: str,
    depth_path: str | None = None,
    camera_info_path: str | None = None,
    backend: str = "hsv_red_rule",
    output_dir: str | None = None,
    model_path_override: str | None = None,
    allow_disabled_backend: bool = False,
) -> Dict[str, Any]:
    start = time.perf_counter()
    config = read_yaml(DEFAULT_BACKENDS)
    requested_backend = backend
    requested_info = _backend_info(config, requested_backend)
    if model_path_override and requested_info.get("model_used") is True:
        requested_info["model_path"] = model_path_override
    fallback_used = False
    fallback_backend: Optional[str] = None
    requested_backend_available = bool(requested_info) and _model_backend_available(
        requested_info,
        allow_disabled_backend=allow_disabled_backend,
    )

    if backend not in (config.get("backends") or {}):
        fallback_used = True
        fallback_backend = config.get("fallback_backend", "hsv_red_rule")
        backend = fallback_backend
    elif not requested_backend_available:
        fallback_used = True
        fallback_backend = config.get("fallback_backend", "hsv_red_rule")
        backend = fallback_backend

    info = _backend_info(config, backend)
    if model_path_override and backend == requested_backend and info.get("model_used") is True:
        info["model_path"] = model_path_override
    rgb = Path(rgb_path) if rgb_path else Path("")
    width, height = (info.get("input_size") or [640, 480])[:2]
    detections: List[Dict[str, Any]] = []
    backend_reason: Optional[str] = None
    model_result: Dict[str, Any] = {}

    if backend == "hsv_red_rule":
        if rgb.is_file():
            bbox, width, height = _red_bbox_from_rgb(rgb)
            if bbox:
                detections.append(
                    {
                        "class_name": "red_object_rule",
                        "confidence": 1.0,
                        "bbox_xywh": bbox,
                        "depth_median_m": None,
                        "bbox_valid_depth_ratio": None,
                        "camera_point_xyz_m": None,
                        "detection_source": "hsv_red_rule",
                    }
                )
        else:
            backend_reason = "rgb_path missing; hsv baseline produced no detection"
    elif backend == "stub_local_model":
        detections.append(_stub_detection(info.get("input_size") or [640, 480]))
    else:
        model_result = _run_model_backend(backend, str(rgb), info)
        detections = list(model_result.get("detections") or [])
        backend_reason = model_result.get("reason")

    depth_status = _apply_depth_localization(detections, depth_path, camera_info_path)

    elapsed_ms = max(0.001, (time.perf_counter() - start) * 1000.0)
    model_used = bool(info.get("model_used") is True and not fallback_used and requested_backend_available)
    backend_available = bool(
        (
            model_result.get("backend_available")
            if model_result
            else _model_backend_available(info, allow_disabled_backend=allow_disabled_backend)
        )
        and not fallback_used
    )
    result = {
        "risk_detection_id": f"risk_detection_{int(time.time())}",
        "timestamp": now_iso(),
        "requested_backend": requested_backend,
        "requested_backend_available": requested_backend_available,
        "backend": backend,
        "backend_available": backend_available,
        "fallback_used": fallback_used,
        "fallback_backend": fallback_backend,
        "detection_mode": "hsv_rule_based_red_color" if backend == "hsv_red_rule" else backend,
        "model_used": model_used,
        "local_inference": bool(info.get("local_inference") is True),
        "online_api_used": False,
        "runtime_backend": info.get("runtime_backend", "unknown"),
        "precision": info.get("precision", "unknown"),
        "acceleration_target": info.get("acceleration_target", "unknown"),
        "inference_ready": bool(model_result.get("inference_ready") is True) if model_result else backend in {"hsv_red_rule", "stub_local_model"},
        "inference_executed": bool(model_result.get("inference_executed") is True) if model_result else backend in {"hsv_red_rule", "stub_local_model"},
        "model_name": str(info.get("model_path") or "none"),
        "model_size_mb": _model_size_mb(info.get("model_path")) if model_used else 0,
        "input_size": [int(width), int(height)],
        "load_latency_ms": model_result.get("load_latency_ms"),
        "inference_latency_ms": model_result.get("inference_latency_ms") if model_result else round(elapsed_ms, 3),
        "inference_fps": model_result.get("inference_fps") if model_result else round(1000.0 / elapsed_ms, 3),
        "latency_ms": model_result.get("latency_ms") if model_result else round(elapsed_ms, 3),
        "fps": model_result.get("fps") if model_result else round(1000.0 / elapsed_ms, 3),
        "rgb_path": str(rgb_path) if rgb_path else None,
        "depth_path": depth_path,
        "camera_info_path": camera_info_path,
        **depth_status,
        "detections": detections,
        "backend_reason": backend_reason,
        "claim_boundary": _claim_boundary_for(info, fallback_used, requested_backend, backend),
    }
    if output_dir:
        out = Path(output_dir)
        write_json(out / "risk_detection.json", result)
        write_json(out / "errors.json", [])
    return result
