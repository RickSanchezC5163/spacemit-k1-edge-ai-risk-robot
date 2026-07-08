"""ncnn int8 backend skeleton for future K1/RVV local inference."""

from __future__ import annotations

from pathlib import Path
from typing import Any, Dict, Mapping, Sequence


def _missing(reason: str, model_path: Any, input_size: Sequence[int]) -> Dict[str, Any]:
    return {
        "backend": "ncnn_int8",
        "backend_available": False,
        "inference_ready": False,
        "inference_executed": False,
        "runtime_backend": "ncnn",
        "precision": "int8",
        "acceleration_target": "riscv_rvv",
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
            "ncnn int8 backend is a deployment candidate only.",
            "Do not claim RVV or NPU acceleration without K1 benchmark evidence.",
        ],
    }


def _paths(model_path: Any) -> tuple[Path | None, Path | None]:
    if isinstance(model_path, Mapping):
        param = model_path.get("param")
        bin_path = model_path.get("bin")
        return Path(param) if param else None, Path(bin_path) if bin_path else None
    if isinstance(model_path, str):
        path = Path(model_path)
        if path.suffix == ".param":
            return path, path.with_suffix(".bin")
    return None, None


def run_ncnn_int8(
    rgb_path: str,
    model_path: Any,
    input_size: Sequence[int] = (640, 640),
) -> Dict[str, Any]:
    param_path, bin_path = _paths(model_path)
    if param_path is None or bin_path is None:
        return _missing("ncnn param/bin model paths missing", model_path, input_size)
    if not param_path.exists() or not bin_path.exists():
        return _missing("ncnn model files missing", model_path, input_size)
    try:
        import ncnn  # type: ignore  # noqa: F401
    except Exception as exc:
        return _missing(f"ncnn python binding unavailable: {exc}", model_path, input_size)
    if not Path(rgb_path).exists():
        return _missing("rgb_path missing", model_path, input_size)
    return {
        "backend": "ncnn_int8",
        "backend_available": True,
        "inference_ready": False,
        "inference_executed": False,
        "runtime_backend": "ncnn",
        "precision": "int8",
        "acceleration_target": "riscv_rvv",
        "model_name": f"{param_path};{bin_path}",
        "model_size_mb": round((param_path.stat().st_size + bin_path.stat().st_size) / (1024.0 * 1024.0), 3),
        "input_size": list(input_size),
        "load_latency_ms": None,
        "inference_latency_ms": None,
        "inference_fps": None,
        "latency_ms": None,
        "fps": None,
        "detections": [],
        "reason": "ncnn model files and binding present; inference adapter not enabled",
        "claim_boundary": [
            "ncnn files are present but this skeleton did not execute detector postprocess.",
            "K1 latency/FPS/CPU/memory must be measured before acceleration claims.",
        ],
    }
