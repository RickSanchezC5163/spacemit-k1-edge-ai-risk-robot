#!/usr/bin/env python3
"""Benchmark a risk-vision backend without ROS, hardware, or online APIs."""

from __future__ import annotations

import argparse
import csv
import json
import platform
import statistics
import sys
from pathlib import Path
from typing import Any, Dict, List

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from src.primitives.risk_vision_primitives import detect_risk
from src.primitives.schemas import read_yaml, write_json, write_text


IMAGE_EXTENSIONS = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}


def image_files(path: Path) -> List[Path]:
    if not path.exists():
        return []
    return sorted(file for file in path.iterdir() if file.is_file() and file.suffix.lower() in IMAGE_EXTENSIONS)


def percentile(values: List[float], pct: float) -> float:
    if not values:
        return 0.0
    if len(values) == 1:
        return values[0]
    rank = (len(values) - 1) * pct
    low = int(rank)
    high = min(low + 1, len(values) - 1)
    fraction = rank - low
    return values[low] * (1.0 - fraction) + values[high] * fraction


def backend_metadata(backend: str) -> Dict[str, Any]:
    cfg = read_yaml(ROOT / "configs" / "risk_detection_backends.yaml")
    return dict(((cfg.get("backends") or {}).get(backend) or {}))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--backend", default="hsv_red_rule")
    parser.add_argument("--model", default=None)
    parser.add_argument("--image-dir", default="assets/risk_print_set_v1/samples")
    parser.add_argument("--output-dir", default="outputs/risk_vision_benchmark_v1/stub")
    args = parser.parse_args()

    out = Path(args.output_dir)
    out.mkdir(parents=True, exist_ok=True)
    images = image_files(Path(args.image_dir))
    predictions = []
    inference_latencies = []
    load_latencies = []
    errors = []

    for image in images:
        try:
            result = detect_risk(
                str(image),
                backend=args.backend,
                output_dir=None,
                model_path_override=args.model,
                allow_disabled_backend=bool(args.model),
            )
            inference_latency = result.get("inference_latency_ms")
            if inference_latency is None:
                inference_latency = result.get("latency_ms")
            load_latency = result.get("load_latency_ms")
            if inference_latency is not None:
                inference_latencies.append(float(inference_latency))
            if load_latency is not None:
                load_latencies.append(float(load_latency))
            predictions.append(
                {
                    "image_path": str(image),
                    "backend": result.get("backend"),
                    "requested_backend": result.get("requested_backend"),
                    "backend_available": result.get("backend_available"),
                    "inference_ready": result.get("inference_ready"),
                    "inference_executed": result.get("inference_executed"),
                    "fallback_used": result.get("fallback_used"),
                    "load_latency_ms": load_latency,
                    "inference_latency_ms": inference_latency,
                    "inference_fps": result.get("inference_fps") or result.get("fps"),
                    "detections": result.get("detections") or [],
                    "claim_boundary": result.get("claim_boundary") or [],
                }
            )
        except Exception as exc:
            errors.append({"image_path": str(image), "error": str(exc)})

    meta = backend_metadata(args.backend)
    inference_latencies_sorted = sorted(inference_latencies)
    load_latencies_sorted = sorted(load_latencies)
    image_count = len(images)
    avg_inference_latency = statistics.mean(inference_latencies) if inference_latencies else 0.0
    avg_load_latency = statistics.mean(load_latencies) if load_latencies else 0.0
    fps = 1000.0 / avg_inference_latency if avg_inference_latency > 0 else 0.0
    model_path = args.model if args.model is not None else meta.get("model_path")
    model_size_mb = 0.0
    if isinstance(model_path, str) and Path(model_path).exists():
        model_size_mb = round(Path(model_path).stat().st_size / (1024.0 * 1024.0), 3)

    summary = {
        "platform_label": "local_dev_or_k1",
        "platform": platform.platform(),
        "hostname": platform.node(),
        "device": platform.node(),
        "system": platform.system(),
        "machine": platform.machine(),
        "backend": args.backend,
        "runtime_backend": meta.get("runtime_backend", "unknown"),
        "precision": meta.get("precision", "unknown"),
        "acceleration_target": meta.get("acceleration_target", "unknown"),
        "model_used": bool(meta.get("model_used") is True and model_path and Path(str(model_path)).exists()),
        "local_inference": bool(meta.get("local_inference") is True),
        "online_api_used": False,
        "model_name": str(model_path or "none"),
        "model_size_mb": model_size_mb,
        "image_dir": args.image_dir,
        "image_count": image_count,
        "avg_latency_ms": round(avg_inference_latency, 3),
        "p50_latency_ms": round(percentile(inference_latencies_sorted, 0.50), 3),
        "p95_latency_ms": round(percentile(inference_latencies_sorted, 0.95), 3),
        "fps": round(fps, 3),
        "avg_inference_latency_ms": round(avg_inference_latency, 3),
        "p50_inference_latency_ms": round(percentile(inference_latencies_sorted, 0.50), 3),
        "p95_inference_latency_ms": round(percentile(inference_latencies_sorted, 0.95), 3),
        "inference_fps": round(fps, 3),
        "avg_load_latency_ms": round(avg_load_latency, 3),
        "p50_load_latency_ms": round(percentile(load_latencies_sorted, 0.50), 3),
        "p95_load_latency_ms": round(percentile(load_latencies_sorted, 0.95), 3),
        "errors": errors,
        "claim_boundary": list(meta.get("claim_boundary") or [])
        + [
            "Benchmark output is platform-specific.",
            "Do not claim K1 performance unless this command ran on K1.",
            "No online API is used.",
        ],
    }

    with (out / "latency.csv").open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(
            handle,
            fieldnames=[
                "image_path",
                "load_latency_ms",
                "inference_latency_ms",
                "inference_fps",
                "backend",
                "fallback_used",
                "inference_executed",
                "detection_count",
            ],
        )
        writer.writeheader()
        for row in predictions:
            writer.writerow(
                {
                    "image_path": row["image_path"],
                    "load_latency_ms": row["load_latency_ms"],
                    "inference_latency_ms": row["inference_latency_ms"],
                    "inference_fps": row["inference_fps"],
                    "backend": row["backend"],
                    "fallback_used": row["fallback_used"],
                    "inference_executed": row["inference_executed"],
                    "detection_count": len(row["detections"]),
                }
            )

    write_json(out / "benchmark_summary.json", summary)
    write_json(out / "predictions.json", predictions)
    write_json(out / "errors.json", errors)
    write_text(
        out / "README.md",
        "# Risk Vision Backend Benchmark\n\n"
        "This benchmark is offline and file-based. It does not start ROS, open cameras, "
        "control hardware, or use online APIs.\n\n"
        f"- backend: `{args.backend}`\n"
        f"- image_count: `{image_count}`\n"
        f"- avg_latency_ms: `{summary['avg_latency_ms']}`\n"
        f"- fps: `{summary['fps']}`\n",
    )
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
