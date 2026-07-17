#!/usr/bin/env python3
"""Benchmark YOLO ONNX Runtime CPU thread settings on K1.

This benchmark intentionally uses one fixed frame so the result isolates model
preprocess/inference/postprocess cost instead of camera motion or ROS timing.
It does not start ROS, publish cmd_vel, or control hardware.
"""

from __future__ import annotations

import argparse
import csv
import json
import statistics
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Sequence, Tuple


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from tools.run_k1_d435_yolo_realtime_display import (  # noqa: E402
    build_gstreamer_pipeline,
    preprocess_bgr,
    resolve_model_input_size,
)
from tools.run_yolo_inference_once import postprocess  # noqa: E402


DEFAULT_MODEL = "models/risk_vision/yolov8n_480x640_q_truncated6_balanced_blockage03.onnx"
DEFAULT_OUTPUT_DIR = "outputs/k1_yolo_cpu_ort_thread_benchmark"


def now_id() -> str:
    return datetime.now().strftime("%Y%m%d_%H%M%S")


def now_iso() -> str:
    return datetime.now().isoformat(timespec="milliseconds")


def write_json(path: Path, data: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def percentile(values: Sequence[float], pct: float) -> float | None:
    if not values:
        return None
    ordered = sorted(float(v) for v in values)
    idx = int(round((pct / 100.0) * (len(ordered) - 1)))
    idx = max(0, min(len(ordered) - 1, idx))
    return ordered[idx]


def summarize(values: Sequence[float]) -> Dict[str, Any]:
    if not values:
        return {
            "count": 0,
            "mean_ms": None,
            "median_ms": None,
            "p90_ms": None,
            "p95_ms": None,
            "min_ms": None,
            "max_ms": None,
            "fps_from_mean": None,
            "fps_from_median": None,
        }
    mean_ms = float(statistics.mean(values))
    median_ms = float(statistics.median(values))
    return {
        "count": len(values),
        "mean_ms": round(mean_ms, 3),
        "median_ms": round(median_ms, 3),
        "p90_ms": round(float(percentile(values, 90) or 0.0), 3),
        "p95_ms": round(float(percentile(values, 95) or 0.0), 3),
        "min_ms": round(float(min(values)), 3),
        "max_ms": round(float(max(values)), 3),
        "fps_from_mean": round(1000.0 / mean_ms, 3) if mean_ms > 0 else None,
        "fps_from_median": round(1000.0 / median_ms, 3) if median_ms > 0 else None,
    }


def parse_threads(spec: str) -> List[int]:
    values: List[int] = []
    for item in str(spec).split(","):
        item = item.strip()
        if not item:
            continue
        value = int(item)
        if value <= 0:
            raise ValueError(f"thread count must be positive: {value}")
        values.append(value)
    if not values:
        raise ValueError("empty thread list")
    return values


def capture_or_load_frame(args: argparse.Namespace) -> Any:
    import cv2

    if args.image:
        frame = cv2.imread(str(Path(args.image)), cv2.IMREAD_COLOR)
        if frame is None:
            raise SystemExit(f"failed to read image: {args.image}")
        return frame

    pipeline = build_gstreamer_pipeline(args)
    print(f"Opening D435 stream: {pipeline}", flush=True)
    cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
    if not cap.isOpened():
        raise SystemExit(f"failed to open D435 stream: {pipeline}")
    try:
        for _ in range(max(0, int(args.camera_warmup_frames))):
            cap.read()
        ok, frame = cap.read()
        if not ok or frame is None:
            raise SystemExit("failed to capture benchmark frame")
        return frame
    finally:
        cap.release()


def make_session(model_path: Path, intra_threads: int, args: argparse.Namespace) -> Tuple[Any, str, str, Tuple[int, int], List[Any]]:
    import onnxruntime as ort

    options = ort.SessionOptions()
    options.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
    options.execution_mode = ort.ExecutionMode.ORT_SEQUENTIAL
    options.inter_op_num_threads = int(args.inter_op_threads)
    options.intra_op_num_threads = int(intra_threads)
    if args.allow_spinning != "default":
        options.add_session_config_entry("session.intra_op.allow_spinning", str(args.allow_spinning))
    session = ort.InferenceSession(str(model_path), sess_options=options, providers=["CPUExecutionProvider"])
    input_name = session.get_inputs()[0].name
    output_name = session.get_outputs()[0].name
    input_size_hw = resolve_model_input_size(session, args.imgsz)
    input_shape = session.get_inputs()[0].shape
    return session, input_name, output_name, input_size_hw, input_shape


def run_one(
    frame_bgr: Any,
    session: Any,
    input_name: str,
    output_name: str,
    input_size_hw: Tuple[int, int],
    args: argparse.Namespace,
) -> Dict[str, Any]:
    start = time.perf_counter()
    tensor, ratio, pad = preprocess_bgr(frame_bgr, input_size_hw)
    preprocess_ms = (time.perf_counter() - start) * 1000.0

    start = time.perf_counter()
    output = session.run([output_name], {input_name: tensor})[0]
    inference_ms = (time.perf_counter() - start) * 1000.0

    start = time.perf_counter()
    detections = postprocess(
        output,
        image_shape=frame_bgr.shape[:2],
        ratio=ratio,
        pad=pad,
        conf_thres=args.conf,
        iou_thres=args.iou,
        max_det=args.max_det,
    )
    postprocess_ms = (time.perf_counter() - start) * 1000.0
    return {
        "preprocess_ms": preprocess_ms,
        "inference_ms": inference_ms,
        "postprocess_ms": postprocess_ms,
        "e2e_compute_ms": preprocess_ms + inference_ms + postprocess_ms,
        "detection_count": len(detections),
        "detections": [
            {
                "class_name": det.get("class_name"),
                "confidence": det.get("confidence"),
                "bbox_xywh": det.get("bbox_xywh"),
            }
            for det in detections[: args.max_det]
        ],
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", default=DEFAULT_MODEL)
    parser.add_argument("--image", default="", help="Optional fixed image. If omitted, capture one D435 V4L2 frame.")
    parser.add_argument("--output-dir", default=DEFAULT_OUTPUT_DIR)
    parser.add_argument("--threads", default="2,3,4,5,6,7")
    parser.add_argument("--inter-op-threads", type=int, default=1)
    parser.add_argument("--allow-spinning", choices=["default", "0", "1"], default="1")
    parser.add_argument("--opencv-num-threads", type=int, default=1)
    parser.add_argument("--warmup", type=int, default=20)
    parser.add_argument("--frames", type=int, default=80)
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--conf", type=float, default=0.15)
    parser.add_argument("--iou", type=float, default=0.45)
    parser.add_argument("--max-det", type=int, default=10)
    parser.add_argument("--video-device", default="/dev/video24")
    parser.add_argument("--pixel-format", default="YUY2")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--camera-warmup-frames", type=int, default=30)
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    import cv2
    import onnxruntime as ort

    cv2.setNumThreads(int(args.opencv_num_threads))
    output_dir = Path(args.output_dir) / now_id()
    output_dir.mkdir(parents=True, exist_ok=True)

    model_path = Path(args.model)
    if not model_path.exists():
        raise SystemExit(f"model not found: {model_path}")

    frame_bgr = capture_or_load_frame(args)
    cv2.imwrite(str(output_dir / "benchmark_frame.png"), frame_bgr)

    threads = parse_threads(args.threads)
    all_records: List[Dict[str, Any]] = []
    summaries: List[Dict[str, Any]] = []
    print(f"ONNX Runtime: {ort.__version__} from {ort.__file__}", flush=True)
    print(f"Benchmark frame: {frame_bgr.shape[1]}x{frame_bgr.shape[0]}", flush=True)

    for intra in threads:
        print(f"\n=== intra_op_num_threads={intra} ===", flush=True)
        session, input_name, output_name, input_size_hw, input_shape = make_session(model_path, intra, args)
        print(f"providers={session.get_providers()} input_shape={input_shape} input_size_hw={input_size_hw}", flush=True)

        for _ in range(max(0, int(args.warmup))):
            run_one(frame_bgr, session, input_name, output_name, input_size_hw, args)

        records: List[Dict[str, Any]] = []
        for idx in range(max(1, int(args.frames))):
            timing = run_one(frame_bgr, session, input_name, output_name, input_size_hw, args)
            record = {
                "timestamp": now_iso(),
                "intra_op_threads": intra,
                "frame_index": idx + 1,
                **{k: round(v, 3) if isinstance(v, float) else v for k, v in timing.items()},
            }
            records.append(record)
            all_records.append(record)

        summary = {
            "intra_op_threads": intra,
            "inter_op_threads": args.inter_op_threads,
            "allow_spinning": args.allow_spinning,
            "preprocess": summarize([float(r["preprocess_ms"]) for r in records]),
            "inference": summarize([float(r["inference_ms"]) for r in records]),
            "postprocess": summarize([float(r["postprocess_ms"]) for r in records]),
            "e2e_compute": summarize([float(r["e2e_compute_ms"]) for r in records]),
            "last_detections": records[-1].get("detections", []),
        }
        summaries.append(summary)
        print(json.dumps(summary, ensure_ascii=False, indent=2), flush=True)

    best = min(
        summaries,
        key=lambda item: float(item["e2e_compute"]["median_ms"] or 1e18),
    )
    result = {
        "schema_version": "k1_yolo_cpu_ort_thread_benchmark_v1",
        "created_at": now_iso(),
        "model": str(model_path),
        "image": str(args.image) if args.image else None,
        "benchmark_frame": str(output_dir / "benchmark_frame.png"),
        "opencv_num_threads": args.opencv_num_threads,
        "inter_op_threads": args.inter_op_threads,
        "warmup": args.warmup,
        "frames": args.frames,
        "threads_tested": threads,
        "best_by_e2e_median": best,
        "summaries": summaries,
    }
    write_json(output_dir / "summary.json", result)
    csv_path = output_dir / "records.csv"
    with csv_path.open("w", encoding="utf-8", newline="") as handle:
        fieldnames = [
            "timestamp",
            "intra_op_threads",
            "frame_index",
            "preprocess_ms",
            "inference_ms",
            "postprocess_ms",
            "e2e_compute_ms",
            "detection_count",
            "detections",
        ]
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(all_records)
    print("\nBEST", json.dumps(best, ensure_ascii=False), flush=True)
    print(f"summary={output_dir / 'summary.json'}", flush=True)
    print(f"records={csv_path}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
