#!/usr/bin/env python3
"""Benchmark K1 D435 YOLO stream inference against PNG file roundtrip.

This is a local vision benchmark only. It does not start ROS, publish cmd_vel,
open serial devices, or control the chassis/arm.
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
from typing import Any, Dict, List, Tuple


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from tools.run_k1_d435_yolo_realtime_display import (  # noqa: E402
    build_gstreamer_pipeline,
    preprocess_bgr,
    resolve_model_input_size,
    select_ort_providers,
)
from tools.run_yolo_inference_once import postprocess  # noqa: E402


DEFAULT_MODEL = "models/risk_vision/yolov8n_480x640_q_truncated6_balanced_blockage03.onnx"
DEFAULT_OUTPUT_DIR = "outputs/k1_d435_yolo_realtime_v1/stream_vs_file_benchmark"


def now_id() -> str:
    return datetime.now().strftime("%Y%m%d_%H%M%S")


def now_iso() -> str:
    return datetime.now().isoformat(timespec="milliseconds")


def write_json(path: Path, data: Dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def percentile(values: List[float], pct: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    idx = min(len(ordered) - 1, max(0, int(round((pct / 100.0) * (len(ordered) - 1)))))
    return ordered[idx]


def summarize(values: List[float]) -> Dict[str, Any]:
    if not values:
        return {"count": 0, "avg_ms": None, "p50_ms": None, "p95_ms": None, "min_ms": None, "max_ms": None}
    return {
        "count": len(values),
        "avg_ms": round(float(statistics.mean(values)), 3),
        "p50_ms": round(float(percentile(values, 50) or 0.0), 3),
        "p95_ms": round(float(percentile(values, 95) or 0.0), 3),
        "min_ms": round(float(min(values)), 3),
        "max_ms": round(float(max(values)), 3),
    }


def infer_frame(
    frame_bgr: Any,
    session: Any,
    input_name: str,
    output_name: str,
    input_size_hw: Tuple[int, int],
    args: argparse.Namespace,
) -> Tuple[List[Dict[str, Any]], Dict[str, float]]:
    start_pre = time.perf_counter()
    tensor, ratio, pad = preprocess_bgr(frame_bgr, input_size_hw)
    pre_ms = (time.perf_counter() - start_pre) * 1000.0

    start_infer = time.perf_counter()
    output = session.run([output_name], {input_name: tensor})[0]
    infer_ms = (time.perf_counter() - start_infer) * 1000.0

    start_post = time.perf_counter()
    detections = postprocess(
        output,
        image_shape=frame_bgr.shape[:2],
        ratio=ratio,
        pad=pad,
        conf_thres=args.conf,
        iou_thres=args.iou,
        max_det=args.max_det,
    )
    post_ms = (time.perf_counter() - start_post) * 1000.0
    return detections, {
        "preprocess_ms": pre_ms,
        "inference_ms": infer_ms,
        "postprocess_ms": post_ms,
        "pipeline_compute_ms": pre_ms + infer_ms + post_ms,
    }


def detect_text(detections: List[Dict[str, Any]]) -> str:
    if not detections:
        return "none"
    return "; ".join(
        f"{det.get('class_name')}:{float(det.get('confidence', 0.0)):.2f}"
        for det in detections[:8]
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", default=DEFAULT_MODEL)
    parser.add_argument("--output-dir", default=DEFAULT_OUTPUT_DIR)
    parser.add_argument("--provider", choices=["auto", "spacemit", "cpu"], default="spacemit")
    parser.add_argument("--video-device", default="/dev/video24")
    parser.add_argument("--pixel-format", default="YUY2")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=15)
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--conf", type=float, default=0.15)
    parser.add_argument("--iou", type=float, default=0.45)
    parser.add_argument("--max-det", type=int, default=10)
    parser.add_argument("--warmup-frames", type=int, default=90)
    parser.add_argument("--frames", type=int, default=120)
    parser.add_argument(
        "--mode",
        choices=["stream", "file", "both"],
        default="both",
        help="stream=in-memory only, file=PNG write+read+infer, both=run both per captured frame.",
    )
    parser.add_argument("--print-period", type=int, default=10)
    parser.add_argument("--keep-temp-png", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    import cv2
    import onnxruntime as ort

    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    temp_dir = output_dir / "temp_png"
    temp_dir.mkdir(parents=True, exist_ok=True)

    providers = select_ort_providers(args.provider)
    print(f"Loading ONNX model: {args.model}", flush=True)
    print(f"ONNX Runtime version: {ort.__version__} from {Path(ort.__file__).as_posix()}", flush=True)
    print(f"Using ONNX Runtime providers: {providers}", flush=True)
    session = ort.InferenceSession(str(Path(args.model)), providers=providers)
    active_providers = session.get_providers()
    input_name = session.get_inputs()[0].name
    output_name = session.get_outputs()[0].name
    input_shape = session.get_inputs()[0].shape
    input_size_hw = resolve_model_input_size(session, args.imgsz)
    print(f"Active providers: {active_providers}", flush=True)
    print(f"Model input shape: {input_shape}, using input_size_hw={input_size_hw}", flush=True)

    pipeline = build_gstreamer_pipeline(args)
    print(f"Opening D435 stream: {pipeline}", flush=True)
    cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
    if not cap.isOpened():
        raise SystemExit(f"Failed to open D435 stream: {pipeline}")

    records: List[Dict[str, Any]] = []
    print(f"Warming up camera: discarding {args.warmup_frames} frames...", flush=True)
    for _ in range(max(0, args.warmup_frames)):
        cap.read()
    print("Camera warmup complete.", flush=True)

    try:
        frame_idx = 0
        while frame_idx < args.frames:
            read_start = time.perf_counter()
            ok, frame_bgr = cap.read()
            capture_read_ms = (time.perf_counter() - read_start) * 1000.0
            if not ok or frame_bgr is None:
                records.append(
                    {
                        "frame_index": frame_idx + 1,
                        "captured_at": now_iso(),
                        "mode": "read_failed",
                        "capture_read_ms": round(capture_read_ms, 3),
                        "error": "frame_read_failed",
                    }
                )
                continue
            frame_idx += 1

            if args.mode in ("stream", "both"):
                e2e_start = time.perf_counter()
                detections, timing = infer_frame(frame_bgr, session, input_name, output_name, input_size_hw, args)
                e2e_ms = (time.perf_counter() - e2e_start) * 1000.0
                records.append(
                    {
                        "frame_index": frame_idx,
                        "captured_at": now_iso(),
                        "mode": "stream_in_memory",
                        "capture_read_ms": round(capture_read_ms, 3),
                        **{k: round(v, 3) for k, v in timing.items()},
                        "io_write_ms": 0.0,
                        "io_read_ms": 0.0,
                        "e2e_ms": round(e2e_ms, 3),
                        "infer_fps": round(1000.0 / max(1e-6, timing["inference_ms"]), 3),
                        "detection_count": len(detections),
                        "detections": detect_text(detections),
                        "error": "",
                    }
                )

            if args.mode in ("file", "both"):
                png_path = temp_dir / f"frame_{frame_idx:04d}.png"
                e2e_start = time.perf_counter()
                write_start = time.perf_counter()
                ok_write = cv2.imwrite(str(png_path), frame_bgr)
                io_write_ms = (time.perf_counter() - write_start) * 1000.0
                read_png_start = time.perf_counter()
                file_bgr = cv2.imread(str(png_path), cv2.IMREAD_COLOR)
                io_read_ms = (time.perf_counter() - read_png_start) * 1000.0
                if not ok_write or file_bgr is None:
                    records.append(
                        {
                            "frame_index": frame_idx,
                            "captured_at": now_iso(),
                            "mode": "png_file_roundtrip",
                            "capture_read_ms": round(capture_read_ms, 3),
                            "io_write_ms": round(io_write_ms, 3),
                            "io_read_ms": round(io_read_ms, 3),
                            "error": "png_write_or_read_failed",
                        }
                    )
                else:
                    detections, timing = infer_frame(file_bgr, session, input_name, output_name, input_size_hw, args)
                    e2e_ms = (time.perf_counter() - e2e_start) * 1000.0
                    records.append(
                        {
                            "frame_index": frame_idx,
                            "captured_at": now_iso(),
                            "mode": "png_file_roundtrip",
                            "capture_read_ms": round(capture_read_ms, 3),
                            **{k: round(v, 3) for k, v in timing.items()},
                            "io_write_ms": round(io_write_ms, 3),
                            "io_read_ms": round(io_read_ms, 3),
                            "e2e_ms": round(e2e_ms, 3),
                            "infer_fps": round(1000.0 / max(1e-6, timing["inference_ms"]), 3),
                            "detection_count": len(detections),
                            "detections": detect_text(detections),
                            "error": "",
                        }
                    )
                if not args.keep_temp_png:
                    png_path.unlink(missing_ok=True)

            if args.print_period > 0 and frame_idx % args.print_period == 0:
                recent = [r for r in records[-2:] if not r.get("error")]
                text = " | ".join(
                    f"{r['mode']}: infer={r.get('inference_ms')}ms e2e={r.get('e2e_ms')}ms det={r.get('detection_count')}"
                    for r in recent
                )
                print(f"frame={frame_idx}/{args.frames} {text}", flush=True)
    finally:
        cap.release()

    csv_path = output_dir / "frame_benchmark.csv"
    fieldnames = [
        "frame_index",
        "captured_at",
        "mode",
        "capture_read_ms",
        "preprocess_ms",
        "inference_ms",
        "postprocess_ms",
        "pipeline_compute_ms",
        "io_write_ms",
        "io_read_ms",
        "e2e_ms",
        "infer_fps",
        "detection_count",
        "detections",
        "error",
    ]
    with csv_path.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(records)

    by_mode: Dict[str, List[Dict[str, Any]]] = {}
    for record in records:
        if record.get("error"):
            continue
        by_mode.setdefault(record["mode"], []).append(record)

    modes_summary: Dict[str, Any] = {}
    for mode, rows in by_mode.items():
        modes_summary[mode] = {
            "frames": len(rows),
            "capture_read_ms": summarize([float(r["capture_read_ms"]) for r in rows if r.get("capture_read_ms") is not None]),
            "preprocess_ms": summarize([float(r["preprocess_ms"]) for r in rows if r.get("preprocess_ms") is not None]),
            "inference_ms": summarize([float(r["inference_ms"]) for r in rows if r.get("inference_ms") is not None]),
            "postprocess_ms": summarize([float(r["postprocess_ms"]) for r in rows if r.get("postprocess_ms") is not None]),
            "pipeline_compute_ms": summarize([float(r["pipeline_compute_ms"]) for r in rows if r.get("pipeline_compute_ms") is not None]),
            "io_write_ms": summarize([float(r["io_write_ms"]) for r in rows if r.get("io_write_ms") is not None]),
            "io_read_ms": summarize([float(r["io_read_ms"]) for r in rows if r.get("io_read_ms") is not None]),
            "e2e_ms": summarize([float(r["e2e_ms"]) for r in rows if r.get("e2e_ms") is not None]),
        }
        avg_e2e = modes_summary[mode]["e2e_ms"]["avg_ms"]
        avg_infer = modes_summary[mode]["inference_ms"]["avg_ms"]
        modes_summary[mode]["e2e_fps_from_avg_ms"] = None if not avg_e2e else round(1000.0 / float(avg_e2e), 3)
        modes_summary[mode]["infer_fps_from_avg_ms"] = None if not avg_infer else round(1000.0 / float(avg_infer), 3)

    summary = {
        "benchmark_id": f"k1_d435_yolo_stream_vs_file_{now_id()}",
        "created_at": now_iso(),
        "script": "tools/benchmark_k1_d435_yolo_stream_vs_file.py",
        "model": str(args.model),
        "provider_requested": args.provider,
        "providers_active": active_providers,
        "model_input_shape": list(input_shape),
        "model_input_size_hw": list(input_size_hw),
        "video_device": args.video_device,
        "width": args.width,
        "height": args.height,
        "fps_requested": args.fps,
        "warmup_frames": args.warmup_frames,
        "frames_requested": args.frames,
        "mode_requested": args.mode,
        "modes": modes_summary,
        "record_count": len(records),
        "error_count": sum(1 for r in records if r.get("error")),
        "claim_boundary": [
            "This benchmark measures local K1 D435 frame ingestion and YOLO ONNX inference only.",
            "No ROS process is started by this script.",
            "No cmd_vel is published.",
            "No serial device or mechanical arm is controlled.",
            "PNG file roundtrip is included only to quantify IO/codec overhead; final runtime should use stream_in_memory.",
        ],
    }
    write_json(output_dir / "benchmark_summary.json", summary)
    (output_dir / "README.md").write_text(
        "# K1 D435 YOLO Stream vs File Benchmark\n\n"
        "This benchmark compares direct in-memory D435 frame inference with a PNG write/read roundtrip.\n\n"
        "Outputs:\n"
        "- `benchmark_summary.json`\n"
        "- `frame_benchmark.csv`\n"
        "- `temp_png/` only when `--keep-temp-png` is used\n\n"
        "Claim boundary: local vision benchmark only; no ROS, cmd_vel, serial, chassis, or arm control.\n",
        encoding="utf-8",
    )
    print(json.dumps(summary, ensure_ascii=False, indent=2), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
