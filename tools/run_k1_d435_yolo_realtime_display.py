#!/usr/bin/env python3
"""Run K1 D435 V4L2 + YOLO ONNX realtime display.

This is a local vision display tool only. It does not start ROS, publish
cmd_vel, open serial devices, or control the chassis/arm.
"""

from __future__ import annotations

import argparse
import csv
import json
import sys
import threading
import time
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Sequence, Tuple


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from tools.export_risk_detection_for_policy import detections_to_policy_risk_fields
from tools.run_yolo_inference_once import letterbox_rgb, postprocess


DEFAULT_MODEL = "models/risk_vision/yolov8n_int8.onnx"
DEFAULT_OUTPUT_DIR = "outputs/k1_d435_yolo_realtime_v1"
CLASSES = ["crack", "corrosion", "leakage", "blockage"]
COLORS = {
    "crack": (40, 40, 255),
    "corrosion": (0, 160, 255),
    "leakage": (255, 120, 0),
    "blockage": (255, 0, 220),
}


def now_id() -> str:
    return datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]


def now_iso() -> str:
    return datetime.now().isoformat(timespec="milliseconds")


def write_json(path: Path, data: Dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def build_gstreamer_pipeline(args: argparse.Namespace) -> str:
    return (
        f"v4l2src device={args.video_device} ! "
        f"video/x-raw,format={args.pixel_format},width={args.width},height={args.height},framerate={args.fps}/1 ! "
        "videoconvert ! video/x-raw,format=BGR ! "
        "appsink drop=true max-buffers=1 sync=false"
    )


def select_ort_providers(provider_mode: str) -> List[str]:
    import onnxruntime as ort

    if provider_mode in ("auto", "spacemit"):
        try:
            import spacemit_ort  # noqa: F401
        except Exception as exc:
            if provider_mode == "spacemit":
                raise SystemExit(f"Failed to import spacemit_ort: {exc}") from exc

    available = ort.get_available_providers()
    if provider_mode == "cpu":
        return ["CPUExecutionProvider"]
    if provider_mode == "spacemit":
        if "SpaceMITExecutionProvider" not in available:
            raise SystemExit(f"SpaceMITExecutionProvider unavailable: {available}")
        return ["SpaceMITExecutionProvider", "CPUExecutionProvider"]
    if "SpaceMITExecutionProvider" in available:
        return ["SpaceMITExecutionProvider", "CPUExecutionProvider"]
    return ["CPUExecutionProvider"]


def resolve_model_input_size(session: Any, fallback_imgsz: int) -> Tuple[int, int]:
    """Return ONNX input size as (height, width), falling back to square imgsz."""
    shape = session.get_inputs()[0].shape
    if len(shape) >= 4 and isinstance(shape[2], int) and isinstance(shape[3], int):
        return int(shape[2]), int(shape[3])
    return int(fallback_imgsz), int(fallback_imgsz)


def preprocess_bgr(frame_bgr: Any, input_size_hw: Tuple[int, int]):
    import cv2
    import numpy as np

    frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
    padded, ratio, pad = letterbox_rgb(frame_rgb, input_size_hw)
    tensor = padded.astype("float32") / 255.0
    tensor = np.transpose(tensor, (2, 0, 1))[None, ...]
    return tensor, ratio, pad


def run_inference(session: Any, input_name: str, output_name: str, frame_bgr: Any, args: argparse.Namespace):
    tensor, ratio, pad = preprocess_bgr(frame_bgr, args.model_input_size_hw)
    start = time.perf_counter()
    output = session.run([output_name], {input_name: tensor})[0]
    latency_ms = (time.perf_counter() - start) * 1000.0
    detections = postprocess(
        output,
        image_shape=frame_bgr.shape[:2],
        ratio=ratio,
        pad=pad,
        conf_thres=args.conf,
        iou_thres=args.iou,
        max_det=args.max_det,
    )
    return detections, latency_ms


def inference_worker(
    state: Dict[str, Any],
    session: Any,
    input_name: str,
    output_name: str,
    args: argparse.Namespace,
) -> None:
    last_processed_seq = -1
    while not state["stop"].is_set():
        state["event"].wait(0.1)
        if state["stop"].is_set():
            break
        with state["lock"]:
            frame = None if state["latest_frame"] is None else state["latest_frame"].copy()
            seq = state["latest_seq"]
            state["event"].clear()
        if frame is None or seq == last_processed_seq:
            continue
        last_processed_seq = seq
        try:
            detections, latency_ms = run_inference(session, input_name, output_name, frame, args)
            error = None
        except Exception as exc:  # Keep display alive even if one inference fails.
            detections = []
            latency_ms = None
            error = f"{type(exc).__name__}: {exc}"
        with state["lock"]:
            state["detections"] = detections
            state["latency_ms"] = latency_ms
            state["infer_fps"] = 0.0 if not latency_ms else 1000.0 / max(1e-6, float(latency_ms))
            state["result_seq"] = seq
            state["result_time"] = time.perf_counter()
            state["error"] = error


def draw_overlay(
    frame_bgr: Any,
    detections: List[Dict[str, Any]],
    latency_ms: float | None,
    display_fps: float,
    infer_fps: float,
    args: argparse.Namespace,
) -> Any:
    import cv2

    out = frame_bgr.copy()
    for det in detections:
        x, y, w, h = [int(round(v)) for v in det["bbox_xywh"]]
        x2, y2 = x + w, y + h
        label = det["class_name"]
        color = COLORS.get(label, (0, 255, 0))
        cv2.rectangle(out, (x, y), (x2, y2), color, 2)
        text = f"{label} {det['confidence']:.2f}"
        y_text = max(18, y - 6)
        cv2.rectangle(out, (x, y_text - 18), (x + min(260, 9 * len(text)), y_text + 4), color, -1)
        cv2.putText(out, text, (x + 3, y_text), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)

    mean_brightness = float(frame_bgr.mean())
    latency_text = "latency=?" if latency_ms is None else f"latency={latency_ms:.0f}ms"
    mode_text = "async" if not args.sync_inference else "sync"
    header = (
        f"K1 D435 YOLO | det={len(detections)} | {latency_text} | "
        f"infer={infer_fps:.2f} FPS | display={display_fps:.1f} FPS | {mode_text}"
    )
    cv2.putText(out, header, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (0, 255, 255), 2, cv2.LINE_AA)
    cv2.putText(
        out,
        f"{Path(args.model).name} | {args.video_device} {args.pixel_format} | q/ESC quit, s save",
        (10, 48),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        (220, 220, 220),
        1,
        cv2.LINE_AA,
    )
    if mean_brightness < args.low_light_mean_threshold:
        cv2.putText(
            out,
            f"LOW LIGHT mean={mean_brightness:.1f}: turn on fill light",
            (10, 76),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 180, 255),
            2,
            cv2.LINE_AA,
        )
    return out


def save_snapshot(
    output_dir: Path,
    frame_bgr: Any,
    overlay_bgr: Any,
    detections: List[Dict[str, Any]],
    latency_ms: float | None,
    args: argparse.Namespace,
) -> Path:
    import cv2

    capture_id = f"{now_id()}_k1_yolo"
    capture_dir = output_dir / "captures" / capture_id
    capture_dir.mkdir(parents=True, exist_ok=True)
    rgb_path = capture_dir / "rgb.png"
    overlay_path = capture_dir / "overlay.png"
    cv2.imwrite(str(rgb_path), frame_bgr)
    cv2.imwrite(str(overlay_path), overlay_bgr)
    policy_risk = detections_to_policy_risk_fields(detections)
    write_json(
        capture_dir / "risk_detection.json",
        {
            "capture_id": capture_id,
            "captured_at": now_iso(),
            "backend": "yolov8n_onnx_cpu",
            "runtime_backend": "spacemit_onnxruntime_cpu",
            "model_path": str(args.model),
            "provider_mode": args.provider,
            "video_device": args.video_device,
            "pixel_format": args.pixel_format,
            "model_used": True,
            "local_inference": True,
            "online_api_used": False,
            "inference_executed": True,
            "latency_ms": None if latency_ms is None else round(float(latency_ms), 3),
            "fps": None if not latency_ms else round(1000.0 / float(latency_ms), 3),
            "detections": detections,
            "policy_risk_fields": policy_risk,
            "rgb_path": str(rgb_path),
            "overlay_path": str(overlay_path),
            "claim_boundary": [
                "This is K1 local D435 V4L2 + YOLO ONNX realtime display.",
                "No ROS, cmd_vel, serial device, chassis control, or arm control is used.",
                "This does not claim real-world defect detection accuracy.",
            ],
        },
    )
    return capture_dir


def detection_position_summary(detections: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    positions: List[Dict[str, Any]] = []
    for det in detections:
        x, y, w, h = [float(v) for v in det.get("bbox_xywh", [0.0, 0.0, 0.0, 0.0])]
        positions.append(
            {
                "class_id": det.get("class_id"),
                "class_name": det.get("class_name"),
                "confidence": det.get("confidence"),
                "bbox_xywh": det.get("bbox_xywh"),
                "bbox_xyxy": det.get("bbox_xyxy"),
                "bbox_center_xy": [round(x + w / 2.0, 2), round(y + h / 2.0, 2)],
            }
        )
    return positions


def write_headless_logs(output_dir: Path, records: List[Dict[str, Any]], args: argparse.Namespace) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    latencies = [float(r["latency_ms"]) for r in records if r.get("latency_ms") is not None]
    fps_values = [float(r["infer_fps"]) for r in records if r.get("infer_fps") is not None]
    detected = [r for r in records if r.get("detection_count", 0) > 0]
    summary = {
        "created_at": now_iso(),
        "frame_count": len(records),
        "detected_frame_count": len(detected),
        "model": args.model,
        "provider_mode": args.provider,
        "video_device": args.video_device,
        "pixel_format": args.pixel_format,
        "headless": True,
        "local_inference": True,
        "online_api_used": False,
        "ros_started": False,
        "cmd_vel_published": False,
        "serial_port_opened": False,
        "avg_latency_ms": round(sum(latencies) / len(latencies), 3) if latencies else None,
        "min_latency_ms": round(min(latencies), 3) if latencies else None,
        "max_latency_ms": round(max(latencies), 3) if latencies else None,
        "avg_infer_fps": round(sum(fps_values) / len(fps_values), 3) if fps_values else None,
        "max_detection_count": max((r.get("detection_count", 0) for r in records), default=0),
        "claim_boundary": [
            "This is K1 headless D435 V4L2 + YOLO ONNX inference measurement.",
            "Positions are 2D image bbox positions unless depth/pose is added by another pipeline.",
            "No ROS, cmd_vel, serial device, chassis control, or arm control is used.",
        ],
    }
    write_json(output_dir / "headless_summary.json", summary)

    csv_path = output_dir / "headless_frame_log.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=[
                "frame_index",
                "captured_at",
                "latency_ms",
                "infer_fps",
                "detection_count",
                "top_class_name",
                "top_confidence",
                "top_bbox_xywh",
                "top_bbox_center_xy",
            ],
        )
        writer.writeheader()
        for r in records:
            positions = r.get("positions", [])
            top = positions[0] if positions else {}
            writer.writerow(
                {
                    "frame_index": r.get("frame_index"),
                    "captured_at": r.get("captured_at"),
                    "latency_ms": r.get("latency_ms"),
                    "infer_fps": r.get("infer_fps"),
                    "detection_count": r.get("detection_count"),
                    "top_class_name": top.get("class_name"),
                    "top_confidence": top.get("confidence"),
                    "top_bbox_xywh": json.dumps(top.get("bbox_xywh"), ensure_ascii=False),
                    "top_bbox_center_xy": json.dumps(top.get("bbox_center_xy"), ensure_ascii=False),
                }
            )

    with (output_dir / "headless_detections.jsonl").open("w", encoding="utf-8") as f:
        for r in records:
            f.write(json.dumps(r, ensure_ascii=False) + "\n")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", default=DEFAULT_MODEL)
    parser.add_argument(
        "--ort-site-packages",
        default="",
        help="Optional site-packages path for an alternate ONNX Runtime, inserted after cv2 imports.",
    )
    parser.add_argument("--provider", choices=["auto", "spacemit", "cpu"], default="auto")
    parser.add_argument("--video-device", default="/dev/video24")
    parser.add_argument("--pixel-format", default="YUY2", help="GStreamer raw format, D435 color is usually YUY2 on /dev/video24.")
    parser.add_argument("--output-dir", default=DEFAULT_OUTPUT_DIR)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--conf", type=float, default=0.25)
    parser.add_argument("--iou", type=float, default=0.7)
    parser.add_argument("--max-det", type=int, default=20)
    parser.add_argument("--low-light-mean-threshold", type=float, default=35.0)
    parser.add_argument("--warmup-frames", type=int, default=0, help="Discard N camera frames before inference/saving.")
    parser.add_argument("--sync-inference", action="store_true", help="Block display on every inference. Default is async display.")
    parser.add_argument("--headless-smoke-frames", type=int, default=0, help="Read and infer N frames without opening a display window.")
    parser.add_argument("--cli-realtime", action="store_true", help="Print realtime detections to stdout without opening a display window.")
    parser.add_argument("--cli-max-frames", type=int, default=0, help="Stop CLI realtime after N frames; 0 means run until Ctrl-C.")
    parser.add_argument("--cli-print-period-s", type=float, default=0.2, help="Minimum seconds between CLI realtime lines.")
    return parser.parse_args()


def main() -> int:
    import cv2

    args = parse_args()
    if args.ort_site_packages:
        # Import cv2 first so it keeps the system NumPy ABI, then route only ORT
        # imports to the alternate site-packages path.
        sys.path.insert(0, args.ort_site_packages)
    import onnxruntime as ort

    model_path = Path(args.model)
    if not model_path.exists():
        raise SystemExit(f"Model not found: {model_path}")

    print(f"Loading ONNX model: {model_path}")
    print(f"ONNX Runtime version: {ort.__version__} from {ort.__file__}")
    providers = select_ort_providers(args.provider)
    print(f"Using ONNX Runtime providers: {providers}")
    session = ort.InferenceSession(str(model_path), providers=providers)
    active_providers = session.get_providers()
    print(f"Active ONNX Runtime providers: {active_providers}")
    input_name = session.get_inputs()[0].name
    output_name = session.get_outputs()[0].name
    args.model_input_shape = session.get_inputs()[0].shape
    args.model_input_size_hw = resolve_model_input_size(session, args.imgsz)
    print(f"Model input shape: {args.model_input_shape}, using input_size_hw={args.model_input_size_hw}")

    pipeline = build_gstreamer_pipeline(args)
    print(f"Opening D435 stream: {pipeline}")
    cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
    if not cap.isOpened():
        raise SystemExit(f"Failed to open GStreamer pipeline for {args.video_device}")

    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    write_json(
        output_dir / "runtime_status.json",
        {
            "started_at": now_iso(),
            "model": str(model_path),
            "onnxruntime_version": getattr(ort, "__version__", None),
            "onnxruntime_file": getattr(ort, "__file__", None),
            "ort_site_packages": args.ort_site_packages,
            "provider_mode": args.provider,
            "providers_requested": providers,
            "providers_active": active_providers,
            "model_input_shape": list(args.model_input_shape),
            "model_input_size_hw": list(args.model_input_size_hw),
            "video_device": args.video_device,
            "pixel_format": args.pixel_format,
            "width": args.width,
            "height": args.height,
            "fps_requested": args.fps,
            "local_inference": True,
            "online_api_used": False,
            "ros_started": False,
            "cmd_vel_published": False,
            "serial_port_opened": False,
            "display_mode": "sync" if args.sync_inference else "async_latest_frame",
        },
    )

    window_name = "K1 D435 YOLO realtime"
    window_enabled = args.headless_smoke_frames <= 0 and not args.cli_realtime
    if window_enabled:
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(window_name, args.width, args.height)

    last_display_t = time.perf_counter()
    display_fps = 0.0
    frame_count = 0
    warmup_remaining = max(0, int(args.warmup_frames))
    last_detections: List[Dict[str, Any]] = []
    last_latency_ms: float | None = None
    infer_fps = 0.0
    headless_records: List[Dict[str, Any]] = []

    async_state: Dict[str, Any] | None = None
    worker: threading.Thread | None = None
    use_async = not args.sync_inference and window_enabled
    last_cli_print_t = 0.0
    if use_async:
        async_state = {
            "lock": threading.Lock(),
            "event": threading.Event(),
            "stop": threading.Event(),
            "latest_frame": None,
            "latest_seq": -1,
            "result_seq": -1,
            "result_time": None,
            "detections": [],
            "latency_ms": None,
            "infer_fps": 0.0,
            "error": None,
        }
        worker = threading.Thread(
            target=inference_worker,
            args=(async_state, session, input_name, output_name, args),
            name="k1-yolo-inference",
            daemon=True,
        )
        worker.start()

    try:
        while True:
            ok, frame_bgr = cap.read()
            if not ok or frame_bgr is None:
                print("Frame read failed; retrying...")
                time.sleep(0.1)
                continue
            if warmup_remaining > 0:
                if warmup_remaining == args.warmup_frames:
                    print(f"Warming up camera: discarding {args.warmup_frames} frames...", flush=True)
                warmup_remaining -= 1
                if warmup_remaining == 0:
                    print("Camera warmup complete.", flush=True)
                continue

            if use_async and async_state is not None:
                with async_state["lock"]:
                    async_state["latest_frame"] = frame_bgr.copy()
                    async_state["latest_seq"] += 1
                    async_state["event"].set()
                    detections = list(async_state["detections"])
                    latency_ms = async_state["latency_ms"]
                    infer_fps = float(async_state["infer_fps"])
                last_detections = detections
                last_latency_ms = latency_ms
            else:
                detections, latency_ms = run_inference(session, input_name, output_name, frame_bgr, args)
                last_detections = detections
                last_latency_ms = latency_ms
                infer_fps = 1000.0 / max(1e-6, latency_ms)

            now = time.perf_counter()
            inst_display_fps = 1.0 / max(1e-6, now - last_display_t)
            last_display_t = now
            display_fps = inst_display_fps if frame_count == 0 else 0.85 * display_fps + 0.15 * inst_display_fps
            frame_count += 1

            if args.cli_realtime:
                if now - last_cli_print_t >= args.cli_print_period_s:
                    if last_detections:
                        det_text = "; ".join(
                            f"{det.get('class_name')} {float(det.get('confidence', 0.0)):.2f} "
                            f"bbox={det.get('bbox_xywh')}"
                            for det in last_detections[: args.max_det]
                        )
                    else:
                        det_text = "none"
                    latency_text = "?" if latency_ms is None else f"{float(latency_ms):.1f}ms"
                    print(
                        f"{now_iso()} frame={frame_count} det={len(last_detections)} "
                        f"latency={latency_text} infer_fps={infer_fps:.2f} "
                        f"display_fps={display_fps:.1f} detections={det_text}",
                        flush=True,
                    )
                    last_cli_print_t = now
                if args.cli_max_frames > 0 and frame_count >= args.cli_max_frames:
                    break
                continue

            overlay = draw_overlay(frame_bgr, detections, latency_ms, display_fps, infer_fps, args)
            if args.headless_smoke_frames > 0:
                headless_records.append(
                    {
                        "frame_index": frame_count,
                        "captured_at": now_iso(),
                        "latency_ms": None if latency_ms is None else round(float(latency_ms), 3),
                        "infer_fps": None if not latency_ms else round(1000.0 / float(latency_ms), 3),
                        "detection_count": len(last_detections),
                        "positions": detection_position_summary(last_detections),
                        "policy_risk_fields": detections_to_policy_risk_fields(last_detections),
                    }
                )
                if frame_count >= args.headless_smoke_frames:
                    saved = save_snapshot(output_dir, frame_bgr, overlay, last_detections, last_latency_ms, args)
                    write_headless_logs(output_dir, headless_records, args)
                    print(f"Smoke saved snapshot: {saved}")
                    break
                continue

            cv2.imshow(window_name, overlay)
            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord("q")):
                break
            if key == ord("s"):
                saved = save_snapshot(output_dir, frame_bgr, overlay, last_detections, last_latency_ms, args)
                print(f"Saved snapshot: {saved}")
    finally:
        if async_state is not None:
            async_state["stop"].set()
            async_state["event"].set()
        if worker is not None:
            worker.join(timeout=2.0)
        cap.release()
        if window_enabled:
            cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
