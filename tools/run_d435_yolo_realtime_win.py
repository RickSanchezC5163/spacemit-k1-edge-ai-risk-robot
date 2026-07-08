#!/usr/bin/env python3
"""Run Windows D435 + local YOLO risk detection with realtime display.

This is a local visual test tool only. It does not start ROS, publish cmd_vel,
open robot serial devices, or control the chassis/arm.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Sequence, Tuple

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from tools.run_yolo_inference_once import letterbox_rgb, postprocess


DEFAULT_MODEL = (
    "datasets/risk_print_yolo_v1/runs/detect/risk_print_yolov8n_e50/weights/best.pt"
)

COLORS = {
    "crack": (40, 40, 255),
    "corrosion": (0, 160, 255),
    "leakage": (255, 120, 0),
    "blockage": (255, 0, 220),
}


def now_id() -> str:
    return datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]


def write_json(path: Path, data: Dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def depth_summary_for_bbox(
    depth: Any,
    bbox_xyxy: Sequence[float],
    depth_scale_m: float,
    min_depth_m: float,
    max_depth_m: float,
) -> Dict[str, Any]:
    import numpy as np

    h, w = depth.shape[:2]
    x1, y1, x2, y2 = [int(round(float(v))) for v in bbox_xyxy]
    x1 = max(0, min(w - 1, x1))
    y1 = max(0, min(h - 1, y1))
    x2 = max(0, min(w, x2))
    y2 = max(0, min(h, y2))
    if x2 <= x1 or y2 <= y1:
        return {
            "depth_median_m": None,
            "bbox_valid_depth_ratio": 0.0,
            "depth_status": "invalid_bbox",
        }
    roi = depth[y1:y2, x1:x2]
    if roi.size == 0:
        return {
            "depth_median_m": None,
            "bbox_valid_depth_ratio": 0.0,
            "depth_status": "empty_roi",
        }
    depth_m = roi.astype("float32") * float(depth_scale_m)
    valid = depth_m[(depth_m >= min_depth_m) & (depth_m <= max_depth_m)]
    if valid.size == 0:
        return {
            "depth_median_m": None,
            "bbox_valid_depth_ratio": 0.0,
            "depth_status": "no_valid_depth",
        }
    return {
        "depth_median_m": round(float(np.median(valid)), 3),
        "bbox_valid_depth_ratio": round(float(valid.size) / float(roi.size), 4),
        "depth_status": "valid",
    }


def camera_info_from_profile(profile: Any) -> Dict[str, Any]:
    intr = profile.as_video_stream_profile().get_intrinsics()
    return {
        "width": intr.width,
        "height": intr.height,
        "fx": intr.fx,
        "fy": intr.fy,
        "cx": intr.ppx,
        "cy": intr.ppy,
        "coeffs": list(intr.coeffs),
        "model": str(intr.model),
        "k": [intr.fx, 0.0, intr.ppx, 0.0, intr.fy, intr.ppy, 0.0, 0.0, 1.0],
    }


def detections_from_result(result: Any, depth: Any, depth_scale_m: float, args: argparse.Namespace) -> List[Dict[str, Any]]:
    detections: List[Dict[str, Any]] = []
    names = result.names
    if result.boxes is None:
        return detections
    boxes = result.boxes
    for idx in range(len(boxes)):
        xyxy = boxes.xyxy[idx].detach().cpu().numpy().tolist()
        cls_id = int(boxes.cls[idx].detach().cpu().item())
        conf = float(boxes.conf[idx].detach().cpu().item())
        label = str(names.get(cls_id, cls_id))
        x1, y1, x2, y2 = xyxy
        depth_info = depth_summary_for_bbox(
            depth,
            xyxy,
            depth_scale_m=depth_scale_m,
            min_depth_m=args.min_depth_m,
            max_depth_m=args.max_depth_m,
        )
        detections.append(
            {
                "class_id": cls_id,
                "label": label,
                "confidence": round(conf, 4),
                "bbox_xyxy": [round(float(v), 2) for v in xyxy],
                "bbox_xywh": [
                    round(float(x1), 2),
                    round(float(y1), 2),
                    round(float(x2 - x1), 2),
                    round(float(y2 - y1), 2),
                ],
                **depth_info,
            }
        )
    return detections


def resolve_model_input_size(session: Any, fallback_imgsz: int) -> Tuple[int, int]:
    """Return ONNX input size as (height, width), falling back to square imgsz."""
    shape = session.get_inputs()[0].shape
    if len(shape) >= 4 and isinstance(shape[2], int) and isinstance(shape[3], int):
        return int(shape[2]), int(shape[3])
    return int(fallback_imgsz), int(fallback_imgsz)


def preprocess_bgr_for_onnx(frame_bgr: Any, input_size_hw: Tuple[int, int]):
    import cv2
    import numpy as np

    frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
    padded, ratio, pad = letterbox_rgb(frame_rgb, input_size_hw)
    tensor = padded.astype("float32") / 255.0
    tensor = np.transpose(tensor, (2, 0, 1))[None, ...]
    return tensor, ratio, pad


def detections_from_onnx(
    session: Any,
    input_name: str,
    output_name: str,
    frame_bgr: Any,
    depth: Any,
    depth_scale_m: float,
    args: argparse.Namespace,
) -> List[Dict[str, Any]]:
    tensor, ratio, pad = preprocess_bgr_for_onnx(frame_bgr, args.model_input_size_hw)
    output = session.run([output_name], {input_name: tensor})[0]
    raw_detections = postprocess(
        output,
        image_shape=frame_bgr.shape[:2],
        ratio=ratio,
        pad=pad,
        conf_thres=args.conf,
        iou_thres=args.iou,
        max_det=args.max_det,
    )
    detections: List[Dict[str, Any]] = []
    for det in raw_detections:
        xyxy = det["bbox_xyxy"]
        depth_info = depth_summary_for_bbox(
            depth,
            xyxy,
            depth_scale_m=depth_scale_m,
            min_depth_m=args.min_depth_m,
            max_depth_m=args.max_depth_m,
        )
        detections.append(
            {
                "class_id": det["class_id"],
                "label": det["class_name"],
                "confidence": det["confidence"],
                "bbox_xyxy": det["bbox_xyxy"],
                "bbox_xywh": det["bbox_xywh"],
                **depth_info,
            }
        )
    return detections


def draw_overlay(frame_bgr: Any, detections: List[Dict[str, Any]], fps: float, model_path: Path, device: str) -> Any:
    import cv2

    out = frame_bgr.copy()
    for det in detections:
        x1, y1, x2, y2 = [int(round(v)) for v in det["bbox_xyxy"]]
        label = det["label"]
        color = COLORS.get(label, (0, 255, 0))
        cv2.rectangle(out, (x1, y1), (x2, y2), color, 2)
        depth = det.get("depth_median_m")
        depth_text = "depth=?m" if depth is None else f"depth={depth:.3f}m"
        text = f"{label} {det['confidence']:.2f} {depth_text}"
        y_text = max(18, y1 - 6)
        cv2.rectangle(out, (x1, y_text - 18), (x1 + min(430, 9 * len(text)), y_text + 4), color, -1)
        cv2.putText(out, text, (x1 + 3, y_text), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
    header = f"D435 YOLO realtime | FPS {fps:.1f} | det {len(detections)} | {device} | q/ESC quit, s save"
    cv2.putText(out, header, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (0, 255, 255), 2, cv2.LINE_AA)
    cv2.putText(out, str(model_path.name), (10, 48), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (220, 220, 220), 1, cv2.LINE_AA)
    return out


def save_snapshot(
    output_dir: Path,
    rgb: Any,
    depth: Any,
    overlay_bgr: Any,
    detections: List[Dict[str, Any]],
    camera_info: Dict[str, Any],
    depth_scale_m: float,
    model_path: Path,
    args: argparse.Namespace,
) -> Path:
    import cv2
    import numpy as np

    capture_id = f"{now_id()}_d435_yolo"
    capture_dir = output_dir / "captures" / capture_id
    capture_dir.mkdir(parents=True, exist_ok=True)
    rgb_bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    cv2.imwrite(str(capture_dir / "rgb.png"), rgb_bgr)
    cv2.imwrite(str(capture_dir / "overlay.png"), overlay_bgr)
    np.save(capture_dir / "depth_raw.npy", depth)
    write_json(capture_dir / "camera_info.json", camera_info)
    write_json(
        capture_dir / "detections.json",
        {
            "capture_id": capture_id,
            "captured_at": datetime.now().isoformat(timespec="milliseconds"),
            "model_path": str(model_path),
            "device": args.device,
            "conf_threshold": args.conf,
            "imgsz": args.imgsz,
            "model_input_shape": getattr(args, "model_input_shape", None),
            "model_input_size_hw": list(getattr(args, "model_input_size_hw", [])),
            "depth_scale_m": depth_scale_m,
            "detections": detections,
            "claim_boundary": [
                "This is local Windows D435 realtime model testing.",
                "This does not start ROS, publish cmd_vel, or control robot hardware.",
                "Depth values are approximate D435 bbox medians, not map-positioned risk points.",
            ],
        },
    )
    return capture_dir


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", default=DEFAULT_MODEL)
    parser.add_argument("--output-dir", default="outputs/d435_yolo_realtime_win_v1")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--conf", type=float, default=0.35)
    parser.add_argument("--iou", type=float, default=0.7)
    parser.add_argument("--runtime", choices=["auto", "onnxruntime", "ultralytics"], default="auto")
    parser.add_argument("--providers", default="CPUExecutionProvider")
    parser.add_argument("--device", default="0", help="Ultralytics device, e.g. 0 or cpu")
    parser.add_argument("--max-det", type=int, default=20)
    parser.add_argument("--min-depth-m", type=float, default=0.15)
    parser.add_argument("--max-depth-m", type=float, default=5.0)
    parser.add_argument("--status-only", action="store_true")
    return parser.parse_args()


def status_only() -> int:
    import pyrealsense2 as rs

    try:
        import onnxruntime as ort
    except Exception as exc:
        ort = None
        print("onnxruntime unavailable", repr(exc))
    try:
        import torch
    except Exception as exc:
        torch = None
        print("torch unavailable", repr(exc))
    try:
        import ultralytics
    except Exception as exc:
        ultralytics = None
        print("ultralytics unavailable", repr(exc))

    ctx = rs.context()
    devices = ctx.query_devices()
    if ort is not None:
        print("onnxruntime", ort.__version__, ort.get_available_providers())
    if ultralytics is not None:
        print("ultralytics", ultralytics.__version__)
    if torch is not None:
        print("torch", torch.__version__)
        print("cuda_available", torch.cuda.is_available())
    if torch is not None and torch.cuda.is_available():
        print("cuda_device", torch.cuda.get_device_name(0))
    if len(devices) == 0:
        print("No RealSense device found.")
        return 1
    for dev in devices:
        print(
            "RealSense:",
            dev.get_info(rs.camera_info.name),
            "serial=" + dev.get_info(rs.camera_info.serial_number),
            "firmware=" + dev.get_info(rs.camera_info.firmware_version),
        )
    return 0


def main() -> int:
    import cv2
    import numpy as np
    import pyrealsense2 as rs

    args = parse_args()
    if args.status_only:
        return status_only()

    model_path = Path(args.model)
    if not model_path.exists():
        raise SystemExit(f"Model not found: {model_path}")
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    print(f"Loading model: {model_path}")
    runtime = args.runtime
    if runtime == "auto":
        runtime = "onnxruntime" if model_path.suffix.lower() == ".onnx" else "ultralytics"
    model = None
    session = None
    input_name = None
    output_name = None
    runtime_label = runtime
    if runtime == "onnxruntime":
        import onnxruntime as ort

        providers = [item.strip() for item in args.providers.split(",") if item.strip()]
        print(f"Using ONNX Runtime providers: {providers}")
        session = ort.InferenceSession(str(model_path), providers=providers)
        print(f"Active ONNX Runtime providers: {session.get_providers()}")
        input_name = session.get_inputs()[0].name
        output_name = session.get_outputs()[0].name
        args.model_input_shape = session.get_inputs()[0].shape
        args.model_input_size_hw = resolve_model_input_size(session, args.imgsz)
        print(f"Model input shape: {args.model_input_shape}, using input_size_hw={args.model_input_size_hw}")
        runtime_label = "onnxruntime:" + ",".join(session.get_providers())
    else:
        from ultralytics import YOLO

        model = YOLO(str(model_path))
        args.model_input_shape = None
        args.model_input_size_hw = (args.imgsz, args.imgsz)
        runtime_label = f"ultralytics:{args.device}"
    print(f"Opening D435 {args.width}x{args.height}@{args.fps}")

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, args.width, args.height, rs.format.rgb8, args.fps)
    config.enable_stream(rs.stream.depth, args.width, args.height, rs.format.z16, args.fps)
    profile = pipeline.start(config)
    align = rs.align(rs.stream.color)
    color_profile = profile.get_stream(rs.stream.color)
    camera_info = camera_info_from_profile(color_profile)
    try:
        depth_scale_m = float(profile.get_device().first_depth_sensor().get_depth_scale())
    except Exception:
        depth_scale_m = 0.001

    window_name = "D435 YOLO realtime - local only"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, args.width, args.height)
    last_t = time.perf_counter()
    fps_smooth = 0.0
    frame_count = 0
    try:
        while True:
            try:
                frames = pipeline.wait_for_frames(timeout_ms=5000)
            except RuntimeError as exc:
                print(f"wait_for_frames timeout: {exc}", flush=True)
                continue
            aligned = align.process(frames)
            color_frame = aligned.get_color_frame()
            depth_frame = aligned.get_depth_frame()
            if not color_frame or not depth_frame:
                continue
            rgb = np.asanyarray(color_frame.get_data()).copy()
            depth = np.asanyarray(depth_frame.get_data()).copy()
            frame_bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            # Ultralytics treats numpy camera frames as OpenCV-style BGR.
            # D435 color frames are rgb8, so convert before inference.
            if runtime == "onnxruntime":
                detections = detections_from_onnx(
                    session,
                    input_name,
                    output_name,
                    frame_bgr,
                    depth,
                    depth_scale_m,
                    args,
                )
            else:
                result = model.predict(
                    source=frame_bgr,
                    imgsz=args.imgsz,
                    conf=args.conf,
                    iou=args.iou,
                    device=args.device,
                    max_det=args.max_det,
                    verbose=False,
                )[0]
                detections = detections_from_result(result, depth, depth_scale_m, args)
            now = time.perf_counter()
            inst_fps = 1.0 / max(1e-6, now - last_t)
            last_t = now
            fps_smooth = inst_fps if frame_count == 0 else 0.88 * fps_smooth + 0.12 * inst_fps
            frame_count += 1
            overlay = draw_overlay(frame_bgr, detections, fps_smooth, model_path, runtime_label)
            cv2.imshow(window_name, overlay)
            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord("q")):
                break
            if key == ord("s"):
                saved = save_snapshot(output_dir, rgb, depth, overlay, detections, camera_info, depth_scale_m, model_path, args)
                print(f"Saved snapshot: {saved}")
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
