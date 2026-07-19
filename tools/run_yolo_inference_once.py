#!/usr/bin/env python3
"""Run one local YOLOv8 ONNX risk inference and export policy risk fields.

This is an offline/local inference utility. It does not start ROS, publish
cmd_vel, open serial devices, or control robot hardware.
"""

from __future__ import annotations

import argparse
import importlib.util
import glob
import json
import math
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Sequence, Tuple


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from tools.export_risk_detection_for_policy import detections_to_policy_risk_fields


def _load_depth_localizer():
    try:
        from src.primitives.d435_depth_localization import localize_bbox_with_depth as func

        return func
    except Exception:
        module_path = ROOT / "src" / "primitives" / "d435_depth_localization.py"
        spec = importlib.util.spec_from_file_location("d435_depth_localization", module_path)
        if spec is None or spec.loader is None:
            raise
        module = importlib.util.module_from_spec(spec)
        sys.modules[spec.name] = module
        spec.loader.exec_module(module)
        return module.localize_bbox_with_depth


localize_bbox_with_depth = _load_depth_localizer()


CLASSES = ["crack", "corrosion", "leakage", "blockage"]
DEFAULT_MODEL = "models/risk_vision/yolov8n.onnx"


def now_id() -> str:
    return datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]


def now_iso() -> str:
    return datetime.now().isoformat(timespec="milliseconds")


def write_json(path: Path, data: Dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def resolve_image(pattern: str) -> Path:
    matches = sorted(glob.glob(pattern))
    if not matches and Path(pattern).exists():
        matches = [pattern]
    if not matches:
        raise SystemExit(f"No image matched: {pattern}")
    return Path(matches[0])


def letterbox_rgb(image_rgb: Any, new_shape: Tuple[int, int] = (640, 640), color: Tuple[int, int, int] = (114, 114, 114)):
    import cv2
    import numpy as np

    height, width = image_rgb.shape[:2]
    target_h, target_w = new_shape
    ratio = min(target_w / width, target_h / height)
    resized_w = int(round(width * ratio))
    resized_h = int(round(height * ratio))
    pad_w = target_w - resized_w
    pad_h = target_h - resized_h
    pad_left = int(round(pad_w / 2 - 0.1))
    pad_right = int(round(pad_w / 2 + 0.1))
    pad_top = int(round(pad_h / 2 - 0.1))
    pad_bottom = int(round(pad_h / 2 + 0.1))
    resized = cv2.resize(image_rgb, (resized_w, resized_h), interpolation=cv2.INTER_LINEAR)
    padded = cv2.copyMakeBorder(
        resized,
        pad_top,
        pad_bottom,
        pad_left,
        pad_right,
        cv2.BORDER_CONSTANT,
        value=color,
    )
    return padded, ratio, (pad_left, pad_top)


def resolve_model_input_size(model_path: Path, fallback_imgsz: int) -> Tuple[int, int]:
    if model_path.suffix.lower() == ".onnx":
        try:
            import onnx

            model = onnx.load(str(model_path))
            shape = model.graph.input[0].type.tensor_type.shape.dim
            if len(shape) >= 4 and shape[2].dim_value and shape[3].dim_value:
                return int(shape[2].dim_value), int(shape[3].dim_value)
        except Exception:
            pass
        try:
            import onnxruntime as ort

            session = ort.InferenceSession(str(model_path), providers=["CPUExecutionProvider"])
            shape = session.get_inputs()[0].shape
            if len(shape) >= 4 and isinstance(shape[2], int) and isinstance(shape[3], int):
                return int(shape[2]), int(shape[3])
        except Exception:
            pass
    return int(fallback_imgsz), int(fallback_imgsz)


def preprocess(image_path: Path, input_size_hw: Tuple[int, int]):
    import cv2
    import numpy as np

    bgr = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
    if bgr is None:
        raise SystemExit(f"Failed to read image: {image_path}")
    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    padded, ratio, pad = letterbox_rgb(rgb, input_size_hw)
    tensor = padded.astype("float32") / 255.0
    tensor = np.transpose(tensor, (2, 0, 1))[None, ...]
    return bgr, tensor, ratio, pad


def xywh_to_xyxy(x: float, y: float, w: float, h: float) -> List[float]:
    return [x - w / 2.0, y - h / 2.0, x + w / 2.0, y + h / 2.0]


def clip_box(box: Sequence[float], width: int, height: int) -> List[float]:
    x1, y1, x2, y2 = box
    return [
        max(0.0, min(float(width), x1)),
        max(0.0, min(float(height), y1)),
        max(0.0, min(float(width), x2)),
        max(0.0, min(float(height), y2)),
    ]


def box_iou(a: Sequence[float], b: Sequence[float]) -> float:
    ax1, ay1, ax2, ay2 = a
    bx1, by1, bx2, by2 = b
    ix1, iy1 = max(ax1, bx1), max(ay1, by1)
    ix2, iy2 = min(ax2, bx2), min(ay2, by2)
    iw, ih = max(0.0, ix2 - ix1), max(0.0, iy2 - iy1)
    inter = iw * ih
    area_a = max(0.0, ax2 - ax1) * max(0.0, ay2 - ay1)
    area_b = max(0.0, bx2 - bx1) * max(0.0, by2 - by1)
    denom = area_a + area_b - inter
    return inter / denom if denom > 0 else 0.0


def nms(detections: List[Dict[str, Any]], iou_thres: float, max_det: int) -> List[Dict[str, Any]]:
    selected: List[Dict[str, Any]] = []
    by_class: Dict[int, List[Dict[str, Any]]] = {}
    for det in detections:
        by_class.setdefault(det["class_id"], []).append(det)
    for _, class_dets in by_class.items():
        ordered = sorted(class_dets, key=lambda item: item["confidence"], reverse=True)
        while ordered and len(selected) < max_det:
            current = ordered.pop(0)
            selected.append(current)
            ordered = [det for det in ordered if box_iou(current["bbox_xyxy"], det["bbox_xyxy"]) <= iou_thres]
    return sorted(selected, key=lambda item: item["confidence"], reverse=True)[:max_det]


def postprocess(
    output: Any,
    image_shape: Tuple[int, int],
    ratio: float,
    pad: Tuple[int, int],
    conf_thres: float,
    iou_thres: float,
    max_det: int,
) -> List[Dict[str, Any]]:
    import numpy as np

    pred = np.asarray(output)
    if pred.ndim == 3:
        pred = pred[0]
    if pred.shape[0] == 4 + len(CLASSES):
        pred = pred.T
    if pred.shape[-1] < 4 + len(CLASSES):
        raise SystemExit(f"Unexpected ONNX output shape: {output.shape}")

    image_h, image_w = image_shape
    pad_x, pad_y = pad
    candidates: List[Dict[str, Any]] = []
    for row in pred:
        class_scores = row[4 : 4 + len(CLASSES)]
        class_id = int(np.argmax(class_scores))
        conf = float(class_scores[class_id])
        if conf < conf_thres:
            continue
        x, y, w, h = [float(v) for v in row[:4]]
        x1, y1, x2, y2 = xywh_to_xyxy(x, y, w, h)
        x1 = (x1 - pad_x) / ratio
        y1 = (y1 - pad_y) / ratio
        x2 = (x2 - pad_x) / ratio
        y2 = (y2 - pad_y) / ratio
        box = clip_box([x1, y1, x2, y2], image_w, image_h)
        if box[2] <= box[0] or box[3] <= box[1]:
            continue
        candidates.append(
            {
                "class_id": class_id,
                "class_name": CLASSES[class_id],
                "confidence": conf,
                "bbox_xyxy": box,
            }
        )
    selected = nms(candidates, iou_thres=iou_thres, max_det=max_det)
    for det in selected:
        x1, y1, x2, y2 = det["bbox_xyxy"]
        det["bbox_xyxy"] = [round(float(v), 2) for v in det["bbox_xyxy"]]
        det["bbox_xywh"] = [
            round(float(x1), 2),
            round(float(y1), 2),
            round(float(x2 - x1), 2),
            round(float(y2 - y1), 2),
        ]
        det["confidence"] = round(float(det["confidence"]), 4)
        det["depth_median_m"] = None
        det["bbox_valid_depth_ratio"] = None
        det["camera_point_xyz_m"] = None
    return selected


def apply_depth(detections: List[Dict[str, Any]], depth_path: str | None, camera_info_path: str | None, depth_scale_m: float) -> Dict[str, Any]:
    status = {
        "depth_localization_attempted": bool(depth_path and camera_info_path),
        "depth_localization_applied_count": 0,
        "depth_localization_status": "not_requested",
    }
    if not depth_path or not camera_info_path:
        return status
    status["depth_localization_status"] = "attempted" if detections else "no_detections"
    for det in detections:
        localized = localize_bbox_with_depth(det["bbox_xywh"], depth_path, camera_info_path, depth_scale_m=depth_scale_m)
        det["depth_localization"] = localized
        if localized.get("depth_status") == "valid":
            det["depth_median_m"] = localized.get("depth_median_m")
            det["bbox_valid_depth_ratio"] = localized.get("bbox_valid_depth_ratio")
            det["camera_point_xyz_m"] = localized.get("camera_point_xyz_m")
            status["depth_localization_applied_count"] += 1
    if status["depth_localization_applied_count"]:
        status["depth_localization_status"] = "valid"
    elif detections:
        status["depth_localization_status"] = "missing_or_invalid"
    return status


def draw_overlay(image_bgr: Any, detections: List[Dict[str, Any]], output_path: Path) -> None:
    import cv2

    colors = {
        "crack": (40, 40, 255),
        "corrosion": (0, 160, 255),
        "leakage": (255, 120, 0),
        "blockage": (255, 0, 220),
    }
    out = image_bgr.copy()
    for det in detections:
        x, y, w, h = [int(round(v)) for v in det["bbox_xywh"]]
        x2, y2 = x + w, y + h
        color = colors.get(det["class_name"], (0, 255, 0))
        cv2.rectangle(out, (x, y), (x2, y2), color, 2)
        depth = det.get("depth_median_m")
        depth_text = "" if depth is None else f" {depth:.3f}m"
        text = f"{det['class_name']} {det['confidence']:.2f}{depth_text}"
        cv2.rectangle(out, (x, max(0, y - 20)), (x + min(420, 9 * len(text)), y), color, -1)
        cv2.putText(out, text, (x + 3, max(14, y - 5)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(output_path), out)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--image", required=True, help="Image path or glob pattern.")
    parser.add_argument("--model", default=DEFAULT_MODEL)
    parser.add_argument("--depth", default=None)
    parser.add_argument("--camera-info", default=None)
    parser.add_argument("--depth-scale-m", type=float, default=0.001)
    parser.add_argument("--output-dir", default="outputs/yolo_inference_v1")
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--conf", type=float, default=0.25)
    parser.add_argument("--iou", type=float, default=0.7)
    parser.add_argument("--max-det", type=int, default=20)
    parser.add_argument("--runtime", choices=["auto", "onnxruntime", "opencv_dnn"], default="auto")
    parser.add_argument("--providers", default="CPUExecutionProvider")
    return parser.parse_args()


def run_onnxruntime(model_path: Path, tensor: Any, providers: List[str]) -> Tuple[Any, str, str, float, float]:
    import onnxruntime as ort

    load_start = time.perf_counter()
    session = ort.InferenceSession(str(model_path), providers=providers)
    load_latency_ms = (time.perf_counter() - load_start) * 1000.0
    input_name = session.get_inputs()[0].name
    output_name = session.get_outputs()[0].name
    infer_start = time.perf_counter()
    outputs = session.run([output_name], {input_name: tensor})
    inference_latency_ms = (time.perf_counter() - infer_start) * 1000.0
    return outputs[0], "onnxruntime", "yolov8n_onnx_cpu", load_latency_ms, inference_latency_ms


def run_opencv_dnn(model_path: Path, tensor: Any) -> Tuple[Any, str, str, float, float]:
    import cv2

    load_start = time.perf_counter()
    net = cv2.dnn.readNetFromONNX(str(model_path))
    load_latency_ms = (time.perf_counter() - load_start) * 1000.0
    infer_start = time.perf_counter()
    net.setInput(tensor)
    output = net.forward()
    inference_latency_ms = (time.perf_counter() - infer_start) * 1000.0
    return output, "opencv_dnn", "opencv_dnn_onnx", load_latency_ms, inference_latency_ms


def run_inference(model_path: Path, tensor: Any, args: argparse.Namespace) -> Tuple[Any, str, str, float, float]:
    providers = [item.strip() for item in args.providers.split(",") if item.strip()]
    if args.runtime in ("auto", "onnxruntime"):
        try:
            return run_onnxruntime(model_path, tensor, providers)
        except Exception as exc:
            if args.runtime == "onnxruntime":
                raise
            print(f"onnxruntime unavailable or failed, falling back to opencv_dnn: {exc}", file=sys.stderr)
    return run_opencv_dnn(model_path, tensor)


def main() -> int:
    args = parse_args()
    model_path = Path(args.model)
    if not model_path.exists():
        raise SystemExit(f"Model not found: {model_path}")
    image_path = resolve_image(args.image)
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    precision = "int8" if "int8" in model_path.name.lower() else "fp32"

    input_size_hw = resolve_model_input_size(model_path, args.imgsz)
    image_bgr, tensor, ratio, pad = preprocess(image_path, input_size_hw)
    raw_output, runtime_backend, backend_name, load_latency_ms, inference_latency_ms = run_inference(model_path, tensor, args)
    detections = postprocess(
        raw_output,
        image_shape=image_bgr.shape[:2],
        ratio=ratio,
        pad=pad,
        conf_thres=args.conf,
        iou_thres=args.iou,
        max_det=args.max_det,
    )
    depth_status = apply_depth(detections, args.depth, args.camera_info, args.depth_scale_m)
    policy_risk_fields = detections_to_policy_risk_fields(detections)
    policy_risk_fields["risk_source"] = backend_name
    risk_detection_id = f"risk_detection_{now_id()}"
    model_size_mb = round(model_path.stat().st_size / (1024.0 * 1024.0), 3)
    result = {
        "risk_detection_id": risk_detection_id,
        "timestamp": now_iso(),
        "backend": backend_name,
        "detection_mode": backend_name,
        "model_used": True,
        "local_inference": True,
        "online_api_used": False,
        "runtime_backend": runtime_backend,
        "precision": precision,
        "acceleration_target": "cpu",
        "backend_available": True,
        "fallback_used": False,
        "fallback_backend": None,
        "inference_ready": True,
        "inference_executed": True,
        "model_name": "yolov8n",
        "model_path": str(model_path),
        "model_size_mb": model_size_mb,
        "input_size": [input_size_hw[1], input_size_hw[0]],
        "model_input_size_hw": [input_size_hw[0], input_size_hw[1]],
        "image_path": str(image_path),
        "load_latency_ms": round(load_latency_ms, 3),
        "inference_latency_ms": round(inference_latency_ms, 3),
        "inference_fps": round(1000.0 / inference_latency_ms, 3) if inference_latency_ms > 0 else None,
        "latency_ms": round(inference_latency_ms, 3),
        "fps": round(1000.0 / inference_latency_ms, 3) if inference_latency_ms > 0 else None,
        **depth_status,
        "detections": detections,
        "policy_risk_fields": policy_risk_fields,
        "claim_boundary": [
            "ONNX inference is local and offline; online_api_used=false.",
            "This does not start ROS, publish cmd_vel, open serial, or control robot hardware.",
            "Model was trained on D435-captured A4 printed risk images; do not claim real-world defect accuracy.",
            "K1 latency and FPS must be measured on K1 ARM CPU before deployment claims.",
        ],
    }
    write_json(output_dir / "risk_detection.json", result)
    write_json(output_dir / "policy_risk_fields.json", policy_risk_fields)
    draw_overlay(image_bgr, detections, output_dir / "overlay.png")
    print(json.dumps({
        "risk_detection_json": str(output_dir / "risk_detection.json"),
        "detections": len(detections),
        "policy_risk_fields": policy_risk_fields,
        "online_api_used": False,
        "latency_ms": result["latency_ms"],
        "fps": result["fps"],
    }, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
