#!/usr/bin/env python3
"""K1 V4L2 D435 color + SpaceMIT EP YOLO + odom risk mapper.

This field runner is used when the ROS RealSense image pipeline is too heavy
for the SpaceMIT EP path. It opens the D435 color stream through V4L2, runs
local YOLO with ONNX Runtime, subscribes to /odom, and writes the same monitor
files used by the prelim dashboard.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path
from typing import Any, Dict, List, Optional

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from tools.d435_capture_once import odom_to_dict  # noqa: E402
from tools.run_k1_d435_yolo_realtime_display import (  # noqa: E402
    build_gstreamer_pipeline,
    resolve_model_input_size,
    run_inference,
    select_ort_providers,
)
from tools.run_prelim_remote_mapping_yolo_arm_demo import (  # noqa: E402
    RiskEventStore,
    append_jsonl,
    base_point_to_odom_xy,
    camera_point_to_base_point,
    draw_overlay,
    evaluate_auto_risk_gate,
    now_id,
    now_iso,
    parse_auto_risk_gates,
    risk_info_for_class,
    write_json,
)


def write_text(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")


class OdomTap(Node):
    def __init__(self, odom_topic: str, alarm_topic: str):
        super().__init__("k1_v4l2_ep_yolo_odom_risk_mapper")
        self.latest_odom: Optional[Odometry] = None
        self.create_subscription(Odometry, odom_topic, self._odom_cb, 20)
        self.alarm_pub = self.create_publisher(String, alarm_topic, 10)

    def _odom_cb(self, msg: Odometry) -> None:
        self.latest_odom = msg

    def odom_dict(self) -> Optional[Dict[str, Any]]:
        return odom_to_dict(self.latest_odom) if self.latest_odom is not None else None

    def publish_alarm(self, event: Dict[str, Any]) -> None:
        msg = String()
        msg.data = json.dumps(
            {
                "event_type": event.get("event_type"),
                "source": "k1_v4l2_ep_yolo_odom_risk_mapper",
                "class_name": event.get("class_name"),
                "confidence": event.get("confidence"),
                "distance_m": event.get("distance_m"),
                "odom_point_xy_m": event.get("odom_point_xy_m"),
                "timestamp": now_iso(),
            },
            ensure_ascii=False,
        )
        self.alarm_pub.publish(msg)


def add_assumed_depth_projection(det: Dict[str, Any], args: argparse.Namespace) -> Dict[str, Any]:
    x, y, w, h = [float(v) for v in det.get("bbox_xywh", [0.0, 0.0, 0.0, 0.0])]
    u = x + w * 0.5
    v = y + h * 0.5
    depth = float(args.assumed_depth_m)
    x_m = (u - float(args.camera_cx)) * depth / float(args.camera_fx)
    y_m = (v - float(args.camera_cy)) * depth / float(args.camera_fy)
    det = dict(det)
    det["depth_median_m"] = round(depth, 4)
    det["bbox_valid_depth_ratio"] = None
    det["camera_point_xyz_m"] = {
        "x": round(float(x_m), 4),
        "y": round(float(y_m), 4),
        "z": round(depth, 4),
    }
    det["depth_localization"] = {
        "depth_status": "assumed",
        "depth_median_m": round(depth, 4),
        "camera_point_xyz_m": det["camera_point_xyz_m"],
        "projection_note": "v4l2 color-only field fallback",
    }
    return det


def write_alarm_state(
    output_dir: Path,
    latest_alarm: Optional[Dict[str, Any]],
    detections: List[Dict[str, Any]],
    event_count: int,
    latency_ms: Optional[float],
    infer_fps: float,
    frame_count: int,
) -> None:
    current = []
    formal = 0
    for det in detections:
        gate = det.get("auto_risk_gate") or {}
        if gate.get("allowed"):
            formal += 1
        current.append(
            {
                "class_name": det.get("class_name") or det.get("label"),
                "confidence": det.get("confidence"),
                "distance_m": det.get("depth_median_m"),
                "bbox_xywh": det.get("bbox_xywh"),
                "auto_risk_gate": gate,
            }
        )
    write_json(
        output_dir / "alarm_state.json",
        {
            "updated_at": now_iso(),
            "alarm_active": latest_alarm is not None,
            "latest_event": latest_alarm,
            "current_detection_count": len(detections),
            "formal_detection_count": formal,
            "rejected_detection_count": max(0, len(detections) - formal),
            "current_detections": current,
            "latency_ms": None if latency_ms is None else round(float(latency_ms), 3),
            "infer_fps": round(float(infer_fps), 3),
            "event_count": int(event_count),
            "frame_count": int(frame_count),
            "runtime_backend": "v4l2_color_spacemit_ep_odom_mapper",
        },
    )


def write_map_snapshot(output_dir: Path, points: List[Dict[str, Any]]) -> None:
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        xs: List[float] = []
        ys: List[float] = []
        labels: List[str] = []
        for point in points:
            xy = point.get("odom_point_xy_m") or {}
            if "x" not in xy or "y" not in xy:
                continue
            xs.append(float(xy["x"]))
            ys.append(float(xy["y"]))
            labels.append(str(point.get("class_name") or "risk"))

        fig, ax = plt.subplots(figsize=(5.2, 5.2))
        ax.set_title("K1 Risk Map")
        ax.set_xlabel("x / m")
        ax.set_ylabel("y / m")
        ax.grid(True, alpha=0.25)
        ax.set_aspect("equal", adjustable="box")
        if xs:
            ax.scatter(xs, ys, c="#dc2626", s=90)
            for idx, (x, y, label) in enumerate(zip(xs, ys, labels), start=1):
                ax.text(x + 0.02, y + 0.02, f"{idx}:{label}", fontsize=9)
            pad = 0.35
            ax.set_xlim(min(xs) - pad, max(xs) + pad)
            ax.set_ylim(min(ys) - pad, max(ys) + pad)
        fig.tight_layout()
        fig.savefig(output_dir / "risk_map_snapshot.png", dpi=160)
        plt.close(fig)
    except Exception as exc:  # pragma: no cover - field fallback.
        write_text(output_dir / "risk_map_snapshot_skipped.txt", str(exc))


def save_event(
    output_dir: Path,
    store: RiskEventStore,
    det: Dict[str, Any],
    frame_bgr: Any,
    odom: Optional[Dict[str, Any]],
    latency_ms: Optional[float],
    node: OdomTap,
) -> Optional[Dict[str, Any]]:
    import cv2

    class_name = str(det.get("class_name") or det.get("label") or "risk")
    event_type, risk_level, recommended_action = risk_info_for_class(class_name)
    camera_point = det.get("camera_point_xyz_m")
    base_point = camera_point_to_base_point(camera_point) if isinstance(camera_point, dict) else None
    odom_point = base_point_to_odom_xy(base_point, odom) if base_point and odom else None
    projection_status = "projected" if odom_point else "missing_pose"
    key = store.dedup_key(det, odom_point)
    event_id = f"risk_event_{now_id()}"
    distance_m = det.get("depth_median_m")
    map_point = {
        "map_point_id": f"map_point_{len(store.map_points) + 1:03d}",
        "event_id": event_id,
        "class_name": class_name,
        "event_type": event_type,
        "risk_level": risk_level,
        "confidence": det.get("confidence"),
        "distance_m": distance_m,
        "bbox_xywh": det.get("bbox_xywh"),
        "camera_point_xyz_m": camera_point,
        "base_point_xyz_m": base_point,
        "odom_point_xy_m": odom_point,
        "projection_status": projection_status,
        "projection_mode": "v4l2_rgb_assumed_depth_odom_approx",
        "robot_odom_pose": (odom or {}).get("pose") if odom else None,
        "auto_risk_gate": det.get("auto_risk_gate"),
    }
    event = {
        "event_id": event_id,
        "dedup_key": key,
        "first_seen": now_iso(),
        "last_seen": now_iso(),
        "seen_count": 1,
        "class_name": class_name,
        "event_type": event_type,
        "risk_level": risk_level,
        "recommended_action": recommended_action,
        "auto_risk_gate": det.get("auto_risk_gate"),
        "confidence": det.get("confidence"),
        "confidence_max": det.get("confidence"),
        "distance_m": distance_m,
        "latest_distance_m": distance_m,
        "projection_status": projection_status,
        "odom_point_xy_m": odom_point,
        "latest_odom_point_xy_m": odom_point,
    }
    is_new, stored = store.register(key, event, map_point)
    if not is_new:
        return None

    capture_dir = output_dir / "captures" / event_id
    capture_dir.mkdir(parents=True, exist_ok=True)
    overlay_path = capture_dir / "overlay.png"
    rgb_path = capture_dir / "rgb.png"
    detection_path = capture_dir / "risk_detection.json"
    event_path = capture_dir / "risk_event.json"
    map_point_path = capture_dir / "risk_map_point.json"

    cv2.imwrite(str(rgb_path), frame_bgr)
    draw_overlay(frame_bgr.copy(), [det], overlay_path)
    write_json(map_point_path, map_point)
    write_json(
        detection_path,
        {
            "risk_detection_id": f"risk_detection_{event_id}",
            "timestamp": now_iso(),
            "backend": "yolov8n_onnx_spacemit_ep",
            "runtime_backend": "v4l2_color_spacemit_ep",
            "local_inference": True,
            "online_api_used": False,
            "latency_ms": None if latency_ms is None else round(float(latency_ms), 3),
            "fps": None if latency_ms is None else round(1000.0 / max(float(latency_ms), 1e-6), 3),
            "detections": [det],
        },
    )
    stored.update(
        {
            "capture_dir": str(capture_dir),
            "rgb_path": str(rgb_path),
            "overlay_path": str(overlay_path),
            "risk_detection_path": str(detection_path),
            "risk_map_point_path": str(map_point_path),
        }
    )
    write_json(event_path, stored)
    append_jsonl(output_dir / "risk_events.jsonl", stored)
    node.publish_alarm(stored)
    return stored


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--provider", choices=["auto", "spacemit", "cpu"], default="spacemit")
    parser.add_argument("--model", required=True)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--video-device", default="/dev/video24")
    parser.add_argument("--pixel-format", default="YUY2")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=15)
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--conf", type=float, default=0.15)
    parser.add_argument("--iou", type=float, default=0.45)
    parser.add_argument("--max-det", type=int, default=10)
    parser.add_argument("--max-events-per-frame", type=int, default=3)
    parser.add_argument("--auto-risk-gates", default="*:0.60:0.0:5.0")
    parser.add_argument("--dedup-map-grid-m", type=float, default=0.20)
    parser.add_argument("--dedup-image-grid-px", type=int, default=80)
    parser.add_argument("--assumed-depth-m", type=float, default=0.72)
    parser.add_argument("--camera-fx", type=float, default=615.0)
    parser.add_argument("--camera-fy", type=float, default=615.0)
    parser.add_argument("--camera-cx", type=float, default=320.0)
    parser.add_argument("--camera-cy", type=float, default=240.0)
    parser.add_argument("--odom-topic", default="/odom")
    parser.add_argument("--alarm-topic", default="/perception/risk_alarm")
    parser.add_argument("--cli-print-period-s", type=float, default=0.5)
    return parser.parse_args()


def main() -> int:
    import cv2
    import onnxruntime as ort

    args = parse_args()
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    gates = parse_auto_risk_gates(args.auto_risk_gates)
    store = RiskEventStore(output_dir, args.dedup_map_grid_m, args.dedup_image_grid_px)

    providers = select_ort_providers(args.provider)
    model_path = Path(args.model)
    session = ort.InferenceSession(str(model_path), providers=providers)
    input_name = session.get_inputs()[0].name
    output_name = session.get_outputs()[0].name
    args.model_input_shape = session.get_inputs()[0].shape
    args.model_input_size_hw = resolve_model_input_size(session, args.imgsz)
    active_providers = session.get_providers()

    write_json(
        output_dir / "runtime_status.json",
        {
            "schema_version": "k1_v4l2_ep_yolo_odom_risk_mapper_v1",
            "started_at": now_iso(),
            "model_path": str(model_path),
            "onnxruntime_providers": active_providers,
            "auto_risk_gates": gates,
            "video_device": args.video_device,
            "assumed_depth_m": args.assumed_depth_m,
            "odom_topic": args.odom_topic,
            "claim_boundary": [
                "Field fallback uses V4L2 RGB plus odom for responsive EP demonstration.",
                "Projection uses configured assumed depth because ROS RealSense is disabled.",
            ],
        },
    )

    pipeline = build_gstreamer_pipeline(args)
    cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
    if not cap.isOpened():
        raise SystemExit(f"failed to open D435 V4L2 stream: {pipeline}")

    rclpy.init()
    node = OdomTap(args.odom_topic, args.alarm_topic)
    frame_count = 0
    last_print = 0.0
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            ok, frame_bgr = cap.read()
            if not ok or frame_bgr is None:
                time.sleep(0.05)
                continue

            detections, latency_ms = run_inference(session, input_name, output_name, frame_bgr, args)
            infer_fps = 1000.0 / max(float(latency_ms), 1e-6)
            frame_count += 1
            odom = node.odom_dict()

            localized: List[Dict[str, Any]] = []
            for raw in sorted(detections, key=lambda item: float(item.get("confidence", 0.0)), reverse=True):
                det = add_assumed_depth_projection(raw, args)
                gate = evaluate_auto_risk_gate(det, gates)
                det["auto_risk_gate"] = gate
                localized.append(det)

            formal = [det for det in localized if (det.get("auto_risk_gate") or {}).get("allowed")]
            draw_overlay(frame_bgr.copy(), localized[: args.max_det], output_dir / "latest_overlay.png")

            new_events: List[Dict[str, Any]] = []
            for det in formal[: args.max_events_per_frame]:
                created = save_event(output_dir, store, det, frame_bgr, odom, latency_ms, node)
                if created is not None:
                    new_events.append(created)

            latest_alarm = new_events[0] if new_events else None
            if latest_alarm is None and formal:
                top = formal[0]
                event_type, risk_level, recommended_action = risk_info_for_class(
                    str(top.get("class_name") or top.get("label") or "risk")
                )
                latest_alarm = {
                    "event_id": "current_detection_gate_pass_not_new_deduped",
                    "class_name": top.get("class_name") or top.get("label"),
                    "event_type": event_type,
                    "risk_level": risk_level,
                    "recommended_action": recommended_action,
                    "confidence": top.get("confidence"),
                    "distance_m": top.get("depth_median_m"),
                    "auto_risk_gate": top.get("auto_risk_gate"),
                    "projection_status": "current_detection_gate_pass",
                }

            store.write_all()
            write_map_snapshot(output_dir, store.map_points)
            write_alarm_state(output_dir, latest_alarm, localized, len(store.events), latency_ms, infer_fps, frame_count)

            now = time.monotonic()
            if now - last_print >= float(args.cli_print_period_s):
                det_text = "none"
                if localized:
                    det_text = "; ".join(
                        f"{det.get('class_name')} {float(det.get('confidence', 0.0)):.2f} gate={(det.get('auto_risk_gate') or {}).get('reason')}"
                        for det in localized[: args.max_det]
                    )
                print(
                    f"{now_iso()} frame={frame_count} infer_fps={infer_fps:.2f} "
                    f"formal={len(formal)} events={len(store.events)} detections={det_text}",
                    flush=True,
                )
                last_print = now
    finally:
        cap.release()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
