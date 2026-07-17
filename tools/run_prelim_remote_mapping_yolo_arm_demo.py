#!/usr/bin/env python3
"""Preliminary remote-mapping demo bridge.

This runner is the thin integration layer for the preliminary competition demo:

manual/remote guarded mapping is launched separately, while this node subscribes
to D435 RGB/depth/camera_info and odom, runs local YOLO, deduplicates visual
risk events, publishes alarms, writes map-point evidence, and generates a
manual/remote arm no-load response candidate.

It never publishes cmd_vel, never starts chassis motion, and never starts the
arm by itself. The operator decides when and where to run the no-load arm
response after positioning the robot by remote control.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from tools.d435_capture_once import (  # noqa: E402
    camera_info_to_dict,
    canonical_rgb,
    depth_to_meters,
    image_msg_to_array,
    odom_to_dict,
    save_depth_vis_png,
    save_rgb_png,
)
from tools.run_k1_d435_yolo_realtime_display import (  # noqa: E402
    resolve_model_input_size,
    run_inference,
    select_ort_providers,
)


DEFAULT_MODEL = "models/risk_vision/yolov8n_320_q_truncated_balanced.onnx"
DEFAULT_OUTPUT_DIR = "outputs/prelim_remote_mapping_yolo_arm_demo_v1"
PROTOCOL_VERSION = "prelim_remote_mapping_yolo_arm_demo_v1"

CAMERA_OFFSET_BASE_M = (0.105, 0.0, 0.11)
AXIS_MAPPING = "d435_optical_to_base_approx:z_forward_to_x_forward,x_right_to_y_right"

CLASS_TO_EVENT = {
    "crack": ("hard_obstacle", "high", "stop_and_report"),
    "corrosion": ("hard_obstacle", "medium", "stop_and_recheck"),
    "leakage": ("hard_obstacle", "high", "stop_and_report"),
    "blockage": ("blocked_path", "high", "stop_and_report"),
}

DEFAULT_AUTO_RISK_GATES_SPEC = "crack:0.29:0.60:0.80,blockage:0.23:0.35:0.75"

COLORS = {
    "crack": (40, 40, 255),
    "corrosion": (0, 160, 255),
    "leakage": (255, 120, 0),
    "blockage": (255, 0, 220),
}


def now_iso() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


def now_id() -> str:
    return datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")[:-4]


def write_json(path: Path, data: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def append_jsonl(path: Path, data: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a", encoding="utf-8") as handle:
        handle.write(json.dumps(data, ensure_ascii=False, sort_keys=True) + "\n")


def write_text(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")


def yaw_from_odom_dict(odom: Dict[str, Any]) -> Optional[float]:
    try:
        return float((odom.get("pose") or {}).get("yaw_rad"))
    except (TypeError, ValueError):
        return None


def camera_point_to_base_point(camera_point: Dict[str, float]) -> Dict[str, float]:
    # D435 optical frame: x right, y down, z forward.
    return {
        "x": round(float(camera_point["z"]) + CAMERA_OFFSET_BASE_M[0], 4),
        "y": round(-float(camera_point["x"]) + CAMERA_OFFSET_BASE_M[1], 4),
        "z": round(-float(camera_point["y"]) + CAMERA_OFFSET_BASE_M[2], 4),
    }


def base_point_to_odom_xy(base_point: Dict[str, float], odom: Dict[str, Any]) -> Optional[Dict[str, float]]:
    pose = odom.get("pose") or {}
    position = pose.get("position") or {}
    yaw = yaw_from_odom_dict(odom)
    if yaw is None:
        return None
    try:
        robot_x = float(position["x"])
        robot_y = float(position["y"])
        base_x = float(base_point["x"])
        base_y = float(base_point["y"])
    except (KeyError, TypeError, ValueError):
        return None
    cos_y = math.cos(yaw)
    sin_y = math.sin(yaw)
    return {
        "x": round(robot_x + base_x * cos_y - base_y * sin_y, 4),
        "y": round(robot_y + base_x * sin_y + base_y * cos_y, 4),
    }


def camera_intrinsics(camera_info: Dict[str, Any]) -> Optional[Tuple[float, float, float, float]]:
    values = camera_info.get("k") or camera_info.get("K")
    if isinstance(values, list) and len(values) >= 9:
        return float(values[0]), float(values[4]), float(values[2]), float(values[5])
    return None


def localize_bbox_in_memory(
    bbox_xywh: Sequence[float],
    depth_m: np.ndarray,
    camera_info: Dict[str, Any],
    min_depth_m: float,
    max_depth_m: float,
) -> Dict[str, Any]:
    if len(bbox_xywh) < 4:
        return {"depth_status": "invalid", "reason": "bbox missing"}
    intrinsics = camera_intrinsics(camera_info)
    if intrinsics is None:
        return {"depth_status": "missing", "reason": "camera intrinsics missing"}
    fx, fy, cx, cy = intrinsics
    if fx == 0.0 or fy == 0.0:
        return {"depth_status": "invalid", "reason": "camera intrinsics invalid"}

    height, width = depth_m.shape[:2]
    x, y, w, h = [int(round(float(value))) for value in bbox_xywh[:4]]
    if w <= 0 or h <= 0:
        return {"depth_status": "invalid", "reason": "bbox width/height invalid"}
    x0 = max(0, min(width - 1, x))
    y0 = max(0, min(height - 1, y))
    x1 = max(0, min(width, x + w))
    y1 = max(0, min(height, y + h))
    if x1 <= x0 or y1 <= y0:
        return {"depth_status": "invalid", "reason": "bbox outside depth image"}

    roi = depth_m[y0:y1, x0:x1]
    finite = np.isfinite(roi)
    valid = roi[finite & (roi >= float(min_depth_m)) & (roi <= float(max_depth_m))]
    if valid.size == 0:
        return {
            "bbox_xywh": list(bbox_xywh[:4]),
            "depth_median_m": None,
            "bbox_valid_depth_ratio": 0.0,
            "camera_point_xyz_m": None,
            "depth_status": "invalid",
            "reason": "no valid depth in bbox",
        }

    depth_median = float(np.median(valid))
    center_u = (x0 + x1 - 1) / 2.0
    center_v = (y0 + y1 - 1) / 2.0
    x_m = (center_u - cx) * depth_median / fx
    y_m = (center_v - cy) * depth_median / fy
    return {
        "bbox_xywh": list(bbox_xywh[:4]),
        "depth_median_m": round(depth_median, 4),
        "bbox_valid_depth_ratio": round(float(valid.size) / float(roi.size), 4),
        "camera_point_xyz_m": {
            "x": round(float(x_m), 4),
            "y": round(float(y_m), 4),
            "z": round(float(depth_median), 4),
        },
        "depth_status": "valid",
        "claim_boundary": [
            "D435 bbox depth localization is approximate unless TF and camera calibration are validated.",
            "Risk map projection is demo-level approximate odom/map projection.",
        ],
    }


def draw_overlay(image_bgr: np.ndarray, detections: List[Dict[str, Any]], output_path: Path) -> None:
    import cv2

    out = image_bgr.copy()
    for det in detections:
        name = str(det.get("class_name") or det.get("label") or "risk")
        color = COLORS.get(name, (0, 255, 255))
        x, y, w, h = [int(round(float(v))) for v in det.get("bbox_xywh", [0, 0, 0, 0])]
        x2, y2 = x + w, y + h
        cv2.rectangle(out, (x, y), (x2, y2), color, 2)
        depth = det.get("depth_median_m")
        depth_text = "" if depth is None else f" {float(depth):.2f}m"
        text = f"{name} {float(det.get('confidence', 0.0)):.2f}{depth_text}"
        text_y = max(18, y - 6)
        cv2.rectangle(out, (x, text_y - 18), (x + min(430, 9 * len(text)), text_y + 4), color, -1)
        cv2.putText(out, text, (x + 3, text_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(output_path), out):
        raise RuntimeError(f"failed to write overlay image: {output_path}")


def risk_info_for_class(class_name: str) -> Tuple[str, str, str]:
    return CLASS_TO_EVENT.get(class_name, ("hard_obstacle", "medium", "stop_and_recheck"))


def parse_auto_risk_gates(spec: str) -> Dict[str, Dict[str, float]]:
    gates: Dict[str, Dict[str, float]] = {}
    if not str(spec or "").strip():
        return gates
    for item in str(spec).split(","):
        parts = [part.strip() for part in item.split(":")]
        if len(parts) != 4 or not parts[0]:
            raise ValueError(
                "auto risk gate entries must use class:min_conf:min_depth_m:max_depth_m, "
                f"got: {item!r}"
            )
        class_name = parts[0]
        min_confidence = float(parts[1])
        min_depth_m = float(parts[2])
        max_depth_m = float(parts[3])
        if min_depth_m > max_depth_m:
            raise ValueError(f"auto risk gate min depth > max depth for {class_name}")
        gates[class_name] = {
            "min_confidence": min_confidence,
            "min_depth_m": min_depth_m,
            "max_depth_m": max_depth_m,
        }
    return gates


def evaluate_auto_risk_gate(det: Dict[str, Any], gates: Dict[str, Dict[str, float]]) -> Dict[str, Any]:
    class_name = str(det.get("class_name") or det.get("label") or "risk")
    confidence = float(det.get("confidence") or 0.0)
    depth_value = det.get("depth_median_m")
    result: Dict[str, Any] = {
        "allowed": False,
        "class_name": class_name,
        "actual_confidence": round(confidence, 4),
        "actual_depth_m": depth_value,
    }
    gate = gates.get(class_name) or gates.get("*") or gates.get("all") or gates.get("any")
    if gate is None:
        result["reason"] = "class_not_enabled_for_auto_alarm_or_map"
        return result
    result.update(gate)
    if confidence < float(gate["min_confidence"]):
        result["reason"] = "confidence_below_gate"
        return result
    if depth_value is None:
        result["reason"] = "missing_depth"
        return result
    try:
        depth_m = float(depth_value)
    except (TypeError, ValueError):
        result["reason"] = "invalid_depth"
        return result
    result["actual_depth_m"] = round(depth_m, 4)
    if depth_m < float(gate["min_depth_m"]):
        result["reason"] = "depth_below_gate"
        return result
    if depth_m > float(gate["max_depth_m"]):
        result["reason"] = "depth_above_gate"
        return result
    result["allowed"] = True
    result["reason"] = "passed"
    return result


def build_ort_session_options(args: argparse.Namespace) -> Any:
    import onnxruntime as ort

    options = ort.SessionOptions()
    graph_levels = {
        "disabled": ort.GraphOptimizationLevel.ORT_DISABLE_ALL,
        "basic": ort.GraphOptimizationLevel.ORT_ENABLE_BASIC,
        "extended": ort.GraphOptimizationLevel.ORT_ENABLE_EXTENDED,
        "all": ort.GraphOptimizationLevel.ORT_ENABLE_ALL,
    }
    options.graph_optimization_level = graph_levels[str(args.ort_graph_optimization_level)]
    if int(args.ort_intra_op_threads) > 0:
        options.intra_op_num_threads = int(args.ort_intra_op_threads)
    if int(args.ort_inter_op_threads) > 0:
        options.inter_op_num_threads = int(args.ort_inter_op_threads)
    if args.ort_execution_mode == "sequential":
        options.execution_mode = ort.ExecutionMode.ORT_SEQUENTIAL
    elif args.ort_execution_mode == "parallel":
        options.execution_mode = ort.ExecutionMode.ORT_PARALLEL
    if args.ort_allow_spinning != "default":
        options.add_session_config_entry("session.intra_op.allow_spinning", str(args.ort_allow_spinning))
    return options


def odom_is_zero(odom: Optional[Dict[str, Any]], linear_eps: float, angular_eps: float) -> bool:
    if not odom:
        return False
    twist = odom.get("twist") or {}
    linear = twist.get("linear") or {}
    angular = twist.get("angular") or {}
    try:
        lx = abs(float(linear.get("x", 0.0)))
        ly = abs(float(linear.get("y", 0.0)))
        az = abs(float(angular.get("z", 0.0)))
    except (TypeError, ValueError):
        return False
    return lx <= linear_eps and ly <= linear_eps and az <= angular_eps


class RiskEventStore:
    def __init__(self, output_dir: Path, map_grid_m: float, image_grid_px: int):
        self.output_dir = output_dir
        self.map_grid_m = float(map_grid_m)
        self.image_grid_px = max(1, int(image_grid_px))
        self.events_by_key: Dict[str, Dict[str, Any]] = {}
        self.events: List[Dict[str, Any]] = []
        self.map_points: List[Dict[str, Any]] = []

    def dedup_key(self, det: Dict[str, Any], odom_point: Optional[Dict[str, float]]) -> str:
        class_name = str(det.get("class_name") or det.get("label") or "risk")
        if odom_point:
            ix = int(math.floor(float(odom_point["x"]) / self.map_grid_m))
            iy = int(math.floor(float(odom_point["y"]) / self.map_grid_m))
            return f"{class_name}:map:{ix}:{iy}"
        x, y, w, h = [float(v) for v in det.get("bbox_xywh", [0, 0, 0, 0])]
        cx = int(math.floor((x + w / 2.0) / self.image_grid_px))
        cy = int(math.floor((y + h / 2.0) / self.image_grid_px))
        return f"{class_name}:image:{cx}:{cy}"

    def register(
        self,
        key: str,
        event: Dict[str, Any],
        map_point: Dict[str, Any],
    ) -> Tuple[bool, Dict[str, Any]]:
        existing = self.events_by_key.get(key)
        if existing is not None:
            existing["last_seen"] = event["first_seen"]
            existing["seen_count"] = int(existing.get("seen_count", 1)) + 1
            existing["confidence_max"] = max(
                float(existing.get("confidence_max") or 0.0),
                float(event.get("confidence") or 0.0),
            )
            existing["latest_distance_m"] = event.get("distance_m")
            existing["latest_odom_point_xy_m"] = map_point.get("odom_point_xy_m")
            return False, existing

        self.events_by_key[key] = event
        self.events.append(event)
        self.map_points.append(map_point)
        return True, event

    def write_all(self) -> None:
        write_json(
            self.output_dir / "risk_event_index.json",
            {
                "schema_version": "risk_event_index_v1",
                "updated_at": now_iso(),
                "dedup": {
                    "map_grid_m": self.map_grid_m,
                    "image_grid_px": self.image_grid_px,
                    "key_count": len(self.events_by_key),
                },
                "event_count": len(self.events),
                "events": self.events,
            },
        )
        write_json(
            self.output_dir / "risk_map_points.json",
            {
                "schema_version": "risk_map_points_v1",
                "updated_at": now_iso(),
                "projection_mode": "d435_bbox_depth_odom_approx",
                "axis_mapping": AXIS_MAPPING,
                "camera_offset_base_m": list(CAMERA_OFFSET_BASE_M),
                "risk_map_points": self.map_points,
                "summary": {
                    "risk_map_points": len(self.map_points),
                    "projected": sum(
                        1 for point in self.map_points if point.get("projection_status") == "projected"
                    ),
                    "missing_projection": sum(
                        1 for point in self.map_points if point.get("projection_status") != "projected"
                    ),
                },
                "claim_boundary": [
                    "Risk map points are approximate odom/map projections.",
                    "Do not claim high-precision absolute defect coordinates.",
                ],
            },
        )


class PrelimDemoNode(Node):
    def __init__(self, args: argparse.Namespace):
        super().__init__("prelim_remote_mapping_yolo_arm_demo")
        self.args = args
        self.output_dir = Path(args.output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.auto_risk_gates = parse_auto_risk_gates(args.auto_risk_gates)
        self.store = RiskEventStore(
            self.output_dir,
            map_grid_m=args.dedup_map_grid_m,
            image_grid_px=args.dedup_image_grid_px,
        )
        self.errors: List[Dict[str, Any]] = []
        self.arm_candidate_count = 0

        self.latest_rgb: Optional[Image] = None
        self.latest_depth: Optional[Image] = None
        self.latest_camera_info: Optional[CameraInfo] = None
        self.latest_odom: Optional[Odometry] = None
        self.latest_times = {"rgb": 0.0, "depth": 0.0, "camera_info": 0.0, "odom": 0.0}
        self.last_infer_time = 0.0
        self.frame_count = 0
        self.inference_count = 0

        import onnxruntime as ort

        if int(args.opencv_num_threads) >= 0:
            try:
                import cv2

                cv2.setNumThreads(int(args.opencv_num_threads))
            except Exception as exc:  # noqa: BLE001 - best-effort runtime tuning.
                self.errors.append(
                    {"timestamp": now_iso(), "stage": "opencv_set_threads", "error": str(exc)}
                )

        model_path = Path(args.model)
        if not model_path.exists():
            raise FileNotFoundError(f"model not found: {model_path}")
        providers = select_ort_providers(args.provider)
        session_options = build_ort_session_options(args)
        load_start = time.perf_counter()
        self.session = ort.InferenceSession(str(model_path), sess_options=session_options, providers=providers)
        self.model_load_latency_ms = round((time.perf_counter() - load_start) * 1000.0, 3)
        self.input_name = self.session.get_inputs()[0].name
        self.output_name = self.session.get_outputs()[0].name
        self.args.model_input_size_hw = resolve_model_input_size(self.session, args.imgsz)
        self.active_providers = self.session.get_providers()

        self.event_pub = self.create_publisher(String, args.event_topic, 10)
        self.demo_event_pub = self.create_publisher(String, args.demo_event_topic, 10)
        self.alarm_pub = self.create_publisher(String, args.alarm_topic, 10)

        self.create_subscription(Image, args.rgb_topic, self._rgb_cb, 10)
        self.create_subscription(Image, args.depth_topic, self._depth_cb, 10)
        self.create_subscription(CameraInfo, args.camera_info_topic, self._camera_info_cb, 10)
        self.create_subscription(Odometry, args.odom_topic, self._odom_cb, 20)
        self.timer = self.create_timer(max(0.05, float(args.timer_period_s)), self._timer_cb)

        write_json(
            self.output_dir / "runtime_status.json",
            {
                "schema_version": PROTOCOL_VERSION,
                "started_at": now_iso(),
                "model_path": str(model_path),
                "model_load_latency_ms": self.model_load_latency_ms,
                "onnxruntime_providers": self.active_providers,
                "runtime_tuning": {
                    "opencv_num_threads": args.opencv_num_threads,
                    "ort_graph_optimization_level": args.ort_graph_optimization_level,
                    "ort_execution_mode": args.ort_execution_mode,
                    "ort_intra_op_threads": args.ort_intra_op_threads,
                    "ort_inter_op_threads": args.ort_inter_op_threads,
                    "ort_allow_spinning": args.ort_allow_spinning,
                    "no_visuals": args.no_visuals,
                },
                "topics": {
                    "rgb": args.rgb_topic,
                    "depth": args.depth_topic,
                    "camera_info": args.camera_info_topic,
                    "odom": args.odom_topic,
                    "event": args.event_topic,
                    "demo_event": args.demo_event_topic,
                    "alarm": args.alarm_topic,
                },
                "cmd_vel_published_by_this_node": False,
                "arm_response_mode": args.arm_response_mode,
                "auto_risk_gates": self.auto_risk_gates,
                "claim_boundary": self.claim_boundary(),
            },
        )
        self.write_readme()
        self.get_logger().info(
            "Prelim demo bridge ready. model=%s providers=%s output=%s"
            % (model_path, self.active_providers, self.output_dir)
        )

    @staticmethod
    def claim_boundary() -> List[str]:
        return [
            "Manual/remote guarded mapping must be launched separately.",
            "This node subscribes to D435 and odom only; it never publishes cmd_vel.",
            "YOLO detections are local model inference results; printed-risk accuracy is not real-world defect accuracy.",
            "Risk map projection uses approximate bbox+depth+odom geometry.",
            "Only detections that pass the configured class confidence/depth gates are promoted to formal alarms and map points.",
            "Arm no-load response is operator-triggered by remote/manual control; this node only records candidates.",
            "RL is not used to control the real vehicle in this demo.",
        ]

    def _rgb_cb(self, msg: Image) -> None:
        self.latest_rgb = msg
        self.latest_times["rgb"] = time.monotonic()

    def _depth_cb(self, msg: Image) -> None:
        self.latest_depth = msg
        self.latest_times["depth"] = time.monotonic()

    def _camera_info_cb(self, msg: CameraInfo) -> None:
        self.latest_camera_info = msg
        self.latest_times["camera_info"] = time.monotonic()

    def _odom_cb(self, msg: Odometry) -> None:
        self.latest_odom = msg
        self.latest_times["odom"] = time.monotonic()

    def freshness(self) -> Dict[str, Any]:
        now = time.monotonic()
        result = {}
        for name in ("rgb", "depth", "camera_info", "odom"):
            latest = getattr(self, f"latest_{name}", None)
            age = None if latest is None else round(now - self.latest_times[name], 3)
            result[name] = {
                "fresh": latest is not None and age is not None and age <= self.args.fresh_timeout_s,
                "age_s": age,
            }
        return result

    def fresh_inputs_ready(self) -> bool:
        status = self.freshness()
        required = ("rgb", "depth", "camera_info", "odom")
        return all(bool(status[name]["fresh"]) for name in required)

    def _timer_cb(self) -> None:
        now = time.monotonic()
        if now - self.last_infer_time < float(self.args.inference_period_s):
            return
        if not self.fresh_inputs_ready():
            return
        self.last_infer_time = now
        try:
            self.process_frame()
        except Exception as exc:  # noqa: BLE001 - demo node records and keeps running.
            error = {"timestamp": now_iso(), "stage": "process_frame", "error": str(exc)}
            self.errors.append(error)
            write_json(self.output_dir / "errors.json", self.errors)
            self.get_logger().warn(f"process_frame failed: {exc}")

    def process_frame(self) -> None:
        assert self.latest_rgb is not None
        assert self.latest_depth is not None
        assert self.latest_camera_info is not None

        rgb_arr = canonical_rgb(image_msg_to_array(self.latest_rgb), self.latest_rgb.encoding)
        depth_raw = image_msg_to_array(self.latest_depth)
        depth_m, depth_scale_m = depth_to_meters(depth_raw, self.latest_depth.encoding, self.args.depth_scale_m)
        camera_info = camera_info_to_dict(self.latest_camera_info)
        odom = odom_to_dict(self.latest_odom)
        frame_bgr = rgb_arr[:, :, ::-1].copy()

        detections, latency_ms = run_inference(
            self.session,
            self.input_name,
            self.output_name,
            frame_bgr,
            self.args,
        )
        self.frame_count += 1
        self.inference_count += 1
        if not detections:
            if not self.args.no_visuals:
                draw_overlay(frame_bgr, [], self.output_dir / "latest_overlay.png")
            self.write_alarm_state(None, [], latency_ms)
            return

        localized: List[Dict[str, Any]] = []
        for det in detections:
            loc = localize_bbox_in_memory(
                det.get("bbox_xywh", []),
                depth_m,
                camera_info,
                self.args.min_depth_m,
                self.args.max_depth_m,
            )
            det = dict(det)
            det["depth_localization"] = loc
            if loc.get("depth_status") == "valid":
                det["depth_median_m"] = loc.get("depth_median_m")
                det["bbox_valid_depth_ratio"] = loc.get("bbox_valid_depth_ratio")
                det["camera_point_xyz_m"] = loc.get("camera_point_xyz_m")
            localized.append(det)

        localized = sorted(localized, key=lambda item: float(item.get("confidence", 0.0)), reverse=True)
        formal_candidates: List[Dict[str, Any]] = []
        for det in localized:
            gate_result = evaluate_auto_risk_gate(det, self.auto_risk_gates)
            det["auto_risk_gate"] = gate_result
            if gate_result.get("allowed"):
                formal_candidates.append(det)

        if not self.args.no_visuals:
            draw_overlay(frame_bgr, localized[: self.args.max_det], self.output_dir / "latest_overlay.png")

        new_events: List[Dict[str, Any]] = []
        for det in formal_candidates[: self.args.max_events_per_frame]:
            created = self.handle_detection(det, rgb_arr, depth_raw, depth_m, camera_info, odom, latency_ms, depth_scale_m)
            if created is not None:
                new_events.append(created)

        latest_alarm = new_events[0] if new_events else None
        if latest_alarm is None and formal_candidates:
            latest_alarm = self.transient_alarm_from_detection(formal_candidates[0])

        self.store.write_all()
        if not self.args.no_visuals:
            self.write_map_snapshot()
            self.write_episode_report(status="running")
            self.write_risk_report()
            self.write_dashboard()
        self.write_alarm_state(latest_alarm, localized, latency_ms)

    def transient_alarm_from_detection(self, det: Dict[str, Any]) -> Dict[str, Any]:
        class_name = str(det.get("class_name") or det.get("label") or "risk")
        event_type, risk_level, recommended_action = risk_info_for_class(class_name)
        return {
            "event_id": "current_detection_gate_pass_not_new_deduped",
            "class_name": class_name,
            "event_type": event_type,
            "risk_level": risk_level,
            "recommended_action": recommended_action,
            "auto_risk_gate": det.get("auto_risk_gate"),
            "confidence": det.get("confidence"),
            "distance_m": det.get("depth_median_m"),
            "projection_status": "current_detection_gate_pass",
        }

    def handle_detection(
        self,
        det: Dict[str, Any],
        rgb_arr: np.ndarray,
        depth_raw: np.ndarray,
        depth_m: np.ndarray,
        camera_info: Dict[str, Any],
        odom: Optional[Dict[str, Any]],
        latency_ms: Optional[float],
        depth_scale_m: float,
    ) -> Optional[Dict[str, Any]]:
        class_name = str(det.get("class_name") or det.get("label") or "risk")
        event_type, risk_level, recommended_action = risk_info_for_class(class_name)
        camera_point = det.get("camera_point_xyz_m")
        base_point = camera_point_to_base_point(camera_point) if isinstance(camera_point, dict) else None
        odom_point = base_point_to_odom_xy(base_point, odom) if base_point and odom else None
        projection_status = "projected" if odom_point else "missing_pose_or_depth"
        key = self.store.dedup_key(det, odom_point)
        event_id = f"risk_event_{now_id()}"
        distance_m = det.get("depth_median_m")

        map_point = {
            "map_point_id": f"map_point_{len(self.store.map_points) + 1:03d}",
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
            "projection_mode": "d435_bbox_depth_odom_approx",
            "axis_mapping": AXIS_MAPPING,
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
            "manual_arm_response_candidate": None,
        }
        is_new, stored = self.store.register(key, event, map_point)
        if not is_new:
            return None

        capture_dir = self.output_dir / "captures" / event_id
        capture_dir.mkdir(parents=True, exist_ok=True)
        rgb_path = capture_dir / "rgb.png"
        depth_raw_path = capture_dir / "depth_raw.npy"
        depth_vis_path = capture_dir / "depth_vis.png"
        overlay_path = capture_dir / "overlay.png"
        camera_info_path = capture_dir / "camera_info.json"
        odom_path = capture_dir / "odom.json"
        detection_path = capture_dir / "risk_detection.json"
        event_path = capture_dir / "risk_event.json"
        map_point_path = capture_dir / "risk_map_point.json"

        save_rgb_png(rgb_path, rgb_arr, "rgb8")
        np.save(depth_raw_path, depth_raw)
        save_depth_vis_png(depth_vis_path, depth_m)
        draw_overlay(rgb_arr[:, :, ::-1].copy(), [det], overlay_path)
        write_json(camera_info_path, camera_info)
        write_json(odom_path, odom)
        write_json(map_point_path, map_point)
        write_json(
            detection_path,
            {
                "risk_detection_id": f"risk_detection_{event_id}",
                "timestamp": now_iso(),
                "backend": "yolov8n_onnx_cpu",
                "runtime_backend": "onnxruntime",
                "model_path": str(self.args.model),
                "model_used": True,
                "local_inference": True,
                "online_api_used": False,
                "inference_executed": True,
                "latency_ms": None if latency_ms is None else round(float(latency_ms), 3),
                "fps": None if not latency_ms else round(1000.0 / float(latency_ms), 3),
                "depth_scale_m": depth_scale_m,
                "detections": [det],
                "claim_boundary": self.claim_boundary(),
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
        stored["manual_arm_response_candidate"] = self.build_manual_arm_candidate(stored, odom)
        write_json(event_path, stored)
        append_jsonl(self.output_dir / "risk_events.jsonl", stored)
        self.publish_event(stored)
        self.publish_alarm(stored)
        self.get_logger().info(
            "new risk event %s class=%s level=%s projection=%s"
            % (event_id, class_name, risk_level, projection_status)
        )
        return stored

    def publish_event(self, event: Dict[str, Any]) -> None:
        payload = {
            "event_type": event.get("event_type"),
            "type": event.get("event_type"),
            "source": "d435_yolo_prelim_demo",
            "distance_m": event.get("distance_m"),
            "confidence": event.get("confidence"),
            "class_name": event.get("class_name"),
            "risk_level_hint": event.get("risk_level"),
            "recommended_action_hint": event.get("recommended_action"),
            "event_id": event.get("event_id"),
            "projection_status": event.get("projection_status"),
            "odom_point_xy_m": event.get("odom_point_xy_m"),
            "timestamp": now_iso(),
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.event_pub.publish(msg)
        self.demo_event_pub.publish(msg)

    def publish_alarm(self, event: Dict[str, Any]) -> None:
        payload = {
            "alarm": True,
            "event_id": event.get("event_id"),
            "risk_level": event.get("risk_level"),
            "recommended_action": event.get("recommended_action"),
            "class_name": event.get("class_name"),
            "distance_m": event.get("distance_m"),
            "projection_status": event.get("projection_status"),
            "timestamp": now_iso(),
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.alarm_pub.publish(msg)

    def build_manual_arm_candidate(self, event: Dict[str, Any], odom: Optional[Dict[str, Any]]) -> Dict[str, Any]:
        if self.args.arm_response_mode == "disabled":
            return {"candidate_generated": False, "reason": "arm_response_mode=disabled"}
        if self.arm_candidate_count >= int(self.args.max_arm_candidates):
            return {"candidate_generated": False, "reason": "max_arm_candidates reached"}
        allowed_levels = {item.strip() for item in self.args.arm_risk_levels.split(",") if item.strip()}
        if event.get("risk_level") not in allowed_levels:
            return {
                "candidate_generated": False,
                "reason": f"risk_level {event.get('risk_level')} not in {sorted(allowed_levels)}",
            }

        arm_dir = Path(event["capture_dir"]) / "manual_arm_response_candidate"
        dry_run_cmd = [
            sys.executable,
            str(ROOT / "tools" / "run_arm_b3_no_load_sample_sequence.py"),
            "--output-dir",
            str(arm_dir / "dry_run"),
            "--serial-port",
            str(self.args.arm_serial_port),
            "--baudrate",
            str(int(self.args.arm_baudrate)),
        ]
        hardware_cmd = [
            sys.executable,
            str(ROOT / "tools" / "run_arm_b3_no_load_sample_sequence.py"),
            "--output-dir",
            str(arm_dir / "hardware"),
            "--serial-port",
            str(self.args.arm_serial_port),
            "--baudrate",
            str(int(self.args.arm_baudrate)),
            "--enable-hardware-write",
            "--confirm-no-load-sample-sequence",
        ]
        base_zero_now = odom_is_zero(
            odom,
            linear_eps=float(self.args.arm_base_zero_linear_eps),
            angular_eps=float(self.args.arm_base_zero_angular_eps),
        )
        candidate = {
            "candidate_generated": True,
            "candidate_id": f"{event.get('event_id')}_manual_arm_candidate",
            "generated_at": now_iso(),
            "source_event_id": event.get("event_id"),
            "operator_decides_when_where": True,
            "remote_control_required": True,
            "auto_execution_by_this_node": False,
            "selected_sequence": "arm_b3_8_step_safety_adjusted_no_load_sample",
            "target_final_pose": "safe_idle_home_like_6b",
            "base_zero_now_estimated": base_zero_now,
            "recommended_place": {
                "risk_odom_point_xy_m": event.get("odom_point_xy_m"),
                "projection_status": event.get("projection_status"),
                "instruction": (
                    "Operator remotely positions the robot at a safe response pose near the selected risk point, "
                    "stops the base, then manually starts the no-load arm response."
                ),
            },
            "required_before_manual_execution": [
                "Robot is stopped and base_zero is confirmed.",
                "Operator has selected the response location by remote control.",
                "No person or object is inside the arm workspace.",
                "This remains no-load: no grasping, payload handling, contact, or obstacle removal claim.",
                "Emergency stop and power cutoff are reachable.",
            ],
            "dry_run_command": dry_run_cmd,
            "hardware_command_after_operator_confirmation": hardware_cmd,
            "claim_boundary": [
                "This is a manual/remote response candidate only.",
                "The perception alarm does not autonomously control the arm.",
                "The command returns to safe_idle_home_like_6b if the underlying Arm-B3 sequence succeeds.",
            ],
        }
        self.arm_candidate_count += 1
        arm_dir.mkdir(parents=True, exist_ok=True)
        write_json(arm_dir / "manual_arm_response_candidate.json", candidate)
        return candidate

    def write_alarm_state(
        self,
        latest_event: Optional[Dict[str, Any]],
        detections: List[Dict[str, Any]],
        latency_ms: Optional[float],
    ) -> None:
        formal_count = sum(1 for det in detections if (det.get("auto_risk_gate") or {}).get("allowed"))
        state = {
            "updated_at": now_iso(),
            "alarm_active": latest_event is not None,
            "latest_event": latest_event,
            "current_detection_count": len(detections),
            "formal_detection_count": formal_count,
            "rejected_detection_count": max(0, len(detections) - formal_count),
            "auto_risk_gates": self.auto_risk_gates,
            "current_detections": [
                {
                    "class_name": det.get("class_name"),
                    "confidence": det.get("confidence"),
                    "distance_m": det.get("depth_median_m"),
                    "auto_risk_gate": det.get("auto_risk_gate"),
                }
                for det in detections[:10]
            ],
            "latency_ms": None if latency_ms is None else round(float(latency_ms), 3),
            "infer_fps": None if not latency_ms else round(1000.0 / float(latency_ms), 3),
            "event_count": len(self.store.events),
            "arm_candidate_count": self.arm_candidate_count,
        }
        write_json(self.output_dir / "alarm_state.json", state)

    def write_map_snapshot(self) -> None:
        points = self.store.map_points
        if not points:
            return
        try:
            import matplotlib

            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
        except Exception as exc:  # noqa: BLE001 - optional visualization.
            write_text(self.output_dir / "risk_map_snapshot_skipped.txt", str(exc))
            return

        fig, ax = plt.subplots(figsize=(7.0, 5.0))
        ax.set_title("Prelim Demo Risk Map Points")
        ax.set_xlabel("odom/map x (m)")
        ax.set_ylabel("odom/map y (m)")
        ax.grid(True, linestyle="--", alpha=0.35)
        for point in points:
            odom_xy = point.get("odom_point_xy_m") or {}
            robot_pose = point.get("robot_odom_pose") or {}
            robot_position = robot_pose.get("position") or {}
            try:
                rx = float(robot_position.get("x", 0.0))
                ry = float(robot_position.get("y", 0.0))
                px = float(odom_xy["x"])
                py = float(odom_xy["y"])
            except (KeyError, TypeError, ValueError):
                continue
            ax.scatter([rx], [ry], marker="o", color="#2b6cb0", s=28)
            ax.scatter([px], [py], marker="x", color="#c00000", s=72)
            ax.plot([rx, px], [ry, py], color="#888888", linewidth=0.8, alpha=0.7)
            ax.annotate(
                str(point.get("class_name")),
                (px, py),
                textcoords="offset points",
                xytext=(5, 5),
                fontsize=8,
            )
        ax.scatter([], [], marker="o", color="#2b6cb0", label="robot pose")
        ax.scatter([], [], marker="x", color="#c00000", label="risk point")
        ax.axis("equal")
        ax.legend(loc="best")
        fig.tight_layout()
        fig.savefig(self.output_dir / "risk_map_snapshot.png", dpi=160)
        plt.close(fig)

    def write_episode_report(self, status: str) -> None:
        captures = []
        risk_points = []
        action_results = []
        for event in self.store.events:
            captures.append(
                {
                    "capture_id": event.get("event_id"),
                    "captured_at": event.get("first_seen"),
                    "rgb_path": event.get("rgb_path"),
                    "overlay_path": event.get("overlay_path"),
                    "risk_detection_path": event.get("risk_detection_path"),
                    "used_for_mapping": True,
                    "used_for_training": False,
                    "pose_available": event.get("projection_status") == "projected",
                }
            )
            risk_points.append(
                {
                    "risk_point_id": event.get("event_id"),
                    "capture_id": event.get("event_id"),
                    "label": event.get("class_name"),
                    "risk_category": event.get("event_type"),
                    "depth_median_m": event.get("distance_m"),
                    "confidence": event.get("confidence"),
                    "odom_point_xy_m": event.get("odom_point_xy_m"),
                    "projection_status": event.get("projection_status"),
                    "evidence_paths": {
                        "rgb": event.get("rgb_path"),
                        "overlay": event.get("overlay_path"),
                        "risk_detection": event.get("risk_detection_path"),
                        "risk_map_point": event.get("risk_map_point_path"),
                    },
                }
            )
            action_results.append(
                {
                    "action_id": f"{event.get('event_id')}_alarm",
                    "action_type": "PRELIM_YOLO_RISK_ALARM",
                    "status": "succeeded",
                    "published_cmd_vel": False,
                    "hardware_executed": False,
                    "recommended_action": event.get("recommended_action"),
                    "manual_arm_response_candidate": event.get("manual_arm_response_candidate"),
                }
            )
        report = {
            "episode_id": "prelim_remote_mapping_yolo_arm_demo",
            "protocol_version": PROTOCOL_VERSION,
            "created_at": now_iso(),
            "status": status,
            "actions": [
                {
                    "action_id": "manual_remote_mapping",
                    "action_type": "REMOTE_GUARDED_MAPPING_EXTERNAL",
                    "publishes_cmd_vel": False,
                    "note": "Mapping motion is operator controlled through the existing guarded launch, not by this script.",
                },
                {
                    "action_id": "local_yolo_risk_monitor",
                    "action_type": "D435_LOCAL_YOLO_RISK_MONITOR",
                    "publishes_cmd_vel": False,
                    "model_path": str(self.args.model),
                },
            ],
            "captures": captures,
            "risk_points": risk_points,
            "action_results": action_results,
            "summary": {
                "event_count": len(self.store.events),
                "risk_map_points": len(self.store.map_points),
                "manual_arm_candidate_count": self.arm_candidate_count,
                "auto_risk_gates": self.auto_risk_gates,
                "cmd_vel_published_by_this_script": False,
                "local_yolo_used": True,
                "rl_controls_vehicle": False,
                "obstacle_removed_claimed": False,
                "arm_auto_executed_by_this_script": False,
            },
            "errors": self.errors,
            "claim_boundary": self.claim_boundary(),
        }
        write_json(self.output_dir / "episode_report.json", report)

    def write_risk_report(self) -> None:
        lines = [
            "# Preliminary Remote Mapping Risk Report",
            "",
            f"- generated_at: `{now_iso()}`",
            f"- event_count: `{len(self.store.events)}`",
            f"- projected_map_points: `{sum(1 for p in self.store.map_points if p.get('projection_status') == 'projected')}`",
            f"- manual_arm_candidate_count: `{self.arm_candidate_count}`",
            "- local_yolo_used: `true`",
            "- online_api_used: `false`",
            "- cmd_vel_published_by_this_script: `false`",
            "",
            "## Events",
            "",
        ]
        if not self.store.events:
            lines.append("- no risk events recorded yet")
        for event in self.store.events:
            lines.append(
                "- `{event_id}` {class_name} level={level} distance={distance}m projection={projection} action={action}".format(
                    event_id=event.get("event_id"),
                    class_name=event.get("class_name"),
                    level=event.get("risk_level"),
                    distance=event.get("distance_m"),
                    projection=event.get("projection_status"),
                    action=event.get("recommended_action"),
                )
            )
        lines.extend(
            [
                "",
                "## Auto Alarm / Map Gates",
                "",
            ]
        )
        for class_name, gate in sorted(self.auto_risk_gates.items()):
            lines.append(
                "- `{}`: confidence >= `{:.2f}`, `{:.2f}m <= depth <= {:.2f}m`".format(
                    class_name,
                    float(gate["min_confidence"]),
                    float(gate["min_depth_m"]),
                    float(gate["max_depth_m"]),
                )
            )
        lines.extend(
            [
                "",
                "## Boundary",
                "",
                "- This is a deterministic report generated from local event evidence.",
                "- It is suitable as the baseline input for a later local llama.cpp report backend.",
                "- Arm no-load response is operator-triggered by remote/manual control.",
            ]
        )
        write_text(self.output_dir / "risk_control_report.md", "\n".join(lines) + "\n")

    def write_dashboard(self) -> None:
        latest = self.store.events[-1] if self.store.events else {}
        overlay = latest.get("overlay_path")
        overlay_rel = None
        if overlay:
            try:
                overlay_rel = Path(overlay).resolve().relative_to(self.output_dir.resolve()).as_posix()
            except Exception:
                overlay_rel = overlay
        if overlay_rel is None and (self.output_dir / "latest_overlay.png").exists():
            overlay_rel = "latest_overlay.png"
        map_img = "risk_map_snapshot.png" if (self.output_dir / "risk_map_snapshot.png").exists() else None
        gate_rows = "\n".join(
            "<tr><td>{}</td><td>{:.2f}</td><td>{:.2f}-{:.2f} m</td></tr>".format(
                class_name,
                float(gate["min_confidence"]),
                float(gate["min_depth_m"]),
                float(gate["max_depth_m"]),
            )
            for class_name, gate in sorted(self.auto_risk_gates.items())
        )
        row_values = []
        for event in self.store.events[-10:]:
            gate = event.get("auto_risk_gate") or {}
            row_values.append(
                "<tr><td>{}</td><td>{}</td><td>{}</td><td>{}</td><td>{}</td><td>{}</td></tr>".format(
                    event.get("event_id"),
                    event.get("class_name"),
                    event.get("risk_level"),
                    event.get("distance_m"),
                    event.get("projection_status"),
                    gate.get("reason", ""),
                )
            )
        rows = "\n".join(row_values)
        html = f"""<!doctype html>
<html>
<head>
  <meta charset="utf-8">
  <meta http-equiv="refresh" content="2">
  <title>Prelim Remote Mapping Demo</title>
  <style>
    body {{ font-family: Arial, sans-serif; margin: 20px; background: #ffffff; color: #000000; }}
    .grid {{ display: grid; grid-template-columns: 1fr 1fr; gap: 16px; }}
    .panel {{ background: #ffffff; border: 2px solid #000000; border-radius: 8px; padding: 12px; color: #000000; }}
    img {{ max-width: 100%; border: 2px solid #000000; background: #ffffff; }}
    table {{ width: 100%; border-collapse: collapse; font-size: 13px; }}
    th, td {{ border-bottom: 1px solid #000000; text-align: left; padding: 6px; color: #000000; }}
    h1, h2, p, span, div {{ color: #000000; }}
    .alarm {{ color: #000000; background: #ffe8e8; font-weight: 700; padding: 4px 6px; display: inline-block; }}
    .ok {{ color: #000000; font-weight: 700; }}
    .muted {{ color: #000000; }}
  </style>
</head>
<body>
  <h1>Prelim Remote Mapping Demo</h1>
  <p>Events: {len(self.store.events)} | Manual arm candidates: {self.arm_candidate_count} | Updated: {now_iso()}</p>
  <div class="grid">
    <div class="panel">
      <h2>Latest YOLO Evidence</h2>
      {f'<img src="{overlay_rel}" alt="latest overlay">' if overlay_rel else '<p>No overlay saved yet.</p>'}
    </div>
    <div class="panel">
      <h2>Risk Map</h2>
      {f'<img src="{map_img}" alt="risk map">' if map_img else '<p>No map snapshot yet.</p>'}
    </div>
    <div class="panel">
      <h2>Alarm</h2>
      <p class="alarm">{latest.get('risk_level', 'none')} {latest.get('class_name', '')}</p>
      <p>Recommended action: {latest.get('recommended_action', 'none')}</p>
      <p>Projection: {latest.get('projection_status', 'none')}</p>
      <p class="muted">Only gate-passed crack/blockage detections are formal alarms/map points.</p>
    </div>
    <div class="panel">
      <h2>Auto Gates</h2>
      <table>
        <tr><th>class</th><th>min confidence</th><th>depth gate</th></tr>
        {gate_rows}
      </table>
    </div>
    <div class="panel">
      <h2>Recent Events</h2>
      <table>
        <tr><th>event</th><th>class</th><th>level</th><th>distance</th><th>projection</th><th>gate</th></tr>
        {rows}
      </table>
    </div>
  </div>
</body>
</html>
"""
        write_text(self.output_dir / "dashboard.html", html)

    def write_readme(self) -> None:
        text = f"""# Preliminary Remote Mapping YOLO Arm Demo

This directory contains live demo evidence generated by:

`tools/run_prelim_remote_mapping_yolo_arm_demo.py`

## What This Node Does

- subscribes to D435 RGB/depth/camera_info and `/odom`
- runs local YOLO ONNX inference
- deduplicates repeated risk detections
- promotes only configured class confidence/depth gate passes into formal alarms and map points
- publishes `/perception/mock_event`, `{self.args.demo_event_topic}`, and `{self.args.alarm_topic}`
- writes risk event, capture, alarm, map-point, and report evidence
- optionally starts one fixed arm no-load response

## Boundary

- Mapping motion is manual/remote and must use the existing guarded mapping launch.
- This node does not publish `cmd_vel`.
- Risk-point projection is approximate.
- Arm no-load response is manual/remote. This node only writes candidate files and commands.
- The demo does not claim RL control of the real vehicle.
"""
        write_text(self.output_dir / "README.md", text)

    def finalize(self) -> None:
        self.store.write_all()
        self.write_map_snapshot()
        self.write_episode_report(status="succeeded" if self.store.events else "completed_no_events")
        self.write_risk_report()
        self.write_dashboard()
        write_json(self.output_dir / "errors.json", self.errors)


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", default=DEFAULT_MODEL)
    parser.add_argument("--output-dir", default=DEFAULT_OUTPUT_DIR)
    parser.add_argument("--provider", choices=["auto", "spacemit", "cpu"], default="auto")
    parser.add_argument("--imgsz", type=int, default=320)
    parser.add_argument("--conf", type=float, default=0.20)
    parser.add_argument("--iou", type=float, default=0.70)
    parser.add_argument("--max-det", type=int, default=20)
    parser.add_argument("--max-events-per-frame", type=int, default=3)
    parser.add_argument("--timer-period-s", type=float, default=0.10)
    parser.add_argument("--inference-period-s", type=float, default=0.35)
    parser.add_argument("--fresh-timeout-s", type=float, default=2.0)
    parser.add_argument(
        "--no-visuals",
        action="store_true",
        help="Skip per-frame overlay/dashboard/map snapshot/report writes; keep JSON alarms/map points and event evidence.",
    )
    parser.add_argument("--opencv-num-threads", type=int, default=1)
    parser.add_argument(
        "--ort-graph-optimization-level",
        choices=["disabled", "basic", "extended", "all"],
        default="all",
    )
    parser.add_argument(
        "--ort-execution-mode",
        choices=["sequential", "parallel", "default"],
        default="sequential",
    )
    parser.add_argument("--ort-intra-op-threads", type=int, default=0)
    parser.add_argument("--ort-inter-op-threads", type=int, default=1)
    parser.add_argument("--ort-allow-spinning", choices=["default", "0", "1"], default="default")
    parser.add_argument("--min-depth-m", type=float, default=0.15)
    parser.add_argument("--max-depth-m", type=float, default=5.0)
    parser.add_argument(
        "--auto-risk-gates",
        default=DEFAULT_AUTO_RISK_GATES_SPEC,
        help="Comma-separated class:min_conf:min_depth_m:max_depth_m gates promoted to formal alarms/map points.",
    )
    parser.add_argument("--depth-scale-m", type=float, default=None)

    parser.add_argument("--rgb-topic", default="/camera/camera/color/image_raw")
    parser.add_argument("--depth-topic", default="/camera/camera/depth/image_rect_raw")
    parser.add_argument("--camera-info-topic", default="/camera/camera/color/camera_info")
    parser.add_argument("--odom-topic", default="/odom")
    parser.add_argument("--event-topic", default="/perception/mock_event")
    parser.add_argument("--demo-event-topic", default="/prelim_demo/risk_event")
    parser.add_argument("--alarm-topic", default="/prelim_demo/alarm")

    parser.add_argument("--dedup-map-grid-m", type=float, default=0.40)
    parser.add_argument("--dedup-image-grid-px", type=int, default=80)

    parser.add_argument("--arm-response-mode", choices=["disabled", "manual-candidate"], default="manual-candidate")
    parser.add_argument("--arm-risk-levels", default="high")
    parser.add_argument("--max-arm-candidates", type=int, default=1)
    parser.add_argument("--arm-serial-port", default="/dev/arm_bus")
    parser.add_argument("--arm-baudrate", type=int, default=9600)
    parser.add_argument("--arm-base-zero-linear-eps", type=float, default=0.03)
    parser.add_argument("--arm-base-zero-angular-eps", type=float, default=0.03)
    return parser.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    rclpy.init()
    node = None
    try:
        node = PrelimDemoNode(args)
        rclpy.spin(node)
        return 0
    except (KeyboardInterrupt, ExternalShutdownException):
        return 0
    finally:
        if node is not None:
            node.finalize()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
