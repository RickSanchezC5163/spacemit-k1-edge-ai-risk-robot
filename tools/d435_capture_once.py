#!/usr/bin/env python3
"""Capture one D435 RGB/depth/camera_info/odom evidence bundle."""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path
from typing import Any, Dict, Optional, Tuple

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image

try:
    import cv2  # type: ignore
except ImportError:  # pragma: no cover - depends on ROS image.
    cv2 = None

try:
    from PIL import Image as PilImage  # type: ignore
except ImportError:  # pragma: no cover - depends on local image stack.
    PilImage = None


ROOT = Path(__file__).resolve().parents[1]
SRC_DIR = ROOT / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from edge_robot_protocol import CaptureMeta, now_iso, to_jsonable  # noqa: E402


DEFAULT_OUTPUT_ROOT = ROOT / "outputs" / "p4x_d435_hold_capture_v1"


def write_json(path: Path, data: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(to_jsonable(data), ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )


def header_to_dict(msg: Any) -> Dict[str, Any]:
    stamp = getattr(msg.header, "stamp", None)
    return {
        "frame_id": str(getattr(msg.header, "frame_id", "")),
        "stamp": {
            "sec": int(getattr(stamp, "sec", 0)),
            "nanosec": int(getattr(stamp, "nanosec", 0)),
        },
    }


def yaw_from_quaternion(q: Any) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def odom_to_dict(msg: Optional[Odometry]) -> Optional[Dict[str, Any]]:
    if msg is None:
        return None
    pos = msg.pose.pose.position
    ori = msg.pose.pose.orientation
    twist = msg.twist.twist
    yaw = yaw_from_quaternion(ori)
    return {
        "header": header_to_dict(msg),
        "child_frame_id": str(msg.child_frame_id),
        "pose": {
            "position": {
                "x": float(pos.x),
                "y": float(pos.y),
                "z": float(pos.z),
            },
            "orientation": {
                "x": float(ori.x),
                "y": float(ori.y),
                "z": float(ori.z),
                "w": float(ori.w),
            },
            "yaw_rad": float(yaw),
            "yaw_deg": float(math.degrees(yaw)),
        },
        "twist": {
            "linear": {
                "x": float(twist.linear.x),
                "y": float(twist.linear.y),
                "z": float(twist.linear.z),
            },
            "angular": {
                "x": float(twist.angular.x),
                "y": float(twist.angular.y),
                "z": float(twist.angular.z),
            },
        },
    }


def camera_info_to_dict(msg: CameraInfo) -> Dict[str, Any]:
    return {
        "header": header_to_dict(msg),
        "height": int(msg.height),
        "width": int(msg.width),
        "distortion_model": str(msg.distortion_model),
        "d": [float(v) for v in msg.d],
        "k": [float(v) for v in msg.k],
        "r": [float(v) for v in msg.r],
        "p": [float(v) for v in msg.p],
        "binning_x": int(msg.binning_x),
        "binning_y": int(msg.binning_y),
        "roi": {
            "x_offset": int(msg.roi.x_offset),
            "y_offset": int(msg.roi.y_offset),
            "height": int(msg.roi.height),
            "width": int(msg.roi.width),
            "do_rectify": bool(msg.roi.do_rectify),
        },
    }


def image_dtype_channels(encoding: str) -> Tuple[np.dtype, int]:
    enc = encoding.lower()
    if enc in ("rgb8", "bgr8"):
        return np.dtype(np.uint8), 3
    if enc in ("rgba8", "bgra8"):
        return np.dtype(np.uint8), 4
    if enc in ("mono8", "8uc1"):
        return np.dtype(np.uint8), 1
    if enc in ("mono16", "16uc1"):
        return np.dtype(np.uint16), 1
    if enc == "32fc1":
        return np.dtype(np.float32), 1
    raise ValueError(f"unsupported image encoding: {encoding}")


def image_msg_to_array(msg: Image) -> np.ndarray:
    dtype, channels = image_dtype_channels(msg.encoding)
    raw = np.frombuffer(msg.data, dtype=dtype)
    if bool(msg.is_bigendian) != (sys.byteorder == "big") and dtype.itemsize > 1:
        raw = raw.byteswap()
    height = int(msg.height)
    width = int(msg.width)
    row_items = int(msg.step) // dtype.itemsize
    if row_items <= 0:
        raise ValueError(f"invalid image step: {msg.step}")
    try:
        rows = raw.reshape((height, row_items))
    except ValueError as exc:
        raise ValueError(
            f"image buffer shape mismatch height={height} step={msg.step} "
            f"encoding={msg.encoding} bytes={len(msg.data)}"
        ) from exc
    useful = rows[:, : width * channels]
    if channels == 1:
        return useful.reshape((height, width)).copy()
    return useful.reshape((height, width, channels)).copy()


def canonical_rgb(array: np.ndarray, encoding: str) -> np.ndarray:
    enc = encoding.lower()
    if enc == "rgb8":
        return array
    if enc == "bgr8":
        return array[:, :, ::-1]
    if enc == "rgba8":
        return array[:, :, :3]
    if enc == "bgra8":
        return array[:, :, [2, 1, 0]]
    if enc in ("mono8", "8uc1"):
        return np.repeat(array[:, :, None], 3, axis=2)
    raise ValueError(f"cannot save RGB PNG from encoding: {encoding}")


def save_rgb_png(path: Path, array: np.ndarray, encoding: str) -> None:
    rgb = canonical_rgb(array, encoding)
    path.parent.mkdir(parents=True, exist_ok=True)
    if cv2 is not None:
        bgr = rgb[:, :, ::-1]
        if not cv2.imwrite(str(path), bgr):
            raise RuntimeError(f"cv2 failed to write {path}")
        return
    if PilImage is not None:
        PilImage.fromarray(rgb).save(path)
        return
    raise RuntimeError("no PNG writer available; install python3-opencv or pillow")


def infer_depth_scale_m(encoding: str, dtype: np.dtype, configured: Optional[float]) -> float:
    if configured is not None:
        return float(configured)
    enc = encoding.lower()
    if enc in ("16uc1", "mono16") or dtype == np.dtype(np.uint16):
        return 0.001
    return 1.0


def depth_to_meters(
    array: np.ndarray,
    encoding: str,
    depth_scale_m: Optional[float] = None,
) -> Tuple[np.ndarray, float]:
    scale = infer_depth_scale_m(encoding, array.dtype, depth_scale_m)
    depth = array.astype(np.float32) * float(scale)
    valid = np.isfinite(depth) & (depth > 0.0)
    return np.where(valid, depth, np.nan), scale


def save_depth_vis_png(path: Path, depth_m: np.ndarray) -> Dict[str, Any]:
    valid = depth_m[np.isfinite(depth_m) & (depth_m > 0.0)]
    path.parent.mkdir(parents=True, exist_ok=True)
    if valid.size == 0:
        vis = np.zeros(depth_m.shape, dtype=np.uint8)
        vmin = None
        vmax = None
    else:
        vmin = float(np.percentile(valid, 2.0))
        vmax = float(np.percentile(valid, 98.0))
        if vmax <= vmin:
            vmax = vmin + 0.001
        normalized = (np.clip(depth_m, vmin, vmax) - vmin) / (vmax - vmin)
        normalized = np.nan_to_num(normalized, nan=0.0, posinf=1.0, neginf=0.0)
        vis = (normalized * 255.0).astype(np.uint8)
    if cv2 is not None:
        color = cv2.applyColorMap(vis, cv2.COLORMAP_TURBO)
        if not cv2.imwrite(str(path), color):
            raise RuntimeError(f"cv2 failed to write {path}")
    elif PilImage is not None:
        PilImage.fromarray(vis).save(path)
    else:
        raise RuntimeError("no PNG writer available; install python3-opencv or pillow")
    return {
        "valid_count": int(valid.size),
        "vis_min_m": vmin,
        "vis_max_m": vmax,
    }


class D435CaptureNode(Node):
    def __init__(
        self,
        rgb_topic: str,
        depth_topic: str,
        camera_info_topic: str,
        odom_topic: str,
        fresh_timeout_s: float = 2.0,
    ):
        super().__init__("d435_capture_once")
        self.rgb_topic = rgb_topic
        self.depth_topic = depth_topic
        self.camera_info_topic = camera_info_topic
        self.odom_topic = odom_topic
        self.fresh_timeout_s = float(fresh_timeout_s)

        self.latest_rgb: Optional[Image] = None
        self.latest_depth: Optional[Image] = None
        self.latest_camera_info: Optional[CameraInfo] = None
        self.latest_odom: Optional[Odometry] = None
        self.latest_rgb_time = 0.0
        self.latest_depth_time = 0.0
        self.latest_camera_info_time = 0.0
        self.latest_odom_time = 0.0

        self.create_subscription(Image, rgb_topic, self._rgb_cb, 10)
        self.create_subscription(Image, depth_topic, self._depth_cb, 10)
        self.create_subscription(CameraInfo, camera_info_topic, self._camera_info_cb, 10)
        self.create_subscription(Odometry, odom_topic, self._odom_cb, 20)

    def _rgb_cb(self, msg: Image) -> None:
        self.latest_rgb = msg
        self.latest_rgb_time = time.monotonic()

    def _depth_cb(self, msg: Image) -> None:
        self.latest_depth = msg
        self.latest_depth_time = time.monotonic()

    def _camera_info_cb(self, msg: CameraInfo) -> None:
        self.latest_camera_info = msg
        self.latest_camera_info_time = time.monotonic()

    def _odom_cb(self, msg: Odometry) -> None:
        self.latest_odom = msg
        self.latest_odom_time = time.monotonic()

    def spin_for(self, duration_s: float, step_s: float = 0.05) -> None:
        deadline = time.monotonic() + max(0.0, duration_s)
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=step_s)

    def capture_freshness(self) -> Dict[str, Any]:
        now = time.monotonic()
        return {
            "rgb_fresh": self.latest_rgb is not None
            and now - self.latest_rgb_time <= self.fresh_timeout_s,
            "depth_fresh": self.latest_depth is not None
            and now - self.latest_depth_time <= self.fresh_timeout_s,
            "camera_info_fresh": self.latest_camera_info is not None
            and now - self.latest_camera_info_time <= self.fresh_timeout_s,
            "odom_fresh": self.latest_odom is not None
            and now - self.latest_odom_time <= self.fresh_timeout_s,
            "age_s": {
                "rgb": None if self.latest_rgb is None else round(now - self.latest_rgb_time, 3),
                "depth": None
                if self.latest_depth is None
                else round(now - self.latest_depth_time, 3),
                "camera_info": None
                if self.latest_camera_info is None
                else round(now - self.latest_camera_info_time, 3),
                "odom": None
                if self.latest_odom is None
                else round(now - self.latest_odom_time, 3),
            },
        }

    def wait_for_capture_inputs(self, timeout_s: float) -> Dict[str, Any]:
        deadline = time.monotonic() + max(0.0, timeout_s)
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            freshness = self.capture_freshness()
            if all(
                freshness[key]
                for key in ("rgb_fresh", "depth_fresh", "camera_info_fresh", "odom_fresh")
            ):
                return freshness
        return self.capture_freshness()

    def capture_to_dir(
        self,
        capture_dir: Path,
        capture_id: str,
        action_id: str,
        sequence: int,
        timeout_s: float,
        depth_scale_m: Optional[float] = None,
    ) -> CaptureMeta:
        freshness = self.wait_for_capture_inputs(timeout_s)
        missing = [
            name
            for name, ok in (
                ("rgb", freshness["rgb_fresh"]),
                ("depth", freshness["depth_fresh"]),
                ("camera_info", freshness["camera_info_fresh"]),
                ("odom", freshness["odom_fresh"]),
            )
            if not ok
        ]
        if missing:
            raise RuntimeError(f"capture inputs unavailable or stale: {', '.join(missing)}")

        assert self.latest_rgb is not None
        assert self.latest_depth is not None
        assert self.latest_camera_info is not None

        capture_dir.mkdir(parents=True, exist_ok=True)
        rgb_path = capture_dir / "rgb.png"
        depth_raw_path = capture_dir / "depth_raw.npy"
        depth_vis_path = capture_dir / "depth_vis.png"
        camera_info_path = capture_dir / "camera_info.json"
        odom_path = capture_dir / "odom.json"
        meta_path = capture_dir / "capture_meta.json"

        rgb_array = image_msg_to_array(self.latest_rgb)
        depth_array = image_msg_to_array(self.latest_depth)
        depth_m, actual_depth_scale_m = depth_to_meters(
            depth_array,
            self.latest_depth.encoding,
            depth_scale_m,
        )

        save_rgb_png(rgb_path, rgb_array, self.latest_rgb.encoding)
        np.save(depth_raw_path, depth_array)
        depth_vis_stats = save_depth_vis_png(depth_vis_path, depth_m)
        camera_info = camera_info_to_dict(self.latest_camera_info)
        odom = odom_to_dict(self.latest_odom)
        write_json(camera_info_path, camera_info)
        write_json(odom_path, odom)

        rgb_header = header_to_dict(self.latest_rgb)
        depth_header = header_to_dict(self.latest_depth)
        depth_total_pixels = int(depth_m.size)
        valid_depth_count = int(depth_vis_stats["valid_count"])
        valid_depth_ratio = (
            0.0 if depth_total_pixels == 0 else valid_depth_count / depth_total_pixels
        )
        rgb_info = {
            "encoding": str(self.latest_rgb.encoding),
            "height": int(self.latest_rgb.height),
            "width": int(self.latest_rgb.width),
            "step": int(self.latest_rgb.step),
            "frame_id": rgb_header["frame_id"],
            "header_stamp": rgb_header["stamp"],
        }
        depth_info = {
            "encoding": str(self.latest_depth.encoding),
            "height": int(self.latest_depth.height),
            "width": int(self.latest_depth.width),
            "step": int(self.latest_depth.step),
            "dtype": str(depth_array.dtype),
            "depth_scale_m": float(actual_depth_scale_m),
            "frame_id": depth_header["frame_id"],
            "header_stamp": depth_header["stamp"],
            "valid_depth_ratio": float(valid_depth_ratio),
            **depth_vis_stats,
        }
        paths = {
            "capture_dir": str(capture_dir),
            "rgb": str(rgb_path),
            "depth_raw": str(depth_raw_path),
            "depth_vis": str(depth_vis_path),
            "camera_info": str(camera_info_path),
            "odom": str(odom_path),
            "capture_meta": str(meta_path),
        }
        meta = CaptureMeta(
            capture_id=capture_id,
            action_id=action_id,
            timestamp=now_iso(),
            topics={
                "rgb": self.rgb_topic,
                "depth": self.depth_topic,
                "camera_info": self.camera_info_topic,
                "odom": self.odom_topic,
            },
            paths=paths,
            rgb=rgb_info,
            depth=depth_info,
            camera_info={
                "frame_id": camera_info["header"]["frame_id"],
                "height": camera_info["height"],
                "width": camera_info["width"],
                "k": camera_info["k"],
                "distortion_model": camera_info["distortion_model"],
            },
            odom=odom,
            sequence=int(sequence),
            rgb_header_stamp=rgb_header["stamp"],
            depth_header_stamp=depth_header["stamp"],
            rgb_frame_id=rgb_header["frame_id"],
            depth_frame_id=depth_header["frame_id"],
            depth_encoding=str(self.latest_depth.encoding),
            depth_scale_m=float(actual_depth_scale_m),
            valid_depth_ratio=float(valid_depth_ratio),
        )
        meta_dict = to_jsonable(meta)
        write_json(meta_path, meta_dict)
        return meta


def next_capture_id(prefix: str = "capture") -> str:
    return f"{prefix}_{time.strftime('%Y%m%d_%H%M%S')}"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Capture one D435 RGB/depth/camera_info/odom evidence bundle."
    )
    parser.add_argument("--output-dir", default=str(DEFAULT_OUTPUT_ROOT))
    parser.add_argument("--capture-id", default=None)
    parser.add_argument("--rgb-topic", default="/camera/camera/color/image_raw")
    parser.add_argument("--depth-topic", default="/camera/camera/depth/image_rect_raw")
    parser.add_argument("--camera-info-topic", default="/camera/camera/color/camera_info")
    parser.add_argument("--odom-topic", default="/odom")
    parser.add_argument("--timeout-s", type=float, default=8.0)
    parser.add_argument("--fresh-timeout-s", type=float, default=2.0)
    parser.add_argument(
        "--depth-scale-m",
        type=float,
        default=None,
        help="Depth unit scale in meters. Defaults to 0.001 for 16UC1 and 1.0 otherwise.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    output_root = Path(args.output_dir)
    capture_id = args.capture_id or next_capture_id()
    capture_dir = output_root / "captures" / capture_id
    errors = []

    rclpy.init()
    node = D435CaptureNode(
        args.rgb_topic,
        args.depth_topic,
        args.camera_info_topic,
        args.odom_topic,
        fresh_timeout_s=args.fresh_timeout_s,
    )
    try:
        meta = node.capture_to_dir(
            capture_dir=capture_dir,
            capture_id=capture_id,
            action_id="manual_capture_once",
            sequence=1,
            timeout_s=args.timeout_s,
            depth_scale_m=args.depth_scale_m,
        )
        write_json(output_root / "errors.json", errors)
        print(json.dumps(to_jsonable(meta), ensure_ascii=False, indent=2))
        return 0
    except Exception as exc:  # noqa: BLE001 - CLI must write error evidence.
        error = {
            "timestamp": now_iso(),
            "tool": "d435_capture_once",
            "capture_id": capture_id,
            "error": str(exc),
        }
        errors.append(error)
        write_json(output_root / "errors.json", errors)
        write_json(capture_dir / "errors.json", errors)
        print(json.dumps(error, ensure_ascii=False, indent=2), file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
