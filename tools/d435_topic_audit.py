#!/usr/bin/env python3
"""Audit D435 RGB/depth/camera_info topics without publishing motion commands."""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path
from typing import Any, Dict, List, Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image


ROOT = Path(__file__).resolve().parents[1]
SRC_DIR = ROOT / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from edge_robot_protocol import now_iso, to_jsonable  # noqa: E402
from d435_capture_once import header_to_dict, write_json  # noqa: E402


DEFAULT_OUTPUT_ROOT = ROOT / "outputs" / "p4x_d435_hold_capture_v1"


class D435TopicAuditNode(Node):
    def __init__(
        self,
        rgb_topic: str,
        depth_topic: str,
        camera_info_topic: str,
    ):
        super().__init__("d435_topic_audit")
        self.rgb_topic = rgb_topic
        self.depth_topic = depth_topic
        self.camera_info_topic = camera_info_topic
        self.counts = {"rgb": 0, "depth": 0, "camera_info": 0}
        self.first_times: Dict[str, Optional[float]] = {
            "rgb": None,
            "depth": None,
            "camera_info": None,
        }
        self.last_times: Dict[str, Optional[float]] = {
            "rgb": None,
            "depth": None,
            "camera_info": None,
        }
        self.samples: Dict[str, Optional[Dict[str, Any]]] = {
            "rgb": None,
            "depth": None,
            "camera_info": None,
        }
        self.create_subscription(Image, rgb_topic, self._rgb_cb, 10)
        self.create_subscription(Image, depth_topic, self._depth_cb, 10)
        self.create_subscription(CameraInfo, camera_info_topic, self._camera_info_cb, 10)

    def _mark(self, key: str) -> None:
        now = time.monotonic()
        if self.first_times[key] is None:
            self.first_times[key] = now
        self.last_times[key] = now
        self.counts[key] += 1

    def _rgb_cb(self, msg: Image) -> None:
        self._mark("rgb")
        self.samples["rgb"] = image_sample(msg)

    def _depth_cb(self, msg: Image) -> None:
        self._mark("depth")
        self.samples["depth"] = image_sample(msg)

    def _camera_info_cb(self, msg: CameraInfo) -> None:
        self._mark("camera_info")
        self.samples["camera_info"] = {
            "header": header_to_dict(msg),
            "height": int(msg.height),
            "width": int(msg.width),
            "distortion_model": str(msg.distortion_model),
            "k": [float(v) for v in msg.k],
        }

    def spin_for(self, duration_s: float) -> None:
        deadline = time.monotonic() + max(0.0, duration_s)
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

    def topic_types(self) -> Dict[str, List[str]]:
        return {name: list(types) for name, types in self.get_topic_names_and_types()}

    def topic_result(self, key: str, topic: str, topic_types: Dict[str, List[str]]) -> Dict[str, Any]:
        first = self.first_times[key]
        last = self.last_times[key]
        duration = 0.0 if first is None or last is None else max(last - first, 0.0)
        hz = None
        if self.counts[key] > 1 and duration > 0.0:
            hz = round((self.counts[key] - 1) / duration, 3)
        return {
            "topic": topic,
            "present": topic in topic_types,
            "types": topic_types.get(topic, []),
            "readable": self.counts[key] > 0,
            "sample_count": int(self.counts[key]),
            "estimated_hz": hz,
            "sample": self.samples[key],
        }

    def result(self, sample_s: float) -> Dict[str, Any]:
        topic_types = self.topic_types()
        topics = {
            "rgb": self.topic_result("rgb", self.rgb_topic, topic_types),
            "depth": self.topic_result("depth", self.depth_topic, topic_types),
            "camera_info": self.topic_result(
                "camera_info", self.camera_info_topic, topic_types
            ),
        }
        errors = []
        for key, item in topics.items():
            if not item["present"]:
                errors.append({"topic_key": key, "topic": item["topic"], "error": "topic_missing"})
            elif not item["readable"]:
                errors.append(
                    {"topic_key": key, "topic": item["topic"], "error": "topic_unreadable"}
                )
        return {
            "tool": "d435_topic_audit",
            "timestamp": now_iso(),
            "sample_s": float(sample_s),
            "published_cmd_vel": False,
            "topics": topics,
            "overall_ok": len(errors) == 0,
            "errors": errors,
        }


def image_sample(msg: Image) -> Dict[str, Any]:
    return {
        "header": header_to_dict(msg),
        "height": int(msg.height),
        "width": int(msg.width),
        "encoding": str(msg.encoding),
        "is_bigendian": int(msg.is_bigendian),
        "step": int(msg.step),
        "data_len": len(msg.data),
    }


def markdown_report(result: Dict[str, Any]) -> str:
    lines = [
        "# D435 Topic Audit",
        "",
        f"- timestamp: `{result['timestamp']}`",
        f"- published_cmd_vel: `{result['published_cmd_vel']}`",
        f"- overall_ok: `{result['overall_ok']}`",
        "",
        "| key | topic | present | readable | samples | estimated_hz | encoding/model | size |",
        "| --- | --- | --- | --- | ---: | ---: | --- | --- |",
    ]
    for key, item in result["topics"].items():
        sample = item.get("sample") or {}
        encoding = sample.get("encoding") or sample.get("distortion_model") or ""
        size = ""
        if sample.get("width") and sample.get("height"):
            size = f"{sample['width']}x{sample['height']}"
        hz = "" if item.get("estimated_hz") is None else str(item["estimated_hz"])
        lines.append(
            "| {key} | `{topic}` | `{present}` | `{readable}` | {samples} | {hz} | {encoding} | {size} |".format(
                key=key,
                topic=item["topic"],
                present=item["present"],
                readable=item["readable"],
                samples=item["sample_count"],
                hz=hz,
                encoding=encoding,
                size=size,
            )
        )
    if result["errors"]:
        lines.extend(["", "## Errors", ""])
        for err in result["errors"]:
            lines.append(f"- `{err['topic']}`: {err['error']}")
    return "\n".join(lines) + "\n"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Audit D435 RGB/depth/camera_info topics.")
    parser.add_argument("--output-dir", default=str(DEFAULT_OUTPUT_ROOT))
    parser.add_argument("--rgb-topic", default="/camera/camera/color/image_raw")
    parser.add_argument("--depth-topic", default="/camera/camera/depth/image_rect_raw")
    parser.add_argument("--camera-info-topic", default="/camera/camera/color/camera_info")
    parser.add_argument("--sample-s", type=float, default=6.0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    output_root = Path(args.output_dir)
    output_root.mkdir(parents=True, exist_ok=True)

    rclpy.init()
    node = D435TopicAuditNode(args.rgb_topic, args.depth_topic, args.camera_info_topic)
    try:
        node.spin_for(args.sample_s)
        result = node.result(args.sample_s)
        errors = [
            {
                "timestamp": now_iso(),
                "tool": "d435_topic_audit",
                **err,
            }
            for err in result["errors"]
        ]
        write_json(output_root / "d435_topic_audit.json", result)
        (output_root / "d435_topic_audit.md").write_text(
            markdown_report(result),
            encoding="utf-8",
        )
        write_json(output_root / "errors.json", errors)
        print(json.dumps(to_jsonable(result), ensure_ascii=False, indent=2))
        return 0 if result["overall_ok"] else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
