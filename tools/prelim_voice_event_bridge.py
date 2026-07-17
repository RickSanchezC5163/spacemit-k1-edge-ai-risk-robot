#!/usr/bin/env python3
"""ROS2 bridge from prelim demo events to a SYN6288 serial TTS module."""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path
from typing import Any, Dict, Optional, Set

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT / "tools") not in sys.path:
    sys.path.insert(0, str(ROOT / "tools"))

from syn6288_serial_tts import CUE_TEXTS, Syn6288Client  # noqa: E402


def parse_json_or_text(data: str) -> Dict[str, Any]:
    text = data.strip()
    if not text:
        return {}
    try:
        obj = json.loads(text)
    except json.JSONDecodeError:
        return {"cue": text}
    if isinstance(obj, dict):
        return obj
    return {"cue": str(obj)}


class VoiceEventBridge:
    def __init__(self, args: argparse.Namespace) -> None:
        try:
            import rclpy
            from rclpy.node import Node
            from std_msgs.msg import String
        except ImportError as exc:
            raise RuntimeError("ROS2 Python packages are required for this bridge") from exc

        self.rclpy = rclpy
        self.String = String

        class _Node(Node):
            pass

        self.node = _Node("prelim_voice_event_bridge")
        self.args = args
        self.client = Syn6288Client(args.port, baudrate=args.baud, dry_run=args.dry_run)
        self.seen_events: Set[str] = set()
        self.last_cue_time: Dict[str, float] = {}

        self.node.create_subscription(String, args.alarm_topic, self._alarm_cb, 10)
        self.node.create_subscription(String, args.cue_topic, self._cue_cb, 10)

        if args.say_startup:
            self.say("startup")
        self.node.get_logger().info(
            "SYN6288 voice bridge ready: alarm_topic=%s cue_topic=%s port=%s dry_run=%s"
            % (args.alarm_topic, args.cue_topic, args.port, args.dry_run)
        )

    def close(self) -> None:
        self.client.close()

    def _cooldown_ok(self, cue: str) -> bool:
        now = time.monotonic()
        last = self.last_cue_time.get(cue, 0.0)
        if now - last < float(self.args.cooldown_s):
            return False
        self.last_cue_time[cue] = now
        return True

    def say(self, cue: str, text: Optional[str] = None, *, force: bool = False) -> None:
        if text is None:
            text = CUE_TEXTS.get(cue)
        if not text:
            self.node.get_logger().warn(f"unknown voice cue: {cue}")
            return
        if not force and not self._cooldown_ok(cue):
            return
        self.node.get_logger().info(f"voice cue={cue} text={text}")
        self.client.speak(
            text,
            volume=self.args.volume,
            background_volume=self.args.background_volume,
            speed=self.args.speed,
            music=self.args.music,
        )

    def _alarm_cb(self, msg: Any) -> None:
        payload = parse_json_or_text(str(msg.data))
        if payload.get("alarm") is False:
            return
        class_name = str(payload.get("class_name") or payload.get("label") or "").lower()
        event_id = str(payload.get("event_id") or "")
        if event_id and event_id in self.seen_events:
            return
        if class_name == "blockage":
            if event_id:
                self.seen_events.add(event_id)
            self.say("blockage_detected")
        elif self.args.announce_all_risks and class_name:
            text = f"发现{class_name}风险，正在生成风险点。"
            if event_id:
                self.seen_events.add(event_id)
            self.say(f"risk_{class_name}", text=text)

    def _cue_cb(self, msg: Any) -> None:
        payload = parse_json_or_text(str(msg.data))
        cue = str(payload.get("cue") or payload.get("event") or payload.get("stage") or "").strip()
        text = payload.get("text")
        force = bool(payload.get("force", False))
        if not cue and isinstance(text, str):
            cue = "custom"
        if not cue:
            return
        self.say(cue, text=str(text) if text is not None else None, force=force)


def main() -> int:
    parser = argparse.ArgumentParser(description="Bridge ROS2 risk events to SYN6288 serial TTS.")
    parser.add_argument("--port", default="/dev/ttyUSB0", help="SYN6288 serial device")
    parser.add_argument("--baud", type=int, default=9600)
    parser.add_argument("--alarm-topic", default="/perception/risk_alarm")
    parser.add_argument("--cue-topic", default="/prelim_demo/voice_cue")
    parser.add_argument("--volume", type=int, default=12)
    parser.add_argument("--background-volume", type=int, default=15)
    parser.add_argument("--speed", type=int, default=5)
    parser.add_argument("--music", type=int, default=0)
    parser.add_argument("--cooldown-s", type=float, default=4.0)
    parser.add_argument("--announce-all-risks", action="store_true")
    parser.add_argument("--say-startup", action="store_true")
    parser.add_argument("--dry-run", action="store_true", help="log frame hex instead of opening serial")
    args = parser.parse_args()

    try:
        import rclpy
    except ImportError as exc:
        raise SystemExit("ROS2 environment is not sourced; run source /opt/ros/humble/setup.bash") from exc

    rclpy.init()
    bridge = VoiceEventBridge(args)
    try:
        rclpy.spin(bridge.node)
    finally:
        bridge.close()
        bridge.node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
