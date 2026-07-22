#!/usr/bin/env python3
"""SYN6288 serial text-to-speech helper.

The vendor examples under ``K:/chrome/1778751105332853`` build SYN6288 frames as:

    FD LEN_H LEN_L 01 PARAM TEXT... XOR

where LEN is ``len(TEXT) + 3`` and XOR covers every byte before the checksum.
Default module baud rate is 9600 bps.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional


DEFAULT_PREFIX = "[d][V12][m15][t5]"

CUE_TEXTS: Dict[str, str] = {
    "startup": "语音播报模块已启动。",
    "blockage_detected": "发现障碍风险，正在生成风险点。",
    "risk_point_generated": "风险点已生成，正在规划靠近位置。",
    "approach_reached": "已到达处置距离，底盘锁定。",
    "base_zero": "底盘已锁定，准备执行机械臂动作。",
    "arm_clear_start": "机械臂开始执行清障动作。",
    "clear_done": "清障完成，正在复核风险状态。",
    "review_clear": "复核完成，障碍风险已解除。",
    "review_required": "复核完成，建议人工再次确认障碍状态。",
    "report_ready": "风险报告已生成。",
}

SEQUENCES: Dict[str, List[str]] = {
    "prelim_clearance": [
        "blockage_detected",
        "risk_point_generated",
        "approach_reached",
        "arm_clear_start",
        "clear_done",
        "report_ready",
    ],
}

COMMAND_FRAMES: Dict[str, bytes] = {
    "stop": bytes([0xFD, 0x00, 0x02, 0x02, 0xFD]),
    "pause": bytes([0xFD, 0x00, 0x02, 0x03, 0xFC]),
    "resume": bytes([0xFD, 0x00, 0x02, 0x04, 0xFB]),
    "status": bytes([0xFD, 0x00, 0x02, 0x21, 0xDE]),
    "power_down": bytes([0xFD, 0x00, 0x02, 0x88, 0x77]),
}

STATUS_BUSY = 0x4E
STATUS_IDLE = 0x4F


def xor_checksum(data: Iterable[int]) -> int:
    checksum = 0
    for byte in data:
        checksum ^= int(byte) & 0xFF
    return checksum & 0xFF


def build_speak_frame(
    text: str,
    *,
    music: int = 0,
    volume: int = 12,
    background_volume: int = 15,
    speed: int = 5,
    encoding: str = "gbk",
    prefix: Optional[str] = None,
) -> bytes:
    """Build a SYN6288 speak frame.

    ``PARAM`` follows the vendor STM32 example:
    ``0x01 | (music << 4)``. The text payload is GBK/GB2312 compatible.
    """

    music = max(0, min(15, int(music)))
    volume = max(0, min(16, int(volume)))
    background_volume = max(0, min(16, int(background_volume)))
    speed = max(0, min(5, int(speed)))
    if prefix is None:
        prefix = f"[d][V{volume}][m{background_volume}][t{speed}]"
    payload = (prefix + text).encode(encoding, errors="replace")
    if len(payload) > 0xFFFC:
        raise ValueError("SYN6288 payload is too long")
    length = len(payload) + 3
    header = bytes([0xFD, (length >> 8) & 0xFF, length & 0xFF, 0x01, 0x01 | (music << 4)])
    checksum = xor_checksum(header + payload)
    return header + payload + bytes([checksum])


def chunk_text_for_gbk(text: str, *, max_payload_bytes: int = 180, prefix: str = DEFAULT_PREFIX) -> List[str]:
    """Split text into chunks that keep SYN6288 frames short.

    The module accepts longer frames, but short chunks make live demos easier to
    interrupt and avoid overflowing low-cost example buffers.
    """

    chunks: List[str] = []
    current = ""
    prefix_len = len(prefix.encode("gbk", errors="replace"))
    for ch in text:
        candidate = current + ch
        if prefix_len + len(candidate.encode("gbk", errors="replace")) > max_payload_bytes and current:
            chunks.append(current)
            current = ch
        else:
            current = candidate
    if current:
        chunks.append(current)
    return chunks or [""]


@dataclass
class Syn6288Client:
    port: str
    baudrate: int = 9600
    timeout_s: float = 0.2
    dry_run: bool = False

    def __post_init__(self) -> None:
        self._serial = None
        if not self.dry_run:
            try:
                import serial  # type: ignore
            except ImportError as exc:
                raise RuntimeError(
                    "pyserial is required. Install with: python3 -m pip install pyserial"
                ) from exc
            self._serial = serial.Serial(
                self.port,
                self.baudrate,
                bytesize=8,
                parity="N",
                stopbits=1,
                timeout=self.timeout_s,
                write_timeout=self.timeout_s,
            )

    def close(self) -> None:
        if self._serial is not None:
            self._serial.close()

    def write_frame(self, frame: bytes) -> None:
        if self.dry_run:
            print(frame.hex(" ").upper())
            return
        assert self._serial is not None
        self._serial.write(frame)
        self._serial.flush()

    def reset_input(self) -> None:
        if self._serial is not None:
            self._serial.reset_input_buffer()

    def wait_until_idle(self, timeout_s: float = 15.0, poll_s: float = 0.1) -> None:
        """Wait for the module's 0x4F playback-complete response."""
        if self.dry_run:
            return
        assert self._serial is not None
        deadline = time.monotonic() + max(0.1, timeout_s)
        while time.monotonic() < deadline:
            self.write_frame(COMMAND_FRAMES["status"])
            query_deadline = min(deadline, time.monotonic() + max(0.05, poll_s))
            while time.monotonic() < query_deadline:
                waiting = self._serial.in_waiting
                response = self._serial.read(waiting or 1)
                if STATUS_IDLE in response:
                    return
                if STATUS_BUSY in response:
                    break
            time.sleep(max(0.01, poll_s))
        raise TimeoutError(f"SYN6288 did not become idle within {timeout_s:.1f}s")

    def command(self, name: str) -> None:
        frame = COMMAND_FRAMES.get(name)
        if frame is None:
            raise ValueError(f"unknown command: {name}")
        self.write_frame(frame)

    def speak(
        self,
        text: str,
        *,
        music: int = 0,
        volume: int = 12,
        background_volume: int = 15,
        speed: int = 5,
        encoding: str = "gbk",
        inter_chunk_delay_s: float = 0.15,
    ) -> None:
        prefix = f"[d][V{volume}][m{background_volume}][t{speed}]"
        chunks = chunk_text_for_gbk(text, prefix=prefix)
        for idx, chunk in enumerate(chunks):
            self.reset_input()
            frame = build_speak_frame(
                chunk,
                music=music,
                volume=volume,
                background_volume=background_volume,
                speed=speed,
                encoding=encoding,
                prefix=prefix,
            )
            self.write_frame(frame)
            if idx != len(chunks) - 1:
                time.sleep(max(0.0, inter_chunk_delay_s))


def list_ports() -> int:
    try:
        from serial.tools import list_ports as serial_list_ports  # type: ignore
    except ImportError:
        print("pyserial is not installed; cannot list serial ports", file=sys.stderr)
        return 2
    for port in serial_list_ports.comports():
        print(f"{port.device}\t{port.description}")
    return 0


def resolve_text(args: argparse.Namespace) -> List[str]:
    texts: List[str] = []
    if args.text:
        texts.extend(args.text)
    if args.cue:
        for cue in args.cue:
            if cue not in CUE_TEXTS:
                raise SystemExit(f"unknown cue {cue!r}; available: {', '.join(sorted(CUE_TEXTS))}")
            texts.append(CUE_TEXTS[cue])
    if args.sequence:
        for sequence in args.sequence:
            cues = SEQUENCES.get(sequence)
            if cues is None:
                raise SystemExit(f"unknown sequence {sequence!r}; available: {', '.join(sorted(SEQUENCES))}")
            texts.extend(CUE_TEXTS[cue] for cue in cues)
    return texts


def main() -> int:
    parser = argparse.ArgumentParser(description="Send Chinese TTS text to a SYN6288 serial module.")
    parser.add_argument("--port", default="/dev/ttyUSB0", help="serial port, e.g. /dev/ttyUSB0 or COM5")
    parser.add_argument("--baud", type=int, default=9600, help="SYN6288 default is 9600")
    parser.add_argument("--text", action="append", help="text to speak; can be repeated")
    parser.add_argument("--cue", action="append", help=f"predefined cue: {', '.join(sorted(CUE_TEXTS))}")
    parser.add_argument("--sequence", action="append", help=f"predefined sequence: {', '.join(sorted(SEQUENCES))}")
    parser.add_argument("--command", choices=sorted(COMMAND_FRAMES), help="send a control command")
    parser.add_argument("--volume", type=int, default=12)
    parser.add_argument("--background-volume", type=int, default=15)
    parser.add_argument("--speed", type=int, default=5)
    parser.add_argument("--music", type=int, default=0)
    parser.add_argument("--delay-s", type=float, default=0.8, help="delay between multiple texts")
    parser.add_argument("--wait", action="store_true", help="wait for the module's playback-complete response")
    parser.add_argument("--wait-timeout-s", type=float, default=15.0)
    parser.add_argument("--encoding", default="gbk", help="text encoding used inside SYN6288 frame")
    parser.add_argument("--dry-run", action="store_true", help="print frame hex instead of opening serial")
    parser.add_argument("--list-ports", action="store_true")
    parser.add_argument("--json", action="store_true", help="print cue table as JSON and exit")
    args = parser.parse_args()

    if args.json:
        print(json.dumps({"cues": CUE_TEXTS, "sequences": SEQUENCES}, ensure_ascii=False, indent=2))
        return 0
    if args.list_ports:
        return list_ports()

    texts = resolve_text(args)
    if not texts and not args.command:
        parser.error("provide --text, --cue, --sequence, --command, --list-ports, or --json")

    client = Syn6288Client(args.port, baudrate=args.baud, dry_run=args.dry_run)
    try:
        if args.command:
            client.command(args.command)
        for idx, text in enumerate(texts):
            client.speak(
                text,
                music=args.music,
                volume=args.volume,
                background_volume=args.background_volume,
                speed=args.speed,
                encoding=args.encoding,
            )
            if args.wait:
                client.wait_until_idle(timeout_s=args.wait_timeout_s)
            if idx != len(texts) - 1:
                time.sleep(max(0.0, args.delay_s))
    finally:
        client.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
