#!/usr/bin/env python3
"""Direct D435 latest-frameset source with detection-triggered depth alignment."""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from typing import Any, Dict, Optional

import numpy as np


@dataclass(frozen=True)
class RealSenseRawFrame:
    color_bgr: np.ndarray
    camera_info: Dict[str, Any]
    depth_scale_m: float
    captured_monotonic: float
    sensor_timestamp_ms: float
    timestamp_domain: str
    color_frame_number: int
    depth_frame_number: int
    frameset: Any


@dataclass(frozen=True)
class RealSenseAlignedDepth:
    depth_raw: np.ndarray
    camera_info: Dict[str, Any]
    depth_frame_number: int
    alignment_latency_ms: float


class RealSenseLatestFrameSource:
    """Keep only a three-slot ring of raw frameset references.

    The capture thread does not align depth and does not copy full images. The
    inference thread copies one color frame when requested and aligns depth only
    after a detection needs localization.
    """

    def __init__(
        self,
        width: int = 640,
        height: int = 480,
        fps: int = 15,
        slots: int = 3,
        wait_timeout_ms: int = 1500,
    ) -> None:
        if slots < 2:
            raise ValueError("slots must be at least 2")
        self.width = int(width)
        self.height = int(height)
        self.fps = int(fps)
        self.slots = int(slots)
        self.wait_timeout_ms = int(wait_timeout_ms)
        self._lock = threading.Lock()
        self._align_lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._pipeline = None
        self._align = None
        self._framesets = [None for _ in range(self.slots)]
        self._metadata = [None for _ in range(self.slots)]
        self._published_slot = -1
        self._write_slot = 0
        self._depth_scale_m = 0.001
        self._captured_frames = 0
        self._dropped_frames = 0
        self._color_copies = 0
        self._depth_alignments = 0
        self._alignment_latency_ms_total = 0.0
        self._last_error: Optional[str] = None

    def start(self) -> "RealSenseLatestFrameSource":
        if self._thread is not None:
            return self
        try:
            import pyrealsense2 as rs
        except ImportError as exc:
            raise RuntimeError(
                "pyrealsense2 is required for --frame-source realsense"
            ) from exc

        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(
            rs.stream.depth, self.width, self.height, rs.format.z16, self.fps
        )
        config.enable_stream(
            rs.stream.color, self.width, self.height, rs.format.bgr8, self.fps
        )
        profile = pipeline.start(config)
        depth_sensor = profile.get_device().first_depth_sensor()
        self._depth_scale_m = float(depth_sensor.get_depth_scale())
        self._pipeline = pipeline
        self._align = rs.align(rs.stream.color)
        self._thread = threading.Thread(
            target=self._capture_loop,
            name="d435-latest-frameset",
            daemon=True,
        )
        self._thread.start()
        return self

    @staticmethod
    def _camera_info(color_frame: Any) -> Dict[str, Any]:
        intrinsics = color_frame.profile.as_video_stream_profile().intrinsics
        return {
            "width": int(intrinsics.width),
            "height": int(intrinsics.height),
            "k": [
                float(intrinsics.fx), 0.0, float(intrinsics.ppx),
                0.0, float(intrinsics.fy), float(intrinsics.ppy),
                0.0, 0.0, 1.0,
            ],
            "d": [float(value) for value in intrinsics.coeffs],
            "distortion_model": str(intrinsics.model),
        }

    def _capture_loop(self) -> None:
        assert self._pipeline is not None
        while not self._stop.is_set():
            try:
                frames = self._pipeline.wait_for_frames(self.wait_timeout_ms)
                color_frame = frames.get_color_frame()
                depth_frame = frames.get_depth_frame()
                if not color_frame or not depth_frame:
                    self._dropped_frames += 1
                    continue
                slot = self._write_slot
                metadata = {
                    "camera_info": self._camera_info(color_frame),
                    "captured_monotonic": time.monotonic(),
                    "sensor_timestamp_ms": float(color_frame.get_timestamp()),
                    "timestamp_domain": str(color_frame.get_frame_timestamp_domain()),
                    "color_frame_number": int(color_frame.get_frame_number()),
                    "depth_frame_number": int(depth_frame.get_frame_number()),
                }
                with self._lock:
                    self._framesets[slot] = frames
                    self._metadata[slot] = metadata
                    self._published_slot = slot
                    self._write_slot = (slot + 1) % self.slots
                    self._captured_frames += 1
                    self._last_error = None
            except RuntimeError as exc:
                if not self._stop.is_set():
                    self._last_error = str(exc)
                    time.sleep(0.05)

    def get_latest(self, copy: bool = True) -> Optional[RealSenseRawFrame]:
        with self._lock:
            slot = self._published_slot
            if slot < 0 or self._metadata[slot] is None or self._framesets[slot] is None:
                return None
            metadata = dict(self._metadata[slot])
            frameset = self._framesets[slot]
        color_frame = frameset.get_color_frame()
        if not color_frame:
            return None
        color = np.asanyarray(color_frame.get_data())
        if color.shape != (self.height, self.width, 3):
            self._dropped_frames += 1
            return None
        if copy:
            color = color.copy()
            self._color_copies += 1
        return RealSenseRawFrame(
            color_bgr=color,
            camera_info=dict(metadata["camera_info"]),
            depth_scale_m=self._depth_scale_m,
            captured_monotonic=float(metadata["captured_monotonic"]),
            sensor_timestamp_ms=float(metadata["sensor_timestamp_ms"]),
            timestamp_domain=str(metadata["timestamp_domain"]),
            color_frame_number=int(metadata["color_frame_number"]),
            depth_frame_number=int(metadata["depth_frame_number"]),
            frameset=frameset,
        )

    def align_depth(self, frame: RealSenseRawFrame, copy: bool = True) -> RealSenseAlignedDepth:
        if self._align is None:
            raise RuntimeError("RealSense source is not started")
        started = time.perf_counter()
        with self._align_lock:
            aligned = self._align.process(frame.frameset)
        color_frame = aligned.get_color_frame()
        depth_frame = aligned.get_depth_frame()
        if not color_frame or not depth_frame:
            raise RuntimeError("aligned D435 frameset is missing color or depth")
        depth = np.asanyarray(depth_frame.get_data())
        if depth.shape != (self.height, self.width):
            raise RuntimeError(f"unexpected aligned depth shape: {depth.shape}")
        if copy:
            depth = depth.copy()
        latency_ms = (time.perf_counter() - started) * 1000.0
        with self._lock:
            self._depth_alignments += 1
            self._alignment_latency_ms_total += latency_ms
        return RealSenseAlignedDepth(
            depth_raw=depth,
            camera_info=self._camera_info(color_frame),
            depth_frame_number=int(depth_frame.get_frame_number()),
            alignment_latency_ms=latency_ms,
        )

    def status(self) -> Dict[str, Any]:
        with self._lock:
            published_slot = self._published_slot
            age_s = None
            if published_slot >= 0 and self._metadata[published_slot] is not None:
                age_s = time.monotonic() - float(
                    self._metadata[published_slot]["captured_monotonic"]
                )
            mean_align_ms = (
                self._alignment_latency_ms_total / self._depth_alignments
                if self._depth_alignments
                else 0.0
            )
            return {
                "source": "pyrealsense2_direct_raw_latest",
                "slots": self.slots,
                "captured_frames": self._captured_frames,
                "dropped_frames": self._dropped_frames,
                "color_copies": self._color_copies,
                "depth_alignments": self._depth_alignments,
                "mean_alignment_latency_ms": round(mean_align_ms, 3),
                "latest_age_s": None if age_s is None else round(age_s, 3),
                "last_error": self._last_error,
            }

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=max(2.0, self.wait_timeout_ms / 1000.0 + 0.5))
        if self._pipeline is not None:
            try:
                self._pipeline.stop()
            except RuntimeError:
                pass
        self._thread = None
        self._pipeline = None
        with self._lock:
            self._framesets = [None for _ in range(self.slots)]

    def __enter__(self) -> "RealSenseLatestFrameSource":
        return self.start()

    def __exit__(self, exc_type, exc, traceback) -> None:
        self.stop()
