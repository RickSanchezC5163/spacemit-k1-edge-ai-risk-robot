#!/usr/bin/env python3
"""Windows D435 dataset capture UI.

This tool is for training-set collection from a D435 connected directly to a
Windows laptop. Manual distance is dataset metadata only; it is not used for
final risk-point mapping.
"""

from __future__ import annotations

import argparse
import csv
import json
import os
import re
import threading
import time
import traceback
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, Optional


MANIFEST_FIELDS = [
    "capture_id",
    "class_name",
    "print_id",
    "manual_distance_m",
    "angle",
    "light",
    "rgb_path",
    "depth_path",
    "depth_vis_path",
    "camera_info_path",
    "meta_path",
    "depth_available",
    "pose_available",
    "used_for_training",
    "used_for_mapping",
    "center_depth_m",
    "center_depth_valid_ratio",
    "center_depth_roi_xywh",
    "center_distance_band",
    "note",
]


def now_id() -> str:
    return datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]


def safe_name(value: str, default: str = "unknown") -> str:
    value = (value or "").strip()
    if not value:
        value = default
    value = re.sub(r"[^A-Za-z0-9_.-]+", "_", value)
    return value.strip("._") or default


def write_json(path: Path, data: Dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def append_manifest(path: Path, row: Dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    exists = path.exists()
    with path.open("a", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=MANIFEST_FIELDS)
        if not exists:
            writer.writeheader()
        writer.writerow({field: row.get(field, "") for field in MANIFEST_FIELDS})


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


def depth_visualization(depth: Any) -> Any:
    import numpy as np
    from PIL import Image

    valid = depth[depth > 0]
    if valid.size == 0:
        scaled = np.zeros(depth.shape, dtype=np.uint8)
    else:
        lo = float(np.percentile(valid, 2))
        hi = float(np.percentile(valid, 98))
        if hi <= lo:
            hi = lo + 1.0
        scaled = np.clip((depth.astype(np.float32) - lo) * 255.0 / (hi - lo), 0, 255).astype(np.uint8)

    # Lightweight false-color map without adding an OpenCV dependency.
    t = scaled.astype(np.float32) / 255.0
    red = np.clip(1.5 - np.abs(4.0 * t - 3.0), 0.0, 1.0)
    green = np.clip(1.5 - np.abs(4.0 * t - 2.0), 0.0, 1.0)
    blue = np.clip(1.5 - np.abs(4.0 * t - 1.0), 0.0, 1.0)
    rgb = np.stack([red, green, blue], axis=-1)
    rgb[depth <= 0] = 0.0
    return Image.fromarray((rgb * 255.0).astype(np.uint8), mode="RGB")


def center_distance_band(center_depth_m: Optional[float]) -> str:
    if center_depth_m is None:
        return "unavailable"
    if center_depth_m < 0.4:
        return "very_close"
    if center_depth_m < 0.8:
        return "near"
    if center_depth_m < 1.2:
        return "medium"
    return "far"


def center_depth_summary(depth: Any, depth_scale_m: float) -> Dict[str, Any]:
    import numpy as np

    if depth is None:
        return {
            "center_depth_m": None,
            "center_depth_valid_ratio": 0.0,
            "center_depth_roi_xywh": None,
            "center_distance_band": "unavailable",
            "center_depth_source": "unavailable",
        }

    height, width = depth.shape[:2]
    roi_w = min(80, max(20, width // 8))
    roi_h = min(80, max(20, height // 8))
    x0 = max(0, (width - roi_w) // 2)
    y0 = max(0, (height - roi_h) // 2)
    roi = depth[y0 : y0 + roi_h, x0 : x0 + roi_w]
    valid = roi[roi > 0]
    if valid.size == 0:
        center_depth_m = None
    else:
        center_depth_m = round(float(np.median(valid)) * float(depth_scale_m), 3)
    return {
        "center_depth_m": center_depth_m,
        "center_depth_valid_ratio": round(float(valid.size) / float(roi.size), 4) if roi.size else 0.0,
        "center_depth_roi_xywh": [int(x0), int(y0), int(roi_w), int(roi_h)],
        "center_distance_band": center_distance_band(center_depth_m),
        "center_depth_source": "d435_depth_center_roi_median",
        "center_depth_note": "Approximate capture-quality check only; not used for map projection.",
    }


@dataclass
class CaptureDefaults:
    output_dir: Path
    class_name: str
    print_id: str
    manual_distance_m: Optional[float]
    angle: str
    light: str
    note: str
    width: int = 640
    height: int = 480
    fps: int = 30


class D435CaptureApp:
    def __init__(self, defaults: CaptureDefaults) -> None:
        import tkinter as tk
        from tkinter import ttk

        self.tk = tk
        self.ttk = ttk
        self.defaults = defaults
        self.root = tk.Tk()
        self.root.title("D435 Dataset Capture - Win")
        self.root.geometry("1060x720")
        self.pipeline = None
        self.align = None
        self.rs = None
        self.np = None
        self.Image = None
        self.ImageTk = None
        self.running = False
        self.latest_color = None
        self.latest_depth = None
        self.latest_color_profile = None
        self.latest_depth_profile = None
        self.depth_scale_m = 0.001
        self.status_var = tk.StringVar(value="Ready. Click Refresh Status.")
        self.preview_var = tk.StringVar(value="Preview not started")
        self.center_depth_var = tk.StringVar(value="Center depth: unavailable")

        self.output_dir_var = tk.StringVar(value=str(defaults.output_dir))
        self.class_var = tk.StringVar(value=defaults.class_name)
        self.print_var = tk.StringVar(value=defaults.print_id)
        self.distance_var = tk.StringVar(value="" if defaults.manual_distance_m is None else str(defaults.manual_distance_m))
        self.angle_var = tk.StringVar(value=defaults.angle)
        self.light_var = tk.StringVar(value=defaults.light)
        self.note_var = tk.StringVar(value=defaults.note)

        self._build_ui()
        self._load_optional_dependencies()
        self.refresh_status()

    def _build_ui(self) -> None:
        tk = self.tk
        ttk = self.ttk
        frame = ttk.Frame(self.root, padding=10)
        frame.pack(fill=tk.BOTH, expand=True)

        controls = ttk.LabelFrame(frame, text="Capture Metadata", padding=8)
        controls.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 10))

        rows = [
            ("Output dir", self.output_dir_var),
            ("Class name", self.class_var),
            ("Print ID", self.print_var),
            ("Manual distance m", self.distance_var),
            ("Angle", self.angle_var),
            ("Light", self.light_var),
            ("Note", self.note_var),
        ]
        for idx, (label, var) in enumerate(rows):
            ttk.Label(controls, text=label).grid(row=idx, column=0, sticky=tk.W, pady=3)
            ttk.Entry(controls, textvariable=var, width=34).grid(row=idx, column=1, sticky=tk.EW, pady=3)

        buttons = ttk.Frame(controls)
        buttons.grid(row=len(rows), column=0, columnspan=2, pady=12, sticky=tk.EW)
        ttk.Button(buttons, text="Refresh Status", command=self.refresh_status).pack(fill=tk.X, pady=2)
        ttk.Button(buttons, text="Start Preview", command=self.start_preview).pack(fill=tk.X, pady=2)
        ttk.Button(buttons, text="Stop Preview", command=self.stop_preview).pack(fill=tk.X, pady=2)
        ttk.Button(buttons, text="Capture", command=self.capture_current).pack(fill=tk.X, pady=2)
        ttk.Button(buttons, text="Open Output Dir", command=self.open_output_dir).pack(fill=tk.X, pady=2)

        status = ttk.LabelFrame(controls, text="Status / Boundary", padding=8)
        status.grid(row=len(rows) + 1, column=0, columnspan=2, sticky=tk.EW, pady=8)
        ttk.Label(status, textvariable=self.status_var, wraplength=330, justify=tk.LEFT).pack(anchor=tk.W)
        ttk.Label(
            status,
            text=(
                "manual_distance_m is training metadata only. Mapping uses "
                "bbox + depth + camera_info + odom/map pose."
            ),
            wraplength=330,
            justify=tk.LEFT,
        ).pack(anchor=tk.W, pady=(8, 0))

        preview = ttk.LabelFrame(frame, text="D435 RGB Preview", padding=8)
        preview.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)
        preview.grid_columnconfigure(0, weight=1)
        preview.grid_rowconfigure(1, weight=1)
        ttk.Label(
            preview,
            textvariable=self.center_depth_var,
            font=("Segoe UI", 12, "bold"),
        ).grid(row=0, column=0, sticky=tk.W, pady=(0, 6))
        self.preview_label = ttk.Label(preview, textvariable=self.preview_var, anchor=tk.CENTER)
        self.preview_label.grid(row=1, column=0, sticky=tk.NSEW)

    def _load_optional_dependencies(self) -> None:
        errors = []
        try:
            import pyrealsense2 as rs  # type: ignore

            self.rs = rs
        except Exception as exc:
            errors.append(f"pyrealsense2 unavailable: {exc}")
        try:
            import numpy as np  # type: ignore

            self.np = np
        except Exception as exc:
            errors.append(f"numpy unavailable: {exc}")
        try:
            from PIL import Image, ImageTk  # type: ignore

            self.Image = Image
            self.ImageTk = ImageTk
        except Exception as exc:
            errors.append(f"Pillow unavailable: {exc}")
        if errors:
            self.status_var.set("Dependency check: " + " | ".join(errors))

    def refresh_status(self) -> None:
        if self.rs is None:
            self.status_var.set(
                "D435 status unavailable: pyrealsense2 is not installed. "
                "Install Intel RealSense SDK / pyrealsense2 for Windows capture."
            )
            return
        try:
            ctx = self.rs.context()
            devices = ctx.query_devices()
            if len(devices) == 0:
                self.status_var.set("No RealSense device found.")
                return
            details = []
            for dev in devices:
                name = dev.get_info(self.rs.camera_info.name)
                serial = dev.get_info(self.rs.camera_info.serial_number)
                firmware = dev.get_info(self.rs.camera_info.firmware_version)
                details.append(f"{name} serial={serial} firmware={firmware}")
            self.status_var.set("RealSense device(s): " + " | ".join(details))
        except Exception as exc:
            self.status_var.set(f"RealSense status failed: {exc}")

    def start_preview(self) -> None:
        if self.running:
            return
        if self.rs is None or self.np is None:
            self.refresh_status()
            return
        try:
            config = self.rs.config()
            config.enable_stream(self.rs.stream.color, self.defaults.width, self.defaults.height, self.rs.format.rgb8, self.defaults.fps)
            config.enable_stream(self.rs.stream.depth, self.defaults.width, self.defaults.height, self.rs.format.z16, self.defaults.fps)
            self.pipeline = self.rs.pipeline()
            profile = self.pipeline.start(config)
            self.align = self.rs.align(self.rs.stream.color)
            self.latest_color_profile = profile.get_stream(self.rs.stream.color)
            self.latest_depth_profile = profile.get_stream(self.rs.stream.depth)
            try:
                self.depth_scale_m = float(profile.get_device().first_depth_sensor().get_depth_scale())
            except Exception:
                self.depth_scale_m = 0.001
            self.running = True
            self.preview_var.set("Starting preview...")
            self.root.after(30, self._update_preview)
        except Exception as exc:
            self.pipeline = None
            self.running = False
            self.status_var.set(f"Failed to start D435 preview: {exc}")

    def stop_preview(self) -> None:
        self.running = False
        if self.pipeline is not None:
            try:
                self.pipeline.stop()
            except Exception:
                pass
        self.pipeline = None
        self.preview_var.set("Preview stopped")
        self.center_depth_var.set("Center depth: unavailable")

    def _update_preview(self) -> None:
        if not self.running or self.pipeline is None:
            return
        try:
            frames = self.pipeline.poll_for_frames()
            if frames:
                aligned = self.align.process(frames) if self.align is not None else frames
                color_frame = aligned.get_color_frame()
                depth_frame = aligned.get_depth_frame()
                if color_frame:
                    self.latest_color = self.np.asanyarray(color_frame.get_data()).copy()
                if depth_frame:
                    self.latest_depth = self.np.asanyarray(depth_frame.get_data()).copy()
                if self.latest_color is not None and self.Image is not None and self.ImageTk is not None:
                    img = self.Image.fromarray(self.latest_color, mode="RGB")
                    img.thumbnail((720, 540))
                    photo = self.ImageTk.PhotoImage(img)
                    self.preview_label.configure(image=photo, text="")
                    self.preview_label.image = photo
                    self.preview_var.set("")
                if self.latest_depth is not None:
                    depth_summary = center_depth_summary(self.latest_depth, self.depth_scale_m)
                    center_depth = depth_summary.get("center_depth_m")
                    valid_ratio = depth_summary.get("center_depth_valid_ratio", 0.0)
                    distance_band = depth_summary.get("center_distance_band", "unavailable")
                    if center_depth is None:
                        self.center_depth_var.set(
                            f"Center distance approx: unavailable | band={distance_band} | valid depth ratio={valid_ratio}"
                        )
                    else:
                        self.center_depth_var.set(
                            f"Center distance approx: {center_depth:.3f} m | band={distance_band} | "
                            f"valid depth ratio={valid_ratio}"
                        )
            self.root.after(30, self._update_preview)
        except Exception as exc:
            self.status_var.set(f"Preview update failed: {exc}")
            self.stop_preview()

    def _manual_distance(self) -> Optional[float]:
        value = self.distance_var.get().strip()
        if not value:
            return None
        return float(value)

    def _current_meta_values(self) -> Dict[str, Any]:
        return {
            "class_name": safe_name(self.class_var.get(), "unknown"),
            "print_id": safe_name(self.print_var.get(), "print_unknown"),
            "manual_distance_m": self._manual_distance(),
            "angle": self.angle_var.get().strip(),
            "light": self.light_var.get().strip(),
            "note": self.note_var.get().strip(),
        }

    def capture_current(self) -> None:
        if self.latest_color is None:
            self.status_var.set("No RGB frame available. Start preview and wait for live frames first.")
            return
        try:
            meta_values = self._current_meta_values()
        except ValueError:
            self.status_var.set("Manual distance must be a number in meters, for example 0.8, or left blank.")
            return
        capture_id = f"{now_id()}_{meta_values['class_name']}_{meta_values['print_id']}"
        out_root = Path(self.output_dir_var.get()).expanduser()
        capture_dir = out_root / "captures" / capture_id
        capture_dir.mkdir(parents=True, exist_ok=True)
        rgb_path = capture_dir / "rgb.png"
        depth_path = capture_dir / "depth_raw.npy"
        depth_vis_path = capture_dir / "depth_vis.png"
        camera_info_path = capture_dir / "camera_info.json"
        camera_info_available = False
        meta_path = capture_dir / "meta.json"

        try:
            from PIL import Image

            Image.fromarray(self.latest_color, mode="RGB").save(rgb_path)
            depth_available = self.latest_depth is not None and self.np is not None
            if depth_available:
                self.np.save(depth_path, self.latest_depth)
                depth_visualization(self.latest_depth).save(depth_vis_path)
                center_depth = center_depth_summary(self.latest_depth, self.depth_scale_m)
            else:
                depth_path = Path("")
                depth_vis_path = Path("")
                center_depth = center_depth_summary(None, self.depth_scale_m)

            if self.latest_color_profile is not None:
                camera_info = camera_info_from_profile(self.latest_color_profile)
                write_json(camera_info_path, camera_info)
                camera_info_available = True
            else:
                camera_info_available = False

            meta = {
                "capture_id": capture_id,
                "captured_at": datetime.now().isoformat(timespec="milliseconds"),
                **meta_values,
                "rgb_path": str(rgb_path),
                "depth_path": str(depth_path) if depth_available else None,
                "depth_vis_path": str(depth_vis_path) if depth_available else None,
                "camera_info_path": str(camera_info_path) if camera_info_available else None,
                "depth_available": bool(depth_available),
                "depth_scale_m": self.depth_scale_m if depth_available else None,
                **center_depth,
                "pose_available": False,
                "used_for_training": True,
                "used_for_mapping": False,
                "distance_source": "manual_for_dataset_coverage",
                "claim_boundary": [
                    "manual_distance_m is for dataset coverage statistics only.",
                    "YOLO training does not use manual distance labels.",
                    "Final map projection must use bbox + depth + camera_info + odom/map pose, not manual_distance_m.",
                ],
            }
            write_json(meta_path, meta)
            append_manifest(
                out_root / "capture_manifest.csv",
                {
                    **meta_values,
                    "capture_id": capture_id,
                    "rgb_path": str(rgb_path),
                    "depth_path": str(depth_path) if depth_available else "",
                    "depth_vis_path": str(depth_vis_path) if depth_available else "",
                    "camera_info_path": str(camera_info_path) if camera_info_available else "",
                    "meta_path": str(meta_path),
                    "depth_available": bool(depth_available),
                    "pose_available": False,
                    "used_for_training": True,
                    "used_for_mapping": False,
                    "center_depth_m": center_depth.get("center_depth_m"),
                    "center_depth_valid_ratio": center_depth.get("center_depth_valid_ratio"),
                    "center_depth_roi_xywh": json.dumps(center_depth.get("center_depth_roi_xywh"), ensure_ascii=False),
                    "center_distance_band": center_depth.get("center_distance_band"),
                },
            )
            self.status_var.set(f"Captured {capture_id}")
        except Exception as exc:
            self.status_var.set(f"Capture failed: {exc}\n{traceback.format_exc(limit=1)}")

    def open_output_dir(self) -> None:
        path = Path(self.output_dir_var.get()).expanduser()
        path.mkdir(parents=True, exist_ok=True)
        try:
            os.startfile(str(path))  # type: ignore[attr-defined]
        except Exception as exc:
            self.status_var.set(f"Open output dir failed: {exc}")

    def run(self) -> None:
        try:
            self.root.mainloop()
        finally:
            self.stop_preview()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", default="outputs/d435_dataset_win_v1")
    parser.add_argument("--class-name", default="unknown")
    parser.add_argument("--print-id", default="print_unknown")
    parser.add_argument("--manual-distance-m", "--distance-m", dest="manual_distance_m", type=float, default=None)
    parser.add_argument("--angle", default="front")
    parser.add_argument("--light", default="normal")
    parser.add_argument("--note", default="")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--status-only", action="store_true", help="Print dependency/device status without opening the UI.")
    return parser.parse_args()


def status_only() -> int:
    try:
        import pyrealsense2 as rs  # type: ignore
    except Exception as exc:
        print(f"pyrealsense2 unavailable: {exc}")
        return 2
    try:
        ctx = rs.context()
        devices = ctx.query_devices()
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
    except Exception as exc:
        print(f"RealSense status failed: {exc}")
        return 1


def main() -> int:
    args = parse_args()
    if args.status_only:
        return status_only()
    defaults = CaptureDefaults(
        output_dir=Path(args.output_dir),
        class_name=args.class_name,
        print_id=args.print_id,
        manual_distance_m=args.manual_distance_m,
        angle=args.angle,
        light=args.light,
        note=args.note,
        width=args.width,
        height=args.height,
        fps=args.fps,
    )
    app = D435CaptureApp(defaults)
    app.run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
