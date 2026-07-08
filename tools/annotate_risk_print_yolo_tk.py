#!/usr/bin/env python3
"""Lightweight YOLO bbox annotator for risk-print dataset images.

It is intentionally small and local: open one image, drag a bbox, save YOLO
label + JSON sidecar, then advance to the next image.
"""

from __future__ import annotations

import argparse
import csv
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple


CLASS_NAMES = ["crack", "corrosion", "leakage", "blockage"]
CLASS_TO_ID = {name: idx for idx, name in enumerate(CLASS_NAMES)}
FILENAME_CLASS_HINTS = [
    ("crack", "crack"),
    ("rust", "corrosion"),
    ("corrosion", "corrosion"),
    ("leak", "leakage"),
    ("leakage", "leakage"),
    ("blockage", "blockage"),
]


@dataclass
class Box:
    label: str
    x1: float
    y1: float
    x2: float
    y2: float

    def normalized(self) -> "Box":
        return Box(
            label=self.label,
            x1=min(self.x1, self.x2),
            y1=min(self.y1, self.y2),
            x2=max(self.x1, self.x2),
            y2=max(self.y1, self.y2),
        )


def load_manifest_classes(dataset_dir: Path) -> Dict[str, str]:
    manifest = dataset_dir / "capture_manifest.csv"
    if not manifest.exists():
        return {}
    classes: Dict[str, str] = {}
    with manifest.open("r", encoding="utf-8", newline="") as handle:
        for row in csv.DictReader(handle):
            image_path = row.get("dataset_image_path", "")
            normalized = row.get("normalized_class", "")
            if image_path and normalized:
                classes[Path(image_path).stem] = normalized
    return classes


def infer_class(stem: str, manifest_classes: Dict[str, str]) -> str:
    if stem in manifest_classes:
        return manifest_classes[stem]
    lower = stem.lower()
    for token, class_name in FILENAME_CLASS_HINTS:
        if token in lower:
            return class_name
    return CLASS_NAMES[0]


def load_json_boxes(path: Path, default_label: str) -> List[Box]:
    if not path.exists():
        return []
    data = json.loads(path.read_text(encoding="utf-8"))
    if isinstance(data, dict):
        entries = [data]
    else:
        entries = data
    boxes: List[Box] = []
    for entry in entries:
        for ann in entry.get("annotations", []):
            label = ann.get("label") or default_label
            coords = ann.get("coordinates", {})
            cx = float(coords.get("x", 0.0))
            cy = float(coords.get("y", 0.0))
            width = float(coords.get("width", 0.0))
            height = float(coords.get("height", 0.0))
            if width <= 0 or height <= 0:
                continue
            boxes.append(Box(label=label, x1=cx - width / 2.0, y1=cy - height / 2.0, x2=cx + width / 2.0, y2=cy + height / 2.0))
    return boxes


def load_yolo_boxes(path: Path, image_size: Tuple[int, int], default_label: str) -> List[Box]:
    if not path.exists():
        return []
    width, height = image_size
    boxes: List[Box] = []
    for line in path.read_text(encoding="utf-8").splitlines():
        parts = line.strip().split()
        if len(parts) != 5:
            continue
        class_id = int(float(parts[0]))
        cx, cy, bw, bh = [float(value) for value in parts[1:]]
        label = CLASS_NAMES[class_id] if 0 <= class_id < len(CLASS_NAMES) else default_label
        x1 = (cx - bw / 2.0) * width
        y1 = (cy - bh / 2.0) * height
        x2 = (cx + bw / 2.0) * width
        y2 = (cy + bh / 2.0) * height
        boxes.append(Box(label=label, x1=x1, y1=y1, x2=x2, y2=y2))
    return boxes


def write_json_sidecar(path: Path, image_name: str, boxes: List[Box]) -> None:
    annotations = []
    for box in boxes:
        b = box.normalized()
        annotations.append(
            {
                "label": b.label,
                "coordinates": {
                    "x": (b.x1 + b.x2) / 2.0,
                    "y": (b.y1 + b.y2) / 2.0,
                    "width": b.x2 - b.x1,
                    "height": b.y2 - b.y1,
                },
            }
        )
    path.write_text(json.dumps([{"image": image_name, "annotations": annotations}], ensure_ascii=False), encoding="utf-8")


def write_yolo_label(path: Path, boxes: List[Box], image_size: Tuple[int, int]) -> None:
    width, height = image_size
    path.parent.mkdir(parents=True, exist_ok=True)
    lines = []
    for box in boxes:
        b = box.normalized()
        class_id = CLASS_TO_ID.get(b.label)
        if class_id is None:
            continue
        bw = max(0.0, min(width, b.x2) - max(0.0, b.x1))
        bh = max(0.0, min(height, b.y2) - max(0.0, b.y1))
        if bw <= 1 or bh <= 1:
            continue
        cx = (max(0.0, b.x1) + min(width, b.x2)) / 2.0 / width
        cy = (max(0.0, b.y1) + min(height, b.y2)) / 2.0 / height
        lines.append(f"{class_id} {cx:.6f} {cy:.6f} {bw / width:.6f} {bh / height:.6f}")
    path.write_text("\n".join(lines) + ("\n" if lines else ""), encoding="utf-8")


class AnnotatorApp:
    def __init__(self, dataset_dir: Path, split: str, auto_next: bool = True, start: Optional[str] = None) -> None:
        import tkinter as tk
        from tkinter import ttk
        from PIL import Image, ImageTk

        self.tk = tk
        self.ttk = ttk
        self.Image = Image
        self.ImageTk = ImageTk
        self.dataset_dir = dataset_dir
        self.split = split
        self.auto_next = auto_next
        self.image_dir = dataset_dir / "images" / split
        self.label_dir = dataset_dir / "labels" / split
        self.images = sorted(self.image_dir.glob("*.png")) + sorted(self.image_dir.glob("*.jpg")) + sorted(self.image_dir.glob("*.jpeg"))
        if not self.images:
            raise SystemExit(f"No images found: {self.image_dir}")
        self.manifest_classes = load_manifest_classes(dataset_dir)
        self.index = self._start_index(start)
        self.boxes: List[Box] = []
        self.current_label = None
        self.original_image = None
        self.display_image = None
        self.photo = None
        self.scale = 1.0
        self.offset_x = 0
        self.offset_y = 0
        self.drag_start: Optional[Tuple[float, float]] = None
        self.preview_rect = None

        self.root = tk.Tk()
        self.root.title("Risk Print YOLO Annotator")
        self.root.geometry("1100x780")
        self.status_var = tk.StringVar(value="")
        self.class_var = tk.StringVar(value=CLASS_NAMES[0])
        self._build_ui()
        self.load_current()

    def _start_index(self, start: Optional[str]) -> int:
        if not start:
            return 0
        for idx, path in enumerate(self.images):
            if path.name == start or path.stem == start:
                return idx
        return 0

    def _build_ui(self) -> None:
        tk = self.tk
        ttk = self.ttk
        root = self.root
        toolbar = ttk.Frame(root, padding=6)
        toolbar.pack(side=tk.TOP, fill=tk.X)
        ttk.Button(toolbar, text="Prev", command=self.prev_image).pack(side=tk.LEFT, padx=2)
        ttk.Button(toolbar, text="Save + Next", command=self.save_and_next).pack(side=tk.LEFT, padx=2)
        ttk.Button(toolbar, text="Next", command=self.next_image).pack(side=tk.LEFT, padx=2)
        ttk.Button(toolbar, text="Undo Box", command=self.undo_box).pack(side=tk.LEFT, padx=2)
        ttk.Button(toolbar, text="Clear Boxes", command=self.clear_boxes).pack(side=tk.LEFT, padx=2)
        ttk.Label(toolbar, text="Class").pack(side=tk.LEFT, padx=(16, 4))
        combo = ttk.Combobox(toolbar, textvariable=self.class_var, values=CLASS_NAMES, width=14, state="readonly")
        combo.pack(side=tk.LEFT)
        ttk.Label(toolbar, text="  Shortcuts: Ctrl+S save, Backspace undo, Left/Right navigate").pack(side=tk.LEFT, padx=12)

        self.canvas = tk.Canvas(root, background="#202020", cursor="crosshair")
        self.canvas.pack(side=tk.TOP, fill=tk.BOTH, expand=True)
        status = ttk.Label(root, textvariable=self.status_var, padding=6)
        status.pack(side=tk.BOTTOM, fill=tk.X)

        self.canvas.bind("<ButtonPress-1>", self.on_mouse_down)
        self.canvas.bind("<B1-Motion>", self.on_mouse_drag)
        self.canvas.bind("<ButtonRelease-1>", self.on_mouse_up)
        root.bind("<Control-s>", lambda _event: self.save_and_next())
        root.bind("<BackSpace>", lambda _event: self.undo_box())
        root.bind("<Left>", lambda _event: self.prev_image())
        root.bind("<Right>", lambda _event: self.next_image())
        root.bind("<Configure>", lambda _event: self.render())

    def current_paths(self) -> Tuple[Path, Path, Path]:
        image_path = self.images[self.index]
        return image_path, self.label_dir / f"{image_path.stem}.txt", image_path.with_suffix(".json")

    def load_current(self) -> None:
        image_path, label_path, json_path = self.current_paths()
        self.original_image = self.Image.open(image_path).convert("RGB")
        default_label = infer_class(image_path.stem, self.manifest_classes)
        self.class_var.set(default_label)
        yolo_boxes = load_yolo_boxes(label_path, self.original_image.size, default_label)
        self.boxes = yolo_boxes if yolo_boxes else load_json_boxes(json_path, default_label)
        self.render()

    def image_to_canvas(self, x: float, y: float) -> Tuple[float, float]:
        return self.offset_x + x * self.scale, self.offset_y + y * self.scale

    def canvas_to_image(self, x: float, y: float) -> Tuple[float, float]:
        assert self.original_image is not None
        width, height = self.original_image.size
        ix = max(0.0, min(width, (x - self.offset_x) / self.scale))
        iy = max(0.0, min(height, (y - self.offset_y) / self.scale))
        return ix, iy

    def render(self) -> None:
        if self.original_image is None:
            return
        canvas_w = max(100, self.canvas.winfo_width())
        canvas_h = max(100, self.canvas.winfo_height())
        img_w, img_h = self.original_image.size
        self.scale = min(canvas_w / img_w, canvas_h / img_h)
        disp_w = max(1, int(img_w * self.scale))
        disp_h = max(1, int(img_h * self.scale))
        self.offset_x = (canvas_w - disp_w) // 2
        self.offset_y = (canvas_h - disp_h) // 2
        self.display_image = self.original_image.resize((disp_w, disp_h))
        self.photo = self.ImageTk.PhotoImage(self.display_image)
        self.canvas.delete("all")
        self.canvas.create_image(self.offset_x, self.offset_y, image=self.photo, anchor=self.tk.NW)
        for box in self.boxes:
            b = box.normalized()
            x1, y1 = self.image_to_canvas(b.x1, b.y1)
            x2, y2 = self.image_to_canvas(b.x2, b.y2)
            self.canvas.create_rectangle(x1, y1, x2, y2, outline="#00ff66", width=2)
            self.canvas.create_text(x1 + 4, y1 + 4, text=b.label, fill="#00ff66", anchor=self.tk.NW)
        image_path, label_path, json_path = self.current_paths()
        self.status_var.set(
            f"{self.index + 1}/{len(self.images)}  {image_path.name}  "
            f"boxes={len(self.boxes)}  yolo={'yes' if label_path.exists() else 'no'}  json={'yes' if json_path.exists() else 'no'}"
        )

    def on_mouse_down(self, event: Any) -> None:
        self.drag_start = self.canvas_to_image(event.x, event.y)
        if self.preview_rect is not None:
            self.canvas.delete(self.preview_rect)
            self.preview_rect = None

    def on_mouse_drag(self, event: Any) -> None:
        if self.drag_start is None:
            return
        x1, y1 = self.image_to_canvas(*self.drag_start)
        ix2, iy2 = self.canvas_to_image(event.x, event.y)
        x2, y2 = self.image_to_canvas(ix2, iy2)
        if self.preview_rect is not None:
            self.canvas.delete(self.preview_rect)
        self.preview_rect = self.canvas.create_rectangle(x1, y1, x2, y2, outline="#ffcc00", width=2)

    def on_mouse_up(self, event: Any) -> None:
        if self.drag_start is None:
            return
        x1, y1 = self.drag_start
        x2, y2 = self.canvas_to_image(event.x, event.y)
        self.drag_start = None
        if self.preview_rect is not None:
            self.canvas.delete(self.preview_rect)
            self.preview_rect = None
        if abs(x2 - x1) < 3 or abs(y2 - y1) < 3:
            return
        self.boxes.append(Box(label=self.class_var.get(), x1=x1, y1=y1, x2=x2, y2=y2))
        self.render()

    def save_current(self) -> None:
        image_path, label_path, json_path = self.current_paths()
        assert self.original_image is not None
        write_yolo_label(label_path, self.boxes, self.original_image.size)
        write_json_sidecar(json_path, image_path.name, self.boxes)

    def save_and_next(self) -> None:
        self.save_current()
        if self.auto_next:
            self.next_image()
        else:
            self.render()

    def next_image(self) -> None:
        self.index = min(len(self.images) - 1, self.index + 1)
        self.load_current()

    def prev_image(self) -> None:
        self.index = max(0, self.index - 1)
        self.load_current()

    def undo_box(self) -> None:
        if self.boxes:
            self.boxes.pop()
            self.render()

    def clear_boxes(self) -> None:
        self.boxes = []
        self.render()

    def run(self) -> None:
        self.root.mainloop()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset-dir", default="datasets/risk_print_yolo_v1")
    parser.add_argument("--split", choices=["train", "val"], default="train")
    parser.add_argument("--start", default=None, help="Optional image file name or stem to start from.")
    parser.add_argument("--no-auto-next", action="store_true", help="Save without advancing to the next image.")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    app = AnnotatorApp(
        dataset_dir=Path(args.dataset_dir),
        split=args.split,
        auto_next=not args.no_auto_next,
        start=args.start,
    )
    app.run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
