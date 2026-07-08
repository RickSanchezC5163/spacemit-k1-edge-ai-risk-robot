#!/usr/bin/env python3
"""Convert existing risk-print JSON annotations to YOLO txt labels."""

from __future__ import annotations

import argparse
from pathlib import Path

from PIL import Image

from annotate_risk_print_yolo_tk import infer_class, load_json_boxes, load_manifest_classes, write_yolo_label


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset-dir", default="datasets/risk_print_yolo_v1")
    parser.add_argument("--split", choices=["train", "val"], default="train")
    parser.add_argument("--overwrite", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    dataset_dir = Path(args.dataset_dir)
    image_dir = dataset_dir / "images" / args.split
    label_dir = dataset_dir / "labels" / args.split
    manifest_classes = load_manifest_classes(dataset_dir)
    converted = 0
    skipped_existing = 0
    skipped_empty = 0
    for json_path in sorted(image_dir.glob("*.json")):
        image_path = json_path.with_suffix(".png")
        if not image_path.exists():
            image_path = json_path.with_suffix(".jpg")
        if not image_path.exists():
            continue
        label_path = label_dir / f"{json_path.stem}.txt"
        if label_path.exists() and not args.overwrite:
            skipped_existing += 1
            continue
        default_label = infer_class(json_path.stem, manifest_classes)
        boxes = load_json_boxes(json_path, default_label)
        if not boxes:
            skipped_empty += 1
            continue
        size = Image.open(image_path).size
        write_yolo_label(label_path, boxes, size)
        converted += 1
    print(f"converted={converted} skipped_existing={skipped_existing} skipped_empty={skipped_empty}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
