#!/usr/bin/env python3
"""Prepare the D435 Windows risk-print captures as a YOLO dataset skeleton.

The source captures do not include bounding-box annotations. This script copies
valid RGB/depth evidence and creates the YOLO directory layout, but it marks the
dataset as not training-ready until manual bbox labels are added.
"""

from __future__ import annotations

import argparse
import csv
import json
import random
import shutil
from collections import Counter, defaultdict
from pathlib import Path
from statistics import median
from typing import Any, Dict, Iterable, List, Tuple


CLASS_MAP = {
    "crack": "crack",
    "rust": "corrosion",
    "corrosion": "corrosion",
    "leak": "leakage",
    "leakage": "leakage",
    "blockage": "blockage",
}

YOLO_CLASSES = ["crack", "corrosion", "leakage", "blockage"]


def read_manifest(path: Path) -> List[Dict[str, str]]:
    with path.open("r", encoding="utf-8", newline="") as handle:
        return list(csv.DictReader(handle))


def write_json(path: Path, data: Dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def rel(path: Path, base: Path) -> str:
    return path.relative_to(base).as_posix()


def parse_float(value: str) -> float | None:
    try:
        if value is None or value == "":
            return None
        return float(value)
    except ValueError:
        return None


def copy_capture_dir(src_dir: Path, dst_dir: Path) -> None:
    dst_dir.mkdir(parents=True, exist_ok=True)
    for item in src_dir.iterdir():
        if item.is_file():
            shutil.copy2(item, dst_dir / item.name)


def split_by_class(records: List[Dict[str, Any]], val_ratio: float, seed: int) -> None:
    rng = random.Random(seed)
    by_class: Dict[str, List[Dict[str, Any]]] = defaultdict(list)
    for record in records:
        by_class[record["normalized_class"]].append(record)
    for class_name, class_records in by_class.items():
        class_records.sort(key=lambda item: item["capture_id"])
        rng.shuffle(class_records)
        val_count = max(1, round(len(class_records) * val_ratio)) if len(class_records) > 1 else 0
        for idx, record in enumerate(class_records):
            record["split"] = "val" if idx < val_count else "train"


def quality_stats(records: List[Dict[str, Any]], skipped: Counter[str]) -> Dict[str, Any]:
    counts = Counter(record["normalized_class"] for record in records)
    split_counts = Counter(f"{record['split']}:{record['normalized_class']}" for record in records)
    center_depths = [
        value
        for value in (parse_float(record.get("center_depth_m", "")) for record in records)
        if value is not None
    ]
    manual_distance_count = sum(1 for record in records if parse_float(record.get("manual_distance_m", "")) is not None)
    depth_available = sum(1 for record in records if str(record.get("depth_available", "")).lower() == "true")
    camera_info_available = sum(1 for record in records if Path(record.get("camera_info_path", "")).exists())
    return {
        "source_records_used": len(records),
        "skipped": dict(skipped),
        "class_counts": {name: counts.get(name, 0) for name in YOLO_CLASSES},
        "split_counts": dict(split_counts),
        "depth_available_count": depth_available,
        "camera_info_available_count": camera_info_available,
        "manual_distance_present_count": manual_distance_count,
        "center_depth_m": {
            "count": len(center_depths),
            "min": round(min(center_depths), 3) if center_depths else None,
            "median": round(median(center_depths), 3) if center_depths else None,
            "max": round(max(center_depths), 3) if center_depths else None,
        },
        "annotation_status": "manual_bbox_required",
        "training_ready": False,
    }


def write_csv(path: Path, records: Iterable[Dict[str, Any]], fields: List[str]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for record in records:
            writer.writerow({field: record.get(field, "") for field in fields})


def write_data_yaml(path: Path, dataset_dir: Path) -> None:
    names = "\n".join(f"  {idx}: {name}" for idx, name in enumerate(YOLO_CLASSES))
    content = f"""# Risk print YOLO dataset skeleton.
# After manual LabelImg annotation, this file can be used from the dataset root:
#   cd datasets/risk_print_yolo_v1
#   yolo train model=yolov8n.pt data=data.yaml epochs=50 imgsz=640
path: .
train: images/train
val: images/val
names:
{names}
"""
    path.write_text(content, encoding="utf-8")


def write_readme(path: Path, stats: Dict[str, Any]) -> None:
    lines = [
        "# Risk Print YOLO v1 Dataset",
        "",
        "This directory is prepared from Windows-side D435 captures.",
        "",
        "## Status",
        "",
        "- RGB images have been split into `images/train` and `images/val`.",
        "- Raw D435 capture folders are copied under `captures_raw/<class>/`.",
        "- YOLO bbox labels must be supplied by manual LabelImg annotation.",
        "- Once `labels/train` and `labels/val` contain matching `.txt` files, use `data.yaml` for training.",
        "",
        "## Class Mapping",
        "",
        "- `rust` -> `corrosion`",
        "- `leak` -> `leakage`",
        "- `crack` -> `crack`",
        "- `blockage` -> `blockage`",
        "",
        "## Quality Summary",
        "",
        f"- usable images: {stats['source_records_used']}",
        f"- class counts: {stats['class_counts']}",
        f"- skipped: {stats['skipped']}",
        f"- depth available: {stats['depth_available_count']}/{stats['source_records_used']}",
        f"- camera_info available: {stats['camera_info_available_count']}/{stats['source_records_used']}",
        f"- manual distance present: {stats['manual_distance_present_count']}/{stats['source_records_used']}",
        f"- center depth summary: {stats['center_depth_m']}",
        "",
        "## Claim Boundary",
        "",
        "- This is a dataset organization step, not a trained model result.",
        "- Manual distance is capture metadata only and is not used for final mapping.",
        "- Depth and camera_info are retained as evidence, but YOLO training uses RGB images plus bbox labels.",
        "- The current labels directory is annotation-pending; empty labels must not be treated as negative samples.",
    ]
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def write_quality_report_md(path: Path, stats: Dict[str, Any]) -> None:
    class_counts = stats["class_counts"]
    lines = [
        "# Risk Print Dataset Quality Report",
        "",
        "## Verdict",
        "",
        "The captures are useful as a D435 risk-print image pool and annotation base, "
        "but the dataset is not YOLO-training-ready until real bounding-box labels are added.",
        "",
        "## Counts",
        "",
        f"- usable RGB images: {stats['source_records_used']}",
        f"- train/val seed: {stats['seed']}",
        f"- val ratio: {stats['val_ratio']}",
        f"- class counts: {class_counts}",
        f"- split counts: {stats['split_counts']}",
        "",
        "## Evidence Coverage",
        "",
        f"- depth available: {stats['depth_available_count']}/{stats['source_records_used']}",
        f"- camera_info available: {stats['camera_info_available_count']}/{stats['source_records_used']}",
        f"- center depth summary: {stats['center_depth_m']}",
        f"- manual distance present: {stats['manual_distance_present_count']}/{stats['source_records_used']}",
        "",
        "## Issues",
        "",
        f"- skipped source rows: {stats['skipped']}",
        "- `blockage` is underrepresented compared with the other classes.",
        "- bbox labels are not present; YOLO training must wait for manual annotation.",
        "- manual distance was not entered for these captures, but this does not block RGB bbox annotation.",
        "",
        "## Recommended Next Step",
        "",
        "Annotate each `images/train` and `images/val` RGB image with exactly the visible risk print/object "
        "region, then populate matching YOLO `.txt` files under `labels/train` and `labels/val`.",
    ]
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def prepare_dataset(source_root: Path, output_dir: Path, val_ratio: float, seed: int) -> Dict[str, Any]:
    manifest_path = source_root / "capture_manifest.csv"
    rows = read_manifest(manifest_path)
    records: List[Dict[str, Any]] = []
    skipped: Counter[str] = Counter()
    for row in rows:
        source_class = row.get("class_name", "").strip()
        normalized = CLASS_MAP.get(source_class)
        if normalized is None:
            skipped["unsupported_class"] += 1
            continue
        rgb_path = Path(row.get("rgb_path", ""))
        if not rgb_path.exists():
            skipped["missing_rgb"] += 1
            continue
        capture_dir = rgb_path.parent
        if not (capture_dir / "depth_raw.npy").exists():
            skipped["missing_depth_raw"] += 1
        if not (capture_dir / "camera_info.json").exists():
            skipped["missing_camera_info"] += 1
        record = dict(row)
        record["source_class"] = source_class
        record["normalized_class"] = normalized
        record["source_capture_dir"] = str(capture_dir)
        records.append(record)

    split_by_class(records, val_ratio=val_ratio, seed=seed)

    for class_name in YOLO_CLASSES:
        (output_dir / "captures_raw" / class_name).mkdir(parents=True, exist_ok=True)
    for split in ["train", "val"]:
        (output_dir / "images" / split).mkdir(parents=True, exist_ok=True)
        (output_dir / "labels" / split).mkdir(parents=True, exist_ok=True)
        (output_dir / "labels" / split / ".gitkeep").write_text("", encoding="utf-8")

    for record in records:
        capture_id = record["capture_id"]
        normalized = record["normalized_class"]
        split = record["split"]
        src_dir = Path(record["source_capture_dir"])
        raw_dst = output_dir / "captures_raw" / normalized / capture_id
        copy_capture_dir(src_dir, raw_dst)
        image_dst = output_dir / "images" / split / f"{capture_id}.png"
        shutil.copy2(src_dir / "rgb.png", image_dst)
        label_dst = output_dir / "labels" / split / f"{capture_id}.txt"
        record["dataset_image_path"] = rel(image_dst, output_dir)
        record["dataset_label_path"] = rel(label_dst, output_dir)
        record["dataset_raw_capture_dir"] = rel(raw_dst, output_dir)
        record["annotation_status"] = "manual_bbox_required"
        record["training_ready"] = False

    stats = quality_stats(records, skipped)
    stats["val_ratio"] = val_ratio
    stats["seed"] = seed

    fields = [
        "capture_id",
        "source_class",
        "normalized_class",
        "print_id",
        "manual_distance_m",
        "angle",
        "light",
        "split",
        "dataset_image_path",
        "dataset_label_path",
        "dataset_raw_capture_dir",
        "depth_available",
        "camera_info_path",
        "center_depth_m",
        "center_depth_valid_ratio",
        "center_distance_band",
        "annotation_status",
        "training_ready",
        "note",
    ]
    write_csv(output_dir / "capture_manifest.csv", records, fields)
    write_json(output_dir / "quality_report.json", stats)
    write_quality_report_md(output_dir / "quality_report.md", stats)
    write_readme(output_dir / "README.md", stats)
    write_data_yaml(output_dir / "data.yaml", output_dir)
    (output_dir / "labels" / "ANNOTATION_REQUIRED.md").write_text(
        "# Annotation Required\n\n"
        "Add real YOLO bbox labels before training. Do not create empty label files "
        "for positive images unless the image truly contains no target.\n",
        encoding="utf-8",
    )
    return stats


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-root", default="outputs/d435_dataset_win_v1")
    parser.add_argument("--output-dir", default="datasets/risk_print_yolo_v1")
    parser.add_argument("--val-ratio", type=float, default=0.2)
    parser.add_argument("--seed", type=int, default=20260702)
    parser.add_argument("--overwrite", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    source_root = Path(args.source_root)
    output_dir = Path(args.output_dir)
    if not (source_root / "capture_manifest.csv").exists():
        raise SystemExit(f"Missing source manifest: {source_root / 'capture_manifest.csv'}")
    if output_dir.exists():
        if not args.overwrite:
            raise SystemExit(f"Output exists, pass --overwrite to rebuild: {output_dir}")
        shutil.rmtree(output_dir)
    stats = prepare_dataset(source_root, output_dir, val_ratio=args.val_ratio, seed=args.seed)
    print(json.dumps(stats, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
