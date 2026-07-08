#!/usr/bin/env python3
"""Manual review UI for YOLO risk detection event overlays.

The tool serves a local browser UI for reviewing saved event frames from a
prelim risk-mapping run. Labels are written back to the run directory as JSON,
CSV, and threshold-suggestion reports. It does not start ROS, publish cmd_vel,
or access robot hardware.
"""

from __future__ import annotations

import argparse
import csv
import json
import mimetypes
import sys
import time
import urllib.parse
import webbrowser
from collections import Counter, defaultdict
from datetime import datetime, timezone
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_RUN_DIR = (
    ROOT
    / "outputs"
    / "k1_pull"
    / "prelim_remote_mapping_yolo_arm_demo_v1"
    / "live_cpu_480_20260703_101632_final"
)

LABELS = {
    "correct": "正确",
    "wrong": "错误",
    "class_wrong": "类别错/框基本对",
    "uncertain": "不确定",
    "duplicate": "重复/无效",
}
NEGATIVE_LABELS = {"wrong", "class_wrong", "duplicate"}
POSITIVE_LABELS = {"correct"}
IGNORED_LABELS = {"uncertain"}
CLASSES = ["crack", "corrosion", "leakage", "blockage"]


def now_iso() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


def write_json(path: Path, data: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def read_json(path: Path, default: Any) -> Any:
    if not path.exists():
        return default
    return json.loads(path.read_text(encoding="utf-8"))


def safe_float(value: Any, default: float = 0.0) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def safe_int(value: Any, default: int = 0) -> int:
    try:
        return int(value)
    except (TypeError, ValueError):
        return default


def local_capture_path(run_dir: Path, event_id: str, name: str) -> Path:
    return run_dir / "captures" / event_id / name


def load_events(run_dir: Path, sort_mode: str) -> List[Dict[str, Any]]:
    index_path = run_dir / "risk_event_index.json"
    if not index_path.exists():
        raise SystemExit(f"risk_event_index.json not found: {index_path}")
    index = read_json(index_path, {})
    events = list(index.get("events", []))
    for event in events:
        event_id = str(event.get("event_id"))
        event["_overlay_exists"] = local_capture_path(run_dir, event_id, "overlay.png").exists()
        event["_rgb_exists"] = local_capture_path(run_dir, event_id, "rgb.png").exists()
        event["_depth_exists"] = local_capture_path(run_dir, event_id, "depth_vis.png").exists()
    if sort_mode == "confidence":
        events.sort(key=lambda item: safe_float(item.get("confidence")), reverse=True)
    elif sort_mode == "confidence_max":
        events.sort(key=lambda item: safe_float(item.get("confidence_max")), reverse=True)
    elif sort_mode == "class":
        events.sort(key=lambda item: (str(item.get("class_name")), item.get("first_seen") or ""))
    else:
        events.sort(key=lambda item: item.get("first_seen") or item.get("event_id") or "")
    return events


def review_paths(run_dir: Path) -> Dict[str, Path]:
    review_dir = run_dir / "manual_review"
    return {
        "dir": review_dir,
        "labels_json": review_dir / "risk_detection_review_labels.json",
        "labels_csv": review_dir / "risk_detection_review_labels.csv",
        "summary_json": review_dir / "risk_detection_review_summary.json",
        "threshold_json": review_dir / "threshold_suggestion.json",
        "threshold_md": review_dir / "threshold_suggestion.md",
    }


def load_labels(run_dir: Path) -> Dict[str, Dict[str, Any]]:
    paths = review_paths(run_dir)
    data = read_json(paths["labels_json"], {"labels": {}})
    labels = data.get("labels", {})
    return labels if isinstance(labels, dict) else {}


def save_labels(run_dir: Path, events: List[Dict[str, Any]], labels: Dict[str, Dict[str, Any]]) -> Dict[str, Any]:
    paths = review_paths(run_dir)
    paths["dir"].mkdir(parents=True, exist_ok=True)
    write_json(
        paths["labels_json"],
        {
            "schema_version": "risk_detection_manual_review_v1",
            "updated_at": now_iso(),
            "run_dir": str(run_dir),
            "labels": labels,
        },
    )
    event_by_id = {str(event.get("event_id")): event for event in events}
    fieldnames = [
        "event_id",
        "review_label",
        "corrected_class",
        "note",
        "reviewed_at",
        "class_name",
        "confidence",
        "confidence_max",
        "seen_count",
        "distance_m",
        "projection_status",
        "odom_x_m",
        "odom_y_m",
    ]
    with paths["labels_csv"].open("w", newline="", encoding="utf-8-sig") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for event_id, record in sorted(labels.items(), key=lambda item: item[1].get("reviewed_at", "")):
            event = event_by_id.get(event_id, {})
            xy = event.get("odom_point_xy_m") or {}
            writer.writerow(
                {
                    "event_id": event_id,
                    "review_label": record.get("label"),
                    "corrected_class": record.get("corrected_class"),
                    "note": record.get("note"),
                    "reviewed_at": record.get("reviewed_at"),
                    "class_name": event.get("class_name"),
                    "confidence": event.get("confidence"),
                    "confidence_max": event.get("confidence_max"),
                    "seen_count": event.get("seen_count"),
                    "distance_m": event.get("distance_m"),
                    "projection_status": event.get("projection_status"),
                    "odom_x_m": xy.get("x"),
                    "odom_y_m": xy.get("y"),
                }
            )
    summary = build_summary(events, labels)
    write_json(paths["summary_json"], summary)
    threshold = build_threshold_suggestion(events, labels)
    write_json(paths["threshold_json"], threshold)
    write_threshold_md(paths["threshold_md"], threshold)
    return summary


def build_summary(events: List[Dict[str, Any]], labels: Dict[str, Dict[str, Any]]) -> Dict[str, Any]:
    label_counts = Counter(record.get("label") for record in labels.values())
    by_class: Dict[str, Counter] = defaultdict(Counter)
    event_by_id = {str(event.get("event_id")): event for event in events}
    for event_id, record in labels.items():
        event = event_by_id.get(event_id, {})
        by_class[str(event.get("class_name", "unknown"))][record.get("label")] += 1
    reviewed = len(labels)
    return {
        "updated_at": now_iso(),
        "total_events": len(events),
        "reviewed_events": reviewed,
        "unreviewed_events": max(0, len(events) - reviewed),
        "label_counts": dict(label_counts),
        "by_class": {name: dict(counts) for name, counts in sorted(by_class.items())},
    }


def threshold_metrics(events: List[Dict[str, Any]], labels: Dict[str, Dict[str, Any]], score_key: str, class_name: Optional[str]) -> Dict[str, Any]:
    rows = []
    for event in events:
        event_id = str(event.get("event_id"))
        record = labels.get(event_id)
        if not record:
            continue
        label = record.get("label")
        if label in IGNORED_LABELS:
            continue
        if label not in POSITIVE_LABELS and label not in NEGATIVE_LABELS:
            continue
        if class_name and event.get("class_name") != class_name:
            continue
        rows.append(
            {
                "score": safe_float(event.get(score_key)),
                "positive": label in POSITIVE_LABELS,
                "event_id": event_id,
            }
        )
    if not rows:
        return {"score_key": score_key, "class_name": class_name or "all", "reviewed_used": 0}

    thresholds = sorted({round(row["score"], 4) for row in rows}, reverse=True)
    best = None
    total_positive = sum(1 for row in rows if row["positive"])
    total_negative = len(rows) - total_positive
    for threshold in thresholds:
        predicted = [row for row in rows if row["score"] >= threshold]
        tp = sum(1 for row in predicted if row["positive"])
        fp = sum(1 for row in predicted if not row["positive"])
        fn = total_positive - tp
        precision = tp / (tp + fp) if tp + fp else 0.0
        recall = tp / (tp + fn) if tp + fn else 0.0
        f1 = (2 * precision * recall / (precision + recall)) if precision + recall else 0.0
        candidate = {
            "threshold": threshold,
            "precision": round(precision, 4),
            "recall": round(recall, 4),
            "f1": round(f1, 4),
            "tp": tp,
            "fp": fp,
            "fn": fn,
        }
        if best is None or (candidate["f1"], candidate["precision"], -candidate["threshold"]) > (
            best["f1"],
            best["precision"],
            -best["threshold"],
        ):
            best = candidate

    correct_scores = [row["score"] for row in rows if row["positive"]]
    wrong_scores = [row["score"] for row in rows if not row["positive"]]
    return {
        "score_key": score_key,
        "class_name": class_name or "all",
        "rule_type": "score_only",
        "reviewed_used": len(rows),
        "positive_count": total_positive,
        "negative_count": total_negative,
        "lowest_correct_score": round(min(correct_scores), 4) if correct_scores else None,
        "highest_wrong_score": round(max(wrong_scores), 4) if wrong_scores else None,
        "best_f1_threshold": best,
    }


def joint_threshold_metrics(
    events: List[Dict[str, Any]],
    labels: Dict[str, Dict[str, Any]],
    score_key: str,
    class_name: Optional[str],
) -> Dict[str, Any]:
    rows = []
    for event in events:
        event_id = str(event.get("event_id"))
        record = labels.get(event_id)
        if not record:
            continue
        label = record.get("label")
        if label in IGNORED_LABELS:
            continue
        if label not in POSITIVE_LABELS and label not in NEGATIVE_LABELS:
            continue
        if class_name and event.get("class_name") != class_name:
            continue
        distance = event.get("distance_m")
        if distance is None:
            continue
        rows.append(
            {
                "score": safe_float(event.get(score_key)),
                "distance_m": safe_float(distance),
                "positive": label in POSITIVE_LABELS,
                "event_id": event_id,
            }
        )
    if not rows:
        return {
            "score_key": score_key,
            "class_name": class_name or "all",
            "rule_type": "score_and_depth_range",
            "reviewed_used": 0,
        }

    score_thresholds = sorted({round(row["score"], 4) for row in rows}, reverse=True)
    depth_values = sorted({round(row["distance_m"], 4) for row in rows})
    min_depth_candidates = [0.0] + depth_values
    max_depth_candidates = depth_values + [float("inf")]
    total_positive = sum(1 for row in rows if row["positive"])
    total_negative = len(rows) - total_positive
    best = None
    best_precision_first = None

    for score_threshold in score_thresholds:
        for min_depth in min_depth_candidates:
            for max_depth in max_depth_candidates:
                if min_depth > max_depth:
                    continue
                predicted = [
                    row
                    for row in rows
                    if row["score"] >= score_threshold and min_depth <= row["distance_m"] <= max_depth
                ]
                tp = sum(1 for row in predicted if row["positive"])
                fp = sum(1 for row in predicted if not row["positive"])
                fn = total_positive - tp
                precision = tp / (tp + fp) if tp + fp else 0.0
                recall = tp / (tp + fn) if tp + fn else 0.0
                f1 = (2 * precision * recall / (precision + recall)) if precision + recall else 0.0
                candidate = {
                    "score_threshold": score_threshold,
                    "min_depth_m": round(min_depth, 4),
                    "max_depth_m": None if max_depth == float("inf") else round(max_depth, 4),
                    "precision": round(precision, 4),
                    "recall": round(recall, 4),
                    "f1": round(f1, 4),
                    "tp": tp,
                    "fp": fp,
                    "fn": fn,
                }
                if best is None or (
                    candidate["f1"],
                    candidate["precision"],
                    candidate["recall"],
                    -candidate["score_threshold"],
                    -(candidate["max_depth_m"] or 9999.0),
                ) > (
                    best["f1"],
                    best["precision"],
                    best["recall"],
                    -best["score_threshold"],
                    -(best["max_depth_m"] or 9999.0),
                ):
                    best = candidate
                # Precision-first rule is useful for demo alarms: avoid false positives
                # while retaining at least one reviewed true positive.
                if tp > 0:
                    if best_precision_first is None or (
                        candidate["precision"],
                        candidate["recall"],
                        candidate["f1"],
                        -candidate["score_threshold"],
                    ) > (
                        best_precision_first["precision"],
                        best_precision_first["recall"],
                        best_precision_first["f1"],
                        -best_precision_first["score_threshold"],
                    ):
                        best_precision_first = candidate

    correct_depths = [row["distance_m"] for row in rows if row["positive"]]
    wrong_depths = [row["distance_m"] for row in rows if not row["positive"]]
    return {
        "score_key": score_key,
        "class_name": class_name or "all",
        "rule_type": "score_and_depth_range",
        "reviewed_used": len(rows),
        "positive_count": total_positive,
        "negative_count": total_negative,
        "correct_depth_range_m": [
            round(min(correct_depths), 4),
            round(max(correct_depths), 4),
        ]
        if correct_depths
        else None,
        "wrong_depth_range_m": [
            round(min(wrong_depths), 4),
            round(max(wrong_depths), 4),
        ]
        if wrong_depths
        else None,
        "best_f1_rule": best,
        "precision_first_rule": best_precision_first,
    }


def build_threshold_suggestion(events: List[Dict[str, Any]], labels: Dict[str, Dict[str, Any]]) -> Dict[str, Any]:
    reviewed_classes = sorted({str(event.get("class_name")) for event in events if str(event.get("event_id")) in labels})
    score_only = []
    score_and_depth = []
    for score_key in ("confidence", "confidence_max"):
        score_only.append(threshold_metrics(events, labels, score_key, None))
        score_and_depth.append(joint_threshold_metrics(events, labels, score_key, None))
        for class_name in reviewed_classes:
            score_only.append(threshold_metrics(events, labels, score_key, class_name))
            score_and_depth.append(joint_threshold_metrics(events, labels, score_key, class_name))
    return {
        "updated_at": now_iso(),
        "method": (
            "Thresholds are computed from manual labels. correct=true positive; "
            "wrong/class_wrong/duplicate=false positive; uncertain is ignored. "
            "Joint rules use score >= threshold and min_depth_m <= distance_m <= max_depth_m."
        ),
        "score_only_recommendations": score_only,
        "score_and_depth_recommendations": score_and_depth,
    }


def write_threshold_md(path: Path, threshold: Dict[str, Any]) -> None:
    lines = [
        "# Threshold Suggestion",
        "",
        f"- updated_at: `{threshold.get('updated_at')}`",
        "- correct is treated as positive",
        "- wrong/class_wrong/duplicate are treated as false positives",
        "- uncertain is ignored",
        "- joint rule format: `score >= threshold` and `min_depth_m <= depth <= max_depth_m`",
        "",
        "## Score Only",
        "",
        "| score | class | n | pos | neg | best_threshold | precision | recall | f1 | lowest_correct | highest_wrong |",
        "|-|-|-:|-:|-:|-:|-:|-:|-:|-:|-:|",
    ]
    for item in threshold.get("score_only_recommendations", []):
        best = item.get("best_f1_threshold") or {}
        lines.append(
            "|{score}|{cls}|{n}|{pos}|{neg}|{thr}|{p}|{r}|{f1}|{lc}|{hw}|".format(
                score=item.get("score_key"),
                cls=item.get("class_name"),
                n=item.get("reviewed_used"),
                pos=item.get("positive_count"),
                neg=item.get("negative_count"),
                thr=best.get("threshold"),
                p=best.get("precision"),
                r=best.get("recall"),
                f1=best.get("f1"),
                lc=item.get("lowest_correct_score"),
                hw=item.get("highest_wrong_score"),
            )
        )
    lines.extend(
        [
            "",
            "## Score + Depth",
            "",
            "| score | class | n | pos | neg | score_thr | min_depth | max_depth | precision | recall | f1 | depth_correct | depth_wrong |",
            "|-|-|-:|-:|-:|-:|-:|-:|-:|-:|-:|-|-|",
        ]
    )
    for item in threshold.get("score_and_depth_recommendations", []):
        best = item.get("best_f1_rule") or {}
        lines.append(
            "|{score}|{cls}|{n}|{pos}|{neg}|{thr}|{mind}|{maxd}|{p}|{r}|{f1}|{dc}|{dw}|".format(
                score=item.get("score_key"),
                cls=item.get("class_name"),
                n=item.get("reviewed_used"),
                pos=item.get("positive_count"),
                neg=item.get("negative_count"),
                thr=best.get("score_threshold"),
                mind=best.get("min_depth_m"),
                maxd=best.get("max_depth_m"),
                p=best.get("precision"),
                r=best.get("recall"),
                f1=best.get("f1"),
                dc=item.get("correct_depth_range_m"),
                dw=item.get("wrong_depth_range_m"),
            )
        )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def event_payload(run_dir: Path, events: List[Dict[str, Any]], labels: Dict[str, Dict[str, Any]]) -> List[Dict[str, Any]]:
    payload = []
    for idx, event in enumerate(events, 1):
        event_id = str(event.get("event_id"))
        xy = event.get("odom_point_xy_m") or {}
        payload.append(
            {
                "seq": idx,
                "event_id": event_id,
                "class_name": event.get("class_name"),
                "confidence": event.get("confidence"),
                "confidence_max": event.get("confidence_max"),
                "seen_count": event.get("seen_count"),
                "distance_m": event.get("distance_m"),
                "first_seen": event.get("first_seen"),
                "last_seen": event.get("last_seen"),
                "projection_status": event.get("projection_status"),
                "odom_point_xy_m": xy,
                "overlay_url": f"/media/overlay/{urllib.parse.quote(event_id)}",
                "rgb_url": f"/media/rgb/{urllib.parse.quote(event_id)}",
                "depth_url": f"/media/depth/{urllib.parse.quote(event_id)}",
                "overlay_exists": bool(event.get("_overlay_exists")),
                "rgb_exists": bool(event.get("_rgb_exists")),
                "depth_exists": bool(event.get("_depth_exists")),
                "review": labels.get(event_id),
            }
        )
    return payload


def html_page() -> bytes:
    labels_js = json.dumps(LABELS, ensure_ascii=False)
    classes_js = json.dumps(CLASSES, ensure_ascii=False)
    html = f"""<!doctype html>
<html>
<head>
  <meta charset="utf-8">
  <title>Risk Detection Manual Review</title>
  <style>
    body {{ margin: 0; font-family: Arial, sans-serif; background: #f4f6f8; color: #17202a; }}
    header {{ position: sticky; top: 0; z-index: 2; background: #111827; color: white; padding: 10px 14px; display: flex; align-items: center; gap: 12px; }}
    header button, header select {{ height: 30px; }}
    main {{ display: grid; grid-template-columns: 290px 1fr 330px; gap: 12px; padding: 12px; }}
    .list {{ background: white; border: 1px solid #d8dee4; overflow: auto; height: calc(100vh - 72px); }}
    .item {{ padding: 8px; border-bottom: 1px solid #edf0f2; cursor: pointer; font-size: 13px; }}
    .item.active {{ background: #e8f1ff; }}
    .item.correct {{ border-left: 5px solid #15803d; }}
    .item.wrong, .item.class_wrong, .item.duplicate {{ border-left: 5px solid #b91c1c; }}
    .item.uncertain {{ border-left: 5px solid #a16207; }}
    .viewer {{ background: white; border: 1px solid #d8dee4; padding: 10px; min-height: calc(100vh - 94px); }}
    .viewer img {{ max-width: 100%; max-height: calc(100vh - 230px); display: block; margin: 0 auto; background: #ddd; }}
    .tabs {{ display: flex; gap: 8px; margin-bottom: 8px; }}
    .tabs button.active {{ background: #2563eb; color: white; }}
    .panel {{ background: white; border: 1px solid #d8dee4; padding: 12px; height: calc(100vh - 96px); overflow: auto; }}
    .meta {{ display: grid; grid-template-columns: 120px 1fr; gap: 6px; font-size: 14px; }}
    .actions {{ display: grid; gap: 8px; margin: 14px 0; }}
    .actions button {{ padding: 11px; font-size: 15px; cursor: pointer; border: 1px solid #cbd5e1; border-radius: 6px; background: #f8fafc; text-align: left; }}
    .actions button.correct {{ border-color: #15803d; }}
    .actions button.wrong {{ border-color: #b91c1c; }}
    textarea, select {{ width: 100%; box-sizing: border-box; }}
    textarea {{ height: 76px; }}
    .summary {{ font-size: 13px; white-space: pre-wrap; background: #f8fafc; padding: 8px; border: 1px solid #e5e7eb; }}
    .kbd {{ color: #94a3b8; }}
  </style>
</head>
<body>
<header>
  <strong>Risk Detection Review</strong>
  <span id="progress"></span>
  <button onclick="prevEvent()">Prev</button>
  <button onclick="nextEvent(false)">Next</button>
  <button onclick="nextEvent(true)">Next Unreviewed</button>
  <select id="filter" onchange="renderList()">
    <option value="all">All</option>
    <option value="unreviewed">Unreviewed</option>
    <option value="reviewed">Reviewed</option>
    <option value="high">confidence &gt; 0.7</option>
  </select>
  <span class="kbd">Keys: 1 correct, 2 wrong, 3 class wrong, 4 uncertain, 5 duplicate, A/D nav</span>
</header>
<main>
  <aside class="list" id="eventList"></aside>
  <section class="viewer">
    <div class="tabs">
      <button id="tab-overlay" onclick="setImage('overlay')" class="active">Overlay</button>
      <button id="tab-rgb" onclick="setImage('rgb')">RGB</button>
      <button id="tab-depth" onclick="setImage('depth')">Depth</button>
    </div>
    <img id="mainImage" alt="event image">
  </section>
  <aside class="panel">
    <h2 id="title"></h2>
    <div class="meta" id="meta"></div>
    <div class="actions">
      <button class="correct" onclick="saveLabel('correct')">1 正确</button>
      <button class="wrong" onclick="saveLabel('wrong')">2 错误</button>
      <button onclick="saveLabel('class_wrong')">3 类别错/框基本对</button>
      <button onclick="saveLabel('uncertain')">4 不确定</button>
      <button onclick="saveLabel('duplicate')">5 重复/无效</button>
    </div>
    <label>修正类别</label>
    <select id="correctedClass"></select>
    <label>备注</label>
    <textarea id="note" placeholder="例如: 框到了椅子腿 / 类别应为 crack / 远距离误检"></textarea>
    <h3>Current Label</h3>
    <pre class="summary" id="currentLabel"></pre>
    <h3>Summary</h3>
    <pre class="summary" id="summary"></pre>
  </aside>
</main>
<script>
const LABELS = {labels_js};
const CLASSES = {classes_js};
let events = [];
let summary = {{}};
let current = 0;
let imageKind = 'overlay';

async function loadData() {{
  const ev = await fetch('/api/events').then(r => r.json());
  events = ev.events;
  summary = await fetch('/api/summary').then(r => r.json());
  const sel = document.getElementById('correctedClass');
  sel.innerHTML = '<option value="">不修正</option>' + CLASSES.map(c => `<option value="${{c}}">${{c}}</option>`).join('');
  renderList();
  showEvent(0);
}}

function filteredIndexes() {{
  const f = document.getElementById('filter').value;
  return events.map((e, i) => [e, i]).filter(([e]) => {{
    if (f === 'unreviewed') return !e.review;
    if (f === 'reviewed') return !!e.review;
    if (f === 'high') return Number(e.confidence || 0) > 0.7;
    return true;
  }}).map(([, i]) => i);
}}

function renderList() {{
  const list = document.getElementById('eventList');
  const indexes = filteredIndexes();
  list.innerHTML = indexes.map(i => {{
    const e = events[i];
    const label = e.review ? e.review.label : '';
    return `<div class="item ${{i === current ? 'active' : ''}} ${{label}}" onclick="showEvent(${{i}})">
      <b>#${{e.seq}} ${{e.class_name}}</b> conf=${{e.confidence}} max=${{e.confidence_max}}<br>
      seen=${{e.seen_count}} dist=${{e.distance_m}}m<br>
      ${{label ? LABELS[label] : '未标注'}}
    </div>`;
  }}).join('');
  document.getElementById('progress').innerText = `${{summary.reviewed_events || 0}}/${{events.length}} reviewed`;
}}

function setImage(kind) {{
  imageKind = kind;
  for (const k of ['overlay', 'rgb', 'depth']) document.getElementById(`tab-${{k}}`).classList.toggle('active', k === kind);
  showEvent(current, false);
}}

function showEvent(idx, scroll=true) {{
  if (!events.length) return;
  current = Math.max(0, Math.min(events.length - 1, idx));
  const e = events[current];
  document.getElementById('title').innerText = `#${{e.seq}} ${{e.class_name}}`;
  const url = imageKind === 'rgb' ? e.rgb_url : imageKind === 'depth' ? e.depth_url : e.overlay_url;
  document.getElementById('mainImage').src = `${{url}}?t=${{Date.now()}}`;
  document.getElementById('meta').innerHTML = [
    ['event_id', e.event_id], ['class', e.class_name], ['confidence', e.confidence],
    ['confidence_max', e.confidence_max], ['seen_count', e.seen_count], ['distance_m', e.distance_m],
    ['projection', e.projection_status], ['odom_xy', JSON.stringify(e.odom_point_xy_m)],
    ['first_seen', e.first_seen], ['last_seen', e.last_seen],
  ].map(([k,v]) => `<b>${{k}}</b><span>${{v}}</span>`).join('');
  document.getElementById('correctedClass').value = e.review?.corrected_class || '';
  document.getElementById('note').value = e.review?.note || '';
  document.getElementById('currentLabel').innerText = e.review ? JSON.stringify(e.review, null, 2) : 'unreviewed';
  document.getElementById('summary').innerText = JSON.stringify(summary, null, 2);
  renderList();
  if (scroll) {{
    const active = document.querySelector('.item.active');
    if (active) active.scrollIntoView({{ block: 'nearest' }});
  }}
}}

async function saveLabel(label) {{
  const e = events[current];
  const body = {{
    event_id: e.event_id,
    label,
    corrected_class: document.getElementById('correctedClass').value,
    note: document.getElementById('note').value,
  }};
  const result = await fetch('/api/review', {{ method: 'POST', headers: {{ 'Content-Type': 'application/json' }}, body: JSON.stringify(body) }}).then(r => r.json());
  events = result.events;
  summary = result.summary;
  showEvent(current, false);
  nextEvent(false);
}}

function nextEvent(unreviewed) {{
  if (!events.length) return;
  if (unreviewed) {{
    for (let i = current + 1; i < events.length; i++) if (!events[i].review) return showEvent(i);
    for (let i = 0; i <= current; i++) if (!events[i].review) return showEvent(i);
  }} else {{
    showEvent(Math.min(events.length - 1, current + 1));
  }}
}}

function prevEvent() {{ showEvent(Math.max(0, current - 1)); }}

document.addEventListener('keydown', (ev) => {{
  if (ev.target.tagName === 'TEXTAREA' || ev.target.tagName === 'SELECT') return;
  if (ev.key === '1') saveLabel('correct');
  if (ev.key === '2') saveLabel('wrong');
  if (ev.key === '3') saveLabel('class_wrong');
  if (ev.key === '4') saveLabel('uncertain');
  if (ev.key === '5') saveLabel('duplicate');
  if (ev.key.toLowerCase() === 'd') nextEvent(false);
  if (ev.key.toLowerCase() === 'a') prevEvent();
}});

loadData();
</script>
</body>
</html>
"""
    return html.encode("utf-8")


class ReviewServer(BaseHTTPRequestHandler):
    server_version = "RiskReview/1.0"

    def _send_json(self, data: Any, status: int = 200) -> None:
        body = json.dumps(data, ensure_ascii=False, indent=2).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _send_bytes(self, body: bytes, content_type: str, status: int = 200) -> None:
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    @property
    def run_dir(self) -> Path:
        return self.server.run_dir  # type: ignore[attr-defined]

    @property
    def events(self) -> List[Dict[str, Any]]:
        return self.server.events  # type: ignore[attr-defined]

    @property
    def labels(self) -> Dict[str, Dict[str, Any]]:
        return self.server.labels  # type: ignore[attr-defined]

    def do_GET(self) -> None:  # noqa: N802
        parsed = urllib.parse.urlparse(self.path)
        path = parsed.path
        if path == "/":
            return self._send_bytes(html_page(), "text/html; charset=utf-8")
        if path == "/api/events":
            return self._send_json({"events": event_payload(self.run_dir, self.events, self.labels)})
        if path == "/api/summary":
            return self._send_json(build_summary(self.events, self.labels))
        if path.startswith("/media/"):
            return self._serve_media(path)
        if path == "/threshold_suggestion.md":
            md_path = review_paths(self.run_dir)["threshold_md"]
            if md_path.exists():
                return self._send_bytes(md_path.read_bytes(), "text/markdown; charset=utf-8")
        return self._send_json({"error": "not found"}, status=404)

    def _serve_media(self, path: str) -> None:
        parts = path.strip("/").split("/")
        if len(parts) != 3:
            return self._send_json({"error": "bad media path"}, status=400)
        _, kind, raw_event_id = parts
        event_id = urllib.parse.unquote(raw_event_id)
        if not any(str(event.get("event_id")) == event_id for event in self.events):
            return self._send_json({"error": "unknown event_id"}, status=404)
        name = {"overlay": "overlay.png", "rgb": "rgb.png", "depth": "depth_vis.png"}.get(kind)
        if not name:
            return self._send_json({"error": "unknown media kind"}, status=400)
        path_obj = local_capture_path(self.run_dir, event_id, name)
        if not path_obj.exists():
            return self._send_json({"error": f"file not found: {name}"}, status=404)
        content_type = mimetypes.guess_type(str(path_obj))[0] or "application/octet-stream"
        self._send_bytes(path_obj.read_bytes(), content_type)

    def do_POST(self) -> None:  # noqa: N802
        parsed = urllib.parse.urlparse(self.path)
        if parsed.path != "/api/review":
            return self._send_json({"error": "not found"}, status=404)
        length = safe_int(self.headers.get("Content-Length"), 0)
        try:
            body = json.loads(self.rfile.read(length).decode("utf-8"))
        except Exception as exc:  # noqa: BLE001
            return self._send_json({"error": f"invalid json: {exc}"}, status=400)
        event_id = str(body.get("event_id") or "")
        label = str(body.get("label") or "")
        if label not in LABELS:
            return self._send_json({"error": f"invalid label: {label}"}, status=400)
        if not any(str(event.get("event_id")) == event_id for event in self.events):
            return self._send_json({"error": f"unknown event_id: {event_id}"}, status=404)
        corrected_class = str(body.get("corrected_class") or "")
        if corrected_class and corrected_class not in CLASSES:
            return self._send_json({"error": f"invalid corrected_class: {corrected_class}"}, status=400)
        self.labels[event_id] = {
            "event_id": event_id,
            "label": label,
            "label_text": LABELS[label],
            "corrected_class": corrected_class or None,
            "note": str(body.get("note") or "").strip(),
            "reviewed_at": now_iso(),
        }
        summary = save_labels(self.run_dir, self.events, self.labels)
        return self._send_json({"ok": True, "summary": summary, "events": event_payload(self.run_dir, self.events, self.labels)})

    def log_message(self, fmt: str, *args: Any) -> None:
        sys.stderr.write("[%s] %s\n" % (self.log_date_time_string(), fmt % args))


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", default=str(DEFAULT_RUN_DIR))
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--sort", choices=["time", "confidence", "confidence_max", "class"], default="time")
    parser.add_argument("--open-browser", action="store_true")
    return parser.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    run_dir = Path(args.run_dir).resolve()
    events = load_events(run_dir, args.sort)
    labels = load_labels(run_dir)
    save_labels(run_dir, events, labels)
    server = ThreadingHTTPServer((args.host, args.port), ReviewServer)
    server.run_dir = run_dir  # type: ignore[attr-defined]
    server.events = events  # type: ignore[attr-defined]
    server.labels = labels  # type: ignore[attr-defined]
    url = f"http://{args.host}:{args.port}/"
    print(json.dumps({"url": url, "run_dir": str(run_dir), "events": len(events)}, ensure_ascii=False, indent=2), flush=True)
    if args.open_browser:
        webbrowser.open(url)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
