#!/usr/bin/env python3
"""Generate an adaptive K1 SLAM map + RRT + risk overlay.

The visualization keeps ROS/map coordinates as the source of truth. If a risk
event only has odom coordinates, it is shown as an approximate candidate instead
of a formal map point.
"""

from __future__ import annotations

import argparse
import html
import json
import math
import re
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np
import yaml
from PIL import Image
from skimage.feature import canny
from skimage.transform import probabilistic_hough_line


def read_jsonl(path: Path) -> List[Dict[str, Any]]:
    rows: List[Dict[str, Any]] = []
    if not path.exists():
        return rows
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        line = line.strip()
        if not line:
            continue
        try:
            item = json.loads(line)
        except json.JSONDecodeError:
            continue
        if isinstance(item, dict):
            rows.append(item)
    return rows


def read_json(path: Path) -> Dict[str, Any]:
    if not path.exists():
        return {}
    try:
        data = json.loads(path.read_text(encoding="utf-8", errors="replace"))
    except json.JSONDecodeError:
        return {}
    return data if isinstance(data, dict) else {}


def load_map(map_yaml: Path) -> Tuple[Dict[str, Any], np.ndarray, np.ndarray]:
    meta = yaml.safe_load(map_yaml.read_text(encoding="utf-8")) or {}
    image_path = Path(meta["image"])
    if not image_path.is_absolute():
        image_path = map_yaml.parent / image_path
    gray = np.array(Image.open(image_path).convert("L"))
    rgb = np.zeros((*gray.shape, 3), dtype=np.uint8)
    rgb[gray >= 250] = (255, 255, 255)
    rgb[(gray > 80) & (gray < 250)] = (205, 205, 205)
    rgb[gray <= 80] = (0, 0, 0)
    return meta, gray, rgb


def line_angle_mod90(angle_deg: float) -> float:
    angle = angle_deg
    while angle < -45.0:
        angle += 90.0
    while angle >= 45.0:
        angle -= 90.0
    return angle


def detect_dominant_wall_angle_deg(gray: np.ndarray) -> Dict[str, Any]:
    occupied = gray < 80
    if occupied.sum() < 8:
        return {"dominant_angle_deg": 0.0, "line_count": 0, "confidence": 0.0}

    edges = canny(occupied.astype(float), sigma=1.0)
    lines = probabilistic_hough_line(edges, threshold=3, line_length=5, line_gap=2)
    weighted: List[Tuple[float, float]] = []
    for (x1, y1), (x2, y2) in lines:
        dx = float(x2 - x1)
        dy = float(y2 - y1)
        length = math.hypot(dx, dy)
        if length < 4.0:
            continue
        weighted.append((line_angle_mod90(math.degrees(math.atan2(dy, dx))), length))

    if not weighted:
        return {"dominant_angle_deg": 0.0, "line_count": 0, "confidence": 0.0}

    best_angle = 0.0
    best_score = -1.0
    total_weight = sum(length for _, length in weighted)
    for candidate in np.linspace(-45.0, 44.5, 180):
        score = 0.0
        for angle, length in weighted:
            delta = ((angle - float(candidate) + 45.0) % 90.0) - 45.0
            score += length * math.exp(-(delta * delta) / (2.0 * 6.0 * 6.0))
        if score > best_score:
            best_angle = float(candidate)
            best_score = score

    return {
        "dominant_angle_deg": round(best_angle, 3),
        "line_count": len(weighted),
        "confidence": round(best_score / max(total_weight, 1e-6), 3),
    }


def rotation_bounds(width: int, height: int, angle_deg: float) -> Dict[str, float]:
    angle = math.radians(angle_deg)
    cos_a = math.cos(angle)
    sin_a = math.sin(angle)
    corners = [(0.0, 0.0), (width - 1.0, 0.0), (0.0, height - 1.0), (width - 1.0, height - 1.0)]
    points: List[Tuple[float, float]] = []
    for px, py in corners:
        dx = px - width / 2.0
        dy = py - height / 2.0
        points.append((dx * cos_a - dy * sin_a, dx * sin_a + dy * cos_a))
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    return {
        "minx": math.floor(min(xs)),
        "miny": math.floor(min(ys)),
        "maxx": math.ceil(max(xs)),
        "maxy": math.ceil(max(ys)),
    }


def render_rectified_image(rgb: np.ndarray, angle_deg: float) -> Tuple[np.ndarray, Dict[str, float]]:
    height, width = rgb.shape[:2]
    bounds = rotation_bounds(width, height, angle_deg)
    out_w = int(bounds["maxx"] - bounds["minx"] + 1)
    out_h = int(bounds["maxy"] - bounds["miny"] + 1)
    out = np.full((out_h, out_w, 3), 232, dtype=np.uint8)
    angle = math.radians(angle_deg)
    cos_a = math.cos(angle)
    sin_a = math.sin(angle)
    for ry in range(out_h):
        for rx in range(out_w):
            xr = rx + bounds["minx"]
            yr = ry + bounds["miny"]
            dx = xr * cos_a + yr * sin_a
            dy = -xr * sin_a + yr * cos_a
            px = int(round(dx + width / 2.0))
            py = int(round(dy + height / 2.0))
            if 0 <= px < width and 0 <= py < height:
                out[ry, rx] = rgb[py, px]
    bounds["width"] = out_w
    bounds["height"] = out_h
    return out, bounds


def parse_rrt_logs(paths: Sequence[Path]) -> List[Dict[str, Any]]:
    points: List[Dict[str, Any]] = []
    by_key: Dict[str, Dict[str, Any]] = {}
    for path in paths:
        if not path.exists():
            continue
        source = path.name
        seq = 0
        for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
            match = re.search(r"RRT_(GOAL|RESULT)\s+({.*})", line)
            if not match:
                continue
            try:
                data = json.loads(match.group(2))
            except json.JSONDecodeError:
                continue
            xy = data.get("xy")
            if not (isinstance(xy, list) and len(xy) >= 2):
                continue
            goal_count = data.get("goal_count")
            key = f"{source}:{goal_count}:{seq if match.group(1) == 'GOAL' else ''}"
            if match.group(1) == "GOAL":
                seq += 1
                item = {
                    "source": source,
                    "goal_count": goal_count,
                    "x": float(xy[0]),
                    "y": float(xy[1]),
                    "reason": data.get("reason"),
                    "status": "sent",
                    "time_s": data.get("time_s"),
                    "goal_clearance_m": data.get("goal_clearance_m"),
                    "unknown_gain": data.get("unknown_gain"),
                }
                by_key[f"{source}:{goal_count}"] = item
                points.append(item)
            else:
                existing = by_key.get(f"{source}:{goal_count}")
                if existing is not None:
                    existing["status"] = data.get("nav_status") or "result"
                    existing["result_time_s"] = data.get("time_s")
                else:
                    points.append(
                        {
                            "source": source,
                            "goal_count": goal_count,
                            "x": float(xy[0]),
                            "y": float(xy[1]),
                            "reason": data.get("reason"),
                            "status": data.get("nav_status") or "result",
                            "time_s": data.get("time_s"),
                            "goal_clearance_m": data.get("goal_clearance_m"),
                            "unknown_gain": data.get("unknown_gain"),
                        }
                    )
    return points


def point_xy(value: Any) -> Optional[Tuple[float, float]]:
    if not isinstance(value, dict):
        return None
    try:
        return float(value["x"]), float(value["y"])
    except (KeyError, TypeError, ValueError):
        return None


def parse_risk_events(paths: Sequence[Path]) -> List[Dict[str, Any]]:
    risks: List[Dict[str, Any]] = []
    seen = set()
    for path in paths:
        for event in read_jsonl(path):
            map_xy = point_xy(event.get("latest_map_point_xy_m")) or point_xy(event.get("map_point_xy_m"))
            odom_xy = point_xy(event.get("latest_odom_point_xy_m")) or point_xy(event.get("odom_point_xy_m"))
            xy = map_xy or odom_xy
            if xy is None:
                continue
            event_id = event.get("event_id") or f"{path.name}:{len(risks)}"
            key = (event_id, round(xy[0], 3), round(xy[1], 3))
            if key in seen:
                continue
            seen.add(key)
            risks.append(
                {
                    "event_id": event_id,
                    "class_name": event.get("class_name") or event.get("type") or "risk",
                    "confidence": event.get("latest_confidence", event.get("confidence")),
                    "distance_m": event.get("latest_distance_m", event.get("distance_m")),
                    "candidate_kind": event.get("candidate_kind"),
                    "x": xy[0],
                    "y": xy[1],
                    "frame_kind": "map" if map_xy else "odom_approx",
                    "projection": event.get("map_projection_status") or event.get("projection_status"),
                    "source": path.name,
                }
            )
    return risks


def parse_confirmed_points(paths: Sequence[Path]) -> List[Dict[str, Any]]:
    points: List[Dict[str, Any]] = []
    for path in paths:
        package = read_json(path)
        for item in package.get("risk_map_points") or package.get("confirmed_map_points") or []:
            xy = point_xy(item.get("map_point_xy_m")) or point_xy(item)
            if xy is None:
                continue
            points.append(
                {
                    "event_id": item.get("event_id"),
                    "class_name": item.get("class_name") or item.get("type") or "risk",
                    "confidence": item.get("confidence"),
                    "distance_m": item.get("distance_m"),
                    "candidate_kind": "confirmed",
                    "x": xy[0],
                    "y": xy[1],
                    "frame_kind": "map",
                    "projection": "confirmed",
                    "source": path.name,
                }
            )
    return points


def parse_approach_records(paths: Sequence[Path]) -> List[Dict[str, Any]]:
    records: List[Dict[str, Any]] = []
    for path in paths:
        for item in read_jsonl(path):
            for field, role in (("robot_odom_xy_m", "robot"), ("risk_odom_xy_m", "approach_target")):
                xy = point_xy(item.get(field))
                if xy is None:
                    continue
                records.append(
                    {
                        "x": xy[0],
                        "y": xy[1],
                        "role": role,
                        "state": item.get("state"),
                        "event_id": item.get("event_id") or (item.get("event") or {}).get("event_id"),
                        "frame_kind": "odom_approx",
                    }
                )
    return records


def world_to_rect(
    x: float,
    y: float,
    meta: Dict[str, Any],
    raw_width: int,
    raw_height: int,
    angle_deg: float,
    bounds: Dict[str, float],
) -> Dict[str, float]:
    resolution = float(meta["resolution"])
    origin = meta.get("origin") or [0.0, 0.0, 0.0]
    px = (float(x) - float(origin[0])) / resolution
    py = (float(origin[1]) + raw_height * resolution - float(y)) / resolution
    angle = math.radians(angle_deg)
    dx = px - raw_width / 2.0
    dy = py - raw_height / 2.0
    rx = dx * math.cos(angle) - dy * math.sin(angle) - bounds["minx"]
    ry = dx * math.sin(angle) + dy * math.cos(angle) - bounds["miny"]
    return {"rx": round(rx, 3), "ry": round(ry, 3)}


def attach_rect(points: Iterable[Dict[str, Any]], meta: Dict[str, Any], raw_width: int, raw_height: int, angle: float, bounds: Dict[str, float]) -> List[Dict[str, Any]]:
    out: List[Dict[str, Any]] = []
    for point in points:
        item = dict(point)
        item.update(world_to_rect(float(point["x"]), float(point["y"]), meta, raw_width, raw_height, angle, bounds))
        out.append(item)
    return out


def write_static_overlay(image: np.ndarray, data: Dict[str, Any], output_path: Path) -> None:
    scale = 16
    base = Image.fromarray(image).resize((image.shape[1] * scale, image.shape[0] * scale), Image.Resampling.NEAREST).convert("RGBA")
    from PIL import ImageDraw

    draw = ImageDraw.Draw(base)

    def pxy(item: Dict[str, Any]) -> Tuple[float, float]:
        x = float(item["rx"]) * scale
        y = float(item["ry"]) * scale
        if data["display"]["default_flip_x"]:
            x = base.width - x
        return x, y

    for point in data["rrt_points"]:
        x, y = pxy(point)
        color = {
            "status_4": "#16a34a",
            "progress_timeout": "#f97316",
            "result_timeout": "#f97316",
            "status_6": "#dc2626",
            "sent": "#2563eb",
        }.get(str(point.get("status")), "#2563eb")
        r = 5
        draw.ellipse((x - r, y - r, x + r, y + r), fill=color, outline="#111827", width=2)

    for risk in data["risk_points"]:
        x, y = pxy(risk)
        color = {
            "blockage": "#dc2626",
            "corrosion": "#9333ea",
            "crack": "#2563eb",
            "leakage": "#0891b2",
        }.get(str(risk.get("class_name")), "#111827")
        r = 7 if risk.get("frame_kind") == "map" else 6
        if risk.get("frame_kind") == "map":
            draw.rectangle((x - r, y - r, x + r, y + r), fill=color, outline="#111827", width=2)
        else:
            draw.line((x - r, y - r, x + r, y + r), fill=color, width=4)
            draw.line((x + r, y - r, x - r, y + r), fill=color, width=4)

    base.save(output_path)


def build_html(data: Dict[str, Any]) -> str:
    payload = json.dumps(data, ensure_ascii=False, separators=(",", ":"))
    title = html.escape(str(data.get("title") or "K1 adaptive map overlay"))
    default_flip = "checked" if data["display"]["default_flip_x"] else ""
    return f"""<!doctype html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>{title}</title>
<style>
body{{margin:0;background:#eef2f7;color:#111827;font-family:-apple-system,BlinkMacSystemFont,"Segoe UI",Arial,sans-serif}}
.wrap{{display:grid;grid-template-columns:minmax(0,1fr) 410px;gap:14px;padding:14px;box-sizing:border-box;max-width:1320px;margin:0 auto}}
.mapbox{{background:#d8dee8;border:1px solid #a7b0bd;border-radius:8px;padding:12px;display:flex;align-items:center;justify-content:center;min-width:0;overflow:hidden}}
.stage{{position:relative;width:min(100%,76vh,900px);aspect-ratio:1/1;background:#fff;border:2px solid #111827;box-sizing:border-box}}
#mapImg,#overlay{{position:absolute;inset:0;width:100%;height:100%;object-fit:contain}}
#mapImg{{image-rendering:pixelated;pointer-events:none}}
#mapImg.flip-x{{transform:scaleX(-1)}}
.panel{{background:#fff;border:1px solid #cbd5e1;border-radius:8px;padding:14px;display:flex;flex-direction:column;gap:10px;min-width:0}}
h1{{font-size:18px;margin:0;font-weight:500}}
.kv{{display:grid;grid-template-columns:112px minmax(0,1fr);gap:8px;font-size:13px}}
.k{{color:#64748b}}
.mono,textarea{{font-family:ui-monospace,SFMono-Regular,Menlo,Consolas,monospace}}
button,label{{border:1px solid #cbd5e1;background:#f8fafc;border-radius:6px;padding:7px 9px;font-size:13px;cursor:pointer}}
.bar{{display:flex;gap:7px;flex-wrap:wrap}}
.note{{font-size:12px;color:#64748b;line-height:1.45}}
textarea{{min-height:160px;width:100%;resize:vertical;border:1px solid #cbd5e1;border-radius:6px;padding:8px;background:#f8fafc;color:#111827;box-sizing:border-box}}
.ok{{color:#047857;font-weight:600}}
table{{border-collapse:collapse;width:100%;font-size:12px}}
td,th{{border-bottom:1px solid #e2e8f0;padding:5px;text-align:left}}
@media(max-width:1050px){{.wrap{{grid-template-columns:1fr}}.stage{{width:min(100%,82vh)}}}}
</style>
</head>
<body>
<div class="wrap">
  <div class="mapbox"><div id="stage" class="stage"><img id="mapImg" src="{html.escape(data["display"]["image"])}"><canvas id="overlay"></canvas></div></div>
  <div class="panel">
    <h1>K1 自适应横平竖直叠图</h1>
    <div class="kv"><div class="k">状态</div><div id="status" class="mono">loading</div></div>
    <div class="kv"><div class="k">ROS/map</div><div id="coord" class="mono">x=--, y=--</div></div>
    <div class="kv"><div class="k">最近对象</div><div id="near" class="mono">--</div></div>
    <div class="kv"><div class="k">自动校正</div><div class="mono">rotate {data["display"]["rotation_deg"]:.2f} deg</div></div>
    <div class="kv"><div class="k">风险坐标</div><div class="mono">map={data["summary"]["risk_map_count"]}, odom≈={data["summary"]["risk_odom_approx_count"]}</div></div>
    <div class="bar">
      <label><input id="flipX" type="checkbox" {default_flip}>左右镜像显示</label>
      <label><input id="grid" type="checkbox" checked>网格</label>
      <label><input id="rrt" type="checkbox" checked>RRT</label>
      <label><input id="risk" type="checkbox" checked>YOLO风险</label>
      <label><input id="approach" type="checkbox" checked>approach</label>
    </div>
    <div class="bar"><button id="undo">撤销</button><button id="clear">清空</button><button id="copyJson">复制JSON</button><button id="copyCsv">复制CSV</button></div>
    <div class="note">实心方块是 map 坐标风险点；叉号是 odom 近似候选，只能用于诊断。点击输出原始 ROS/map 坐标，不受左右镜像显示影响。</div>
    <table><thead><tr><th>#</th><th>x</th><th>y</th><th>label</th></tr></thead><tbody id="tbody"></tbody></table>
    <textarea id="out" spellcheck="false"></textarea>
  </div>
</div>
<script>
const DATA = {payload};
const stage = document.getElementById('stage');
const canvas = document.getElementById('overlay');
const ctx = canvas.getContext('2d');
const img = document.getElementById('mapImg');
const coord = document.getElementById('coord');
const nearBox = document.getElementById('near');
const tbody = document.getElementById('tbody');
const out = document.getElementById('out');
let manual = [];
let hover = null;

function isFlip() {{ return document.getElementById('flipX').checked; }}
function fit() {{
  const r = stage.getBoundingClientRect();
  const iw = DATA.rectified_image.width, ih = DATA.rectified_image.height;
  const s = Math.min(r.width / iw, r.height / ih);
  const w = iw * s, h = ih * s;
  return {{s, x:(r.width-w)/2, y:(r.height-h)/2, w, h}};
}}
function rectToScreen(rx, ry) {{
  const f = fit();
  let x = rx;
  if (isFlip()) x = DATA.rectified_image.width - x;
  return {{x:f.x + x*f.s, y:f.y + ry*f.s}};
}}
function screenToRect(x, y) {{
  const f = fit();
  let rx = (x - f.x) / f.s;
  if (isFlip()) rx = DATA.rectified_image.width - rx;
  return {{rx, ry:(y - f.y) / f.s}};
}}
function rectToWorld(rx, ry) {{
  const W = DATA.raw_map.width, H = DATA.raw_map.height;
  const res = DATA.raw_map.resolution, ox = DATA.raw_map.origin[0], oy = DATA.raw_map.origin[1];
  const b = DATA.rotation_bounds, angle = -DATA.display.rotation_deg * Math.PI / 180;
  const xr = rx + b.minx, yr = ry + b.miny;
  const dx = xr * Math.cos(angle) - yr * Math.sin(angle);
  const dy = xr * Math.sin(angle) + yr * Math.cos(angle);
  const px = dx + W/2, py = dy + H/2;
  return {{x: ox + px*res, y: oy + H*res - py*res}};
}}
function worldToRect(x, y) {{
  const W = DATA.raw_map.width, H = DATA.raw_map.height;
  const res = DATA.raw_map.resolution, ox = DATA.raw_map.origin[0], oy = DATA.raw_map.origin[1];
  const b = DATA.rotation_bounds, angle = DATA.display.rotation_deg * Math.PI / 180;
  const px = (x - ox) / res, py = (oy + H*res - y) / res;
  const dx = px - W/2, dy = py - H/2;
  return {{rx: dx*Math.cos(angle) - dy*Math.sin(angle) - b.minx,
           ry: dx*Math.sin(angle) + dy*Math.cos(angle) - b.miny}};
}}
function colorRisk(cls) {{ return {{blockage:'#dc2626', corrosion:'#9333ea', crack:'#2563eb', leakage:'#0891b2'}}[cls] || '#111827'; }}
function colorRrt(status) {{ return {{status_4:'#16a34a', progress_timeout:'#f97316', result_timeout:'#f97316', status_6:'#dc2626', sent:'#2563eb'}}[status] || '#2563eb'; }}
function mark(rx, ry, color, shape, text, dashed=false) {{
  const p = rectToScreen(rx, ry);
  ctx.save();
  ctx.lineWidth = 2.4;
  ctx.strokeStyle = color;
  ctx.fillStyle = color;
  if (dashed) ctx.setLineDash([4, 4]);
  ctx.beginPath();
  if (shape === 'x') {{
    ctx.lineWidth = 4;
    ctx.moveTo(p.x-8, p.y-8); ctx.lineTo(p.x+8, p.y+8);
    ctx.moveTo(p.x+8, p.y-8); ctx.lineTo(p.x-8, p.y+8);
    ctx.stroke();
  }} else if (shape === 'square') {{
    ctx.rect(p.x-7, p.y-7, 14, 14); ctx.fill(); ctx.strokeStyle = '#111827'; ctx.stroke();
  }} else if (shape === 'diamond') {{
    ctx.moveTo(p.x, p.y-8); ctx.lineTo(p.x+8, p.y); ctx.lineTo(p.x, p.y+8); ctx.lineTo(p.x-8, p.y); ctx.closePath(); ctx.fill(); ctx.strokeStyle = '#111827'; ctx.stroke();
  }} else {{
    ctx.arc(p.x, p.y, 6, 0, Math.PI*2); ctx.fill(); ctx.strokeStyle = '#111827'; ctx.stroke();
  }}
  if (text) {{
    ctx.setLineDash([]);
    ctx.font = '12px ui-monospace,monospace';
    ctx.lineWidth = 4;
    ctx.strokeStyle = 'rgba(255,255,255,.95)';
    ctx.fillStyle = '#111827';
    ctx.strokeText(text, p.x+10, p.y-6);
    ctx.fillText(text, p.x+10, p.y-6);
  }}
  ctx.restore();
}}
function resize() {{
  const r = stage.getBoundingClientRect(), d = window.devicePixelRatio || 1;
  canvas.width = Math.round(r.width*d); canvas.height = Math.round(r.height*d);
  ctx.setTransform(d,0,0,d,0,0);
  draw();
}}
function drawGrid() {{
  const f = fit();
  ctx.save();
  ctx.strokeStyle = 'rgba(15,23,42,.15)';
  ctx.lineWidth = 1;
  for (let x=f.x; x<=f.x+f.w; x+=f.s*5) {{ ctx.beginPath(); ctx.moveTo(x,f.y); ctx.lineTo(x,f.y+f.h); ctx.stroke(); }}
  for (let y=f.y; y<=f.y+f.h; y+=f.s*5) {{ ctx.beginPath(); ctx.moveTo(f.x,y); ctx.lineTo(f.x+f.w,y); ctx.stroke(); }}
  ctx.restore();
}}
function draw() {{
  img.classList.toggle('flip-x', isFlip());
  const r = stage.getBoundingClientRect();
  ctx.clearRect(0,0,r.width,r.height);
  if (document.getElementById('grid').checked) drawGrid();
  if (document.getElementById('rrt').checked) DATA.rrt_points.forEach(p => mark(p.rx, p.ry, colorRrt(p.status), 'circle', 'R'+p.goal_count));
  if (document.getElementById('risk').checked) DATA.risk_points.forEach(p => {{
    const mapPoint = p.frame_kind === 'map';
    mark(p.rx, p.ry, colorRisk(p.class_name), mapPoint ? 'square' : 'x', p.class_name + ' ' + Number(p.confidence || 0).toFixed(2) + (mapPoint ? '' : ' odom≈'), !mapPoint);
  }});
  if (document.getElementById('approach').checked) DATA.approach_points.forEach(p => mark(p.rx, p.ry, p.role === 'robot' ? '#ec4899' : '#111827', 'diamond', p.role));
  manual.forEach((p, i) => {{
    const q = worldToRect(p.x, p.y);
    mark(q.rx, q.ry, '#06b6d4', 'diamond', 'P'+(i+1)+(p.label ? ' '+p.label : ''));
  }});
  if (hover) mark(hover.rx, hover.ry, '#111827', 'circle', '');
  renderRows();
}}
function visibleObjects() {{
  let out = [];
  if (document.getElementById('rrt').checked) out = out.concat(DATA.rrt_points.map(p => Object.assign({{kind:'RRT', label:'RRT R'+p.goal_count+' '+p.status}}, p)));
  if (document.getElementById('risk').checked) out = out.concat(DATA.risk_points.map(p => Object.assign({{kind:'risk', label:p.class_name+' '+Number(p.confidence || 0).toFixed(2)+' '+p.frame_kind}}, p)));
  return out;
}}
function nearest(rx, ry) {{
  let best = null, bestD = 1e9;
  visibleObjects().forEach(p => {{
    const d = Math.hypot(p.rx-rx, p.ry-ry);
    if (d < bestD) {{ best = p; bestD = d; }}
  }});
  return bestD < 14 ? best : null;
}}
function rows() {{ return manual.map((p,i) => ({{id:'manual_'+(i+1), x:+p.x.toFixed(4), y:+p.y.toFixed(4), label:p.label || ''}})); }}
function updateOut() {{ out.value = JSON.stringify(rows(), null, 2); }}
function renderRows() {{
  tbody.innerHTML = '';
  manual.forEach((p,i) => {{
    const tr = document.createElement('tr');
    tr.innerHTML = '<td>'+(i+1)+'</td><td>'+p.x.toFixed(3)+'</td><td>'+p.y.toFixed(3)+'</td><td contenteditable data-i="'+i+'">'+(p.label || '')+'</td>';
    tbody.appendChild(tr);
  }});
  tbody.querySelectorAll('[contenteditable]').forEach(td => td.oninput = e => {{ manual[Number(e.target.dataset.i)].label = e.target.textContent.trim(); updateOut(); draw(); }});
  updateOut();
}}
canvas.addEventListener('mousemove', ev => {{
  const r = canvas.getBoundingClientRect();
  const q = screenToRect(ev.clientX-r.left, ev.clientY-r.top);
  const w = rectToWorld(q.rx, q.ry);
  const n = nearest(q.rx, q.ry);
  hover = q;
  coord.textContent = 'x=' + w.x.toFixed(4) + ', y=' + w.y.toFixed(4);
  nearBox.textContent = n ? n.label + ' @ ' + n.x.toFixed(3) + ',' + n.y.toFixed(3) : '--';
  draw();
}});
canvas.addEventListener('mouseleave', () => {{ hover = null; coord.textContent='x=--, y=--'; nearBox.textContent='--'; draw(); }});
canvas.addEventListener('click', ev => {{
  const r = canvas.getBoundingClientRect();
  const q = screenToRect(ev.clientX-r.left, ev.clientY-r.top);
  const w = rectToWorld(q.rx, q.ry);
  manual.push({{x:w.x, y:w.y, label:''}});
  draw();
}});
['flipX','grid','rrt','risk','approach'].forEach(id => document.getElementById(id).onchange = draw);
document.getElementById('undo').onclick = () => {{ manual.pop(); draw(); }};
document.getElementById('clear').onclick = () => {{ manual = []; draw(); }};
document.getElementById('copyJson').onclick = () => navigator.clipboard.writeText(out.value);
document.getElementById('copyCsv').onclick = () => navigator.clipboard.writeText(['id,x,y,label'].concat(rows().map(p => p.id+','+p.x+','+p.y+','+String(p.label).replace(/,/g,' '))).join('\\n'));
document.getElementById('status').textContent = 'ready rrt=' + DATA.summary.rrt_count + ' risk=' + DATA.summary.risk_count;
document.getElementById('status').className = 'mono ok';
window.addEventListener('resize', resize);
resize();
</script>
</body>
</html>
"""


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--map-yaml", required=True, type=Path)
    parser.add_argument("--rrt-log", action="append", type=Path, default=[])
    parser.add_argument("--risk-events", action="append", type=Path, default=[])
    parser.add_argument("--confirmed-risk-points", action="append", type=Path, default=[])
    parser.add_argument("--approach-records", action="append", type=Path, default=[])
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("--name", default="k1_adaptive_map_rrt_risk")
    parser.add_argument("--title", default="K1 adaptive map RRT risk overlay")
    parser.add_argument("--rotation-deg", default="auto")
    parser.add_argument("--default-flip-x", action="store_true")
    return parser.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    args.output_dir.mkdir(parents=True, exist_ok=True)

    meta, gray, rgb = load_map(args.map_yaml)
    angle_info = detect_dominant_wall_angle_deg(gray)
    if str(args.rotation_deg).lower() == "auto":
        rotation_deg = -float(angle_info["dominant_angle_deg"])
    else:
        rotation_deg = float(args.rotation_deg)

    rectified, bounds = render_rectified_image(rgb, rotation_deg)
    image_name = f"{args.name}_rectified.png"
    Image.fromarray(rectified).save(args.output_dir / image_name)

    raw_height, raw_width = gray.shape[:2]
    rrt_points = attach_rect(parse_rrt_logs(args.rrt_log), meta, raw_width, raw_height, rotation_deg, bounds)
    risks = parse_risk_events(args.risk_events) + parse_confirmed_points(args.confirmed_risk_points)
    risk_points = attach_rect(risks, meta, raw_width, raw_height, rotation_deg, bounds)
    approach_points = attach_rect(parse_approach_records(args.approach_records), meta, raw_width, raw_height, rotation_deg, bounds)

    data = {
        "title": args.title,
        "raw_map": {
            "width": raw_width,
            "height": raw_height,
            "resolution": float(meta["resolution"]),
            "origin": meta.get("origin") or [0.0, 0.0, 0.0],
            "yaml": str(args.map_yaml),
        },
        "rectified_image": {"width": int(bounds["width"]), "height": int(bounds["height"])},
        "rotation_bounds": bounds,
        "display": {
            "image": image_name,
            "rotation_deg": round(rotation_deg, 3),
            "dominant_wall_angle_deg": angle_info["dominant_angle_deg"],
            "dominant_wall_confidence": angle_info["confidence"],
            "hough_line_count": angle_info["line_count"],
            "default_flip_x": bool(args.default_flip_x),
        },
        "rrt_points": rrt_points,
        "risk_points": risk_points,
        "approach_points": approach_points,
        "summary": {
            "rrt_count": len(rrt_points),
            "risk_count": len(risk_points),
            "risk_map_count": sum(1 for item in risk_points if item.get("frame_kind") == "map"),
            "risk_odom_approx_count": sum(1 for item in risk_points if item.get("frame_kind") == "odom_approx"),
            "approach_count": len(approach_points),
        },
    }

    data_path = args.output_dir / f"{args.name}.json"
    html_path = args.output_dir / f"{args.name}.html"
    static_path = args.output_dir / f"{args.name}_static.png"
    data_path.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")
    html_path.write_text(build_html(data), encoding="utf-8")
    write_static_overlay(rectified, data, static_path)

    print(json.dumps({"html": str(html_path), "static": str(static_path), "data": str(data_path), "summary": data["summary"], "display": data["display"]}, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
