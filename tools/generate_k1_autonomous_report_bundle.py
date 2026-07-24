#!/usr/bin/env python3
"""Generate JSON, HTML and PDF reports from an autonomous inspection run."""

from __future__ import annotations

import argparse
import html
import json
import os
import re
import shutil
import subprocess
import time
from collections import Counter
from datetime import datetime
from pathlib import Path
from typing import Any


DEFAULT_MODEL = Path("/home/soc/local-llm/models/qwen2.5-0.5b-instruct-q4_k_m.gguf")
DEFAULT_LLAMA = Path("/home/soc/local-llm/pkg/llama-tools-spacemit/usr/bin/llama-cli")
DEFAULT_LLAMA_LIB = Path("/home/soc/local-llm/pkg/llama-tools-spacemit/usr/lib")
CLASS_ZH = {"crack": "裂缝", "corrosion": "腐蚀", "blockage": "障碍", "leakage": "泄漏"}


def read_json(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8"))


def write_json(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    os.replace(temporary, path)


def load_jsonl(path: Path) -> list[dict[str, Any]]:
    if not path.is_file():
        return []
    rows: list[dict[str, Any]] = []
    for line in path.read_text(encoding="utf-8").splitlines():
        if line.strip():
            rows.append(json.loads(line))
    return rows


def normalize_risk(item: dict[str, Any]) -> dict[str, Any]:
    point = item.get("map_point_xy_m") or item.get("latest_map_point_xy_m") or {}
    return {
        "risk_id": str(item.get("risk_id") or item.get("event_id") or "unknown"),
        "event_id": item.get("event_id"),
        "class_name": str(item.get("class_name") or "unknown"),
        "risk_level": str(item.get("risk_level") or "unknown"),
        "confidence": item.get("confidence_max", item.get("confidence")),
        "distance_m": item.get("latest_distance_m", item.get("distance_m")),
        "map_x_m": point.get("x"),
        "map_y_m": point.get("y"),
        "recommended_action": item.get("recommended_action"),
        "projection_status": item.get("projection_status"),
        "evidence_path": item.get("overlay_path") or (item.get("best_evidence") or {}).get("overlay_path"),
    }


def collect_risks(run_dir: Path) -> list[dict[str, Any]]:
    candidates: list[dict[str, Any]] = []
    confirmed = run_dir / "risk_approach" / "confirmed_risk_map_points.json"
    if confirmed.is_file():
        payload = read_json(confirmed)
        candidates.extend(payload if isinstance(payload, list) else payload.get("risk_map_points", []))

    points = run_dir / "yolo_risk" / "risk_map_points.json"
    if points.is_file():
        payload = read_json(points)
        candidates.extend(payload if isinstance(payload, list) else payload.get("risk_map_points", []))

    for item in load_jsonl(run_dir / "yolo_risk" / "risk_events.jsonl"):
        gate = item.get("auto_risk_gate") or {}
        if item.get("candidate_kind") == "formal" or gate.get("allowed") is True:
            candidates.append(item)

    deduplicated: dict[str, dict[str, Any]] = {}
    for raw in candidates:
        risk = normalize_risk(raw)
        key = str(risk.get("event_id") or risk["risk_id"])
        current = deduplicated.get(key)
        if current is None or float(risk.get("confidence") or 0) > float(current.get("confidence") or 0):
            deduplicated[key] = risk
    return sorted(deduplicated.values(), key=lambda item: (item["class_name"], item["risk_id"]))


def collect_autonomy_evidence(run_dir: Path) -> dict[str, Any]:
    rrt_reports = sorted(run_dir.glob("rrt_frontier_*report.json"))
    map_files = sorted((run_dir / "maps").glob("*.yaml")) if (run_dir / "maps").is_dir() else []
    return {
        "autonomous_mapping_supported": bool(rrt_reports and map_files),
        "method": "SLAM + Frontier/RRT goal selection + Nav2 path execution",
        "rrt_reports": [str(path) for path in rrt_reports],
        "saved_maps": [str(path) for path in map_files],
    }


def build_prompt(risks: list[dict[str, Any]], autonomy: dict[str, Any]) -> str:
    evidence_statement = (
        "本轮运行目录同时包含RRT探索报告和SLAM地图，可表述为自主探索建图。"
        if autonomy["autonomous_mapping_supported"]
        else "本轮自主建图证据不完整，只能表述为部分运行记录。"
    )
    lines = [
        "你是部署在K1机器人本地的离线巡检报告模型。",
        "只能依据下列结构化实测数据生成两段中文摘要，不得虚构风险、坐标或动作结果。",
        evidence_statement,
    ]
    for risk in risks:
        lines.append(
            f"{CLASS_ZH.get(risk['class_name'], risk['class_name'])}: "
            f"置信度={float(risk.get('confidence') or 0):.3f}, "
            f"深度={float(risk.get('distance_m') or 0):.3f}m, "
            f"地图坐标=({risk.get('map_x_m')},{risk.get('map_y_m')})"
        )
    lines.extend(["首行写“报告正文开始”，末行写“报告正文结束”，总长度不超过160个汉字。"])
    return "\n".join(lines)


def extract_marked_report(raw: str) -> str:
    match = re.search(r"报告正文开始\s*(.*?)\s*报告正文结束", raw, flags=re.S)
    if match:
        return match.group(1).strip()
    cleaned = raw.strip()
    if not cleaned:
        raise RuntimeError("local Qwen returned no report text")
    return cleaned[-600:]


def run_qwen(
    prompt: str, model: Path, llama: Path, llama_lib: Path
) -> tuple[str, dict[str, Any], str]:
    env = os.environ.copy()
    env["LD_LIBRARY_PATH"] = str(llama_lib) + ":" + env.get("LD_LIBRARY_PATH", "")
    env["LC_ALL"] = "C.UTF-8"
    env["LANG"] = "C.UTF-8"
    started = time.perf_counter()
    result = subprocess.run(
        [
            str(llama), "-m", str(model), "-p", prompt, "-n", "140", "-t", "4",
            "--temp", "0.2", "--top-p", "0.9", "--log-disable", "--no-display-prompt",
            "--simple-io", "-cnv", "-st",
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        encoding="utf-8",
        errors="replace",
        env=env,
        timeout=300,
        check=False,
    )
    elapsed = time.perf_counter() - started
    if result.returncode != 0:
        raise RuntimeError(f"local Qwen failed with exit code {result.returncode}")
    return (
        extract_marked_report(result.stdout),
        {
            "backend": "llama.cpp",
            "model": str(model),
            "elapsed_s": round(elapsed, 3),
            "returncode": result.returncode,
            "local_llm_used": True,
            "online_api_used": False,
            "modalities": "text from structured D435+YOLO+SLAM evidence",
        },
        result.stdout,
    )


def deterministic_summary(risks: list[dict[str, Any]], autonomy: dict[str, Any]) -> str:
    counts = Counter(CLASS_ZH.get(item["class_name"], item["class_name"]) for item in risks)
    risk_text = "、".join(f"{name}{count}处" for name, count in sorted(counts.items())) or "未形成正式风险点"
    mapping = "机器人通过SLAM与Frontier/RRT/Nav2完成自主探索建图。" if autonomy["autonomous_mapping_supported"] else "本轮自主建图证据不完整。"
    return f"{mapping}共记录{risk_text}。检测结果用于风险复核和人工处置，不替代工程确诊。"


def render_html(payload: dict[str, Any]) -> str:
    rows = []
    for index, risk in enumerate(payload["risks"], start=1):
        coordinate = "-" if risk.get("map_x_m") is None else f"({float(risk['map_x_m']):.2f}, {float(risk['map_y_m']):.2f})"
        confidence = "-" if risk.get("confidence") is None else f"{float(risk['confidence']):.3f}"
        distance = "-" if risk.get("distance_m") is None else f"{float(risk['distance_m']):.2f} m"
        rows.append(
            "<tr>"
            f"<td>{index}</td><td>{html.escape(CLASS_ZH.get(risk['class_name'], risk['class_name']))}</td>"
            f"<td>{confidence}</td><td>{distance}</td><td>{coordinate}</td>"
            f"<td>{html.escape(str(risk.get('recommended_action') or '-'))}</td></tr>"
        )
    autonomy = payload["autonomy"]
    status = "证据完整" if autonomy["autonomous_mapping_supported"] else "证据不完整"
    mapping_label = "自主探索" if autonomy["autonomous_mapping_supported"] else "待核验"
    return f"""<!doctype html>
<html lang="zh-CN"><head><meta charset="utf-8"><title>K1 自主巡检报告</title>
<style>
body{{font-family:Arial,"Microsoft YaHei",sans-serif;margin:0;color:#17202a;background:#f4f6f7}}
header{{background:#16324f;color:white;padding:26px 7%}} main{{max-width:1080px;margin:auto;padding:24px}}
section{{background:white;border:1px solid #d5d8dc;border-radius:6px;padding:18px;margin-bottom:16px}}
h1,h2{{letter-spacing:0}} table{{width:100%;border-collapse:collapse}} th,td{{padding:9px;border-bottom:1px solid #ddd;text-align:left}}
.meta{{display:grid;grid-template-columns:repeat(3,1fr);gap:12px}} .value{{font-size:22px;font-weight:700}}
@media(max-width:700px){{.meta{{grid-template-columns:1fr}} table{{font-size:12px}}}}
</style></head><body><header><h1>K1 自主巡检风险报告</h1><div>{html.escape(payload['generated_at'])}</div></header><main>
<section class="meta"><div><div>建图链路</div><div class="value">{mapping_label}</div></div><div><div>风险数量</div><div class="value">{len(payload['risks'])}</div></div><div><div>证据状态</div><div class="value">{status}</div></div></section>
<section><h2>本地模型摘要</h2><p>{html.escape(payload['narrative'])}</p></section>
<section><h2>风险明细</h2><table><thead><tr><th>#</th><th>类型</th><th>置信度</th><th>深度</th><th>地图坐标</th><th>建议</th></tr></thead><tbody>{''.join(rows)}</tbody></table></section>
<section><h2>证据边界</h2><p>建图结论来自保存的 SLAM 地图与 Frontier/RRT 运行报告；风险来自 D435 深度与 YOLO 结构化结果。</p></section>
</main></body></html>"""


def publish_latest(root: Path, report_dir: Path) -> None:
    root.mkdir(parents=True, exist_ok=True)
    latest = root / "latest"
    temporary = root / f".latest-{os.getpid()}"
    temporary.unlink(missing_ok=True)
    temporary.symlink_to(report_dir.name, target_is_directory=True)
    if latest.is_symlink() or latest.is_file():
        latest.unlink()
    elif latest.exists():
        latest.rename(root / f"previous_{datetime.now().strftime('%Y%m%d_%H%M%S')}")
    os.replace(temporary, latest)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", type=Path, required=True)
    parser.add_argument("--reports-root", type=Path, default=Path("outputs/k1_autonomous_reports"))
    parser.add_argument("--backend", choices=("auto", "qwen", "deterministic"), default="auto")
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL)
    parser.add_argument("--llama-cli", type=Path, default=DEFAULT_LLAMA)
    parser.add_argument("--llama-lib", type=Path, default=DEFAULT_LLAMA_LIB)
    parser.add_argument("--chrome", type=Path, default=Path("/usr/bin/chromium-browser"))
    parser.add_argument("--no-pdf", action="store_true")
    args = parser.parse_args()

    run_dir = args.run_dir.resolve()
    reports_root = args.reports_root.resolve()
    report_dir = reports_root / run_dir.name
    if report_dir.exists():
        if report_dir.parent != reports_root or report_dir == reports_root:
            raise RuntimeError(f"refusing to replace report path: {report_dir}")
        shutil.rmtree(report_dir)
    report_dir.mkdir(parents=True, exist_ok=True)
    risks = collect_risks(run_dir)
    autonomy = collect_autonomy_evidence(run_dir)
    prompt = build_prompt(risks, autonomy)
    use_qwen = args.backend == "qwen" or (
        args.backend == "auto" and args.model.is_file() and args.llama_cli.is_file()
    )
    if use_qwen:
        narrative, llm, raw_llm = run_qwen(prompt, args.model, args.llama_cli, args.llama_lib)
    else:
        narrative = deterministic_summary(risks, autonomy)
        llm = {"backend": "deterministic", "local_llm_used": False, "online_api_used": False}
        raw_llm = ""
    payload = {
        "schema_version": "k1_autonomous_inspection_report_v1",
        "generated_at": datetime.now().astimezone().isoformat(timespec="seconds"),
        "run_dir": str(run_dir),
        "autonomy": autonomy,
        "risks": risks,
        "risk_count": len(risks),
        "narrative": narrative,
        "llm": llm,
    }
    write_json(report_dir / "report.json", payload)
    (report_dir / "local_llm_prompt.txt").write_text(prompt + "\n", encoding="utf-8")
    if raw_llm:
        (report_dir / "local_llm_raw.log").write_text(raw_llm, encoding="utf-8")
    html_text = render_html(payload)
    (report_dir / "index.html").write_text(html_text, encoding="utf-8")
    if not args.no_pdf:
        if not args.chrome.is_file():
            raise RuntimeError(f"Chromium is required for PDF rendering: {args.chrome}")
        subprocess.run(
            [str(args.chrome), "--headless", "--disable-gpu", "--no-sandbox", f"--print-to-pdf={report_dir / 'report.pdf'}", (report_dir / "index.html").as_uri()],
            check=True,
            timeout=180,
        )
    publish_latest(reports_root, report_dir)
    print(json.dumps({"ok": True, "report_dir": str(report_dir), "risk_count": len(risks), "autonomous_mapping": autonomy["autonomous_mapping_supported"]}, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
