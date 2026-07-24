import json
from pathlib import Path

from tools.generate_k1_autonomous_report_bundle import (
    collect_autonomy_evidence,
    collect_risks,
    deterministic_summary,
    render_html,
)


def test_collects_formal_risks_and_autonomy_evidence(tmp_path: Path) -> None:
    yolo = tmp_path / "yolo_risk"
    maps = tmp_path / "maps"
    yolo.mkdir()
    maps.mkdir()
    event = {
        "event_id": "risk-1",
        "risk_id": "spatial-1",
        "candidate_kind": "formal",
        "class_name": "blockage",
        "confidence": 0.81,
        "distance_m": 0.42,
        "map_point_xy_m": {"x": 1.0, "y": 2.0},
    }
    (yolo / "risk_events.jsonl").write_text(json.dumps(event) + "\n", encoding="utf-8")
    confirmed = tmp_path / "risk_approach"
    confirmed.mkdir()
    (confirmed / "confirmed_risk_map_points.json").write_text(
        json.dumps({"risk_map_points": [{**event, "confidence": 0.92}]}) + "\n",
        encoding="utf-8",
    )
    (maps / "map.yaml").write_text("image: map.pgm\n", encoding="utf-8")
    (tmp_path / "rrt_frontier_nav2_report.json").write_text("{}\n", encoding="utf-8")

    risks = collect_risks(tmp_path)
    autonomy = collect_autonomy_evidence(tmp_path)
    assert len(risks) == 1
    assert risks[0]["class_name"] == "blockage"
    assert risks[0]["confidence"] == 0.92
    assert autonomy["autonomous_mapping_supported"] is True
    assert "自主探索建图" in deterministic_summary(risks, autonomy)


def test_html_states_autonomous_evidence_boundary(tmp_path: Path) -> None:
    payload = {
        "generated_at": "2026-07-25T00:00:00+08:00",
        "risks": [],
        "narrative": "测试摘要",
        "autonomy": {"autonomous_mapping_supported": True},
    }
    output = render_html(payload)
    assert "Frontier/RRT 运行报告" in output
    assert "自主探索" in output
