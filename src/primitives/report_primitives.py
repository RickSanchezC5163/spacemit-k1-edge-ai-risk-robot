"""Deterministic and local-LLM report interface."""

from __future__ import annotations

import time
from pathlib import Path
from typing import Any, Dict

from .schemas import now_iso, read_json, read_yaml, write_json, write_text


ROOT = Path(__file__).resolve().parents[2]
DEFAULT_LLM_CONFIG = ROOT / "configs" / "local_llm_config.yaml"


def _benchmark(backend: str, local_llm_used: bool, elapsed_ms: float) -> Dict[str, Any]:
    return {
        "backend": backend,
        "local_llm_used": local_llm_used,
        "online_api_used": False,
        "model_name": "none" if not local_llm_used else backend,
        "model_size_mb": 0,
        "ttft_ms": None if not local_llm_used else round(elapsed_ms, 3),
        "tokens_per_second": None if not local_llm_used else 0.0,
        "total_tokens": 0,
        "peak_memory_mb": None,
    }


def generate_risk_report(
    episode_report_path: str,
    risk_map_summary_path: str | None,
    backend: str = "deterministic",
    output_dir: str | None = None,
) -> Dict[str, Any]:
    start = time.perf_counter()
    cfg = read_yaml(DEFAULT_LLM_CONFIG)
    info = ((cfg.get("backends") or {}).get(backend) or {})
    if backend not in (cfg.get("backends") or {}) or info.get("available") is not True:
        backend = cfg.get("default_backend", "deterministic")
        info = ((cfg.get("backends") or {}).get(backend) or {})

    episode = read_json(episode_report_path)
    risk_map = read_json(risk_map_summary_path) if risk_map_summary_path else {}
    risk_count = int(risk_map.get("risk_count_total", 0))
    status = (episode.get("summary") or {}).get("status") or episode.get("status") or "unknown"
    lines = [
        "# Risk Control Report",
        "",
        f"- generated_at: `{now_iso()}`",
        f"- backend: `{backend}`",
        f"- source_episode: `{episode_report_path}`",
        f"- episode_status: `{status}`",
        f"- risk_count_total: `{risk_count}`",
        f"- hardware_executed: `{(episode.get('summary') or {}).get('hardware_executed', False)}`",
        "",
        "## Claim Boundary",
        "",
        "- Deterministic backend is not a real LLM.",
        "- Local LLM claims require llama.cpp benchmark evidence.",
        "- No online API is used.",
    ]
    elapsed = (time.perf_counter() - start) * 1000.0
    benchmark = _benchmark(backend, bool(info.get("local_llm_used") is True), elapsed)
    result = {
        "schema_version": "local_report_interface_v1",
        "generated_at": now_iso(),
        "backend": backend,
        "source_episode_report": episode_report_path,
        "risk_map_summary_path": risk_map_summary_path,
        "risk_count_total": risk_count,
        "report_markdown": "\n".join(lines) + "\n",
        "llm_benchmark": benchmark,
        "claim_boundary": [
            "deterministic backend is not a real LLM",
            "local LLM claims require benchmark data",
            "online_api_used=false",
        ],
    }
    if output_dir:
        out = Path(output_dir)
        write_text(out / "risk_control_report.md", result["report_markdown"])
        write_json(out / "risk_control_report.json", result)
        write_json(out / "llm_benchmark.json", benchmark)
        write_text(out / "prompt.txt", "Summarize episode risk state and safety boundaries.\n")
        write_text(out / "raw_output.txt", result["report_markdown"])
        write_text(out / "claim_boundary.md", "\n".join(f"- {x}" for x in result["claim_boundary"]) + "\n")
    return result
