"""D435 primitive dry-run helpers."""

from __future__ import annotations

from pathlib import Path
from typing import Any, Dict

from .safety_gate import evaluate_safety_gate
from .schemas import now_iso


def d435_capture_dryrun(output_dir: str | Path, observation: Dict[str, Any] | None = None) -> Dict[str, Any]:
    output = Path(output_dir)
    output.mkdir(parents=True, exist_ok=True)
    gate = evaluate_safety_gate("D435_CAPTURE", observation or {"base_zero": True}, execution_mode="dry_run")
    return {
        "capture_id": "dryrun_d435_capture",
        "primitive": "D435_CAPTURE",
        "status": "succeeded_dry_run" if gate["allowed"] else "blocked",
        "timestamp": now_iso(),
        "requires_base_zero": True,
        "base_zero_ok_before": gate.get("base_zero_ok"),
        "published_cmd_vel": False,
        "hardware_executed": False,
        "evidence_paths": {
            "rgb": str(output / "fixture_rgb.png"),
            "depth_raw": str(output / "fixture_depth_raw.npy"),
            "camera_info": str(output / "fixture_camera_info.json"),
            "odom": str(output / "fixture_odom.json"),
            "capture_meta": str(output / "fixture_capture_meta.json"),
        },
        "safety_gate": gate,
    }
