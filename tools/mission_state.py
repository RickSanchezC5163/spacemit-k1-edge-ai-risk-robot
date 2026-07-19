#!/usr/bin/env python3
"""Bounded in-memory mission state with atomic snapshots."""

from __future__ import annotations

import json
import os
import tempfile
import time
from collections import OrderedDict
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, Iterable, Optional


def now_iso() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


def atomic_write_json(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary = tempfile.mkstemp(prefix=f".{path.name}.", dir=str(path.parent))
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as handle:
            json.dump(value, handle, ensure_ascii=False, indent=2)
            handle.write("\n")
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, path)
    except BaseException:
        try:
            os.unlink(temporary)
        except FileNotFoundError:
            pass
        raise


class MissionStateStore:
    SCHEMA_VERSION = "k1_mission_state_v2"

    def __init__(
        self,
        path: Path,
        max_candidates: int = 64,
        candidate_ttl_s: float = 60.0,
    ) -> None:
        self.path = Path(path)
        self.max_candidates = max(1, int(max_candidates))
        self.candidate_ttl_s = max(1.0, float(candidate_ttl_s))
        self.candidates: "OrderedDict[str, Dict[str, Any]]" = OrderedDict()
        self.confirmed_risks: "OrderedDict[str, Dict[str, Any]]" = OrderedDict()
        self._dirty = True
        self._started_at = now_iso()
        self.interfaces = {
            "visualization": {"state": "reserved", "input": "mission_state.json"},
            "arm": {"state": "reserved", "input": "confirmed blockage"},
            "llm_report": {"state": "reserved", "input": "confirmed risks + evidence refs"},
            "voice": {"state": "reserved", "input": "mission/risk transition events"},
        }

    def _prune(self, now_monotonic: Optional[float] = None) -> None:
        now_monotonic = time.monotonic() if now_monotonic is None else now_monotonic
        expired = [
            key
            for key, value in self.candidates.items()
            if now_monotonic - float(value.get("last_seen_monotonic", now_monotonic))
            > self.candidate_ttl_s
        ]
        for key in expired:
            self.candidates.pop(key, None)
            self._dirty = True
        while len(self.candidates) > self.max_candidates:
            self.candidates.popitem(last=False)
            self._dirty = True

    def update_candidate(self, key: str, candidate: Dict[str, Any]) -> Dict[str, Any]:
        now_monotonic = time.monotonic()
        existing = self.candidates.get(key)
        confidence = float(candidate.get("confidence") or 0.0)
        if existing is None:
            stored = dict(candidate)
            stored.update(
                {
                    "candidate_key": key,
                    "first_seen": candidate.get("first_seen") or now_iso(),
                    "last_seen": candidate.get("last_seen") or now_iso(),
                    "seen_count": 1,
                    "confidence_max": confidence,
                    "last_seen_monotonic": now_monotonic,
                }
            )
            self.candidates[key] = stored
        else:
            stored = existing
            stored["last_seen"] = candidate.get("last_seen") or now_iso()
            stored["last_seen_monotonic"] = now_monotonic
            stored["seen_count"] = int(stored.get("seen_count", 1)) + 1
            stored["latest_confidence"] = confidence
            if confidence >= float(stored.get("confidence_max") or 0.0):
                preserved = {
                    "first_seen": stored.get("first_seen"),
                    "seen_count": stored.get("seen_count"),
                }
                stored.update(candidate)
                stored.update(preserved)
                stored["confidence_max"] = confidence
                stored["best_evidence_updated_at"] = now_iso()
            self.candidates.move_to_end(key)
        self._dirty = True
        self._prune(now_monotonic)
        return stored

    def confirm_risk(self, risk: Dict[str, Any]) -> Dict[str, Any]:
        risk_id = str(risk.get("risk_id") or risk.get("event_id") or f"risk_{len(self.confirmed_risks) + 1}")
        stored = dict(risk)
        stored.pop("last_seen_monotonic", None)
        stored["risk_id"] = risk_id
        stored["confirmed_at"] = stored.get("confirmed_at") or now_iso()
        self.confirmed_risks[risk_id] = stored
        candidate_key = stored.get("candidate_key") or stored.get("dedup_key") or risk_id
        if candidate_key:
            self.candidates.pop(str(candidate_key), None)
        self._dirty = True
        return stored

    def snapshot(self) -> Dict[str, Any]:
        self._prune()
        candidates = []
        for value in self.candidates.values():
            item = dict(value)
            item.pop("last_seen_monotonic", None)
            candidates.append(item)
        return {
            "schema_version": self.SCHEMA_VERSION,
            "started_at": self._started_at,
            "updated_at": now_iso(),
            "coordinate_contract": {
                "navigation_frame": "map",
                "sensor_projection": "capture_time_d435_optical_to_base_to_odom_to_map",
                "confirmed_risk_required_fields": [
                    "risk_id",
                    "class_name",
                    "confidence",
                    "coordinate.frame_id",
                    "coordinate.xy_m",
                    "evidence_refs",
                ],
            },
            "limits": {
                "candidate_capacity": self.max_candidates,
                "candidate_ttl_s": self.candidate_ttl_s,
            },
            "candidate_count": len(candidates),
            "confirmed_risk_count": len(self.confirmed_risks),
            "candidates": candidates,
            "confirmed_risks": list(self.confirmed_risks.values()),
            "interfaces": self.interfaces,
        }

    def write_if_dirty(self, force: bool = False) -> bool:
        self._prune()
        if not force and not self._dirty:
            return False
        atomic_write_json(self.path, self.snapshot())
        self._dirty = False
        return True

    def ingest_confirmed(self, risks: Iterable[Dict[str, Any]]) -> None:
        for risk in risks:
            self.confirm_risk(risk)
