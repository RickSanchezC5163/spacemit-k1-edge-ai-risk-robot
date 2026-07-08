"""Shared JSON/YAML helpers for primitive dry-run tooling."""

from __future__ import annotations

import json
from datetime import datetime
from pathlib import Path
from typing import Any, Dict

try:
    import yaml
except ImportError:  # pragma: no cover
    yaml = None  # type: ignore


def now_iso() -> str:
    return datetime.now().isoformat(timespec="milliseconds")


def read_json(path: str | Path) -> Any:
    return json.loads(Path(path).read_text(encoding="utf-8"))


def write_json(path: str | Path, data: Any) -> None:
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def write_text(path: str | Path, text: str) -> None:
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(text, encoding="utf-8")


def read_yaml(path: str | Path) -> Dict[str, Any]:
    if yaml is None:
        raise RuntimeError("PyYAML is required for primitive config files")
    data = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError(f"{path} must contain a YAML mapping")
    return data


def write_yaml(path: str | Path, data: Dict[str, Any]) -> None:
    if yaml is None:
        raise RuntimeError("PyYAML is required for primitive config files")
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(yaml.safe_dump(data, sort_keys=False, allow_unicode=True), encoding="utf-8")


def simple_validate_required(data: Dict[str, Any], required: list[str], name: str) -> list[str]:
    errors: list[str] = []
    for key in required:
        if key not in data:
            errors.append(f"{name}: missing required key {key}")
    return errors
