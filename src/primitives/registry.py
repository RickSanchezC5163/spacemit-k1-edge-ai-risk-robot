"""Primitive registry and semantic validation helpers."""

from __future__ import annotations

from pathlib import Path
from typing import Any, Dict, Iterable, List

from .schemas import read_yaml


ROOT = Path(__file__).resolve().parents[2]
DEFAULT_REGISTRY = ROOT / "configs" / "primitive_registry.yaml"
DEFAULT_ACTION_SEMANTICS = ROOT / "configs" / "action_semantics.yaml"


def load_registry(path: str | Path = DEFAULT_REGISTRY) -> Dict[str, Any]:
    return read_yaml(path)


def load_action_semantics(path: str | Path = DEFAULT_ACTION_SEMANTICS) -> Dict[str, Any]:
    return read_yaml(path)


def primitive_names(registry: Dict[str, Any]) -> set[str]:
    return set((registry.get("primitives") or {}).keys())


def category_names(registry: Dict[str, Any]) -> set[str]:
    return set((registry.get("categories") or {}).keys())


def validate_registry(registry: Dict[str, Any]) -> List[str]:
    errors: List[str] = []
    if registry.get("schema_version") != "primitive_registry_v1":
        errors.append("schema_version must be primitive_registry_v1")
    primitives = registry.get("primitives")
    categories = registry.get("categories")
    if not isinstance(primitives, dict) or not primitives:
        errors.append("primitives must be a non-empty mapping")
        return errors
    if not isinstance(categories, dict) or not categories:
        errors.append("categories must be a non-empty mapping")
        return errors

    known_categories = set(categories.keys())
    listed: set[str] = set()
    for category, names in categories.items():
        if not isinstance(names, list):
            errors.append(f"category {category} must list primitive names")
            continue
        listed.update(str(name) for name in names)

    for name, item in primitives.items():
        if not isinstance(item, dict):
            errors.append(f"primitive {name} must be a mapping")
            continue
        category = item.get("category")
        if category not in known_categories:
            errors.append(f"primitive {name} has unknown category {category}")
        if name not in listed:
            errors.append(f"primitive {name} is not listed in categories")
        if item.get("category") == "chassis" and item.get("publishes_cmd_vel") is True:
            if item.get("publish_topic") != "/input_cmd_vel":
                errors.append(f"chassis primitive {name} must publish only /input_cmd_vel")
            if item.get("direct_cmd_vel_guarded_allowed") is not False:
                errors.append(f"chassis primitive {name} must forbid direct /cmd_vel_guarded")
        if item.get("category") == "arm":
            if item.get("requires_base_zero") is not True:
                errors.append(f"arm primitive {name} must require base_zero")
    return errors


def validate_action_semantics(semantics: Dict[str, Any], registry: Dict[str, Any]) -> List[str]:
    errors: List[str] = []
    if semantics.get("schema_version") != "action_semantics_v1":
        errors.append("schema_version must be action_semantics_v1")
    actions = semantics.get("actions")
    if not isinstance(actions, dict) or not actions:
        errors.append("actions must be a non-empty mapping")
        return errors
    names = primitive_names(registry)
    for name, item in actions.items():
        if name not in names:
            errors.append(f"action {name} is not in primitive registry")
        if not isinstance(item, dict):
            errors.append(f"action {name} must be a mapping")
            continue
        if item.get("real_enabled") is True and name.startswith("ARM_"):
            if item.get("requires_base_zero") is not True:
                errors.append(f"real-enabled arm action {name} must require base_zero")
        if name in {"FORWARD_0P15", "ARC_FAST_LEFT", "ARC_FAST_RIGHT"}:
            boundary = semantics.get("real_vehicle_boundary") or {}
            if boundary.get("no_direct_cmd_vel_guarded") is not True:
                errors.append("real_vehicle_boundary must forbid direct /cmd_vel_guarded")
    return errors


def ordered_enabled_actions(action_space: Dict[str, Any], include_disabled: bool = False) -> List[Dict[str, Any]]:
    actions = action_space.get("actions") or []
    selected = []
    for action in actions:
        if include_disabled or action.get("enabled_by_default") is True:
            selected.append(action)
    return sorted(selected, key=lambda item: int(item.get("id", 0)))
