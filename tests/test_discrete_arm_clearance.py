from __future__ import annotations

import json
import subprocess
import sys
import tempfile
import unittest
from datetime import datetime, timezone
from pathlib import Path

from src.discrete_arm_clearance import (
    REQUIRED_PHASE_ORDER,
    audit_sequence_with_arm_safety,
    classify_grid,
    evaluate_base_zero_evidence,
    summarize_marker_observations,
    validate_clearance_config,
    validate_sequence_document,
    verify_yolo_result,
)


ROOT = Path(__file__).resolve().parents[1]


def valid_config() -> dict:
    return {
        "schema_version": "arm_discrete_clearance_v1",
        "camera_mount_mode": "BODY_FIXED",
        "near_camera": {
            "device": 0,
            "stabilize_frames": 2,
            "capture_frames": 7,
            "frame_timeout_s": 5.0,
        },
        "object_verification": {
            "yolo_class": "blockage",
            "confidence_min": 0.6,
            "require_single_detection": True,
        },
        "marker_detection": {
            "method": "HSV_CONTOUR",
            "calibrated": True,
            "hsv_lower": [0, 120, 100],
            "hsv_upper": [10, 255, 255],
            "min_area_px": 100,
            "stable_frames_required": 3,
            "max_center_jitter_px": 4.0,
        },
        "grid_regions": {
            "LEFT": {"xyxy_px": [0, 0, 90, 100]},
            "CENTER": {"xyxy_px": [110, 0, 190, 100]},
            "RIGHT": {"xyxy_px": [210, 0, 300, 100]},
        },
        "reject_regions": [
            {"name": "left_center", "xyxy_px": [90, 0, 110, 100]},
            {"name": "center_right", "xyxy_px": [190, 0, 210, 100]},
        ],
        "sequence_paths": {"LEFT": "left.json", "CENTER": "center.json", "RIGHT": "right.json"},
        "base_zero": {"required": True, "stable_duration_s": 1.0},
        "hardware": {
            "enabled": False,
            "one_shot_confirmation_required": True,
            "automatic_retry": False,
            "automatic_home_on_fault": False,
        },
    }


def valid_sequence(grid_id: str = "CENTER") -> dict:
    home = {"1": 510, "2": 771, "3": 426, "4": 503, "5": 497}
    return {
        "schema_version": "arm_discrete_sequence_v1",
        "grid_id": grid_id,
        "verified": True,
        "calibration_required": False,
        "calibration_id": "test-calibration",
        "verified_by": "unit-test",
        "verified_at": "2026-07-23T00:00:00+00:00",
        "interpolation_allowed": False,
        "workspace_verified": False,
        "workspace_evidence": None,
        "sequence": [
            {"name": f"test_{phase}", "phase": phase, "servos": dict(home), "duration_ms": 1000}
            for phase in REQUIRED_PHASE_ORDER
        ],
    }


class DiscreteArmClearanceTests(unittest.TestCase):
    def test_valid_config(self) -> None:
        errors, _warnings = validate_clearance_config(valid_config())
        self.assertEqual(errors, [])

    def test_arm_tip_camera_requires_fixed_observation_contract(self) -> None:
        config = valid_config()
        config["camera_mount_mode"] = "ARM_MOVING_FIXED_OBSERVATION"
        errors, _warnings = validate_clearance_config(config)
        self.assertIn(
            "near_camera.mount_location must be ARM_TIP for the configured mount mode",
            errors,
        )
        config["near_camera"]["mount_location"] = "ARM_TIP"
        config["near_camera"]["localization_valid_only_at_observation_pose"] = True
        errors, _warnings = validate_clearance_config(config)
        self.assertEqual(errors, [])

    def test_three_grids_and_reject_bands(self) -> None:
        config = valid_config()
        self.assertEqual(classify_grid((40, 50), config)["grid_id"], "LEFT")
        self.assertEqual(classify_grid((150, 50), config)["grid_id"], "CENTER")
        self.assertEqual(classify_grid((250, 50), config)["grid_id"], "RIGHT")
        self.assertEqual(classify_grid((100, 50), config)["decision"], "REJECT")
        self.assertEqual(classify_grid((400, 50), config)["reason"], "center_outside_calibrated_grids")

    def test_stable_marker_summary(self) -> None:
        observations = [
            {"detected": True, "center_uv": [148, 50], "area_px": 1200},
            {"detected": True, "center_uv": [150, 51], "area_px": 1210},
            {"detected": True, "center_uv": [151, 49], "area_px": 1190},
        ]
        result = summarize_marker_observations(observations, valid_config())
        self.assertEqual(result["decision"], "ACCEPT")
        self.assertEqual(result["grid_id"], "CENTER")

    def test_marker_jitter_is_rejected(self) -> None:
        observations = [
            {"detected": True, "center_uv": [120, 50], "area_px": 1200},
            {"detected": True, "center_uv": [150, 50], "area_px": 1200},
            {"detected": True, "center_uv": [180, 50], "area_px": 1200},
        ]
        result = summarize_marker_observations(observations, valid_config())
        self.assertEqual(result["decision"], "REJECT")
        self.assertIn("marker_jitter_too_large", result["reason"])

    def test_yolo_requires_one_matching_object(self) -> None:
        config = valid_config()
        accepted = verify_yolo_result(
            {"detections": [{"class_name": "blockage", "confidence": 0.84}]}, config
        )
        self.assertTrue(accepted["accepted"])
        rejected = verify_yolo_result(
            {
                "detections": [
                    {"class_name": "blockage", "confidence": 0.84},
                    {"class_name": "blockage", "confidence": 0.81},
                ]
            },
            config,
        )
        self.assertFalse(rejected["accepted"])

    def test_placeholder_sequence_is_rejected(self) -> None:
        sequence = valid_sequence()
        sequence["verified"] = False
        sequence["calibration_required"] = True
        sequence["sequence"][1]["servos"]["2"] = 0
        errors = validate_sequence_document(sequence, "CENTER")
        self.assertTrue(any("verified" in error for error in errors))
        self.assertTrue(any("placeholder pulse 0" in error for error in errors))

    def test_arm_safety_audit_accepts_safe_dry_run_fixture(self) -> None:
        arm_config = json.loads((ROOT / "configs" / "arm_safety_config.json").read_text(encoding="utf-8"))
        arm_config["joints"]["5"]["gripper_open_pulse"] = 360
        with tempfile.TemporaryDirectory() as temporary:
            config_path = Path(temporary) / "arm_safety.json"
            config_path.write_text(json.dumps(arm_config), encoding="utf-8")
            result = audit_sequence_with_arm_safety(
                valid_sequence(),
                config_path,
                execution_stage="no_load",
                hardware_mode=False,
                workspace_policy={"hardware_policy": "REQUIRE_ESTIMATE_PASS", "allow_preverified_override": False},
            )
        self.assertTrue(result["allowed"], result["errors"])

    def test_live_base_zero_evidence(self) -> None:
        evidence = {
            "evidence_type": "live_base_zero_observation",
            "valid_for_arm_c1_hardware": True,
            "generated_at": datetime.now(timezone.utc).isoformat(timespec="seconds"),
            "base_zero_ok_before_arm": True,
            "published_cmd_vel": False,
        }
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "base_zero.json"
            path.write_text(json.dumps(evidence), encoding="utf-8")
            result = evaluate_base_zero_evidence(path, max_age_s=10.0)
        self.assertTrue(result["allowed"], result["errors"])

    def test_cli_center_dry_run_with_calibrated_fixture(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            temp = Path(temporary)
            arm_config = json.loads((ROOT / "configs" / "arm_safety_config.json").read_text(encoding="utf-8"))
            arm_config["joints"]["5"]["gripper_open_pulse"] = 360
            arm_config_path = temp / "arm_safety.json"
            arm_config_path.write_text(json.dumps(arm_config), encoding="utf-8")

            config = valid_config()
            for grid_id in ("LEFT", "CENTER", "RIGHT"):
                sequence_path = temp / f"{grid_id.lower()}.json"
                sequence_path.write_text(json.dumps(valid_sequence(grid_id)), encoding="utf-8")
                config["sequence_paths"][grid_id] = str(sequence_path)
            config["workspace"] = {
                "hardware_policy": "REQUIRE_ESTIMATE_PASS",
                "allow_preverified_override": False,
            }
            config_path = temp / "clearance.json"
            config_path.write_text(json.dumps(config), encoding="utf-8")
            output_dir = temp / "output"
            completed = subprocess.run(
                [
                    sys.executable,
                    str(ROOT / "tools" / "run_real_k1_discrete_arm_clearance.py"),
                    "--config",
                    str(config_path),
                    "--arm-config",
                    str(arm_config_path),
                    "run",
                    "--grid",
                    "CENTER",
                    "--manual-grid-for-no-load",
                    "--output-dir",
                    str(output_dir),
                ],
                cwd=str(ROOT),
                capture_output=True,
                text=True,
                check=False,
            )
            result = json.loads((output_dir / "result.json").read_text(encoding="utf-8"))
        self.assertEqual(completed.returncode, 0, completed.stdout + completed.stderr)
        self.assertEqual(result["status"], "succeeded_dry_run")
        self.assertFalse(result["hardware_executed"])


if __name__ == "__main__":
    unittest.main()
