import json
import tempfile
import unittest
from pathlib import Path

from tools.mission_state import MissionStateStore


class MissionStateStoreTest(unittest.TestCase):
    def test_candidates_are_bounded_and_keep_best_evidence(self):
        with tempfile.TemporaryDirectory() as temporary:
            store = MissionStateStore(Path(temporary) / "mission.json", max_candidates=2)
            store.update_candidate("a", {"confidence": 0.2, "best_evidence": {"rgb": "low"}})
            store.update_candidate("a", {"confidence": 0.8, "best_evidence": {"rgb": "high"}})
            store.update_candidate("a", {"confidence": 0.3, "best_evidence": {"rgb": "worse"}})
            self.assertEqual(0.8, store.candidates["a"]["confidence_max"])
            self.assertEqual("high", store.candidates["a"]["best_evidence"]["rgb"])
            store.update_candidate("b", {"confidence": 0.4})
            store.update_candidate("c", {"confidence": 0.5})

            self.assertEqual(2, len(store.candidates))
            self.assertNotIn("a", store.candidates)
            self.assertEqual(["b", "c"], list(store.candidates))

    def test_confirmed_risk_is_separate_and_snapshot_is_atomic_json(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "mission.json"
            store = MissionStateStore(path, max_candidates=2)
            store.update_candidate("a", {"event_id": "evt", "confidence": 0.8})
            store.confirm_risk(
                {
                    "event_id": "evt",
                    "candidate_key": "a",
                    "class_name": "blockage",
                    "confidence": 0.9,
                    "coordinate": {"frame_id": "odom", "xy_m": {"x": 1.0, "y": 2.0}},
                    "evidence_refs": ["usb.png"],
                }
            )
            self.assertTrue(store.write_if_dirty())
            value = json.loads(path.read_text(encoding="utf-8"))

            self.assertEqual(0, value["candidate_count"])
            self.assertEqual(1, value["confirmed_risk_count"])
            self.assertEqual("blockage", value["confirmed_risks"][0]["class_name"])
            self.assertEqual(
                {"visualization", "arm", "llm_report", "voice"},
                set(value["interfaces"]),
            )

    def test_confirmation_by_risk_id_removes_candidate_and_internal_clock(self):
        with tempfile.TemporaryDirectory() as temporary:
            store = MissionStateStore(Path(temporary) / "mission.json")
            store.update_candidate("spatial_1", {"confidence": 0.4})
            store.confirm_risk(
                {
                    "risk_id": "spatial_1",
                    "class_name": "corrosion",
                    "last_seen_monotonic": 123.0,
                }
            )

            snapshot = store.snapshot()
            self.assertEqual(0, snapshot["candidate_count"])
            self.assertEqual(1, snapshot["confirmed_risk_count"])
            self.assertNotIn("last_seen_monotonic", snapshot["confirmed_risks"][0])


if __name__ == "__main__":
    unittest.main()
