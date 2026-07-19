import math
import unittest

import numpy as np

from tools.risk_spatial_memory import (
    ObservationSnapshot,
    PoseSample,
    PoseSampleCache,
    RiskFusionTracker,
    Transform2D,
    project_risk_to_map,
)


class RiskSpatialMemoryTest(unittest.TestCase):
    def test_pose_cache_interpolates_capture_time_pose(self):
        cache = PoseSampleCache(duration_s=3.0)
        cache.append(PoseSample(10.0, 100, Transform2D(1.0, 0.0, 0.0), Transform2D(0.0, 0.0, 0.0), 0.1, 0.0, True))
        cache.append(PoseSample(10.2, 300, Transform2D(1.0, 0.0, 0.0), Transform2D(0.2, 0.0, math.pi / 2.0), 0.1, 0.0, True))

        snapshot = cache.snapshot_at(10.1, max_age_s=0.2)

        self.assertIsNotNone(snapshot)
        self.assertEqual("interpolated", snapshot.interpolation_mode)
        self.assertAlmostEqual(0.1, snapshot.odom_to_base.x)
        self.assertAlmostEqual(math.pi / 4.0, snapshot.odom_to_base.yaw)
        self.assertEqual(200, snapshot.capture_ros_time_ns)

    def test_projection_uses_full_capture_time_transform_chain(self):
        base_from_camera = np.eye(4, dtype=np.float64)
        snapshot = ObservationSnapshot(
            capture_monotonic_s=5.0,
            capture_ros_time_ns=123,
            map_to_odom=Transform2D(10.0, 0.0, 0.0),
            odom_to_base=Transform2D(1.0, 2.0, math.pi / 2.0),
            base_from_camera_optical=base_from_camera,
            linear_velocity_mps=0.0,
            angular_velocity_rps=0.0,
            pose_age_ms=5.0,
            pose_quality=0.9,
            interpolation_mode="interpolated",
        )
        depth = np.ones((20, 20), dtype=np.float32)
        camera_info = {"k": [10.0, 0.0, 10.0, 0.0, 10.0, 10.0, 0.0, 0.0, 1.0]}

        observation = project_risk_to_map(
            "corrosion", 0.8, (8, 8, 4, 4), depth, camera_info, snapshot, 0.2, 2.0
        )

        self.assertEqual("projected_capture_time_tf_chain", observation.projection_status)
        self.assertAlmostEqual(11.0, observation.map_point_xy[0], places=3)
        self.assertAlmostEqual(2.0, observation.map_point_xy[1], places=3)
        self.assertGreater(observation.position_quality, 0.7)

    def test_fusion_requires_two_nearby_valid_observations(self):
        tracker = RiskFusionTracker(merge_distance_m=0.25, merge_time_s=2.0)

        def observation(x, timestamp):
            return project_risk_to_map(
                "blockage",
                0.8,
                (8, 8, 4, 4),
                np.ones((20, 20), dtype=np.float32),
                {"k": [10.0, 0.0, 10.0, 0.0, 10.0, 10.0, 0.0, 0.0, 1.0]},
                ObservationSnapshot(
                    timestamp,
                    int(timestamp * 1e9),
                    Transform2D(x, 0.0, 0.0),
                    Transform2D(0.0, 0.0, 0.0),
                    np.eye(4),
                    0.0,
                    0.0,
                    0.0,
                    1.0,
                    "nearest",
                ),
                0.2,
                2.0,
            )

        first = tracker.update(observation(0.0, 1.0))
        second = tracker.update(observation(0.05, 2.0))
        repeat = tracker.update(observation(0.02, 10.0))

        self.assertFalse(first["confirmed"])
        self.assertTrue(second["confirmed"])
        self.assertTrue(second["newly_confirmed"])
        self.assertEqual(first["candidate_id"], second["candidate_id"])
        self.assertEqual(second["candidate_id"], repeat["candidate_id"])
        self.assertFalse(repeat["newly_confirmed"])


if __name__ == "__main__":
    unittest.main()
