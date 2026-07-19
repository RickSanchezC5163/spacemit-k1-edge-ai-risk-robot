#!/usr/bin/env python3
"""Time-aligned risk projection and bounded multi-frame fusion."""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field
from typing import Any, Deque, Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np


def _wrap_angle(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


def _lerp_angle(first: float, second: float, alpha: float) -> float:
    return _wrap_angle(first + _wrap_angle(second - first) * alpha)


@dataclass(frozen=True)
class Transform2D:
    x: float
    y: float
    yaw: float

    def interpolate(self, other: "Transform2D", alpha: float) -> "Transform2D":
        alpha = max(0.0, min(1.0, float(alpha)))
        return Transform2D(
            x=self.x + (other.x - self.x) * alpha,
            y=self.y + (other.y - self.y) * alpha,
            yaw=_lerp_angle(self.yaw, other.yaw, alpha),
        )

    def transform_xyz(self, point: Sequence[float]) -> Tuple[float, float, float]:
        x, y, z = [float(value) for value in point[:3]]
        cos_yaw = math.cos(self.yaw)
        sin_yaw = math.sin(self.yaw)
        return (
            self.x + x * cos_yaw - y * sin_yaw,
            self.y + x * sin_yaw + y * cos_yaw,
            z,
        )

    def as_dict(self) -> Dict[str, float]:
        return {"x": self.x, "y": self.y, "yaw_rad": self.yaw}


# D435 optical: +x right, +y down, +z forward. Robot base: +x forward,
# +y left, +z up. Translation is the measured camera origin in base.
DEFAULT_BASE_FROM_CAMERA_OPTICAL = np.asarray(
    [
        [0.0, 0.0, 1.0, 0.105],
        [-1.0, 0.0, 0.0, 0.0],
        [0.0, -1.0, 0.0, 0.11],
        [0.0, 0.0, 0.0, 1.0],
    ],
    dtype=np.float64,
)


@dataclass(frozen=True)
class PoseSample:
    monotonic_s: float
    ros_time_ns: int
    map_to_odom: Optional[Transform2D]
    odom_to_base: Transform2D
    linear_velocity_mps: float
    angular_velocity_rps: float
    map_tf_valid: bool


@dataclass(frozen=True)
class ObservationSnapshot:
    capture_monotonic_s: float
    capture_ros_time_ns: int
    map_to_odom: Optional[Transform2D]
    odom_to_base: Transform2D
    base_from_camera_optical: np.ndarray
    linear_velocity_mps: float
    angular_velocity_rps: float
    pose_age_ms: float
    pose_quality: float
    interpolation_mode: str

    def as_dict(self) -> Dict[str, Any]:
        return {
            "capture_monotonic_s": self.capture_monotonic_s,
            "capture_ros_time_ns": self.capture_ros_time_ns,
            "map_to_odom": None if self.map_to_odom is None else self.map_to_odom.as_dict(),
            "odom_to_base": self.odom_to_base.as_dict(),
            "base_from_camera_optical": self.base_from_camera_optical.tolist(),
            "linear_velocity_mps": self.linear_velocity_mps,
            "angular_velocity_rps": self.angular_velocity_rps,
            "pose_age_ms": self.pose_age_ms,
            "pose_quality": self.pose_quality,
            "interpolation_mode": self.interpolation_mode,
        }


class PoseSampleCache:
    """Keep a short pose history and recover the pose at camera capture time."""

    def __init__(self, duration_s: float = 3.0, max_samples: int = 96) -> None:
        self.duration_s = max(0.5, float(duration_s))
        self.samples: Deque[PoseSample] = deque(maxlen=max(4, int(max_samples)))

    def append(self, sample: PoseSample) -> None:
        if self.samples and sample.monotonic_s < self.samples[-1].monotonic_s:
            return
        self.samples.append(sample)
        cutoff = sample.monotonic_s - self.duration_s
        while len(self.samples) > 1 and self.samples[0].monotonic_s < cutoff:
            self.samples.popleft()

    def snapshot_at(
        self,
        capture_monotonic_s: float,
        max_age_s: float = 0.20,
        base_from_camera_optical: np.ndarray = DEFAULT_BASE_FROM_CAMERA_OPTICAL,
    ) -> Optional[ObservationSnapshot]:
        if not self.samples:
            return None
        target = float(capture_monotonic_s)
        before: Optional[PoseSample] = None
        after: Optional[PoseSample] = None
        for sample in self.samples:
            if sample.monotonic_s <= target:
                before = sample
            if sample.monotonic_s >= target:
                after = sample
                break

        if before is not None and after is not None and before is not after:
            span = after.monotonic_s - before.monotonic_s
            alpha = 0.0 if span <= 0.0 else (target - before.monotonic_s) / span
            if before.map_to_odom is not None and after.map_to_odom is not None:
                map_to_odom = before.map_to_odom.interpolate(after.map_to_odom, alpha)
                map_valid = before.map_tf_valid and after.map_tf_valid
            else:
                nearest = before if alpha <= 0.5 else after
                map_to_odom = nearest.map_to_odom
                map_valid = nearest.map_tf_valid
            odom_to_base = before.odom_to_base.interpolate(after.odom_to_base, alpha)
            ros_time_ns = int(before.ros_time_ns + (after.ros_time_ns - before.ros_time_ns) * alpha)
            linear = before.linear_velocity_mps + (
                after.linear_velocity_mps - before.linear_velocity_mps
            ) * alpha
            angular = before.angular_velocity_rps + (
                after.angular_velocity_rps - before.angular_velocity_rps
            ) * alpha
            age_s = max(target - before.monotonic_s, after.monotonic_s - target)
            mode = "interpolated"
        else:
            nearest = before or after
            assert nearest is not None
            map_to_odom = nearest.map_to_odom
            map_valid = nearest.map_tf_valid
            odom_to_base = nearest.odom_to_base
            ros_time_ns = nearest.ros_time_ns
            linear = nearest.linear_velocity_mps
            angular = nearest.angular_velocity_rps
            age_s = abs(target - nearest.monotonic_s)
            mode = "nearest"

        max_age_s = max(0.001, float(max_age_s))
        if age_s > max_age_s or not map_valid or map_to_odom is None:
            return None
        quality = max(0.0, 1.0 - age_s / max_age_s)
        return ObservationSnapshot(
            capture_monotonic_s=target,
            capture_ros_time_ns=ros_time_ns,
            map_to_odom=map_to_odom,
            odom_to_base=odom_to_base,
            base_from_camera_optical=np.asarray(base_from_camera_optical, dtype=np.float64),
            linear_velocity_mps=linear,
            angular_velocity_rps=angular,
            pose_age_ms=age_s * 1000.0,
            pose_quality=quality,
            interpolation_mode=mode,
        )


@dataclass(frozen=True)
class RiskObservation:
    class_name: str
    confidence: float
    bbox_xywh: Tuple[float, float, float, float]
    capture_monotonic_s: float
    capture_ros_time_ns: int
    depth_status: str
    depth_m: Optional[float]
    depth_valid_ratio: float
    depth_dispersion_m: Optional[float]
    depth_quality: float
    camera_point_xyz: Optional[Tuple[float, float, float]]
    base_point_xyz: Optional[Tuple[float, float, float]]
    odom_point_xy: Optional[Tuple[float, float]]
    map_point_xy: Optional[Tuple[float, float]]
    pose_quality: float
    position_quality: float
    projection_status: str
    pose_snapshot: Optional[ObservationSnapshot]

    def as_dict(self) -> Dict[str, Any]:
        def xyz(value: Optional[Tuple[float, ...]], keys: Sequence[str]) -> Optional[Dict[str, float]]:
            if value is None:
                return None
            return {key: round(float(component), 4) for key, component in zip(keys, value)}

        return {
            "class_name": self.class_name,
            "confidence": round(self.confidence, 4),
            "bbox_xywh": list(self.bbox_xywh),
            "capture_monotonic_s": self.capture_monotonic_s,
            "capture_ros_time_ns": self.capture_ros_time_ns,
            "depth_status": self.depth_status,
            "depth_median_m": None if self.depth_m is None else round(self.depth_m, 4),
            "bbox_valid_depth_ratio": round(self.depth_valid_ratio, 4),
            "depth_dispersion_m": (
                None if self.depth_dispersion_m is None else round(self.depth_dispersion_m, 4)
            ),
            "depth_quality": round(self.depth_quality, 4),
            "camera_point_xyz_m": xyz(self.camera_point_xyz, ("x", "y", "z")),
            "base_point_xyz_m": xyz(self.base_point_xyz, ("x", "y", "z")),
            "odom_point_xy_m": xyz(self.odom_point_xy, ("x", "y")),
            "map_point_xy_m": xyz(self.map_point_xy, ("x", "y")),
            "pose_quality": round(self.pose_quality, 4),
            "position_quality": round(self.position_quality, 4),
            "projection_status": self.projection_status,
            "pose_snapshot": None if self.pose_snapshot is None else self.pose_snapshot.as_dict(),
        }


def _transform_matrix_point(matrix: np.ndarray, point: Sequence[float]) -> Tuple[float, float, float]:
    homogeneous = np.asarray([float(point[0]), float(point[1]), float(point[2]), 1.0])
    transformed = np.asarray(matrix, dtype=np.float64) @ homogeneous
    return float(transformed[0]), float(transformed[1]), float(transformed[2])


def project_risk_to_map(
    class_name: str,
    confidence: float,
    bbox_xywh: Sequence[float],
    aligned_depth_m: np.ndarray,
    camera_info: Dict[str, Any],
    observation_snapshot: Optional[ObservationSnapshot],
    min_depth_m: float,
    max_depth_m: float,
    central_fraction: float = 0.5,
) -> RiskObservation:
    """Project one detection using robust depth and the capture-time pose chain."""

    bbox = tuple(float(value) for value in bbox_xywh[:4])
    if len(bbox) != 4 or aligned_depth_m.ndim < 2:
        raise ValueError("bbox_xywh and aligned_depth_m are required")
    values = camera_info.get("k") or camera_info.get("K")
    if not isinstance(values, list) or len(values) < 9:
        return RiskObservation(
            class_name, confidence, bbox, 0.0, 0, "missing_intrinsics", None, 0.0,
            None, 0.0, None, None, None, None, 0.0, 0.0,
            "camera_intrinsics_unavailable", observation_snapshot,
        )
    fx, fy, cx, cy = float(values[0]), float(values[4]), float(values[2]), float(values[5])
    if fx == 0.0 or fy == 0.0:
        raise ValueError("camera intrinsics contain zero focal length")

    height, width = aligned_depth_m.shape[:2]
    x, y, box_w, box_h = bbox
    fraction = max(0.1, min(1.0, float(central_fraction)))
    center_u = x + box_w * 0.5
    center_v = y + box_h * 0.5
    roi_w = box_w * fraction
    roi_h = box_h * fraction
    x0 = max(0, min(width - 1, int(math.floor(center_u - roi_w * 0.5))))
    x1 = max(0, min(width, int(math.ceil(center_u + roi_w * 0.5))))
    y0 = max(0, min(height - 1, int(math.floor(center_v - roi_h * 0.5))))
    y1 = max(0, min(height, int(math.ceil(center_v + roi_h * 0.5))))
    roi = aligned_depth_m[y0:y1, x0:x1]
    valid_mask = np.isfinite(roi) & (roi >= float(min_depth_m)) & (roi <= float(max_depth_m))
    valid = roi[valid_mask]
    valid_ratio = 0.0 if roi.size == 0 else float(valid.size) / float(roi.size)
    capture_mono = 0.0 if observation_snapshot is None else observation_snapshot.capture_monotonic_s
    capture_ros = 0 if observation_snapshot is None else observation_snapshot.capture_ros_time_ns
    pose_quality = 0.0 if observation_snapshot is None else observation_snapshot.pose_quality
    if valid.size == 0:
        return RiskObservation(
            class_name, confidence, bbox, capture_mono, capture_ros, "invalid", None,
            valid_ratio, None, 0.0, None, None, None, None, pose_quality, 0.0,
            "depth_unavailable", observation_snapshot,
        )

    depth = float(np.median(valid))
    dispersion = float(np.median(np.abs(valid - depth)))
    ratio_quality = min(1.0, valid_ratio / 0.60)
    dispersion_quality = max(0.0, 1.0 - dispersion / max(0.03, depth * 0.10))
    depth_quality = ratio_quality * dispersion_quality
    camera_point = (
        (center_u - cx) * depth / fx,
        (center_v - cy) * depth / fy,
        depth,
    )
    if observation_snapshot is None or observation_snapshot.map_to_odom is None:
        return RiskObservation(
            class_name, confidence, bbox, capture_mono, capture_ros, "valid", depth,
            valid_ratio, dispersion, depth_quality, camera_point, None, None, None,
            pose_quality, 0.0, "capture_pose_unavailable", observation_snapshot,
        )

    base_point = _transform_matrix_point(
        observation_snapshot.base_from_camera_optical, camera_point
    )
    odom_point = observation_snapshot.odom_to_base.transform_xyz(base_point)
    map_point = observation_snapshot.map_to_odom.transform_xyz(odom_point)
    position_quality = max(
        0.0,
        min(1.0, float(confidence) * depth_quality * observation_snapshot.pose_quality),
    )
    return RiskObservation(
        class_name=class_name,
        confidence=float(confidence),
        bbox_xywh=bbox,
        capture_monotonic_s=capture_mono,
        capture_ros_time_ns=capture_ros,
        depth_status="valid",
        depth_m=depth,
        depth_valid_ratio=valid_ratio,
        depth_dispersion_m=dispersion,
        depth_quality=depth_quality,
        camera_point_xyz=camera_point,
        base_point_xyz=base_point,
        odom_point_xy=(odom_point[0], odom_point[1]),
        map_point_xy=(map_point[0], map_point[1]),
        pose_quality=observation_snapshot.pose_quality,
        position_quality=position_quality,
        projection_status="projected_capture_time_tf_chain",
        pose_snapshot=observation_snapshot,
    )


@dataclass
class _Candidate:
    candidate_id: str
    class_name: str
    observations: Deque[RiskObservation] = field(default_factory=lambda: deque(maxlen=3))
    confirmed: bool = False


class RiskFusionTracker:
    """Merge nearby observations and require two valid hits in the latest three."""

    def __init__(
        self,
        merge_distance_m: float = 0.25,
        merge_time_s: float = 2.0,
        required_observations: int = 2,
        observation_window: int = 3,
        max_candidates: int = 64,
    ) -> None:
        self.merge_distance_m = max(0.01, float(merge_distance_m))
        self.merge_time_s = max(0.1, float(merge_time_s))
        self.required_observations = max(2, int(required_observations))
        self.observation_window = max(self.required_observations, int(observation_window))
        self.max_candidates = max(1, int(max_candidates))
        self._candidates: List[_Candidate] = []
        self._next_id = 1

    @staticmethod
    def _distance(first: Sequence[float], second: Sequence[float]) -> float:
        return math.hypot(float(first[0]) - float(second[0]), float(first[1]) - float(second[1]))

    @staticmethod
    def _fused(candidate: _Candidate) -> Tuple[float, float, float]:
        valid = [obs for obs in candidate.observations if obs.map_point_xy is not None]
        weights = [max(1e-6, obs.confidence * obs.depth_quality * obs.pose_quality) for obs in valid]
        total = sum(weights)
        x = sum(obs.map_point_xy[0] * weight for obs, weight in zip(valid, weights)) / total
        y = sum(obs.map_point_xy[1] * weight for obs, weight in zip(valid, weights)) / total
        return x, y, total

    def _prune(self, now_s: float) -> None:
        self._candidates = [
            candidate
            for candidate in self._candidates
            if candidate.observations
            and (
                candidate.confirmed
                or now_s - candidate.observations[-1].capture_monotonic_s <= self.merge_time_s
            )
        ]
        if len(self._candidates) > self.max_candidates:
            self._candidates = self._candidates[-self.max_candidates :]

    def update(self, observation: RiskObservation) -> Dict[str, Any]:
        now_s = observation.capture_monotonic_s
        self._prune(now_s)
        if observation.map_point_xy is None:
            return {
                "state": "invalid_projection",
                "confirmed": False,
                "newly_confirmed": False,
                "candidate_id": None,
            }

        match: Optional[_Candidate] = None
        best_distance = float("inf")
        for candidate in self._candidates:
            if candidate.class_name != observation.class_name or not candidate.observations:
                continue
            fused_x, fused_y, _ = self._fused(candidate)
            distance = self._distance((fused_x, fused_y), observation.map_point_xy)
            if distance <= self.merge_distance_m and distance < best_distance:
                match = candidate
                best_distance = distance
        if match is None:
            match = _Candidate(
                candidate_id=f"spatial_candidate_{self._next_id:04d}",
                class_name=observation.class_name,
                observations=deque(maxlen=self.observation_window),
            )
            self._next_id += 1
            self._candidates.append(match)
        match.observations.append(observation)
        was_confirmed = match.confirmed
        match.confirmed = len(match.observations) >= self.required_observations
        fused_x, fused_y, weight_sum = self._fused(match)
        confidences = [item.confidence for item in match.observations]
        dispersions = [
            self._distance(item.map_point_xy, (fused_x, fused_y))
            for item in match.observations
            if item.map_point_xy is not None
        ]
        return {
            "state": "confirmed" if match.confirmed else "candidate",
            "candidate_id": match.candidate_id,
            "class_name": match.class_name,
            "confirmed": match.confirmed,
            "newly_confirmed": match.confirmed and not was_confirmed,
            "valid_observations": len(match.observations),
            "required_observations": self.required_observations,
            "observation_window": self.observation_window,
            "fused_map_point_xy_m": {"x": round(fused_x, 4), "y": round(fused_y, 4)},
            "position_uncertainty_m": round(max(dispersions, default=0.0), 4),
            "mean_confidence": round(sum(confidences) / len(confidences), 4),
            "confidence_max": round(max(confidences), 4),
            "fusion_weight_sum": round(weight_sum, 6),
        }
