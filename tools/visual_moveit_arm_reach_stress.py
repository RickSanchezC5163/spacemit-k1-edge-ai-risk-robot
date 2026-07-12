#!/usr/bin/env python3
"""Visual MoveIt reach stress test for the four-joint arm.

For each sampled target, the script computes a reachable Cartesian target for
the physical tip point on Link4, asks MoveIt to move that offset point into a
sphere around the target, replays the returned trajectory on /joint_states so
RViz shows the motion, then records final TCP error.
"""

from __future__ import annotations

import argparse
import json
import math
import random
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

import rclpy
from geometry_msgs.msg import Point, Pose
from moveit_msgs.msg import Constraints, JointConstraint
from moveit_msgs.srv import GetMotionPlan, GetPositionFK
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


JOINT_NAMES = ["j1", "j2", "j3", "j4"]
HOME = {
    "j1": 0.415948,
    "j2": -1.5708,
    "j3": 1.8,
    "j4": -0.636802,
}
LIMITS = {
    "j1": (-1.5708, 1.5708),
    "j2": (-1.5708, 1.2217),
    "j3": (-1.2217, 2.6180),
    "j4": (-1.5708, 1.5708),
}
GROUND_TASK_LIMITS = {
    "j1": (-1.25, 1.25),
    "j2": (-1.55, -1.05),
    "j3": (1.45, 1.95),
    "j4": (-0.95, -0.35),
}
PROJECT_ROOT = Path(__file__).resolve().parent.parent
TCP_LINK = "link4_tip_link"
TCP_OFFSET = (0.0, 0.0, 0.0)


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def stamp_to_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def sample_joint_states(seed: int, count: int, margin: float, profile: str) -> list[dict[str, float]]:
    rng = random.Random(seed)
    targets = [dict(HOME)]
    limits = GROUND_TASK_LIMITS if profile == "ground" else LIMITS
    for _ in range(max(0, count - 1)):
        target = {}
        for joint in JOINT_NAMES:
            low, high = limits[joint]
            span = high - low
            target[joint] = rng.uniform(low + span * margin, high - span * margin)
        targets.append(target)
    return targets


def rotate_vector_by_quaternion(vector: tuple[float, float, float], quat) -> tuple[float, float, float]:
    x, y, z = vector
    qx, qy, qz, qw = quat.x, quat.y, quat.z, quat.w
    # v' = v + 2*qw*(q_xyz x v) + 2*(q_xyz x (q_xyz x v))
    cx = qy * z - qz * y
    cy = qz * x - qx * z
    cz = qx * y - qy * x
    c2x = qy * cz - qz * cy
    c2y = qz * cx - qx * cz
    c2z = qx * cy - qy * cx
    return (
        x + 2.0 * (qw * cx + c2x),
        y + 2.0 * (qw * cy + c2y),
        z + 2.0 * (qw * cz + c2z),
    )


def pose_to_tcp_point(pose: Pose) -> Point:
    dx, dy, dz = rotate_vector_by_quaternion(TCP_OFFSET, pose.orientation)
    point = Point()
    point.x = pose.position.x + dx
    point.y = pose.position.y + dy
    point.z = pose.position.z + dz
    return point


def point_to_dict(point: Point | None) -> dict[str, float] | None:
    if point is None:
        return None
    return {"x": float(point.x), "y": float(point.y), "z": float(point.z)}


def point_distance(a: Point, b: Point) -> float:
    return math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) ** 2)


class VisualReachStress(Node):
    def __init__(self, args: argparse.Namespace):
        super().__init__("visual_moveit_arm_reach_stress")
        self.args = args
        self.client = self.create_client(GetMotionPlan, "/plan_kinematic_path")
        self.fk_client = self.create_client(GetPositionFK, "/compute_fk")
        self.joint_pub = self.create_publisher(JointState, "/joint_states", 20)
        self.observed = dict(HOME)
        self.joint_sub = self.create_subscription(
            JointState, "/joint_states", self.on_joint_state, 20
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        marker_qos = QoSProfile(depth=1)
        marker_qos.reliability = ReliabilityPolicy.RELIABLE
        marker_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.marker_pub = self.create_publisher(
            MarkerArray, "/visual_reach_stress_markers_world", marker_qos
        )
        self.planning_frame = "base_link"
        self.marker_frame = "world"
        self.clear_markers()

    def on_joint_state(self, msg: JointState) -> None:
        for name, position in zip(msg.name, msg.position):
            if name in self.observed:
                self.observed[name] = float(position)

    def wait_ready(self) -> bool:
        return self.client.wait_for_service(timeout_sec=self.args.wait_service_s) and self.fk_client.wait_for_service(timeout_sec=self.args.wait_service_s)

    def publish_state(self, positions: dict[str, float]) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(JOINT_NAMES)
        msg.position = [float(positions[name]) for name in JOINT_NAMES]
        for name in JOINT_NAMES:
            self.observed[name] = float(positions[name])
        self.joint_pub.publish(msg)

    def hold_state(self, positions: dict[str, float], seconds: float) -> None:
        deadline = time.monotonic() + max(0.0, seconds)
        while time.monotonic() < deadline:
            self.publish_state(positions)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(1.0 / self.args.publish_hz)

    def interpolate_state(
        self, start: dict[str, float], target: dict[str, float], seconds: float
    ) -> dict[str, float]:
        steps = max(2, int(max(0.0, seconds) * self.args.publish_hz))
        current = dict(start)
        for step in range(1, steps + 1):
            ratio = step / steps
            current = {
                name: float(start[name]) + (float(target[name]) - float(start[name])) * ratio
                for name in JOINT_NAMES
            }
            self.publish_state(current)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(1.0 / self.args.publish_hz)
        return current

    def lookup_tcp_point(self) -> Point | None:
        try:
            tf = self.tf_buffer.lookup_transform(
                self.planning_frame,
                TCP_LINK,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.2),
            )
        except TransformException as exc:
            self.get_logger().warn(f"cannot lookup {self.planning_frame} -> {TCP_LINK}: {exc}")
            return None
        pose = Pose()
        pose.position.x = tf.transform.translation.x
        pose.position.y = tf.transform.translation.y
        pose.position.z = tf.transform.translation.z
        pose.orientation = tf.transform.rotation
        return pose_to_tcp_point(pose)

    def clear_markers(self) -> None:
        marker = Marker()
        marker.action = Marker.DELETEALL
        self.marker_pub.publish(MarkerArray(markers=[marker]))

    def publish_start_end_markers(
        self, index: int, start: Point | None, target: Point | None, end: Point | None
    ) -> None:
        if start is None or target is None:
            return
        now = self.get_clock().now().to_msg()
        clear = Marker()
        clear.action = Marker.DELETEALL

        def sphere(ns: str, marker_id: int, point: Point, rgba: tuple[float, float, float, float]) -> Marker:
            marker = Marker()
            marker.header.frame_id = self.marker_frame
            marker.header.stamp = now
            marker.ns = ns
            marker.id = marker_id
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position = point
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.035
            marker.scale.y = 0.035
            marker.scale.z = 0.035
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = rgba
            return marker

        def label(ns: str, marker_id: int, point: Point, text: str) -> Marker:
            marker = Marker()
            marker.header.frame_id = self.marker_frame
            marker.header.stamp = now
            marker.ns = ns
            marker.id = marker_id
            marker.type = Marker.TEXT_VIEW_FACING
            marker.action = Marker.ADD
            marker.pose.position.x = point.x
            marker.pose.position.y = point.y
            marker.pose.position.z = point.z + 0.085
            marker.pose.orientation.w = 1.0
            marker.scale.z = 0.035
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 1.0
            marker.color.a = 1.0
            marker.text = text
            return marker

        line = Marker()
        line.header.frame_id = self.marker_frame
        line.header.stamp = now
        line.ns = "reach_start_to_end"
        line.id = index
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        if end is not None:
            line.points = [start, end]
        line.scale.x = 0.010
        line.color.r = 1.0
        line.color.g = 1.0
        line.color.b = 0.0
        line.color.a = 1.0

        self.marker_pub.publish(
            MarkerArray(
                markers=[
                    clear,
                    sphere("reach_start_world_big", index, start, (0.0, 1.0, 0.0, 1.0)),
                    label("reach_start_world_label", index, start, "START"),
                    sphere("reach_target_world_big", index, target, (0.0, 0.35, 1.0, 1.0)),
                    label("reach_target_world_label", index, target, "TARGET"),
                    *(
                        [
                            sphere("reach_end_world_big", index, end, (1.0, 0.0, 0.0, 1.0)),
                            label("reach_end_world_label", index, end, "END"),
                            line,
                        ]
                        if end is not None
                        else []
                    ),
                ]
            )
        )

    def compute_tcp_target(self, joints: dict[str, float]) -> Point | None:
        request = GetPositionFK.Request()
        request.header.frame_id = self.planning_frame
        request.fk_link_names = [TCP_LINK]
        request.robot_state.joint_state.name = list(JOINT_NAMES)
        request.robot_state.joint_state.position = [joints[name] for name in JOINT_NAMES]
        future = self.fk_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self.args.call_timeout_s)
        if not future.done():
            return None
        response = future.result()
        if not response.pose_stamped:
            return None
        return pose_to_tcp_point(response.pose_stamped[0].pose)

    def plan(self, start: dict[str, float], target_joints: dict[str, float]) -> dict:
        request = GetMotionPlan.Request()
        motion_request = request.motion_plan_request
        motion_request.group_name = self.args.group
        motion_request.num_planning_attempts = self.args.attempts
        motion_request.allowed_planning_time = self.args.allowed_planning_time_s
        motion_request.planner_id = self.args.planner_id

        motion_request.start_state.joint_state.name = list(JOINT_NAMES)
        motion_request.start_state.joint_state.position = [start[name] for name in JOINT_NAMES]

        constraints = Constraints()
        constraints.name = "visual_reach_tcp_via_joint_target"
        for joint in JOINT_NAMES:
            joint_constraint = JointConstraint()
            joint_constraint.joint_name = joint
            joint_constraint.position = float(target_joints[joint])
            joint_constraint.tolerance_above = self.args.joint_tolerance_rad
            joint_constraint.tolerance_below = self.args.joint_tolerance_rad
            joint_constraint.weight = 1.0
            constraints.joint_constraints.append(joint_constraint)
        motion_request.goal_constraints = [constraints]

        started = time.monotonic()
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self.args.call_timeout_s)
        elapsed = time.monotonic() - started
        if not future.done():
            return {"ok": False, "error_code": "client_timeout", "elapsed_s": elapsed}

        response = future.result()
        result = response.motion_plan_response
        points = result.trajectory.joint_trajectory.points
        return {
            "ok": int(result.error_code.val) == 1 and len(points) > 0,
            "error_code": int(result.error_code.val),
            "elapsed_s": elapsed,
            "planning_time_s": float(result.planning_time or 0.0),
            "trajectory": result.trajectory.joint_trajectory,
        }

    def replay(self, trajectory, fallback_target: dict[str, float]) -> dict[str, float]:
        if not trajectory.points:
            self.hold_state(fallback_target, self.args.dwell_s)
            return dict(fallback_target)

        names = list(trajectory.joint_names)
        previous_time = 0.0
        final = dict(self.observed)
        for point in trajectory.points:
            point_time = stamp_to_seconds(point.time_from_start)
            dt = max(self.args.min_point_dt_s, (point_time - previous_time) * self.args.playback_scale)
            previous_time = point_time
            positions = dict(final)
            for name, position in zip(names, point.positions):
                if name in positions:
                    positions[name] = float(position)
            self.hold_state(positions, dt)
            final = positions
        self.hold_state(final, self.args.dwell_s)
        return final


def write_report(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def write_markdown(path: Path, payload: dict) -> None:
    summary = payload["summary"]
    lines = [
        "# Mechanical Arm Visual Reach Stress Test",
        "",
        f"Generated: `{payload['generated_at']}`",
        "",
        "## Summary",
        "",
        f"- requested: `{summary['requested_count']}`",
        f"- planned: `{summary['planned_count']}`",
        f"- reached: `{summary['reached_count']}`",
        f"- failed_plan: `{summary['failed_plan_count']}`",
        f"- failed_reach: `{summary['failed_reach_count']}`",
        f"- max_tcp_error_m: `{summary['max_tcp_error_m']:.6f}`",
        f"- mean_tcp_error_m: `{summary['mean_tcp_error_m']:.6f}`",
        "",
        "## Notes",
        "",
        "- This is RViz/fake-state visual execution, not hardware execution.",
        "- The script publishes `/joint_states` directly after each MoveIt plan.",
    ]
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--count", type=int, default=1000)
    parser.add_argument("--seed", type=int, default=20260712)
    parser.add_argument("--output-dir", default=str(PROJECT_ROOT / "outputs" / "moveit_arm_visual_reach_stress_1000"))
    parser.add_argument("--group", default="arm")
    parser.add_argument("--planner-id", default="RRTConnectkConfigDefault")
    parser.add_argument("--attempts", type=int, default=1)
    parser.add_argument("--allowed-planning-time-s", type=float, default=0.6)
    parser.add_argument("--call-timeout-s", type=float, default=3.0)
    parser.add_argument("--wait-service-s", type=float, default=10.0)
    parser.add_argument("--joint-tolerance-rad", type=float, default=0.005)
    parser.add_argument("--reach-tolerance-m", type=float, default=0.035)
    parser.add_argument("--limit-margin-ratio", type=float, default=0.05)
    parser.add_argument("--profile", choices=["ground", "full"], default="ground")
    parser.add_argument("--sample-multiplier", type=int, default=40)
    parser.add_argument("--tcp-z-min", type=float, default=-0.05)
    parser.add_argument("--tcp-z-max", type=float, default=0.18)
    parser.add_argument("--publish-hz", type=float, default=30.0)
    parser.add_argument("--playback-scale", type=float, default=0.12)
    parser.add_argument("--min-point-dt-s", type=float, default=0.004)
    parser.add_argument("--dwell-s", type=float, default=0.015)
    parser.add_argument("--final-hold-s", type=float, default=0.08)
    parser.add_argument("--settle-s", type=float, default=0.0)
    parser.add_argument("--checkpoint-every", type=int, default=25)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    candidate_count = max(args.count, args.count * max(1, args.sample_multiplier))
    samples = sample_joint_states(args.seed, candidate_count, args.limit_margin_ratio, args.profile)
    output_dir = Path(args.output_dir)
    json_path = output_dir / "visual_reach_stress.json"
    md_path = output_dir / "visual_reach_stress.md"

    rclpy.init()
    node = VisualReachStress(args)
    results = []
    current = dict(HOME)
    try:
        if not node.wait_ready():
            print("/plan_kinematic_path or /compute_fk not available", file=sys.stderr)
            return 3
        node.hold_state(current, 0.5)
        skipped_by_height = 0
        for candidate_index, sample_joints in enumerate(samples, start=1):
            if len(results) >= args.count:
                break
            target_tcp = node.compute_tcp_target(sample_joints)
            if target_tcp is not None and not (args.tcp_z_min <= target_tcp.z <= args.tcp_z_max):
                skipped_by_height += 1
                continue
            index = len(results) + 1
            record = {
                "index": index,
                "candidate_index": candidate_index,
                "tcp_offset_link4": {
                    "x": TCP_OFFSET[0],
                    "y": TCP_OFFSET[1],
                    "z": TCP_OFFSET[2],
                },
                "sample_joints_for_fk": sample_joints,
                "target_tcp": point_to_dict(target_tcp),
                "planned": False,
                "error_code": "fk_failed",
                "plan_elapsed_s": 0.0,
                "planning_time_s": 0.0,
            }
            if target_tcp is None:
                record.update(
                    {
                        "trajectory_points": 0,
                        "final_commanded": dict(current),
                        "observed": dict(node.observed),
                        "final_tcp": None,
                        "tcp_error_m": None,
                        "reached": False,
                    }
                )
                results.append(record)
                continue

            plan = node.plan(current, sample_joints)
            record.update(
                {
                    "planned": bool(plan["ok"]),
                    "error_code": plan["error_code"],
                    "plan_elapsed_s": round(float(plan["elapsed_s"]), 4),
                    "planning_time_s": round(float(plan.get("planning_time_s", 0.0)), 4),
                }
            )
            if plan["ok"]:
                start_point = node.lookup_tcp_point()
                node.publish_start_end_markers(index, start_point, target_tcp, None)
                node.replay(plan["trajectory"], current)
                final_joints = dict(node.observed)
                if args.settle_s > 0.0:
                    final_joints = node.interpolate_state(final_joints, sample_joints, args.settle_s)
                node.hold_state(final_joints, args.final_hold_s)
                for _ in range(5):
                    rclpy.spin_once(node, timeout_sec=0.01)
                end_point = node.lookup_tcp_point()
                node.publish_start_end_markers(index, start_point, target_tcp, end_point)
                observed = dict(node.observed)
                tcp_error = point_distance(end_point, target_tcp) if end_point is not None else math.inf
                record.update(
                    {
                        "trajectory_points": len(plan["trajectory"].points),
                        "settle_to_sample_joints": args.settle_s > 0.0,
                        "final_commanded": final_joints,
                        "observed": observed,
                        "final_tcp": point_to_dict(end_point),
                        "tcp_error_m": tcp_error,
                        "reached": tcp_error <= args.reach_tolerance_m,
                    }
                )
                current = observed
            else:
                record.update(
                    {
                        "trajectory_points": 0,
                        "final_commanded": dict(current),
                        "observed": dict(node.observed),
                        "final_tcp": point_to_dict(node.lookup_tcp_point()),
                        "tcp_error_m": None,
                        "reached": False,
                    }
                )
                node.hold_state(current, args.dwell_s)

            results.append(record)
            status = "REACH" if record["reached"] else ("PLAN_FAIL" if not record["planned"] else "REACH_FAIL")
            print(
                f"[{index:04d}/{args.count:04d}] {status} "
                f"points={record['trajectory_points']} "
                f"plan={record['planning_time_s']:.3f}s "
                f"tcp_err={record['tcp_error_m']}",
                flush=True,
            )

            if index % args.checkpoint_every == 0:
                payload = build_payload(args, results)
                write_report(json_path, payload)
                write_markdown(md_path, payload)
    finally:
        node.hold_state(current, 0.2)
        node.destroy_node()
        rclpy.shutdown()

    payload = build_payload(args, results)
    payload["summary"]["skipped_by_height"] = skipped_by_height if "skipped_by_height" in locals() else 0
    write_report(json_path, payload)
    write_markdown(md_path, payload)
    print(f"wrote {json_path}")
    print(f"wrote {md_path}")
    print(json.dumps(payload["summary"], ensure_ascii=False))
    return 0 if payload["summary"]["failed_plan_count"] == 0 and payload["summary"]["failed_reach_count"] == 0 else 1


def build_payload(args: argparse.Namespace, results: list[dict]) -> dict:
    planned_count = sum(1 for item in results if item["planned"])
    reached_count = sum(1 for item in results if item["reached"])
    errors = [
        float(item["tcp_error_m"])
        for item in results
        if item.get("tcp_error_m") is not None
    ]
    return {
        "schema_version": "moveit_arm_visual_reach_stress_v1",
        "generated_at": datetime.now(timezone.utc).isoformat(timespec="seconds"),
        "config": vars(args),
        "summary": {
            "requested_count": int(args.count),
            "completed_count": len(results),
            "planned_count": planned_count,
            "reached_count": reached_count,
            "failed_plan_count": len(results) - planned_count,
            "failed_reach_count": planned_count - reached_count,
            "max_tcp_error_m": max(errors) if errors else math.nan,
            "mean_tcp_error_m": sum(errors) / len(errors) if errors else math.nan,
        },
        "results": results,
    }


if __name__ == "__main__":
    raise SystemExit(main())
