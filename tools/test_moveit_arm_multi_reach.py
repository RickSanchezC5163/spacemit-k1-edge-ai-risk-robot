#!/usr/bin/env python3
"""
Batch-test MoveIt reachability for mechanical_arm_1.

The script calls /plan_kinematic_path for many joint-space targets and writes
a JSON/Markdown report. It plans only j1-j4. The simulated gripper state from
the pick-place plan is recorded as metadata and is not passed to MoveIt.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

import rclpy
from moveit_msgs.msg import Constraints, JointConstraint
from moveit_msgs.srv import GetMotionPlan
from rclpy.node import Node

PROJECT_ROOT = Path(__file__).resolve().parent.parent


def load_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def pulse_to_angle_rad(pulse: int, angle_range_deg: list[float]) -> float:
    lo_deg, hi_deg = angle_range_deg
    t = pulse / 1000.0
    return math.radians(lo_deg + t * (hi_deg - lo_deg))


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def arm_pulses_to_joint_radians(pulses: dict[str, int], config: dict) -> dict[str, float]:
    # Current URDF temporary limits. Keep targets inside the model used by MoveIt.
    urdf_limits = {
        "j1": (-1.5708, 1.5708),
        "j2": (-1.2217, 1.2217),
        "j3": (-1.2217, 1.2217),
        "j4": (-1.5708, 1.5708),
    }
    result = {}
    for servo_id in (1, 2, 3, 4):
        joint_name = f"j{servo_id}"
        joint_cfg = config["joints"][str(servo_id)]
        rad = pulse_to_angle_rad(int(pulses[str(servo_id)]), joint_cfg["angle_range_deg"])
        lo, hi = urdf_limits[joint_name]
        result[joint_name] = clamp(rad, lo + 1e-4, hi - 1e-4)
    return result


def build_targets(plan: dict, config: dict, max_targets: int) -> list[dict]:
    targets: list[dict] = []
    seen = set()

    for waypoint in plan.get("waypoints", []):
        pulses = waypoint["servo_pulse_targets"]
        key = tuple((str(i), int(pulses[str(i)])) for i in range(1, 5))
        if key in seen:
            continue
        seen.add(key)
        targets.append(
            {
                "source": "planned_waypoint",
                "name": waypoint["name"],
                "task_phase": waypoint["task_phase"],
                "servo_pulse_targets": {str(i): int(pulses[str(i)]) for i in range(1, 5)},
                "simulated_gripper": waypoint["simulated_gripper"],
                "joint_targets_rad": arm_pulses_to_joint_radians(pulses, config),
            }
        )

    # Extra conservative sampled positions around reach/lift/place families.
    yaw_values = [390, 450, 510, 590, 650]
    shoulder_values = [430, 520, 610, 700]
    elbow_values = [470, 560, 650]
    wrist_values = [465, 488]
    for yaw in yaw_values:
        for shoulder in shoulder_values:
            for elbow in elbow_values:
                for wrist in wrist_values:
                    pulses = {"1": yaw, "2": shoulder, "3": elbow, "4": wrist}
                    key = tuple((str(i), int(pulses[str(i)])) for i in range(1, 5))
                    if key in seen:
                        continue
                    seen.add(key)
                    targets.append(
                        {
                            "source": "sampled_grid",
                            "name": f"sample_y{yaw}_s{shoulder}_e{elbow}_w{wrist}",
                            "task_phase": "reachability_sample",
                            "servo_pulse_targets": pulses,
                            "simulated_gripper": {
                                "state": "unchanged",
                                "event": "none",
                                "mode": "not_part_of_moveit_plan",
                            },
                            "joint_targets_rad": arm_pulses_to_joint_radians(pulses, config),
                        }
                    )
                    if len(targets) >= max_targets:
                        return targets
    return targets[:max_targets]


class ReachabilityTester(Node):
    def __init__(self, args):
        super().__init__("mechanical_arm_1_moveit_reachability_tester")
        self.args = args
        self.client = self.create_client(GetMotionPlan, "/plan_kinematic_path")

    def wait_ready(self) -> bool:
        return self.client.wait_for_service(timeout_sec=self.args.wait_service_s)

    def plan_to_target(self, start_rad: dict[str, float], target: dict) -> dict:
        request = GetMotionPlan.Request()
        motion_request = request.motion_plan_request
        motion_request.group_name = self.args.group
        motion_request.num_planning_attempts = self.args.attempts
        motion_request.allowed_planning_time = self.args.allowed_planning_time_s
        motion_request.planner_id = self.args.planner_id

        joint_names = ["j1", "j2", "j3", "j4"]
        motion_request.start_state.joint_state.name = joint_names
        motion_request.start_state.joint_state.position = [start_rad[name] for name in joint_names]

        constraints = Constraints()
        constraints.name = target["name"]
        for joint_name in joint_names:
            joint_constraint = JointConstraint()
            joint_constraint.joint_name = joint_name
            joint_constraint.position = float(target["joint_targets_rad"][joint_name])
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
            return {
                **target,
                "success": False,
                "error_code": "client_timeout",
                "planning_time_s": round(elapsed, 4),
                "trajectory_points": 0,
            }
        response = future.result()
        result = response.motion_plan_response
        error_value = int(result.error_code.val)
        points = len(result.trajectory.joint_trajectory.points)
        return {
            **target,
            "success": error_value == 1 and points > 0,
            "error_code": error_value,
            "planning_time_s": round(float(result.planning_time or elapsed), 4),
            "client_elapsed_s": round(elapsed, 4),
            "trajectory_points": points,
        }


def write_markdown(report: dict, path: Path) -> None:
    lines = [
        "# Mechanical Arm MoveIt Multi-Reach Test",
        "",
        f"Generated: `{report['generated_at']}`",
        "",
        "## Summary",
        "",
        f"- targets: `{report['summary']['target_count']}`",
        f"- success: `{report['summary']['success_count']}`",
        f"- failed: `{report['summary']['failure_count']}`",
        f"- success_rate: `{report['summary']['success_rate']:.1%}`",
        "",
        "## Results",
        "",
        "| # | Source | Target | Success | Points | Time(s) | Gripper Event |",
        "|---|---|---|---|---:|---:|---|",
    ]
    for index, result in enumerate(report["results"]):
        gripper = result["simulated_gripper"]
        lines.append(
            f"| {index} | {result['source']} | `{result['name']}` | "
            f"{'PASS' if result['success'] else 'FAIL'} | "
            f"{result['trajectory_points']} | {result['planning_time_s']} | "
            f"{gripper.get('event')} -> {gripper.get('state')} |"
        )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--plan", default=str(PROJECT_ROOT / "outputs" / "moveit_gripper_sim_pick_place_plan_v1" / "moveit_gripper_sim_pick_place_plan.json"))
    parser.add_argument("--config", default=str(PROJECT_ROOT / "configs" / "arm_safety_config.json"))
    parser.add_argument("--output-dir", default=str(PROJECT_ROOT / "outputs" / "moveit_arm_multi_reach_test_v1"))
    parser.add_argument("--group", default="arm")
    parser.add_argument("--planner-id", default="RRTConnectkConfigDefault")
    parser.add_argument("--max-targets", type=int, default=64)
    parser.add_argument("--attempts", type=int, default=5)
    parser.add_argument("--allowed-planning-time-s", type=float, default=2.0)
    parser.add_argument("--call-timeout-s", type=float, default=6.0)
    parser.add_argument("--wait-service-s", type=float, default=10.0)
    parser.add_argument("--joint-tolerance-rad", type=float, default=0.03)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    plan = load_json(Path(args.plan))
    config = load_json(Path(args.config))
    targets = build_targets(plan, config, args.max_targets)
    if not targets:
        print("no targets", file=sys.stderr)
        return 2

    start_rad = targets[0]["joint_targets_rad"]
    rclpy.init()
    node = ReachabilityTester(args)
    try:
        if not node.wait_ready():
            print("/plan_kinematic_path not available", file=sys.stderr)
            return 3
        results = []
        for index, target in enumerate(targets, start=1):
            result = node.plan_to_target(start_rad, target)
            results.append(result)
            status = "PASS" if result["success"] else "FAIL"
            print(
                f"[{index:03d}/{len(targets):03d}] {status} "
                f"{target['name']} points={result['trajectory_points']} "
                f"time={result['planning_time_s']}",
                flush=True,
            )
    finally:
        node.destroy_node()
        rclpy.shutdown()

    success_count = sum(1 for result in results if result["success"])
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    report = {
        "schema_version": "moveit_arm_multi_reach_test_v1",
        "generated_at": datetime.now(timezone.utc).isoformat(timespec="seconds"),
        "boundary": {
            "plans_arm_joints_only": True,
            "simulated_gripper_not_sent_to_moveit": True,
            "hardware_executed": False,
            "cmd_vel_published": False,
        },
        "summary": {
            "target_count": len(results),
            "success_count": success_count,
            "failure_count": len(results) - success_count,
            "success_rate": success_count / len(results),
        },
        "results": results,
    }
    json_path = output_dir / "moveit_arm_multi_reach_test.json"
    md_path = output_dir / "moveit_arm_multi_reach_test.md"
    json_path.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    write_markdown(report, md_path)
    print(f"wrote {json_path}")
    print(f"wrote {md_path}")
    print(json.dumps(report["summary"], ensure_ascii=False))
    return 0 if success_count == len(results) else 1


if __name__ == "__main__":
    raise SystemExit(main())
