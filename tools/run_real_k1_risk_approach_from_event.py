#!/usr/bin/env python3
import argparse
import json
import math
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String

from nav2_msgs.action import NavigateToPose


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def yaw_to_quat(yaw):
    half = yaw * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


def write_jsonl(path: Path, item):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a", encoding="utf-8") as handle:
        handle.write(json.dumps(item, ensure_ascii=False) + "\n")


class RealK1RiskApproach(Node):
    def __init__(self, args):
        super().__init__("real_k1_risk_approach_from_event")
        self.args = args
        self.output_dir = Path(args.output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.records_path = self.output_dir / "risk_approach_records.jsonl"
        self.latest_odom = None
        self.completed_ids = set()
        self.active = False
        self.event_count = 0
        self.started_at = time.monotonic()

        self.status_pub = self.create_publisher(String, args.status_topic, 10)
        self.create_subscription(Odometry, args.odom_topic, self.odom_cb, 20)
        self.create_subscription(String, args.event_topic, self.event_cb, 10)
        self.nav2_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.get_logger().info(
            f"risk approach ready: event={args.event_topic} stand_off={args.stand_off_m:.2f}m"
        )

    def odom_cb(self, msg):
        self.latest_odom = msg

    def publish_status(self, state, payload):
        out = dict(payload)
        out["state"] = state
        out["stamp_s"] = round(time.monotonic() - self.started_at, 3)
        msg = String()
        msg.data = json.dumps(out, ensure_ascii=False)
        self.status_pub.publish(msg)
        write_jsonl(self.records_path, out)

    def event_cb(self, msg):
        if self.active or self.event_count >= self.args.max_events:
            return
        try:
            event = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        event_id = str(event.get("event_id") or "")
        if not event_id or event_id in self.completed_ids:
            return
        if float(event.get("confidence") or 0.0) < self.args.min_confidence:
            return
        odom_xy = event.get("odom_point_xy_m") or {}
        if not isinstance(odom_xy, dict) or odom_xy.get("x") is None or odom_xy.get("y") is None:
            self.publish_status("skipped_no_odom_point", {"event": event})
            self.completed_ids.add(event_id)
            return
        if self.latest_odom is None:
            self.publish_status("skipped_no_odom", {"event": event})
            return

        risk_x = float(odom_xy["x"])
        risk_y = float(odom_xy["y"])
        pose = self.latest_odom.pose.pose
        robot_x = float(pose.position.x)
        robot_y = float(pose.position.y)
        dx = risk_x - robot_x
        dy = risk_y - robot_y
        distance = math.hypot(dx, dy)
        base_payload = {
            "event": event,
            "robot_odom_xy_m": {"x": round(robot_x, 4), "y": round(robot_y, 4)},
            "risk_odom_xy_m": {"x": round(risk_x, 4), "y": round(risk_y, 4)},
            "distance_to_risk_m": round(distance, 4),
        }
        if distance <= self.args.stand_off_m + self.args.already_near_margin_m:
            self.completed_ids.add(event_id)
            self.event_count += 1
            self.publish_status("already_near", base_payload)
            return

        ux = dx / max(distance, 1e-6)
        uy = dy / max(distance, 1e-6)
        goal_x = risk_x - ux * self.args.stand_off_m
        goal_y = risk_y - uy * self.args.stand_off_m
        goal_yaw = math.atan2(risk_y - goal_y, risk_x - goal_x)

        goal = PoseStamped()
        goal.header.frame_id = self.args.goal_frame
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = goal_x
        goal.pose.position.y = goal_y
        qx, qy, qz, qw = yaw_to_quat(goal_yaw)
        goal.pose.orientation.x = qx
        goal.pose.orientation.y = qy
        goal.pose.orientation.z = qz
        goal.pose.orientation.w = qw

        payload = dict(base_payload)
        payload["approach_goal"] = {
            "frame_id": self.args.goal_frame,
            "x": round(goal_x, 4),
            "y": round(goal_y, 4),
            "yaw": round(goal_yaw, 4),
        }
        if not self.nav2_client.wait_for_server(timeout_sec=self.args.nav2_wait_s):
            self.completed_ids.add(event_id)
            self.event_count += 1
            self.publish_status("nav2_unavailable", payload)
            return

        self.active = True
        self.completed_ids.add(event_id)
        self.event_count += 1
        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = goal
        self.publish_status("approach_goal_sent", payload)
        future = self.nav2_client.send_goal_async(nav_goal)
        future.add_done_callback(lambda fut: self.goal_response_cb(fut, payload))

    def goal_response_cb(self, future, payload):
        try:
            handle = future.result()
        except Exception as exc:
            self.active = False
            payload = dict(payload)
            payload["error"] = str(exc)
            self.publish_status("goal_send_error", payload)
            return
        if handle is None or not handle.accepted:
            self.active = False
            self.publish_status("goal_rejected", payload)
            return
        self.publish_status("goal_accepted", payload)
        result_future = handle.get_result_async()
        result_future.add_done_callback(lambda fut: self.result_cb(fut, payload))

    def result_cb(self, future, payload):
        self.active = False
        try:
            result = future.result()
            status = getattr(result, "status", None)
        except Exception as exc:
            payload = dict(payload)
            payload["error"] = str(exc)
            self.publish_status("result_error", payload)
            return
        payload = dict(payload)
        payload["nav_status"] = status
        self.publish_status("approach_complete", payload)


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--event-topic", default="/prelim_demo/risk_event")
    parser.add_argument("--status-topic", default="/risk/approach_status")
    parser.add_argument("--odom-topic", default="/odom")
    parser.add_argument("--goal-frame", default="odom")
    parser.add_argument("--stand-off-m", type=float, default=0.45)
    parser.add_argument("--already-near-margin-m", type=float, default=0.08)
    parser.add_argument("--min-confidence", type=float, default=0.60)
    parser.add_argument("--max-events", type=int, default=1)
    parser.add_argument("--nav2-wait-s", type=float, default=1.5)
    parser.add_argument("--output-dir", required=True)
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = RealK1RiskApproach(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
