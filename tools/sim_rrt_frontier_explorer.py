#!/usr/bin/env python3
import argparse
import json
import math
import random
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

try:
    from nav2_msgs.action import NavigateToPose
    from rclpy.action import ActionClient
except ImportError:
    NavigateToPose = None
    ActionClient = None


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def yaw_to_quat(yaw):
    half = yaw * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


def quantize_angle(angle, step_rad):
    if step_rad <= 1e-6:
        return angle
    return round(angle / step_rad) * step_rad


def clamp(value, lo, hi):
    return max(lo, min(hi, value))


class RRTFrontierExplorer(Node):
    def __init__(self, args):
        super().__init__("sim_rrt_frontier_explorer")
        self.args = args
        self.map_msg = None
        self.risk_detected = False
        self.records = []
        self.goal_count = 0
        self.random = random.Random(args.seed)
        self.goal_pub = self.create_publisher(PoseStamped, args.goal_topic, 10)
        self.create_subscription(OccupancyGrid, args.map_topic, self.map_cb, 10)
        if args.stop_on_risk_topic:
            self.create_subscription(String, args.stop_on_risk_topic, self.risk_cb, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.nav2_client = None
        if args.send_nav2_action and NavigateToPose is not None and ActionClient is not None:
            self.nav2_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

    def map_cb(self, msg):
        self.map_msg = msg

    def risk_cb(self, msg):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if int(payload.get("detection_count") or 0) > 0:
            self.risk_detected = True

    def robot_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.args.map_frame,
                self.args.base_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05),
            )
        except TransformException:
            return None
        t = tf.transform.translation
        return float(t.x), float(t.y), yaw_from_quat(tf.transform.rotation)

    def wait_ready(self):
        deadline = time.monotonic() + self.args.wait_ready_s
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.map_msg is not None and self.robot_pose() is not None:
                if self.nav2_client is None:
                    return True
                if self.nav2_client.wait_for_server(timeout_sec=0.2):
                    return True
        return False

    def world_to_cell(self, x, y):
        info = self.map_msg.info
        mx = int((x - info.origin.position.x) / info.resolution)
        my = int((y - info.origin.position.y) / info.resolution)
        if 0 <= mx < info.width and 0 <= my < info.height:
            return mx, my
        return None

    def cell_to_world(self, cell):
        info = self.map_msg.info
        return (
            info.origin.position.x + (cell[0] + 0.5) * info.resolution,
            info.origin.position.y + (cell[1] + 0.5) * info.resolution,
        )

    def idx(self, cell):
        return cell[1] * self.map_msg.info.width + cell[0]

    def value(self, cell):
        return self.map_msg.data[self.idx(cell)]

    def is_free(self, cell):
        return 0 <= self.value(cell) <= self.args.free_threshold

    def is_unknown(self, cell):
        return self.value(cell) < 0

    def neighbors8(self, cell):
        x, y = cell
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                if dx == 0 and dy == 0:
                    continue
                nb = (x + dx, y + dy)
                if 0 <= nb[0] < self.map_msg.info.width and 0 <= nb[1] < self.map_msg.info.height:
                    yield nb

    def near_obstacle(self, cell, radius_cells):
        cx, cy = cell
        for y in range(max(0, cy - radius_cells), min(self.map_msg.info.height, cy + radius_cells + 1)):
            for x in range(max(0, cx - radius_cells), min(self.map_msg.info.width, cx + radius_cells + 1)):
                if self.map_msg.data[y * self.map_msg.info.width + x] >= self.args.occupied_threshold:
                    return True
        return False

    def is_frontier(self, cell):
        if not self.is_free(cell):
            return False
        if self.near_obstacle(cell, self.inflation_cells()):
            return False
        return any(self.is_unknown(nb) for nb in self.neighbors8(cell))

    def inflation_cells(self):
        return max(1, int(self.args.inflation_m / self.map_msg.info.resolution))

    def sample_free_cell(self, start):
        radius_cells = int(self.args.sample_radius_m / self.map_msg.info.resolution)
        for _ in range(100):
            x = self.random.randint(max(0, start[0] - radius_cells), min(self.map_msg.info.width - 1, start[0] + radius_cells))
            y = self.random.randint(max(0, start[1] - radius_cells), min(self.map_msg.info.height - 1, start[1] + radius_cells))
            cell = (x, y)
            if self.is_free(cell) and not self.near_obstacle(cell, self.inflation_cells()):
                return cell
        return None

    def steer(self, from_cell, to_cell):
        dx = to_cell[0] - from_cell[0]
        dy = to_cell[1] - from_cell[1]
        dist = math.hypot(dx, dy)
        if dist < 1e-6:
            return from_cell
        step = min(self.args.rrt_step_cells, dist)
        return (
            int(round(from_cell[0] + dx / dist * step)),
            int(round(from_cell[1] + dy / dist * step)),
        )

    def segment_free(self, a, b):
        dx = b[0] - a[0]
        dy = b[1] - a[1]
        steps = max(abs(dx), abs(dy), 1)
        for i in range(1, steps + 1):
            cell = (int(round(a[0] + dx * i / steps)), int(round(a[1] + dy * i / steps)))
            if not self.is_free(cell) or self.near_obstacle(cell, self.inflation_cells()):
                return False
        return True

    def choose_frontier_goal(self):
        pose = self.robot_pose()
        if pose is None or self.map_msg is None:
            return None, "not_ready"
        start = self.world_to_cell(pose[0], pose[1])
        if start is None or not self.is_free(start):
            return None, "bad_start"

        nodes = [start]
        min_goal_cells = int(self.args.min_goal_distance_m / self.map_msg.info.resolution)
        for _ in range(self.args.max_samples):
            sample = self.sample_free_cell(start)
            if sample is None:
                continue
            nearest = min(nodes, key=lambda c: (c[0] - sample[0]) ** 2 + (c[1] - sample[1]) ** 2)
            new_cell = self.steer(nearest, sample)
            if new_cell == nearest or new_cell in nodes:
                continue
            if not (0 <= new_cell[0] < self.map_msg.info.width and 0 <= new_cell[1] < self.map_msg.info.height):
                continue
            if not self.segment_free(nearest, new_cell):
                continue
            nodes.append(new_cell)
            if self.is_frontier(new_cell):
                d = abs(new_cell[0] - start[0]) + abs(new_cell[1] - start[1])
                if d >= min_goal_cells:
                    return new_cell, "frontier"
        return None, "no_frontier"

    def make_goal(self, cell):
        pose = self.robot_pose()
        gx, gy = self.cell_to_world(cell)
        yaw = 0.0 if pose is None else math.atan2(gy - pose[1], gx - pose[0])
        yaw = quantize_angle(yaw, math.radians(self.args.yaw_step_deg))
        msg = PoseStamped()
        msg.header.frame_id = self.args.map_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = gx
        msg.pose.position.y = gy
        qx, qy, qz, qw = yaw_to_quat(yaw)
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        return msg

    def send_goal(self, goal):
        self.goal_pub.publish(goal)
        if self.nav2_client is None:
            return "published_only"
        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = goal
        send_future = self.nav2_client.send_goal_async(nav_goal)
        deadline = time.monotonic() + self.args.goal_send_timeout_s
        while rclpy.ok() and not send_future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        if not send_future.done():
            return "send_timeout"
        handle = send_future.result()
        if handle is None or not handle.accepted:
            return "rejected"
        result_future = handle.get_result_async()
        deadline = time.monotonic() + self.args.goal_result_timeout_s
        while rclpy.ok() and not result_future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.risk_detected:
                cancel_future = handle.cancel_goal_async()
                while rclpy.ok() and not cancel_future.done():
                    rclpy.spin_once(self, timeout_sec=0.05)
                    break
                return "risk_detected"
        if not result_future.done():
            handle.cancel_goal_async()
            return "result_timeout"
        result = result_future.result()
        return f"status_{getattr(result, 'status', 'unknown')}"

    def run(self):
        started = time.monotonic()
        if not self.wait_ready():
            print("RRT_NOT_READY", flush=True)
            return 2
        print("RRT_READY", flush=True)
        while rclpy.ok() and time.monotonic() - started < self.args.runtime_s and self.goal_count < self.args.max_goals:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.risk_detected:
                status = "risk_detected"
                break
            cell, reason = self.choose_frontier_goal()
            if cell is None:
                status = reason
                time.sleep(self.args.replan_sleep_s)
                continue
            goal = self.make_goal(cell)
            self.goal_count += 1
            record = {
                "goal_count": self.goal_count,
                "cell": [cell[0], cell[1]],
                "xy": [round(goal.pose.position.x, 3), round(goal.pose.position.y, 3)],
                "reason": reason,
                "time_s": round(time.monotonic() - started, 2),
            }
            print("RRT_GOAL", json.dumps(record), flush=True)
            nav_status = self.send_goal(goal)
            record["nav_status"] = nav_status
            self.records.append(record)
            if nav_status == "risk_detected":
                status = nav_status
                break
        else:
            status = "complete"
        summary = {
            "status": status,
            "goals": self.goal_count,
            "runtime_s": round(time.monotonic() - started, 2),
            "records": self.records,
            "published_cmd_vel": False,
        }
        if self.args.report:
            path = Path(self.args.report)
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        print("RRT_SUMMARY", json.dumps(summary, ensure_ascii=False), flush=True)
        return 0


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--map-topic", default="/map")
    parser.add_argument("--goal-topic", default="/goal_pose")
    parser.add_argument("--stop-on-risk-topic", default="/risk/sim_detections")
    parser.add_argument("--map-frame", default="map")
    parser.add_argument("--base-frame", default="base_footprint")
    parser.add_argument("--runtime-s", type=float, default=180.0)
    parser.add_argument("--max-goals", type=int, default=12)
    parser.add_argument("--sample-radius-m", type=float, default=4.0)
    parser.add_argument("--min-goal-distance-m", type=float, default=0.55)
    parser.add_argument("--inflation-m", type=float, default=0.20)
    parser.add_argument("--rrt-step-cells", type=int, default=8)
    parser.add_argument("--max-samples", type=int, default=900)
    parser.add_argument("--free-threshold", type=int, default=20)
    parser.add_argument("--occupied-threshold", type=int, default=65)
    parser.add_argument("--wait-ready-s", type=float, default=35.0)
    parser.add_argument("--goal-send-timeout-s", type=float, default=4.0)
    parser.add_argument("--goal-result-timeout-s", type=float, default=25.0)
    parser.add_argument("--replan-sleep-s", type=float, default=1.0)
    parser.add_argument("--yaw-step-deg", type=float, default=30.0)
    parser.add_argument("--send-nav2-action", action="store_true")
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--report", default="")
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = RRTFrontierExplorer(args)
    try:
        code = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(code)


if __name__ == "__main__":
    main()
