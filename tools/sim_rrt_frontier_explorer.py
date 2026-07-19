#!/usr/bin/env python3
import argparse
from collections import deque
import json
import math
import random
import time
import zlib
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

try:
    from nav2_msgs.action import NavigateToPose
    from rclpy.action import ActionClient
except ImportError:
    NavigateToPose = None
    ActionClient = None


def is_rcl_context_shutdown_error(exc):
    text = str(exc)
    return "context is not valid" in text or "rcl_shutdown already called" in text


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def yaw_to_quat(yaw):
    half = yaw * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


def quantize_angle(angle, step_rad):
    if step_rad <= 1e-6:
        return angle
    return round(angle / step_rad) * step_rad


def angle_diff(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


def clamp(value, lo, hi):
    return max(lo, min(hi, value))


def occupancy_grid_signature(msg):
    """Return a content signature without retaining another map-sized copy."""
    try:
        payload = memoryview(msg.data).cast("B")
    except TypeError:
        payload = bytes((int(value) & 0xFF for value in msg.data))
    info = msg.info
    origin = info.origin.position
    return (
        int(info.width),
        int(info.height),
        round(float(info.resolution), 9),
        round(float(origin.x), 6),
        round(float(origin.y), 6),
        zlib.crc32(payload),
    )


class RRTFrontierExplorer(Node):
    def __init__(self, args):
        super().__init__("sim_rrt_frontier_explorer")
        self.args = args
        self.map_msg = None
        self.map_signature = None
        self.map_generation = 0
        self.last_planned_map_generation = -1
        self.last_map_change_monotonic = None
        self.planning_cycles = 0
        self.identical_map_updates = 0
        self.risk_detected = False
        self.records = []
        self.goal_count = 0
        self.consecutive_failures = 0
        self.failure_backoff_until = 0.0
        self.rejected_cells = []
        self.recent_goal_cells = []
        self.last_goal_meta = {}
        self.current_sample_radius_m = float(args.sample_radius_m)
        self.random = random.Random(args.seed)
        self.last_physical_cmd_linear = 0.0
        self.last_physical_cmd_angular = 0.0
        self.last_physical_cmd_time = None
        self.last_odom_linear = 0.0
        self.last_odom_angular = 0.0
        self.last_odom_time = None
        self.physical_stuck_since = None
        self.physical_stuck_kind = None
        self.physical_stuck_escape_sign = 1.0
        self.goal_pub = self.create_publisher(PoseStamped, args.goal_topic, 10)
        self.physical_stuck_escape_pub = None
        if args.physical_stuck_escape_cmd_topic:
            self.physical_stuck_escape_pub = self.create_publisher(
                Twist, args.physical_stuck_escape_cmd_topic, 10
            )
        self.create_subscription(OccupancyGrid, args.map_topic, self.map_cb, 10)
        if args.stop_on_risk_topic:
            self.create_subscription(String, args.stop_on_risk_topic, self.risk_cb, 10)
        if args.physical_stuck_s > 0:
            self.create_subscription(Twist, args.physical_stuck_cmd_topic, self.physical_cmd_cb, 10)
            self.create_subscription(Odometry, args.physical_stuck_odom_topic, self.odom_cb, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.nav2_client = None
        if args.send_nav2_action and NavigateToPose is not None and ActionClient is not None:
            self.nav2_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

    def map_cb(self, msg):
        signature = occupancy_grid_signature(msg)
        if signature != self.map_signature:
            self.map_signature = signature
            self.map_generation += 1
            self.last_map_change_monotonic = time.monotonic()
        else:
            self.identical_map_updates += 1
        self.map_msg = msg

    def spin_wait(self, timeout_s, wake_on_map_change=False):
        deadline = time.monotonic() + max(0.0, float(timeout_s))
        initial_generation = self.map_generation
        while rclpy.ok() and time.monotonic() < deadline:
            if self.risk_detected:
                return "risk_detected"
            if wake_on_map_change and self.map_generation != initial_generation:
                return "map_changed"
            rclpy.spin_once(self, timeout_sec=min(0.1, max(0.0, deadline - time.monotonic())))
        return "timeout"

    def physical_cmd_cb(self, msg):
        self.last_physical_cmd_linear = float(msg.linear.x)
        self.last_physical_cmd_angular = float(msg.angular.z)
        self.last_physical_cmd_time = time.monotonic()

    def odom_cb(self, msg):
        self.last_odom_linear = float(msg.twist.twist.linear.x)
        self.last_odom_angular = float(msg.twist.twist.angular.z)
        self.last_odom_time = time.monotonic()

    def risk_cb(self, msg):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if payload.get("alarm") is True:
            self.risk_detected = True
            return
        if payload.get("event_id") is not None and payload.get("class_name") is not None:
            self.risk_detected = True
            return
        if payload.get("active_risk_id") is not None and payload.get("state") != "idle":
            self.risk_detected = True
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

    def physical_stuck_detected(self, now):
        if self.args.physical_stuck_s <= 0:
            self.physical_stuck_since = None
            self.physical_stuck_kind = None
            return False
        if self.last_physical_cmd_time is None or self.last_odom_time is None:
            self.physical_stuck_since = None
            self.physical_stuck_kind = None
            return False
        if now - self.last_physical_cmd_time > self.args.physical_stuck_cmd_timeout_s:
            self.physical_stuck_since = None
            self.physical_stuck_kind = None
            return False
        if now - self.last_odom_time > self.args.physical_stuck_odom_timeout_s:
            self.physical_stuck_since = None
            self.physical_stuck_kind = None
            return False
        linear_stalled = (
            abs(self.last_physical_cmd_linear) > self.args.physical_stuck_cmd_linear_mps
            and abs(self.last_odom_linear) < self.args.physical_stuck_odom_linear_mps
        )
        angular_stalled = (
            abs(self.last_physical_cmd_angular) > self.args.physical_stuck_cmd_angular_radps
            and abs(self.last_odom_angular) < self.args.physical_stuck_odom_angular_radps
        )
        if linear_stalled or angular_stalled:
            self.physical_stuck_kind = "linear" if linear_stalled else "angular"
            if self.physical_stuck_since is None:
                self.physical_stuck_since = now
                return False
            return now - self.physical_stuck_since >= self.args.physical_stuck_s
        self.physical_stuck_since = None
        self.physical_stuck_kind = None
        return False

    def cancel_goal(self, handle):
        cancel_future = handle.cancel_goal_async()
        cancel_deadline = time.monotonic() + self.args.goal_cancel_timeout_s
        while rclpy.ok() and not cancel_future.done() and time.monotonic() < cancel_deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

    def publish_physical_stuck_escape(self, stuck_kind):
        if self.physical_stuck_escape_pub is None:
            return {"published": False}
        if stuck_kind == "angular" and abs(self.last_physical_cmd_angular) > 1e-6:
            sign = -math.copysign(1.0, self.last_physical_cmd_angular)
        else:
            sign = self.physical_stuck_escape_sign
        self.physical_stuck_escape_sign *= -1.0
        reverse_s = (
            0.0
            if stuck_kind == "angular"
            else max(0.0, self.args.physical_stuck_escape_reverse_s)
        )
        turn_s = max(0.0, self.args.physical_stuck_escape_turn_s)
        rate_s = 1.0 / max(1.0, self.args.physical_stuck_escape_rate_hz)
        started = time.monotonic()
        msg = Twist()
        msg.linear.x = -abs(self.args.physical_stuck_escape_reverse_mps)
        msg.angular.z = sign * abs(self.args.physical_stuck_escape_angular_radps)
        while rclpy.ok() and time.monotonic() - started < reverse_s:
            self.physical_stuck_escape_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(rate_s)
        turn_started = time.monotonic()
        msg = Twist()
        msg.angular.z = sign * abs(self.args.physical_stuck_escape_angular_radps)
        while rclpy.ok() and time.monotonic() - turn_started < turn_s:
            self.physical_stuck_escape_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(rate_s)
        self.physical_stuck_escape_pub.publish(Twist())
        self.physical_stuck_since = None
        detail = {
            "published": True,
            "stuck_kind": stuck_kind,
            "reverse_mps": round(-abs(self.args.physical_stuck_escape_reverse_mps), 3),
            "reverse_s": round(reverse_s, 3),
            "angular_radps": round(sign * abs(self.args.physical_stuck_escape_angular_radps), 3),
            "turn_s": round(turn_s, 3),
        }
        print("RRT_ESCAPE", json.dumps({"reason": "physical_stuck", **detail}), flush=True)
        return detail

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

    def in_bounds(self, cell):
        return 0 <= cell[0] < self.map_msg.info.width and 0 <= cell[1] < self.map_msg.info.height

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
                if self.in_bounds(nb):
                    yield nb

    def neighbors4(self, cell):
        x, y = cell
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nb = (x + dx, y + dy)
            if self.in_bounds(nb):
                yield nb

    def near_obstacle(self, cell, radius_cells):
        cx, cy = cell
        for y in range(max(0, cy - radius_cells), min(self.map_msg.info.height, cy + radius_cells + 1)):
            for x in range(max(0, cx - radius_cells), min(self.map_msg.info.width, cx + radius_cells + 1)):
                if self.map_msg.data[y * self.map_msg.info.width + x] >= self.args.occupied_threshold:
                    return True
        return False

    def is_safe_free(self, cell, inflation_cells=None):
        if not self.is_free(cell):
            return False
        if self.near_map_edge(cell):
            return False
        if inflation_cells is None:
            inflation_cells = self.inflation_cells()
        return not self.near_obstacle(cell, inflation_cells)

    def is_frontier(self, cell):
        return self.is_frontier_with_inflation(cell, self.inflation_cells())

    def is_frontier_with_inflation(self, cell, inflation_cells):
        if not self.is_safe_free(cell, inflation_cells):
            return False
        return any(self.is_unknown(nb) for nb in self.neighbors8(cell))

    def near_map_edge(self, cell):
        margin_cells = max(0, int(self.args.map_edge_margin_m / self.map_msg.info.resolution))
        return (
            cell[0] < margin_cells
            or cell[1] < margin_cells
            or cell[0] >= self.map_msg.info.width - margin_cells
            or cell[1] >= self.map_msg.info.height - margin_cells
        )

    def near_suppressed_goal(self, cell):
        now = time.monotonic()
        recent_dist_cells = max(1, int(self.args.goal_separation_m / self.map_msg.info.resolution))
        recent_dist_sq = recent_dist_cells * recent_dist_cells
        if self.args.rejected_goal_memory > 0:
            if self.args.rejected_goal_cooldown_s > 0:
                self.rejected_cells = [
                    entry
                    for entry in self.rejected_cells
                    if now - entry[1] < self.args.rejected_goal_cooldown_s
                ]
            rejected_dist_cells = max(
                recent_dist_cells,
                int(self.args.rejected_goal_separation_m / self.map_msg.info.resolution),
            )
            rejected_dist_sq = rejected_dist_cells * rejected_dist_cells
            for old, _ in self.rejected_cells[-self.args.rejected_goal_memory :]:
                if (cell[0] - old[0]) ** 2 + (cell[1] - old[1]) ** 2 <= rejected_dist_sq:
                    return True
        if self.args.recent_goal_memory > 0:
            if self.args.recent_goal_cooldown_s > 0:
                self.recent_goal_cells = [
                    entry
                    for entry in self.recent_goal_cells
                    if now - entry[1] < self.args.recent_goal_cooldown_s
                ]
            for old, _ in self.recent_goal_cells[-self.args.recent_goal_memory :]:
                if (cell[0] - old[0]) ** 2 + (cell[1] - old[1]) ** 2 <= recent_dist_sq:
                    return True
        return False

    def is_free_roam_goal(self, cell, start, min_goal_cells):
        if not self.is_free(cell):
            return False
        if self.near_map_edge(cell):
            return False
        if self.near_obstacle(cell, self.inflation_cells()):
            return False
        if not self.goal_clearance_ok(cell):
            return False
        if self.near_suppressed_goal(cell):
            return False
        dist = math.hypot(cell[0] - start[0], cell[1] - start[1])
        min_free_roam_cells = max(
            min_goal_cells,
            int(self.args.free_roam_min_distance_m / self.map_msg.info.resolution),
        )
        return dist >= min_free_roam_cells

    def nearest_free_start_cell(self, start):
        if start is None:
            return None
        if self.is_free(start):
            return start
        max_radius_cells = max(1, int(self.args.start_free_search_m / self.map_msg.info.resolution))
        best = None
        best_dist_sq = None
        sx, sy = start
        for radius in range(1, max_radius_cells + 1):
            for y in range(max(0, sy - radius), min(self.map_msg.info.height, sy + radius + 1)):
                for x in range(max(0, sx - radius), min(self.map_msg.info.width, sx + radius + 1)):
                    cell = (x, y)
                    if not self.is_free(cell):
                        continue
                    if self.near_obstacle(cell, self.inflation_cells()):
                        continue
                    dist_sq = (x - sx) ** 2 + (y - sy) ** 2
                    if best is None or dist_sq < best_dist_sq:
                        best = cell
                        best_dist_sq = dist_sq
            if best is not None:
                return best
        return None

    def frontier_backoffs_m(self):
        values = [max(0.0, float(self.args.frontier_standoff_m))]
        for raw in str(self.args.frontier_backoffs_m).split(","):
            raw = raw.strip()
            if not raw:
                continue
            try:
                value = max(0.0, float(raw))
            except ValueError:
                continue
            if all(abs(value - existing) > 1e-6 for existing in values):
                values.append(value)
        return values

    def unknown_direction(self, frontier_cell, start_cell):
        fx, fy = frontier_cell
        ux = 0.0
        uy = 0.0
        for nb in self.neighbors8(frontier_cell):
            if not self.is_unknown(nb):
                continue
            ux += nb[0] - fx
            uy += nb[1] - fy
        dist = math.hypot(ux, uy)
        if dist >= 1e-6:
            return ux / dist, uy / dist, "unknown_normal"

        sx, sy = start_cell
        ux = fx - sx
        uy = fy - sy
        dist = math.hypot(ux, uy)
        if dist >= 1e-6:
            return ux / dist, uy / dist, "start_vector"
        return 0.0, 0.0, "none"

    def nearest_safe_goal_cell(self, target_cell, max_radius_cells=2):
        if self.in_bounds(target_cell) and self.is_safe_free(target_cell):
            return target_cell
        tx, ty = target_cell
        best = None
        best_dist_sq = None
        for radius in range(1, max(1, max_radius_cells) + 1):
            for y in range(max(0, ty - radius), min(self.map_msg.info.height, ty + radius + 1)):
                for x in range(max(0, tx - radius), min(self.map_msg.info.width, tx + radius + 1)):
                    cell = (x, y)
                    if not self.is_safe_free(cell):
                        continue
                    dist_sq = (x - tx) ** 2 + (y - ty) ** 2
                    if best is None or dist_sq < best_dist_sq:
                        best = cell
                        best_dist_sq = dist_sq
            if best is not None:
                return best
        return None

    def clearance_m(self, cell, max_distance_m=None):
        max_distance = self.args.goal_clearance_check_m if max_distance_m is None else max_distance_m
        max_radius_cells = max(1, int(max_distance / self.map_msg.info.resolution))
        cx, cy = cell
        best_dist_sq = None
        for y in range(max(0, cy - max_radius_cells), min(self.map_msg.info.height, cy + max_radius_cells + 1)):
            for x in range(max(0, cx - max_radius_cells), min(self.map_msg.info.width, cx + max_radius_cells + 1)):
                if self.map_msg.data[y * self.map_msg.info.width + x] < self.args.occupied_threshold:
                    continue
                dist_sq = (x - cx) ** 2 + (y - cy) ** 2
                if best_dist_sq is None or dist_sq < best_dist_sq:
                    best_dist_sq = dist_sq
        if best_dist_sq is None:
            return max_radius_cells * self.map_msg.info.resolution
        return math.sqrt(best_dist_sq) * self.map_msg.info.resolution

    def goal_clearance_ok(self, cell):
        if cell is None:
            return False
        return self.clearance_m(cell) >= self.args.min_goal_clearance_m

    def unknown_gain(self, cell, radius_m=None):
        radius_m = self.args.frontier_unknown_gain_radius_m if radius_m is None else radius_m
        radius_cells = max(1, int(radius_m / self.map_msg.info.resolution))
        radius_sq = radius_cells * radius_cells
        cx, cy = cell
        count = 0
        for y in range(max(0, cy - radius_cells), min(self.map_msg.info.height, cy + radius_cells + 1)):
            for x in range(max(0, cx - radius_cells), min(self.map_msg.info.width, cx + radius_cells + 1)):
                if (x - cx) ** 2 + (y - cy) ** 2 > radius_sq:
                    continue
                if self.map_msg.data[y * self.map_msg.info.width + x] < 0:
                    count += 1
        return count

    def sample_radius_m(self, start):
        if not self.args.adaptive_sample_radius:
            self.current_sample_radius_m = float(self.args.sample_radius_m)
            self.last_goal_meta["sample_radius_mode"] = "fixed"
            self.last_goal_meta["sample_radius_m"] = round(self.current_sample_radius_m, 3)
            return self.current_sample_radius_m

        local_clearance = self.clearance_m(start, self.args.adaptive_wide_clearance_m)
        if local_clearance >= self.args.adaptive_wide_clearance_m:
            mode = "wide"
            radius = self.args.adaptive_wide_sample_radius_m
        else:
            mode = "tight"
            radius = self.args.adaptive_tight_sample_radius_m
        self.current_sample_radius_m = float(radius)
        self.last_goal_meta["sample_radius_mode"] = mode
        self.last_goal_meta["sample_radius_m"] = round(self.current_sample_radius_m, 3)
        self.last_goal_meta["local_clearance_m"] = round(local_clearance, 3)
        self.last_goal_meta["wide_clearance_threshold_m"] = round(
            float(self.args.adaptive_wide_clearance_m), 3
        )
        return self.current_sample_radius_m

    def frontier_goal_cell(self, frontier_cell, start_cell, reachable_cells=None):
        resolution = self.map_msg.info.resolution
        ux, uy, direction_source = self.unknown_direction(frontier_cell, start_cell)
        if direction_source == "none":
            if not self.goal_clearance_ok(frontier_cell):
                return None, {
                    "frontier_cell": [frontier_cell[0], frontier_cell[1]],
                    "candidate_type": "frontier_raw_rejected_clearance",
                    "direction_source": direction_source,
                    "goal_clearance_m": round(self.clearance_m(frontier_cell), 3),
                    "min_goal_clearance_m": round(self.args.min_goal_clearance_m, 3),
                    "unknown_gain": self.unknown_gain(frontier_cell),
                }
            return frontier_cell, {
                "frontier_cell": [frontier_cell[0], frontier_cell[1]],
                "candidate_type": "frontier_raw",
                "direction_source": direction_source,
                "goal_clearance_m": round(self.clearance_m(frontier_cell), 3),
                "unknown_gain": self.unknown_gain(frontier_cell),
            }

        candidates = []
        for backoff_m in self.frontier_backoffs_m():
            step = backoff_m / resolution
            raw_cell = (
                int(round(frontier_cell[0] - ux * step)),
                int(round(frontier_cell[1] - uy * step)),
            )
            goal_cell = self.nearest_safe_goal_cell(raw_cell, max_radius_cells=2)
            if goal_cell is None:
                continue
            if reachable_cells is not None and goal_cell not in reachable_cells:
                continue
            clearance = self.clearance_m(goal_cell)
            if clearance < self.args.min_goal_clearance_m:
                continue
            dist_from_start = math.hypot(goal_cell[0] - start_cell[0], goal_cell[1] - start_cell[1])
            candidates.append(
                {
                    "cell": goal_cell,
                    "backoff_m": backoff_m,
                    "clearance_m": clearance,
                    "dist_from_start": dist_from_start,
                    "raw_cell": raw_cell,
                }
            )

        if not candidates:
            return None, {
                "frontier_cell": [frontier_cell[0], frontier_cell[1]],
                "candidate_type": "frontier_no_clearance_candidate",
                "direction_source": direction_source,
                "goal_clearance_m": round(self.clearance_m(frontier_cell), 3),
                "min_goal_clearance_m": round(self.args.min_goal_clearance_m, 3),
                "unknown_gain": self.unknown_gain(frontier_cell),
                "candidate_count": 0,
            }

        best = max(
            candidates,
            key=lambda item: (
                item["clearance_m"],
                -abs(item["backoff_m"] - self.args.frontier_standoff_m),
                -item["dist_from_start"],
                self.random.random() * 0.001,
            ),
        )
        cell = best["cell"]
        return cell, {
            "frontier_cell": [frontier_cell[0], frontier_cell[1]],
            "candidate_type": "frontier_backoff",
            "direction_source": direction_source,
            "candidate_count": len(candidates),
            "candidate_backoff_m": round(best["backoff_m"], 3),
            "goal_clearance_m": round(best["clearance_m"], 3),
            "unknown_gain": self.unknown_gain(frontier_cell),
            "candidate_raw_cell": [best["raw_cell"][0], best["raw_cell"][1]],
        }

    def inflation_cells(self):
        return max(1, int(self.args.inflation_m / self.map_msg.info.resolution))

    def sample_free_cell(self, start):
        radius_cells = int(self.current_sample_radius_m / self.map_msg.info.resolution)
        for _ in range(100):
            x = self.random.randint(max(0, start[0] - radius_cells), min(self.map_msg.info.width - 1, start[0] + radius_cells))
            y = self.random.randint(max(0, start[1] - radius_cells), min(self.map_msg.info.height - 1, start[1] + radius_cells))
            cell = (x, y)
            if self.is_free(cell) and not self.near_obstacle(cell, self.inflation_cells()):
                return cell
        return None

    def legacy_frontier_goal_cell(self, frontier_cell, start_cell):
        retreat_cells = max(0, int(self.args.frontier_standoff_m / self.map_msg.info.resolution))
        fx, fy = frontier_cell
        sx, sy = start_cell
        dx = sx - fx
        dy = sy - fy
        dist = math.hypot(dx, dy)
        if dist < 1e-6 or retreat_cells == 0:
            return frontier_cell
        for step in range(retreat_cells, -1, -1):
            cell = (
                int(round(fx + dx / dist * step)),
                int(round(fy + dy / dist * step)),
            )
            if (
                self.in_bounds(cell)
                and self.is_safe_free(cell)
            ):
                return cell
        return frontier_cell

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

    def cluster_frontiers(self, frontier_cells):
        remaining = set(frontier_cells)
        clusters = []
        while remaining:
            seed = remaining.pop()
            queue = deque([seed])
            cluster = [seed]
            while queue:
                cell = queue.popleft()
                for nb in self.neighbors8(cell):
                    if nb not in remaining:
                        continue
                    remaining.remove(nb)
                    queue.append(nb)
                    cluster.append(nb)
            clusters.append(cluster)
        return clusters

    def choose_wfd_goal(self, start, min_goal_cells):
        radius_cells = max(1, int(self.current_sample_radius_m / self.map_msg.info.resolution))
        radius_sq = radius_cells * radius_cells
        inflation_cells = self.inflation_cells()
        frontier_cells = set()
        visited = {start}
        queue = deque([start])
        best_free_roam = None
        best_free_roam_score = -1.0

        while queue and len(visited) < self.args.wfd_max_cells:
            cell = queue.popleft()
            if self.is_frontier_with_inflation(cell, inflation_cells):
                frontier_cells.add(cell)
            if self.args.free_roam_when_no_frontier and self.is_free_roam_goal(cell, start, min_goal_cells):
                dist = math.hypot(cell[0] - start[0], cell[1] - start[1])
                gain = self.unknown_gain(cell, self.args.free_roam_unknown_gain_radius_m)
                score = (
                    self.args.free_roam_unknown_weight * gain
                    + self.args.free_roam_distance_weight * dist
                    + self.random.random() * 0.01
                )
                if score > best_free_roam_score:
                    best_free_roam = cell
                    best_free_roam_score = score

            for nb in self.neighbors4(cell):
                if nb in visited:
                    continue
                if (nb[0] - start[0]) ** 2 + (nb[1] - start[1]) ** 2 > radius_sq:
                    continue
                if not self.is_safe_free(nb, inflation_cells):
                    continue
                visited.add(nb)
                queue.append(nb)

        best_goal = None
        best_meta = {}
        best_score = None
        for cluster in self.cluster_frontiers(frontier_cells):
            if len(cluster) < self.args.min_frontier_cluster_cells:
                continue
            cx = sum(c[0] for c in cluster) / len(cluster)
            cy = sum(c[1] for c in cluster) / len(cluster)
            cluster_candidates = sorted(
                cluster,
                key=lambda c: (
                    (c[0] - start[0]) ** 2 + (c[1] - start[1]) ** 2,
                    (c[0] - cx) ** 2 + (c[1] - cy) ** 2,
                ),
            )[: self.args.frontier_cluster_candidate_limit]
            for frontier_cell in cluster_candidates:
                goal_cell, meta = self.frontier_goal_cell(frontier_cell, start, visited)
                if goal_cell not in visited:
                    continue
                if self.near_suppressed_goal(goal_cell):
                    continue
                dist_cells = math.hypot(goal_cell[0] - start[0], goal_cell[1] - start[1])
                if dist_cells < min_goal_cells:
                    continue
                unknown_gain = int(meta.get("unknown_gain") or self.unknown_gain(frontier_cell))
                score = (
                    self.args.frontier_size_weight * len(cluster)
                    + self.args.frontier_unknown_weight * unknown_gain
                    - self.args.frontier_distance_weight * dist_cells
                    + self.random.random() * 0.001
                )
                if best_score is None or score > best_score:
                    best_goal = goal_cell
                    best_meta = {
                        **meta,
                        "frontier_size": len(cluster),
                        "frontier_id": "%d_%d_%d" % (int(cx), int(cy), len(cluster)),
                        "frontier_score": round(score, 3),
                    }
                    best_score = score

        if best_goal is not None:
            self.last_goal_meta = best_meta
            return best_goal, "wfd_frontier"
        if best_free_roam is not None:
            self.last_goal_meta = {
                "candidate_type": "wfd_free_roam",
                "goal_clearance_m": round(self.clearance_m(best_free_roam), 3),
                "unknown_gain": self.unknown_gain(best_free_roam, self.args.free_roam_unknown_gain_radius_m),
                "free_roam_score": round(best_free_roam_score, 3),
            }
            return best_free_roam, "wfd_free_roam"
        return None, "wfd_no_frontier"

    def choose_rrt_goal(self, start, min_goal_cells):
        nodes = [start]
        node_set = {start}
        best_free_roam = None
        best_free_roam_score = -1.0
        for _ in range(self.args.max_samples):
            sample = self.sample_free_cell(start)
            if sample is None:
                continue
            nearest = min(nodes, key=lambda c: (c[0] - sample[0]) ** 2 + (c[1] - sample[1]) ** 2)
            new_cell = self.steer(nearest, sample)
            if new_cell == nearest or new_cell in node_set:
                continue
            if not self.in_bounds(new_cell):
                continue
            if not self.segment_free(nearest, new_cell):
                continue
            nodes.append(new_cell)
            node_set.add(new_cell)
            if self.args.free_roam_when_no_frontier and self.is_free_roam_goal(new_cell, start, min_goal_cells):
                dist = math.hypot(new_cell[0] - start[0], new_cell[1] - start[1])
                gain = self.unknown_gain(new_cell, self.args.free_roam_unknown_gain_radius_m)
                score = (
                    self.args.free_roam_unknown_weight * gain
                    + self.args.free_roam_distance_weight * dist
                    + self.random.random() * 0.01
                )
                if score > best_free_roam_score:
                    best_free_roam = new_cell
                    best_free_roam_score = score
            if self.is_frontier(new_cell):
                goal_cell, meta = self.frontier_goal_cell(new_cell, start)
                if goal_cell is None:
                    continue
                if self.near_suppressed_goal(goal_cell):
                    continue
                d = abs(goal_cell[0] - start[0]) + abs(goal_cell[1] - start[1])
                if d >= min_goal_cells:
                    self.last_goal_meta = {
                        **meta,
                        "frontier_size": 1,
                        "frontier_id": "%d_%d_1" % (new_cell[0], new_cell[1]),
                    }
                    return goal_cell, "frontier_standoff"
        if best_free_roam is not None:
            self.last_goal_meta = {
                "candidate_type": "free_roam",
                "goal_clearance_m": round(self.clearance_m(best_free_roam), 3),
                "unknown_gain": self.unknown_gain(best_free_roam, self.args.free_roam_unknown_gain_radius_m),
                "free_roam_score": round(best_free_roam_score, 3),
            }
            return best_free_roam, "free_roam"
        return None, "no_frontier"

    def choose_frontier_goal(self):
        self.last_goal_meta = {}
        pose = self.robot_pose()
        if pose is None or self.map_msg is None:
            return None, "not_ready"
        start = self.world_to_cell(pose[0], pose[1])
        start = self.nearest_free_start_cell(start)
        if start is None:
            return None, "bad_start"

        min_goal_cells = int(self.args.min_goal_distance_m / self.map_msg.info.resolution)
        self.sample_radius_m(start)
        sample_meta = dict(self.last_goal_meta)
        if self.args.frontier_mode in ("wfd", "hybrid"):
            goal, reason = self.choose_wfd_goal(start, min_goal_cells)
            if goal is not None or self.args.frontier_mode == "wfd":
                self.last_goal_meta = {**sample_meta, **self.last_goal_meta}
                return goal, reason
        goal, reason = self.choose_rrt_goal(start, min_goal_cells)
        self.last_goal_meta = {**sample_meta, **self.last_goal_meta}
        return goal, reason

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
        progress = {
            "feedback_seen": False,
            "last_distance": None,
            "last_improved": time.monotonic(),
            "last_pose_distance": None,
            "last_heading_error": None,
        }
        goal_x = float(goal.pose.position.x)
        goal_y = float(goal.pose.position.y)
        yaw_epsilon = math.radians(self.args.goal_progress_yaw_epsilon_deg)

        def feedback_cb(msg):
            feedback = getattr(msg, "feedback", None)
            distance = getattr(feedback, "distance_remaining", None)
            if distance is None:
                return
            try:
                distance = float(distance)
            except (TypeError, ValueError):
                return
            if not math.isfinite(distance):
                return
            now = time.monotonic()
            progress["feedback_seen"] = True
            last_distance = progress["last_distance"]
            if last_distance is None or distance < last_distance - self.args.goal_progress_epsilon_m:
                progress["last_distance"] = distance
                progress["last_improved"] = now

        send_future = self.nav2_client.send_goal_async(nav_goal, feedback_callback=feedback_cb)
        deadline = time.monotonic() + self.args.goal_send_timeout_s
        while rclpy.ok() and not send_future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        if not send_future.done():
            return "send_timeout"
        handle = send_future.result()
        if handle is None or not handle.accepted:
            return "rejected"
        result_future = handle.get_result_async()
        accepted_at = time.monotonic()
        deadline = time.monotonic() + self.args.goal_result_timeout_s
        while rclpy.ok() and not result_future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.risk_detected:
                self.cancel_goal(handle)
                return "risk_detected"
            now = time.monotonic()
            if self.physical_stuck_detected(now):
                detail = {
                    "stuck_kind": self.physical_stuck_kind,
                    "cmd_linear_x": round(self.last_physical_cmd_linear, 3),
                    "cmd_angular_z": round(self.last_physical_cmd_angular, 3),
                    "odom_linear_x": round(self.last_odom_linear, 3),
                    "odom_angular_z": round(self.last_odom_angular, 3),
                    "duration_s": round(now - (self.physical_stuck_since or now), 2),
                }
                print("RRT_PHYSICAL_STUCK", json.dumps(detail), flush=True)
                self.cancel_goal(handle)
                return "physical_stuck"
            pose = self.robot_pose()
            if pose is not None:
                dx = goal_x - pose[0]
                dy = goal_y - pose[1]
                pose_distance = math.hypot(dx, dy)
                target_yaw = math.atan2(dy, dx) if pose_distance > 1e-6 else pose[2]
                heading_error = abs(angle_diff(target_yaw, pose[2]))
                pose_improved = (
                    progress["last_pose_distance"] is None
                    or pose_distance
                    < progress["last_pose_distance"] - self.args.goal_progress_epsilon_m
                    or (
                        progress["last_heading_error"] is not None
                        and heading_error < progress["last_heading_error"] - yaw_epsilon
                    )
                )
                if pose_improved:
                    progress["last_pose_distance"] = pose_distance
                    progress["last_heading_error"] = heading_error
                    progress["last_improved"] = now
            if (
                self.args.goal_progress_timeout_s > 0
                and progress["feedback_seen"]
                and now - accepted_at >= self.args.goal_progress_grace_s
                and now - progress["last_improved"] >= self.args.goal_progress_timeout_s
            ):
                self.cancel_goal(handle)
                return "progress_timeout"
        if not result_future.done():
            self.cancel_goal(handle)
            return "result_timeout"
        result = result_future.result()
        return f"status_{getattr(result, 'status', 'unknown')}"

    def run(self):
        started = time.monotonic()
        if not self.wait_ready():
            print("RRT_NOT_READY", flush=True)
            return 2
        print("RRT_READY", flush=True)
        last_idle_log = 0.0
        force_replan = True
        no_frontier_retry_at = 0.0
        idle_wait_s = max(0.1, float(self.args.replan_sleep_s))
        while rclpy.ok() and time.monotonic() - started < self.args.runtime_s and self.goal_count < self.args.max_goals:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.risk_detected:
                status = "risk_detected"
                break
            now = time.monotonic()
            if now < self.failure_backoff_until:
                if now - last_idle_log >= self.args.idle_log_period_s:
                    print(
                        "RRT_WAIT",
                        json.dumps(
                            {
                                "reason": "failure_backoff",
                                "goals": self.goal_count,
                                "time_s": round(now - started, 2),
                                "remaining_s": round(self.failure_backoff_until - now, 2),
                            }
                        ),
                        flush=True,
                    )
                    last_idle_log = now
                self.spin_wait(
                    min(self.args.replan_sleep_s, max(0.1, self.failure_backoff_until - now)),
                    wake_on_map_change=False,
                )
                continue
            if (
                self.args.event_driven_replan
                and not force_replan
                and self.map_generation == self.last_planned_map_generation
                and now < no_frontier_retry_at
            ):
                wake_reason = self.spin_wait(idle_wait_s, wake_on_map_change=True)
                if wake_reason == "risk_detected":
                    status = wake_reason
                    break
                if wake_reason != "map_changed":
                    now = time.monotonic()
                    if now - last_idle_log >= self.args.idle_log_period_s:
                        print(
                            "RRT_WAIT",
                            json.dumps(
                                {
                                    "reason": "unchanged_map",
                                    "goals": self.goal_count,
                                    "time_s": round(now - started, 2),
                                    "map_generation": self.map_generation,
                                    "next_wait_s": round(idle_wait_s, 2),
                                }
                            ),
                            flush=True,
                        )
                        last_idle_log = now
                    idle_wait_s = min(
                        float(self.args.map_idle_max_wait_s),
                        max(float(self.args.replan_sleep_s), idle_wait_s * 2.0),
                    )
                    continue
            cell, reason = self.choose_frontier_goal()
            self.planning_cycles += 1
            self.last_planned_map_generation = self.map_generation
            force_replan = False
            if cell is None:
                status = reason
                now = time.monotonic()
                no_frontier_retry_at = now + max(
                    self.args.replan_sleep_s,
                    self.args.no_frontier_retry_s,
                )
                if now - last_idle_log >= self.args.idle_log_period_s:
                    print(
                        "RRT_WAIT",
                        json.dumps(
                            {
                                "reason": reason,
                                "goals": self.goal_count,
                                "time_s": round(now - started, 2),
                            }
                        ),
                        flush=True,
                    )
                    last_idle_log = now
                idle_wait_s = min(
                    float(self.args.map_idle_max_wait_s),
                    max(float(self.args.replan_sleep_s), idle_wait_s * 2.0),
                )
                continue
            idle_wait_s = max(0.1, float(self.args.replan_sleep_s))
            no_frontier_retry_at = 0.0
            goal = self.make_goal(cell)
            self.goal_count += 1
            record = {
                "goal_count": self.goal_count,
                "cell": [cell[0], cell[1]],
                "xy": [round(goal.pose.position.x, 3), round(goal.pose.position.y, 3)],
                "reason": reason,
                "time_s": round(time.monotonic() - started, 2),
            }
            record.update(self.last_goal_meta)
            print("RRT_GOAL", json.dumps(record), flush=True)
            self.recent_goal_cells.append((cell, time.monotonic()))
            nav_status = self.send_goal(goal)
            force_replan = True
            record["nav_status"] = nav_status
            if nav_status == "physical_stuck":
                record["physical_stuck_escape"] = self.publish_physical_stuck_escape(
                    self.physical_stuck_kind
                )
            self.records.append(record)
            print("RRT_RESULT", json.dumps(record), flush=True)
            if nav_status == "risk_detected":
                status = nav_status
                break
            if nav_status not in ("status_4", "published_only"):
                self.consecutive_failures += 1
                self.rejected_cells.append((cell, time.monotonic()))
                if self.args.rejected_goal_memory > 0:
                    self.rejected_cells = self.rejected_cells[-self.args.rejected_goal_memory :]
                if (
                    self.args.failure_backoff_s > 0
                    and self.args.failure_backoff_after > 0
                    and self.consecutive_failures >= self.args.failure_backoff_after
                ):
                    self.failure_backoff_until = time.monotonic() + self.args.failure_backoff_s
                    self.consecutive_failures = 0
                    print(
                        "RRT_BACKOFF",
                        json.dumps(
                            {
                                "reason": nav_status,
                                "sleep_s": self.args.failure_backoff_s,
                                "goals": self.goal_count,
                            }
                        ),
                        flush=True,
                    )
            else:
                self.consecutive_failures = 0
        else:
            status = "complete"
        summary = {
            "status": status,
            "goals": self.goal_count,
            "runtime_s": round(time.monotonic() - started, 2),
            "records": self.records,
            "published_cmd_vel": False,
            "planning_metrics": {
                "planning_cycles": self.planning_cycles,
                "map_generation": self.map_generation,
                "identical_map_updates_skipped": self.identical_map_updates,
                "event_driven_replan": self.args.event_driven_replan,
            },
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
    parser.add_argument("--adaptive-sample-radius", action="store_true")
    parser.add_argument("--adaptive-tight-sample-radius-m", type=float, default=0.60)
    parser.add_argument("--adaptive-wide-sample-radius-m", type=float, default=1.00)
    parser.add_argument("--adaptive-wide-clearance-m", type=float, default=0.55)
    parser.add_argument("--min-goal-distance-m", type=float, default=0.55)
    parser.add_argument("--inflation-m", type=float, default=0.20)
    parser.add_argument("--rrt-step-cells", type=int, default=8)
    parser.add_argument("--max-samples", type=int, default=900)
    parser.add_argument("--frontier-mode", choices=("rrt", "wfd", "hybrid"), default="hybrid")
    parser.add_argument("--wfd-max-cells", type=int, default=12000)
    parser.add_argument("--min-frontier-cluster-cells", type=int, default=2)
    parser.add_argument("--frontier-cluster-candidate-limit", type=int, default=8)
    parser.add_argument("--frontier-distance-weight", type=float, default=0.25)
    parser.add_argument("--frontier-size-weight", type=float, default=1.0)
    parser.add_argument("--frontier-unknown-weight", type=float, default=0.35)
    parser.add_argument("--frontier-unknown-gain-radius-m", type=float, default=0.35)
    parser.add_argument("--free-threshold", type=int, default=20)
    parser.add_argument("--occupied-threshold", type=int, default=65)
    parser.add_argument("--wait-ready-s", type=float, default=35.0)
    parser.add_argument("--goal-send-timeout-s", type=float, default=4.0)
    parser.add_argument("--goal-result-timeout-s", type=float, default=25.0)
    parser.add_argument("--goal-cancel-timeout-s", type=float, default=2.0)
    parser.add_argument("--goal-progress-timeout-s", type=float, default=12.0)
    parser.add_argument("--goal-progress-grace-s", type=float, default=5.0)
    parser.add_argument("--goal-progress-epsilon-m", type=float, default=0.03)
    parser.add_argument("--goal-progress-yaw-epsilon-deg", type=float, default=5.0)
    parser.add_argument("--failure-backoff-after", type=int, default=8)
    parser.add_argument("--failure-backoff-s", type=float, default=5.0)
    parser.add_argument("--idle-log-period-s", type=float, default=5.0)
    parser.add_argument("--replan-sleep-s", type=float, default=1.0)
    parser.add_argument("--map-idle-max-wait-s", type=float, default=10.0)
    parser.add_argument("--no-frontier-retry-s", type=float, default=10.0)
    parser.add_argument(
        "--event-driven-replan",
        action=argparse.BooleanOptionalAction,
        default=True,
    )
    parser.add_argument("--yaw-step-deg", type=float, default=30.0)
    parser.add_argument("--goal-separation-m", type=float, default=0.45)
    parser.add_argument("--recent-goal-memory", type=int, default=24)
    parser.add_argument("--recent-goal-cooldown-s", type=float, default=30.0)
    parser.add_argument("--rejected-goal-memory", type=int, default=80)
    parser.add_argument("--rejected-goal-cooldown-s", type=float, default=30.0)
    parser.add_argument("--rejected-goal-separation-m", type=float, default=0.25)
    parser.add_argument("--frontier-standoff-m", type=float, default=0.35)
    parser.add_argument("--frontier-backoffs-m", default="0.10,0.18,0.25,0.35")
    parser.add_argument("--goal-clearance-check-m", type=float, default=0.50)
    parser.add_argument("--min-goal-clearance-m", type=float, default=0.16)
    parser.add_argument("--free-roam-when-no-frontier", action="store_true")
    parser.add_argument("--free-roam-min-distance-m", type=float, default=0.25)
    parser.add_argument("--free-roam-unknown-weight", type=float, default=0.30)
    parser.add_argument("--free-roam-distance-weight", type=float, default=0.20)
    parser.add_argument("--free-roam-unknown-gain-radius-m", type=float, default=0.40)
    parser.add_argument("--map-edge-margin-m", type=float, default=0.15)
    parser.add_argument("--start-free-search-m", type=float, default=0.35)
    parser.add_argument("--send-nav2-action", action="store_true")
    parser.add_argument("--physical-stuck-cmd-topic", default="/cmd_vel_guarded")
    parser.add_argument("--physical-stuck-odom-topic", default="/odom")
    parser.add_argument("--physical-stuck-cmd-linear-mps", type=float, default=0.10)
    parser.add_argument("--physical-stuck-cmd-angular-radps", type=float, default=0.20)
    parser.add_argument("--physical-stuck-odom-linear-mps", type=float, default=0.02)
    parser.add_argument("--physical-stuck-odom-angular-radps", type=float, default=0.05)
    parser.add_argument("--physical-stuck-s", type=float, default=2.0)
    parser.add_argument("--physical-stuck-cmd-timeout-s", type=float, default=0.60)
    parser.add_argument("--physical-stuck-odom-timeout-s", type=float, default=0.80)
    parser.add_argument("--physical-stuck-escape-cmd-topic", default="/cmd_vel_raw")
    parser.add_argument("--physical-stuck-escape-reverse-mps", type=float, default=0.14)
    parser.add_argument("--physical-stuck-escape-angular-radps", type=float, default=0.35)
    parser.add_argument("--physical-stuck-escape-reverse-s", type=float, default=0.80)
    parser.add_argument("--physical-stuck-escape-turn-s", type=float, default=0.80)
    parser.add_argument("--physical-stuck-escape-rate-hz", type=float, default=20.0)
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--report", default="")
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = RRTFrontierExplorer(args)
    try:
        code = node.run()
    except KeyboardInterrupt:
        print("RRT_STOP", json.dumps({"reason": "keyboard_interrupt"}), flush=True)
        code = 130
    except Exception as exc:
        if not is_rcl_context_shutdown_error(exc):
            raise
        print("RRT_STOP", json.dumps({"reason": "rcl_context_shutdown"}), flush=True)
        code = 130
    finally:
        try:
            node.destroy_node()
        except Exception as exc:
            if not is_rcl_context_shutdown_error(exc):
                raise
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(code)


if __name__ == "__main__":
    main()
