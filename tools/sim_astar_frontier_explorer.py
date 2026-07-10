#!/usr/bin/env python3
import argparse
import heapq
import json
import math
import time
from collections import deque
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener


def clamp(value, lo, hi):
    return max(lo, min(hi, value))


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def norm_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


class AStarFrontierExplorer(Node):
    def __init__(self, args):
        super().__init__("sim_astar_frontier_explorer")
        self.args = args
        self.map_msg = None
        self.status = {}
        self.goal_cell = None
        self.path = []
        self.replan_at = 0.0
        self.goal_count = 0
        self.records = []
        self.cmd_pub = self.create_publisher(Twist, args.cmd_topic, 10)
        self.create_subscription(OccupancyGrid, args.map_topic, self.map_cb, 10)
        self.create_subscription(String, args.status_topic, self.status_cb, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def map_cb(self, msg):
        self.map_msg = msg

    def status_cb(self, msg):
        try:
            self.status = json.loads(msg.data)
        except json.JSONDecodeError:
            self.status = {}

    def wait_ready(self):
        deadline = time.monotonic() + self.args.wait_ready_s
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.map_msg is not None and self.robot_pose() is not None:
                return True
        return False

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

    def idx(self, x, y):
        return y * self.map_msg.info.width + x

    def cell_value(self, cell):
        return self.map_msg.data[self.idx(cell[0], cell[1])]

    def neighbors4(self, cell):
        x, y = cell
        w = self.map_msg.info.width
        h = self.map_msg.info.height
        for nx, ny in ((x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)):
            if 0 <= nx < w and 0 <= ny < h:
                yield (nx, ny)

    def is_free(self, cell):
        value = self.cell_value(cell)
        return 0 <= value <= self.args.free_threshold

    def is_unknown(self, cell):
        return self.cell_value(cell) < 0

    def near_obstacle(self, cell, radius_cells):
        cx, cy = cell
        w = self.map_msg.info.width
        h = self.map_msg.info.height
        for y in range(max(0, cy - radius_cells), min(h, cy + radius_cells + 1)):
            for x in range(max(0, cx - radius_cells), min(w, cx + radius_cells + 1)):
                if self.map_msg.data[self.idx(x, y)] >= self.args.occupied_threshold:
                    return True
        return False

    def frontier_cells(self, start):
        obstacle_radius = max(1, int(self.args.inflation_m / self.map_msg.info.resolution))
        visited = {start}
        q = deque([start])
        frontiers = []
        max_cells = int(self.args.frontier_search_radius_m / self.map_msg.info.resolution)
        while q:
            cell = q.popleft()
            if abs(cell[0] - start[0]) + abs(cell[1] - start[1]) > max_cells:
                continue
            if (
                self.is_free(cell)
                and not self.near_obstacle(cell, obstacle_radius)
                and any(self.is_unknown(nb) for nb in self.neighbors4(cell))
            ):
                frontiers.append(cell)
            for nb in self.neighbors4(cell):
                if nb in visited or not self.is_free(nb) or self.near_obstacle(nb, obstacle_radius):
                    continue
                visited.add(nb)
                q.append(nb)
        return frontiers

    def astar(self, start, goal):
        def heuristic(a, b):
            return abs(a[0] - b[0]) + abs(a[1] - b[1])

        open_heap = [(0, start)]
        came_from = {}
        g = {start: 0}
        obstacle_radius = max(1, int(self.args.inflation_m / self.map_msg.info.resolution))
        while open_heap:
            _, current = heapq.heappop(open_heap)
            if current == goal:
                path = [current]
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                path.reverse()
                return path
            for nb in self.neighbors4(current):
                if not self.is_free(nb) or self.near_obstacle(nb, obstacle_radius):
                    continue
                ng = g[current] + 1
                if ng < g.get(nb, 10**9):
                    came_from[nb] = current
                    g[nb] = ng
                    heapq.heappush(open_heap, (ng + heuristic(nb, goal), nb))
        return []

    def choose_goal(self, start):
        frontiers = self.frontier_cells(start)
        if not frontiers:
            return None, []
        min_cells = int(self.args.min_goal_distance_m / self.map_msg.info.resolution)
        candidates = []
        for cell in frontiers:
            d = abs(cell[0] - start[0]) + abs(cell[1] - start[1])
            if d >= min_cells:
                candidates.append((d, cell))
        candidates.sort(reverse=True)
        for _, cell in candidates[: self.args.max_frontier_candidates]:
            path = self.astar(start, cell)
            if path:
                return cell, path
        return None, []

    def publish_zero(self):
        self.cmd_pub.publish(Twist())

    def step(self):
        pose = self.robot_pose()
        if self.map_msg is None or pose is None:
            self.publish_zero()
            return "not_ready"
        start = self.world_to_cell(pose[0], pose[1])
        if start is None:
            self.publish_zero()
            return "robot_outside_map"

        now = time.monotonic()
        if self.goal_cell is None or now >= self.replan_at:
            self.goal_cell, self.path = self.choose_goal(start)
            self.replan_at = now + self.args.replan_period_s
            if self.goal_cell is not None:
                self.goal_count += 1
                gx, gy = self.cell_to_world(self.goal_cell)
                record = {
                    "goal_count": self.goal_count,
                    "goal_cell": self.goal_cell,
                    "goal_xy": [round(gx, 3), round(gy, 3)],
                    "path_len": len(self.path),
                    "time_s": round(now - self.started, 2),
                }
                self.records.append(record)
                print("ASTAR_GOAL", json.dumps(record), flush=True)

        if self.goal_cell is None or not self.path:
            self.publish_zero()
            return "no_frontier"

        gx, gy = self.cell_to_world(self.goal_cell)
        distance_to_goal = math.hypot(gx - pose[0], gy - pose[1])
        if distance_to_goal <= self.args.goal_tolerance_m:
            self.goal_cell = None
            self.path = []
            self.publish_zero()
            return "goal_reached"

        target = self.path[min(len(self.path) - 1, self.args.lookahead_cells)]
        tx, ty = self.cell_to_world(target)
        heading = math.atan2(ty - pose[1], tx - pose[0])
        err = norm_angle(heading - pose[2])
        msg = Twist()

        front_p10 = self.status.get("front_p10_range_m")
        if front_p10 is not None and float(front_p10) < self.args.hard_stop_m:
            msg.angular.z = self.args.turn_speed
        elif abs(err) > self.args.turn_in_place_rad:
            msg.angular.z = clamp(1.4 * err, -self.args.turn_speed, self.args.turn_speed)
        else:
            msg.linear.x = self.args.linear_speed * max(0.2, 1.0 - abs(err))
            msg.angular.z = clamp(1.2 * err, -self.args.turn_speed, self.args.turn_speed)
        self.cmd_pub.publish(msg)
        return "driving"

    def run(self):
        self.started = time.monotonic()
        if not self.wait_ready():
            print("ASTAR_NOT_READY", flush=True)
            return 2
        print("ASTAR_READY", flush=True)
        rate = self.create_rate(self.args.rate_hz)
        end = self.started + self.args.runtime_s
        status = "running"
        while rclpy.ok() and time.monotonic() < end and self.goal_count < self.args.max_goals:
            status = self.step()
            rclpy.spin_once(self, timeout_sec=0.0)
            rate.sleep()
        self.publish_zero()
        summary = {
            "status": status,
            "goals": self.goal_count,
            "runtime_s": round(time.monotonic() - self.started, 2),
            "records": self.records,
        }
        if self.args.report:
            path = Path(self.args.report)
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(json.dumps(summary, indent=2) + "\n")
        print("ASTAR_SUMMARY", json.dumps(summary), flush=True)
        return 0


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--cmd-topic", default="/input_cmd_vel")
    parser.add_argument("--map-topic", default="/map")
    parser.add_argument("--status-topic", default="/safety/front_obstacle")
    parser.add_argument("--map-frame", default="map")
    parser.add_argument("--base-frame", default="base_footprint")
    parser.add_argument("--runtime-s", type=float, default=120.0)
    parser.add_argument("--max-goals", type=int, default=12)
    parser.add_argument("--rate-hz", type=float, default=10.0)
    parser.add_argument("--linear-speed", type=float, default=0.12)
    parser.add_argument("--turn-speed", type=float, default=0.45)
    parser.add_argument("--turn-in-place-rad", type=float, default=0.45)
    parser.add_argument("--goal-tolerance-m", type=float, default=0.18)
    parser.add_argument("--min-goal-distance-m", type=float, default=0.55)
    parser.add_argument("--frontier-search-radius-m", type=float, default=4.0)
    parser.add_argument("--inflation-m", type=float, default=0.18)
    parser.add_argument("--hard-stop-m", type=float, default=0.55)
    parser.add_argument("--lookahead-cells", type=int, default=8)
    parser.add_argument("--replan-period-s", type=float, default=2.0)
    parser.add_argument("--max-frontier-candidates", type=int, default=30)
    parser.add_argument("--free-threshold", type=int, default=20)
    parser.add_argument("--occupied-threshold", type=int, default=65)
    parser.add_argument("--wait-ready-s", type=float, default=20.0)
    parser.add_argument("--report", default="")
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = AStarFrontierExplorer(args)
    try:
        code = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(code)


if __name__ == "__main__":
    main()
