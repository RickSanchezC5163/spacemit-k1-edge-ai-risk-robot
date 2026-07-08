#!/usr/bin/env python3
import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateThroughPoses
from rclpy.action import ActionClient


ALLOWED_PATTERNS = {
    "line_2": [(0.20, 0.0), (0.35, 0.0)],
    "line_3": [(0.15, 0.0), (0.30, 0.0), (0.45, 0.0)],
    "micro_l": [(0.20, 0.0), (0.30, 0.12), (0.45, 0.12)],
}
MAX_ABS_X = 0.50
MAX_ABS_Y = 0.20


def reject(message: str) -> None:
    print(f"REFUSE: {message}", file=sys.stderr)
    sys.exit(2)


def yaw_to_quaternion(yaw: float):
    half = yaw * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


def spin_until_done_or_timeout(node, future, timeout_sec: float) -> bool:
    deadline = time.monotonic() + max(0.1, timeout_sec)
    while rclpy.ok() and not future.done() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    return future.done()


class GuardedThroughPoses:
    def __init__(self, args):
        self.args = args
        self.node = rclpy.create_node("nav2_guarded_through_poses")
        self.client = ActionClient(self.node, NavigateThroughPoses, "navigate_through_poses")
        self.raw_samples = []
        self.guarded_samples = []
        self.raw_sub = self.node.create_subscription(
            Twist, args.raw_cmd_topic, self._raw_cb, 20
        )
        self.guarded_sub = self.node.create_subscription(
            Twist, args.guarded_cmd_topic, self._guarded_cb, 20
        )
        self.raw_zero_pub = self.node.create_publisher(Twist, args.raw_cmd_topic, 10)
        self.guarded_zero_pub = self.node.create_publisher(Twist, args.guarded_cmd_topic, 10)

    def _raw_cb(self, msg: Twist):
        self.raw_samples.append((time.monotonic(), float(msg.linear.x), float(msg.angular.z)))

    def _guarded_cb(self, msg: Twist):
        self.guarded_samples.append((time.monotonic(), float(msg.linear.x), float(msg.angular.z)))

    def publish_zero(self, duration: float):
        msg = Twist()
        period = 1.0 / max(1.0, self.args.zero_rate)
        end = time.monotonic() + max(0.2, duration)
        while time.monotonic() < end:
            self.raw_zero_pub.publish(msg)
            self.guarded_zero_pub.publish(msg)
            rclpy.spin_once(self.node, timeout_sec=0.0)
            time.sleep(period)
        self.raw_zero_pub.publish(msg)
        self.guarded_zero_pub.publish(msg)
        rclpy.spin_once(self.node, timeout_sec=0.05)

    def make_pose(self, x: float, y: float, yaw: float) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = self.args.frame
        pose.header.stamp = self.node.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        qx, qy, qz, qw = yaw_to_quaternion(yaw)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return pose

    def run(self) -> int:
        if not self.client.wait_for_server(timeout_sec=self.args.server_timeout):
            reject("navigate_through_poses action server is not available")

        points = ALLOWED_PATTERNS[self.args.pattern]
        goal = NavigateThroughPoses.Goal()
        goal.poses = [self.make_pose(x, y, 0.0) for x, y in points]

        send_future = self.client.send_goal_async(goal)
        if not spin_until_done_or_timeout(self.node, send_future, self.args.server_timeout):
            reject("timed out while sending through-poses goal request")
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            reject("through-poses goal was rejected")

        exit_code = 1
        start_time = time.monotonic()
        try:
            self.node.get_logger().info(
                f"Through-poses goal accepted: pattern={self.args.pattern} frame={self.args.frame}"
            )
            result_future = goal_handle.get_result_async()
            if not spin_until_done_or_timeout(self.node, result_future, self.args.result_timeout):
                self.node.get_logger().error("Result timeout; canceling through-poses goal.")
                cancel_future = goal_handle.cancel_goal_async()
                spin_until_done_or_timeout(self.node, cancel_future, self.args.cancel_timeout)
                exit_code = 3
            else:
                result = result_future.result()
                status = None if result is None else result.status
                self.node.get_logger().info(f"NavigateThroughPoses finished with status {status}.")
                exit_code = 0 if status == 4 else 1
        except KeyboardInterrupt:
            self.node.get_logger().warn("Interrupted; canceling through-poses goal.")
            cancel_future = goal_handle.cancel_goal_async()
            spin_until_done_or_timeout(self.node, cancel_future, self.args.cancel_timeout)
            exit_code = 130
        finally:
            self.publish_zero(self.args.zero_seconds)
            self.print_report(start_time)
        return exit_code

    def print_report(self, start_time: float):
        raw = [s for s in self.raw_samples if s[0] >= start_time]
        guarded = [s for s in self.guarded_samples if s[0] >= start_time]
        raw_nonzero = [s for s in raw if abs(s[1]) > 1e-4 or abs(s[2]) > 1e-4]
        guarded_nonzero = [
            s for s in guarded if abs(s[1]) > 1e-4 or abs(s[2]) > 1e-4
        ]
        max_raw_x = max([abs(s[1]) for s in raw], default=0.0)
        max_guarded_x = max([abs(s[1]) for s in guarded], default=0.0)
        print("NAV2_THROUGH_POSES_REPORT")
        print(f"raw_samples={len(raw)} raw_nonzero={len(raw_nonzero)} max_abs_raw_x={max_raw_x:.3f}")
        print(
            "guarded_samples=%d guarded_nonzero=%d max_abs_guarded_x=%.3f"
            % (len(guarded), len(guarded_nonzero), max_guarded_x)
        )

    def destroy(self):
        self.node.destroy_node()


def parse_args():
    parser = argparse.ArgumentParser(
        description="Send one guarded Nav2 NavigateThroughPoses micro pattern."
    )
    parser.add_argument("--pattern", choices=sorted(ALLOWED_PATTERNS.keys()), default="line_2")
    parser.add_argument("--frame", default="base_footprint")
    parser.add_argument("--server-timeout", type=float, default=10.0)
    parser.add_argument("--result-timeout", type=float, default=30.0)
    parser.add_argument("--cancel-timeout", type=float, default=3.0)
    parser.add_argument("--raw-cmd-topic", default="/cmd_vel_raw")
    parser.add_argument("--guarded-cmd-topic", default="/cmd_vel_guarded")
    parser.add_argument("--zero-seconds", type=float, default=6.0)
    parser.add_argument("--zero-rate", type=float, default=50.0)
    parser.add_argument("--confirm", default="")
    args = parser.parse_args()

    if args.frame not in ("base_footprint", "base_link", "map"):
        reject("frame must be base_footprint, base_link, or map")
    for x, y in ALLOWED_PATTERNS[args.pattern]:
        if abs(x) > MAX_ABS_X or abs(y) > MAX_ABS_Y:
            reject("pattern exceeds micro-test bounds")
    if args.confirm != "YES":
        reject("requires --confirm YES")
    return args


def main():
    args = parse_args()
    print("SAFETY WARNING")
    print("- This is a supervised guarded Nav2 through-poses micro test.")
    print("- It is not RRT, exploration, or unattended navigation.")
    print("- Nav2 must publish /cmd_vel_raw, scan_safety_guard must publish /cmd_vel_guarded.")
    print("- A human must guard the robot and be ready to lift or power it off.")
    print(f"- Pattern: {args.pattern} in frame {args.frame}: {ALLOWED_PATTERNS[args.pattern]}")

    rclpy.init()
    runner = GuardedThroughPoses(args)
    try:
        code = runner.run()
    finally:
        runner.destroy()
        rclpy.shutdown()
    sys.exit(code)


if __name__ == "__main__":
    main()
