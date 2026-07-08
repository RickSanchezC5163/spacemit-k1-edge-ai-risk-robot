#!/usr/bin/env python3
import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient


ALLOWED_DISTANCES = (0.2, 0.3, 0.5)


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


class GuardedSmallGoal:
    def __init__(self, args):
        self.args = args
        self.node = rclpy.create_node("nav2_guarded_small_goal")
        self.client = ActionClient(self.node, NavigateToPose, "navigate_to_pose")
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

    def run(self) -> int:
        if not self.client.wait_for_server(timeout_sec=self.args.server_timeout):
            reject("navigate_to_pose action server is not available")

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self.args.frame
        goal.pose.header.stamp = self.node.get_clock().now().to_msg()
        goal.pose.pose.position.x = self.args.distance
        goal.pose.pose.position.y = 0.0
        goal.pose.pose.position.z = 0.0
        qx, qy, qz, qw = yaw_to_quaternion(0.0)
        goal.pose.pose.orientation.x = qx
        goal.pose.pose.orientation.y = qy
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw

        send_future = self.client.send_goal_async(goal)
        if not spin_until_done_or_timeout(self.node, send_future, self.args.server_timeout):
            reject("timed out while sending goal request")
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            reject("goal was rejected")

        exit_code = 1
        start_time = time.monotonic()
        try:
            self.node.get_logger().info(
                f"Goal accepted: {self.args.distance:.1f}m forward in {self.args.frame}"
            )
            result_future = goal_handle.get_result_async()
            if not spin_until_done_or_timeout(self.node, result_future, self.args.result_timeout):
                self.node.get_logger().error("Result timeout; canceling goal.")
                cancel_future = goal_handle.cancel_goal_async()
                spin_until_done_or_timeout(self.node, cancel_future, self.args.cancel_timeout)
                exit_code = 3
            else:
                result = result_future.result()
                status = None if result is None else result.status
                self.node.get_logger().info(f"NavigateToPose finished with status {status}.")
                exit_code = 0 if status == 4 else 1
        except KeyboardInterrupt:
            self.node.get_logger().warn("Interrupted; canceling goal.")
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
        print("NAV2_SMALL_GOAL_REPORT")
        print(f"raw_samples={len(raw)} raw_nonzero={len(raw_nonzero)} max_abs_raw_x={max_raw_x:.3f}")
        print(
            "guarded_samples=%d guarded_nonzero=%d max_abs_guarded_x=%.3f"
            % (len(guarded), len(guarded_nonzero), max_guarded_x)
        )

    def destroy(self):
        self.node.destroy_node()


def parse_args():
    parser = argparse.ArgumentParser(
        description="Send one guarded Nav2 small forward goal. Allowed distances: 0.2, 0.3, 0.5 m."
    )
    parser.add_argument("--distance", type=float, default=0.2)
    parser.add_argument("--frame", default="base_footprint")
    parser.add_argument("--server-timeout", type=float, default=10.0)
    parser.add_argument("--result-timeout", type=float, default=20.0)
    parser.add_argument("--cancel-timeout", type=float, default=3.0)
    parser.add_argument("--raw-cmd-topic", default="/cmd_vel_raw")
    parser.add_argument("--guarded-cmd-topic", default="/cmd_vel_guarded")
    parser.add_argument("--zero-seconds", type=float, default=5.0)
    parser.add_argument("--zero-rate", type=float, default=50.0)
    parser.add_argument("--confirm", default="")
    args = parser.parse_args()

    if not any(abs(args.distance - value) < 1e-6 for value in ALLOWED_DISTANCES):
        reject("distance must be exactly one of: 0.2, 0.3, 0.5")
    if args.distance > 0.5:
        reject("distance must be <= 0.5m")
    if args.frame not in ("base_footprint", "base_link", "map"):
        reject("frame must be base_footprint, base_link, or map")
    if args.confirm != "YES":
        reject("requires --confirm YES")
    return args


def main():
    args = parse_args()
    print("SAFETY WARNING")
    print("- This is a supervised Nav2 small-goal validation tool, not autonomous exploration.")
    print("- Allowed goal distances are only 0.2m, 0.3m, and 0.5m.")
    print("- Nav2 must publish /cmd_vel_raw, scan_safety_guard must publish /cmd_vel_guarded.")
    print("- A human must guard the robot and be ready to lift or power it off.")
    print(f"- Goal: {args.distance:.1f}m forward in frame {args.frame}")

    rclpy.init()
    runner = GuardedSmallGoal(args)
    try:
        code = runner.run()
    finally:
        runner.destroy()
        rclpy.shutdown()
    sys.exit(code)


if __name__ == "__main__":
    main()

