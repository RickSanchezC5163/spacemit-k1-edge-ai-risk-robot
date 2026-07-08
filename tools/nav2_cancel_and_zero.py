#!/usr/bin/env python3
import argparse
import json
import time

import rclpy
from action_msgs.msg import GoalInfo
from action_msgs.srv import CancelGoal
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Twist
from std_msgs.msg import String


def main():
    parser = argparse.ArgumentParser(
        description="Cancel NavigateToPose goals, then continuously publish zero velocity."
    )
    parser.add_argument("--action-name", default="/navigate_to_pose")
    parser.add_argument("--raw-cmd-topic", default="/cmd_vel_raw")
    parser.add_argument("--guarded-cmd-topic", default="/cmd_vel_guarded")
    parser.add_argument("--stop-request-topic", default="/chassis/stop_request")
    parser.add_argument("--duration", type=float, default=6.0)
    parser.add_argument("--rate", type=float, default=50.0)
    parser.add_argument("--stop-request-bursts", type=int, default=20)
    args = parser.parse_args()

    rclpy.init()
    node = rclpy.create_node("nav2_cancel_and_zero")
    cancel_client = node.create_client(CancelGoal, f"{args.action_name}/_action/cancel_goal")
    raw_pub = node.create_publisher(Twist, args.raw_cmd_topic, 10)
    guarded_pub = node.create_publisher(Twist, args.guarded_cmd_topic, 10)
    stop_pub = node.create_publisher(String, args.stop_request_topic, 10)

    if cancel_client.wait_for_service(timeout_sec=3.0):
        request = CancelGoal.Request()
        request.goal_info = GoalInfo()
        request.goal_info.goal_id.uuid = [0] * 16
        request.goal_info.stamp = Time(sec=0, nanosec=0)
        future = cancel_client.call_async(request)
        deadline = time.monotonic() + 3.0
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if future.done():
            node.get_logger().info(f"Cancel response: {future.result()}")
        else:
            node.get_logger().warn("Cancel request timed out; continuing with zero output.")
    else:
        node.get_logger().warn("Cancel service unavailable; continuing with zero output.")

    zero = Twist()
    stop = String()
    stop.data = json.dumps(
        {
            "request": "STOP_REQUEST",
            "reason": "nav2_cancel_and_zero",
            "source": "nav2_cancel_and_zero",
        },
        ensure_ascii=False,
    )

    duration = max(0.2, args.duration)
    period = 1.0 / max(1.0, args.rate)
    end = time.monotonic() + duration
    bursts = max(0, args.stop_request_bursts)
    sent = 0
    node.get_logger().info(
        f"Publishing zero to {args.raw_cmd_topic} and {args.guarded_cmd_topic} for {duration:.1f}s"
    )
    while time.monotonic() < end:
        raw_pub.publish(zero)
        guarded_pub.publish(zero)
        if sent < bursts:
            stop_pub.publish(stop)
            sent += 1
        rclpy.spin_once(node, timeout_sec=0.0)
        time.sleep(period)
    raw_pub.publish(zero)
    guarded_pub.publish(zero)
    rclpy.spin_once(node, timeout_sec=0.05)
    node.get_logger().info("Cancel-and-zero complete.")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

