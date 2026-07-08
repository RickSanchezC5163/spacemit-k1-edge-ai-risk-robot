#!/usr/bin/env python3
import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient


MAX_DISTANCE = 0.5
MAX_ABS_YAW = math.pi
DEFAULT_ZERO_SECONDS = 3.0
DEFAULT_ZERO_RATE = 50.0


def reject(message: str) -> None:
    print(f"REFUSE: {message}", file=sys.stderr)
    sys.exit(2)


def yaw_to_quaternion(yaw: float):
    half = yaw * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


def publish_zero(node, topic: str, duration: float, rate: float) -> None:
    pub = node.create_publisher(Twist, topic, 10)
    msg = Twist()
    end_time = time.monotonic() + max(0.1, duration)
    period = 1.0 / max(1.0, rate)
    while time.monotonic() < end_time:
        pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.0)
        time.sleep(period)
    pub.publish(msg)
    rclpy.spin_once(node, timeout_sec=0.05)


def spin_until_done_or_timeout(node, future, timeout_sec: float) -> bool:
    deadline = time.monotonic() + max(0.1, timeout_sec)
    while rclpy.ok() and not future.done() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    return future.done()


def main():
    parser = argparse.ArgumentParser(
        description="Send one guarded Nav2 NavigateToPose goal for supervised tests."
    )
    parser.add_argument("--x", type=float, default=0.25, help="goal x in map frame")
    parser.add_argument("--y", type=float, default=0.0, help="goal y in map frame")
    parser.add_argument("--yaw", type=float, default=0.0, help="goal yaw in radians")
    parser.add_argument("--frame", default="map")
    parser.add_argument("--server-timeout", type=float, default=8.0)
    parser.add_argument("--result-timeout", type=float, default=12.0)
    parser.add_argument("--cancel-timeout", type=float, default=3.0)
    parser.add_argument("--zero-topic", default="/cmd_vel_guarded")
    parser.add_argument("--zero-seconds", type=float, default=DEFAULT_ZERO_SECONDS)
    parser.add_argument("--zero-rate", type=float, default=DEFAULT_ZERO_RATE)
    args = parser.parse_args()

    distance = math.hypot(args.x, args.y)
    if distance > MAX_DISTANCE:
        reject(f"goal distance must be <= {MAX_DISTANCE:.2f}m, got {distance:.2f}m")
    if abs(args.yaw) > MAX_ABS_YAW:
        reject(f"abs(yaw) must be <= pi, got {args.yaw}")

    print("SAFETY WARNING")
    print("- This sends one Nav2 NavigateToPose goal.")
    print("- It cancels on timeout and then publishes zero Twist.")
    print("- A person must physically follow the robot and be ready to lift/disable it.")
    print("- Do not use this until /scan, /odom, TF, slam_toolbox, and Nav2 are healthy.")
    print(f"- Goal: frame={args.frame}, x={args.x:.3f}, y={args.y:.3f}, yaw={args.yaw:.3f}")
    print(f"- Result timeout: {args.result_timeout:.1f}s")
    print(f"- Forced zero: topic={args.zero_topic}, duration={args.zero_seconds:.1f}s")
    confirmation = input("Type YES to send the goal: ").strip()
    if confirmation != "YES":
        reject("confirmation was not YES")

    rclpy.init()
    node = rclpy.create_node("nav2_send_guarded_goal")
    client = ActionClient(node, NavigateToPose, "navigate_to_pose")

    if not client.wait_for_server(timeout_sec=args.server_timeout):
        node.destroy_node()
        rclpy.shutdown()
        reject("navigate_to_pose action server is not available")

    goal = NavigateToPose.Goal()
    goal.pose = PoseStamped()
    goal.pose.header.frame_id = args.frame
    goal.pose.header.stamp = node.get_clock().now().to_msg()
    goal.pose.pose.position.x = args.x
    goal.pose.pose.position.y = args.y
    qx, qy, qz, qw = yaw_to_quaternion(args.yaw)
    goal.pose.pose.orientation.x = qx
    goal.pose.pose.orientation.y = qy
    goal.pose.pose.orientation.z = qz
    goal.pose.pose.orientation.w = qw

    send_future = client.send_goal_async(goal)
    if not spin_until_done_or_timeout(node, send_future, args.server_timeout):
        node.destroy_node()
        rclpy.shutdown()
        reject("timed out while sending goal request")
    goal_handle = send_future.result()
    if goal_handle is None or not goal_handle.accepted:
        node.destroy_node()
        rclpy.shutdown()
        reject("goal was rejected")

    exit_code = 1
    try:
        node.get_logger().info("Goal accepted; waiting for result.")
        result_future = goal_handle.get_result_async()
        if not spin_until_done_or_timeout(node, result_future, args.result_timeout):
            node.get_logger().error("NavigateToPose result timeout; canceling goal.")
            cancel_future = goal_handle.cancel_goal_async()
            spin_until_done_or_timeout(node, cancel_future, args.cancel_timeout)
            exit_code = 3
        else:
            result = result_future.result()
            if result is None:
                node.get_logger().error("No NavigateToPose result received.")
                exit_code = 1
            else:
                node.get_logger().info(f"NavigateToPose finished with status {result.status}.")
                exit_code = 0 if result.status == 4 else 1
    except KeyboardInterrupt:
        node.get_logger().warn("Interrupted; canceling goal.")
        cancel_future = goal_handle.cancel_goal_async()
        spin_until_done_or_timeout(node, cancel_future, args.cancel_timeout)
        exit_code = 130
    finally:
        node.get_logger().info("Publishing forced zero command.")
        publish_zero(node, args.zero_topic, args.zero_seconds, args.zero_rate)
        node.destroy_node()
        rclpy.shutdown()

    sys.exit(exit_code)


if __name__ == "__main__":
    main()
