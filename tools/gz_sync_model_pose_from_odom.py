#!/usr/bin/env python3
import argparse
import math
import subprocess
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node


class GazeboPoseSync(Node):
    def __init__(self, args):
        super().__init__("gz_sync_model_pose_from_odom")
        self.args = args
        self.latest = None
        self.last_sync = 0.0
        self.last_log = 0.0
        self.create_subscription(Odometry, args.odom_topic, self.odom_cb, 10)
        self.create_timer(1.0 / args.rate_hz, self.tick)
        self.get_logger().info(
            f"Syncing {args.model_name} pose from {args.odom_topic} to {args.world_name}"
        )

    def odom_cb(self, msg):
        self.latest = msg

    def tick(self):
        if self.latest is None:
            return
        now = time.monotonic()
        if now - self.last_sync < 1.0 / self.args.rate_hz:
            return
        self.last_sync = now
        pose = self.latest.pose.pose
        yaw = math.atan2(
            2.0
            * (
                pose.orientation.w * pose.orientation.z
                + pose.orientation.x * pose.orientation.y
            ),
            1.0
            - 2.0
            * (
                pose.orientation.y * pose.orientation.y
                + pose.orientation.z * pose.orientation.z
            ),
        )
        cos_yaw = math.cos(self.args.offset_yaw)
        sin_yaw = math.sin(self.args.offset_yaw)
        world_x = (
            self.args.offset_x
            + cos_yaw * pose.position.x
            - sin_yaw * pose.position.y
        )
        world_y = (
            self.args.offset_y
            + sin_yaw * pose.position.x
            + cos_yaw * pose.position.y
        )
        world_yaw = self.args.offset_yaw + yaw
        world_qz = math.sin(world_yaw * 0.5)
        world_qw = math.cos(world_yaw * 0.5)
        req = (
            f'name: "{self.args.model_name}" '
            f'position {{ x: {world_x:.6f} y: {world_y:.6f} z: {self.args.z:.6f} }} '
            f'orientation {{ x: 0.000000000 y: 0.000000000 '
            f'z: {world_qz:.9f} w: {world_qw:.9f} }}'
        )
        result = subprocess.run(
            [
                "ign",
                "service",
                "-s",
                f"/world/{self.args.world_name}/set_pose",
                "--reqtype",
                "ignition.msgs.Pose",
                "--reptype",
                "ignition.msgs.Boolean",
                "--timeout",
                str(self.args.timeout_ms),
                "--req",
                req,
            ],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.PIPE,
            text=True,
            check=False,
        )
        if now - self.last_log > self.args.log_interval_s:
            self.last_log = now
            err = (result.stderr or "").strip().replace("\n", " ")
            if len(err) > 160:
                err = err[:157] + "..."
            self.get_logger().info(
                f"set_pose x={world_x:.3f} y={world_y:.3f} "
                f"yaw={math.degrees(world_yaw):.1f}deg rc={result.returncode} err={err}"
            )


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--odom-topic", default="/odom")
    parser.add_argument("--world-name", default="tracked_robot_world")
    parser.add_argument("--model-name", default="tracked_robot")
    parser.add_argument("--rate-hz", type=float, default=5.0)
    parser.add_argument("--timeout-ms", type=int, default=1200)
    parser.add_argument("--log-interval-s", type=float, default=3.0)
    parser.add_argument("--z", type=float, default=0.0)
    parser.add_argument("--offset-x", type=float, default=0.0)
    parser.add_argument("--offset-y", type=float, default=0.0)
    parser.add_argument("--offset-yaw", type=float, default=0.0)
    return parser.parse_args()


def main():
    rclpy.init()
    node = GazeboPoseSync(parse_args())
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
