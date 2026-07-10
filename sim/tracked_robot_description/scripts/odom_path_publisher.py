#!/usr/bin/env python3
import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node


class OdomPathPublisher(Node):
    def __init__(self):
        super().__init__("odom_path_publisher")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("path_topic", "/trajectory")
        self.declare_parameter("path_frame", "odom")
        self.declare_parameter("max_poses", 3000)
        self.declare_parameter("min_distance_m", 0.03)

        self.odom_topic = self.get_parameter("odom_topic").value
        self.path_topic = self.get_parameter("path_topic").value
        self.path_frame = self.get_parameter("path_frame").value
        self.max_poses = int(self.get_parameter("max_poses").value)
        self.min_distance_m = float(self.get_parameter("min_distance_m").value)

        self.path = Path()
        self.path.header.frame_id = self.path_frame
        self.last_xy = None
        self.publisher = self.create_publisher(Path, self.path_topic, 10)
        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 30)
        self.create_timer(0.5, self.republish_path)
        self.get_logger().info(
            f"Publishing odom trajectory {self.odom_topic} -> {self.path_topic}"
        )

    def odom_callback(self, msg):
        pose = msg.pose.pose
        xy = (pose.position.x, pose.position.y)
        if self.last_xy is not None:
            dx = xy[0] - self.last_xy[0]
            dy = xy[1] - self.last_xy[1]
            if math.hypot(dx, dy) < self.min_distance_m:
                return

        stamped = PoseStamped()
        stamped.header = msg.header
        stamped.header.frame_id = self.path_frame or msg.header.frame_id
        stamped.pose = pose
        self.path.header.stamp = msg.header.stamp
        self.path.header.frame_id = stamped.header.frame_id
        self.path.poses.append(stamped)
        if len(self.path.poses) > self.max_poses:
            self.path.poses = self.path.poses[-self.max_poses :]
        self.last_xy = xy
        self.publisher.publish(self.path)

    def republish_path(self):
        if self.path.poses:
            self.publisher.publish(self.path)


def main():
    rclpy.init()
    node = OdomPathPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
