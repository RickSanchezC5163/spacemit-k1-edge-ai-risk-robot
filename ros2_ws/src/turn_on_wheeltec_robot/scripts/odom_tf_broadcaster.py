#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class OdomTfBroadcaster(Node):
    def __init__(self):
        super().__init__("odom_tf_broadcaster")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("odom_frame", "")
        self.declare_parameter("base_frame", "")
        self.declare_parameter("publish_rate_hz", 30.0)
        self.declare_parameter("use_current_time", True)
        self.declare_parameter("publish_identity_until_odom", True)

        self.odom_topic = self.get_parameter("odom_topic").value
        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.publish_rate_hz = max(1.0, float(self.get_parameter("publish_rate_hz").value))
        self.use_current_time = bool(self.get_parameter("use_current_time").value)
        self.publish_identity_until_odom = bool(
            self.get_parameter("publish_identity_until_odom").value
        )
        self.latest_odom = None
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 20)
        self.create_timer(1.0 / self.publish_rate_hz, self.publish_latest)
        self.get_logger().info(
            f"Broadcasting TF from {self.odom_topic} at {self.publish_rate_hz:.1f}Hz"
        )

    def odom_callback(self, msg):
        self.latest_odom = msg
        self.publish_transform(msg)

    def publish_latest(self):
        if self.latest_odom is not None:
            self.publish_transform(self.latest_odom)
        elif self.publish_identity_until_odom and self.odom_frame and self.base_frame:
            self.publish_identity_transform()

    def publish_transform(self, msg):
        odom_frame = self.odom_frame or msg.header.frame_id
        base_frame = self.base_frame or msg.child_frame_id
        if not odom_frame or not base_frame:
            self.get_logger().warn("Skipping odom TF with empty frame id")
            return

        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg() if self.use_current_time else msg.header.stamp
        tf.header.frame_id = odom_frame
        tf.child_frame_id = base_frame
        tf.transform.translation.x = msg.pose.pose.position.x
        tf.transform.translation.y = msg.pose.pose.position.y
        tf.transform.translation.z = msg.pose.pose.position.z
        tf.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(tf)

    def publish_identity_transform(self):
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = self.odom_frame
        tf.child_frame_id = self.base_frame
        tf.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(tf)


def main():
    rclpy.init()
    node = OdomTfBroadcaster()
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
