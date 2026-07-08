#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8


class ChassisSecurityKeepalive(Node):
    def __init__(self):
        super().__init__("chassis_security_keepalive")
        self.declare_parameter("enabled", True)
        self.declare_parameter("rate_hz", 1.0)
        self.enabled = bool(self.get_parameter("enabled").value)
        rate_hz = max(0.2, float(self.get_parameter("rate_hz").value))
        self.publisher = self.create_publisher(Int8, "/chassis_security", 10)
        self.message = Int8()
        self.message.data = 1 if self.enabled else 0
        self.create_timer(1.0 / rate_hz, self.publish_security)
        self.get_logger().info(
            f"Publishing /chassis_security={self.message.data} at {rate_hz:.2f}Hz"
        )

    def publish_security(self):
        self.publisher.publish(self.message)


def main():
    rclpy.init()
    node = ChassisSecurityKeepalive()
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
