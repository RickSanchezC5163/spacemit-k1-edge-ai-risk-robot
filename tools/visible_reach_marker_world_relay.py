#!/usr/bin/env python3
"""Republish reach markers in RViz world frame with larger visible glyphs."""

import rclpy
import copy
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray


SRC_TOPIC = "/visual_reach_stress_markers"
DST_TOPIC = "/visual_reach_stress_markers_world"


class VisibleReachMarkerWorldRelay(Node):
    def __init__(self):
        super().__init__("visible_reach_marker_world_relay")
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.pub = self.create_publisher(MarkerArray, DST_TOPIC, qos)
        self.sub = self.create_subscription(MarkerArray, SRC_TOPIC, self.cb, qos)
        self.get_logger().info(f"republishing enlarged world-frame markers on {DST_TOPIC}")

    def cb(self, msg):
        source_markers = [
            marker
            for marker in msg.markers
            if marker.ns in ("reach_start", "reach_target", "reach_end", "reach_start_to_end")
        ]
        if not source_markers:
            return
        clear = Marker()
        clear.action = Marker.DELETEALL
        out = [clear]
        now = self.get_clock().now().to_msg()
        for marker in source_markers:
            if marker.ns in ("reach_start", "reach_target", "reach_end"):
                sphere = Marker()
                sphere.header.frame_id = "world"
                sphere.header.stamp = now
                sphere.ns = marker.ns + "_world_big"
                sphere.id = marker.id
                sphere.type = Marker.SPHERE
                sphere.action = Marker.ADD
                sphere.pose = copy.deepcopy(marker.pose)
                sphere.pose.orientation.w = 1.0
                sphere.scale.x = 0.035
                sphere.scale.y = 0.035
                sphere.scale.z = 0.035
                if marker.ns == "reach_start":
                    sphere.color.r = 0.0
                    sphere.color.g = 1.0
                    sphere.color.b = 0.0
                elif marker.ns == "reach_target":
                    sphere.color.r = 0.0
                    sphere.color.g = 0.35
                    sphere.color.b = 1.0
                else:
                    sphere.color.r = 1.0
                    sphere.color.g = 0.0
                    sphere.color.b = 0.0
                sphere.color.a = 1.0
                out.append(sphere)

                label = Marker()
                label.header.frame_id = "world"
                label.header.stamp = now
                label.ns = marker.ns + "_world_label"
                label.id = marker.id
                label.type = Marker.TEXT_VIEW_FACING
                label.action = Marker.ADD
                label.pose = copy.deepcopy(marker.pose)
                label.pose.position.z += 0.085
                label.pose.orientation.w = 1.0
                label.scale.z = 0.035
                label.color.r = 1.0
                label.color.g = 1.0
                label.color.b = 1.0
                label.color.a = 1.0
                if marker.ns == "reach_start":
                    label.text = "START"
                elif marker.ns == "reach_target":
                    label.text = "TARGET"
                else:
                    label.text = "END"
                out.append(label)
                continue

            line = Marker()
            line.header.frame_id = "world"
            line.header.stamp = now
            line.ns = "reach_start_to_end_world_big"
            line.id = marker.id
            line.type = Marker.LINE_STRIP
            line.action = Marker.ADD
            line.pose.orientation.w = 1.0
            line.scale.x = 0.010
            line.color.r = 1.0
            line.color.g = 1.0
            line.color.b = 0.0
            line.color.a = 1.0
            line.points = []
            for point in marker.points:
                shifted = Point()
                shifted.x = point.x
                shifted.y = point.y
                shifted.z = point.z
                line.points.append(shifted)
            out.append(line)

        if out:
            self.pub.publish(MarkerArray(markers=out))


def main():
    rclpy.init()
    node = VisibleReachMarkerWorldRelay()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
