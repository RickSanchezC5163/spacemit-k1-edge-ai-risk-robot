#!/usr/bin/env python3
import json
import math
from dataclasses import dataclass

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


DEFAULT_RISK_POINTS = [
    {"id": "risk_leakage_01", "class": "leakage", "severity": "high", "x": 2.45, "y": 1.55, "z": 0.24},
    {"id": "risk_blockage_01", "class": "blockage", "severity": "medium", "x": -2.40, "y": -1.70, "z": 0.24},
    {"id": "risk_crack_01", "class": "crack", "severity": "medium", "x": -2.25, "y": 2.05, "z": 0.24},
]


@dataclass
class Pose2D:
    x: float
    y: float
    yaw: float


def yaw_from_quat(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def norm_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def as_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def color_for_class(risk_class: str):
    if risk_class == "leakage":
        return 0.95, 0.08, 0.05, 0.95
    if risk_class == "blockage":
        return 0.12, 0.32, 0.95, 0.95
    if risk_class == "crack":
        return 0.98, 0.72, 0.10, 0.95
    return 0.8, 0.8, 0.8, 0.95


class SimRiskMarkerDetector(Node):
    def __init__(self):
        super().__init__("sim_risk_marker_detector")
        self.declare_parameter("risk_points_json", json.dumps(DEFAULT_RISK_POINTS))
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("marker_topic", "/risk/sim_markers")
        self.declare_parameter("detection_topic", "/risk/sim_detections")
        self.declare_parameter("event_topic", "/risk/current_event")
        self.declare_parameter("detection_radius_m", 3.2)
        self.declare_parameter("camera_fov_deg", 115.0)
        self.declare_parameter("publish_all_as_detected", False)
        self.declare_parameter("publish_rate_hz", 2.0)

        self.map_frame = str(self.get_parameter("map_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.detection_radius_m = float(self.get_parameter("detection_radius_m").value)
        self.camera_fov_rad = math.radians(float(self.get_parameter("camera_fov_deg").value))
        self.publish_all_as_detected = as_bool(self.get_parameter("publish_all_as_detected").value)
        self.risk_points = self._load_risk_points()

        self.latest_odom = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.marker_pub = self.create_publisher(MarkerArray, str(self.get_parameter("marker_topic").value), 10)
        self.detection_pub = self.create_publisher(String, str(self.get_parameter("detection_topic").value), 10)
        self.event_pub = self.create_publisher(String, str(self.get_parameter("event_topic").value), 10)
        self.create_subscription(Odometry, str(self.get_parameter("odom_topic").value), self._odom_cb, 20)
        period = 1.0 / max(0.2, float(self.get_parameter("publish_rate_hz").value))
        self.create_timer(period, self._tick)
        self.get_logger().info(f"Loaded {len(self.risk_points)} simulated risk points.")

    def _load_risk_points(self):
        raw = str(self.get_parameter("risk_points_json").value)
        try:
            points = json.loads(raw)
        except json.JSONDecodeError as exc:
            self.get_logger().error(f"Invalid risk_points_json: {exc}; using defaults.")
            points = DEFAULT_RISK_POINTS
        out = []
        for idx, point in enumerate(points):
            try:
                out.append(
                    {
                        "id": str(point.get("id") or f"risk_{idx:02d}"),
                        "class": str(point.get("class") or "unknown"),
                        "severity": str(point.get("severity") or "medium"),
                        "x": float(point["x"]),
                        "y": float(point["y"]),
                        "z": float(point.get("z", 0.24)),
                    }
                )
            except (KeyError, TypeError, ValueError) as exc:
                self.get_logger().warn(f"Skipping invalid risk point {point}: {exc}")
        return out

    def _odom_cb(self, msg):
        pose = msg.pose.pose
        self.latest_odom = Pose2D(pose.position.x, pose.position.y, yaw_from_quat(pose.orientation))

    def _robot_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.03),
            )
            t = tf.transform.translation
            return Pose2D(float(t.x), float(t.y), yaw_from_quat(tf.transform.rotation))
        except TransformException:
            return self.latest_odom

    def _risk_visible(self, pose: Pose2D, point):
        dx = point["x"] - pose.x
        dy = point["y"] - pose.y
        distance = math.hypot(dx, dy)
        bearing = norm_angle(math.atan2(dy, dx) - pose.yaw)
        visible = distance <= self.detection_radius_m and abs(bearing) <= self.camera_fov_rad * 0.5
        return visible, distance, bearing

    def _make_card_marker(self, point, marker_id, detected):
        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "sim_risk_cards"
        marker.id = marker_id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = point["x"]
        marker.pose.position.y = point["y"]
        marker.pose.position.z = point["z"]
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.18 if detected else 0.12
        marker.scale.y = 0.18 if detected else 0.12
        marker.scale.z = 0.34
        r, g, b, a = color_for_class(point["class"])
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.color.a = a if detected else 0.45
        return marker

    def _make_text_marker(self, point, marker_id, detected):
        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "sim_risk_labels"
        marker.id = marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = point["x"]
        marker.pose.position.y = point["y"]
        marker.pose.position.z = point["z"] + 0.34
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.16
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0 if detected else 0.55
        marker.text = f"{point['class']}\\n{point['id']}"
        return marker

    def _tick(self):
        robot = self._robot_pose()
        detections = []
        markers = MarkerArray()
        for idx, point in enumerate(self.risk_points):
            if robot is None:
                visible, distance, bearing = False, None, None
            else:
                visible, distance, bearing = self._risk_visible(robot, point)
            detected = visible or self.publish_all_as_detected
            if detected:
                detections.append(
                    {
                        **point,
                        "distance_m": None if distance is None else round(distance, 3),
                        "bearing_rad": None if bearing is None else round(bearing, 3),
                        "confidence": 0.92 if visible else 0.70,
                        "source": "simulated_d435_yolo_marker",
                    }
                )
            markers.markers.append(self._make_card_marker(point, idx, detected))
            markers.markers.append(self._make_text_marker(point, idx + 100, detected))

        payload = {
            "frame_id": self.map_frame,
            "robot_pose": None if robot is None else {"x": robot.x, "y": robot.y, "yaw": robot.yaw},
            "detection_count": len(detections),
            "detections": detections,
        }
        self.marker_pub.publish(markers)
        self.detection_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))
        if detections:
            self.event_pub.publish(String(data=json.dumps(detections[0], ensure_ascii=False)))


def main():
    rclpy.init()
    node = SimRiskMarkerDetector()
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
