#!/usr/bin/env python3
import json
import math
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker

try:
    from nav2_msgs.action import NavigateToPose
    from rclpy.action import ActionClient
except ImportError:
    NavigateToPose = None
    ActionClient = None


def yaw_from_quat(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def yaw_to_quat(yaw: float):
    half = yaw * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


def norm_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def as_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def sanitize_id(value: str) -> str:
    keep = []
    for char in value:
        if char.isalnum() or char in ("-", "_"):
            keep.append(char)
        else:
            keep.append("_")
    return "".join(keep) or "risk"


class RiskApproachGoalNode(Node):
    def __init__(self):
        super().__init__("risk_approach_goal_node")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("image_topic", "/camera/color/image_raw")
        self.declare_parameter("detection_topic", "/risk/sim_detections")
        self.declare_parameter("goal_topic", "/goal_pose")
        self.declare_parameter("status_topic", "/risk/approach_status")
        self.declare_parameter("goal_marker_topic", "/risk/approach_goal_marker")
        self.declare_parameter("output_dir", "/tmp/k1_sim_risk_approach")
        self.declare_parameter("stand_off_m", 0.65)
        self.declare_parameter("min_stand_off_m", 0.50)
        self.declare_parameter("max_stand_off_m", 0.80)
        self.declare_parameter("arrival_tolerance_m", 0.18)
        self.declare_parameter("settle_time_s", 1.2)
        self.declare_parameter("goal_cooldown_s", 8.0)
        self.declare_parameter("send_nav2_action", True)
        self.declare_parameter("nav2_server_timeout_s", 1.0)

        self.map_frame = str(self.get_parameter("map_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.stand_off_m = min(
            float(self.get_parameter("max_stand_off_m").value),
            max(float(self.get_parameter("min_stand_off_m").value), float(self.get_parameter("stand_off_m").value)),
        )
        self.arrival_tolerance_m = float(self.get_parameter("arrival_tolerance_m").value)
        self.settle_time_s = float(self.get_parameter("settle_time_s").value)
        self.goal_cooldown_s = float(self.get_parameter("goal_cooldown_s").value)
        self.send_nav2_action = as_bool(self.get_parameter("send_nav2_action").value)
        self.nav2_server_timeout_s = float(self.get_parameter("nav2_server_timeout_s").value)

        self.output_dir = Path(str(self.get_parameter("output_dir").value)).expanduser()
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.records_path = self.output_dir / "risk_approach_records.jsonl"

        self.latest_odom = None
        self.latest_image = None
        self.completed_ids = set()
        self.active = None
        self.last_goal_time = 0.0
        self.arrival_started_at = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.goal_pub = self.create_publisher(PoseStamped, str(self.get_parameter("goal_topic").value), 10)
        self.status_pub = self.create_publisher(String, str(self.get_parameter("status_topic").value), 10)
        self.marker_pub = self.create_publisher(Marker, str(self.get_parameter("goal_marker_topic").value), 10)
        self.create_subscription(Odometry, str(self.get_parameter("odom_topic").value), self._odom_cb, 20)
        self.create_subscription(Image, str(self.get_parameter("image_topic").value), self._image_cb, 5)
        self.create_subscription(String, str(self.get_parameter("detection_topic").value), self._detection_cb, 10)

        self.nav2_client = None
        if self.send_nav2_action and NavigateToPose is not None and ActionClient is not None:
            self.nav2_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        self.create_timer(0.25, self._tick)
        self.get_logger().info(
            f"Risk approach ready: stand_off={self.stand_off_m:.2f}m output={self.output_dir}"
        )

    def _odom_cb(self, msg):
        self.latest_odom = msg

    def _image_cb(self, msg):
        self.latest_image = msg

    def _robot_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.03),
            )
            t = tf.transform.translation
            yaw = yaw_from_quat(tf.transform.rotation)
            return float(t.x), float(t.y), yaw, "tf"
        except TransformException:
            if self.latest_odom is None:
                return None
            pose = self.latest_odom.pose.pose
            return pose.position.x, pose.position.y, yaw_from_quat(pose.orientation), "odom_fallback"

    def _detection_cb(self, msg):
        if self.active is not None:
            return
        if time.monotonic() - self.last_goal_time < self.goal_cooldown_s:
            return
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        detections = payload.get("detections") or []
        candidates = [d for d in detections if d.get("id") not in self.completed_ids]
        if not candidates:
            return
        candidates.sort(key=lambda item: float(item.get("distance_m") or 999.0))
        robot = self._robot_pose()
        if robot is None:
            return
        self._start_goal(candidates[0], robot)

    def _start_goal(self, risk, robot_pose):
        rx, ry, _, pose_source = robot_pose
        risk_x = float(risk["x"])
        risk_y = float(risk["y"])
        dx = rx - risk_x
        dy = ry - risk_y
        length = math.hypot(dx, dy)
        if length < 1e-3:
            dx, dy, length = -1.0, 0.0, 1.0
        ux = dx / length
        uy = dy / length
        goal_x = risk_x + ux * self.stand_off_m
        goal_y = risk_y + uy * self.stand_off_m
        goal_yaw = math.atan2(risk_y - goal_y, risk_x - goal_x)

        goal = PoseStamped()
        goal.header.frame_id = self.map_frame
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = goal_x
        goal.pose.position.y = goal_y
        qx, qy, qz, qw = yaw_to_quat(goal_yaw)
        goal.pose.orientation.x = qx
        goal.pose.orientation.y = qy
        goal.pose.orientation.z = qz
        goal.pose.orientation.w = qw

        self.goal_pub.publish(goal)
        self._send_nav2_goal(goal)
        self._publish_goal_marker(goal, risk)
        self.active = {
            "risk": risk,
            "goal": {"x": goal_x, "y": goal_y, "yaw": goal_yaw, "frame_id": self.map_frame},
            "started_at_monotonic": time.monotonic(),
            "pose_source": pose_source,
        }
        self.last_goal_time = time.monotonic()
        self.arrival_started_at = None
        self._publish_status("goal_sent", self.active)
        self.get_logger().info(
            f"Sent approach goal for {risk['id']} at x={goal_x:.2f} y={goal_y:.2f} yaw={goal_yaw:.2f}"
        )

    def _send_nav2_goal(self, goal):
        if self.nav2_client is None:
            return
        if not self.nav2_client.wait_for_server(timeout_sec=self.nav2_server_timeout_s):
            self.get_logger().warn("navigate_to_pose action server unavailable; published /goal_pose only.")
            return
        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = goal
        future = self.nav2_client.send_goal_async(nav_goal)
        future.add_done_callback(self._nav2_goal_response_cb)

    def _nav2_goal_response_cb(self, future):
        try:
            handle = future.result()
        except Exception as exc:
            self.get_logger().warn(f"Nav2 goal send failed: {exc}")
            return
        if handle is None or not handle.accepted:
            self.get_logger().warn("Nav2 rejected risk approach goal.")
            return
        self.get_logger().info("Nav2 accepted risk approach goal.")

    def _publish_goal_marker(self, goal, risk):
        marker = Marker()
        marker.header = goal.header
        marker.ns = "risk_approach_goal"
        marker.id = 1
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose = goal.pose
        marker.scale.x = 0.45
        marker.scale.y = 0.06
        marker.scale.z = 0.06
        marker.color.r = 0.1
        marker.color.g = 1.0
        marker.color.b = 0.35
        marker.color.a = 0.95
        self.marker_pub.publish(marker)

    def _publish_status(self, state, detail=None):
        payload = {
            "state": state,
            "active_risk_id": None if self.active is None else self.active["risk"].get("id"),
            "completed_ids": sorted(self.completed_ids),
            "detail": detail,
        }
        self.status_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))

    def _tick(self):
        if self.active is None:
            self._publish_status("idle")
            return
        robot = self._robot_pose()
        if robot is None:
            self._publish_status("waiting_robot_pose")
            return
        x, y, yaw, pose_source = robot
        goal = self.active["goal"]
        risk = self.active["risk"]
        goal_distance = math.hypot(goal["x"] - x, goal["y"] - y)
        risk_distance = math.hypot(float(risk["x"]) - x, float(risk["y"]) - y)
        yaw_to_risk = math.atan2(float(risk["y"]) - y, float(risk["x"]) - x)
        yaw_error = abs(norm_angle(yaw_to_risk - yaw))
        if goal_distance <= self.arrival_tolerance_m:
            if self.arrival_started_at is None:
                self.arrival_started_at = time.monotonic()
            elif time.monotonic() - self.arrival_started_at >= self.settle_time_s:
                self._record_arrival(x, y, yaw, pose_source, goal_distance, risk_distance, yaw_error)
        else:
            self.arrival_started_at = None
        self._publish_status(
            "approaching",
            {
                "goal_distance_m": round(goal_distance, 3),
                "risk_distance_m": round(risk_distance, 3),
                "yaw_error_rad": round(yaw_error, 3),
            },
        )

    def _record_arrival(self, x, y, yaw, pose_source, goal_distance, risk_distance, yaw_error):
        risk = self.active["risk"]
        stamp = time.strftime("%Y%m%d_%H%M%S")
        safe_id = sanitize_id(str(risk.get("id", "risk")))
        image_path = self._write_latest_image(stamp, safe_id)
        record = {
            "recorded_at": stamp,
            "risk": risk,
            "goal": self.active["goal"],
            "robot_pose": {"x": x, "y": y, "yaw": yaw, "frame_id": self.map_frame, "source": pose_source},
            "goal_distance_m": round(goal_distance, 3),
            "risk_distance_m": round(risk_distance, 3),
            "yaw_error_rad": round(yaw_error, 3),
            "image_path": str(image_path) if image_path else None,
            "stand_off_target_m": self.stand_off_m,
            "published_cmd_vel": False,
            "motion_source": "Nav2 NavigateToPose or /goal_pose; this node does not publish Twist",
        }
        with self.records_path.open("a", encoding="utf-8") as handle:
            handle.write(json.dumps(record, ensure_ascii=False) + "\n")
        (self.output_dir / "latest_risk_approach.json").write_text(
            json.dumps(record, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )
        self.completed_ids.add(risk["id"])
        self.get_logger().info(
            f"Recorded risk approach for {risk['id']}: risk_distance={risk_distance:.2f}m image={image_path}"
        )
        self.active = None
        self.arrival_started_at = None
        self._publish_status("arrived_recorded", record)

    def _write_latest_image(self, stamp, risk_id):
        msg = self.latest_image
        if msg is None or not msg.data:
            return None
        encoding = (msg.encoding or "").lower()
        if encoding not in ("rgb8", "bgr8", "rgba8", "bgra8"):
            self.get_logger().warn(f"Unsupported image encoding for PPM snapshot: {msg.encoding}")
            return None
        channels = 4 if encoding in ("rgba8", "bgra8") else 3
        row_step = int(msg.step)
        width = int(msg.width)
        height = int(msg.height)
        raw = bytes(msg.data)
        rows = []
        for row in range(height):
            src = raw[row * row_step : row * row_step + width * channels]
            if encoding in ("rgb8", "rgba8"):
                rgb = b"".join(src[i : i + 3] for i in range(0, len(src), channels))
            else:
                rgb = b"".join(bytes((src[i + 2], src[i + 1], src[i])) for i in range(0, len(src), channels))
            rows.append(rgb)
        path = self.output_dir / f"{stamp}_{risk_id}_camera.ppm"
        with path.open("wb") as handle:
            handle.write(f"P6\n{width} {height}\n255\n".encode("ascii"))
            for row in rows:
                handle.write(row)
        return path


def main():
    rclpy.init()
    node = RiskApproachGoalNode()
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
