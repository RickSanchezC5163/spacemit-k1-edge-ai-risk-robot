#!/usr/bin/env python3
import argparse
import json
import math
import os
import subprocess
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

from nav2_msgs.action import NavigateToPose


ROOT = Path(__file__).resolve().parents[1]


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def yaw_to_quat(yaw):
    half = yaw * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


def write_jsonl(path: Path, item):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a", encoding="utf-8") as handle:
        handle.write(json.dumps(item, ensure_ascii=False) + "\n")


class RealK1RiskApproach(Node):
    def __init__(self, args):
        super().__init__("real_k1_risk_approach_from_event")
        self.args = args
        self.output_dir = Path(args.output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.records_path = self.output_dir / "risk_approach_records.jsonl"
        self.confirmed_map_points_path = self.output_dir / "confirmed_risk_map_points.json"
        self.confirmed_map_points_jsonl_path = self.output_dir / "confirmed_risk_map_points.jsonl"
        self.confirmed_map_points = []
        self.latest_odom = None
        self.completed_ids = set()
        self.active = False
        self.event_count = 0
        self.started_at = time.monotonic()

        self.status_pub = self.create_publisher(String, args.status_topic, 10)
        self.creep_cmd_pub = self.create_publisher(Twist, args.final_creep_cmd_topic, 10)
        self.create_subscription(Odometry, args.odom_topic, self.odom_cb, 20)
        self.create_subscription(String, args.event_topic, self.event_cb, 10)
        self.nav2_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.get_logger().info(
            f"risk approach ready: event={args.event_topic} stand_off={args.stand_off_m:.2f}m"
        )

    def odom_cb(self, msg):
        self.latest_odom = msg

    def publish_status(self, state, payload):
        out = dict(payload)
        out["state"] = state
        out["stamp_s"] = round(time.monotonic() - self.started_at, 3)
        msg = String()
        msg.data = json.dumps(out, ensure_ascii=False)
        self.status_pub.publish(msg)
        write_jsonl(self.records_path, out)

    def event_cb(self, msg):
        if self.active or self.event_count >= self.args.max_events:
            return
        try:
            event = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        event_id = str(event.get("event_id") or "")
        if not event_id or event_id in self.completed_ids:
            return
        if float(event.get("confidence") or 0.0) < self.args.min_confidence:
            return
        odom_xy = event.get("odom_point_xy_m") or {}
        if not isinstance(odom_xy, dict) or odom_xy.get("x") is None or odom_xy.get("y") is None:
            self.publish_status("skipped_no_odom_point", {"event": event})
            self.completed_ids.add(event_id)
            return
        if self.latest_odom is None:
            self.publish_status("skipped_no_odom", {"event": event})
            return

        risk_x = float(odom_xy["x"])
        risk_y = float(odom_xy["y"])
        pose = self.latest_odom.pose.pose
        robot_x = float(pose.position.x)
        robot_y = float(pose.position.y)
        dx = risk_x - robot_x
        dy = risk_y - robot_y
        distance = math.hypot(dx, dy)
        base_payload = {
            "event": event,
            "robot_odom_xy_m": {"x": round(robot_x, 4), "y": round(robot_y, 4)},
            "risk_odom_xy_m": {"x": round(risk_x, 4), "y": round(risk_y, 4)},
            "distance_to_risk_m": round(distance, 4),
        }
        class_name = str(event.get("class_name") or "")
        if class_name != "blockage":
            self.handle_passive_surface_risk(event_id, event, base_payload, distance)
            return

        if self.args.interrupt_rrt_on_event:
            self.interrupt_rrt(event_id)

        if distance <= self.args.stand_off_m + self.args.already_near_margin_m:
            self.completed_ids.add(event_id)
            self.event_count += 1
            payload = dict(base_payload)
            payload["nav_status"] = "already_near"
            self.publish_status("already_near", payload)
            arm_payload = self.complete_arm_semantic_and_resume(payload, run_close_confirm=True)
            self.maybe_write_confirmed_map_point(arm_payload, payload["nav_status"])
            return

        ux = dx / max(distance, 1e-6)
        uy = dy / max(distance, 1e-6)
        goal_x = risk_x - ux * self.args.stand_off_m
        goal_y = risk_y - uy * self.args.stand_off_m
        goal_yaw = math.atan2(risk_y - goal_y, risk_x - goal_x)

        goal = PoseStamped()
        goal.header.frame_id = self.args.goal_frame
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = goal_x
        goal.pose.position.y = goal_y
        qx, qy, qz, qw = yaw_to_quat(goal_yaw)
        goal.pose.orientation.x = qx
        goal.pose.orientation.y = qy
        goal.pose.orientation.z = qz
        goal.pose.orientation.w = qw

        payload = dict(base_payload)
        payload["approach_goal"] = {
            "frame_id": self.args.goal_frame,
            "x": round(goal_x, 4),
            "y": round(goal_y, 4),
            "yaw": round(goal_yaw, 4),
        }
        if not self.nav2_client.wait_for_server(timeout_sec=self.args.nav2_wait_s):
            self.completed_ids.add(event_id)
            self.event_count += 1
            self.publish_status("nav2_unavailable", payload)
            return

        self.active = True
        self.completed_ids.add(event_id)
        self.event_count += 1
        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = goal
        self.publish_status("approach_goal_sent", payload)

    def handle_passive_surface_risk(self, event_id, event, base_payload, odom_distance):
        self.completed_ids.add(event_id)
        self.event_count += 1
        payload = dict(base_payload)
        payload["passive_policy"] = {
            "class_policy": "record_only_until_natural_close",
            "rrt_interrupted": False,
            "nav2_approach_requested": False,
            "close_confirm_distance_m": self.args.passive_close_confirm_distance_m,
        }
        distances = []
        try:
            distances.append(float(odom_distance))
        except (TypeError, ValueError):
            pass
        try:
            distances.append(float(event.get("distance_m")))
        except (TypeError, ValueError):
            pass
        nearest = min(distances) if distances else None
        payload["passive_nearest_distance_m"] = None if nearest is None else round(nearest, 4)
        if nearest is None or nearest > self.args.passive_close_confirm_distance_m:
            payload["nav_status"] = "passive_candidate_recorded"
            payload["close_usb_confirm"] = {
                "capture_ok": False,
                "inference_ok": False,
                "close_confirmed_for_mapping": False,
                "close_confirm_reason": "passive_candidate_not_close_enough",
            }
            self.publish_status("passive_candidate_recorded", payload)
            return

        payload["nav_status"] = "passive_natural_close"
        self.publish_status("passive_natural_close", payload)
        close_confirm = self.run_close_usb_confirm(event)
        if close_confirm:
            payload["close_usb_confirm"] = close_confirm
            self.publish_status("close_usb_confirm_complete", payload)
        self.maybe_write_confirmed_map_point(payload, payload["nav_status"])
        return

    def goal_response_cb(self, future, payload):
        try:
            handle = future.result()
        except Exception as exc:
            self.active = False
            payload = dict(payload)
            payload["error"] = str(exc)
            self.publish_status("goal_send_error", payload)
            return
        if handle is None or not handle.accepted:
            self.active = False
            self.publish_status("goal_rejected", payload)
            return
        self.publish_status("goal_accepted", payload)
        result_future = handle.get_result_async()
        result_future.add_done_callback(lambda fut: self.result_cb(fut, payload))

    def interrupt_rrt(self, event_id):
        try:
            result = subprocess.run(
                ["pkill", "-INT", "-f", "sim_rrt_frontier_explorer.py"],
                check=False,
                capture_output=True,
                text=True,
                timeout=1.0,
            )
            self.publish_status(
                "rrt_interrupted_for_risk",
                {
                    "event_id": event_id,
                    "returncode": result.returncode,
                },
            )
        except Exception as exc:
            self.publish_status(
                "rrt_interrupt_failed",
                {
                    "event_id": event_id,
                    "error": str(exc),
                },
            )

    def result_cb(self, future, payload):
        self.active = False
        try:
            result = future.result()
            status = getattr(result, "status", None)
        except Exception as exc:
            payload = dict(payload)
            payload["error"] = str(exc)
            self.publish_status("result_error", payload)
            return
        payload = dict(payload)
        payload["nav_status"] = status
        if self.nav_succeeded(status):
            self.final_creep(payload)
        self.publish_status("approach_complete", payload)
        arm_payload = self.complete_arm_semantic_and_resume(payload, run_close_confirm=self.nav_succeeded(status))
        self.maybe_write_confirmed_map_point(arm_payload, status)

    @staticmethod
    def nav_succeeded(status):
        try:
            return int(status) == 4
        except (TypeError, ValueError):
            return False

    def final_creep(self, payload):
        creep_distance = max(0.0, self.args.stand_off_m - self.args.final_stand_off_m)
        if creep_distance <= 1e-3 or self.args.final_creep_linear_x <= 0.0:
            payload["final_creep"] = {"executed": False, "reason": "zero_distance_or_speed"}
            return
        duration_s = min(
            self.args.final_creep_max_s,
            creep_distance / max(self.args.final_creep_linear_x, 1e-6),
        )
        cmd = Twist()
        cmd.linear.x = float(self.args.final_creep_linear_x)
        payload["final_creep"] = {
            "executed": True,
            "cmd_topic": self.args.final_creep_cmd_topic,
            "target_delta_m": round(creep_distance, 3),
            "linear_x": round(float(self.args.final_creep_linear_x), 3),
            "duration_s": round(duration_s, 3),
            "final_stand_off_m": round(float(self.args.final_stand_off_m), 3),
        }
        self.publish_status("final_creep_started", payload)
        deadline = time.monotonic() + duration_s
        while rclpy.ok() and time.monotonic() < deadline:
            self.creep_cmd_pub.publish(cmd)
            time.sleep(0.05)
        stop = Twist()
        for _ in range(6):
            self.creep_cmd_pub.publish(stop)
            time.sleep(0.05)
        self.publish_status("final_creep_complete", payload)

    def complete_arm_semantic_and_resume(self, payload, run_close_confirm):
        event = payload.get("event") or {}
        semantic = (
            event.get("arm_operation_semantic")
            or ("blockage_response_candidate" if event.get("class_name") == "blockage" else "none")
        )
        arm_payload = dict(payload)
        if run_close_confirm:
            close_confirm = self.run_close_usb_confirm(event)
            if close_confirm:
                arm_payload["close_usb_confirm"] = close_confirm
                self.publish_status("close_usb_confirm_complete", arm_payload)
        else:
            arm_payload["close_usb_confirm"] = {
                "capture_ok": False,
                "inference_ok": False,
                "close_confirmed_for_mapping": False,
                "close_confirm_reason": "skipped_because_nav2_approach_not_successful",
            }
        arm_payload["arm_operation_semantic"] = semantic
        arm_payload["arm_result"] = {
            "mode": self.args.arm_simulation_mode,
            "duration_s": self.args.arm_simulation_s,
            "executed_hardware": False,
        }
        if self.args.arm_simulation_s > 0:
            self.publish_status("arm_semantic_switch_started", arm_payload)
            time.sleep(self.args.arm_simulation_s)
        self.publish_status("arm_semantic_switch_complete", arm_payload)
        if self.args.resume_rrt_after_event:
            self.resume_rrt(event.get("event_id"))
        return arm_payload

    def run_close_usb_confirm(self, event):
        device = str(self.args.close_confirm_usb_device or "")
        if not device:
            return None
        confirm_dir = self.output_dir / f"close_usb_confirm_{event.get('event_id') or int(time.time())}"
        confirm_dir.mkdir(parents=True, exist_ok=True)
        image_path = confirm_dir / "usb_frame.png"
        capture_script = f"""
import cv2
import sys
cap = cv2.VideoCapture({device!r}, cv2.CAP_V4L2)
if not cap.isOpened():
    raise SystemExit('failed_to_open_usb_camera')
cap.set(cv2.CAP_PROP_FRAME_WIDTH, {int(self.args.close_confirm_width)})
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, {int(self.args.close_confirm_height)})
for _ in range({int(self.args.close_confirm_warmup_frames)}):
    cap.read()
ok, frame = cap.read()
cap.release()
if not ok or frame is None:
    raise SystemExit('failed_to_read_usb_frame')
if not cv2.imwrite({str(image_path)!r}, frame):
    raise SystemExit('failed_to_write_usb_frame')
"""
        result = {
            "device": device,
            "output_dir": str(confirm_dir),
            "image_path": str(image_path),
            "model": self.args.close_confirm_model,
            "capture_ok": False,
            "inference_ok": False,
        }
        try:
            capture = subprocess.run(
                ["python3", "-c", capture_script],
                cwd=str(ROOT),
                check=False,
                capture_output=True,
                text=True,
                timeout=self.args.close_confirm_capture_timeout_s,
            )
            result["capture_returncode"] = capture.returncode
            result["capture_stdout"] = capture.stdout.strip()
            result["capture_stderr"] = capture.stderr.strip()
            result["capture_ok"] = capture.returncode == 0 and image_path.exists()
            if not result["capture_ok"]:
                return result

            inference = subprocess.run(
                [
                    "python3",
                    "tools/run_yolo_inference_once.py",
                    "--image",
                    str(image_path),
                    "--model",
                    self.args.close_confirm_model,
                    "--output-dir",
                    str(confirm_dir / "yolo"),
                    "--imgsz",
                    str(self.args.close_confirm_imgsz),
                    "--conf",
                    str(self.args.close_confirm_conf),
                    "--iou",
                    str(self.args.close_confirm_iou),
                    "--max-det",
                    str(self.args.close_confirm_max_det),
                    "--providers",
                    self.args.close_confirm_providers,
                ],
                cwd=str(ROOT),
                check=False,
                capture_output=True,
                text=True,
                timeout=self.args.close_confirm_infer_timeout_s,
            )
            result["inference_returncode"] = inference.returncode
            result["inference_stdout"] = inference.stdout.strip()
            result["inference_stderr"] = inference.stderr.strip()
            result["inference_ok"] = inference.returncode == 0
            result["risk_detection_json"] = str(confirm_dir / "yolo" / "risk_detection.json")
            self.add_close_confirm_detection_summary(result, event)
            return result
        except Exception as exc:
            result["error"] = str(exc)
            return result

    def add_close_confirm_detection_summary(self, result, event):
        detection_json = Path(result.get("risk_detection_json") or "")
        result["close_confirm_threshold"] = self.args.confirm_map_confidence
        result["close_confirmed_for_mapping"] = False
        result["close_confirm_reason"] = "missing_detection_json"
        result["close_confirm_max_confidence"] = 0.0
        if not detection_json.exists():
            return
        try:
            data = json.loads(detection_json.read_text(encoding="utf-8"))
        except Exception as exc:
            result["close_confirm_reason"] = f"invalid_detection_json:{exc}"
            return
        detections = data.get("detections") or []
        if not isinstance(detections, list) or not detections:
            result["close_confirm_reason"] = "no_close_usb_detection"
            result["close_confirm_detections"] = []
            return
        event_class = str(event.get("class_name") or "")
        risk_classes = {"crack", "corrosion", "leakage", "blockage"}
        compact = []
        for det in detections:
            if not isinstance(det, dict):
                continue
            class_name = str(det.get("class_name") or det.get("label") or "")
            try:
                confidence = float(det.get("confidence") or 0.0)
            except (TypeError, ValueError):
                confidence = 0.0
            compact.append(
                {
                    "class_name": class_name,
                    "confidence": round(confidence, 4),
                    "bbox_xywh": det.get("bbox_xywh"),
                }
            )
        result["close_confirm_detections"] = compact
        same_class = [det for det in compact if event_class and det.get("class_name") == event_class]
        risk_class = [det for det in compact if det.get("class_name") in risk_classes]
        candidates = same_class or risk_class or compact
        best = max(candidates, key=lambda det: float(det.get("confidence") or 0.0), default=None)
        if not best:
            result["close_confirm_reason"] = "no_valid_close_usb_detection"
            return
        confidence = float(best.get("confidence") or 0.0)
        result["close_confirm_best_detection"] = best
        result["close_confirm_max_confidence"] = round(confidence, 4)
        result["close_confirm_class_match"] = bool(same_class)
        if confidence >= self.args.confirm_map_confidence:
            result["close_confirmed_for_mapping"] = True
            result["close_confirm_reason"] = "passed_close_usb_confidence"
        else:
            result["close_confirm_reason"] = "close_usb_confidence_below_confirm_gate"

    def resume_rrt(self, event_id):
        env = os.environ.copy()
        env.update(
            {
                "RRT_RUNTIME_S": str(self.args.resume_rrt_runtime_s),
                "RRT_MAX_GOALS": str(self.args.resume_rrt_max_goals),
                "RRT_MIN_GOAL_CLEARANCE_M": str(self.args.resume_rrt_min_goal_clearance_m),
                "RRT_MAP_EDGE_MARGIN_M": str(self.args.resume_rrt_map_edge_margin_m),
                "RRT_MIN_GOAL_DISTANCE_M": str(self.args.resume_rrt_min_goal_distance_m),
                "RRT_FREE_ROAM_MIN_DISTANCE_M": str(self.args.resume_rrt_free_roam_min_distance_m),
                "RRT_START_FREE_SEARCH_M": str(self.args.resume_rrt_start_free_search_m),
                "RRT_REPLAN_SLEEP_S": str(self.args.resume_rrt_replan_sleep_s),
            }
        )
        script = f"""
set -e
cd {str(ROOT)!r}
RUN=$(cat .current_real_k1_rrt_nav2_run_dir)
if ps -eo pid=,args= | awk '/sim_rrt_frontier_explorer.py/ && $0 !~ /bash -c/ && $0 !~ /awk/ {{found=1}} END {{exit found ? 0 : 1}}'; then
  echo "already_running $RUN"
  exit 0
fi
LOG="$RUN/rrt_resume_after_risk_$(date +%H%M%S).log"
nohup bash tools/start_real_k1_rrt_nav2_mapping.sh rrt-run-2m-unlimited "$RUN" > "$LOG" 2>&1 < /dev/null &
echo $! > "$RUN/rrt_resume_after_risk.pid"
echo "$LOG"
"""
        try:
            result = subprocess.run(
                ["bash", "-lc", script],
                cwd=str(ROOT),
                env=env,
                check=False,
                capture_output=True,
                text=True,
                timeout=3.0,
            )
            self.publish_status(
                "rrt_resume_requested",
                {
                    "event_id": event_id,
                    "returncode": result.returncode,
                    "stdout": result.stdout.strip(),
                    "stderr": result.stderr.strip(),
                },
            )
        except Exception as exc:
            self.publish_status(
                "rrt_resume_failed",
                {
                    "event_id": event_id,
                    "error": str(exc),
                },
            )

    def maybe_write_confirmed_map_point(self, payload, nav_status):
        event = payload.get("event") or {}
        if (
            nav_status not in ("already_near", "passive_natural_close")
            and not self.nav_succeeded(nav_status)
        ):
            payload["confirmed_risk_map_point_skipped"] = {
                "reason": "nav2_approach_not_successful",
                "nav_status": nav_status,
            }
            self.publish_status("confirmed_map_point_skipped", payload)
            return
        close_confirm = payload.get("close_usb_confirm") or {}
        if not close_confirm.get("close_confirmed_for_mapping"):
            payload["confirmed_risk_map_point_skipped"] = {
                "reason": close_confirm.get("close_confirm_reason") or "missing_close_usb_confirmation",
                "confirm_map_confidence": self.args.confirm_map_confidence,
            }
            self.publish_status("confirmed_map_point_skipped", payload)
            return
        best_detection = close_confirm.get("close_confirm_best_detection") or {}
        confidence = float(best_detection.get("confidence") or close_confirm.get("close_confirm_max_confidence") or 0.0)
        odom_xy = event.get("odom_point_xy_m") or {}
        if not isinstance(odom_xy, dict) or odom_xy.get("x") is None or odom_xy.get("y") is None:
            return
        class_name = str(best_detection.get("class_name") or event.get("class_name") or "risk")
        confirmed = {
            "schema_version": "confirmed_risk_map_point_v1",
            "confirmed_at": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
            "source": "real_k1_risk_approach_from_event",
            "event_id": event.get("event_id"),
            "class_name": class_name,
            "event_type": event.get("event_type") or event.get("type"),
            "risk_level": event.get("risk_level_hint"),
            "confidence": confidence,
            "initial_event_confidence": event.get("confidence"),
            "distance_m": event.get("distance_m"),
            "odom_point_xy_m": odom_xy,
            "approach_goal": payload.get("approach_goal"),
            "nav_status": nav_status,
            "confirmation_rule": {
                "close_usb_confidence_gte": self.args.confirm_map_confidence,
                "source": "usb_close_confirm",
                "nav2_approach_result": nav_status,
            },
            "close_usb_confirm": close_confirm,
            "semantic_mode": event.get("semantic_mode") or "approach_then_confirm",
            "recommended_action": event.get("recommended_action_hint"),
            "arm_operation_semantic": (
                event.get("arm_operation_semantic")
                or ("blockage_response_candidate" if class_name == "blockage" else "none")
            ),
        }
        self.confirmed_map_points.append(confirmed)
        write_jsonl(self.confirmed_map_points_jsonl_path, confirmed)
        self.confirmed_map_points_path.write_text(
            json.dumps(
                {
                    "schema_version": "confirmed_risk_map_points_v1",
                    "updated_at": confirmed["confirmed_at"],
                    "confirm_map_confidence": self.args.confirm_map_confidence,
                    "risk_map_points": self.confirmed_map_points,
                },
                ensure_ascii=False,
                indent=2,
            )
            + "\n",
            encoding="utf-8",
        )
        payload["confirmed_risk_map_point"] = confirmed
        self.publish_status("confirmed_map_point_written", payload)


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--event-topic", default="/prelim_demo/risk_event")
    parser.add_argument("--status-topic", default="/risk/approach_status")
    parser.add_argument("--odom-topic", default="/odom")
    parser.add_argument("--goal-frame", default="odom")
    parser.add_argument("--stand-off-m", type=float, default=0.40)
    parser.add_argument("--final-stand-off-m", type=float, default=0.20)
    parser.add_argument("--final-creep-cmd-topic", default="/cmd_vel_raw")
    parser.add_argument("--final-creep-linear-x", type=float, default=0.04)
    parser.add_argument("--final-creep-max-s", type=float, default=5.0)
    parser.add_argument("--already-near-margin-m", type=float, default=0.03)
    parser.add_argument("--passive-close-confirm-distance-m", type=float, default=0.40)
    parser.add_argument("--min-confidence", type=float, default=0.20)
    parser.add_argument("--confirm-map-confidence", type=float, default=0.60)
    parser.add_argument("--max-events", type=int, default=1)
    parser.add_argument("--nav2-wait-s", type=float, default=1.5)
    parser.add_argument("--interrupt-rrt-on-event", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--resume-rrt-after-event", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--arm-simulation-mode", default="semantic_only")
    parser.add_argument("--arm-simulation-s", type=float, default=1.0)
    parser.add_argument("--resume-rrt-runtime-s", type=int, default=86400)
    parser.add_argument("--resume-rrt-max-goals", type=int, default=1000000)
    parser.add_argument("--resume-rrt-min-goal-clearance-m", type=float, default=0.24)
    parser.add_argument("--resume-rrt-map-edge-margin-m", type=float, default=0.10)
    parser.add_argument("--resume-rrt-min-goal-distance-m", type=float, default=0.30)
    parser.add_argument("--resume-rrt-free-roam-min-distance-m", type=float, default=0.15)
    parser.add_argument("--resume-rrt-start-free-search-m", type=float, default=0.45)
    parser.add_argument("--resume-rrt-replan-sleep-s", type=float, default=2.5)
    parser.add_argument("--close-confirm-usb-device", default="/dev/video26")
    parser.add_argument("--close-confirm-width", type=int, default=640)
    parser.add_argument("--close-confirm-height", type=int, default=480)
    parser.add_argument("--close-confirm-warmup-frames", type=int, default=3)
    parser.add_argument("--close-confirm-capture-timeout-s", type=float, default=20.0)
    parser.add_argument("--close-confirm-infer-timeout-s", type=float, default=45.0)
    parser.add_argument(
        "--close-confirm-model",
        default="models/risk_vision/yolov8n_480x640_q_truncated6_balanced_blockage03.onnx",
    )
    parser.add_argument("--close-confirm-imgsz", type=int, default=640)
    parser.add_argument("--close-confirm-conf", type=float, default=0.15)
    parser.add_argument("--close-confirm-iou", type=float, default=0.45)
    parser.add_argument("--close-confirm-max-det", type=int, default=10)
    parser.add_argument("--close-confirm-providers", default="SpaceMITExecutionProvider,CPUExecutionProvider")
    parser.add_argument("--output-dir", required=True)
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = RealK1RiskApproach(args)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
