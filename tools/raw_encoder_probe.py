#!/usr/bin/env python3
import argparse
import json
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import Log
from rclpy.node import Node
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import String


ACCEL_SCALE = 1671.84
GYRO_SCALE = 0.00026644
DIAG_PREFIX = "diag "


def reject(message: str) -> None:
    print(f"REFUSE: {message}", file=sys.stderr)
    sys.exit(2)


def signed_stats(values):
    if not values:
        return {"samples": 0, "mean": None, "min": None, "max": None, "max_abs": None}
    return {
        "samples": len(values),
        "mean": round(sum(values) / len(values), 3),
        "min": round(min(values), 3),
        "max": round(max(values), 3),
        "max_abs": round(max(abs(v) for v in values), 3),
    }


def window_rows(rows, start_time, end_time):
    return [row for row in rows if start_time <= row["t"] <= end_time]


def yaw_from_odom(msg: Odometry) -> float:
    q = msg.pose.pose.orientation
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class RawEncoderProbe(Node):
    def __init__(self, args):
        super().__init__("raw_encoder_probe")
        self.args = args
        self.cmd_pub = self.create_publisher(Twist, args.input_cmd_topic, 10)

        self.latest_scan_time = 0.0
        self.latest_status_time = 0.0
        self.latest_status = None
        self.latest_odom = None
        self.latest_odom_time = 0.0
        self.latest_guarded = None
        self.latest_guarded_time = 0.0
        self.latest_robot_vel = None
        self.latest_robot_vel_time = 0.0
        self.latest_diag = None
        self.latest_diag_time = 0.0

        self.raw_rows = []
        self.guarded_rows = []
        self.robot_vel_rows = []
        self.odom_rows = []
        self.diag_rows = []

        self.create_subscription(LaserScan, args.scan_topic, self.scan_cb, 10)
        self.create_subscription(String, args.status_topic, self.status_cb, 20)
        self.create_subscription(Odometry, args.odom_topic, self.odom_cb, 20)
        self.create_subscription(Twist, args.guarded_cmd_topic, self.guarded_cb, 20)
        self.create_subscription(Vector3, args.robot_vel_topic, self.robot_vel_cb, 20)
        self.create_subscription(Imu, args.imu_topic, self.imu_cb, 50)
        self.create_subscription(Log, "/rosout", self.rosout_cb, 50)

    def scan_cb(self, _msg):
        self.latest_scan_time = time.monotonic()

    def status_cb(self, msg):
        try:
            self.latest_status = json.loads(msg.data)
            self.latest_status_time = time.monotonic()
        except json.JSONDecodeError:
            pass

    def odom_cb(self, msg):
        now = time.monotonic()
        self.latest_odom = msg
        self.latest_odom_time = now
        self.odom_rows.append(
            {
                "t": now,
                "yaw": yaw_from_odom(msg),
                "wz": float(msg.twist.twist.angular.z),
            }
        )

    def guarded_cb(self, msg):
        now = time.monotonic()
        self.latest_guarded = msg
        self.latest_guarded_time = now
        self.guarded_rows.append({"t": now, "wz": float(msg.angular.z)})

    def robot_vel_cb(self, msg):
        now = time.monotonic()
        self.latest_robot_vel = msg
        self.latest_robot_vel_time = now
        self.robot_vel_rows.append({"t": now, "wz": float(msg.z)})

    def imu_cb(self, msg):
        now = time.monotonic()
        self.raw_rows.append(
            {
                "t": now,
                "enc_a_tim2":           int(round(float(msg.linear_acceleration.x) * ACCEL_SCALE)),
                "enc_b_tim3":           int(round(float(msg.linear_acceleration.y) * ACCEL_SCALE)),
                "motor_a_target_mmps":  int(round(float(msg.linear_acceleration.z) * ACCEL_SCALE)),
                "motor_b_target_mmps":  int(round(float(msg.angular_velocity.x) / GYRO_SCALE)),
                "motor_a_pwm":          int(round(float(msg.angular_velocity.y) / GYRO_SCALE)),
                "motor_b_pwm":          int(round(float(msg.angular_velocity.z) / GYRO_SCALE)),
            }
        )

    def rosout_cb(self, msg):
        if "wheeltec_tank_base" not in msg.name or DIAG_PREFIX not in msg.msg:
            return
        self.latest_diag = msg.msg
        self.latest_diag_time = time.monotonic()
        self.diag_rows.append({"t": self.latest_diag_time, "msg": msg.msg})

    def spin_for(self, duration, step=0.05):
        end = time.monotonic() + max(0.0, duration)
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=step)

    def wait_ready(self):
        deadline = time.monotonic() + self.args.wait_ready_s
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if (
                self.cmd_pub.get_subscription_count() >= 1
                and self.latest_scan_time > 0.0
                and self.latest_status_time > 0.0
                and self.latest_odom_time > 0.0
                and self.raw_rows
            ):
                return
        reject("not ready: need /scan, /safety/front_obstacle, /odom, /imu, and input cmd subscriber")

    def zero_hold(self, duration):
        msg = Twist()
        end = time.monotonic() + max(0.2, duration)
        period = 1.0 / self.args.rate
        while rclpy.ok() and time.monotonic() < end:
            self.cmd_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(period)
        self.cmd_pub.publish(msg)
        self.spin_for(0.2)

    def publish_turn(self, angular, duration):
        msg = Twist()
        msg.angular.z = angular
        start = time.monotonic()
        end = start + duration
        period = 1.0 / self.args.rate
        while rclpy.ok() and time.monotonic() < end:
            self.cmd_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(period)
        return start, time.monotonic()

    def summarize_segment(self, name, angular, duration):
        self.spin_for(self.args.precheck_s)
        status = dict(self.latest_status or {})
        if self.args.require_front_clear and status.get("state") not in ("clear", None):
            reject(f"front obstacle state is {status.get('state')}; refusing turn")

        odom_start = self.latest_odom
        command_start, command_end = self.publish_turn(angular, duration)
        self.zero_hold(self.args.zero_hold_s)
        segment_end = time.monotonic()
        odom_end = self.latest_odom

        raw_cmd = window_rows(self.raw_rows, command_start, command_end)
        raw_full = window_rows(self.raw_rows, command_start, segment_end)
        guarded_cmd = window_rows(self.guarded_rows, command_start, command_end)
        robot_cmd = window_rows(self.robot_vel_rows, command_start, command_end)
        odom_cmd = window_rows(self.odom_rows, command_start, command_end)

        yaw_delta = None
        if odom_start is not None and odom_end is not None:
            yaw_delta = math.atan2(
                math.sin(yaw_from_odom(odom_end) - yaw_from_odom(odom_start)),
                math.cos(yaw_from_odom(odom_end) - yaw_from_odom(odom_start)),
            )

        result = {
            "name": name,
            "command": {"linear": 0.0, "angular": angular, "duration_s": duration},
            "front": {
                "state": status.get("state"),
                "action": status.get("action"),
                "front_p10_range_m": status.get("front_p10_range_m"),
            },
            "samples": {
                "raw_command": len(raw_cmd),
                "raw_full": len(raw_full),
                "guarded_command": len(guarded_cmd),
                "robot_vel_command": len(robot_cmd),
                "odom_command": len(odom_cmd),
            },
            "raw_command_stats": {
                key: signed_stats([row[key] for row in raw_cmd])
                for key in (
                    "enc_a_tim2", "enc_b_tim3",
                    "motor_a_target_mmps", "motor_b_target_mmps",
                    "motor_a_pwm", "motor_b_pwm",
                )
            },
            "raw_full_stats": {
                key: signed_stats([row[key] for row in raw_full])
                for key in (
                    "enc_a_tim2", "enc_b_tim3",
                    "motor_a_target_mmps", "motor_b_target_mmps",
                    "motor_a_pwm", "motor_b_pwm",
                )
            },
            "guarded_wz_command": signed_stats([row["wz"] for row in guarded_cmd]),
            "robot_vel_wz_command": signed_stats([row["wz"] for row in robot_cmd]),
            "odom_wz_command": signed_stats([row["wz"] for row in odom_cmd]),
            "yaw_delta_deg": None if yaw_delta is None else round(math.degrees(yaw_delta), 3),
        }
        print("RAW_ENCODER_SEGMENT " + json.dumps(result, ensure_ascii=False), flush=True)
        return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--input-cmd-topic", default="/input_cmd_vel")
    parser.add_argument("--guarded-cmd-topic", default="/cmd_vel_guarded")
    parser.add_argument("--robot-vel-topic", default="/robot_vel")
    parser.add_argument("--odom-topic", default="/odom")
    parser.add_argument("--scan-topic", default="/scan")
    parser.add_argument("--status-topic", default="/safety/front_obstacle")
    parser.add_argument("--imu-topic", default="/imu")
    parser.add_argument("--rate", type=float, default=50.0)
    parser.add_argument("--duration-s", type=float, default=1.0)
    parser.add_argument("--zero-hold-s", type=float, default=4.0)
    parser.add_argument("--precheck-s", type=float, default=0.5)
    parser.add_argument("--wait-ready-s", type=float, default=8.0)
    parser.add_argument("--angular", type=float, nargs="+", default=[0.4, -0.4, 0.8, -0.8])
    parser.add_argument("--require-front-clear", action="store_true")
    parser.add_argument("--report", required=True)
    parser.add_argument("--confirm", required=True)
    args = parser.parse_args()

    if args.confirm != "YES":
        reject("--confirm YES is required")
    if args.duration_s <= 0.0 or args.duration_s > 1.0:
        reject("--duration-s must be in (0, 1.0]")
    if any(abs(v) > 0.8 for v in args.angular):
        reject("--angular values must be within +/-0.8 rad/s")

    print(
        "RAW ENCODER PROBE: publishes angular-only commands to /input_cmd_vel. "
        "Requires temporary firmware that maps raw TIM2/TIM3/TIM4/TIM5 counts into /imu.",
        flush=True,
    )

    rclpy.init()
    node = RawEncoderProbe(args)
    records = []
    try:
        node.wait_ready()
        node.zero_hold(args.zero_hold_s)
        for angular in args.angular:
            name = f"turn_{angular:+.2f}".replace("+", "pos_").replace("-", "neg_").replace(".", "p")
            records.append(node.summarize_segment(name, angular, args.duration_s))
        node.zero_hold(args.zero_hold_s)
    finally:
        node.zero_hold(0.5)
        node.destroy_node()
        rclpy.shutdown()

    payload = {
        "mode": "raw_encoder_probe",
        "raw_mapping": {
            "enc_a_tim2":           "/imu.linear_acceleration.x * 1671.84",
            "enc_b_tim3":           "/imu.linear_acceleration.y * 1671.84",
            "motor_a_target_mmps":  "/imu.linear_acceleration.z * 1671.84  → MOTOR_A.Target*1000",
            "motor_b_target_mmps":  "/imu.angular_velocity.x / 0.00026644 → MOTOR_B.Target*1000",
            "motor_a_pwm":          "/imu.angular_velocity.y / 0.00026644 → MOTOR_A.Motor_Pwm",
            "motor_b_pwm":          "/imu.angular_velocity.z / 0.00026644 → MOTOR_B.Motor_Pwm",
        },
        "records": records,
    }
    with open(args.report, "w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2, ensure_ascii=False)
    print(f"REPORT {args.report}", flush=True)
    print("RESULT_JSON " + json.dumps(payload, ensure_ascii=False), flush=True)


if __name__ == "__main__":
    main()
