#!/usr/bin/env python3
import math
import json
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, Twist, Vector3
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32, String
from tf2_ros import TransformBroadcaster

import serial


FRAME_HEADER = 0x7B
FRAME_TAIL = 0x7D


def clamp(value, lo, hi):
    return max(lo, min(hi, value))


def s16_to_float_mmps(high, low):
    value = (high << 8) | low
    if value >= 0x8000:
        value -= 0x10000
    return value / 1000.0


def s16(high, low):
    value = (high << 8) | low
    if value >= 0x8000:
        value -= 0x10000
    return value


def yaw_to_quat(yaw):
    half = yaw * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


def make_cmd_frame(vx_mps, wz_radps, auto_recharge=0, security_ply=0):
    vx = int(round(vx_mps * 1000.0))
    vy = 0
    wz = int(round(wz_radps * 1000.0))

    def bytes_s16(value):
        if value < 0:
            value = (1 << 16) + value
        return [(value >> 8) & 0xFF, value & 0xFF]

    data = [FRAME_HEADER, auto_recharge & 0xFF, security_ply & 0xFF]
    data += bytes_s16(vx)
    data += bytes_s16(vy)
    data += bytes_s16(wz)
    check = 0
    for byte in data:
        check ^= byte
    return bytes(data + [check, FRAME_TAIL])


def make_security_frame(enabled, legacy=False):
    data = [FRAME_HEADER, 0x00, 0x01 if enabled and legacy else 0x00]
    if not legacy:
        data[2] = 0xB1 if enabled else 0xB0
    data += [0x00] * 6
    check = 0
    for byte in data:
        check ^= byte
    return bytes(data + [check, FRAME_TAIL])


class WheeltecTankBase(Node):
    def __init__(self):
        super().__init__("wheeltec_tank_base")

        self.declare_parameter("port", "/dev/base_controller")
        self.declare_parameter("baud", 115200)
        self.declare_parameter("auto_recharge", 0)
        self.declare_parameter("security_ply", 0)
        self.declare_parameter("send_security_enable_on_start", False)
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("stop_request_topic", "/chassis/stop_request")
        self.declare_parameter("max_linear", 0.005)
        self.declare_parameter("max_angular", 0.03)
        self.declare_parameter("cmd_timeout", 0.25)
        self.declare_parameter("send_rate", 50.0)
        self.declare_parameter("brake_duration", 1.0)
        self.declare_parameter("cruise_linear_limit", 0.08)
        self.declare_parameter("cruise_angular_limit", 0.20)
        self.declare_parameter("start_kick_duration", 0.0)
        self.declare_parameter("start_kick_linear", 0.0)
        self.declare_parameter("start_kick_angular", 0.0)
        self.declare_parameter("stop_kick_duration", 0.0)
        self.declare_parameter("stop_kick_linear", 0.0)
        self.declare_parameter("stop_kick_angular", 0.0)
        self.declare_parameter("stop_kick_match_cmd", False)
        self.declare_parameter("stop_kick_match_duration", False)
        self.declare_parameter("stop_kick_speed_gain", 1.0)
        self.declare_parameter("stop_kick_duration_mode", "duration_ratio")
        self.declare_parameter("stop_kick_duration_ratio", 1.0)
        self.declare_parameter("stop_kick_impulse_ratio", 1.0)
        self.declare_parameter("stop_kick_duration_offset", 0.0)
        self.declare_parameter("stop_kick_max_duration", 1.0)
        self.declare_parameter("stop_kick_min_duration", 0.12)
        self.declare_parameter("stop_kick_until_stopped", False)
        self.declare_parameter("stop_kick_velocity_epsilon", 0.02)
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("imu_frame", "gyro_link")
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("odom_linear_scale", 1.0)
        self.declare_parameter("odom_angular_scale", 1.0)

        self.port = self.get_parameter("port").value
        self.baud = int(self.get_parameter("baud").value)
        self.auto_recharge = int(self.get_parameter("auto_recharge").value) & 0xFF
        self.security_ply = int(self.get_parameter("security_ply").value) & 0xFF
        self.send_security_enable_on_start = bool(
            self.get_parameter("send_security_enable_on_start").value
        )
        self.max_linear = float(self.get_parameter("max_linear").value)
        self.max_angular = float(self.get_parameter("max_angular").value)
        self.cmd_timeout = float(self.get_parameter("cmd_timeout").value)
        self.send_rate = float(self.get_parameter("send_rate").value)
        self.brake_duration = float(self.get_parameter("brake_duration").value)
        self.cruise_linear_limit = abs(float(self.get_parameter("cruise_linear_limit").value))
        self.cruise_angular_limit = abs(float(self.get_parameter("cruise_angular_limit").value))
        self.start_kick_duration = float(self.get_parameter("start_kick_duration").value)
        self.start_kick_linear = abs(float(self.get_parameter("start_kick_linear").value))
        self.start_kick_angular = abs(float(self.get_parameter("start_kick_angular").value))
        self.stop_kick_duration = float(self.get_parameter("stop_kick_duration").value)
        self.stop_kick_linear = abs(float(self.get_parameter("stop_kick_linear").value))
        self.stop_kick_angular = abs(float(self.get_parameter("stop_kick_angular").value))
        self.stop_kick_match_cmd = bool(self.get_parameter("stop_kick_match_cmd").value)
        self.stop_kick_match_duration = bool(self.get_parameter("stop_kick_match_duration").value)
        self.stop_kick_speed_gain = max(
            0.0, float(self.get_parameter("stop_kick_speed_gain").value)
        )
        self.stop_kick_duration_mode = str(
            self.get_parameter("stop_kick_duration_mode").value
        ).strip().lower()
        if self.stop_kick_duration_mode not in ("duration_ratio", "impulse", "fixed"):
            self.stop_kick_duration_mode = "duration_ratio"
        self.stop_kick_duration_ratio = max(
            0.0, float(self.get_parameter("stop_kick_duration_ratio").value)
        )
        self.stop_kick_impulse_ratio = max(
            0.0, float(self.get_parameter("stop_kick_impulse_ratio").value)
        )
        self.stop_kick_duration_offset = max(
            0.0, float(self.get_parameter("stop_kick_duration_offset").value)
        )
        self.stop_kick_max_duration = max(
            0.0, float(self.get_parameter("stop_kick_max_duration").value)
        )
        self.stop_kick_min_duration = max(
            0.0, float(self.get_parameter("stop_kick_min_duration").value)
        )
        self.stop_kick_until_stopped = bool(
            self.get_parameter("stop_kick_until_stopped").value
        )
        self.stop_kick_velocity_epsilon = max(
            0.0, float(self.get_parameter("stop_kick_velocity_epsilon").value)
        )
        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.imu_frame = self.get_parameter("imu_frame").value
        self.publish_tf = bool(self.get_parameter("publish_tf").value)
        self.odom_linear_scale = float(self.get_parameter("odom_linear_scale").value)
        self.odom_angular_scale = float(self.get_parameter("odom_angular_scale").value)

        self.serial = serial.Serial(self.port, self.baud, timeout=0.02)
        self.serial.reset_input_buffer()
        self.serial.reset_output_buffer()
        if self.send_security_enable_on_start and self.security_ply:
            self.serial.write(make_security_frame(True, legacy=False))
            time.sleep(0.02)
            self.serial.write(make_security_frame(True, legacy=True))
            time.sleep(0.02)

        self.cmd_vx = 0.0
        self.cmd_wz = 0.0
        self.last_cmd_time = 0.0
        self.brake_until = 0.0
        self.start_kick_until = 0.0
        self.start_kick_vx = 0.0
        self.start_kick_wz = 0.0
        self.stop_kick_until = 0.0
        self.stop_kick_vx = 0.0
        self.stop_kick_wz = 0.0
        self.serial_motion_active = False
        self.serial_motion_started_at = 0.0
        self.serial_motion_vx = 0.0
        self.serial_motion_wz = 0.0
        self.last_nonzero_serial_vx = 0.0
        self.last_nonzero_serial_wz = 0.0
        self.last_nonzero_serial_duration = 0.0
        self.last_nonzero_serial_updated_at = 0.0
        self.last_motion_vx = 0.0
        self.last_motion_wz = 0.0
        self.motion_started_at = 0.0
        self.stop_kick_started_at = 0.0
        self.stop_kick_feedback_vx = 0.0
        self.stop_kick_feedback_wz = 0.0
        self.stop_kick_active_logged = False
        self.latest_feedback_vx = 0.0
        self.latest_feedback_wz = 0.0
        self.timeout_stop_started = False
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.last_odom_time = self.get_clock().now()
        self.rx_buffer = bytearray()
        self.tx_count = 0
        self.last_tx_vx = 0.0
        self.last_tx_wz = 0.0
        self.rx_bytes = 0
        self.frame_count = 0
        self.last_diag_time = time.monotonic()
        self.last_diag_tx = 0
        self.last_diag_rx = 0
        self.last_diag_frames = 0
        self.last_stop_request_time = 0.0

        cmd_topic = self.get_parameter("cmd_vel_topic").value
        stop_request_topic = self.get_parameter("stop_request_topic").value
        self.create_subscription(Twist, cmd_topic, self.cmd_callback, 10)
        self.create_subscription(String, stop_request_topic, self.stop_request_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.imu_pub = self.create_publisher(Imu, "/imu", 10)
        self.voltage_pub = self.create_publisher(Float32, "/battery_voltage", 10)
        self.robot_vel_pub = self.create_publisher(Vector3, "/robot_vel", 10)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None

        period = 1.0 / self.send_rate
        self.create_timer(period, self.control_tick)
        self.create_timer(0.02, self.read_tick)
        self.create_timer(1.0, self.diag_tick)

        self.get_logger().info(
            f"Tank base on {self.port}@{self.baud}; max_linear={self.max_linear}, "
            f"max_angular={self.max_angular}, cruise_linear_limit={self.cruise_linear_limit}, "
            f"cruise_angular_limit={self.cruise_angular_limit}, timeout={self.cmd_timeout}s, "
            f"stop_kick_match_cmd={self.stop_kick_match_cmd}, "
            f"stop_kick_match_duration={self.stop_kick_match_duration}, "
            f"stop_kick_speed_gain={self.stop_kick_speed_gain}, "
            f"stop_kick_duration_mode={self.stop_kick_duration_mode}, "
            f"stop_kick_duration_ratio={self.stop_kick_duration_ratio}, "
            f"stop_kick_impulse_ratio={self.stop_kick_impulse_ratio}, "
            f"stop_kick_duration_offset={self.stop_kick_duration_offset}, "
            f"stop_kick_until_stopped={self.stop_kick_until_stopped}, "
            f"auto_recharge={self.auto_recharge}, security_ply={self.security_ply}, "
            f"security_start={self.send_security_enable_on_start}"
        )

    def stop_request_callback(self, msg):
        now = time.monotonic()
        if now - self.last_stop_request_time < 0.05:
            return
        try:
            request = json.loads(msg.data) if msg.data else {}
        except json.JSONDecodeError:
            request = {"request": msg.data}
        if request.get("request") != "STOP_REQUEST":
            return
        force_duration = float(request.get("force_kick_duration", 0.0) or 0.0)
        force_vx = float(request.get("force_kick_vx", 0.0) or 0.0)
        force_wz = float(request.get("force_kick_wz", 0.0) or 0.0)
        force_requested = force_duration > 0.0 and (
            abs(force_vx) > 1e-6 or abs(force_wz) > 1e-6
        )

        if force_requested:
            if now < self.stop_kick_until:
                return
        elif now < self.stop_kick_until or now < self.brake_until:
            return

        self.last_stop_request_time = now
        if force_requested:
            self.cmd_vx = 0.0
            self.cmd_wz = 0.0
            self.last_cmd_time = now
            self.start_kick_until = 0.0
            self.timeout_stop_started = True
            self.start_forced_stop_kick(
                now,
                force_vx,
                force_wz,
                force_duration,
                reason=str(request.get("reason", "force_stop_request")),
            )
            self.brake_until = max(self.brake_until, self.stop_kick_until + self.brake_duration)
            return

        prev_vx, prev_wz, motion_duration = self.serial_motion_for_stop(now)
        self.cmd_vx = 0.0
        self.cmd_wz = 0.0
        self.last_cmd_time = now
        self.start_kick_until = 0.0
        self.timeout_stop_started = True

        if abs(prev_vx) <= 1e-6 and abs(prev_wz) <= 1e-6:
            self.brake_until = now + self.brake_duration
            self.get_logger().info(
                "stop_request_no_motion reason=%s front_p10=%s"
                % (request.get("reason", "unknown"), request.get("front_p10_range_m"))
            )
            return

        self.get_logger().info(
            "stop_request reason=%s prev_serial=(%.3f,%.3f) serial_duration=%.3fs "
            "front_p10=%s"
            % (
                request.get("reason", "unknown"),
                prev_vx,
                prev_wz,
                motion_duration,
                request.get("front_p10_range_m"),
            )
        )
        self.start_stop_kick(
            now,
            prev_vx,
            prev_wz,
            duration_override=motion_duration,
            reason="stop_request",
        )
        self.brake_until = max(self.brake_until, self.stop_kick_until + self.brake_duration)

    def start_forced_stop_kick(self, now, force_vx, force_wz, force_duration, reason):
        duration = max(0.0, force_duration)
        if self.stop_kick_max_duration > 0.0:
            duration = min(duration, self.stop_kick_max_duration)
        duration = max(duration, self.stop_kick_min_duration)
        self.stop_kick_vx = clamp(force_vx, -self.max_linear, self.max_linear)
        self.stop_kick_wz = clamp(force_wz, -self.max_angular, self.max_angular)
        self.stop_kick_started_at = now
        self.stop_kick_feedback_vx = self.latest_feedback_vx
        self.stop_kick_feedback_wz = self.latest_feedback_wz
        self.stop_kick_until = now + duration
        self.stop_kick_active_logged = True
        self.get_logger().info(
            "stop_kick_start reason=%s prev=(%.3f,%.3f) kick=(%.3f,%.3f) duration=%.3fs "
            "feedback_start=(%.3f,%.3f)"
            % (
                reason,
                0.0,
                0.0,
                self.stop_kick_vx,
                self.stop_kick_wz,
                duration,
                self.stop_kick_feedback_vx,
                self.stop_kick_feedback_wz,
            )
        )

    def cmd_callback(self, msg):
        prev_vx = self.cmd_vx
        prev_wz = self.cmd_wz
        self.cmd_vx = clamp(float(msg.linear.x), -self.max_linear, self.max_linear)
        self.cmd_wz = clamp(float(msg.angular.z), -self.max_angular, self.max_angular)
        self.last_cmd_time = time.monotonic()
        moving_before = abs(prev_vx) > 1e-6 or abs(prev_wz) > 1e-6
        stopped_now = abs(self.cmd_vx) < 1e-6 and abs(self.cmd_wz) < 1e-6
        starting_now = not moving_before and not stopped_now
        if not stopped_now:
            self.timeout_stop_started = False
            self.last_motion_vx = self.cmd_vx
            self.last_motion_wz = self.cmd_wz
            if starting_now:
                self.motion_started_at = self.last_cmd_time
            if starting_now and self.start_kick_duration > 0.0:
                if abs(self.cmd_vx) > 1e-6 and self.start_kick_linear > 0.0:
                    self.start_kick_vx = math.copysign(
                        min(self.start_kick_linear, self.max_linear), self.cmd_vx
                    )
                else:
                    self.start_kick_vx = 0.0
                if abs(self.cmd_wz) > 1e-6 and self.start_kick_angular > 0.0:
                    self.start_kick_wz = math.copysign(
                        min(self.start_kick_angular, self.max_angular), self.cmd_wz
                    )
                else:
                    self.start_kick_wz = 0.0
                self.start_kick_until = self.last_cmd_time + self.start_kick_duration
        elif moving_before:
            self.start_kick_until = 0.0
            serial_vx, serial_wz, serial_duration = self.serial_motion_for_stop(self.last_cmd_time)
            if abs(serial_vx) > 1e-6 or abs(serial_wz) > 1e-6:
                prev_vx = serial_vx
                prev_wz = serial_wz
                duration_override = serial_duration
            else:
                duration_override = None
            self.start_stop_kick(
                self.last_cmd_time,
                prev_vx,
                prev_wz,
                duration_override=duration_override,
                reason="cmd_zero",
            )
            self.timeout_stop_started = True
            self.brake_until = max(self.brake_until, self.stop_kick_until + self.brake_duration)

    def serial_motion_for_stop(self, now):
        if self.serial_motion_active:
            duration = max(0.0, now - self.serial_motion_started_at)
            return self.serial_motion_vx, self.serial_motion_wz, duration
        if now - self.last_nonzero_serial_updated_at <= max(self.cmd_timeout, 0.25) + 0.10:
            return (
                self.last_nonzero_serial_vx,
                self.last_nonzero_serial_wz,
                self.last_nonzero_serial_duration,
            )
        return 0.0, 0.0, 0.0

    def update_serial_motion_segment(self, now, vx, wz):
        moving = abs(vx) > 1e-6 or abs(wz) > 1e-6
        if moving:
            if not self.serial_motion_active:
                self.serial_motion_active = True
                self.serial_motion_started_at = now
            self.serial_motion_vx = vx
            self.serial_motion_wz = wz
            self.last_nonzero_serial_vx = vx
            self.last_nonzero_serial_wz = wz
            self.last_nonzero_serial_duration = max(0.0, now - self.serial_motion_started_at)
            self.last_nonzero_serial_updated_at = now
            return
        if self.serial_motion_active:
            self.last_nonzero_serial_duration = max(0.0, now - self.serial_motion_started_at)
            self.last_nonzero_serial_updated_at = now
        self.serial_motion_active = False
        self.serial_motion_started_at = 0.0
        self.serial_motion_vx = 0.0
        self.serial_motion_wz = 0.0

    def start_stop_kick(self, now, prev_vx, prev_wz, duration_override=None, reason="cmd_zero"):
        if now < self.stop_kick_until:
            return
        linear_brake = (
            min(abs(prev_vx) * self.stop_kick_speed_gain, self.max_linear)
            if self.stop_kick_match_cmd
            else self.stop_kick_linear
        )
        angular_brake = (
            min(abs(prev_wz) * self.stop_kick_speed_gain, self.max_angular)
            if self.stop_kick_match_cmd
            else self.stop_kick_angular
        )

        motion_duration = None
        if duration_override is not None:
            motion_duration = max(0.0, duration_override)
        elif self.stop_kick_match_duration:
            motion_duration = max(0.0, now - self.motion_started_at)

        stop_duration = self.stop_kick_duration
        if self.stop_kick_duration_mode == "impulse" and motion_duration is not None:
            impulse_durations = []
            if abs(prev_vx) > 1e-6 and linear_brake > 1e-6:
                impulse_durations.append(
                    abs(prev_vx) * motion_duration * self.stop_kick_impulse_ratio / linear_brake
                )
            if abs(prev_wz) > 1e-6 and angular_brake > 1e-6:
                impulse_durations.append(
                    abs(prev_wz) * motion_duration * self.stop_kick_impulse_ratio / angular_brake
                )
            if impulse_durations:
                stop_duration = max(impulse_durations) + self.stop_kick_duration_offset
        elif self.stop_kick_duration_mode == "duration_ratio" and motion_duration is not None:
            stop_duration = (
                motion_duration * self.stop_kick_duration_ratio
                + self.stop_kick_duration_offset
            )
        elif self.stop_kick_until_stopped and self.stop_kick_max_duration > 0.0:
            stop_duration = self.stop_kick_max_duration

        if self.stop_kick_max_duration > 0.0:
            stop_duration = min(stop_duration, self.stop_kick_max_duration)
        if stop_duration <= 0.0:
            return
        stop_duration = max(stop_duration, self.stop_kick_min_duration)
        if abs(prev_vx) > 1e-6 and linear_brake > 0.0:
            self.stop_kick_vx = -math.copysign(min(linear_brake, self.max_linear), prev_vx)
        else:
            self.stop_kick_vx = 0.0
        if abs(prev_wz) > 1e-6 and angular_brake > 0.0:
            self.stop_kick_wz = -math.copysign(min(angular_brake, self.max_angular), prev_wz)
        else:
            self.stop_kick_wz = 0.0
        self.stop_kick_started_at = now
        self.stop_kick_feedback_vx = self.latest_feedback_vx
        self.stop_kick_feedback_wz = self.latest_feedback_wz
        self.stop_kick_until = now + stop_duration
        self.stop_kick_active_logged = True
        self.get_logger().info(
            "stop_kick_start reason=%s prev=(%.3f,%.3f) kick=(%.3f,%.3f) duration=%.3fs "
            "feedback_start=(%.3f,%.3f)"
            % (
                reason,
                prev_vx,
                prev_wz,
                self.stop_kick_vx,
                self.stop_kick_wz,
                stop_duration,
                self.stop_kick_feedback_vx,
                self.stop_kick_feedback_wz,
            )
        )

    def control_tick(self):
        now = time.monotonic()
        if now < self.stop_kick_until:
            if self.stop_kick_until_stopped and self.stop_kick_is_done():
                self.stop_kick_until = 0.0
                self.brake_until = now + self.brake_duration
                vx = 0.0
                wz = 0.0
            else:
                vx = self.stop_kick_vx
                wz = self.stop_kick_wz
        elif now < self.start_kick_until:
            vx = self.start_kick_vx
            wz = self.start_kick_wz
        elif now < self.brake_until:
            self.log_stop_kick_end_once(now, "brake_hold")
            vx = 0.0
            wz = 0.0
        elif now - self.last_cmd_time > self.cmd_timeout:
            self.log_stop_kick_end_once(now, "timeout_zero")
            if not self.timeout_stop_started:
                self.start_kick_until = 0.0
                serial_vx, serial_wz, serial_duration = self.serial_motion_for_stop(now)
                if abs(serial_vx) > 1e-6 or abs(serial_wz) > 1e-6:
                    self.start_stop_kick(
                        now,
                        serial_vx,
                        serial_wz,
                        duration_override=serial_duration,
                        reason="cmd_timeout",
                    )
                else:
                    self.start_stop_kick(now, self.last_motion_vx, self.last_motion_wz, reason="cmd_timeout")
                self.timeout_stop_started = True
                vx = self.stop_kick_vx if now < self.stop_kick_until else 0.0
                wz = self.stop_kick_wz if now < self.stop_kick_until else 0.0
            else:
                vx = 0.0
                wz = 0.0
        else:
            vx = clamp(self.cmd_vx, -self.cruise_linear_limit, self.cruise_linear_limit)
            wz = clamp(self.cmd_wz, -self.cruise_angular_limit, self.cruise_angular_limit)
        self.update_serial_motion_segment(now, vx, wz)
        self.serial.write(make_cmd_frame(vx, wz, self.auto_recharge, self.security_ply))
        self.last_tx_vx = vx
        self.last_tx_wz = wz
        self.tx_count += 1

    def log_stop_kick_end_once(self, now, phase):
        if not self.stop_kick_active_logged:
            return
        if now < self.stop_kick_until:
            return
        self.get_logger().info(
            "stop_kick_end phase=%s elapsed=%.3fs feedback_now=(%.3f,%.3f)"
            % (
                phase,
                max(0.0, now - self.stop_kick_started_at),
                self.latest_feedback_vx,
                self.latest_feedback_wz,
            )
        )
        self.stop_kick_active_logged = False

    def stop_kick_is_done(self):
        if time.monotonic() - self.stop_kick_started_at < self.stop_kick_min_duration:
            return False

        linear_done = True
        angular_done = True
        if abs(self.stop_kick_vx) > 1e-6:
            linear_done = (
                abs(self.latest_feedback_vx) <= self.stop_kick_velocity_epsilon
                or self.latest_feedback_vx * self.stop_kick_feedback_vx <= 0.0
            )
        if abs(self.stop_kick_wz) > 1e-6:
            angular_done = (
                abs(self.latest_feedback_wz) <= self.stop_kick_velocity_epsilon
                or self.latest_feedback_wz * self.stop_kick_feedback_wz <= 0.0
            )
        return linear_done and angular_done

    def read_tick(self):
        data = self.serial.read(256)
        if data:
            self.rx_buffer.extend(data)
            self.rx_bytes += len(data)
        while len(self.rx_buffer) >= 24:
            try:
                head = self.rx_buffer.index(FRAME_HEADER)
            except ValueError:
                self.rx_buffer.clear()
                return
            if head:
                del self.rx_buffer[:head]
            if len(self.rx_buffer) < 24:
                return
            frame = bytes(self.rx_buffer[:24])
            if frame[23] != FRAME_TAIL:
                del self.rx_buffer[0]
                continue
            check = 0
            for byte in frame[:22]:
                check ^= byte
            if check != frame[22]:
                del self.rx_buffer[0]
                continue
            del self.rx_buffer[:24]
            try:
                self.handle_frame(frame)
                self.frame_count += 1
            except Exception as exc:
                self.get_logger().warn(f"ignored feedback publish error: {exc}")

    def diag_tick(self):
        now = time.monotonic()
        dt = max(1e-6, now - self.last_diag_time)
        tx_rate = (self.tx_count - self.last_diag_tx) / dt
        rx_rate = (self.rx_bytes - self.last_diag_rx) / dt
        frame_rate = (self.frame_count - self.last_diag_frames) / dt
        self.last_diag_time = now
        self.last_diag_tx = self.tx_count
        self.last_diag_rx = self.rx_bytes
        self.last_diag_frames = self.frame_count
        self.get_logger().info(
            f"diag tx={tx_rate:.1f}/s rx={rx_rate:.0f}B/s frames={frame_rate:.1f}/s "
            f"cmd=({self.cmd_vx:.3f},{self.cmd_wz:.3f}) "
            f"serial=({self.last_tx_vx:.3f},{self.last_tx_wz:.3f}) "
            f"feedback=({self.latest_feedback_vx:.3f},{self.latest_feedback_wz:.3f})"
        )

    def handle_frame(self, frame):
        now = self.get_clock().now()
        dt = (now - self.last_odom_time).nanoseconds / 1e9
        self.last_odom_time = now

        vx = s16_to_float_mmps(frame[2], frame[3])
        vy = s16_to_float_mmps(frame[4], frame[5])
        wz = s16_to_float_mmps(frame[6], frame[7])
        self.latest_feedback_vx = vx
        self.latest_feedback_wz = wz

        odom_vx = vx * self.odom_linear_scale
        odom_vy = vy * self.odom_linear_scale
        odom_wz = wz * self.odom_angular_scale

        self.x += (odom_vx * math.cos(self.yaw) - odom_vy * math.sin(self.yaw)) * dt
        self.y += (odom_vx * math.sin(self.yaw) + odom_vy * math.cos(self.yaw)) * dt
        self.yaw += odom_wz * dt

        qx, qy, qz, qw = yaw_to_quat(self.yaw)

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = odom_vx
        odom.twist.twist.linear.y = odom_vy
        odom.twist.twist.angular.z = odom_wz
        self.odom_pub.publish(odom)

        if self.tf_broadcaster is not None:
            tf = TransformStamped()
            tf.header = odom.header
            tf.child_frame_id = self.base_frame
            tf.transform.translation.x = self.x
            tf.transform.translation.y = self.y
            tf.transform.rotation = odom.pose.pose.orientation
            self.tf_broadcaster.sendTransform(tf)

        imu = Imu()
        imu.header.stamp = now.to_msg()
        imu.header.frame_id = self.imu_frame
        imu.linear_acceleration.x = s16(frame[8], frame[9]) / 1671.84
        imu.linear_acceleration.y = s16(frame[10], frame[11]) / 1671.84
        imu.linear_acceleration.z = s16(frame[12], frame[13]) / 1671.84
        imu.angular_velocity.x = s16(frame[14], frame[15]) * 0.00026644
        imu.angular_velocity.y = s16(frame[16], frame[17]) * 0.00026644
        imu.angular_velocity.z = s16(frame[18], frame[19]) * 0.00026644
        imu.orientation.w = 1.0
        self.imu_pub.publish(imu)

        voltage = ((frame[20] << 8) | frame[21]) / 1000.0
        voltage_msg = Float32()
        voltage_msg.data = voltage
        self.voltage_pub.publish(voltage_msg)

        vel_msg = Vector3()
        vel_msg.x = vx
        vel_msg.y = vy
        vel_msg.z = wz
        self.robot_vel_pub.publish(vel_msg)


def main():
    rclpy.init()
    node = WheeltecTankBase()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            zero = make_cmd_frame(0.0, 0.0, node.auto_recharge, node.security_ply)
            for _ in range(20):
                node.serial.write(zero)
                time.sleep(0.01)
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
