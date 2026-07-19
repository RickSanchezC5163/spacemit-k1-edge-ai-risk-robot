#!/usr/bin/env python3
import json
import math
import time
from datetime import datetime, timezone

import rclpy
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


class ScanSafetyGuardNode(Node):
    def __init__(self):
        super().__init__("scan_safety_guard_node")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("input_cmd_topic", "/input_cmd_vel")
        self.declare_parameter("output_cmd_topic", "/cmd_vel")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("status_topic", "/safety/front_obstacle")
        self.declare_parameter("event_topic", "/perception/mock_event")
        self.declare_parameter("stop_request_topic", "/chassis/stop_request")
        self.declare_parameter("front_sector_deg", 35.0)
        self.declare_parameter("front_collision_corridor_half_width_m", 0.26)
        self.declare_parameter("front_collision_min_x_m", 0.12)
        self.declare_parameter("micro_adjust_sector_deg", 45.0)
        self.declare_parameter("micro_adjust_trigger_m", 0.28)
        self.declare_parameter("micro_adjust_clear_m", 0.34)
        self.declare_parameter("micro_adjust_min_valid_range_m", 0.01)
        self.declare_parameter("micro_adjust_angular_z", 0.22)
        self.declare_parameter("micro_adjust_direction_deadband_m", 0.03)
        self.declare_parameter("micro_adjust_direction_latch_s", 1.50)
        self.declare_parameter("enable_micro_adjust", True)
        self.declare_parameter("enable_spin_escape", True)
        self.declare_parameter("spin_escape_turn_changes", 3)
        self.declare_parameter("spin_escape_degrees", 180.0)
        self.declare_parameter("spin_escape_angular_z", 0.35)
        self.declare_parameter("spin_escape_cooldown_s", 3.0)
        self.declare_parameter("enable_micro_adjust_stuck_spin_escape", True)
        self.declare_parameter("micro_adjust_stuck_spin_min_s", 6.0)
        self.declare_parameter("micro_adjust_stuck_spin_front_blocked_m", 0.30)
        self.declare_parameter("micro_adjust_stuck_spin_clear_m", 0.40)
        self.declare_parameter("micro_adjust_stuck_spin_cmd_angular_mps", 0.05)
        self.declare_parameter("enable_corridor_stuck_spin_escape", True)
        self.declare_parameter("corridor_stuck_spin_trigger_m", 0.18)
        self.declare_parameter("corridor_stuck_spin_clear_m", 0.24)
        self.declare_parameter("corridor_stuck_spin_min_s", 3.0)
        self.declare_parameter("corridor_stuck_spin_cmd_angular_mps", 0.06)
        self.declare_parameter("corridor_stuck_spin_front_blocked_m", 0.30)
        self.declare_parameter("corridor_stuck_spin_front_sector_deg", 20.0)
        self.declare_parameter("corridor_stuck_spin_require_sides", True)
        self.declare_parameter("corridor_stuck_spin_side_blocked_m", 0.32)
        self.declare_parameter("enable_corridor_trial", True)
        self.declare_parameter("corridor_trial_center_half_width_m", 0.10)
        self.declare_parameter("corridor_trial_enter_front_p10_m", 0.40)
        self.declare_parameter("corridor_trial_keep_front_p10_m", 0.34)
        self.declare_parameter("corridor_trial_side_near_m", 0.40)
        self.declare_parameter("corridor_trial_enter_stable_s", 0.50)
        self.declare_parameter("corridor_trial_exit_stable_s", 0.80)
        self.declare_parameter("corridor_trial_forward_intent_mps", 0.04)
        self.declare_parameter("corridor_trial_max_linear_mps", 0.24)
        self.declare_parameter("corridor_trial_max_angular_radps", 0.16)
        self.declare_parameter("corridor_trial_wall_turn_limit_radps", 0.06)
        self.declare_parameter("corridor_trial_progress_window_s", 2.50)
        self.declare_parameter("corridor_trial_min_forward_progress_m", 0.04)
        self.declare_parameter("corridor_trial_blocked_front_p10_m", 0.30)
        self.declare_parameter("corridor_trial_blocked_stable_s", 0.80)
        self.declare_parameter("corridor_trial_stop_s", 0.40)
        self.declare_parameter("corridor_trial_rear_clear_m", 0.18)
        self.declare_parameter("corridor_trial_max_recoveries", 2)
        self.declare_parameter("corridor_trial_odom_timeout_s", 0.60)
        self.declare_parameter("enable_corridor_deadend_turn", True)
        self.declare_parameter("corridor_deadend_side_clear_m", 0.45)
        self.declare_parameter("corridor_deadend_side_turn_degrees", 35.0)
        self.declare_parameter("corridor_deadend_side_turn_angular_z", 0.25)
        self.declare_parameter("corridor_deadend_turnaround_degrees", 180.0)
        self.declare_parameter("corridor_deadend_turnaround_angular_z", 0.35)
        self.declare_parameter("enable_escape_reverse", True)
        self.declare_parameter("escape_reverse_trigger_m", 0.16)
        self.declare_parameter("escape_reverse_clear_m", 0.24)
        self.declare_parameter("escape_reverse_linear_x", -0.14)
        self.declare_parameter("escape_reverse_angular_z", 0.20)
        self.declare_parameter("escape_reverse_max_s", 0.80)
        self.declare_parameter("escape_reverse_cooldown_s", 0.40)
        self.declare_parameter("hard_stop_m", 1.00)
        self.declare_parameter("slow_down_m", 1.60)
        self.declare_parameter("slow_clear_m", 1.75)
        self.declare_parameter("emergency_stop_m", 0.45)
        self.declare_parameter("soft_max_linear", 0.30)
        self.declare_parameter("clear_max_linear", 0.30)
        self.declare_parameter("min_effective_forward", 0.12)
        self.declare_parameter("approach_stop_m", 1.60)
        self.declare_parameter("approach_rate_stop_mps", 0.35)
        self.declare_parameter("ttc_stop_s", 1.20)
        self.declare_parameter("enable_dynamic_stop", False)
        self.declare_parameter("hard_stop_latch_s", 1.50)
        self.declare_parameter("min_front_valid_count", 3)
        self.declare_parameter("min_valid_range_m", 0.05)
        self.declare_parameter("max_valid_range_m", 8.0)
        self.declare_parameter("scan_timeout_s", 0.6)
        self.declare_parameter("cmd_timeout_s", 0.35)
        self.declare_parameter("output_rate_hz", 50.0)
        self.declare_parameter("event_rate_hz", 1.0)
        self.declare_parameter("fail_closed_without_scan", True)
        self.declare_parameter("publish_events", True)

        self.scan_topic = str(self.get_parameter("scan_topic").value)
        self.input_cmd_topic = str(self.get_parameter("input_cmd_topic").value)
        self.output_cmd_topic = str(self.get_parameter("output_cmd_topic").value)
        self.odom_topic = str(self.get_parameter("odom_topic").value)
        self.status_topic = str(self.get_parameter("status_topic").value)
        self.event_topic = str(self.get_parameter("event_topic").value)
        self.stop_request_topic = str(self.get_parameter("stop_request_topic").value)
        self.front_sector_deg = float(self.get_parameter("front_sector_deg").value)
        self.front_collision_corridor_half_width_m = float(
            self.get_parameter("front_collision_corridor_half_width_m").value
        )
        self.front_collision_min_x_m = float(
            self.get_parameter("front_collision_min_x_m").value
        )
        self.micro_adjust_sector_deg = float(self.get_parameter("micro_adjust_sector_deg").value)
        self.micro_adjust_trigger_m = float(self.get_parameter("micro_adjust_trigger_m").value)
        self.micro_adjust_clear_m = float(self.get_parameter("micro_adjust_clear_m").value)
        self.micro_adjust_min_valid_range_m = float(
            self.get_parameter("micro_adjust_min_valid_range_m").value
        )
        self.micro_adjust_angular_z = float(self.get_parameter("micro_adjust_angular_z").value)
        self.micro_adjust_direction_deadband_m = float(
            self.get_parameter("micro_adjust_direction_deadband_m").value
        )
        self.micro_adjust_direction_latch_s = float(
            self.get_parameter("micro_adjust_direction_latch_s").value
        )
        self.enable_micro_adjust = self._param_bool(
            self.get_parameter("enable_micro_adjust").value
        )
        self.enable_spin_escape = self._param_bool(
            self.get_parameter("enable_spin_escape").value
        )
        self.spin_escape_turn_changes = int(
            self.get_parameter("spin_escape_turn_changes").value
        )
        self.spin_escape_degrees = float(self.get_parameter("spin_escape_degrees").value)
        self.spin_escape_angular_z = float(self.get_parameter("spin_escape_angular_z").value)
        self.spin_escape_cooldown_s = float(self.get_parameter("spin_escape_cooldown_s").value)
        self.enable_micro_adjust_stuck_spin_escape = self._param_bool(
            self.get_parameter("enable_micro_adjust_stuck_spin_escape").value
        )
        self.micro_adjust_stuck_spin_min_s = float(
            self.get_parameter("micro_adjust_stuck_spin_min_s").value
        )
        self.micro_adjust_stuck_spin_front_blocked_m = float(
            self.get_parameter("micro_adjust_stuck_spin_front_blocked_m").value
        )
        self.micro_adjust_stuck_spin_clear_m = float(
            self.get_parameter("micro_adjust_stuck_spin_clear_m").value
        )
        self.micro_adjust_stuck_spin_cmd_angular_mps = float(
            self.get_parameter("micro_adjust_stuck_spin_cmd_angular_mps").value
        )
        self.enable_corridor_stuck_spin_escape = self._param_bool(
            self.get_parameter("enable_corridor_stuck_spin_escape").value
        )
        self.corridor_stuck_spin_trigger_m = float(
            self.get_parameter("corridor_stuck_spin_trigger_m").value
        )
        self.corridor_stuck_spin_clear_m = float(
            self.get_parameter("corridor_stuck_spin_clear_m").value
        )
        self.corridor_stuck_spin_min_s = float(
            self.get_parameter("corridor_stuck_spin_min_s").value
        )
        self.corridor_stuck_spin_cmd_angular_mps = float(
            self.get_parameter("corridor_stuck_spin_cmd_angular_mps").value
        )
        self.corridor_stuck_spin_front_blocked_m = float(
            self.get_parameter("corridor_stuck_spin_front_blocked_m").value
        )
        self.corridor_stuck_spin_front_sector_deg = float(
            self.get_parameter("corridor_stuck_spin_front_sector_deg").value
        )
        self.corridor_stuck_spin_require_sides = self._param_bool(
            self.get_parameter("corridor_stuck_spin_require_sides").value
        )
        self.corridor_stuck_spin_side_blocked_m = float(
            self.get_parameter("corridor_stuck_spin_side_blocked_m").value
        )
        self.enable_corridor_trial = self._param_bool(
            self.get_parameter("enable_corridor_trial").value
        )
        self.corridor_trial_center_half_width_m = float(
            self.get_parameter("corridor_trial_center_half_width_m").value
        )
        self.corridor_trial_enter_front_p10_m = float(
            self.get_parameter("corridor_trial_enter_front_p10_m").value
        )
        self.corridor_trial_keep_front_p10_m = float(
            self.get_parameter("corridor_trial_keep_front_p10_m").value
        )
        self.corridor_trial_side_near_m = float(
            self.get_parameter("corridor_trial_side_near_m").value
        )
        self.corridor_trial_enter_stable_s = float(
            self.get_parameter("corridor_trial_enter_stable_s").value
        )
        self.corridor_trial_exit_stable_s = float(
            self.get_parameter("corridor_trial_exit_stable_s").value
        )
        self.corridor_trial_forward_intent_mps = float(
            self.get_parameter("corridor_trial_forward_intent_mps").value
        )
        self.corridor_trial_max_linear_mps = float(
            self.get_parameter("corridor_trial_max_linear_mps").value
        )
        self.corridor_trial_max_angular_radps = float(
            self.get_parameter("corridor_trial_max_angular_radps").value
        )
        self.corridor_trial_wall_turn_limit_radps = float(
            self.get_parameter("corridor_trial_wall_turn_limit_radps").value
        )
        self.corridor_trial_progress_window_s = float(
            self.get_parameter("corridor_trial_progress_window_s").value
        )
        self.corridor_trial_min_forward_progress_m = float(
            self.get_parameter("corridor_trial_min_forward_progress_m").value
        )
        self.corridor_trial_blocked_front_p10_m = float(
            self.get_parameter("corridor_trial_blocked_front_p10_m").value
        )
        self.corridor_trial_blocked_stable_s = float(
            self.get_parameter("corridor_trial_blocked_stable_s").value
        )
        self.corridor_trial_stop_s = float(self.get_parameter("corridor_trial_stop_s").value)
        self.corridor_trial_rear_clear_m = float(
            self.get_parameter("corridor_trial_rear_clear_m").value
        )
        self.corridor_trial_max_recoveries = int(
            self.get_parameter("corridor_trial_max_recoveries").value
        )
        self.corridor_trial_odom_timeout_s = float(
            self.get_parameter("corridor_trial_odom_timeout_s").value
        )
        self.enable_corridor_deadend_turn = self._param_bool(
            self.get_parameter("enable_corridor_deadend_turn").value
        )
        self.corridor_deadend_side_clear_m = float(
            self.get_parameter("corridor_deadend_side_clear_m").value
        )
        self.corridor_deadend_side_turn_degrees = float(
            self.get_parameter("corridor_deadend_side_turn_degrees").value
        )
        self.corridor_deadend_side_turn_angular_z = float(
            self.get_parameter("corridor_deadend_side_turn_angular_z").value
        )
        self.corridor_deadend_turnaround_degrees = float(
            self.get_parameter("corridor_deadend_turnaround_degrees").value
        )
        self.corridor_deadend_turnaround_angular_z = float(
            self.get_parameter("corridor_deadend_turnaround_angular_z").value
        )
        self.enable_escape_reverse = self._param_bool(
            self.get_parameter("enable_escape_reverse").value
        )
        self.escape_reverse_trigger_m = float(
            self.get_parameter("escape_reverse_trigger_m").value
        )
        self.escape_reverse_clear_m = float(self.get_parameter("escape_reverse_clear_m").value)
        self.escape_reverse_linear_x = float(self.get_parameter("escape_reverse_linear_x").value)
        self.escape_reverse_angular_z = float(self.get_parameter("escape_reverse_angular_z").value)
        self.escape_reverse_max_s = float(self.get_parameter("escape_reverse_max_s").value)
        self.escape_reverse_cooldown_s = float(
            self.get_parameter("escape_reverse_cooldown_s").value
        )
        self.hard_stop_m = float(self.get_parameter("hard_stop_m").value)
        self.slow_down_m = float(self.get_parameter("slow_down_m").value)
        self.slow_clear_m = float(self.get_parameter("slow_clear_m").value)
        self.emergency_stop_m = float(self.get_parameter("emergency_stop_m").value)
        self.soft_max_linear = float(self.get_parameter("soft_max_linear").value)
        self.clear_max_linear = float(self.get_parameter("clear_max_linear").value)
        self.min_effective_forward = float(self.get_parameter("min_effective_forward").value)
        self.approach_stop_m = float(self.get_parameter("approach_stop_m").value)
        self.approach_rate_stop_mps = float(self.get_parameter("approach_rate_stop_mps").value)
        self.ttc_stop_s = float(self.get_parameter("ttc_stop_s").value)
        self.enable_dynamic_stop = self._param_bool(self.get_parameter("enable_dynamic_stop").value)
        self.hard_stop_latch_s = float(self.get_parameter("hard_stop_latch_s").value)
        self.min_front_valid_count = int(self.get_parameter("min_front_valid_count").value)
        self.min_valid_range_m = float(self.get_parameter("min_valid_range_m").value)
        self.max_valid_range_m = float(self.get_parameter("max_valid_range_m").value)
        self.scan_timeout_s = float(self.get_parameter("scan_timeout_s").value)
        self.cmd_timeout_s = float(self.get_parameter("cmd_timeout_s").value)
        self.event_period_s = 1.0 / max(float(self.get_parameter("event_rate_hz").value), 0.1)
        output_period_s = 1.0 / max(float(self.get_parameter("output_rate_hz").value), 1.0)
        self.fail_closed_without_scan = self._param_bool(
            self.get_parameter("fail_closed_without_scan").value
        )
        self.publish_events = self._param_bool(self.get_parameter("publish_events").value)

        self.cmd_pub = self.create_publisher(Twist, self.output_cmd_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.event_pub = self.create_publisher(String, self.event_topic, 10)
        self.stop_request_pub = self.create_publisher(String, self.stop_request_topic, 10)
        self.create_subscription(LaserScan, self.scan_topic, self._scan_cb, 10)
        self.create_subscription(Twist, self.input_cmd_topic, self._cmd_cb, 10)
        self.create_subscription(Odometry, self.odom_topic, self._odom_cb, 10)
        self.create_timer(output_period_s, self._output_tick)

        self.last_cmd = Twist()
        self.last_cmd_time = 0.0
        self.last_scan_time = 0.0
        self.front_min = None
        self.front_p10 = None
        self.front_valid_count = 0
        self.corridor_front_min = None
        self.corridor_front_p10 = None
        self.corridor_front_valid_count = 0
        self.corridor_trial_front_min = None
        self.corridor_trial_front_p10 = None
        self.corridor_trial_front_valid_count = 0
        self.rear_corridor_min = None
        self.rear_corridor_p10 = None
        self.rear_corridor_valid_count = 0
        self.deadend_front_min = None
        self.deadend_front_p10 = None
        self.deadend_front_valid_count = 0
        self.micro_front_min = None
        self.micro_left_min = None
        self.micro_right_min = None
        self.prev_front_p10 = None
        self.prev_front_time = 0.0
        self.approach_rate_mps = 0.0
        self.ttc_s = math.inf
        self.hard_stop_latch_until = 0.0
        self.micro_adjust_turn_sign = 1.0
        self.micro_adjust_turn_latch_until = 0.0
        self.micro_adjust_last_turn_sign = 0.0
        self.micro_adjust_turn_change_count = 0
        self.micro_adjust_stuck_since = 0.0
        self.corridor_stuck_since = 0.0
        self.corridor_trial_candidate_since = 0.0
        self.corridor_trial_exit_since = 0.0
        self.corridor_trial_blocked_since = 0.0
        self.corridor_trial_enter_time = 0.0
        self.corridor_trial_stuck_until = 0.0
        self.corridor_trial_recoveries = 0
        self.corridor_deadend_side_turn_until = 0.0
        self.corridor_deadend_side_turn_sign = 1.0
        self.corridor_deadend_side_turn_tried = set()
        self.corridor_deadend_turnaround_until = 0.0
        self.corridor_deadend_turnaround_sign = 1.0
        self.odom_history = []
        self.last_odom_time = 0.0
        self.spin_escape_until = 0.0
        self.spin_escape_cooldown_until = 0.0
        self.spin_escape_turn_sign = 1.0
        self.escape_reverse_until = 0.0
        self.escape_reverse_cooldown_until = 0.0
        self.escape_reverse_turn_sign = 1.0
        self.state = "stale_scan" if self.fail_closed_without_scan else "clear"
        self.last_logged_state = self.state
        self.last_status_json = None
        self.last_event_time = 0.0
        self.last_status_time = 0.0
        self.last_diagnostic_log_time = 0.0
        self.last_stop_request_time = 0.0
        self.hard_stop_stop_request_sent = False
        self.add_on_set_parameters_callback(self._on_parameters)

        self.get_logger().info(
            "Scan safety guard: %s -> %s using %s, hard<%.2fm, slow<%.2fm, micro<=%.2fm/%gdeg"
            % (
                self.input_cmd_topic,
                self.output_cmd_topic,
                self.scan_topic,
                self.hard_stop_m,
                self.slow_down_m,
                self.micro_adjust_trigger_m,
                self.micro_adjust_sector_deg,
            )
        )

    @staticmethod
    def _param_bool(value) -> bool:
        if isinstance(value, bool):
            return value
        return str(value).strip().lower() in ("1", "true", "yes", "on")

    def _on_parameters(self, parameters):
        try:
            for param in parameters:
                name = param.name
                value = param.value
                if name == "front_collision_corridor_half_width_m":
                    self.front_collision_corridor_half_width_m = float(value)
                elif name == "front_collision_min_x_m":
                    self.front_collision_min_x_m = float(value)
                elif name == "micro_adjust_sector_deg":
                    self.micro_adjust_sector_deg = float(value)
                elif name == "micro_adjust_trigger_m":
                    self.micro_adjust_trigger_m = float(value)
                elif name == "micro_adjust_clear_m":
                    self.micro_adjust_clear_m = float(value)
                elif name == "micro_adjust_min_valid_range_m":
                    self.micro_adjust_min_valid_range_m = float(value)
                elif name == "micro_adjust_angular_z":
                    self.micro_adjust_angular_z = float(value)
                elif name == "micro_adjust_direction_deadband_m":
                    self.micro_adjust_direction_deadband_m = float(value)
                elif name == "micro_adjust_direction_latch_s":
                    self.micro_adjust_direction_latch_s = float(value)
                elif name == "enable_micro_adjust":
                    self.enable_micro_adjust = self._param_bool(value)
                elif name == "enable_spin_escape":
                    self.enable_spin_escape = self._param_bool(value)
                elif name == "spin_escape_turn_changes":
                    self.spin_escape_turn_changes = int(value)
                elif name == "spin_escape_degrees":
                    self.spin_escape_degrees = float(value)
                elif name == "spin_escape_angular_z":
                    self.spin_escape_angular_z = float(value)
                elif name == "spin_escape_cooldown_s":
                    self.spin_escape_cooldown_s = float(value)
                elif name == "enable_micro_adjust_stuck_spin_escape":
                    self.enable_micro_adjust_stuck_spin_escape = self._param_bool(value)
                elif name == "micro_adjust_stuck_spin_min_s":
                    self.micro_adjust_stuck_spin_min_s = float(value)
                elif name == "micro_adjust_stuck_spin_front_blocked_m":
                    self.micro_adjust_stuck_spin_front_blocked_m = float(value)
                elif name == "micro_adjust_stuck_spin_clear_m":
                    self.micro_adjust_stuck_spin_clear_m = float(value)
                elif name == "micro_adjust_stuck_spin_cmd_angular_mps":
                    self.micro_adjust_stuck_spin_cmd_angular_mps = float(value)
                elif name == "enable_corridor_stuck_spin_escape":
                    self.enable_corridor_stuck_spin_escape = self._param_bool(value)
                elif name == "corridor_stuck_spin_trigger_m":
                    self.corridor_stuck_spin_trigger_m = float(value)
                elif name == "corridor_stuck_spin_clear_m":
                    self.corridor_stuck_spin_clear_m = float(value)
                elif name == "corridor_stuck_spin_min_s":
                    self.corridor_stuck_spin_min_s = float(value)
                elif name == "corridor_stuck_spin_cmd_angular_mps":
                    self.corridor_stuck_spin_cmd_angular_mps = float(value)
                elif name == "corridor_stuck_spin_front_blocked_m":
                    self.corridor_stuck_spin_front_blocked_m = float(value)
                elif name == "corridor_stuck_spin_front_sector_deg":
                    self.corridor_stuck_spin_front_sector_deg = float(value)
                elif name == "corridor_stuck_spin_require_sides":
                    self.corridor_stuck_spin_require_sides = self._param_bool(value)
                elif name == "corridor_stuck_spin_side_blocked_m":
                    self.corridor_stuck_spin_side_blocked_m = float(value)
                elif name == "enable_corridor_trial":
                    self.enable_corridor_trial = self._param_bool(value)
                elif name == "corridor_trial_center_half_width_m":
                    self.corridor_trial_center_half_width_m = float(value)
                elif name == "corridor_trial_enter_front_p10_m":
                    self.corridor_trial_enter_front_p10_m = float(value)
                elif name == "corridor_trial_keep_front_p10_m":
                    self.corridor_trial_keep_front_p10_m = float(value)
                elif name == "corridor_trial_side_near_m":
                    self.corridor_trial_side_near_m = float(value)
                elif name == "corridor_trial_enter_stable_s":
                    self.corridor_trial_enter_stable_s = float(value)
                elif name == "corridor_trial_exit_stable_s":
                    self.corridor_trial_exit_stable_s = float(value)
                elif name == "corridor_trial_forward_intent_mps":
                    self.corridor_trial_forward_intent_mps = float(value)
                elif name == "corridor_trial_max_linear_mps":
                    self.corridor_trial_max_linear_mps = float(value)
                elif name == "corridor_trial_max_angular_radps":
                    self.corridor_trial_max_angular_radps = float(value)
                elif name == "corridor_trial_wall_turn_limit_radps":
                    self.corridor_trial_wall_turn_limit_radps = float(value)
                elif name == "corridor_trial_progress_window_s":
                    self.corridor_trial_progress_window_s = float(value)
                elif name == "corridor_trial_min_forward_progress_m":
                    self.corridor_trial_min_forward_progress_m = float(value)
                elif name == "corridor_trial_blocked_front_p10_m":
                    self.corridor_trial_blocked_front_p10_m = float(value)
                elif name == "corridor_trial_blocked_stable_s":
                    self.corridor_trial_blocked_stable_s = float(value)
                elif name == "corridor_trial_stop_s":
                    self.corridor_trial_stop_s = float(value)
                elif name == "corridor_trial_rear_clear_m":
                    self.corridor_trial_rear_clear_m = float(value)
                elif name == "corridor_trial_max_recoveries":
                    self.corridor_trial_max_recoveries = int(value)
                elif name == "corridor_trial_odom_timeout_s":
                    self.corridor_trial_odom_timeout_s = float(value)
                elif name == "enable_corridor_deadend_turn":
                    self.enable_corridor_deadend_turn = self._param_bool(value)
                elif name == "corridor_deadend_side_clear_m":
                    self.corridor_deadend_side_clear_m = float(value)
                elif name == "corridor_deadend_side_turn_degrees":
                    self.corridor_deadend_side_turn_degrees = float(value)
                elif name == "corridor_deadend_side_turn_angular_z":
                    self.corridor_deadend_side_turn_angular_z = float(value)
                elif name == "corridor_deadend_turnaround_degrees":
                    self.corridor_deadend_turnaround_degrees = float(value)
                elif name == "corridor_deadend_turnaround_angular_z":
                    self.corridor_deadend_turnaround_angular_z = float(value)
                elif name == "enable_escape_reverse":
                    self.enable_escape_reverse = self._param_bool(value)
                elif name == "escape_reverse_trigger_m":
                    self.escape_reverse_trigger_m = float(value)
                elif name == "escape_reverse_clear_m":
                    self.escape_reverse_clear_m = float(value)
                elif name == "escape_reverse_linear_x":
                    self.escape_reverse_linear_x = float(value)
                elif name == "escape_reverse_angular_z":
                    self.escape_reverse_angular_z = float(value)
                elif name == "escape_reverse_max_s":
                    self.escape_reverse_max_s = float(value)
                elif name == "escape_reverse_cooldown_s":
                    self.escape_reverse_cooldown_s = float(value)
                elif name == "hard_stop_m":
                    self.hard_stop_m = float(value)
                elif name == "slow_down_m":
                    self.slow_down_m = float(value)
                elif name == "slow_clear_m":
                    self.slow_clear_m = float(value)
                elif name == "emergency_stop_m":
                    self.emergency_stop_m = float(value)
                elif name == "soft_max_linear":
                    self.soft_max_linear = float(value)
                elif name == "clear_max_linear":
                    self.clear_max_linear = float(value)
                elif name == "min_effective_forward":
                    self.min_effective_forward = float(value)
                elif name == "approach_stop_m":
                    self.approach_stop_m = float(value)
                elif name == "approach_rate_stop_mps":
                    self.approach_rate_stop_mps = float(value)
                elif name == "ttc_stop_s":
                    self.ttc_stop_s = float(value)
                elif name == "enable_dynamic_stop":
                    self.enable_dynamic_stop = self._param_bool(value)
                elif name == "hard_stop_latch_s":
                    self.hard_stop_latch_s = float(value)
            return SetParametersResult(successful=True)
        except Exception as exc:
            return SetParametersResult(successful=False, reason=str(exc))

    def _cmd_cb(self, msg: Twist) -> None:
        self.last_cmd = msg
        self.last_cmd_time = time.monotonic()

    def _odom_cb(self, msg: Odometry) -> None:
        now = time.monotonic()
        pose = msg.pose.pose
        yaw = self._yaw_from_quat(pose.orientation)
        self.odom_history.append((now, float(pose.position.x), float(pose.position.y), yaw))
        keep_after = now - max(5.0, self.corridor_trial_progress_window_s + 1.0)
        while self.odom_history and self.odom_history[0][0] < keep_after:
            self.odom_history.pop(0)
        self.last_odom_time = now

    def _scan_cb(self, scan: LaserScan) -> None:
        try:
            now = time.monotonic()
            stats = self._front_stats(scan)
            self.front_min = stats["front_min"]
            self.front_p10 = stats["front_p10"]
            self.front_valid_count = stats["valid_count"]
            self.corridor_front_min = stats["corridor_front_min"]
            self.corridor_front_p10 = stats["corridor_front_p10"]
            self.corridor_front_valid_count = stats["corridor_valid_count"]
            self.corridor_trial_front_min = stats["corridor_trial_front_min"]
            self.corridor_trial_front_p10 = stats["corridor_trial_front_p10"]
            self.corridor_trial_front_valid_count = stats["corridor_trial_valid_count"]
            self.rear_corridor_min = stats["rear_corridor_min"]
            self.rear_corridor_p10 = stats["rear_corridor_p10"]
            self.rear_corridor_valid_count = stats["rear_corridor_valid_count"]
            self.deadend_front_min = stats["deadend_front_min"]
            self.deadend_front_p10 = stats["deadend_front_p10"]
            self.deadend_front_valid_count = stats["deadend_front_valid_count"]
            self.micro_front_min = stats["micro_front_min"]
            self.micro_left_min = stats["micro_left_min"]
            self.micro_right_min = stats["micro_right_min"]
            self._update_approach_rate(self.corridor_front_p10, now)
            self._update_ttc()
            self.last_scan_time = now
            self._update_state(now)
        except Exception as exc:
            self.get_logger().warn(f"Cannot process LaserScan frame: {exc}")

    def _output_tick(self) -> None:
        now = time.monotonic()
        cmd_fresh = now - self.last_cmd_time <= self.cmd_timeout_s
        scan_fresh = now - self.last_scan_time <= self.scan_timeout_s

        if not scan_fresh and self.fail_closed_without_scan:
            self.state = "stale_scan"
            self._log_state_transition("scan_timeout")

        source = self.last_cmd if cmd_fresh else Twist()
        filtered, action = self._filter_cmd(source, scan_fresh, now)
        self.cmd_pub.publish(filtered)
        self._publish_status(now, scan_fresh, cmd_fresh, action)

    def _filter_cmd(self, source: Twist, scan_fresh: bool, now: float):
        result = Twist()
        result.linear.x = float(source.linear.x)
        result.linear.y = float(source.linear.y)
        result.linear.z = float(source.linear.z)
        result.angular.x = float(source.angular.x)
        result.angular.y = float(source.angular.y)
        result.angular.z = float(source.angular.z)

        motion_requested = abs(result.linear.x) > 1e-6 or abs(result.angular.z) > 1e-6

        if not scan_fresh and self.fail_closed_without_scan:
            if motion_requested:
                self._publish_stop_request(now, "stale_scan", source)
            return Twist(), "stale_scan_zero"

        if self.state == "hard_stop" and motion_requested:
            if not self.hard_stop_stop_request_sent:
                self._publish_stop_request(now, "hard_stop", source)
                self.hard_stop_stop_request_sent = True
                return Twist(), "hard_stop_stop_request"
            return Twist(), "hard_stop_latched_zero"

        if self.state == "corridor_stuck":
            return Twist(), "corridor_stuck_zero"

        if self.state == "corridor_side_turn":
            return self._corridor_side_turn_cmd(), "corridor_side_turn"

        if self.state == "corridor_turnaround":
            return self._corridor_turnaround_cmd(), "corridor_turnaround"

        if self.state == "escape_reverse":
            action = "escape_reverse" if motion_requested else "escape_reverse_autonomous"
            return self._escape_reverse_cmd(source), action

        if self.state == "spin_escape":
            action = "spin_escape" if motion_requested else "spin_escape_autonomous"
            return self._spin_escape_cmd(), action

        if self.state == "micro_adjust":
            action = "micro_adjust_rotate" if motion_requested else "micro_adjust_autonomous_rotate"
            return self._micro_adjust_cmd(result, record_turn_change=motion_requested), action

        if self.state == "corridor_trial":
            return self._corridor_trial_cmd(result), "corridor_trial_limit"

        if result.linear.x > 0.0:
            if result.linear.x < self.min_effective_forward:
                return Twist(), "below_min_motion_zero"
            limit = self._forward_limit()
            if limit <= 0.0:
                return Twist(), "distance_brake_zero"
            if result.linear.x > limit:
                result.linear.x = limit
                return result, "limit_forward_speed"

        return result, "pass"

    def _publish_stop_request(self, now: float, reason: str, source: Twist) -> None:
        if now - self.last_stop_request_time < 0.10:
            return
        request = {
            "request": "STOP_REQUEST",
            "reason": reason,
            "source": "scan_safety_guard",
            "input_linear_x": round(float(source.linear.x), 3),
            "input_angular_z": round(float(source.angular.z), 3),
            "front_min_range_m": None if self.front_min is None else round(float(self.front_min), 3),
            "front_p10_range_m": None if self.front_p10 is None else round(float(self.front_p10), 3),
            "front_valid_count": int(self.front_valid_count),
            "corridor_front_min_m": (
                None if self.corridor_front_min is None else round(float(self.corridor_front_min), 3)
            ),
            "corridor_front_p10_m": (
                None if self.corridor_front_p10 is None else round(float(self.corridor_front_p10), 3)
            ),
            "corridor_front_valid_count": int(self.corridor_front_valid_count),
            "rear_corridor_min_m": (
                None if self.rear_corridor_min is None else round(float(self.rear_corridor_min), 3)
            ),
            "rear_corridor_p10_m": (
                None if self.rear_corridor_p10 is None else round(float(self.rear_corridor_p10), 3)
            ),
            "rear_corridor_valid_count": int(self.rear_corridor_valid_count),
            "deadend_front_min_m": (
                None if self.deadend_front_min is None else round(float(self.deadend_front_min), 3)
            ),
            "deadend_front_p10_m": (
                None if self.deadend_front_p10 is None else round(float(self.deadend_front_p10), 3)
            ),
            "deadend_front_valid_count": int(self.deadend_front_valid_count),
            "micro_front_min_range_m": (
                None if self.micro_front_min is None else round(float(self.micro_front_min), 3)
            ),
            "micro_left_min_range_m": (
                None if self.micro_left_min is None else round(float(self.micro_left_min), 3)
            ),
            "micro_right_min_range_m": (
                None if self.micro_right_min is None else round(float(self.micro_right_min), 3)
            ),
            "approach_rate_mps": round(float(self.approach_rate_mps), 3),
            "ttc_s": None if math.isinf(self.ttc_s) else round(float(self.ttc_s), 3),
            "timestamp": datetime.now(timezone.utc).isoformat(),
        }
        msg = String()
        msg.data = json.dumps(request, ensure_ascii=False)
        self.stop_request_pub.publish(msg)
        self.last_stop_request_time = now

    def _forward_limit(self) -> float:
        if self.front_min is None and self.corridor_front_min is None:
            return self.clear_max_linear
        if self._hard_condition():
            return 0.0
        if self.state in (
            "warning",
            "micro_adjust",
            "escape_reverse",
            "spin_escape",
            "corridor_side_turn",
            "corridor_turnaround",
        ):
            return self.soft_max_linear
        return self.clear_max_linear

    def _update_state(self, now: float) -> None:
        old_state = self.state
        if self._hard_condition():
            if old_state != "hard_stop":
                self.hard_stop_stop_request_sent = False
            self.state = "hard_stop"
            self.hard_stop_latch_until = max(
                self.hard_stop_latch_until, now + self.hard_stop_latch_s
            )
            if self.state != old_state:
                self._log_state_transition("hard_condition")
            return

        if self.state == "corridor_stuck":
            if now < self.corridor_trial_stuck_until:
                self._log_state_transition("corridor_stuck_hold")
                return
            if self.enable_corridor_deadend_turn:
                if self._begin_corridor_side_turn(now):
                    self.state = "corridor_side_turn"
                    self.hard_stop_stop_request_sent = False
                    self._log_state_transition("corridor_deadend_side_turn")
                    return
                self._begin_corridor_turnaround(now)
                self.state = "corridor_turnaround"
                self.hard_stop_stop_request_sent = False
                self._log_state_transition("corridor_deadend_turnaround")
                return
            if self._rear_corridor_clear() and self._begin_corridor_recovery(now):
                self.state = "escape_reverse"
                self.hard_stop_stop_request_sent = False
                self._log_state_transition("corridor_stuck_recover")
                return
            self.state = "hard_stop"
            self.hard_stop_latch_until = max(
                self.hard_stop_latch_until, now + self.hard_stop_latch_s
            )
            self._publish_stop_request(now, "corridor_stuck_no_rear_clear", self.last_cmd)
            self._log_state_transition("corridor_stuck_no_rear_clear")
            return

        if self.state == "corridor_side_turn":
            if now < self.corridor_deadend_side_turn_until:
                return
            if self._corridor_front_open(self.corridor_trial_keep_front_p10_m):
                self.state = "clear"
                self.corridor_trial_blocked_since = 0.0
                self._log_state_transition("corridor_side_turn_open")
                return
            if self._begin_corridor_side_turn(now):
                self._log_state_transition("corridor_deadend_side_turn_next")
                return
            self._begin_corridor_turnaround(now)
            self.state = "corridor_turnaround"
            self.hard_stop_stop_request_sent = False
            self._log_state_transition("corridor_side_turn_no_open")
            return

        if self.state == "corridor_turnaround":
            if now < self.corridor_deadend_turnaround_until:
                return
            self.state = "clear"
            self.corridor_deadend_side_turn_tried.clear()
            self.corridor_trial_blocked_since = 0.0
            self._log_state_transition("corridor_turnaround_done")
            return

        if self.state == "corridor_trial" and self._corridor_trial_stuck_condition(now):
            self.state = "corridor_stuck"
            self.corridor_trial_stuck_until = now + max(0.0, self.corridor_trial_stop_s)
            self._publish_stop_request(now, "corridor_trial_no_progress", self.last_cmd)
            self._log_state_transition("corridor_trial_no_progress")
            return

        if self._corridor_trial_condition(now):
            self.state = "corridor_trial"
            self.hard_stop_stop_request_sent = False
            self._reset_micro_adjust_turn_changes()
            if self.state != old_state:
                self.corridor_trial_enter_time = now
                self.corridor_trial_blocked_since = 0.0
                self._log_state_transition("corridor_trial")
            if self._corridor_trial_stuck_condition(now):
                self.state = "corridor_stuck"
                self.corridor_trial_stuck_until = now + max(0.0, self.corridor_trial_stop_s)
                self._publish_stop_request(now, "corridor_trial_no_progress", self.last_cmd)
                self._log_state_transition("corridor_trial_no_progress")
            return

        if old_state == "corridor_trial":
            self.corridor_trial_enter_time = 0.0
            self.corridor_trial_blocked_since = 0.0

        if self._spin_escape_condition(now):
            self.state = "spin_escape"
            self.hard_stop_stop_request_sent = False
            if self.state != old_state:
                self._log_state_transition("spin_escape")
            return

        if self._micro_adjust_stuck_spin_condition(now):
            self.state = "spin_escape"
            self.hard_stop_stop_request_sent = False
            if self.state != old_state:
                self._log_state_transition("micro_adjust_stuck_spin")
            return

        if self._corridor_stuck_spin_condition(now):
            self.state = "spin_escape"
            self.hard_stop_stop_request_sent = False
            if self.state != old_state:
                self._log_state_transition("corridor_stuck_spin")
            return

        if self._escape_reverse_condition(now):
            self.state = "escape_reverse"
            self.hard_stop_stop_request_sent = False
            if self.state != old_state:
                self._log_state_transition("escape_reverse")
            return

        if self._micro_adjust_condition():
            self.state = "micro_adjust"
            self.hard_stop_stop_request_sent = False
            if self.state != old_state:
                self._log_state_transition("micro_adjust")
            return

        if self.front_valid_count < self.min_front_valid_count or self.front_p10 is None:
            return

        if self.state == "hard_stop":
            if now < self.hard_stop_latch_until:
                self._log_state_transition("latched")
                return
            self.state = "warning" if self.front_p10 < self.slow_clear_m else "clear"
            self.hard_stop_stop_request_sent = False
            self._log_state_transition("scan")
            return
        if self.state == "micro_adjust":
            self.state = "warning" if self.front_p10 < self.slow_clear_m else "clear"
            self._log_state_transition("scan")
            return
        if self.state == "escape_reverse":
            self.state = "warning" if self.front_p10 < self.slow_clear_m else "clear"
            self._log_state_transition("scan")
            return
        if self.state == "spin_escape":
            self.state = "warning" if self.front_p10 < self.slow_clear_m else "clear"
            self._log_state_transition("scan")
            return
        if self.state == "corridor_stuck":
            return
        if self.front_p10 < self.slow_down_m:
            self.state = "warning"
        else:
            self.state = "clear"
        if self.state == "clear" and not self._micro_adjust_condition():
            self._reset_micro_adjust_turn_changes()
        if self.state != old_state:
            self._log_state_transition("scan")

    def _hard_condition(self) -> bool:
        if self.corridor_front_min is not None and self.corridor_front_min <= self.emergency_stop_m:
            return True
        if (
            self.corridor_front_p10 is None
            or self.corridor_front_valid_count < self.min_front_valid_count
        ):
            return False
        if self.corridor_front_p10 <= self.hard_stop_m:
            return True
        if not self.enable_dynamic_stop:
            return False
        forward_cmd_active = (
            time.monotonic() - self.last_cmd_time <= self.cmd_timeout_s
            and float(self.last_cmd.linear.x) > 0.0
        )
        inside_approach_zone = self.corridor_front_p10 < self.approach_stop_m
        if (
            forward_cmd_active
            and inside_approach_zone
            and self.approach_rate_mps > self.approach_rate_stop_mps
        ):
            return True
        if forward_cmd_active and inside_approach_zone and self.ttc_s < self.ttc_stop_s:
            return True
        return False

    def _forward_intent(self, now: float) -> bool:
        return (
            now - self.last_cmd_time <= self.cmd_timeout_s
            and float(self.last_cmd.linear.x) > self.corridor_trial_forward_intent_mps
        )

    def _odom_fresh(self, now: float) -> bool:
        return (
            self.last_odom_time > 0.0
            and now - self.last_odom_time <= self.corridor_trial_odom_timeout_s
        )

    def _corridor_sides_near(self) -> bool:
        return (
            self.micro_left_min is not None
            and self.micro_right_min is not None
            and self.micro_left_min <= self.corridor_trial_side_near_m
            and self.micro_right_min <= self.corridor_trial_side_near_m
        )

    def _corridor_front_open(self, threshold: float) -> bool:
        return (
            self.corridor_trial_front_p10 is not None
            and self.corridor_trial_front_valid_count >= self.min_front_valid_count
            and self.corridor_trial_front_p10 > threshold
        )

    def _spin_escape_should_abort(self) -> bool:
        open_values = [self.deadend_front_p10, self.front_p10, self.corridor_front_p10]
        return any(
            value is not None and value >= self.micro_adjust_stuck_spin_clear_m
            for value in open_values
        )

    def _corridor_trial_condition(self, now: float) -> bool:
        if not self.enable_corridor_trial:
            self.corridor_trial_candidate_since = 0.0
            self.corridor_trial_exit_since = 0.0
            return False
        if not self._forward_intent(now):
            self.corridor_trial_candidate_since = 0.0
            return False

        sides_near = self._corridor_sides_near()
        threshold = (
            self.corridor_trial_keep_front_p10_m
            if self.state == "corridor_trial"
            else self.corridor_trial_enter_front_p10_m
        )
        front_open = self._corridor_front_open(threshold)
        candidate = sides_near and front_open

        if candidate:
            self.corridor_trial_exit_since = 0.0
            if self.corridor_trial_candidate_since <= 0.0:
                self.corridor_trial_candidate_since = now
            if self.state == "corridor_trial":
                return True
            return (
                now - self.corridor_trial_candidate_since
                >= max(0.0, self.corridor_trial_enter_stable_s)
            )

        self.corridor_trial_candidate_since = 0.0
        if self.state != "corridor_trial":
            return False
        if not sides_near:
            if self.corridor_trial_exit_since <= 0.0:
                self.corridor_trial_exit_since = now
                return True
            if now - self.corridor_trial_exit_since < max(0.0, self.corridor_trial_exit_stable_s):
                return True
            self.corridor_trial_exit_since = 0.0
            self.corridor_trial_recoveries = 0
            return False
        self.corridor_trial_exit_since = 0.0
        return True

    def _corridor_trial_stuck_condition(self, now: float) -> bool:
        if self.state != "corridor_trial" or not self._forward_intent(now):
            self.corridor_trial_blocked_since = 0.0
            return False
        if not self._odom_fresh(now):
            self.corridor_trial_blocked_since = 0.0
            return False

        front_blocked = (
            self.corridor_trial_front_p10 is not None
            and self.corridor_trial_front_valid_count >= self.min_front_valid_count
            and self.corridor_trial_front_p10 < self.corridor_trial_blocked_front_p10_m
        )
        if not front_blocked:
            self.corridor_trial_blocked_since = 0.0
            return False
        if self.corridor_trial_blocked_since <= 0.0:
            self.corridor_trial_blocked_since = now
            return False
        if now - self.corridor_trial_blocked_since < max(0.0, self.corridor_trial_blocked_stable_s):
            return False

        progress = self._forward_progress_window(now, self.corridor_trial_progress_window_s)
        if progress is None:
            return False
        return progress < self.corridor_trial_min_forward_progress_m

    def _forward_progress_window(self, now: float, window_s: float):
        if len(self.odom_history) < 2:
            return None
        start_time = now - max(0.1, window_s)
        start = None
        for sample in self.odom_history:
            if sample[0] >= start_time:
                start = sample
                break
        if start is None:
            start = self.odom_history[0]
        end = self.odom_history[-1]
        if end[0] - start[0] < max(0.1, window_s * 0.75):
            return None
        _, x0, y0, yaw0 = start
        _, x1, y1, _ = end
        dx = x1 - x0
        dy = y1 - y0
        return dx * math.cos(yaw0) + dy * math.sin(yaw0)

    def _rounded_forward_progress(self, now: float):
        progress = self._forward_progress_window(now, self.corridor_trial_progress_window_s)
        return None if progress is None else round(float(progress), 3)

    def _rear_corridor_clear(self) -> bool:
        return (
            self.rear_corridor_p10 is not None
            and self.rear_corridor_valid_count >= self.min_front_valid_count
            and self.rear_corridor_p10 >= self.corridor_trial_rear_clear_m
        )

    def _begin_corridor_recovery(self, now: float) -> bool:
        if self.corridor_trial_recoveries >= max(0, self.corridor_trial_max_recoveries):
            return False
        self.corridor_trial_recoveries += 1
        self.escape_reverse_until = now + max(0.0, self.escape_reverse_max_s)
        self.escape_reverse_cooldown_until = (
            self.escape_reverse_until + max(0.0, self.escape_reverse_cooldown_s)
        )
        self.escape_reverse_turn_sign = self._micro_adjust_turn_direction()
        self.corridor_trial_blocked_since = 0.0
        return True

    def _begin_corridor_side_turn(self, now: float) -> bool:
        candidates = []
        if (
            self.micro_left_min is not None
            and self.micro_left_min >= self.corridor_deadend_side_clear_m
            and "left" not in self.corridor_deadend_side_turn_tried
        ):
            candidates.append(("left", float(self.micro_left_min), 1.0))
        if (
            self.micro_right_min is not None
            and self.micro_right_min >= self.corridor_deadend_side_clear_m
            and "right" not in self.corridor_deadend_side_turn_tried
        ):
            candidates.append(("right", float(self.micro_right_min), -1.0))
        if not candidates:
            return False
        side, _, sign = max(candidates, key=lambda item: item[1])
        angular = max(abs(self.corridor_deadend_side_turn_angular_z), 1e-3)
        degrees = max(0.0, self.corridor_deadend_side_turn_degrees)
        if self.corridor_deadend_side_turn_tried and sign != self.corridor_deadend_side_turn_sign:
            degrees *= 2.0
        duration_s = math.radians(degrees) / angular
        self.corridor_deadend_side_turn_sign = sign
        self.corridor_deadend_side_turn_until = now + duration_s
        self.corridor_deadend_side_turn_tried.add(side)
        self.corridor_trial_blocked_since = 0.0
        return True

    def _begin_corridor_turnaround(self, now: float) -> None:
        angular = max(abs(self.corridor_deadend_turnaround_angular_z), 1e-3)
        duration_s = math.radians(max(0.0, self.corridor_deadend_turnaround_degrees)) / angular
        left = -math.inf if self.micro_left_min is None else float(self.micro_left_min)
        right = -math.inf if self.micro_right_min is None else float(self.micro_right_min)
        self.corridor_deadend_turnaround_sign = 1.0 if left >= right else -1.0
        self.corridor_deadend_turnaround_until = now + duration_s
        self.corridor_trial_blocked_since = 0.0

    def _micro_adjust_condition(self) -> bool:
        if not self.enable_micro_adjust:
            return False
        candidates = [self.micro_front_min, self.micro_left_min, self.micro_right_min]
        threshold = self.micro_adjust_trigger_m
        if self.state == "micro_adjust":
            threshold = max(threshold, self.micro_adjust_clear_m)
        return any(value is not None and value <= threshold for value in candidates)

    def _escape_reverse_condition(self, now: float) -> bool:
        if not self.enable_escape_reverse:
            return False
        candidates = [self.micro_front_min, self.micro_left_min, self.micro_right_min]
        if self.state == "escape_reverse":
            if now >= self.escape_reverse_until:
                return False
            threshold = max(self.escape_reverse_trigger_m, self.escape_reverse_clear_m)
            return any(value is not None and value <= threshold for value in candidates)

        if now < self.escape_reverse_cooldown_until:
            return False
        if not any(
            value is not None and value <= self.escape_reverse_trigger_m
            for value in candidates
        ):
            return False

        self.escape_reverse_until = now + max(0.0, self.escape_reverse_max_s)
        self.escape_reverse_cooldown_until = (
            self.escape_reverse_until + max(0.0, self.escape_reverse_cooldown_s)
        )
        self.escape_reverse_turn_sign = self._micro_adjust_turn_direction()
        return True

    def _escape_reverse_cmd(self, source: Twist) -> Twist:
        result = Twist()
        result.linear.x = -abs(self.escape_reverse_linear_x)
        self.escape_reverse_turn_sign = self._micro_adjust_turn_direction()
        result.angular.z = self.escape_reverse_turn_sign * abs(self.escape_reverse_angular_z)
        return result

    def _spin_escape_condition(self, now: float) -> bool:
        if not self.enable_spin_escape:
            return False
        if self.state in ("corridor_trial", "corridor_stuck"):
            return False
        if self.state == "spin_escape":
            if self._spin_escape_should_abort():
                self.spin_escape_until = 0.0
                self.spin_escape_cooldown_until = now + max(0.0, self.spin_escape_cooldown_s)
                return False
            return now < self.spin_escape_until
        if now < self.spin_escape_cooldown_until:
            return False
        if self.micro_adjust_turn_change_count < max(1, self.spin_escape_turn_changes):
            return False
        self._begin_spin_escape(self.micro_adjust_turn_sign)
        return True

    def _micro_adjust_stuck_spin_condition(self, now: float) -> bool:
        if not self.enable_spin_escape or not self.enable_micro_adjust_stuck_spin_escape:
            self.micro_adjust_stuck_since = 0.0
            return False
        if self.state in ("corridor_trial", "corridor_stuck"):
            self.micro_adjust_stuck_since = 0.0
            return False
        if self._corridor_sides_near() and self._corridor_front_open(self.corridor_trial_keep_front_p10_m):
            self.micro_adjust_stuck_since = 0.0
            return False
        if self.state == "spin_escape":
            return now < self.spin_escape_until
        if now < self.spin_escape_cooldown_until:
            return False

        front_blocked_values = [self.deadend_front_min, self.deadend_front_p10]
        front_blocked = any(
            value is not None and value <= self.micro_adjust_stuck_spin_front_blocked_m
            for value in front_blocked_values
        )
        micro_values = [self.micro_front_min, self.micro_left_min, self.micro_right_min]
        micro_active = self.state == "micro_adjust" or any(
            value is not None and value <= self.micro_adjust_clear_m
            for value in micro_values
        )
        clearly_free = (
            not front_blocked
            and all(
                value is None or value >= self.micro_adjust_stuck_spin_clear_m
                for value in micro_values
            )
        )
        cmd_active = now - self.last_cmd_time <= self.cmd_timeout_s
        cmd_motion = (
            float(self.last_cmd.linear.x) > 0.0
            or abs(float(self.last_cmd.angular.z)) >= self.micro_adjust_stuck_spin_cmd_angular_mps
        )
        if (
            clearly_free
            or not micro_active
            or not front_blocked
            or not (cmd_active or self.state == "micro_adjust")
        ):
            self.micro_adjust_stuck_since = 0.0
            return False

        if self.micro_adjust_stuck_since <= 0.0:
            self.micro_adjust_stuck_since = now
            return False
        if now - self.micro_adjust_stuck_since < max(0.0, self.micro_adjust_stuck_spin_min_s):
            return False

        turn = self.micro_adjust_turn_sign
        if cmd_motion and abs(float(self.last_cmd.angular.z)) >= self.micro_adjust_stuck_spin_cmd_angular_mps:
            turn = 1.0 if float(self.last_cmd.angular.z) >= 0.0 else -1.0
        self._begin_spin_escape(turn)
        self.micro_adjust_stuck_since = 0.0
        return True

    def _corridor_stuck_spin_condition(self, now: float) -> bool:
        if not self.enable_spin_escape or not self.enable_corridor_stuck_spin_escape:
            self.corridor_stuck_since = 0.0
            return False
        if self.state in ("corridor_trial", "corridor_stuck"):
            self.corridor_stuck_since = 0.0
            return False
        if self._corridor_sides_near() and self._corridor_front_open(self.corridor_trial_keep_front_p10_m):
            self.corridor_stuck_since = 0.0
            return False
        if self.state == "spin_escape":
            return now < self.spin_escape_until
        if now < self.spin_escape_cooldown_until:
            return False

        close_values = [self.corridor_front_min, self.corridor_front_p10]
        close = any(
            value is not None and value <= self.corridor_stuck_spin_trigger_m
            for value in close_values
        )
        front_blocked_values = [self.deadend_front_min, self.deadend_front_p10]
        front_blocked = any(
            value is not None and value <= self.corridor_stuck_spin_front_blocked_m
            for value in front_blocked_values
        )
        sides_blocked = (
            self.micro_left_min is not None
            and self.micro_right_min is not None
            and self.micro_left_min <= self.corridor_stuck_spin_side_blocked_m
            and self.micro_right_min <= self.corridor_stuck_spin_side_blocked_m
        )
        clear = all(
            value is None or value >= self.corridor_stuck_spin_clear_m
            for value in close_values
        )
        cmd_active = now - self.last_cmd_time <= self.cmd_timeout_s
        cmd_motion = (
            float(self.last_cmd.linear.x) > 0.0
            or abs(float(self.last_cmd.angular.z)) >= self.corridor_stuck_spin_cmd_angular_mps
        )
        if (
            clear
            or not close
            or not front_blocked
            or (self.corridor_stuck_spin_require_sides and not sides_blocked)
            or not (cmd_active and cmd_motion)
        ):
            self.corridor_stuck_since = 0.0
            return False
        if self.corridor_stuck_since <= 0.0:
            self.corridor_stuck_since = now
            return False
        if now - self.corridor_stuck_since < max(0.0, self.corridor_stuck_spin_min_s):
            return False

        turn = self.micro_adjust_turn_sign
        if abs(float(self.last_cmd.angular.z)) >= self.corridor_stuck_spin_cmd_angular_mps:
            turn = 1.0 if float(self.last_cmd.angular.z) >= 0.0 else -1.0
        self._begin_spin_escape(turn)
        self.corridor_stuck_since = 0.0
        return True

    def _begin_spin_escape(self, turn: float) -> None:
        now = time.monotonic()
        angular = max(abs(self.spin_escape_angular_z), 1e-3)
        duration_s = math.radians(max(0.0, self.spin_escape_degrees)) / angular
        self.spin_escape_until = now + duration_s
        self.spin_escape_cooldown_until = self.spin_escape_until + max(
            0.0, self.spin_escape_cooldown_s
        )
        self.spin_escape_turn_sign = 1.0 if turn >= 0.0 else -1.0
        self._reset_micro_adjust_turn_changes()

    def _spin_escape_cmd(self) -> Twist:
        result = Twist()
        result.angular.z = self.spin_escape_turn_sign * abs(self.spin_escape_angular_z)
        return result

    def _corridor_side_turn_cmd(self) -> Twist:
        result = Twist()
        result.angular.z = (
            self.corridor_deadend_side_turn_sign
            * abs(self.corridor_deadend_side_turn_angular_z)
        )
        return result

    def _corridor_turnaround_cmd(self) -> Twist:
        result = Twist()
        result.angular.z = (
            self.corridor_deadend_turnaround_sign
            * abs(self.corridor_deadend_turnaround_angular_z)
        )
        return result

    def _micro_adjust_cmd(self, source: Twist, record_turn_change: bool = True) -> Twist:
        turn = self._micro_adjust_turn_direction(float(source.angular.z))
        if record_turn_change:
            self._record_micro_adjust_turn(turn)
        if self._spin_escape_condition(time.monotonic()):
            self.state = "spin_escape"
            self._log_state_transition("micro_adjust_turn_changes")
            return self._spin_escape_cmd()

        result = Twist()
        result.angular.z = turn * abs(self.micro_adjust_angular_z)
        return result

    def _corridor_trial_cmd(self, source: Twist) -> Twist:
        result = Twist()
        linear = max(0.0, float(source.linear.x))
        if 0.0 < linear < self.min_effective_forward:
            linear = self.min_effective_forward
        linear_cap = self.corridor_trial_max_linear_mps
        if linear_cap <= 0.0:
            linear_cap = self.clear_max_linear
        result.linear.x = min(linear, max(0.0, linear_cap))
        result.linear.y = 0.0
        result.linear.z = float(source.linear.z)
        result.angular.x = float(source.angular.x)
        result.angular.y = float(source.angular.y)

        angular = float(source.angular.z)
        max_angular = max(0.0, self.corridor_trial_max_angular_radps)
        if max_angular > 0.0:
            angular = max(-max_angular, min(max_angular, angular))

        wall_limit = max(0.0, self.corridor_trial_wall_turn_limit_radps)
        left_too_close = (
            self.micro_left_min is not None
            and self.micro_left_min <= self.corridor_trial_side_near_m
        )
        right_too_close = (
            self.micro_right_min is not None
            and self.micro_right_min <= self.corridor_trial_side_near_m
        )
        if left_too_close and angular > wall_limit:
            angular = wall_limit
        if right_too_close and angular < -wall_limit:
            angular = -wall_limit
        result.angular.z = angular
        return result

    def _record_micro_adjust_turn(self, turn: float) -> None:
        if self.micro_adjust_last_turn_sign == 0.0:
            self.micro_adjust_last_turn_sign = turn
            return
        if turn != self.micro_adjust_last_turn_sign:
            self.micro_adjust_turn_change_count += 1
            self.micro_adjust_last_turn_sign = turn

    def _reset_micro_adjust_turn_changes(self) -> None:
        self.micro_adjust_last_turn_sign = 0.0
        self.micro_adjust_turn_change_count = 0

    def _micro_adjust_turn_direction(self, current_angular_z: float = 0.0) -> float:
        now = time.monotonic()
        left = self.micro_left_min
        right = self.micro_right_min
        deadband = max(0.0, self.micro_adjust_direction_deadband_m)
        side_bias_threshold = max(
            self.micro_adjust_trigger_m,
            min(self.micro_adjust_clear_m, self.micro_adjust_trigger_m + 0.10),
        )
        left_close = left is not None and left <= side_bias_threshold
        right_close = right is not None and right <= side_bias_threshold

        turn = None
        if left_close and not right_close:
            turn = -1.0
        elif right_close and not left_close:
            turn = 1.0
        elif left_close and right_close and abs(left - right) >= deadband:
            turn = -1.0 if left < right else 1.0
        elif left is not None and right is not None and abs(left - right) >= deadband:
            turn = -1.0 if left < right else 1.0
        elif now < self.micro_adjust_turn_latch_until:
            turn = self.micro_adjust_turn_sign
        elif abs(current_angular_z) > 1e-3:
            turn = 1.0 if current_angular_z > 0.0 else -1.0
        else:
            turn = self.micro_adjust_turn_sign

        self.micro_adjust_turn_sign = turn
        self.micro_adjust_turn_latch_until = now + max(0.0, self.micro_adjust_direction_latch_s)
        return turn

    def _update_approach_rate(self, new_front_p10, now: float) -> None:
        if new_front_p10 is None:
            self.approach_rate_mps = 0.0
            return
        if self.prev_front_p10 is not None and self.prev_front_time > 0.0:
            dt = max(now - self.prev_front_time, 1e-3)
            # Positive means the nearest front obstacle is getting closer.
            self.approach_rate_mps = max(0.0, (self.prev_front_p10 - new_front_p10) / dt)
        self.prev_front_p10 = new_front_p10
        self.prev_front_time = now

    def _update_ttc(self) -> None:
        if self.corridor_front_p10 is None or self.approach_rate_mps <= 1e-6:
            self.ttc_s = math.inf
            return
        self.ttc_s = self.corridor_front_p10 / self.approach_rate_mps

    def _log_state_transition(self, reason: str) -> None:
        if self.state == self.last_logged_state:
            return
        front_min_text = "none" if self.front_min is None else f"{self.front_min:.3f}m"
        front_p10_text = "none" if self.front_p10 is None else f"{self.front_p10:.3f}m"
        corridor_p10_text = (
            "none" if self.corridor_front_p10 is None else f"{self.corridor_front_p10:.3f}m"
        )
        self.get_logger().info(
            "guard_state %s -> %s reason=%s front_min=%s front_p10=%s corridor_p10=%s "
            "valid_count=%d approach_rate=%.3fm/s ttc=%s latch_remaining=%.3fs"
            % (
                self.last_logged_state,
                self.state,
                reason,
                front_min_text,
                front_p10_text,
                corridor_p10_text,
                self.front_valid_count,
                self.approach_rate_mps,
                self._format_ttc(),
                self._latch_remaining(),
            )
        )
        self.last_logged_state = self.state

    def _log_diagnostics(self, now: float) -> None:
        if now - self.last_diagnostic_log_time < 1.0:
            return
        self.last_diagnostic_log_time = now
        self.get_logger().info(
            "guard_status state=%s front_min=%s front_p10=%s valid_count=%d "
            "corridor_min=%s corridor_p10=%s corridor_count=%d trial_p10=%s "
            "deadend_front=%s deadend_p10=%s deadend_count=%d "
            "micro_front=%s micro_left=%s micro_right=%s "
            "approach_rate=%.3fm/s ttc=%s latch_remaining=%.3fs "
            "micro_turn=%+.0f micro_turn_changes=%d escape_remaining=%.3fs "
            "micro_stuck=%.3fs corridor_stuck=%.3fs spin_remaining=%.3fs"
            % (
                self.state,
                "none" if self.front_min is None else f"{self.front_min:.3f}m",
                "none" if self.front_p10 is None else f"{self.front_p10:.3f}m",
                self.front_valid_count,
                "none" if self.corridor_front_min is None else f"{self.corridor_front_min:.3f}m",
                "none" if self.corridor_front_p10 is None else f"{self.corridor_front_p10:.3f}m",
                self.corridor_front_valid_count,
                (
                    "none"
                    if self.corridor_trial_front_p10 is None
                    else f"{self.corridor_trial_front_p10:.3f}m"
                ),
                "none" if self.deadend_front_min is None else f"{self.deadend_front_min:.3f}m",
                "none" if self.deadend_front_p10 is None else f"{self.deadend_front_p10:.3f}m",
                self.deadend_front_valid_count,
                "none" if self.micro_front_min is None else f"{self.micro_front_min:.3f}m",
                "none" if self.micro_left_min is None else f"{self.micro_left_min:.3f}m",
                "none" if self.micro_right_min is None else f"{self.micro_right_min:.3f}m",
                self.approach_rate_mps,
                self._format_ttc(),
                self._latch_remaining(),
                self.micro_adjust_turn_sign,
                self.micro_adjust_turn_change_count,
                max(0.0, self.escape_reverse_until - time.monotonic()),
                0.0
                if self.micro_adjust_stuck_since <= 0.0
                else now - self.micro_adjust_stuck_since,
                0.0 if self.corridor_stuck_since <= 0.0 else now - self.corridor_stuck_since,
                max(0.0, self.spin_escape_until - time.monotonic()),
            )
        )

    def _format_ttc(self) -> str:
        return "inf" if math.isinf(self.ttc_s) else f"{self.ttc_s:.3f}s"

    def _latch_remaining(self) -> float:
        return max(0.0, self.hard_stop_latch_until - time.monotonic())

    def _publish_status(self, now: float, scan_fresh: bool, cmd_fresh: bool, action: str) -> None:
        status = {
            "state": self.state,
            "action": action,
            "front_min_range_m": None if self.front_min is None else round(float(self.front_min), 3),
            "front_p10_range_m": None if self.front_p10 is None else round(float(self.front_p10), 3),
            "front_valid_count": int(self.front_valid_count),
            "corridor_front_min_m": (
                None if self.corridor_front_min is None else round(float(self.corridor_front_min), 3)
            ),
            "corridor_front_p10_m": (
                None if self.corridor_front_p10 is None else round(float(self.corridor_front_p10), 3)
            ),
            "corridor_front_valid_count": int(self.corridor_front_valid_count),
            "front_collision_corridor_half_width_m": self.front_collision_corridor_half_width_m,
            "front_collision_min_x_m": self.front_collision_min_x_m,
            "deadend_front_min_m": (
                None if self.deadend_front_min is None else round(float(self.deadend_front_min), 3)
            ),
            "deadend_front_p10_m": (
                None if self.deadend_front_p10 is None else round(float(self.deadend_front_p10), 3)
            ),
            "deadend_front_valid_count": int(self.deadend_front_valid_count),
            "corridor_stuck_spin_front_sector_deg": self.corridor_stuck_spin_front_sector_deg,
            "micro_front_min_range_m": (
                None if self.micro_front_min is None else round(float(self.micro_front_min), 3)
            ),
            "micro_left_min_range_m": (
                None if self.micro_left_min is None else round(float(self.micro_left_min), 3)
            ),
            "micro_right_min_range_m": (
                None if self.micro_right_min is None else round(float(self.micro_right_min), 3)
            ),
            "micro_adjust_sector_deg": self.micro_adjust_sector_deg,
            "micro_adjust_trigger_m": self.micro_adjust_trigger_m,
            "micro_adjust_clear_m": self.micro_adjust_clear_m,
            "micro_adjust_angular_z": self.micro_adjust_angular_z,
            "micro_adjust_turn_sign": self.micro_adjust_turn_sign,
            "micro_adjust_direction_deadband_m": self.micro_adjust_direction_deadband_m,
            "micro_adjust_direction_latch_s": self.micro_adjust_direction_latch_s,
            "enable_micro_adjust": self.enable_micro_adjust,
            "enable_spin_escape": self.enable_spin_escape,
            "spin_escape_turn_changes": self.spin_escape_turn_changes,
            "micro_adjust_turn_change_count": self.micro_adjust_turn_change_count,
            "enable_micro_adjust_stuck_spin_escape": self.enable_micro_adjust_stuck_spin_escape,
            "micro_adjust_stuck_spin_min_s": self.micro_adjust_stuck_spin_min_s,
            "micro_adjust_stuck_spin_front_blocked_m": (
                self.micro_adjust_stuck_spin_front_blocked_m
            ),
            "micro_adjust_stuck_spin_clear_m": self.micro_adjust_stuck_spin_clear_m,
            "micro_adjust_stuck_spin_elapsed_s": round(
                0.0 if self.micro_adjust_stuck_since <= 0.0 else now - self.micro_adjust_stuck_since,
                3,
            ),
            "enable_corridor_stuck_spin_escape": self.enable_corridor_stuck_spin_escape,
            "corridor_stuck_spin_trigger_m": self.corridor_stuck_spin_trigger_m,
            "corridor_stuck_spin_clear_m": self.corridor_stuck_spin_clear_m,
            "corridor_stuck_spin_min_s": self.corridor_stuck_spin_min_s,
            "corridor_stuck_spin_front_blocked_m": self.corridor_stuck_spin_front_blocked_m,
            "corridor_stuck_spin_front_blocked": any(
                value is not None and value <= self.corridor_stuck_spin_front_blocked_m
                for value in (self.deadend_front_min, self.deadend_front_p10)
            ),
            "corridor_stuck_spin_require_sides": self.corridor_stuck_spin_require_sides,
            "corridor_stuck_spin_side_blocked_m": self.corridor_stuck_spin_side_blocked_m,
            "corridor_stuck_spin_sides_blocked": (
                self.micro_left_min is not None
                and self.micro_right_min is not None
                and self.micro_left_min <= self.corridor_stuck_spin_side_blocked_m
                and self.micro_right_min <= self.corridor_stuck_spin_side_blocked_m
            ),
            "corridor_stuck_spin_elapsed_s": round(
                0.0 if self.corridor_stuck_since <= 0.0 else now - self.corridor_stuck_since,
                3,
            ),
            "enable_corridor_trial": self.enable_corridor_trial,
            "corridor_trial_enter_front_p10_m": self.corridor_trial_enter_front_p10_m,
            "corridor_trial_keep_front_p10_m": self.corridor_trial_keep_front_p10_m,
            "corridor_trial_side_near_m": self.corridor_trial_side_near_m,
            "corridor_trial_enter_stable_s": self.corridor_trial_enter_stable_s,
            "corridor_trial_exit_stable_s": self.corridor_trial_exit_stable_s,
            "corridor_trial_max_linear_mps": self.corridor_trial_max_linear_mps,
            "corridor_trial_max_angular_radps": self.corridor_trial_max_angular_radps,
            "corridor_trial_wall_turn_limit_radps": (
                self.corridor_trial_wall_turn_limit_radps
            ),
            "corridor_trial_forward_progress_m": self._rounded_forward_progress(now),
            "corridor_trial_min_forward_progress_m": (
                self.corridor_trial_min_forward_progress_m
            ),
            "corridor_trial_blocked_front_p10_m": self.corridor_trial_blocked_front_p10_m,
            "corridor_trial_blocked_elapsed_s": round(
                0.0
                if self.corridor_trial_blocked_since <= 0.0
                else now - self.corridor_trial_blocked_since,
                3,
            ),
            "corridor_trial_recoveries": int(self.corridor_trial_recoveries),
            "corridor_trial_max_recoveries": int(self.corridor_trial_max_recoveries),
            "corridor_trial_odom_fresh": self._odom_fresh(now),
            "corridor_trial_rear_clear": self._rear_corridor_clear(),
            "enable_corridor_deadend_turn": self.enable_corridor_deadend_turn,
            "corridor_deadend_side_clear_m": self.corridor_deadend_side_clear_m,
            "corridor_deadend_side_turn_degrees": self.corridor_deadend_side_turn_degrees,
            "corridor_deadend_side_turn_angular_z": self.corridor_deadend_side_turn_angular_z,
            "corridor_deadend_side_turn_sign": self.corridor_deadend_side_turn_sign,
            "corridor_deadend_side_turn_remaining_s": round(
                max(0.0, self.corridor_deadend_side_turn_until - time.monotonic()), 3
            ),
            "corridor_deadend_side_turn_tried": sorted(self.corridor_deadend_side_turn_tried),
            "corridor_deadend_turnaround_degrees": self.corridor_deadend_turnaround_degrees,
            "corridor_deadend_turnaround_angular_z": self.corridor_deadend_turnaround_angular_z,
            "corridor_deadend_turnaround_sign": self.corridor_deadend_turnaround_sign,
            "corridor_deadend_turnaround_remaining_s": round(
                max(0.0, self.corridor_deadend_turnaround_until - time.monotonic()), 3
            ),
            "spin_escape_degrees": self.spin_escape_degrees,
            "spin_escape_angular_z": self.spin_escape_angular_z,
            "spin_escape_remaining_s": round(
                max(0.0, self.spin_escape_until - time.monotonic()), 3
            ),
            "spin_escape_cooldown_remaining_s": round(
                max(0.0, self.spin_escape_cooldown_until - time.monotonic()), 3
            ),
            "enable_escape_reverse": self.enable_escape_reverse,
            "escape_reverse_trigger_m": self.escape_reverse_trigger_m,
            "escape_reverse_clear_m": self.escape_reverse_clear_m,
            "escape_reverse_linear_x": self.escape_reverse_linear_x,
            "escape_reverse_angular_z": self.escape_reverse_angular_z,
            "escape_reverse_turn_sign": self.escape_reverse_turn_sign,
            "escape_reverse_remaining_s": round(
                max(0.0, self.escape_reverse_until - time.monotonic()), 3
            ),
            "escape_reverse_cooldown_remaining_s": round(
                max(0.0, self.escape_reverse_cooldown_until - time.monotonic()), 3
            ),
            "scan_fresh": bool(scan_fresh),
            "cmd_fresh": bool(cmd_fresh),
            "odom_fresh": self._odom_fresh(now),
            "hard_stop_m": self.hard_stop_m,
            "slow_down_m": self.slow_down_m,
            "emergency_stop_m": self.emergency_stop_m,
            "clear_max_linear": self.clear_max_linear,
            "soft_max_linear": self.soft_max_linear,
            "min_effective_forward": self.min_effective_forward,
            "enable_dynamic_stop": self.enable_dynamic_stop,
            "approach_rate_mps": round(float(self.approach_rate_mps), 3),
            "ttc_s": None if math.isinf(self.ttc_s) else round(float(self.ttc_s), 3),
            "hard_stop_latch_remaining_s": round(self._latch_remaining(), 3),
            "hard_stop_stop_request_sent": bool(self.hard_stop_stop_request_sent),
            "timestamp": datetime.now(timezone.utc).isoformat(),
        }
        status_json = json.dumps(status, ensure_ascii=False)
        if status_json != self.last_status_json or now - self.last_status_time > 1.0:
            msg = String()
            msg.data = status_json
            self.status_pub.publish(msg)
            self.last_status_json = status_json
            self.last_status_time = now
        self._log_diagnostics(now)

        if not self.publish_events or now - self.last_event_time < self.event_period_s:
            return
        if self.state not in (
            "hard_stop",
            "warning",
            "micro_adjust",
            "corridor_trial",
            "corridor_stuck",
            "corridor_side_turn",
            "corridor_turnaround",
            "escape_reverse",
            "spin_escape",
        ):
            return

        event_type = "blocked_path" if self.state == "hard_stop" else "soft_obstacle"
        event = {
            "event_type": event_type,
            "distance_m": -1.0 if self.front_p10 is None else round(float(self.front_p10), 3),
            "confidence": 0.9 if self.state == "hard_stop" else 0.75,
            "source": "scan_safety_guard",
            "front_min_range_m": None if self.front_min is None else round(float(self.front_min), 3),
            "front_p10_range_m": None if self.front_p10 is None else round(float(self.front_p10), 3),
            "front_valid_count": int(self.front_valid_count),
            "corridor_front_min_m": (
                None if self.corridor_front_min is None else round(float(self.corridor_front_min), 3)
            ),
            "corridor_front_p10_m": (
                None if self.corridor_front_p10 is None else round(float(self.corridor_front_p10), 3)
            ),
            "corridor_front_valid_count": int(self.corridor_front_valid_count),
            "rear_corridor_min_m": (
                None if self.rear_corridor_min is None else round(float(self.rear_corridor_min), 3)
            ),
            "rear_corridor_p10_m": (
                None if self.rear_corridor_p10 is None else round(float(self.rear_corridor_p10), 3)
            ),
            "rear_corridor_valid_count": int(self.rear_corridor_valid_count),
            "deadend_front_min_m": (
                None if self.deadend_front_min is None else round(float(self.deadend_front_min), 3)
            ),
            "deadend_front_p10_m": (
                None if self.deadend_front_p10 is None else round(float(self.deadend_front_p10), 3)
            ),
            "deadend_front_valid_count": int(self.deadend_front_valid_count),
            "micro_front_min_range_m": (
                None if self.micro_front_min is None else round(float(self.micro_front_min), 3)
            ),
            "micro_left_min_range_m": (
                None if self.micro_left_min is None else round(float(self.micro_left_min), 3)
            ),
            "micro_right_min_range_m": (
                None if self.micro_right_min is None else round(float(self.micro_right_min), 3)
            ),
            "micro_adjust_sector_deg": self.micro_adjust_sector_deg,
            "micro_adjust_trigger_m": self.micro_adjust_trigger_m,
            "micro_adjust_clear_m": self.micro_adjust_clear_m,
            "micro_adjust_angular_z": self.micro_adjust_angular_z,
            "micro_adjust_turn_sign": self.micro_adjust_turn_sign,
            "micro_adjust_turn_change_count": self.micro_adjust_turn_change_count,
            "micro_adjust_stuck_spin_front_blocked_m": (
                self.micro_adjust_stuck_spin_front_blocked_m
            ),
            "micro_adjust_stuck_spin_elapsed_s": round(
                0.0 if self.micro_adjust_stuck_since <= 0.0 else now - self.micro_adjust_stuck_since,
                3,
            ),
            "corridor_stuck_spin_trigger_m": self.corridor_stuck_spin_trigger_m,
            "corridor_stuck_spin_front_blocked_m": self.corridor_stuck_spin_front_blocked_m,
            "corridor_stuck_spin_side_blocked_m": self.corridor_stuck_spin_side_blocked_m,
            "corridor_stuck_spin_elapsed_s": round(
                0.0 if self.corridor_stuck_since <= 0.0 else now - self.corridor_stuck_since,
                3,
            ),
            "corridor_trial_forward_progress_m": self._rounded_forward_progress(now),
            "corridor_trial_recoveries": int(self.corridor_trial_recoveries),
            "corridor_trial_rear_clear": self._rear_corridor_clear(),
            "spin_escape_degrees": self.spin_escape_degrees,
            "spin_escape_angular_z": self.spin_escape_angular_z,
            "spin_escape_remaining_s": round(
                max(0.0, self.spin_escape_until - time.monotonic()), 3
            ),
            "escape_reverse_trigger_m": self.escape_reverse_trigger_m,
            "escape_reverse_clear_m": self.escape_reverse_clear_m,
            "escape_reverse_linear_x": self.escape_reverse_linear_x,
            "escape_reverse_angular_z": self.escape_reverse_angular_z,
            "escape_reverse_turn_sign": self.escape_reverse_turn_sign,
            "front_sector_deg": self.front_sector_deg,
            "guard_action": action,
            "ttc_s": None if math.isinf(self.ttc_s) else round(float(self.ttc_s), 3),
            "timestamp": datetime.now(timezone.utc).isoformat(),
        }
        msg = String()
        msg.data = json.dumps(event, ensure_ascii=False)
        self.event_pub.publish(msg)
        self.last_event_time = now

    def _front_stats(self, scan: LaserScan):
        if not scan.ranges:
            return {
                "front_min": None,
                "front_p10": None,
                "valid_count": 0,
            "corridor_front_min": None,
            "corridor_front_p10": None,
            "corridor_valid_count": 0,
            "corridor_trial_front_min": None,
            "corridor_trial_front_p10": None,
            "corridor_trial_valid_count": 0,
            "rear_corridor_min": None,
                "rear_corridor_p10": None,
                "rear_corridor_valid_count": 0,
                "deadend_front_min": None,
                "deadend_front_p10": None,
                "deadend_front_valid_count": 0,
                "micro_front_min": None,
                "micro_left_min": None,
                "micro_right_min": None,
            }
        if not math.isfinite(scan.angle_increment) or abs(scan.angle_increment) < 1e-9:
            raise ValueError("LaserScan angle_increment is zero or invalid")

        half_sector_rad = math.radians(max(self.front_sector_deg, 0.0) / 2.0)
        deadend_half_rad = math.radians(max(self.corridor_stuck_spin_front_sector_deg, 0.0) / 2.0)
        micro_half_rad = math.radians(max(self.micro_adjust_sector_deg, 0.0))
        front_values = []
        deadend_front_values = []
        micro_front_values = []
        micro_left_values = []
        micro_right_values = []
        corridor_values = []
        corridor_trial_values = []
        rear_corridor_values = []
        for index, value in enumerate(scan.ranges):
            if not math.isfinite(value):
                continue
            value_f = float(value)
            if value_f <= 0.0 or value_f > self.max_valid_range_m:
                continue
            angle = scan.angle_min + index * scan.angle_increment
            angle = self._normalize_angle(angle)
            if value_f >= self.min_valid_range_m and abs(angle) <= half_sector_rad:
                front_values.append(value_f)
            if value_f >= self.min_valid_range_m and abs(angle) <= deadend_half_rad:
                deadend_front_values.append(value_f)
            if value_f >= self.min_valid_range_m:
                x_forward = value_f * math.cos(angle)
                y_lateral = value_f * math.sin(angle)
                if (
                    x_forward >= self.front_collision_min_x_m
                    and abs(y_lateral) <= self.front_collision_corridor_half_width_m
                ):
                    corridor_values.append(x_forward)
                if (
                    x_forward >= self.front_collision_min_x_m
                    and abs(y_lateral) <= self.corridor_trial_center_half_width_m
                ):
                    corridor_trial_values.append(x_forward)
                x_backward = -x_forward
                if (
                    x_backward >= self.front_collision_min_x_m
                    and abs(y_lateral) <= self.front_collision_corridor_half_width_m
                ):
                    rear_corridor_values.append(x_backward)
            if value_f >= self.micro_adjust_min_valid_range_m:
                if abs(angle) <= micro_half_rad:
                    micro_front_values.append(value_f)
                if 0.0 < angle <= micro_half_rad:
                    micro_left_values.append(value_f)
                if -micro_half_rad <= angle < 0.0:
                    micro_right_values.append(value_f)
        result = {
            "micro_front_min": min(micro_front_values) if micro_front_values else None,
            "micro_left_min": min(micro_left_values) if micro_left_values else None,
            "micro_right_min": min(micro_right_values) if micro_right_values else None,
        }
        if corridor_values:
            corridor_values.sort()
            corridor_p10_index = max(
                0,
                min(len(corridor_values) - 1, math.ceil(len(corridor_values) * 0.10) - 1),
            )
            result.update(
                {
                    "corridor_front_min": corridor_values[0],
                    "corridor_front_p10": corridor_values[corridor_p10_index],
                    "corridor_valid_count": len(corridor_values),
                }
            )
        else:
            result.update(
                {
                    "corridor_front_min": None,
                    "corridor_front_p10": None,
                    "corridor_valid_count": 0,
                }
            )
        if corridor_trial_values:
            corridor_trial_values.sort()
            corridor_trial_p10_index = max(
                0,
                min(
                    len(corridor_trial_values) - 1,
                    math.ceil(len(corridor_trial_values) * 0.10) - 1,
                ),
            )
            result.update(
                {
                    "corridor_trial_front_min": corridor_trial_values[0],
                    "corridor_trial_front_p10": corridor_trial_values[
                        corridor_trial_p10_index
                    ],
                    "corridor_trial_valid_count": len(corridor_trial_values),
                }
            )
        else:
            result.update(
                {
                    "corridor_trial_front_min": None,
                    "corridor_trial_front_p10": None,
                    "corridor_trial_valid_count": 0,
                }
            )
        if deadend_front_values:
            deadend_front_values.sort()
            deadend_p10_index = max(
                0,
                min(
                    len(deadend_front_values) - 1,
                    math.ceil(len(deadend_front_values) * 0.10) - 1,
                ),
            )
            result.update(
                {
                    "deadend_front_min": deadend_front_values[0],
                    "deadend_front_p10": deadend_front_values[deadend_p10_index],
                    "deadend_front_valid_count": len(deadend_front_values),
                }
            )
        else:
            result.update(
                {
                    "deadend_front_min": None,
                    "deadend_front_p10": None,
                    "deadend_front_valid_count": 0,
                }
            )
        if rear_corridor_values:
            rear_corridor_values.sort()
            rear_p10_index = max(
                0,
                min(
                    len(rear_corridor_values) - 1,
                    math.ceil(len(rear_corridor_values) * 0.10) - 1,
                ),
            )
            result.update(
                {
                    "rear_corridor_min": rear_corridor_values[0],
                    "rear_corridor_p10": rear_corridor_values[rear_p10_index],
                    "rear_corridor_valid_count": len(rear_corridor_values),
                }
            )
        else:
            result.update(
                {
                    "rear_corridor_min": None,
                    "rear_corridor_p10": None,
                    "rear_corridor_valid_count": 0,
                }
            )
        if not front_values:
            result.update({"front_min": None, "front_p10": None, "valid_count": 0})
            return result
        front_values.sort()
        p10_index = max(0, min(len(front_values) - 1, math.ceil(len(front_values) * 0.10) - 1))
        result.update(
            {
                "front_min": front_values[0],
                "front_p10": front_values[p10_index],
                "valid_count": len(front_values),
            }
        )
        return result

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

    @staticmethod
    def _yaw_from_quat(quat) -> float:
        return math.atan2(
            2.0 * (float(quat.w) * float(quat.z) + float(quat.x) * float(quat.y)),
            1.0 - 2.0 * (float(quat.y) * float(quat.y) + float(quat.z) * float(quat.z)),
        )


def main(args=None):
    rclpy.init(args=args)
    node = ScanSafetyGuardNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
