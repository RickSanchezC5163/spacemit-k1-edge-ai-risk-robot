from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    scan_topic = LaunchConfiguration("scan_topic")
    odom_topic = LaunchConfiguration("odom_topic")
    input_cmd_vel = LaunchConfiguration("input_cmd_vel")
    guarded_cmd_vel = LaunchConfiguration("guarded_cmd_vel")
    stop_request_topic = LaunchConfiguration("stop_request_topic")
    front_sector_deg = LaunchConfiguration("front_sector_deg")
    front_collision_corridor_half_width_m = LaunchConfiguration(
        "front_collision_corridor_half_width_m"
    )
    front_collision_min_x_m = LaunchConfiguration("front_collision_min_x_m")
    micro_adjust_sector_deg = LaunchConfiguration("micro_adjust_sector_deg")
    micro_adjust_trigger_m = LaunchConfiguration("micro_adjust_trigger_m")
    micro_adjust_clear_m = LaunchConfiguration("micro_adjust_clear_m")
    micro_adjust_direction_deadband_m = LaunchConfiguration("micro_adjust_direction_deadband_m")
    micro_adjust_direction_latch_s = LaunchConfiguration("micro_adjust_direction_latch_s")
    enable_spin_escape = LaunchConfiguration("enable_spin_escape")
    spin_escape_turn_changes = LaunchConfiguration("spin_escape_turn_changes")
    spin_escape_degrees = LaunchConfiguration("spin_escape_degrees")
    spin_escape_angular_z = LaunchConfiguration("spin_escape_angular_z")
    spin_escape_cooldown_s = LaunchConfiguration("spin_escape_cooldown_s")
    enable_micro_adjust_stuck_spin_escape = LaunchConfiguration(
        "enable_micro_adjust_stuck_spin_escape"
    )
    micro_adjust_stuck_spin_min_s = LaunchConfiguration("micro_adjust_stuck_spin_min_s")
    micro_adjust_stuck_spin_front_blocked_m = LaunchConfiguration(
        "micro_adjust_stuck_spin_front_blocked_m"
    )
    micro_adjust_stuck_spin_clear_m = LaunchConfiguration("micro_adjust_stuck_spin_clear_m")
    micro_adjust_stuck_spin_cmd_angular_mps = LaunchConfiguration(
        "micro_adjust_stuck_spin_cmd_angular_mps"
    )
    enable_corridor_stuck_spin_escape = LaunchConfiguration("enable_corridor_stuck_spin_escape")
    corridor_stuck_spin_trigger_m = LaunchConfiguration("corridor_stuck_spin_trigger_m")
    corridor_stuck_spin_clear_m = LaunchConfiguration("corridor_stuck_spin_clear_m")
    corridor_stuck_spin_min_s = LaunchConfiguration("corridor_stuck_spin_min_s")
    corridor_stuck_spin_cmd_angular_mps = LaunchConfiguration(
        "corridor_stuck_spin_cmd_angular_mps"
    )
    corridor_stuck_spin_front_blocked_m = LaunchConfiguration("corridor_stuck_spin_front_blocked_m")
    corridor_stuck_spin_front_sector_deg = LaunchConfiguration("corridor_stuck_spin_front_sector_deg")
    corridor_stuck_spin_require_sides = LaunchConfiguration("corridor_stuck_spin_require_sides")
    corridor_stuck_spin_side_blocked_m = LaunchConfiguration("corridor_stuck_spin_side_blocked_m")
    enable_corridor_trial = LaunchConfiguration("enable_corridor_trial")
    corridor_trial_center_half_width_m = LaunchConfiguration("corridor_trial_center_half_width_m")
    corridor_trial_enter_front_p10_m = LaunchConfiguration("corridor_trial_enter_front_p10_m")
    corridor_trial_keep_front_p10_m = LaunchConfiguration("corridor_trial_keep_front_p10_m")
    corridor_trial_side_near_m = LaunchConfiguration("corridor_trial_side_near_m")
    corridor_trial_enter_stable_s = LaunchConfiguration("corridor_trial_enter_stable_s")
    corridor_trial_exit_stable_s = LaunchConfiguration("corridor_trial_exit_stable_s")
    corridor_trial_forward_intent_mps = LaunchConfiguration("corridor_trial_forward_intent_mps")
    corridor_trial_max_linear_mps = LaunchConfiguration("corridor_trial_max_linear_mps")
    corridor_trial_max_angular_radps = LaunchConfiguration("corridor_trial_max_angular_radps")
    corridor_trial_wall_turn_limit_radps = LaunchConfiguration(
        "corridor_trial_wall_turn_limit_radps"
    )
    corridor_trial_progress_window_s = LaunchConfiguration("corridor_trial_progress_window_s")
    corridor_trial_min_forward_progress_m = LaunchConfiguration(
        "corridor_trial_min_forward_progress_m"
    )
    corridor_trial_blocked_front_p10_m = LaunchConfiguration("corridor_trial_blocked_front_p10_m")
    corridor_trial_blocked_stable_s = LaunchConfiguration("corridor_trial_blocked_stable_s")
    corridor_trial_stop_s = LaunchConfiguration("corridor_trial_stop_s")
    corridor_trial_rear_clear_m = LaunchConfiguration("corridor_trial_rear_clear_m")
    corridor_trial_max_recoveries = LaunchConfiguration("corridor_trial_max_recoveries")
    corridor_trial_odom_timeout_s = LaunchConfiguration("corridor_trial_odom_timeout_s")
    enable_escape_reverse = LaunchConfiguration("enable_escape_reverse")
    escape_reverse_trigger_m = LaunchConfiguration("escape_reverse_trigger_m")
    escape_reverse_clear_m = LaunchConfiguration("escape_reverse_clear_m")
    escape_reverse_linear_x = LaunchConfiguration("escape_reverse_linear_x")
    escape_reverse_angular_z = LaunchConfiguration("escape_reverse_angular_z")
    escape_reverse_max_s = LaunchConfiguration("escape_reverse_max_s")
    escape_reverse_cooldown_s = LaunchConfiguration("escape_reverse_cooldown_s")
    hard_stop_m = LaunchConfiguration("hard_stop_m")
    slow_down_m = LaunchConfiguration("slow_down_m")
    soft_max_linear = LaunchConfiguration("soft_max_linear")
    clear_max_linear = LaunchConfiguration("clear_max_linear")
    min_effective_forward = LaunchConfiguration("min_effective_forward")
    emergency_stop_m = LaunchConfiguration("emergency_stop_m")
    approach_stop_m = LaunchConfiguration("approach_stop_m")
    approach_rate_stop_mps = LaunchConfiguration("approach_rate_stop_mps")
    ttc_stop_s = LaunchConfiguration("ttc_stop_s")
    hard_stop_latch_s = LaunchConfiguration("hard_stop_latch_s")
    enable_dynamic_stop = LaunchConfiguration("enable_dynamic_stop")

    return LaunchDescription(
        [
            DeclareLaunchArgument("serial_port", default_value="/dev/base_controller"),
            DeclareLaunchArgument("max_linear_x", default_value="0.45"),
            DeclareLaunchArgument("max_angular_z", default_value="0.80"),
            DeclareLaunchArgument("brake_duration_sec", default_value="1.5"),
            DeclareLaunchArgument("brake_speed_gain", default_value="1.0"),
            DeclareLaunchArgument("brake_duration_mode", default_value="duration_ratio"),
            DeclareLaunchArgument("brake_duration_gain", default_value="0.45"),
            DeclareLaunchArgument("brake_impulse_ratio", default_value="1.0"),
            DeclareLaunchArgument("brake_duration_offset", default_value="0.0"),
            DeclareLaunchArgument("scan_topic", default_value="/scan"),
            DeclareLaunchArgument("odom_topic", default_value="/odom"),
            DeclareLaunchArgument("input_cmd_vel", default_value="/input_cmd_vel"),
            DeclareLaunchArgument("guarded_cmd_vel", default_value="/cmd_vel_guarded"),
            DeclareLaunchArgument("stop_request_topic", default_value="/chassis/stop_request"),
            DeclareLaunchArgument("front_sector_deg", default_value="35.0"),
            DeclareLaunchArgument("front_collision_corridor_half_width_m", default_value="0.26"),
            DeclareLaunchArgument("front_collision_min_x_m", default_value="0.12"),
            DeclareLaunchArgument("micro_adjust_sector_deg", default_value="45.0"),
            DeclareLaunchArgument("micro_adjust_trigger_m", default_value="0.28"),
            DeclareLaunchArgument("micro_adjust_clear_m", default_value="0.34"),
            DeclareLaunchArgument("micro_adjust_direction_deadband_m", default_value="0.03"),
            DeclareLaunchArgument("micro_adjust_direction_latch_s", default_value="1.50"),
            DeclareLaunchArgument("enable_spin_escape", default_value="true"),
            DeclareLaunchArgument("spin_escape_turn_changes", default_value="3"),
            DeclareLaunchArgument("spin_escape_degrees", default_value="180.0"),
            DeclareLaunchArgument("spin_escape_angular_z", default_value="0.35"),
            DeclareLaunchArgument("spin_escape_cooldown_s", default_value="3.0"),
            DeclareLaunchArgument("enable_micro_adjust_stuck_spin_escape", default_value="true"),
            DeclareLaunchArgument("micro_adjust_stuck_spin_min_s", default_value="6.0"),
            DeclareLaunchArgument("micro_adjust_stuck_spin_front_blocked_m", default_value="0.30"),
            DeclareLaunchArgument("micro_adjust_stuck_spin_clear_m", default_value="0.40"),
            DeclareLaunchArgument("micro_adjust_stuck_spin_cmd_angular_mps", default_value="0.05"),
            DeclareLaunchArgument("enable_corridor_stuck_spin_escape", default_value="true"),
            DeclareLaunchArgument("corridor_stuck_spin_trigger_m", default_value="0.18"),
            DeclareLaunchArgument("corridor_stuck_spin_clear_m", default_value="0.24"),
            DeclareLaunchArgument("corridor_stuck_spin_min_s", default_value="3.0"),
            DeclareLaunchArgument("corridor_stuck_spin_cmd_angular_mps", default_value="0.06"),
            DeclareLaunchArgument("corridor_stuck_spin_front_blocked_m", default_value="0.30"),
            DeclareLaunchArgument("corridor_stuck_spin_front_sector_deg", default_value="20.0"),
            DeclareLaunchArgument("corridor_stuck_spin_require_sides", default_value="true"),
            DeclareLaunchArgument("corridor_stuck_spin_side_blocked_m", default_value="0.32"),
            DeclareLaunchArgument("enable_corridor_trial", default_value="true"),
            DeclareLaunchArgument("corridor_trial_center_half_width_m", default_value="0.10"),
            DeclareLaunchArgument("corridor_trial_enter_front_p10_m", default_value="0.40"),
            DeclareLaunchArgument("corridor_trial_keep_front_p10_m", default_value="0.34"),
            DeclareLaunchArgument("corridor_trial_side_near_m", default_value="0.40"),
            DeclareLaunchArgument("corridor_trial_enter_stable_s", default_value="0.50"),
            DeclareLaunchArgument("corridor_trial_exit_stable_s", default_value="0.80"),
            DeclareLaunchArgument("corridor_trial_forward_intent_mps", default_value="0.04"),
            DeclareLaunchArgument("corridor_trial_max_linear_mps", default_value="0.24"),
            DeclareLaunchArgument("corridor_trial_max_angular_radps", default_value="0.16"),
            DeclareLaunchArgument("corridor_trial_wall_turn_limit_radps", default_value="0.06"),
            DeclareLaunchArgument("corridor_trial_progress_window_s", default_value="2.50"),
            DeclareLaunchArgument("corridor_trial_min_forward_progress_m", default_value="0.04"),
            DeclareLaunchArgument("corridor_trial_blocked_front_p10_m", default_value="0.30"),
            DeclareLaunchArgument("corridor_trial_blocked_stable_s", default_value="0.80"),
            DeclareLaunchArgument("corridor_trial_stop_s", default_value="0.40"),
            DeclareLaunchArgument("corridor_trial_rear_clear_m", default_value="0.18"),
            DeclareLaunchArgument("corridor_trial_max_recoveries", default_value="2"),
            DeclareLaunchArgument("corridor_trial_odom_timeout_s", default_value="0.60"),
            DeclareLaunchArgument("enable_escape_reverse", default_value="true"),
            DeclareLaunchArgument("escape_reverse_trigger_m", default_value="0.16"),
            DeclareLaunchArgument("escape_reverse_clear_m", default_value="0.24"),
            DeclareLaunchArgument("escape_reverse_linear_x", default_value="-0.14"),
            DeclareLaunchArgument("escape_reverse_angular_z", default_value="0.20"),
            DeclareLaunchArgument("escape_reverse_max_s", default_value="0.80"),
            DeclareLaunchArgument("escape_reverse_cooldown_s", default_value="0.40"),
            DeclareLaunchArgument("hard_stop_m", default_value="1.00"),
            DeclareLaunchArgument("slow_down_m", default_value="1.60"),
            DeclareLaunchArgument("soft_max_linear", default_value="0.30"),
            DeclareLaunchArgument("clear_max_linear", default_value="0.30"),
            DeclareLaunchArgument("min_effective_forward", default_value="0.12"),
            DeclareLaunchArgument("emergency_stop_m", default_value="0.45"),
            DeclareLaunchArgument("approach_stop_m", default_value="1.60"),
            DeclareLaunchArgument("approach_rate_stop_mps", default_value="0.35"),
            DeclareLaunchArgument("ttc_stop_s", default_value="1.20"),
            DeclareLaunchArgument("enable_dynamic_stop", default_value="false"),
            DeclareLaunchArgument("hard_stop_latch_s", default_value="1.50"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("turn_on_wheeltec_robot"),
                            "launch",
                            "n10p_tank_mapping.launch.py",
                        ]
                    )
                ),
                launch_arguments={
                    "serial_port": LaunchConfiguration("serial_port"),
                    "max_linear_x": LaunchConfiguration("max_linear_x"),
                    "max_angular_z": LaunchConfiguration("max_angular_z"),
                    "brake_duration_sec": LaunchConfiguration("brake_duration_sec"),
                    "brake_speed_gain": LaunchConfiguration("brake_speed_gain"),
                    "brake_duration_mode": LaunchConfiguration("brake_duration_mode"),
                    "brake_duration_gain": LaunchConfiguration("brake_duration_gain"),
                    "brake_impulse_ratio": LaunchConfiguration("brake_impulse_ratio"),
                    "brake_duration_offset": LaunchConfiguration("brake_duration_offset"),
                    "base_cmd_vel": guarded_cmd_vel,
                    "stop_request_topic": stop_request_topic,
                }.items(),
            ),
            Node(
                package="k1_sensor_event_adapter",
                executable="scan_safety_guard_node",
                name="scan_safety_guard_node",
                output="screen",
                parameters=[
                    {
                        "scan_topic": scan_topic,
                        "odom_topic": odom_topic,
                        "input_cmd_topic": input_cmd_vel,
                        "output_cmd_topic": guarded_cmd_vel,
                        "stop_request_topic": stop_request_topic,
                        "status_topic": "/safety/front_obstacle",
                        "event_topic": "/perception/mock_event",
                        "front_sector_deg": front_sector_deg,
                        "front_collision_corridor_half_width_m": front_collision_corridor_half_width_m,
                        "front_collision_min_x_m": front_collision_min_x_m,
                        "micro_adjust_sector_deg": micro_adjust_sector_deg,
                        "micro_adjust_trigger_m": micro_adjust_trigger_m,
                        "micro_adjust_clear_m": micro_adjust_clear_m,
                        "micro_adjust_direction_deadband_m": micro_adjust_direction_deadband_m,
                        "micro_adjust_direction_latch_s": micro_adjust_direction_latch_s,
                        "enable_spin_escape": enable_spin_escape,
                        "spin_escape_turn_changes": spin_escape_turn_changes,
                        "spin_escape_degrees": spin_escape_degrees,
                        "spin_escape_angular_z": spin_escape_angular_z,
                        "spin_escape_cooldown_s": spin_escape_cooldown_s,
                        "enable_micro_adjust_stuck_spin_escape": (
                            enable_micro_adjust_stuck_spin_escape
                        ),
                        "micro_adjust_stuck_spin_min_s": micro_adjust_stuck_spin_min_s,
                        "micro_adjust_stuck_spin_front_blocked_m": (
                            micro_adjust_stuck_spin_front_blocked_m
                        ),
                        "micro_adjust_stuck_spin_clear_m": micro_adjust_stuck_spin_clear_m,
                        "micro_adjust_stuck_spin_cmd_angular_mps": (
                            micro_adjust_stuck_spin_cmd_angular_mps
                        ),
                        "enable_corridor_stuck_spin_escape": enable_corridor_stuck_spin_escape,
                        "corridor_stuck_spin_trigger_m": corridor_stuck_spin_trigger_m,
                        "corridor_stuck_spin_clear_m": corridor_stuck_spin_clear_m,
                        "corridor_stuck_spin_min_s": corridor_stuck_spin_min_s,
                        "corridor_stuck_spin_cmd_angular_mps": corridor_stuck_spin_cmd_angular_mps,
                        "corridor_stuck_spin_front_blocked_m": corridor_stuck_spin_front_blocked_m,
                        "corridor_stuck_spin_front_sector_deg": corridor_stuck_spin_front_sector_deg,
                        "corridor_stuck_spin_require_sides": corridor_stuck_spin_require_sides,
                        "corridor_stuck_spin_side_blocked_m": corridor_stuck_spin_side_blocked_m,
                        "enable_corridor_trial": enable_corridor_trial,
                        "corridor_trial_center_half_width_m": (
                            corridor_trial_center_half_width_m
                        ),
                        "corridor_trial_enter_front_p10_m": corridor_trial_enter_front_p10_m,
                        "corridor_trial_keep_front_p10_m": corridor_trial_keep_front_p10_m,
                        "corridor_trial_side_near_m": corridor_trial_side_near_m,
                        "corridor_trial_enter_stable_s": corridor_trial_enter_stable_s,
                        "corridor_trial_exit_stable_s": corridor_trial_exit_stable_s,
                        "corridor_trial_forward_intent_mps": corridor_trial_forward_intent_mps,
                        "corridor_trial_max_linear_mps": corridor_trial_max_linear_mps,
                        "corridor_trial_max_angular_radps": corridor_trial_max_angular_radps,
                        "corridor_trial_wall_turn_limit_radps": (
                            corridor_trial_wall_turn_limit_radps
                        ),
                        "corridor_trial_progress_window_s": corridor_trial_progress_window_s,
                        "corridor_trial_min_forward_progress_m": (
                            corridor_trial_min_forward_progress_m
                        ),
                        "corridor_trial_blocked_front_p10_m": corridor_trial_blocked_front_p10_m,
                        "corridor_trial_blocked_stable_s": corridor_trial_blocked_stable_s,
                        "corridor_trial_stop_s": corridor_trial_stop_s,
                        "corridor_trial_rear_clear_m": corridor_trial_rear_clear_m,
                        "corridor_trial_max_recoveries": corridor_trial_max_recoveries,
                        "corridor_trial_odom_timeout_s": corridor_trial_odom_timeout_s,
                        "enable_escape_reverse": enable_escape_reverse,
                        "escape_reverse_trigger_m": escape_reverse_trigger_m,
                        "escape_reverse_clear_m": escape_reverse_clear_m,
                        "escape_reverse_linear_x": escape_reverse_linear_x,
                        "escape_reverse_angular_z": escape_reverse_angular_z,
                        "escape_reverse_max_s": escape_reverse_max_s,
                        "escape_reverse_cooldown_s": escape_reverse_cooldown_s,
                        "hard_stop_m": hard_stop_m,
                        "slow_down_m": slow_down_m,
                        "soft_max_linear": soft_max_linear,
                        "clear_max_linear": clear_max_linear,
                        "min_effective_forward": min_effective_forward,
                        "emergency_stop_m": emergency_stop_m,
                        "approach_stop_m": approach_stop_m,
                        "approach_rate_stop_mps": approach_rate_stop_mps,
                        "ttc_stop_s": ttc_stop_s,
                        "enable_dynamic_stop": enable_dynamic_stop,
                        "hard_stop_latch_s": hard_stop_latch_s,
                        "output_rate_hz": 15.0,
                        "fail_closed_without_scan": True,
                        "publish_events": True,
                    }
                ],
            ),
        ]
    )
