from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    scan_topic = LaunchConfiguration("scan_topic")
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
            DeclareLaunchArgument("input_cmd_vel", default_value="/input_cmd_vel"),
            DeclareLaunchArgument("guarded_cmd_vel", default_value="/cmd_vel_guarded"),
            DeclareLaunchArgument("stop_request_topic", default_value="/chassis/stop_request"),
            DeclareLaunchArgument("front_sector_deg", default_value="35.0"),
            DeclareLaunchArgument("front_collision_corridor_half_width_m", default_value="0.26"),
            DeclareLaunchArgument("front_collision_min_x_m", default_value="0.02"),
            DeclareLaunchArgument("micro_adjust_sector_deg", default_value="45.0"),
            DeclareLaunchArgument("micro_adjust_trigger_m", default_value="0.22"),
            DeclareLaunchArgument("micro_adjust_clear_m", default_value="0.30"),
            DeclareLaunchArgument("micro_adjust_direction_deadband_m", default_value="0.03"),
            DeclareLaunchArgument("micro_adjust_direction_latch_s", default_value="1.50"),
            DeclareLaunchArgument("enable_escape_reverse", default_value="true"),
            DeclareLaunchArgument("escape_reverse_trigger_m", default_value="0.16"),
            DeclareLaunchArgument("escape_reverse_clear_m", default_value="0.24"),
            DeclareLaunchArgument("escape_reverse_linear_x", default_value="-0.08"),
            DeclareLaunchArgument("escape_reverse_angular_z", default_value="0.20"),
            DeclareLaunchArgument("escape_reverse_max_s", default_value="0.80"),
            DeclareLaunchArgument("escape_reverse_cooldown_s", default_value="0.40"),
            DeclareLaunchArgument("hard_stop_m", default_value="1.00"),
            DeclareLaunchArgument("slow_down_m", default_value="1.60"),
            DeclareLaunchArgument("soft_max_linear", default_value="0.30"),
            DeclareLaunchArgument("clear_max_linear", default_value="0.30"),
            DeclareLaunchArgument("min_effective_forward", default_value="0.08"),
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
                        "output_rate_hz": 50.0,
                        "fail_closed_without_scan": True,
                        "publish_events": True,
                    }
                ],
            ),
        ]
    )
