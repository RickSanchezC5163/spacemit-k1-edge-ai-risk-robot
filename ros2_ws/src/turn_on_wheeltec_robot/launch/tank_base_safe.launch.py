from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    serial_port = LaunchConfiguration("serial_port")
    baud = LaunchConfiguration("baud")
    auto_recharge = LaunchConfiguration("auto_recharge")
    security_ply = LaunchConfiguration("security_ply")
    send_security_enable_on_start = LaunchConfiguration("send_security_enable_on_start")
    send_rate = LaunchConfiguration("send_rate")
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic")
    max_linear = LaunchConfiguration("max_linear")
    max_angular = LaunchConfiguration("max_angular")
    cmd_timeout = LaunchConfiguration("cmd_timeout")
    brake_duration = LaunchConfiguration("brake_duration")
    cruise_linear_limit = LaunchConfiguration("cruise_linear_limit")
    cruise_angular_limit = LaunchConfiguration("cruise_angular_limit")
    start_kick_duration = LaunchConfiguration("start_kick_duration")
    start_kick_linear = LaunchConfiguration("start_kick_linear")
    start_kick_angular = LaunchConfiguration("start_kick_angular")
    stop_kick_duration = LaunchConfiguration("stop_kick_duration")
    stop_kick_linear = LaunchConfiguration("stop_kick_linear")
    stop_kick_angular = LaunchConfiguration("stop_kick_angular")
    stop_kick_match_cmd = LaunchConfiguration("stop_kick_match_cmd")
    stop_kick_match_duration = LaunchConfiguration("stop_kick_match_duration")
    stop_kick_speed_gain = LaunchConfiguration("stop_kick_speed_gain")
    stop_kick_duration_mode = LaunchConfiguration("stop_kick_duration_mode")
    stop_kick_duration_ratio = LaunchConfiguration("stop_kick_duration_ratio")
    stop_kick_impulse_ratio = LaunchConfiguration("stop_kick_impulse_ratio")
    stop_kick_duration_offset = LaunchConfiguration("stop_kick_duration_offset")
    stop_kick_max_duration = LaunchConfiguration("stop_kick_max_duration")
    stop_kick_min_duration = LaunchConfiguration("stop_kick_min_duration")
    stop_kick_until_stopped = LaunchConfiguration("stop_kick_until_stopped")
    stop_kick_velocity_epsilon = LaunchConfiguration("stop_kick_velocity_epsilon")
    stop_request_topic = LaunchConfiguration("stop_request_topic")
    publish_tf = LaunchConfiguration("publish_tf")
    odom_linear_scale = LaunchConfiguration("odom_linear_scale")
    odom_angular_scale = LaunchConfiguration("odom_angular_scale")

    return LaunchDescription([
        DeclareLaunchArgument(
            "serial_port",
            default_value="/dev/base_controller",
            description="C30D ROS chassis serial device.",
        ),
        DeclareLaunchArgument(
            "baud",
            default_value="115200",
            description="C30D ROS chassis serial baud rate.",
        ),
        DeclareLaunchArgument(
            "auto_recharge",
            default_value="0",
            description="Reserved C30D command byte used by the official driver.",
        ),
        DeclareLaunchArgument(
            "security_ply",
            default_value="1",
            description="Reserved C30D safety byte used by the official driver.",
        ),
        DeclareLaunchArgument(
            "send_security_enable_on_start",
            default_value="true",
            description="Send the official chassis_security enable frames before motion commands.",
        ),
        DeclareLaunchArgument(
            "send_rate",
            default_value="50.0",
            description="Rate used to refresh chassis command frames.",
        ),
        DeclareLaunchArgument(
            "cmd_vel_topic",
            default_value="/cmd_vel",
            description="Input velocity topic consumed by the tank base node.",
        ),
        DeclareLaunchArgument(
            "max_linear",
            default_value="0.005",
            description="Bring-up linear speed limit in m/s.",
        ),
        DeclareLaunchArgument(
            "max_angular",
            default_value="0.03",
            description="Bring-up angular speed limit in rad/s.",
        ),
        DeclareLaunchArgument(
            "cmd_timeout",
            default_value="0.25",
            description="Seconds before sending zero speed after /cmd_vel stops.",
        ),
        DeclareLaunchArgument(
            "brake_duration",
            default_value="1.0",
            description="Seconds to force zero speed after an explicit stop command.",
        ),
        DeclareLaunchArgument(
            "cruise_linear_limit",
            default_value="0.08",
            description="Sustained linear speed limit after the optional start kick.",
        ),
        DeclareLaunchArgument(
            "cruise_angular_limit",
            default_value="0.20",
            description="Sustained angular speed limit after the optional start kick.",
        ),
        DeclareLaunchArgument(
            "start_kick_duration",
            default_value="0.0",
            description="Short breakaway pulse duration when starting from rest.",
        ),
        DeclareLaunchArgument(
            "start_kick_linear",
            default_value="0.0",
            description="Breakaway linear speed in m/s for start_kick_duration.",
        ),
        DeclareLaunchArgument(
            "start_kick_angular",
            default_value="0.0",
            description="Breakaway angular speed in rad/s for start_kick_duration.",
        ),
        DeclareLaunchArgument(
            "stop_kick_duration",
            default_value="0.0",
            description="Seconds to send a small reverse command before zero braking.",
        ),
        DeclareLaunchArgument(
            "stop_kick_linear",
            default_value="0.0",
            description="Reverse linear speed in m/s for stop_kick_duration.",
        ),
        DeclareLaunchArgument(
            "stop_kick_angular",
            default_value="0.0",
            description="Reverse angular speed in rad/s for stop_kick_duration.",
        ),
        DeclareLaunchArgument(
            "stop_kick_match_cmd",
            default_value="false",
            description="Use the last commanded velocity magnitude for reverse braking.",
        ),
        DeclareLaunchArgument(
            "stop_kick_match_duration",
            default_value="false",
            description="Use the previous motion command duration as reverse braking duration.",
        ),
        DeclareLaunchArgument(
            "stop_kick_speed_gain",
            default_value="1.0",
            description="Scale matched reverse braking speed before clamping to max_linear/max_angular.",
        ),
        DeclareLaunchArgument(
            "stop_kick_duration_mode",
            default_value="duration_ratio",
            description="Reverse braking duration mode: duration_ratio, impulse, or fixed.",
        ),
        DeclareLaunchArgument(
            "stop_kick_duration_ratio",
            default_value="1.0",
            description="Scale the previous motion duration before reverse braking.",
        ),
        DeclareLaunchArgument(
            "stop_kick_impulse_ratio",
            default_value="1.0",
            description="Scale command impulse cancellation when stop_kick_duration_mode=impulse.",
        ),
        DeclareLaunchArgument(
            "stop_kick_duration_offset",
            default_value="0.0",
            description="Additive reverse braking duration after applying stop_kick_duration_ratio.",
        ),
        DeclareLaunchArgument(
            "stop_kick_max_duration",
            default_value="1.0",
            description="Upper bound for duration-matched reverse braking.",
        ),
        DeclareLaunchArgument(
            "stop_kick_min_duration",
            default_value="0.12",
            description="Minimum reverse braking time before feedback can end the stop kick.",
        ),
        DeclareLaunchArgument(
            "stop_kick_until_stopped",
            default_value="false",
            description="End reverse braking early when C30D feedback velocity reaches near zero.",
        ),
        DeclareLaunchArgument(
            "stop_kick_velocity_epsilon",
            default_value="0.02",
            description="Velocity threshold used by feedback-based reverse braking.",
        ),
        DeclareLaunchArgument(
            "stop_request_topic",
            default_value="/chassis/stop_request",
            description="STOP_REQUEST topic used for matched reverse braking.",
        ),
        DeclareLaunchArgument(
            "publish_tf",
            default_value="true",
            description="Publish odom to base_footprint transform.",
        ),
        DeclareLaunchArgument(
            "odom_linear_scale",
            default_value="1.0",
            description="Scale C30D feedback linear velocity before odom integration.",
        ),
        DeclareLaunchArgument(
            "odom_angular_scale",
            default_value="1.0",
            description="Scale C30D feedback angular velocity before odom integration.",
        ),
        Node(
            package="turn_on_wheeltec_robot",
            executable="wheeltec_tank_base_safe.py",
            name="wheeltec_tank_base",
            output="screen",
            parameters=[{
                "port": serial_port,
                "baud": baud,
                "auto_recharge": auto_recharge,
                "security_ply": security_ply,
                "send_security_enable_on_start": send_security_enable_on_start,
                "send_rate": send_rate,
                "cmd_vel_topic": cmd_vel_topic,
                "max_linear": max_linear,
                "max_angular": max_angular,
                "cmd_timeout": cmd_timeout,
                "brake_duration": brake_duration,
                "cruise_linear_limit": cruise_linear_limit,
                "cruise_angular_limit": cruise_angular_limit,
                "start_kick_duration": start_kick_duration,
                "start_kick_linear": start_kick_linear,
                "start_kick_angular": start_kick_angular,
                "stop_kick_duration": stop_kick_duration,
                "stop_kick_linear": stop_kick_linear,
                "stop_kick_angular": stop_kick_angular,
                "stop_kick_match_cmd": stop_kick_match_cmd,
                "stop_kick_match_duration": stop_kick_match_duration,
                "stop_kick_speed_gain": stop_kick_speed_gain,
                "stop_kick_duration_mode": stop_kick_duration_mode,
                "stop_kick_duration_ratio": stop_kick_duration_ratio,
                "stop_kick_impulse_ratio": stop_kick_impulse_ratio,
                "stop_kick_duration_offset": stop_kick_duration_offset,
                "stop_kick_max_duration": stop_kick_max_duration,
                "stop_kick_min_duration": stop_kick_min_duration,
                "stop_kick_until_stopped": stop_kick_until_stopped,
                "stop_kick_velocity_epsilon": stop_kick_velocity_epsilon,
                "stop_request_topic": stop_request_topic,
                "publish_tf": publish_tf,
                "odom_linear_scale": odom_linear_scale,
                "odom_angular_scale": odom_angular_scale,
            }],
        ),
    ])
