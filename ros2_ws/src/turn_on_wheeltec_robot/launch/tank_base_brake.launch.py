from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    serial_port = LaunchConfiguration("serial_port")
    baud = LaunchConfiguration("baud")
    send_rate_hz = LaunchConfiguration("send_rate_hz")
    cmd_timeout_sec = LaunchConfiguration("cmd_timeout_sec")
    max_linear_x = LaunchConfiguration("max_linear_x")
    max_linear_y = LaunchConfiguration("max_linear_y")
    max_angular_z = LaunchConfiguration("max_angular_z")
    brake_enable = LaunchConfiguration("brake_enable")
    brake_ratio = LaunchConfiguration("brake_ratio")
    brake_duration_sec = LaunchConfiguration("brake_duration_sec")
    brake_duration_gain = LaunchConfiguration("brake_duration_gain")
    brake_min_motion_sec = LaunchConfiguration("brake_min_motion_sec")
    brake_trigger_threshold = LaunchConfiguration("brake_trigger_threshold")
    enable_chassis_security = LaunchConfiguration("enable_chassis_security")

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
            "send_rate_hz",
            default_value="50.0",
            description="Rate used to refresh chassis command frames.",
        ),
        DeclareLaunchArgument(
            "cmd_timeout_sec",
            default_value="0.25",
            description="Seconds before the last command is considered stale.",
        ),
        DeclareLaunchArgument(
            "max_linear_x",
            default_value="0.45",
            description="Absolute linear-x speed limit in m/s.",
        ),
        DeclareLaunchArgument(
            "max_linear_y",
            default_value="0.0",
            description="Tank chassis does not use lateral velocity.",
        ),
        DeclareLaunchArgument(
            "max_angular_z",
            default_value="2.40",
            description="Absolute yaw speed limit in rad/s.",
        ),
        DeclareLaunchArgument(
            "brake_enable",
            default_value="true",
            description="Enable one reverse brake pulse when motion ends.",
        ),
        DeclareLaunchArgument(
            "brake_ratio",
            default_value="1.0",
            description="Reverse brake command ratio relative to last motion command.",
        ),
        DeclareLaunchArgument(
            "brake_duration_sec",
            default_value="0.50",
            description="Maximum reverse brake pulse duration in seconds.",
        ),
        DeclareLaunchArgument(
            "brake_duration_gain",
            default_value="1.0",
            description="Scale brake duration by the preceding motion duration, capped by brake_duration_sec.",
        ),
        DeclareLaunchArgument(
            "brake_min_motion_sec",
            default_value="0.12",
            description="Do not brake if the preceding motion command was shorter than this.",
        ),
        DeclareLaunchArgument(
            "brake_trigger_threshold",
            default_value="0.02",
            description="Velocity threshold below which a command is treated as stop.",
        ),
        DeclareLaunchArgument(
            "enable_chassis_security",
            default_value="true",
            description="Publish /chassis_security=1 so C30D accepts ROS motion commands.",
        ),
        Node(
            package="turn_on_wheeltec_robot",
            executable="chassis_security_keepalive.py",
            name="chassis_security_keepalive",
            output="screen",
            parameters=[{
                "enabled": enable_chassis_security,
                "rate_hz": 1.0,
            }],
        ),
        Node(
            package="turn_on_wheeltec_robot",
            executable="wheeltec_robot_node",
            name="wheeltec_robot",
            output="screen",
            parameters=[{
                "usart_port_name": serial_port,
                "serial_baud_rate": baud,
                "robot_frame_id": "base_footprint",
                "gyro_frame_id": "gyro_link",
                "odom_frame_id": "odom",
                "cmd_vel": "/cmd_vel",
                "akm_cmd_vel": "none",
                "product_number": 0,
                "odom_x_scale": 1.0,
                "odom_y_scale": 1.0,
                "odom_z_scale_positive": 1.0,
                "odom_z_scale_negative": 1.0,
                "car_mode": "tank",
                "ranger_avoid_flag": False,
                "ultrasonic_avoid": False,
                "send_rate_hz": send_rate_hz,
                "cmd_timeout_sec": cmd_timeout_sec,
                "max_linear_x": max_linear_x,
                "max_linear_y": max_linear_y,
                "max_angular_z": max_angular_z,
                "brake_enable": brake_enable,
                "brake_ratio": brake_ratio,
                "brake_duration_sec": brake_duration_sec,
                "brake_duration_gain": brake_duration_gain,
                "brake_min_motion_sec": brake_min_motion_sec,
                "brake_trigger_threshold": brake_trigger_threshold,
            }],
        ),
    ])
