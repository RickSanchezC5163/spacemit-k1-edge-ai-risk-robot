import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("turn_on_wheeltec_robot")
    lidar_share = get_package_share_directory("lslidar_driver")

    serial_port = LaunchConfiguration("serial_port")
    baud = LaunchConfiguration("baud")
    send_rate_hz = LaunchConfiguration("send_rate_hz")
    cmd_timeout_sec = LaunchConfiguration("cmd_timeout_sec")
    max_linear_x = LaunchConfiguration("max_linear_x")
    max_angular_z = LaunchConfiguration("max_angular_z")
    brake_duration_sec = LaunchConfiguration("brake_duration_sec")
    brake_speed_gain = LaunchConfiguration("brake_speed_gain")
    brake_duration_mode = LaunchConfiguration("brake_duration_mode")
    brake_duration_gain = LaunchConfiguration("brake_duration_gain")
    brake_impulse_ratio = LaunchConfiguration("brake_impulse_ratio")
    brake_duration_offset = LaunchConfiguration("brake_duration_offset")
    brake_min_motion_sec = LaunchConfiguration("brake_min_motion_sec")
    brake_trigger_threshold = LaunchConfiguration("brake_trigger_threshold")
    base_cmd_vel = LaunchConfiguration("base_cmd_vel")
    stop_request_topic = LaunchConfiguration("stop_request_topic")
    enable_chassis_security = LaunchConfiguration("enable_chassis_security")
    security_ply = LaunchConfiguration("security_ply")
    publish_tf = LaunchConfiguration("publish_tf")
    laser_x = LaunchConfiguration("laser_x")
    laser_y = LaunchConfiguration("laser_y")
    laser_z = LaunchConfiguration("laser_z")
    laser_yaw = LaunchConfiguration("laser_yaw")
    slam_params = LaunchConfiguration("slam_params")

    return LaunchDescription([
        DeclareLaunchArgument("serial_port", default_value="/dev/base_controller"),
        DeclareLaunchArgument("baud", default_value="115200"),
        DeclareLaunchArgument("send_rate_hz", default_value="50.0"),
        DeclareLaunchArgument("cmd_timeout_sec", default_value="0.25"),
        DeclareLaunchArgument("max_linear_x", default_value="0.45"),
        DeclareLaunchArgument("max_angular_z", default_value="0.80"),
        DeclareLaunchArgument("brake_duration_sec", default_value="1.50"),
        DeclareLaunchArgument("brake_speed_gain", default_value="1.0"),
        DeclareLaunchArgument("brake_duration_mode", default_value="duration_ratio"),
        DeclareLaunchArgument("brake_duration_gain", default_value="0.45"),
        DeclareLaunchArgument("brake_impulse_ratio", default_value="1.0"),
        DeclareLaunchArgument("brake_duration_offset", default_value="0.0"),
        DeclareLaunchArgument("brake_min_motion_sec", default_value="0.12"),
        DeclareLaunchArgument("brake_trigger_threshold", default_value="0.02"),
        DeclareLaunchArgument("base_cmd_vel", default_value="/cmd_vel"),
        DeclareLaunchArgument("stop_request_topic", default_value="/chassis/stop_request"),
        DeclareLaunchArgument("enable_chassis_security", default_value="true"),
        DeclareLaunchArgument("security_ply", default_value="1"),
        DeclareLaunchArgument("publish_tf", default_value="true"),
        DeclareLaunchArgument("laser_x", default_value="0.12"),
        DeclareLaunchArgument("laser_y", default_value="0.0"),
        DeclareLaunchArgument("laser_z", default_value="0.12"),
        DeclareLaunchArgument("laser_yaw", default_value="0.0"),
        DeclareLaunchArgument(
            "slam_params",
            default_value=os.path.join(pkg_share, "config", "slam_toolbox_n10p_tank.yaml"),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(lidar_share, "launch", "lsn10p_launch.py")
            )
        ),
        Node(
            package="turn_on_wheeltec_robot",
            executable="wheeltec_tank_base_safe.py",
            name="wheeltec_tank_base",
            output="screen",
            parameters=[{
                "port": serial_port,
                "baud": baud,
                "auto_recharge": 0,
                "security_ply": security_ply,
                "send_security_enable_on_start": enable_chassis_security,
                "cmd_vel_topic": base_cmd_vel,
                "stop_request_topic": stop_request_topic,
                "send_rate": send_rate_hz,
                "cmd_timeout": cmd_timeout_sec,
                "max_linear": max_linear_x,
                "max_angular": max_angular_z,
                "cruise_linear_limit": max_linear_x,
                "cruise_angular_limit": max_angular_z,
                "brake_duration": 1.0,
                "stop_kick_match_cmd": True,
                "stop_kick_match_duration": True,
                "stop_kick_speed_gain": brake_speed_gain,
                "stop_kick_duration_mode": brake_duration_mode,
                "stop_kick_duration_ratio": brake_duration_gain,
                "stop_kick_impulse_ratio": brake_impulse_ratio,
                "stop_kick_duration_offset": brake_duration_offset,
                "stop_kick_max_duration": brake_duration_sec,
                "stop_kick_min_duration": brake_min_motion_sec,
                "stop_kick_until_stopped": False,
                "stop_kick_velocity_epsilon": brake_trigger_threshold,
                "odom_frame": "odom",
                "base_frame": "base_footprint",
                "imu_frame": "gyro_link",
                "publish_tf": publish_tf,
            }],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_to_laser_tf",
            arguments=[
                laser_x, laser_y, laser_z,
                "0.0", "0.0", laser_yaw,
                "base_footprint", "laser",
            ],
        ),
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package="slam_toolbox",
                    executable="async_slam_toolbox_node",
                    name="slam_toolbox",
                    output="screen",
                    parameters=[slam_params],
                ),
            ],
        ),
    ])
