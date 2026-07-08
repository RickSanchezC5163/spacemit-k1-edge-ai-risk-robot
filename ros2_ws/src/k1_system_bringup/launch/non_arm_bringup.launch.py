import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _as_bool(value: str) -> bool:
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def _include_pkg_launch(package_name: str, launch_file: str, launch_arguments=None):
    share = get_package_share_directory(package_name)
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, "launch", launch_file)),
        launch_arguments=(launch_arguments or {}).items(),
    )


def _build_actions(context, *args, **kwargs):
    actions = [
        LogInfo(msg="Starting K1 non-arm bring-up. This launch does not publish /cmd_vel."),
    ]

    use_base = _as_bool(LaunchConfiguration("use_base").perform(context))
    use_lidar = _as_bool(LaunchConfiguration("use_lidar").perform(context))
    use_camera = _as_bool(LaunchConfiguration("use_camera").perform(context))
    use_light = _as_bool(LaunchConfiguration("use_light").perform(context))
    use_light_action_bridge = _as_bool(
        LaunchConfiguration("use_light_action_bridge").perform(context)
    )
    use_adaptive_light_controller = _as_bool(
        LaunchConfiguration("use_adaptive_light_controller").perform(context)
    )
    use_risk_engine = _as_bool(LaunchConfiguration("use_risk_engine").perform(context))
    use_event_logger = _as_bool(LaunchConfiguration("use_event_logger").perform(context))
    use_scan_event_adapter = _as_bool(LaunchConfiguration("use_scan_event_adapter").perform(context))
    use_camera_low_light_adapter = _as_bool(
        LaunchConfiguration("use_camera_low_light_adapter").perform(context)
    )

    if use_light_action_bridge and use_adaptive_light_controller:
        actions.append(
            LogInfo(
                msg=(
                    "WARNING: both use_light_action_bridge and "
                    "use_adaptive_light_controller are enabled; both publish "
                    "/light/brightness_cmd. Use only one controller in normal tests."
                )
            )
        )

    if use_base:
        actions.append(
            _include_pkg_launch(
                "turn_on_wheeltec_robot",
                "tank_base_brake.launch.py",
                {
                    "serial_port": LaunchConfiguration("base_serial_port").perform(context),
                    "baud": LaunchConfiguration("base_baud").perform(context),
                    "max_linear_x": LaunchConfiguration("max_linear_x").perform(context),
                    "max_angular_z": LaunchConfiguration("max_angular_z").perform(context),
                },
            )
        )
    else:
        actions.append(LogInfo(msg="Base driver disabled by use_base:=false."))

    if use_lidar:
        actions.append(
            _include_pkg_launch(
                LaunchConfiguration("lidar_launch_package").perform(context),
                LaunchConfiguration("lidar_launch_file").perform(context),
                {"lidar_type": LaunchConfiguration("lidar_type").perform(context)},
            )
        )
    else:
        actions.append(LogInfo(msg="Lidar disabled by use_lidar:=false."))

    if use_camera:
        actions.append(
            _include_pkg_launch(
                "realsense2_camera",
                "rs_launch.py",
                {
                    "enable_color": LaunchConfiguration("camera_enable_color").perform(context),
                    "enable_depth": LaunchConfiguration("camera_enable_depth").perform(context),
                    "enable_infra": LaunchConfiguration("camera_enable_infra").perform(context),
                    "depth_module.depth_profile": LaunchConfiguration("depth_profile").perform(context),
                    "depth_module.infra_profile": LaunchConfiguration("infra_profile").perform(context),
                    "rgb_camera.color_profile": LaunchConfiguration("color_profile").perform(context),
                    "pointcloud.enable": LaunchConfiguration("enable_pointcloud").perform(context),
                },
            )
        )
    else:
        actions.append(LogInfo(msg="Camera disabled by use_camera:=false."))

    if use_light:
        actions.append(
            Node(
                package="k1_light_control",
                executable="gpio37_light_node",
                name="gpio37_light_node",
                output="screen",
                parameters=[
                    {
                        "gpio": LaunchConfiguration("light_gpio"),
                        "default_brightness": 0,
                        "frequency": LaunchConfiguration("light_frequency"),
                        "dry_run": LaunchConfiguration("light_dry_run"),
                    }
                ],
            )
        )

    if use_light_action_bridge:
        actions.append(
            Node(
                package="k1_light_control",
                executable="risk_light_bridge_node",
                name="risk_light_bridge_node",
                output="screen",
                parameters=[
                    {
                        "action_topic": LaunchConfiguration("risk_action_topic"),
                        "brightness_topic": LaunchConfiguration("light_brightness_topic"),
                        "trigger_action": LaunchConfiguration("light_trigger_action"),
                        "on_brightness": LaunchConfiguration("light_auto_on_brightness"),
                        "off_brightness": LaunchConfiguration("light_auto_off_brightness"),
                        "hold_seconds": LaunchConfiguration("light_auto_hold_seconds"),
                        "ramp_step_percent": LaunchConfiguration("light_auto_ramp_step_percent"),
                        "ramp_period_s": LaunchConfiguration("light_auto_ramp_period_s"),
                        "command_period_s": LaunchConfiguration("light_auto_command_period_s"),
                        "dry_run": LaunchConfiguration("light_action_bridge_dry_run"),
                    }
                ],
            )
        )
    else:
        actions.append(LogInfo(msg="Light action bridge disabled by use_light_action_bridge:=false."))

    if use_adaptive_light_controller:
        actions.append(
            Node(
                package="k1_light_control",
                executable="adaptive_light_controller_node",
                name="adaptive_light_controller_node",
                output="screen",
                parameters=[
                    {
                        "image_topic": LaunchConfiguration("adaptive_light_image_topic"),
                        "brightness_topic": LaunchConfiguration("light_brightness_topic"),
                        "status_topic": LaunchConfiguration("adaptive_light_status_topic"),
                        "target_luma": LaunchConfiguration("adaptive_light_target_luma"),
                        "dark_pixel_threshold": LaunchConfiguration(
                            "adaptive_light_dark_pixel_threshold"
                        ),
                        "min_brightness": LaunchConfiguration("adaptive_light_min_brightness"),
                        "max_brightness": LaunchConfiguration("adaptive_light_max_brightness"),
                        "update_rate_hz": LaunchConfiguration("adaptive_light_update_rate_hz"),
                        "step_limit": LaunchConfiguration("adaptive_light_step_limit"),
                        "stable_frames": LaunchConfiguration("adaptive_light_stable_frames"),
                        "resize_width": LaunchConfiguration("adaptive_light_resize_width"),
                        "image_timeout_s": LaunchConfiguration("adaptive_light_image_timeout_s"),
                        "enable_auto_light": LaunchConfiguration("enable_auto_light"),
                        "dry_run": LaunchConfiguration("adaptive_light_dry_run"),
                    }
                ],
            )
        )
    else:
        actions.append(
            LogInfo(msg="Adaptive light controller disabled by use_adaptive_light_controller:=false.")
        )

    if use_scan_event_adapter:
        actions.append(
            Node(
                package="k1_sensor_event_adapter",
                executable="scan_event_adapter_node",
                name="scan_event_adapter_node",
                output="screen",
                parameters=[
                    {
                        "scan_topic": LaunchConfiguration("scan_topic"),
                        "event_topic": LaunchConfiguration("perception_event_topic"),
                        "front_sector_deg": LaunchConfiguration("front_sector_deg"),
                        "soft_threshold_m": LaunchConfiguration("scan_soft_threshold_m"),
                        "blocked_threshold_m": LaunchConfiguration("scan_blocked_threshold_m"),
                        "publish_rate_hz": LaunchConfiguration("scan_event_rate_hz"),
                        "min_valid_range_m": LaunchConfiguration("scan_min_valid_range_m"),
                        "max_valid_range_m": LaunchConfiguration("scan_max_valid_range_m"),
                        "dry_run": LaunchConfiguration("sensor_adapter_dry_run"),
                    }
                ],
            )
        )
    else:
        actions.append(LogInfo(msg="Scan event adapter disabled by use_scan_event_adapter:=false."))

    if use_camera_low_light_adapter:
        actions.append(
            Node(
                package="k1_sensor_event_adapter",
                executable="camera_low_light_adapter_node",
                name="camera_low_light_adapter_node",
                output="screen",
                parameters=[
                    {
                        "image_topic": LaunchConfiguration("camera_image_topic"),
                        "event_topic": LaunchConfiguration("perception_event_topic"),
                        "luma_threshold": LaunchConfiguration("camera_luma_threshold"),
                        "dark_pixel_threshold": LaunchConfiguration("camera_dark_pixel_threshold"),
                        "dark_ratio_threshold": LaunchConfiguration("camera_dark_ratio_threshold"),
                        "publish_rate_hz": LaunchConfiguration("camera_event_rate_hz"),
                        "resize_width": LaunchConfiguration("camera_resize_width"),
                        "warmup_seconds": LaunchConfiguration("camera_warmup_seconds"),
                        "required_consecutive_frames": LaunchConfiguration(
                            "camera_required_consecutive_frames"
                        ),
                        "dry_run": LaunchConfiguration("sensor_adapter_dry_run"),
                    }
                ],
            )
        )
    else:
        actions.append(
            LogInfo(msg="Camera low-light adapter disabled by use_camera_low_light_adapter:=false.")
        )

    if use_risk_engine:
        actions.append(
            Node(
                package="k1_risk_engine",
                executable="risk_engine_node",
                name="risk_engine_node",
                output="screen",
            )
        )

    if use_event_logger:
        actions.append(
            Node(
                package="k1_event_logger",
                executable="event_logger_node",
                name="event_logger_node",
                output="screen",
                parameters=[{"log_dir": LaunchConfiguration("event_log_dir")}],
            )
        )

    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_base", default_value="false"),
            DeclareLaunchArgument("use_lidar", default_value="false"),
            DeclareLaunchArgument("use_camera", default_value="false"),
            DeclareLaunchArgument("use_light", default_value="true"),
            DeclareLaunchArgument("use_light_action_bridge", default_value="false"),
            DeclareLaunchArgument("use_adaptive_light_controller", default_value="false"),
            DeclareLaunchArgument("use_risk_engine", default_value="true"),
            DeclareLaunchArgument("use_event_logger", default_value="true"),
            DeclareLaunchArgument("use_scan_event_adapter", default_value="false"),
            DeclareLaunchArgument("use_camera_low_light_adapter", default_value="false"),
            DeclareLaunchArgument("base_serial_port", default_value="/dev/base_controller"),
            DeclareLaunchArgument("base_baud", default_value="115200"),
            DeclareLaunchArgument("max_linear_x", default_value="0.45"),
            DeclareLaunchArgument("max_angular_z", default_value="2.40"),
            DeclareLaunchArgument("lidar_launch_package", default_value="turn_on_wheeltec_robot"),
            DeclareLaunchArgument("lidar_launch_file", default_value="wheeltec_lidar.launch.py"),
            DeclareLaunchArgument("lidar_type", default_value="ls_N10Plus_uart"),
            DeclareLaunchArgument("camera_enable_color", default_value="true"),
            DeclareLaunchArgument("camera_enable_depth", default_value="false"),
            DeclareLaunchArgument("camera_enable_infra", default_value="false"),
            DeclareLaunchArgument("depth_profile", default_value="640,480,30"),
            DeclareLaunchArgument("infra_profile", default_value="640,480,30"),
            DeclareLaunchArgument("color_profile", default_value="640,480,30"),
            DeclareLaunchArgument("enable_pointcloud", default_value="false"),
            DeclareLaunchArgument("light_gpio", default_value="37"),
            DeclareLaunchArgument("light_frequency", default_value="50.0"),
            DeclareLaunchArgument("light_dry_run", default_value="true"),
            DeclareLaunchArgument("risk_action_topic", default_value="/risk/recommended_action"),
            DeclareLaunchArgument("light_brightness_topic", default_value="/light/brightness_cmd"),
            DeclareLaunchArgument("light_trigger_action", default_value="turn_on_light_and_recheck"),
            DeclareLaunchArgument("light_auto_on_brightness", default_value="5"),
            DeclareLaunchArgument("light_auto_off_brightness", default_value="0"),
            DeclareLaunchArgument("light_auto_hold_seconds", default_value="8.0"),
            DeclareLaunchArgument("light_auto_ramp_step_percent", default_value="5"),
            DeclareLaunchArgument("light_auto_ramp_period_s", default_value="0.2"),
            DeclareLaunchArgument("light_auto_command_period_s", default_value="1.0"),
            DeclareLaunchArgument("light_action_bridge_dry_run", default_value="false"),
            DeclareLaunchArgument(
                "adaptive_light_image_topic", default_value="/camera/camera/color/image_raw"
            ),
            DeclareLaunchArgument("adaptive_light_status_topic", default_value="/light/adaptive_status"),
            DeclareLaunchArgument("adaptive_light_target_luma", default_value="75.0"),
            DeclareLaunchArgument("adaptive_light_dark_pixel_threshold", default_value="50.0"),
            DeclareLaunchArgument("adaptive_light_min_brightness", default_value="0"),
            DeclareLaunchArgument("adaptive_light_max_brightness", default_value="5"),
            DeclareLaunchArgument("adaptive_light_update_rate_hz", default_value="1.0"),
            DeclareLaunchArgument("adaptive_light_step_limit", default_value="5"),
            DeclareLaunchArgument("adaptive_light_stable_frames", default_value="3"),
            DeclareLaunchArgument("adaptive_light_resize_width", default_value="320"),
            DeclareLaunchArgument("adaptive_light_image_timeout_s", default_value="3.0"),
            DeclareLaunchArgument("adaptive_light_dry_run", default_value="false"),
            DeclareLaunchArgument("enable_auto_light", default_value="true"),
            DeclareLaunchArgument("event_log_dir", default_value="logs/events"),
            DeclareLaunchArgument("perception_event_topic", default_value="/perception/mock_event"),
            DeclareLaunchArgument("scan_topic", default_value="/scan"),
            DeclareLaunchArgument("front_sector_deg", default_value="30.0"),
            DeclareLaunchArgument("scan_soft_threshold_m", default_value="1.0"),
            DeclareLaunchArgument("scan_blocked_threshold_m", default_value="0.5"),
            DeclareLaunchArgument("scan_event_rate_hz", default_value="2.0"),
            DeclareLaunchArgument("scan_min_valid_range_m", default_value="0.05"),
            DeclareLaunchArgument("scan_max_valid_range_m", default_value="8.0"),
            DeclareLaunchArgument("camera_image_topic", default_value="/camera/camera/color/image_raw"),
            DeclareLaunchArgument("camera_luma_threshold", default_value="55.0"),
            DeclareLaunchArgument("camera_dark_pixel_threshold", default_value="50.0"),
            DeclareLaunchArgument("camera_dark_ratio_threshold", default_value="0.6"),
            DeclareLaunchArgument("camera_event_rate_hz", default_value="1.0"),
            DeclareLaunchArgument("camera_resize_width", default_value="320"),
            DeclareLaunchArgument("camera_warmup_seconds", default_value="3.0"),
            DeclareLaunchArgument("camera_required_consecutive_frames", default_value="2"),
            DeclareLaunchArgument("sensor_adapter_dry_run", default_value="false"),
            OpaqueFunction(function=_build_actions),
        ]
    )
