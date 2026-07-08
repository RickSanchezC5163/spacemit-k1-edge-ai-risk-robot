import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


def _nav2_nodes(
    configured_params,
    map_yaml,
    nav2_cmd_vel,
    use_sim_time,
    autostart,
    use_amcl,
    use_static_map_to_odom,
    map_to_odom_x,
    map_to_odom_y,
    map_to_odom_yaw,
):
    common_remaps = [("/tf", "tf"), ("/tf_static", "tf_static")]
    localization_nodes = ["map_server", "amcl"]
    navigation_nodes = [
        "controller_server",
        "smoother_server",
        "planner_server",
        "bt_navigator",
        "waypoint_follower",
        "velocity_smoother",
    ]

    return [
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            output="screen",
            parameters=[configured_params, {"yaml_filename": map_yaml}],
            remappings=common_remaps,
        ),
        Node(
            package="nav2_amcl",
            executable="amcl",
            name="amcl",
            output="screen",
            parameters=[configured_params],
            remappings=common_remaps + [("scan", "/scan")],
            condition=IfCondition(use_amcl),
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_localization",
            output="screen",
            parameters=[
                {"use_sim_time": use_sim_time},
                {"autostart": autostart},
                {"node_names": localization_nodes},
            ],
            condition=IfCondition(use_amcl),
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_map_only",
            output="screen",
            parameters=[
                {"use_sim_time": use_sim_time},
                {"autostart": autostart},
                {"node_names": ["map_server"]},
            ],
            condition=UnlessCondition(use_amcl),
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="map_to_odom_tf",
            output="screen",
            arguments=[
                map_to_odom_x,
                map_to_odom_y,
                "0.0",
                "0.0",
                "0.0",
                map_to_odom_yaw,
                "map",
                "odom",
            ],
            condition=IfCondition(use_static_map_to_odom),
        ),
        Node(
            package="nav2_controller",
            executable="controller_server",
            name="controller_server",
            output="screen",
            parameters=[configured_params],
            remappings=common_remaps + [("cmd_vel", "cmd_vel_nav")],
        ),
        Node(
            package="nav2_smoother",
            executable="smoother_server",
            name="smoother_server",
            output="screen",
            parameters=[configured_params],
            remappings=common_remaps,
        ),
        Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            output="screen",
            parameters=[configured_params],
            remappings=common_remaps,
        ),
        Node(
            package="nav2_bt_navigator",
            executable="bt_navigator",
            name="bt_navigator",
            output="screen",
            parameters=[configured_params],
            remappings=common_remaps,
        ),
        Node(
            package="nav2_waypoint_follower",
            executable="waypoint_follower",
            name="waypoint_follower",
            output="screen",
            parameters=[configured_params],
            remappings=common_remaps,
        ),
        Node(
            package="nav2_velocity_smoother",
            executable="velocity_smoother",
            name="velocity_smoother",
            output="screen",
            parameters=[configured_params],
            remappings=common_remaps + [
                ("cmd_vel", "cmd_vel_nav"),
                ("cmd_vel_smoothed", nav2_cmd_vel),
            ],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            output="screen",
            parameters=[
                {"use_sim_time": use_sim_time},
                {"autostart": autostart},
                {"node_names": navigation_nodes},
            ],
        ),
    ]


def generate_launch_description():
    pkg_share = get_package_share_directory("turn_on_wheeltec_robot")
    lidar_share = get_package_share_directory("lslidar_driver")
    default_map = os.path.join(
        os.path.expanduser("~"),
        "edge-ai-robot-k1",
        "maps",
        "mapping_fixed_odom_20260628_085623.yaml",
    )

    start_lidar = LaunchConfiguration("start_lidar")
    start_base = LaunchConfiguration("start_base")
    start_guard = LaunchConfiguration("start_guard")
    start_nav2 = LaunchConfiguration("start_nav2")
    autostart = LaunchConfiguration("autostart")
    use_sim_time = LaunchConfiguration("use_sim_time")
    nav2_params = LaunchConfiguration("nav2_params")
    map_yaml = LaunchConfiguration("map")
    nav2_cmd_vel = LaunchConfiguration("nav2_cmd_vel")
    guarded_cmd_vel = LaunchConfiguration("guarded_cmd_vel")
    stop_request_topic = LaunchConfiguration("stop_request_topic")
    use_amcl = LaunchConfiguration("use_amcl")
    use_static_map_to_odom = LaunchConfiguration("use_static_map_to_odom")
    map_to_odom_x = LaunchConfiguration("map_to_odom_x")
    map_to_odom_y = LaunchConfiguration("map_to_odom_y")
    map_to_odom_yaw = LaunchConfiguration("map_to_odom_yaw")

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=nav2_params,
            root_key="",
            param_rewrites={"use_sim_time": use_sim_time},
            convert_types=True,
        ),
        allow_substs=True,
    )

    return LaunchDescription(
        [
            SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"),
            DeclareLaunchArgument("start_lidar", default_value="true"),
            DeclareLaunchArgument("start_base", default_value="true"),
            DeclareLaunchArgument("start_guard", default_value="true"),
            DeclareLaunchArgument("start_nav2", default_value="true"),
            DeclareLaunchArgument("autostart", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("serial_port", default_value="/dev/base_controller"),
            DeclareLaunchArgument("map", default_value=default_map),
            DeclareLaunchArgument(
                "nav2_params",
                default_value=os.path.join(pkg_share, "config", "nav2_n10p_tank_guarded_map.yaml"),
            ),
            DeclareLaunchArgument("nav2_cmd_vel", default_value="/cmd_vel_raw"),
            DeclareLaunchArgument("guarded_cmd_vel", default_value="/cmd_vel_guarded"),
            DeclareLaunchArgument("stop_request_topic", default_value="/chassis/stop_request"),
            DeclareLaunchArgument("use_amcl", default_value="true"),
            DeclareLaunchArgument("use_static_map_to_odom", default_value="false"),
            DeclareLaunchArgument("map_to_odom_x", default_value="0.0"),
            DeclareLaunchArgument("map_to_odom_y", default_value="0.0"),
            DeclareLaunchArgument("map_to_odom_yaw", default_value="0.0"),
            DeclareLaunchArgument("laser_x", default_value="0.12"),
            DeclareLaunchArgument("laser_y", default_value="0.0"),
            DeclareLaunchArgument("laser_z", default_value="0.12"),
            DeclareLaunchArgument("laser_yaw", default_value="0.0"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(lidar_share, "launch", "lsn10p_launch.py")
                ),
                condition=IfCondition(start_lidar),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("turn_on_wheeltec_robot"),
                            "launch",
                            "tank_base_safe.launch.py",
                        ]
                    )
                ),
                launch_arguments={
                    "serial_port": LaunchConfiguration("serial_port"),
                    "cmd_vel_topic": guarded_cmd_vel,
                    "stop_request_topic": stop_request_topic,
                    "send_security_enable_on_start": "true",
                    "security_ply": "1",
                    "send_rate": "50.0",
                    "cmd_timeout": "0.25",
                    "max_linear": "0.45",
                    "max_angular": "0.80",
                    "cruise_linear_limit": "0.45",
                    "cruise_angular_limit": "0.80",
                    "brake_duration": "0.30",
                    "stop_kick_match_cmd": "true",
                    "stop_kick_match_duration": "false",
                    "stop_kick_speed_gain": "1.50",
                    "stop_kick_duration_mode": "fixed",
                    "stop_kick_duration": "0.55",
                    "stop_kick_max_duration": "0.75",
                    "stop_kick_min_duration": "0.12",
                    "stop_kick_until_stopped": "false",
                    "odom_linear_scale": "1.0",
                    "odom_angular_scale": "1.0",
                }.items(),
                condition=IfCondition(start_base),
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_to_laser_tf",
                arguments=[
                    LaunchConfiguration("laser_x"),
                    LaunchConfiguration("laser_y"),
                    LaunchConfiguration("laser_z"),
                    "0.0",
                    "0.0",
                    LaunchConfiguration("laser_yaw"),
                    "base_footprint",
                    "laser",
                ],
            ),
            Node(
                package="k1_sensor_event_adapter",
                executable="scan_safety_guard_node",
                name="scan_safety_guard_node",
                output="screen",
                parameters=[
                    {
                        "scan_topic": "/scan",
                        "input_cmd_topic": nav2_cmd_vel,
                        "output_cmd_topic": guarded_cmd_vel,
                        "stop_request_topic": stop_request_topic,
                        "status_topic": "/safety/front_obstacle",
                        "event_topic": "/perception/mock_event",
                        "front_sector_deg": 35.0,
                        "hard_stop_m": 1.00,
                        "slow_down_m": 1.60,
                        "soft_max_linear": 0.20,
                        "clear_max_linear": 0.20,
                        "min_effective_forward": 0.02,
                        "emergency_stop_m": 0.45,
                        "enable_dynamic_stop": False,
                        "hard_stop_latch_s": 1.50,
                        "output_rate_hz": 50.0,
                        "fail_closed_without_scan": True,
                        "publish_events": True,
                    }
                ],
                condition=IfCondition(start_guard),
            ),
            TimerAction(
                period=6.0,
                actions=[
                    GroupAction(
                        actions=_nav2_nodes(
                            configured_params,
                            map_yaml,
                            nav2_cmd_vel,
                            use_sim_time,
                            autostart,
                            use_amcl,
                            use_static_map_to_odom,
                            map_to_odom_x,
                            map_to_odom_y,
                            map_to_odom_yaw,
                        ),
                        condition=IfCondition(start_nav2),
                    ),
                ],
            ),
        ]
    )
