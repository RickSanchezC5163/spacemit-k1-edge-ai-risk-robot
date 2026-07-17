import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def _nav2_nodes(configured_params, nav2_cmd_vel, use_sim_time, nav2_autostart):
    common_remaps = [("/tf", "tf"), ("/tf_static", "tf_static")]
    lifecycle_nodes = [
        "controller_server",
        "smoother_server",
        "planner_server",
        "behavior_server",
        "bt_navigator",
        "waypoint_follower",
        "velocity_smoother",
    ]

    return [
        Node(
            package="nav2_controller",
            executable="controller_server",
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
            package="nav2_behaviors",
            executable="behavior_server",
            name="behavior_server",
            output="screen",
            parameters=[configured_params],
            remappings=common_remaps + [("cmd_vel", nav2_cmd_vel)],
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
            remappings=common_remaps + [("cmd_vel", "cmd_vel_nav"), ("cmd_vel_smoothed", nav2_cmd_vel)],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            output="screen",
            parameters=[
                {"use_sim_time": use_sim_time},
                {"autostart": nav2_autostart},
                {"node_names": lifecycle_nodes},
            ],
        ),
    ]


def generate_launch_description():
    pkg_share = get_package_share_directory("turn_on_wheeltec_robot")

    start_mapping_stack = LaunchConfiguration("start_mapping_stack")
    start_nav2 = LaunchConfiguration("start_nav2")
    nav2_params = LaunchConfiguration("nav2_params")
    nav2_autostart = LaunchConfiguration("nav2_autostart")
    nav2_cmd_vel = LaunchConfiguration("nav2_cmd_vel")
    guarded_cmd_vel = LaunchConfiguration("guarded_cmd_vel")
    use_sim_time = LaunchConfiguration("use_sim_time")
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
    emergency_stop_m = LaunchConfiguration("emergency_stop_m")
    slow_down_m = LaunchConfiguration("slow_down_m")
    approach_stop_m = LaunchConfiguration("approach_stop_m")
    min_effective_forward = LaunchConfiguration("min_effective_forward")
    clear_max_linear = LaunchConfiguration("clear_max_linear")
    soft_max_linear = LaunchConfiguration("soft_max_linear")

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=nav2_params,
            root_key="",
            param_rewrites={"use_sim_time": use_sim_time, "autostart": nav2_autostart},
            convert_types=True,
        ),
        allow_substs=True,
    )

    return LaunchDescription(
        [
            SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"),
            DeclareLaunchArgument("start_mapping_stack", default_value="true"),
            DeclareLaunchArgument("start_nav2", default_value="true"),
            DeclareLaunchArgument("nav2_autostart", default_value="true"),
            DeclareLaunchArgument("nav2_cmd_vel", default_value="/cmd_vel_raw"),
            DeclareLaunchArgument("guarded_cmd_vel", default_value="/cmd_vel_guarded"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
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
            DeclareLaunchArgument("emergency_stop_m", default_value="0.45"),
            DeclareLaunchArgument("slow_down_m", default_value="1.60"),
            DeclareLaunchArgument("approach_stop_m", default_value="1.60"),
            DeclareLaunchArgument("min_effective_forward", default_value="0.08"),
            DeclareLaunchArgument("clear_max_linear", default_value="0.30"),
            DeclareLaunchArgument("soft_max_linear", default_value="0.30"),
            DeclareLaunchArgument(
                "nav2_params",
                default_value=os.path.join(pkg_share, "config", "nav2_n10p_tank_guarded.yaml"),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_share, "launch", "n10p_tank_mapping_safety_guard.launch.py")
                ),
                launch_arguments={
                    "input_cmd_vel": nav2_cmd_vel,
                    "guarded_cmd_vel": guarded_cmd_vel,
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
                    "emergency_stop_m": emergency_stop_m,
                    "slow_down_m": slow_down_m,
                    "approach_stop_m": approach_stop_m,
                    "min_effective_forward": min_effective_forward,
                    "clear_max_linear": clear_max_linear,
                    "soft_max_linear": soft_max_linear,
                }.items(),
                condition=IfCondition(start_mapping_stack),
            ),
            TimerAction(
                period=8.0,
                actions=[
                    GroupAction(
                        actions=_nav2_nodes(configured_params, nav2_cmd_vel, use_sim_time, nav2_autostart),
                        condition=IfCondition(start_nav2),
                    ),
                ],
            ),
        ]
    )
