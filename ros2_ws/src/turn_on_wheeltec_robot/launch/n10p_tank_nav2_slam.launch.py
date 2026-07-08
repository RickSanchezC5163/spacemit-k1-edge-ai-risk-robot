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
