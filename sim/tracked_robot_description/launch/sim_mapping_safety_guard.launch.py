from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch.substitutions import FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("tracked_robot_description")

    robot_xacro = PathJoinSubstitution([pkg_share, "urdf", "tracked_robot.urdf.xacro"])
    world = LaunchConfiguration("world")
    spawn_x = LaunchConfiguration("spawn_x")
    spawn_y = LaunchConfiguration("spawn_y")
    spawn_z = LaunchConfiguration("spawn_z")
    spawn_yaw = LaunchConfiguration("spawn_yaw")
    bridge_config = PathJoinSubstitution([pkg_share, "config", "ros_gz_bridge.yaml"])
    slam_config = LaunchConfiguration("slam_params")
    enable_safety_guard = LaunchConfiguration("enable_safety_guard")

    robot_description = {
        "robot_description": Command([FindExecutable(name="xacro"), " ", robot_xacro])
    }

    gz_sim_launch = PathJoinSubstitution(
        [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"]
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "tracked_robot",
            "-topic", "robot_description",
            "-x", spawn_x,
            "-y", spawn_y,
            "-z", spawn_z,
            "-Y", spawn_yaw,
        ],
        output="screen",
    )

    odom_tf_broadcaster = Node(
        package="tracked_robot_description",
        executable="odom_tf_broadcaster.py",
        name="odom_tf_broadcaster",
        output="screen",
        parameters=[{
            "odom_topic": "/odom",
            "odom_frame": "odom",
            "base_frame": "base_footprint",
            "publish_rate_hz": 30.0,
            "use_current_time": False,
            "publish_identity_until_odom": True,
            "use_sim_time": True,
        }],
    )

    odom_path_publisher = Node(
        package="tracked_robot_description",
        executable="odom_path_publisher.py",
        name="odom_path_publisher",
        output="screen",
        parameters=[{
            "odom_topic": "/odom",
            "path_topic": "/trajectory",
            "path_frame": "odom",
            "max_poses": 4000,
            "min_distance_m": 0.025,
            "use_sim_time": True,
        }],
    )

    scan_safety_guard = Node(
        package="k1_sensor_event_adapter",
        executable="scan_safety_guard_node",
        name="scan_safety_guard_node",
        condition=IfCondition(enable_safety_guard),
        output="screen",
        parameters=[{
            "scan_topic": "/scan",
            "input_cmd_topic": "/input_cmd_vel",
            "output_cmd_topic": "/cmd_vel_guarded",
            "stop_request_topic": "/chassis/stop_request",
            "status_topic": "/safety/front_obstacle",
            "event_topic": "/perception/mock_event",
            "front_sector_deg": 35.0,
            "hard_stop_m": 0.10,
            "slow_down_m": 0.20,
            "slow_clear_m": 0.30,
            "soft_max_linear": 2.00,
            "clear_max_linear": 2.00,
            "min_effective_forward": 0.03,
            "emergency_stop_m": 0.10,
            "approach_stop_m": 0.20,
            "approach_rate_stop_mps": 0.35,
            "ttc_stop_s": 1.20,
            "enable_dynamic_stop": False,
            "hard_stop_latch_s": 0.40,
            "fail_closed_without_scan": True,
            "publish_events": True,
            "output_rate_hz": 50.0,
            "use_sim_time": True,
        }],
    )

    slam_toolbox = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[slam_config, {"use_sim_time": True}],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world",
                default_value=PathJoinSubstitution(
                    [pkg_share, "worlds", "empty_tracked_robot.sdf"]
                ),
            ),
            DeclareLaunchArgument(
                "slam_params",
                default_value=PathJoinSubstitution(
                    [pkg_share, "config", "slam_toolbox_sim.yaml"]
                ),
            ),
            DeclareLaunchArgument("spawn_x", default_value="0.0"),
            DeclareLaunchArgument("spawn_y", default_value="0.0"),
            DeclareLaunchArgument("spawn_z", default_value="0.05"),
            DeclareLaunchArgument("spawn_yaw", default_value="0.0"),
            DeclareLaunchArgument(
                "enable_safety_guard",
                default_value="true",
                description="Start scan_safety_guard_node if k1_sensor_event_adapter is available.",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(gz_sim_launch),
                launch_arguments={"gz_args": ["-r ", world]}.items(),
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[robot_description, {"use_sim_time": True}],
                output="screen",
            ),
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                parameters=[{"config_file": bridge_config}],
                output="screen",
            ),
            TimerAction(period=2.0, actions=[spawn_robot]),
            TimerAction(period=3.0, actions=[odom_tf_broadcaster]),
            TimerAction(period=3.5, actions=[odom_path_publisher]),
            TimerAction(period=4.0, actions=[scan_safety_guard]),
            TimerAction(period=6.0, actions=[slam_toolbox]),
        ]
    )
