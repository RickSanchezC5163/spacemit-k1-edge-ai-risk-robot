from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.actions import TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch.substitutions import FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("tracked_robot_description")

    robot_xacro = PathJoinSubstitution([pkg_share, "urdf", "tracked_robot.urdf.xacro"])
    world = LaunchConfiguration("world")
    bridge_config = PathJoinSubstitution([pkg_share, "config", "ros_gz_bridge.yaml"])

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
            "-name",
            "tracked_robot",
            "-topic",
            "robot_description",
            "-x",
            "0.0",
            "-y",
            "0.0",
            "-z",
            "0.05",
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

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world",
                default_value=PathJoinSubstitution(
                    [pkg_share, "worlds", "empty_tracked_robot.sdf"]
                ),
            ),
            SetEnvironmentVariable(name="GZ_SIM_RESOURCE_PATH", value=pkg_share),
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
        ]
    )
