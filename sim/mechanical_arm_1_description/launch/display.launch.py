from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch.substitutions import FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


HOME_JOINT_ZEROS = {
    "zeros": {
        # Calibrated from joint_state_publisher_gui on 2026-07-12.
        "j1": 0.415948,
        "j2": -1.5708,
        "j3": 1.8,
        "j4": -0.636802,
    }
}


def generate_launch_description():
    pkg_share = FindPackageShare("mechanical_arm_1_description")
    robot_urdf = PathJoinSubstitution(
        [pkg_share, "urdf", "mechanical_arm_1_visual.urdf"]
    )
    rviz_config = PathJoinSubstitution([pkg_share, "config", "arm_display.rviz"])

    robot_description = {
        "robot_description": Command([FindExecutable(name="xacro"), " ", robot_urdf])
    }

    return LaunchDescription(
        [
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[robot_description],
                output="screen",
            ),
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                parameters=[HOME_JOINT_ZEROS],
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", rviz_config],
                output="screen",
            ),
        ]
    )
