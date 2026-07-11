from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch.substitutions import FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("mechanical_arm_1_description")
    robot_urdf = PathJoinSubstitution(
        [pkg_share, "urdf", "mechanical_arm_1_visual.urdf"]
    )

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
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                output="screen",
            ),
        ]
    )
