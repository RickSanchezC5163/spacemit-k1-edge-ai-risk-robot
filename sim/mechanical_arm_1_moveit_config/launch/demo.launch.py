import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
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


def load_yaml(package_share_dir, relative_path):
    path = package_share_dir / relative_path
    with path.open("r", encoding="utf-8") as handle:
        return yaml.safe_load(handle)


def generate_launch_description():
    from pathlib import Path

    moveit_share_dir = Path(get_package_share_directory("mechanical_arm_1_moveit_config"))
    desc_pkg = FindPackageShare("mechanical_arm_1_description")
    moveit_pkg = FindPackageShare("mechanical_arm_1_moveit_config")

    robot_urdf = PathJoinSubstitution(
        [desc_pkg, "urdf", "mechanical_arm_1_visual.urdf"]
    )
    semantic_file = moveit_share_dir / "srdf" / "mechanical_arm_1.srdf"
    rviz_config = PathJoinSubstitution([moveit_pkg, "config", "moveit.rviz"])

    robot_description = {
        "robot_description": Command([FindExecutable(name="xacro"), " ", robot_urdf])
    }
    robot_description_semantic = {
        "robot_description_semantic": semantic_file.read_text(encoding="utf-8")
    }

    kinematics = {
        "robot_description_kinematics": load_yaml(
            moveit_share_dir, "config/kinematics.yaml"
        )
    }
    joint_limits = {
        "robot_description_planning": load_yaml(
            moveit_share_dir, "config/joint_limits.yaml"
        )
    }
    ompl = {"ompl": load_yaml(moveit_share_dir, "config/ompl_planning.yaml")}
    controllers = load_yaml(moveit_share_dir, "config/moveit_controllers.yaml")
    scene_monitor = load_yaml(moveit_share_dir, "config/planning_scene_monitor.yaml")
    attach_to_chassis = LaunchConfiguration("attach_to_chassis")

    move_group_params = [
        robot_description,
        robot_description_semantic,
        kinematics,
        joint_limits,
        ompl,
        controllers,
        scene_monitor,
        {"use_sim_time": False},
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "attach_to_chassis",
                default_value="true",
                description=(
                    "Publish base_footprint -> base_link so the arm and head "
                    "camera move with the tracked chassis TF tree."
                ),
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="arm_base_to_chassis_tf",
                arguments=[
                    "-0.005",
                    "0.0",
                    "0.13",
                    "0.0",
                    "0.0",
                    "0.0",
                    "base_footprint",
                    "base_link",
                ],
                condition=IfCondition(attach_to_chassis),
                output="screen",
            ),
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
                package="moveit_ros_move_group",
                executable="move_group",
                output="screen",
                parameters=move_group_params,
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2_moveit",
                output="screen",
                arguments=["-d", rviz_config],
                parameters=[
                    robot_description,
                    robot_description_semantic,
                    kinematics,
                    joint_limits,
                    ompl,
                ],
            ),
        ]
    )
