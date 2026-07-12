from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("tracked_robot_description")
    rviz_config = PathJoinSubstitution([pkg_share, "config", "sim_mapping.rviz"])

    output_dir = LaunchConfiguration("output_dir")
    publish_all_as_detected = LaunchConfiguration("publish_all_as_detected")
    send_nav2_action = LaunchConfiguration("send_nav2_action")
    start_rviz = LaunchConfiguration("start_rviz")
    detection_radius_m = LaunchConfiguration("detection_radius_m")
    arrival_tolerance_m = LaunchConfiguration("arrival_tolerance_m")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "output_dir",
                default_value="/tmp/k1_sim_risk_approach",
                description="Directory for risk approach JSONL records and camera snapshots.",
            ),
            DeclareLaunchArgument(
                "publish_all_as_detected",
                default_value="false",
                description="If true, bypass FOV/range gating and publish every simulated risk as detected.",
            ),
            DeclareLaunchArgument(
                "send_nav2_action",
                default_value="true",
                description="Send NavigateToPose action in addition to publishing /goal_pose.",
            ),
            DeclareLaunchArgument(
                "start_rviz",
                default_value="true",
                description="Open RViz with sim_mapping.rviz.",
            ),
            DeclareLaunchArgument(
                "detection_radius_m",
                default_value="1.8",
                description="Simulated D435/YOLO detection radius for risk cards.",
            ),
            DeclareLaunchArgument(
                "arrival_tolerance_m",
                default_value="0.18",
                description="Goal-distance threshold for recording a risk observation.",
            ),
            Node(
                package="tracked_robot_description",
                executable="sim_risk_marker_detector.py",
                name="sim_risk_marker_detector",
                output="screen",
                parameters=[
                    {
                        "publish_all_as_detected": publish_all_as_detected,
                        "detection_radius_m": detection_radius_m,
                        "camera_fov_deg": 115.0,
                        "map_frame": "map",
                        "base_frame": "base_footprint",
                        "use_sim_time": True,
                    }
                ],
            ),
            Node(
                package="tracked_robot_description",
                executable="risk_approach_goal_node.py",
                name="risk_approach_goal_node",
                output="screen",
                parameters=[
                    {
                        "output_dir": output_dir,
                        "stand_off_m": 0.65,
                        "arrival_tolerance_m": arrival_tolerance_m,
                        "settle_time_s": 1.2,
                        "send_nav2_action": send_nav2_action,
                        "map_frame": "map",
                        "base_frame": "base_footprint",
                        "use_sim_time": True,
                    }
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", rviz_config],
                parameters=[{"use_sim_time": True}],
                condition=IfCondition(start_rviz),
                output="screen",
            ),
        ]
    )
