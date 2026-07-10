cmake_minimum_required(VERSION 3.8)
project(tracked_robot_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config docs launch meshes urdf worlds
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/odom_tf_broadcaster.py
    scripts/odom_path_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
