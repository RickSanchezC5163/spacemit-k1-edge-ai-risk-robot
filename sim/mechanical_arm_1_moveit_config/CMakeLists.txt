cmake_minimum_required(VERSION 3.8)
project(mechanical_arm_1_moveit_config)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch srdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
