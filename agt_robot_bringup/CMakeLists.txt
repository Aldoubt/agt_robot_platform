cmake_minimum_required(VERSION 3.8)
project(agt_robot_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS tools/check_robot_bringup_contract.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
