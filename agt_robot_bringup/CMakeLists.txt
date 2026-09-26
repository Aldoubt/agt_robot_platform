cmake_minimum_required(VERSION 3.8)
project(agt_robot_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config tools
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS tools/check_robot_bringup_contract.py tools/robot_config.py tools/hardware_launch_plan.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_hardware_launch_plan test/test_hardware_launch_plan.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
  ament_add_pytest_test(test_robot_config test/test_robot_config.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
endif()

ament_package()
