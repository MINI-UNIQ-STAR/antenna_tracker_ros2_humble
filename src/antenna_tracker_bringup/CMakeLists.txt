cmake_minimum_required(VERSION 3.8)
project(antenna_tracker_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_bringup_launch test/test_bringup_launch.py)
endif()

ament_package()
