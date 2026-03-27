cmake_minimum_required(VERSION 3.8)
project(antenna_tracker_simulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

option(ENABLE_COVERAGE "Build with gcov/lcov coverage instrumentation" OFF)
if(ENABLE_COVERAGE AND (CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang"))
  add_compile_options(-fprofile-arcs -ftest-coverage -O0 -g)
  add_link_options(-lgcov --coverage)
  message(STATUS "[coverage] instrumentation enabled")
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(ament_cmake_pytest REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(antenna_tracker_msgs REQUIRED)

include_directories(include)

add_library(sim_motor_model src/sim_motor_model.cpp)
ament_target_dependencies(sim_motor_model antenna_tracker_msgs)

add_executable(sim_motor_bridge_node src/sim_motor_bridge_node.cpp)
target_link_libraries(sim_motor_bridge_node sim_motor_model)
ament_target_dependencies(sim_motor_bridge_node rclcpp std_msgs sensor_msgs antenna_tracker_msgs)

install(TARGETS sim_motor_model sim_motor_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY urdf worlds launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  ament_add_gtest(test_sim_motor_model test/test_sim_motor_model.cpp)
  target_link_libraries(test_sim_motor_model sim_motor_model)
  ament_target_dependencies(test_sim_motor_model antenna_tracker_msgs)

  ament_add_pytest_test(test_sim_launch test/test_sim_launch.py)
endif()

ament_package()
