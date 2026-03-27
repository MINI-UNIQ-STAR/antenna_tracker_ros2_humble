cmake_minimum_required(VERSION 3.8)
project(antenna_tracker_hardware)

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
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(antenna_tracker_msgs REQUIRED)

include_directories(include)

add_library(can_frame_codec src/can_frame_codec.cpp)
ament_target_dependencies(can_frame_codec
  sensor_msgs
  std_msgs
  antenna_tracker_msgs
)

add_executable(can_bridge_node src/can_bridge_node.cpp)
target_link_libraries(can_bridge_node can_frame_codec)
ament_target_dependencies(can_bridge_node
  rclcpp
  sensor_msgs
  std_msgs
  antenna_tracker_msgs
)

install(TARGETS
  can_frame_codec
  can_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_can_frame_codec test/test_can_frame_codec.cpp)
  target_link_libraries(test_can_frame_codec can_frame_codec)
  ament_target_dependencies(test_can_frame_codec sensor_msgs std_msgs antenna_tracker_msgs)
endif()

ament_package()
