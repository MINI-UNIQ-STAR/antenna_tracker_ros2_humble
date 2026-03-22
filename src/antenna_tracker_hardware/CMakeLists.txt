cmake_minimum_required(VERSION 3.8)
project(antenna_tracker_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(antenna_tracker_msgs REQUIRED)

include_directories(include)

add_executable(can_bridge_node src/can_bridge_node.cpp)
ament_target_dependencies(can_bridge_node
  rclcpp
  sensor_msgs
  std_msgs
  antenna_tracker_msgs
)

install(TARGETS
  can_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
