cmake_minimum_required(VERSION 3.8)
project(antenna_tracker_simulation)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(antenna_tracker_msgs REQUIRED)

add_executable(sim_motor_bridge_node src/sim_motor_bridge_node.cpp)
ament_target_dependencies(sim_motor_bridge_node rclcpp std_msgs sensor_msgs antenna_tracker_msgs)

install(TARGETS sim_motor_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY urdf worlds launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
