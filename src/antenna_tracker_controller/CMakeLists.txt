cmake_minimum_required(VERSION 3.8)
project(antenna_tracker_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Code coverage instrumentation (enable with -DENABLE_COVERAGE=ON)
option(ENABLE_COVERAGE "Build with gcov/lcov coverage instrumentation" OFF)
if(ENABLE_COVERAGE AND (CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang"))
  add_compile_options(-fprofile-arcs -ftest-coverage -O0 -g)
  add_link_options(-lgcov --coverage)
  message(STATUS "[coverage] instrumentation enabled")
endif()

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(antenna_tracker_msgs REQUIRED)

include_directories(include)

# Acados Integration
set(ACADOS_DIR "/opt/acados")
include_directories(
  ${ACADOS_DIR}/include
  ${ACADOS_DIR}/include/acados
  ${ACADOS_DIR}/include/blasfeo/include
  ${ACADOS_DIR}/include/hpipm/include
  ${CMAKE_CURRENT_SOURCE_DIR}/../../scripts/c_generated_code
)
link_directories(${ACADOS_DIR}/lib)

set(ACADOS_GENERATED_SOURCES
  ${CMAKE_CURRENT_SOURCE_DIR}/../../scripts/c_generated_code/acados_solver_antenna_tracker.c
  ${CMAKE_CURRENT_SOURCE_DIR}/../../scripts/c_generated_code/antenna_tracker_model/antenna_tracker_expl_ode_fun.c
  ${CMAKE_CURRENT_SOURCE_DIR}/../../scripts/c_generated_code/antenna_tracker_model/antenna_tracker_expl_vde_forw.c
  ${CMAKE_CURRENT_SOURCE_DIR}/../../scripts/c_generated_code/antenna_tracker_model/antenna_tracker_expl_vde_adj.c
)
# Libraries
add_library(complementary_filter src/complementary_filter.cpp)
ament_target_dependencies(complementary_filter rclcpp)

add_library(kalman_filter src/kalman_filter.cpp)
ament_target_dependencies(kalman_filter rclcpp)

add_library(pid_controller src/pid_controller.cpp)
ament_target_dependencies(pid_controller rclcpp)

add_library(mpc_controller src/mpc_controller.cpp ${ACADOS_GENERATED_SOURCES})
ament_target_dependencies(mpc_controller rclcpp)
target_link_libraries(mpc_controller acados hpipm blasfeo)
# Nodes
add_executable(sensor_fusion_node src/sensor_fusion_node.cpp)
target_link_libraries(sensor_fusion_node complementary_filter kalman_filter)
ament_target_dependencies(sensor_fusion_node
  rclcpp sensor_msgs antenna_tracker_msgs)

add_executable(navigation_node src/navigation_node.cpp)
ament_target_dependencies(navigation_node
  rclcpp sensor_msgs antenna_tracker_msgs)

add_executable(controller_node src/controller_node.cpp)
target_link_libraries(controller_node mpc_controller)
ament_target_dependencies(controller_node
  rclcpp rclcpp_action antenna_tracker_msgs)

add_executable(state_machine_node src/state_machine_node.cpp)
ament_target_dependencies(state_machine_node
  rclcpp antenna_tracker_msgs)

install(TARGETS
  sensor_fusion_node
  navigation_node
  controller_node
  state_machine_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_complementary_filter test/test_complementary_filter.cpp)
  target_link_libraries(test_complementary_filter complementary_filter)

  ament_add_gtest(test_kalman_filter test/test_kalman_filter.cpp)
  target_link_libraries(test_kalman_filter kalman_filter)

  ament_add_gtest(test_pid_controller test/test_pid_controller.cpp)
  target_link_libraries(test_pid_controller pid_controller)

  ament_add_gtest(test_navigation test/test_navigation.cpp src/navigation_node.cpp)
  target_compile_definitions(test_navigation PRIVATE ANTENNA_TRACKER_TEST_ENV)
  ament_target_dependencies(test_navigation rclcpp sensor_msgs antenna_tracker_msgs std_msgs)

  ament_add_gtest(test_mpc_controller test/test_mpc_controller.cpp)
  target_link_libraries(test_mpc_controller mpc_controller)
  ament_target_dependencies(test_mpc_controller rclcpp)
endif()

ament_package()
