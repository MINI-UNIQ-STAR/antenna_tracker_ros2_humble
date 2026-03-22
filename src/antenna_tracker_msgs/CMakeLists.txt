cmake_minimum_required(VERSION 3.8)
project(antenna_tracker_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ImuFusion.msg"
  "msg/EncoderFeedback.msg"
  "msg/AntennaState.msg"
  "msg/MotorCommand.msg"
  "msg/TargetGPS.msg"
  "msg/TrackerDiagnostics.msg"
  "msg/Heartbeat.msg"
  "srv/SetMode.srv"
  "srv/SetManualTarget.srv"
  "srv/GetStatus.srv"
  "srv/SetZeroOffset.srv"
  "action/TrackTarget.action"
  DEPENDENCIES std_msgs
)

ament_package()
