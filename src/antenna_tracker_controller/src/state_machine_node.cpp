#include "antenna_tracker_controller/state_machine_node.hpp"
#include <fstream>

namespace antenna_tracker_controller
{

StateMachineNode::StateMachineNode(const rclcpp::NodeOptions & options)
: Node("state_machine_node", options)
{
  declare_parameter<double>("imu_timeout_sec", 1.0);
  declare_parameter<double>("lora_timeout_sec", 12.0);
  declare_parameter<double>("diagnostics_rate_hz", 1.0);

  sub_state_ = create_subscription<antenna_tracker_msgs::msg::AntennaState>(
    "/antenna/state", 10,
    std::bind(&StateMachineNode::state_callback, this, std::placeholders::_1));

  sub_fusion_ = create_subscription<antenna_tracker_msgs::msg::ImuFusion>(
    "/antenna/imu_fusion", rclcpp::SensorDataQoS(),
    std::bind(&StateMachineNode::fusion_callback, this, std::placeholders::_1));

  sub_encoder_ = create_subscription<antenna_tracker_msgs::msg::EncoderFeedback>(
    "/antenna/encoder_feedback", rclcpp::SensorDataQoS(),
    std::bind(&StateMachineNode::encoder_callback, this, std::placeholders::_1));

  sub_target_ = create_subscription<antenna_tracker_msgs::msg::TargetGPS>(
    "/antenna/target_gps", rclcpp::SensorDataQoS(),
    std::bind(&StateMachineNode::target_callback, this, std::placeholders::_1));

  sub_heartbeat_ = create_subscription<antenna_tracker_msgs::msg::Heartbeat>(
    "/antenna/heartbeat", 10,
    std::bind(&StateMachineNode::heartbeat_callback, this, std::placeholders::_1));

  pub_diagnostics_ = create_publisher<antenna_tracker_msgs::msg::TrackerDiagnostics>(
    "/antenna/diagnostics", 10);

  /* Publish current mode so controller_node can sync tracking_enabled_ */
  pub_mode_ = create_publisher<std_msgs::msg::UInt8>("/antenna/mode", 10);

  /* Publish manual targets for controller_node */
  pub_target_az_ = create_publisher<std_msgs::msg::Float64>("/antenna/target_azimuth", 10);
  pub_target_el_ = create_publisher<std_msgs::msg::Float64>("/antenna/target_elevation", 10);

  srv_set_mode_ = create_service<antenna_tracker_msgs::srv::SetMode>(
    "/antenna/set_mode",
    std::bind(&StateMachineNode::set_mode_callback, this,
              std::placeholders::_1, std::placeholders::_2));

  srv_manual_target_ = create_service<antenna_tracker_msgs::srv::SetManualTarget>(
    "/antenna/set_manual_target",
    std::bind(&StateMachineNode::set_manual_target_callback, this,
              std::placeholders::_1, std::placeholders::_2));

  srv_get_status_ = create_service<antenna_tracker_msgs::srv::GetStatus>(
    "/antenna/get_status",
    std::bind(&StateMachineNode::get_status_callback, this,
              std::placeholders::_1, std::placeholders::_2));

  srv_set_zero_ = create_service<antenna_tracker_msgs::srv::SetZeroOffset>(
    "/antenna/set_zero",
    std::bind(&StateMachineNode::set_zero_offset_callback, this,
              std::placeholders::_1, std::placeholders::_2));

  double diag_rate = get_parameter("diagnostics_rate_hz").as_double();
  auto period = std::chrono::duration<double>(1.0 / diag_rate);
  diagnostics_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&StateMachineNode::diagnostics_timer_callback, this));

  last_imu_time_ = now();
  last_encoder_time_ = now();
  last_target_time_ = now();
  last_heartbeat_time_ = now();
  loop_start_time_ = now();

  RCLCPP_INFO(get_logger(), "StateMachineNode initialized (mode: STANDBY)");
}

void StateMachineNode::state_callback(
  const antenna_tracker_msgs::msg::AntennaState::SharedPtr msg)
{
  current_azimuth_ = msg->current_azimuth;
  current_elevation_ = msg->current_elevation;
  loop_count_++;
}

void StateMachineNode::fusion_callback(
  const antenna_tracker_msgs::msg::ImuFusion::SharedPtr)
{
  last_imu_time_ = now();
}

void StateMachineNode::encoder_callback(
  const antenna_tracker_msgs::msg::EncoderFeedback::SharedPtr)
{
  last_encoder_time_ = now();
}

void StateMachineNode::target_callback(
  const antenna_tracker_msgs::msg::TargetGPS::SharedPtr)
{
  last_target_time_ = now();
}

void StateMachineNode::heartbeat_callback(
  const antenna_tracker_msgs::msg::Heartbeat::SharedPtr msg)
{
  last_heartbeat_time_ = now();
  fw_status_ = msg->status;
}

void StateMachineNode::diagnostics_timer_callback()
{
  auto diag = antenna_tracker_msgs::msg::TrackerDiagnostics();
  diag.header.stamp = now();

  double imu_timeout = get_parameter("imu_timeout_sec").as_double();
  double lora_timeout = get_parameter("lora_timeout_sec").as_double();

  double imu_age = (now() - last_imu_time_).seconds();
  double encoder_age = (now() - last_encoder_time_).seconds();
  double target_age = (now() - last_target_time_).seconds();
  double hb_age = (now() - last_heartbeat_time_).seconds();

  diag.imu_ok = (imu_age < imu_timeout);
  diag.mag_ok = diag.imu_ok;  /* BNO055 mag comes with IMU data */
  diag.encoder_ok = (encoder_age < imu_timeout);
  diag.can_ok = (target_age < lora_timeout);
  /* ── FIX: gps_ok reflects actual ground GPS reception state ── */
  diag.gps_ok = ground_gps_valid_ && (hb_age < 2.0); /* Assume GS GPS comes via heartbeat/micro-ROS */

  /* CPU temperature (RPi4B thermal zone) */
  std::ifstream temp_file("/sys/class/thermal/thermal_zone0/temp");
  if (temp_file.is_open()) {
    int temp_millic;
    temp_file >> temp_millic;
    diag.cpu_temp_c = temp_millic / 1000.0f;
  }

  /* Loop rate calculation */
  double elapsed = (now() - loop_start_time_).seconds();
  if (elapsed > 0.0) {
    diag.loop_rate_hz = static_cast<float>(loop_count_ / elapsed);
  }

  pub_diagnostics_->publish(diag);

  /* Auto-enter emergency on IMU timeout (hysteresis: 3 consecutive failures) */
  if (!diag.imu_ok) {
    imu_fail_count_++;
    if (imu_fail_count_ >= 3 && current_mode_ != OperationMode::EMERGENCY) {
      current_mode_ = OperationMode::EMERGENCY;
      imu_fail_count_ = 0;
      std_msgs::msg::UInt8 mode_msg;
      mode_msg.data = static_cast<uint8_t>(OperationMode::EMERGENCY);
      pub_mode_->publish(mode_msg);
      RCLCPP_ERROR(get_logger(), "IMU timeout: auto-entering EMERGENCY mode");
    }
  } else {
    imu_fail_count_ = 0;
  }

  /* ── FIX: Publish current mode so controller_node stays in sync ── */
  auto mode_msg = std_msgs::msg::UInt8();
  mode_msg.data = static_cast<uint8_t>(current_mode_);
  pub_mode_->publish(mode_msg);
}

void StateMachineNode::set_mode_callback(
  const std::shared_ptr<antenna_tracker_msgs::srv::SetMode::Request> request,
  std::shared_ptr<antenna_tracker_msgs::srv::SetMode::Response> response)
{
  uint8_t prev = static_cast<uint8_t>(current_mode_);

  if (request->mode > 3) {
    response->success = false;
    response->message = "Invalid mode";
    response->previous_mode = prev;
    return;
  }

  OperationMode new_mode = static_cast<OperationMode>(request->mode);

  /* Cannot exit EMERGENCY except via STANDBY */
  if (current_mode_ == OperationMode::EMERGENCY &&
      new_mode != OperationMode::STANDBY)
  {
    response->success = false;
    response->message = "Must go to STANDBY before exiting EMERGENCY";
    response->previous_mode = prev;
    return;
  }

  RCLCPP_INFO(get_logger(), "Mode change: %s -> %s",
              mode_name(current_mode_), mode_name(new_mode));

  current_mode_ = new_mode;
  response->success = true;
  response->message = std::string("Mode set to ") + mode_name(new_mode);
  response->previous_mode = prev;

  /* ── FIX: Immediately publish new mode for controller_node ── */
  auto mode_msg = std_msgs::msg::UInt8();
  mode_msg.data = static_cast<uint8_t>(current_mode_);
  pub_mode_->publish(mode_msg);
  RCLCPP_INFO(get_logger(), "Published /antenna/mode: %d", mode_msg.data);
}

void StateMachineNode::set_manual_target_callback(
  const std::shared_ptr<antenna_tracker_msgs::srv::SetManualTarget::Request> request,
  std::shared_ptr<antenna_tracker_msgs::srv::SetManualTarget::Response> response)
{
  if (current_mode_ != OperationMode::MANUAL) {
    response->success = false;
    response->message = "Must be in MANUAL mode";
    return;
  }

  if (request->azimuth_deg < 0.0 || request->azimuth_deg > 360.0 ||
      request->elevation_deg < 0.0 || request->elevation_deg > 90.0)
  {
    response->success = false;
    response->message = "Target out of range";
    return;
  }

  /* ── FIX: Actually publish the manual targets ── */
  auto az_msg = std_msgs::msg::Float64();
  az_msg.data = request->azimuth_deg;
  pub_target_az_->publish(az_msg);

  auto el_msg = std_msgs::msg::Float64();
  el_msg.data = request->elevation_deg;
  pub_target_el_->publish(el_msg);

  response->success = true;
  response->message = "Manual target set";
}

void StateMachineNode::set_zero_offset_callback(
  const std::shared_ptr<antenna_tracker_msgs::srv::SetZeroOffset::Request>,
  std::shared_ptr<antenna_tracker_msgs::srv::SetZeroOffset::Response> response)
{
  if (current_mode_ != OperationMode::STANDBY) {
    response->success = false;
    response->message = "Must be in STANDBY mode to calibrate";
    return;
  }

  /* Record current positions as offsets */
  az_offset_ = current_azimuth_;
  el_offset_ = current_elevation_;

  RCLCPP_INFO(get_logger(), "Calibration: New Zero Offset Az=%.2f, El=%.2f",
              az_offset_, el_offset_);
  
  response->success = true;
  response->message = "Zero offset recorded successfully";
}

void StateMachineNode::get_status_callback(
  const std::shared_ptr<antenna_tracker_msgs::srv::GetStatus::Request>,
  std::shared_ptr<antenna_tracker_msgs::srv::GetStatus::Response> response)
{
  response->current_mode = static_cast<uint8_t>(current_mode_);
  response->current_azimuth = current_azimuth_;
  response->current_elevation = current_elevation_;
  response->tracking_active = (current_mode_ == OperationMode::AUTO);
}

const char * StateMachineNode::mode_name(OperationMode mode)
{
  switch (mode) {
    case OperationMode::AUTO: return "AUTO";
    case OperationMode::MANUAL: return "MANUAL";
    case OperationMode::STANDBY: return "STANDBY";
    case OperationMode::EMERGENCY: return "EMERGENCY";
    default: return "UNKNOWN";
  }
}

}  // namespace antenna_tracker_controller

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<antenna_tracker_controller::StateMachineNode>());
  rclcpp::shutdown();
  return 0;
}
