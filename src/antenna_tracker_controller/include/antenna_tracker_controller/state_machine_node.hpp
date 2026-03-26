#ifndef ANTENNA_TRACKER_CONTROLLER__STATE_MACHINE_NODE_HPP_
#define ANTENNA_TRACKER_CONTROLLER__STATE_MACHINE_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <antenna_tracker_msgs/msg/antenna_state.hpp>
#include <antenna_tracker_msgs/msg/imu_fusion.hpp>
#include <antenna_tracker_msgs/msg/encoder_feedback.hpp>
#include <antenna_tracker_msgs/msg/target_gps.hpp>
#include <antenna_tracker_msgs/msg/tracker_diagnostics.hpp>
#include <antenna_tracker_msgs/srv/set_mode.hpp>
#include <antenna_tracker_msgs/srv/set_manual_target.hpp>
#include <antenna_tracker_msgs/srv/get_status.hpp>
#include <antenna_tracker_msgs/srv/set_zero_offset.hpp>
#include <antenna_tracker_msgs/msg/heartbeat.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/float64.hpp>

namespace antenna_tracker_controller
{

enum class OperationMode : uint8_t {
  AUTO = 0,
  MANUAL = 1,
  STANDBY = 2,
  EMERGENCY = 3
};

class StateMachineNode : public rclcpp::Node
{
public:
  explicit StateMachineNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void state_callback(const antenna_tracker_msgs::msg::AntennaState::SharedPtr msg);
  void fusion_callback(const antenna_tracker_msgs::msg::ImuFusion::SharedPtr msg);
  void encoder_callback(const antenna_tracker_msgs::msg::EncoderFeedback::SharedPtr msg);
  void target_callback(const antenna_tracker_msgs::msg::TargetGPS::SharedPtr msg);
  void heartbeat_callback(const antenna_tracker_msgs::msg::Heartbeat::SharedPtr msg);
  void diagnostics_timer_callback();

  void set_mode_callback(
    const std::shared_ptr<antenna_tracker_msgs::srv::SetMode::Request> request,
    std::shared_ptr<antenna_tracker_msgs::srv::SetMode::Response> response);
  void set_manual_target_callback(
    const std::shared_ptr<antenna_tracker_msgs::srv::SetManualTarget::Request> request,
    std::shared_ptr<antenna_tracker_msgs::srv::SetManualTarget::Response> response);
  void get_status_callback(
    const std::shared_ptr<antenna_tracker_msgs::srv::GetStatus::Request> request,
    std::shared_ptr<antenna_tracker_msgs::srv::GetStatus::Response> response);
  void set_zero_offset_callback(
    const std::shared_ptr<antenna_tracker_msgs::srv::SetZeroOffset::Request> request,
    std::shared_ptr<antenna_tracker_msgs::srv::SetZeroOffset::Response> response);

  static const char * mode_name(OperationMode mode);

  rclcpp::Subscription<antenna_tracker_msgs::msg::AntennaState>::SharedPtr sub_state_;
  rclcpp::Subscription<antenna_tracker_msgs::msg::ImuFusion>::SharedPtr sub_fusion_;
  rclcpp::Subscription<antenna_tracker_msgs::msg::EncoderFeedback>::SharedPtr sub_encoder_;
  rclcpp::Subscription<antenna_tracker_msgs::msg::TargetGPS>::SharedPtr sub_target_;
  rclcpp::Subscription<antenna_tracker_msgs::msg::Heartbeat>::SharedPtr sub_heartbeat_;

  rclcpp::Publisher<antenna_tracker_msgs::msg::TrackerDiagnostics>::SharedPtr pub_diagnostics_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pub_mode_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_target_az_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_target_el_;

  rclcpp::Service<antenna_tracker_msgs::srv::SetMode>::SharedPtr srv_set_mode_;
  rclcpp::Service<antenna_tracker_msgs::srv::SetManualTarget>::SharedPtr srv_manual_target_;
  rclcpp::Service<antenna_tracker_msgs::srv::GetStatus>::SharedPtr srv_get_status_;
  rclcpp::Service<antenna_tracker_msgs::srv::SetZeroOffset>::SharedPtr srv_set_zero_;

  rclcpp::TimerBase::SharedPtr diagnostics_timer_;

  OperationMode current_mode_{OperationMode::STANDBY};
  double current_azimuth_{0.0};
  double current_elevation_{0.0};
  
  /* Calibration offsets */
  double az_offset_{0.0};
  double el_offset_{0.0};

  /* Cached parameters — set at construction to avoid repeated get_parameter() */
  double imu_timeout_sec_{1.0};
  double lora_timeout_sec_{12.0};

  /* Health tracking */
  rclcpp::Time last_imu_time_;
  rclcpp::Time last_encoder_time_;
  rclcpp::Time last_target_time_;
  rclcpp::Time last_heartbeat_time_;
  uint64_t loop_count_{0};
  rclcpp::Time loop_start_time_;
  bool ground_gps_valid_{false};  /* true once /gps/fix is received */
  uint8_t fw_status_{0};
  int imu_fail_count_{0};
};

}  // namespace antenna_tracker_controller

#endif
