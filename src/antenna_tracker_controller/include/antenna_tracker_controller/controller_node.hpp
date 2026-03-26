#ifndef ANTENNA_TRACKER_CONTROLLER__CONTROLLER_NODE_HPP_
#define ANTENNA_TRACKER_CONTROLLER__CONTROLLER_NODE_HPP_

#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <antenna_tracker_msgs/msg/imu_fusion.hpp>
#include <antenna_tracker_msgs/msg/encoder_feedback.hpp>
#include <antenna_tracker_msgs/msg/motor_command.hpp>
#include <antenna_tracker_msgs/msg/antenna_state.hpp>
#include <antenna_tracker_msgs/action/track_target.hpp>
#include "antenna_tracker_controller/mpc_controller.hpp"

namespace antenna_tracker_controller
{

using TrackTarget = antenna_tracker_msgs::action::TrackTarget;
using GoalHandleTrackTarget = rclcpp_action::ServerGoalHandle<TrackTarget>;

class ControllerNode : public rclcpp::Node
{
public:
  explicit ControllerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void imu_fusion_callback(const antenna_tracker_msgs::msg::ImuFusion::SharedPtr msg);
  void encoder_callback(const antenna_tracker_msgs::msg::EncoderFeedback::SharedPtr msg);
  void target_az_callback(const std_msgs::msg::Float64::SharedPtr msg);
  void target_el_callback(const std_msgs::msg::Float64::SharedPtr msg);
  void mode_callback(const std_msgs::msg::UInt8::SharedPtr msg);
  void control_timer_callback();

  /* Action server callbacks */
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const TrackTarget::Goal> goal);
  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleTrackTarget> goal_handle);
  void handle_accepted(const std::shared_ptr<GoalHandleTrackTarget> goal_handle);

  rclcpp::Subscription<antenna_tracker_msgs::msg::ImuFusion>::SharedPtr sub_fusion_;
  rclcpp::Subscription<antenna_tracker_msgs::msg::EncoderFeedback>::SharedPtr sub_encoder_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_target_az_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_target_el_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr sub_mode_;
  rclcpp::Publisher<antenna_tracker_msgs::msg::MotorCommand>::SharedPtr pub_motor_;
  rclcpp::Publisher<antenna_tracker_msgs::msg::AntennaState>::SharedPtr pub_state_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp_action::Server<TrackTarget>::SharedPtr action_server_;

  MpcController mpc_;

  double current_azimuth_{0.0};
  double current_elevation_{0.0};
  double az_velocity_{0.0};
  double el_velocity_{0.0};
  double target_azimuth_{0.0};
  double target_elevation_{0.0};
  bool fusion_valid_{false};
  bool tracking_enabled_{false};  /* false by default — must receive AUTO mode */
  uint8_t current_mode_{2};       /* 2 = STANDBY */

  /* Cached safety limits — set at construction, updated via parameter callback */
  double az_min_{0.0};
  double az_max_{360.0};
  double el_min_{0.0};
  double el_max_{90.0};

  /* Thread safety — protects all shared state including active_goal_ */
  std::mutex state_mutex_;

  /* Active goal */
  std::shared_ptr<GoalHandleTrackTarget> active_goal_;
  rclcpp::Time goal_start_time_;

  /* Parameter event callback handle */
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
};

}  // namespace antenna_tracker_controller

#endif
