#include "antenna_tracker_controller/controller_node.hpp"
#include <cmath>
#include <mutex>

namespace antenna_tracker_controller
{

ControllerNode::ControllerNode(const rclcpp::NodeOptions & options)
: Node("controller_node", options)
{
  declare_parameter<double>("loop_rate_hz", 100.0);
  declare_parameter<double>("mpc_to_hz_scale", 1.0);
  declare_parameter<double>("az_pos_kp", 15.0);
  declare_parameter<double>("az_pos_ki", 0.1);
  declare_parameter<double>("az_pos_kd", 0.3);
  declare_parameter<double>("az_vel_kp", 4.0);
  declare_parameter<double>("az_vel_ki", 0.05);
  declare_parameter<double>("az_vel_kd", 0.1);
  declare_parameter<double>("el_pos_kp", 8.0);
  declare_parameter<double>("el_pos_ki", 1.0);
  declare_parameter<double>("el_pos_kd", 0.2);
  declare_parameter<double>("el_vel_kp", 5.0);
  declare_parameter<double>("el_vel_ki", 0.15);
  declare_parameter<double>("el_vel_kd", 0.08);
  declare_parameter<double>("azimuth_min_deg", 0.0);
  declare_parameter<double>("azimuth_max_deg", 360.0);
  declare_parameter<double>("elevation_min_deg", 0.0);
  declare_parameter<double>("elevation_max_deg", 90.0);

  double loop_rate = get_parameter("loop_rate_hz").as_double();
  double dt = 1.0 / loop_rate;

  /* Cache safety limits — avoid repeated get_parameter() in 100 Hz callback */
  az_min_ = get_parameter("azimuth_min_deg").as_double();
  az_max_ = get_parameter("azimuth_max_deg").as_double();
  el_min_ = get_parameter("elevation_min_deg").as_double();
  el_max_ = get_parameter("elevation_max_deg").as_double();

  /* Update cache when parameters change at runtime */
  param_cb_handle_ = add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & params) {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;
      for (const auto & p : params) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (p.get_name() == "azimuth_min_deg")  { az_min_ = p.as_double(); }
        else if (p.get_name() == "azimuth_max_deg")  { az_max_ = p.as_double(); }
        else if (p.get_name() == "elevation_min_deg") { el_min_ = p.as_double(); }
        else if (p.get_name() == "elevation_max_deg") { el_max_ = p.as_double(); }
      }
      return result;
    });

  mpc_.init();
  mpc_.set_mpc_to_hz_scale(get_parameter("mpc_to_hz_scale").as_double());

  sub_fusion_ = create_subscription<antenna_tracker_msgs::msg::ImuFusion>(
    "/antenna/imu_fusion", rclcpp::SensorDataQoS(),
    std::bind(&ControllerNode::imu_fusion_callback, this, std::placeholders::_1));

  sub_encoder_ = create_subscription<antenna_tracker_msgs::msg::EncoderFeedback>(
    "/antenna/encoder_feedback", rclcpp::SensorDataQoS(),
    std::bind(&ControllerNode::encoder_callback, this, std::placeholders::_1));

  sub_target_az_ = create_subscription<std_msgs::msg::Float64>(
    "/antenna/target_azimuth", 10,
    std::bind(&ControllerNode::target_az_callback, this, std::placeholders::_1));

  sub_target_el_ = create_subscription<std_msgs::msg::Float64>(
    "/antenna/target_elevation", 10,
    std::bind(&ControllerNode::target_el_callback, this, std::placeholders::_1));

  /* Subscribe to mode from state_machine_node */
  sub_mode_ = create_subscription<std_msgs::msg::UInt8>(
    "/antenna/mode", 10,
    std::bind(&ControllerNode::mode_callback, this, std::placeholders::_1));

  pub_motor_ = create_publisher<antenna_tracker_msgs::msg::MotorCommand>(
    "/antenna/motor_cmd", rclcpp::SensorDataQoS());

  pub_state_ = create_publisher<antenna_tracker_msgs::msg::AntennaState>(
    "/antenna/state", 10);

  auto period = std::chrono::duration<double>(dt);
  control_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&ControllerNode::control_timer_callback, this));

  action_server_ = rclcpp_action::create_server<TrackTarget>(
    this, "/antenna/track_target",
    std::bind(&ControllerNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&ControllerNode::handle_cancel, this, std::placeholders::_1),
    std::bind(&ControllerNode::handle_accepted, this, std::placeholders::_1));

  RCLCPP_INFO(get_logger(), "ControllerNode initialized (%.0f Hz)", loop_rate);
}

void ControllerNode::imu_fusion_callback(
  const antenna_tracker_msgs::msg::ImuFusion::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  current_azimuth_ = msg->azimuth;
  current_elevation_ = msg->elevation;
  az_velocity_ = msg->az_velocity;
  el_velocity_ = msg->el_velocity;
  fusion_valid_ = msg->kalman_valid;
}

void ControllerNode::encoder_callback(
  const antenna_tracker_msgs::msg::EncoderFeedback::SharedPtr msg)
{
  /* Encoder can override IMU for position if valid */
  if (msg->az_valid && msg->el_valid) {
    /* Blend encoder position with IMU (encoder is more reliable for position) */
    std::lock_guard<std::mutex> lock(state_mutex_);
    current_azimuth_ = msg->az_angle_deg;
    current_elevation_ = msg->el_angle_deg;
    az_velocity_ = msg->az_velocity_dps;
    el_velocity_ = msg->el_velocity_dps;
    fusion_valid_ = true;
  }
}

void ControllerNode::target_az_callback(const std_msgs::msg::Float64::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  target_azimuth_ = msg->data;
}

void ControllerNode::target_el_callback(const std_msgs::msg::Float64::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  target_elevation_ = msg->data;
}

void ControllerNode::mode_callback(const std_msgs::msg::UInt8::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  current_mode_ = msg->data;
  /* Enable tracking in AUTO (0) and MANUAL (1) modes */
  tracking_enabled_ = (current_mode_ == 0 || current_mode_ == 1);
}

void ControllerNode::control_timer_callback()
{
  /* Copy ALL shared state under lock — includes safety limits and active_goal_ */
  double current_azimuth, current_elevation, az_velocity, el_velocity;
  double target_azimuth, target_elevation;
  bool fusion_valid, tracking_enabled;
  uint8_t current_mode;
  double az_min, az_max, el_min, el_max;
  std::shared_ptr<GoalHandleTrackTarget> active_goal;
  rclcpp::Time goal_start;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    current_azimuth  = current_azimuth_;
    current_elevation = current_elevation_;
    az_velocity      = az_velocity_;
    el_velocity      = el_velocity_;
    target_azimuth   = target_azimuth_;
    target_elevation = target_elevation_;
    fusion_valid     = fusion_valid_;
    tracking_enabled = tracking_enabled_;
    current_mode     = current_mode_;
    az_min = az_min_; az_max = az_max_;
    el_min = el_min_; el_max = el_max_;
    active_goal = active_goal_;
    goal_start  = goal_start_time_;
  }

  /* Always publish antenna state for the UI, regardless of tracking status */
  auto state_msg = antenna_tracker_msgs::msg::AntennaState();
  state_msg.header.stamp = now();
  state_msg.current_azimuth = current_azimuth;
  state_msg.current_elevation = current_elevation;
  state_msg.target_azimuth = target_azimuth;
  state_msg.target_elevation = target_elevation;
  state_msg.mode = current_mode;
  state_msg.tracking_active = tracking_enabled;

  if (!tracking_enabled || !fusion_valid) {
    /* Publish zero motor command */
    auto cmd = antenna_tracker_msgs::msg::MotorCommand();
    cmd.header.stamp = now();
    cmd.emergency_stop = !tracking_enabled; // true if standby/emergency
    pub_motor_->publish(cmd);

    /* Finish state msg and publish */
    state_msg.az_error = 0.0;
    state_msg.el_error = 0.0;
    state_msg.az_motor_cmd = 0.0;
    state_msg.el_motor_cmd = 0.0;
    pub_state_->publish(state_msg);

    if (active_goal && active_goal->is_canceling()) {
      auto result = std::make_shared<TrackTarget::Result>();
      result->success = false;
      result->final_azimuth = current_azimuth;
      result->final_elevation = current_elevation;
      result->tracking_duration_sec = (now() - goal_start).seconds();
      active_goal->canceled(result);
      std::lock_guard<std::mutex> lock(state_mutex_);
      active_goal_.reset();
    }
    return;
  }

  /* ── FIX: Shortest-path wrap-around for azimuth (359°→1° = +2°, not -358°) ── */
  double az_error_raw = target_azimuth - current_azimuth;
  while (az_error_raw >  180.0) az_error_raw -= 360.0;
  while (az_error_raw < -180.0) az_error_raw += 360.0;
  double effective_az_target = current_azimuth + az_error_raw;

  double az_cmd = 0.0, el_cmd = 0.0;
  mpc_.compute(
    effective_az_target, current_azimuth, az_velocity,
    target_elevation, current_elevation, el_velocity,
    az_cmd, el_cmd);

  /* Safety limits — use cached values (updated by parameter callback) */

  if (current_azimuth < az_min) {
    az_cmd = std::max(0.0, az_cmd);   // only allow positive (CW) to return
  } else if (current_azimuth > az_max) {
    az_cmd = std::min(0.0, az_cmd);   // only allow negative (CCW) to return
  }
  if (current_elevation < el_min) {
    el_cmd = std::max(0.0, el_cmd);   // only allow positive to return
  } else if (current_elevation > el_max) {
    el_cmd = std::min(0.0, el_cmd);   // only allow negative to return
  }

  /* AZ deadband: suppress tiny commands near target (no gravity on AZ axis).
   * EL deadband removed: double-integrator + gravity needs continuous control;
   * NMPC naturally provides gravity equilibrium (u_el ≈ 10.77*cos(el)) at steady state. */
  static constexpr double AZ_DEADBAND_DEG = 0.5;
  if (std::abs(az_error_raw) < AZ_DEADBAND_DEG) az_cmd = 0.0;

  /* Publish motor command */
  auto motor_msg = antenna_tracker_msgs::msg::MotorCommand();
  motor_msg.header.stamp = now();
  motor_msg.az_frequency_hz = std::abs(az_cmd);
  motor_msg.el_frequency_hz = std::abs(el_cmd);
  motor_msg.az_direction = (az_cmd >= 0.0);
  motor_msg.el_direction = (el_cmd >= 0.0);
  motor_msg.emergency_stop = false;
  pub_motor_->publish(motor_msg);

  /* Update active tracking state fields */
  state_msg.az_error = az_error_raw;
  state_msg.el_error = target_elevation - current_elevation;
  state_msg.az_motor_cmd = az_cmd;
  state_msg.el_motor_cmd = el_cmd;
  pub_state_->publish(state_msg);

  /* Action feedback — uses local copy of active_goal (thread-safe) */
  if (active_goal) {
    if (active_goal->is_canceling()) {
      auto result = std::make_shared<TrackTarget::Result>();
      result->success = false;
      result->final_azimuth = current_azimuth;
      result->final_elevation = current_elevation;
      result->tracking_duration_sec = (now() - goal_start).seconds();
      active_goal->canceled(result);
      std::lock_guard<std::mutex> lock(state_mutex_);
      active_goal_.reset();
      return;
    }

    auto feedback = std::make_shared<TrackTarget::Feedback>();
    feedback->current_azimuth = current_azimuth;
    feedback->current_elevation = current_elevation;
    feedback->az_error = az_error_raw;  /* 최단경로 랩핑 오차 사용 (line 143-145) */
    feedback->el_error = target_elevation - current_elevation;
    active_goal->publish_feedback(feedback);

    /* Check completion */
    auto goal = active_goal->get_goal();
    double error = std::sqrt(
      feedback->az_error * feedback->az_error +
      feedback->el_error * feedback->el_error);

    double elapsed = (now() - goal_start).seconds();

    if (error <= goal->tolerance_deg) {
      auto result = std::make_shared<TrackTarget::Result>();
      result->success = true;
      result->final_azimuth = current_azimuth;
      result->final_elevation = current_elevation;
      result->tracking_duration_sec = elapsed;
      active_goal->succeed(result);
      std::lock_guard<std::mutex> lock(state_mutex_);
      active_goal_.reset();
    } else if (elapsed > goal->timeout_sec) {
      auto result = std::make_shared<TrackTarget::Result>();
      result->success = false;
      result->final_azimuth = current_azimuth;
      result->final_elevation = current_elevation;
      result->tracking_duration_sec = elapsed;
      active_goal->abort(result);
      std::lock_guard<std::mutex> lock(state_mutex_);
      active_goal_.reset();
    }
  }
}

rclcpp_action::GoalResponse ControllerNode::handle_goal(
  const rclcpp_action::GoalUUID &,
  std::shared_ptr<const TrackTarget::Goal> goal)
{
  RCLCPP_INFO(get_logger(), "TrackTarget goal: az=%.1f, el=%.1f, tol=%.1f",
              goal->target_azimuth_deg, goal->target_elevation_deg, goal->tolerance_deg);
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse ControllerNode::handle_cancel(
  const std::shared_ptr<GoalHandleTrackTarget>)
{
  RCLCPP_INFO(get_logger(), "TrackTarget goal cancelled");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void ControllerNode::handle_accepted(
  const std::shared_ptr<GoalHandleTrackTarget> goal_handle)
{
  auto goal = goal_handle->get_goal();
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (active_goal_) {
    auto prev_result = std::make_shared<TrackTarget::Result>();
    prev_result->success = false;
    active_goal_->abort(prev_result);
  }
  active_goal_ = goal_handle;
  goal_start_time_ = now();
  target_azimuth_ = goal->target_azimuth_deg;
  target_elevation_ = goal->target_elevation_deg;
}

}  // namespace antenna_tracker_controller

#ifndef ANTENNA_TRACKER_TEST_ENV
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<antenna_tracker_controller::ControllerNode>());
  rclcpp::shutdown();
  return 0;
}
#endif
