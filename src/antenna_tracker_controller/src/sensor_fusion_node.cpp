#include "antenna_tracker_controller/sensor_fusion_node.hpp"

namespace antenna_tracker_controller
{

SensorFusionNode::SensorFusionNode(const rclcpp::NodeOptions & options)
: Node("sensor_fusion_node", options)
{
  declare_parameter<double>("complementary_alpha", 0.98);
  declare_parameter<double>("mag_declination_deg", -8.0);
  declare_parameter<double>("kalman_q_process", 0.001);
  declare_parameter<double>("kalman_r_measurement", 2.0);
  declare_parameter<double>("kalman_q_vel_multiplier", 10.0);
  declare_parameter<double>("loop_rate_hz", 100.0);

  double alpha = get_parameter("complementary_alpha").as_double();
  double declination = get_parameter("mag_declination_deg").as_double();
  double kf_q = get_parameter("kalman_q_process").as_double();
  double kf_r = get_parameter("kalman_r_measurement").as_double();
  double kf_q_vel_mult = get_parameter("kalman_q_vel_multiplier").as_double();
  double loop_rate = get_parameter("loop_rate_hz").as_double();

  comp_filter_.set_alpha(alpha);
  comp_filter_.set_declination(declination);

  double dt = 1.0 / loop_rate;
  kalman_filter_.init(dt, kf_q, kf_r, kf_q_vel_mult);

  sub_imu_ = create_subscription<sensor_msgs::msg::Imu>(
    "/imu/raw", rclcpp::SensorDataQoS(),
    std::bind(&SensorFusionNode::imu_callback, this, std::placeholders::_1));

  sub_mag_ = create_subscription<sensor_msgs::msg::MagneticField>(
    "/magnetic_field", rclcpp::SensorDataQoS(),
    std::bind(&SensorFusionNode::mag_callback, this, std::placeholders::_1));

  sub_gps_ = create_subscription<sensor_msgs::msg::NavSatFix>(
    "/gps/fix", 10,
    std::bind(&SensorFusionNode::gps_callback, this, std::placeholders::_1));

  // Subscribe to encoder feedback — used as primary azimuth/elevation in simulation
  sub_encoder_ = create_subscription<antenna_tracker_msgs::msg::EncoderFeedback>(
    "/antenna/encoder_feedback", rclcpp::SensorDataQoS(),
    std::bind(&SensorFusionNode::encoder_callback, this, std::placeholders::_1));

  pub_fusion_ = create_publisher<antenna_tracker_msgs::msg::ImuFusion>(
    "/antenna/imu_fusion", rclcpp::SensorDataQoS());

  auto period = std::chrono::duration<double>(dt);
  timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&SensorFusionNode::timer_callback, this));

  RCLCPP_INFO(get_logger(), "SensorFusionNode initialized (%.0f Hz, alpha=%.2f, decl=%.1f)",
              loop_rate, alpha, declination);
}

void SensorFusionNode::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  latest_imu_ = msg;
  imu_received_ = true;
}

void SensorFusionNode::mag_callback(const sensor_msgs::msg::MagneticField::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  latest_mag_ = msg;
  mag_received_ = true;
}

void SensorFusionNode::encoder_callback(const antenna_tracker_msgs::msg::EncoderFeedback::SharedPtr msg)
{
  if (msg->az_valid) {
    encoder_az_deg_ = msg->az_angle_deg;
  }
  if (msg->el_valid) {
    encoder_el_deg_ = msg->el_angle_deg;
  }
  encoder_received_ = msg->az_valid || msg->el_valid;
}

void SensorFusionNode::gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
  if (msg->status.status < 0) return; /* No fix */

  /* 
   * Simple Magnetic Declination Approximation (Example: South Korea is ~-8.5deg)
   * Real implementation should use WMM (World Magnetic Model)
   */
  double lat = msg->latitude;
  double lon = msg->longitude;
  
  /* Fallback: user-defined or simple lookup */
  double decl = get_parameter("mag_declination_deg").as_double();
  
  /* Log update if significant change? */
  comp_filter_.set_declination(decl);
  
  RCLCPP_DEBUG(get_logger(), "GPS Fix: Lat=%.4f, Lon=%.4f -> Declination adjusted", lat, lon);
}


void SensorFusionNode::timer_callback()
{
  /* Copy shared pointers under lock to avoid data race with callbacks */
  sensor_msgs::msg::Imu::SharedPtr local_imu;
  sensor_msgs::msg::MagneticField::SharedPtr local_mag;
  bool mag_ok = false;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (!imu_received_) {
      return;  /* Wait for first IMU sample */
    }
    local_imu = latest_imu_;
    local_mag = latest_mag_;
    mag_ok = mag_received_;
  }

  /* BMI270 outputs raw accel + gyro only (no orientation quaternion).
   * Always run complementary filter to compute roll/pitch/yaw.
   * If MLX90393 mag is not yet available, heading is gyro-integrated only. */
  double timestamp = local_imu->header.stamp.sec +
                     local_imu->header.stamp.nanosec * 1e-9;

  double mag_x = 0.0, mag_y = 0.0, mag_z = 0.0;
  if (mag_ok && local_mag) {
    mag_x = local_mag->magnetic_field.x * 1e6;  /* T → µT */
    mag_y = local_mag->magnetic_field.y * 1e6;
    mag_z = local_mag->magnetic_field.z * 1e6;
  }

  static constexpr double kRadToDeg = 180.0 / M_PI;
  comp_filter_.update(
    local_imu->linear_acceleration.x,
    local_imu->linear_acceleration.y,
    local_imu->linear_acceleration.z,
    local_imu->angular_velocity.x * kRadToDeg,
    local_imu->angular_velocity.y * kRadToDeg,
    local_imu->angular_velocity.z * kRadToDeg,
    mag_x, mag_y, mag_z,
    timestamp);

  const auto & orient = comp_filter_.orientation();
  double roll_deg      = orient.roll;
  double pitch_deg     = orient.pitch;
  double yaw_deg       = orient.yaw;
  double azimuth_deg   = orient.azimuth;
  double elevation_deg = orient.elevation;

  /* Encoder overrides azimuth/elevation when valid */
  if (encoder_received_) {
    azimuth_deg   = encoder_az_deg_;
    elevation_deg = encoder_el_deg_;
  }

  kalman_filter_.update(azimuth_deg, elevation_deg);
  kalman_initialized_ = true;

  auto msg = antenna_tracker_msgs::msg::ImuFusion();
  msg.header.stamp    = now();
  msg.header.frame_id = "imu_link";
  msg.roll      = roll_deg;
  msg.pitch     = pitch_deg;
  msg.yaw       = yaw_deg;
  msg.azimuth   = kalman_filter_.azimuth();
  msg.elevation = kalman_filter_.elevation();
  msg.az_velocity   = kalman_filter_.az_velocity();
  msg.el_velocity   = kalman_filter_.el_velocity();
  msg.kalman_valid  = kalman_initialized_;

  pub_fusion_->publish(msg);
}

}  // namespace antenna_tracker_controller


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<antenna_tracker_controller::SensorFusionNode>());
  rclcpp::shutdown();
  return 0;
}
