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
  declare_parameter<double>("loop_rate_hz", 100.0);

  double alpha = get_parameter("complementary_alpha").as_double();
  double declination = get_parameter("mag_declination_deg").as_double();
  double kf_q = get_parameter("kalman_q_process").as_double();
  double kf_r = get_parameter("kalman_r_measurement").as_double();
  double loop_rate = get_parameter("loop_rate_hz").as_double();

  comp_filter_.set_alpha(alpha);
  comp_filter_.set_declination(declination);

  double dt = 1.0 / loop_rate;
  kalman_filter_.init(dt, kf_q, kf_r);

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
  latest_imu_ = msg;
  imu_received_ = true;
}

void SensorFusionNode::mag_callback(const sensor_msgs::msg::MagneticField::SharedPtr msg)
{
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
  if (!imu_received_) {
    return;  /* Wait for first IMU sample */
  }

  /* ── FIX: BNO055 emits absolute orientation in /imu/raw quaternion.
   *  Use it directly so fusion works without a separate /magnetic_field topic. ── */
  const auto & q = latest_imu_->orientation;

  /* Quaternion → roll/pitch/yaw(azimuth) in degrees */
  double sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z);
  double cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
  double roll_deg  = std::atan2(sinr_cosp, cosr_cosp) * 57.29577951;

  double sinp      = 2.0 * (q.w * q.y - q.z * q.x);
  double pitch_deg = std::asin(std::clamp(sinp, -1.0, 1.0)) * 57.29577951;

  double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  double yaw_deg   = std::atan2(siny_cosp, cosy_cosp) * 57.29577951;
  if (yaw_deg < 0.0) yaw_deg += 360.0;

  double azimuth_deg   = yaw_deg;
  double elevation_deg = -pitch_deg;  /* flip: nose-up = positive elevation */

  /* Encoder feedback overrides IMU yaw when valid (critical for simulation where
   * Gazebo IMU reports identity quaternion → yaw=0° in headless Docker) */
  if (encoder_received_) {
    azimuth_deg   = encoder_az_deg_;
    elevation_deg = encoder_el_deg_;
  }

  /* Optional: blend with compass via complementary filter if mag is available */
  if (mag_received_ && !encoder_received_) {
    double timestamp = latest_imu_->header.stamp.sec +
                       latest_imu_->header.stamp.nanosec * 1e-9;
    double gyro_x = latest_imu_->angular_velocity.x * 57.29577951;
    double gyro_y = latest_imu_->angular_velocity.y * 57.29577951;
    double gyro_z = latest_imu_->angular_velocity.z * 57.29577951;
    double mag_x  = latest_mag_->magnetic_field.x * 1e6;
    double mag_y  = latest_mag_->magnetic_field.y * 1e6;
    double mag_z  = latest_mag_->magnetic_field.z * 1e6;

    comp_filter_.update(
      latest_imu_->linear_acceleration.x,
      latest_imu_->linear_acceleration.y,
      latest_imu_->linear_acceleration.z,
      gyro_x, gyro_y, gyro_z,
      mag_x, mag_y, mag_z,
      timestamp);

    const auto & orient = comp_filter_.orientation();
    azimuth_deg   = orient.azimuth;   /* compass-corrected */
    elevation_deg = orient.elevation;
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
