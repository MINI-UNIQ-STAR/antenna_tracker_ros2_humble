#include "antenna_tracker_controller/navigation_node.hpp"
#include <cmath>

namespace antenna_tracker_controller
{

static constexpr double EARTH_RADIUS_M = 6371000.0;
static constexpr double DEG_TO_RAD = M_PI / 180.0;
static constexpr double RAD_TO_DEG = 180.0 / M_PI;

NavigationNode::NavigationNode(const rclcpp::NodeOptions & options)
: Node("navigation_node", options)
{
  declare_parameter<double>("default_ground_lat", 37.5665);
  declare_parameter<double>("default_ground_lon", 126.9780);
  declare_parameter<double>("default_ground_alt", 30.0);

  ground_lat_ = get_parameter("default_ground_lat").as_double();
  ground_lon_ = get_parameter("default_ground_lon").as_double();
  ground_alt_ = get_parameter("default_ground_alt").as_double();

  sub_gps_ = create_subscription<sensor_msgs::msg::NavSatFix>(
    "/gps/fix", rclcpp::SensorDataQoS(),
    std::bind(&NavigationNode::gps_callback, this, std::placeholders::_1));

  sub_target_ = create_subscription<antenna_tracker_msgs::msg::TargetGPS>(
    "/antenna/target_gps", rclcpp::SensorDataQoS(),
    std::bind(&NavigationNode::target_callback, this, std::placeholders::_1));

  sub_mode_ = create_subscription<std_msgs::msg::UInt8>(
    "/antenna/mode", 10,
    std::bind(&NavigationNode::mode_callback, this, std::placeholders::_1));

  pub_target_az_ = create_publisher<std_msgs::msg::Float64>(
    "/antenna/target_azimuth", 10);
  pub_target_el_ = create_publisher<std_msgs::msg::Float64>(
    "/antenna/target_elevation", 10);

  RCLCPP_INFO(get_logger(), "NavigationNode initialized (default pos: %.4f, %.4f, %.0fm)",
              ground_lat_, ground_lon_, ground_alt_);
}

void NavigationNode::gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
  if (msg->status.status >= 0) {
    ground_lat_ = msg->latitude;
    ground_lon_ = msg->longitude;
    ground_alt_ = msg->altitude;
    ground_gps_valid_ = true;
  }
}

void NavigationNode::target_callback(const antenna_tracker_msgs::msg::TargetGPS::SharedPtr msg)
{
  target_lat_ = msg->latitude;
  target_lon_ = msg->longitude;
  target_alt_ = msg->altitude_m;
  target_valid_ = true;

  compute_and_publish();
}

void NavigationNode::mode_callback(const std_msgs::msg::UInt8::SharedPtr msg)
{
  current_mode_ = msg->data;
}

void NavigationNode::compute_and_publish()
{
  if (!target_valid_) {
    return;
  }
  /* Only publish nav targets in AUTO mode — MANUAL mode uses set_manual_target service */
  if (current_mode_ != 0) {
    return;
  }

  double bearing = haversine_bearing(ground_lat_, ground_lon_, target_lat_, target_lon_);
  double elev = elevation_angle(
    ground_lat_, ground_lon_, ground_alt_,
    target_lat_, target_lon_, target_alt_);

  auto az_msg = std_msgs::msg::Float64();
  az_msg.data = bearing;
  pub_target_az_->publish(az_msg);

  auto el_msg = std_msgs::msg::Float64();
  el_msg.data = std::max(0.0, std::min(90.0, elev));
  pub_target_el_->publish(el_msg);
}

double NavigationNode::haversine_bearing(
  double lat1, double lon1, double lat2, double lon2)
{
  double phi1 = lat1 * DEG_TO_RAD;
  double phi2 = lat2 * DEG_TO_RAD;
  double delta_lambda = (lon2 - lon1) * DEG_TO_RAD;

  double y = std::sin(delta_lambda) * std::cos(phi2);
  double x = std::cos(phi1) * std::sin(phi2) -
             std::sin(phi1) * std::cos(phi2) * std::cos(delta_lambda);

  double bearing = std::atan2(y, x) * RAD_TO_DEG;
  if (bearing < 0.0) {
    bearing += 360.0;
  }
  return bearing;
}

double NavigationNode::elevation_angle(
  double lat1, double lon1, double alt1,
  double lat2, double lon2, double alt2)
{
  double distance = haversine_distance(lat1, lon1, lat2, lon2);
  if (distance < 1.0) {
    return 90.0;
  }

  double delta_alt = alt2 - alt1;
  return std::atan2(delta_alt, distance) * RAD_TO_DEG;
}

double NavigationNode::haversine_distance(
  double lat1, double lon1, double lat2, double lon2)
{
  double phi1 = lat1 * DEG_TO_RAD;
  double phi2 = lat2 * DEG_TO_RAD;
  double delta_phi = (lat2 - lat1) * DEG_TO_RAD;
  double delta_lambda = (lon2 - lon1) * DEG_TO_RAD;

  double a = std::sin(delta_phi / 2.0) * std::sin(delta_phi / 2.0) +
             std::cos(phi1) * std::cos(phi2) *
             std::sin(delta_lambda / 2.0) * std::sin(delta_lambda / 2.0);
  double c = 2.0 * std::atan2(std::sqrt(a), std::sqrt(1.0 - a));

  return EARTH_RADIUS_M * c;
}

}  // namespace antenna_tracker_controller

#ifndef ANTENNA_TRACKER_TEST_ENV
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<antenna_tracker_controller::NavigationNode>());
  rclcpp::shutdown();
  return 0;
}
#endif
