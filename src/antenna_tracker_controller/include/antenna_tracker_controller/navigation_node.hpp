#ifndef ANTENNA_TRACKER_CONTROLLER__NAVIGATION_NODE_HPP_
#define ANTENNA_TRACKER_CONTROLLER__NAVIGATION_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <antenna_tracker_msgs/msg/target_gps.hpp>

namespace antenna_tracker_controller
{

class NavigationNode : public rclcpp::Node
{
public:
  explicit NavigationNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
  void target_callback(const antenna_tracker_msgs::msg::TargetGPS::SharedPtr msg);
  void mode_callback(const std_msgs::msg::UInt8::SharedPtr msg);
  void compute_and_publish();

public:
  static double haversine_bearing(double lat1, double lon1, double lat2, double lon2);
  static double elevation_angle(double lat1, double lon1, double alt1,
                                double lat2, double lon2, double alt2);
  static double haversine_distance(double lat1, double lon1, double lat2, double lon2);

  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_gps_;
  rclcpp::Subscription<antenna_tracker_msgs::msg::TargetGPS>::SharedPtr sub_target_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr sub_mode_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_target_az_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_target_el_;

  uint8_t current_mode_{0};  // 0=AUTO, 1=MANUAL, 2=STANDBY

  double ground_lat_{0.0};
  double ground_lon_{0.0};
  double ground_alt_{0.0};
  bool ground_gps_valid_{false};

  double target_lat_{0.0};
  double target_lon_{0.0};
  double target_alt_{0.0};
  bool target_valid_{false};
};

}  // namespace antenna_tracker_controller

#endif
