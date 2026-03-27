#ifndef ANTENNA_TRACKER_HARDWARE__CAN_FRAME_CODEC_HPP_
#define ANTENNA_TRACKER_HARDWARE__CAN_FRAME_CODEC_HPP_

#include <array>
#include <optional>

#include <linux/can.h>

#include <antenna_tracker_msgs/msg/balloon_telemetry.hpp>
#include <antenna_tracker_msgs/msg/encoder_feedback.hpp>
#include <antenna_tracker_msgs/msg/motor_command.hpp>
#include <antenna_tracker_msgs/msg/target_gps.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

namespace antenna_tracker_hardware
{

struct ImuAssemblyState
{
  sensor_msgs::msg::Imu pending_imu;
  bool accel_ready{false};
  bool gyro_ready{false};
};

class BalloonTelemetryAssembler
{
public:
  std::optional<antenna_tracker_msgs::msg::TargetGPS> process_target_gps(
    const struct can_frame & frame);
  void process_target_status(const struct can_frame & frame);
  void process_balloon_utc(const struct can_frame & frame);
  void process_balloon_accel(const struct can_frame & frame);
  void process_balloon_gyromag(const struct can_frame & frame);
  void process_balloon_orient(const struct can_frame & frame);
  void process_balloon_env(const struct can_frame & frame);
  void process_balloon_press(const struct can_frame & frame);
  void process_balloon_air(const struct can_frame & frame);
  void process_balloon_sys(const struct can_frame & frame);
  void process_balloon_meta(const struct can_frame & frame);

  std::optional<antenna_tracker_msgs::msg::BalloonTelemetry> take_complete_message();
  uint16_t rx_mask() const;

private:
  antenna_tracker_msgs::msg::BalloonTelemetry pending_balloon_;
  uint16_t balloon_rx_mask_{0};
  float rssi_dbm_{0.0f};
  uint8_t link_quality_{0};
  bool has_status_{false};
};

std::optional<sensor_msgs::msg::Imu> process_accel_frame(
  const struct can_frame & frame,
  ImuAssemblyState & state);

std::optional<sensor_msgs::msg::Imu> process_gyro_frame(
  const struct can_frame & frame,
  ImuAssemblyState & state);

std::optional<sensor_msgs::msg::MagneticField> decode_mag_frame(
  const struct can_frame & frame);

std::optional<sensor_msgs::msg::NavSatFix> decode_gps_fix_frame(
  const struct can_frame & frame);

std::optional<antenna_tracker_msgs::msg::EncoderFeedback> decode_encoder_frame(
  const struct can_frame & frame);

std::array<uint8_t, 5> encode_motor_command_payload(
  const antenna_tracker_msgs::msg::MotorCommand & msg);

}  // namespace antenna_tracker_hardware

#endif  // ANTENNA_TRACKER_HARDWARE__CAN_FRAME_CODEC_HPP_
