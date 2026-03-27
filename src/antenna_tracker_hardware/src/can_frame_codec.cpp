#include "antenna_tracker_hardware/can_frame_codec.hpp"

#include <cstring>

namespace antenna_tracker_hardware
{
namespace
{

constexpr uint16_t BIT_GPS     = (1 << 0);
constexpr uint16_t BIT_STATUS  = (1 << 1);
constexpr uint16_t BIT_UTC     = (1 << 2);
constexpr uint16_t BIT_ACCEL   = (1 << 3);
constexpr uint16_t BIT_GYROMAG = (1 << 4);
constexpr uint16_t BIT_ORIENT  = (1 << 5);
constexpr uint16_t BIT_ENV     = (1 << 6);
constexpr uint16_t BIT_PRESS   = (1 << 7);
constexpr uint16_t BIT_AIR     = (1 << 8);
constexpr uint16_t BIT_SYS     = (1 << 9);
constexpr uint16_t BIT_META    = (1 << 10);
constexpr uint16_t BALLOON_FULL_MASK = 0x07FFu;

int16_t read_i16_be(const uint8_t * data)
{
  return static_cast<int16_t>(
    (static_cast<uint16_t>(data[0]) << 8) |
    static_cast<uint16_t>(data[1]));
}

template<typename T>
T read_le(const uint8_t * data)
{
  T value{};
  std::memcpy(&value, data, sizeof(T));
  return value;
}

}  // namespace

std::optional<antenna_tracker_msgs::msg::TargetGPS> BalloonTelemetryAssembler::process_target_gps(
  const struct can_frame & frame)
{
  if (frame.can_dlc < 8) {
    return std::nullopt;
  }

  const auto lat_raw = read_le<int32_t>(&frame.data[0]);
  const auto lon_raw = read_le<int32_t>(&frame.data[4]);

  antenna_tracker_msgs::msg::TargetGPS msg;
  msg.latitude = lat_raw / 1e7;
  msg.longitude = lon_raw / 1e7;
  msg.altitude_m = has_status_ ? pending_balloon_.kf_altitude_m : 0.0;
  msg.rssi_dbm = has_status_ ? rssi_dbm_ : 0.0f;
  msg.link_quality = has_status_ ? link_quality_ : 0;

  pending_balloon_.latitude = msg.latitude;
  pending_balloon_.longitude = msg.longitude;
  balloon_rx_mask_ |= BIT_GPS;

  return msg;
}

void BalloonTelemetryAssembler::process_target_status(const struct can_frame & frame)
{
  if (frame.can_dlc < 8) {
    return;
  }

  const auto altitude = read_le<int16_t>(&frame.data[0]);
  const auto rssi = read_le<int16_t>(&frame.data[2]);

  rssi_dbm_ = static_cast<float>(rssi);
  link_quality_ = frame.data[4];
  has_status_ = true;

  pending_balloon_.kf_altitude_m = static_cast<float>(altitude);
  pending_balloon_.rssi_dbm = rssi;
  pending_balloon_.gps_fix = frame.data[4];
  pending_balloon_.gps_sats_used = frame.data[5];
  pending_balloon_.utc_hour = frame.data[6];
  pending_balloon_.utc_min = frame.data[7];
  balloon_rx_mask_ |= BIT_STATUS;
}

void BalloonTelemetryAssembler::process_balloon_utc(const struct can_frame & frame)
{
  if (frame.can_dlc < 8) {
    return;
  }

  pending_balloon_.utc_sec = frame.data[0];
  pending_balloon_.utc_day = frame.data[1];
  pending_balloon_.utc_month = frame.data[2];
  pending_balloon_.gps_sats_in_view = frame.data[3];
  pending_balloon_.utc_year = read_le<uint16_t>(&frame.data[4]);
  pending_balloon_.status_flags = read_le<uint16_t>(&frame.data[6]);
  balloon_rx_mask_ |= BIT_UTC;
}

void BalloonTelemetryAssembler::process_balloon_accel(const struct can_frame & frame)
{
  if (frame.can_dlc < 8) {
    return;
  }

  pending_balloon_.accel_x_mps2 = read_le<int16_t>(&frame.data[0]) / 100.0f;
  pending_balloon_.accel_y_mps2 = read_le<int16_t>(&frame.data[2]) / 100.0f;
  pending_balloon_.accel_z_mps2 = read_le<int16_t>(&frame.data[4]) / 100.0f;
  pending_balloon_.gyro_x_rads = read_le<int16_t>(&frame.data[6]) / 1000.0f;
  balloon_rx_mask_ |= BIT_ACCEL;
}

void BalloonTelemetryAssembler::process_balloon_gyromag(const struct can_frame & frame)
{
  if (frame.can_dlc < 8) {
    return;
  }

  pending_balloon_.gyro_y_rads = read_le<int16_t>(&frame.data[0]) / 1000.0f;
  pending_balloon_.gyro_z_rads = read_le<int16_t>(&frame.data[2]) / 1000.0f;
  pending_balloon_.mag_x_ut = read_le<int16_t>(&frame.data[4]) / 10.0f;
  pending_balloon_.mag_y_ut = read_le<int16_t>(&frame.data[6]) / 10.0f;
  balloon_rx_mask_ |= BIT_GYROMAG;
}

void BalloonTelemetryAssembler::process_balloon_orient(const struct can_frame & frame)
{
  if (frame.can_dlc < 8) {
    return;
  }

  pending_balloon_.mag_z_ut = read_le<int16_t>(&frame.data[0]) / 10.0f;
  pending_balloon_.kf_roll_deg = read_le<int16_t>(&frame.data[2]) / 100.0f;
  pending_balloon_.kf_pitch_deg = read_le<int16_t>(&frame.data[4]) / 100.0f;
  pending_balloon_.press_altitude_m = static_cast<float>(read_le<int16_t>(&frame.data[6]));
  balloon_rx_mask_ |= BIT_ORIENT;
}

void BalloonTelemetryAssembler::process_balloon_env(const struct can_frame & frame)
{
  if (frame.can_dlc < 8) {
    return;
  }

  pending_balloon_.board_temp_c = read_le<int16_t>(&frame.data[0]) / 100.0f;
  pending_balloon_.external_temp_c = read_le<int16_t>(&frame.data[2]) / 100.0f;
  pending_balloon_.sht31_temp_c = read_le<int16_t>(&frame.data[4]) / 100.0f;
  pending_balloon_.sht31_rh_percent = read_le<uint16_t>(&frame.data[6]) / 100.0f;
  balloon_rx_mask_ |= BIT_ENV;
}

void BalloonTelemetryAssembler::process_balloon_press(const struct can_frame & frame)
{
  if (frame.can_dlc < 8) {
    return;
  }

  pending_balloon_.ms5611_press_pa = read_le<uint32_t>(&frame.data[0]);
  pending_balloon_.ms5611_temp_c = read_le<int16_t>(&frame.data[4]) / 100.0f;
  pending_balloon_.co2_ppm = read_le<uint16_t>(&frame.data[6]);
  balloon_rx_mask_ |= BIT_PRESS;
}

void BalloonTelemetryAssembler::process_balloon_air(const struct can_frame & frame)
{
  if (frame.can_dlc < 8) {
    return;
  }

  pending_balloon_.pm1_ugm3 = read_le<uint16_t>(&frame.data[0]);
  pending_balloon_.pm25_ugm3 = read_le<uint16_t>(&frame.data[2]);
  pending_balloon_.pm10_ugm3 = read_le<uint16_t>(&frame.data[4]);
  pending_balloon_.ozone_ppb = read_le<int16_t>(&frame.data[6]);
  balloon_rx_mask_ |= BIT_AIR;
}

void BalloonTelemetryAssembler::process_balloon_sys(const struct can_frame & frame)
{
  if (frame.can_dlc < 8) {
    return;
  }

  pending_balloon_.gdk101_usvh = read_le<uint16_t>(&frame.data[0]) / 100.0f;
  pending_balloon_.bat_mv = read_le<uint16_t>(&frame.data[2]);
  pending_balloon_.bat_temp_c = read_le<int16_t>(&frame.data[4]) / 100.0f;
  pending_balloon_.heater_bat_duty_percent = frame.data[6];
  pending_balloon_.heater_board_duty_percent = frame.data[7];
  balloon_rx_mask_ |= BIT_SYS;
}

void BalloonTelemetryAssembler::process_balloon_meta(const struct can_frame & frame)
{
  if (frame.can_dlc < 6) {
    return;
  }

  pending_balloon_.seq = read_le<uint16_t>(&frame.data[0]);
  pending_balloon_.uptime_ms = read_le<uint32_t>(&frame.data[2]);
  balloon_rx_mask_ |= BIT_META;
}

std::optional<antenna_tracker_msgs::msg::BalloonTelemetry>
BalloonTelemetryAssembler::take_complete_message()
{
  if (balloon_rx_mask_ != BALLOON_FULL_MASK) {
    return std::nullopt;
  }

  balloon_rx_mask_ = 0;
  return pending_balloon_;
}

uint16_t BalloonTelemetryAssembler::rx_mask() const
{
  return balloon_rx_mask_;
}

std::optional<sensor_msgs::msg::Imu> process_accel_frame(
  const struct can_frame & frame,
  ImuAssemblyState & state)
{
  if (frame.can_dlc < 6) {
    return std::nullopt;
  }

  state.pending_imu.linear_acceleration.x = read_i16_be(&frame.data[0]) / 1000.0;
  state.pending_imu.linear_acceleration.y = read_i16_be(&frame.data[2]) / 1000.0;
  state.pending_imu.linear_acceleration.z = read_i16_be(&frame.data[4]) / 1000.0;
  state.pending_imu.linear_acceleration_covariance[0] = -1;
  state.accel_ready = true;

  if (!state.gyro_ready) {
    return std::nullopt;
  }

  auto msg = state.pending_imu;
  state.accel_ready = false;
  state.gyro_ready = false;
  return msg;
}

std::optional<sensor_msgs::msg::Imu> process_gyro_frame(
  const struct can_frame & frame,
  ImuAssemblyState & state)
{
  if (frame.can_dlc < 6) {
    return std::nullopt;
  }

  state.pending_imu.angular_velocity.x = read_i16_be(&frame.data[0]) / 1000.0;
  state.pending_imu.angular_velocity.y = read_i16_be(&frame.data[2]) / 1000.0;
  state.pending_imu.angular_velocity.z = read_i16_be(&frame.data[4]) / 1000.0;
  state.pending_imu.angular_velocity_covariance[0] = -1;
  state.pending_imu.orientation_covariance[0] = -1;
  state.gyro_ready = true;

  if (!state.accel_ready) {
    return std::nullopt;
  }

  auto msg = state.pending_imu;
  state.accel_ready = false;
  state.gyro_ready = false;
  return msg;
}

std::optional<sensor_msgs::msg::MagneticField> decode_mag_frame(const struct can_frame & frame)
{
  if (frame.can_dlc < 6) {
    return std::nullopt;
  }

  sensor_msgs::msg::MagneticField msg;
  msg.magnetic_field.x = read_i16_be(&frame.data[0]) / 100.0 * 1e-6;
  msg.magnetic_field.y = read_i16_be(&frame.data[2]) / 100.0 * 1e-6;
  msg.magnetic_field.z = read_i16_be(&frame.data[4]) / 100.0 * 1e-6;
  return msg;
}

std::optional<sensor_msgs::msg::NavSatFix> decode_gps_fix_frame(const struct can_frame & frame)
{
  if (frame.can_dlc < 8) {
    return std::nullopt;
  }

  sensor_msgs::msg::NavSatFix msg;
  msg.latitude = read_le<int32_t>(&frame.data[0]) / 1e7;
  msg.longitude = read_le<int32_t>(&frame.data[4]) / 1e7;
  msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
  return msg;
}

std::optional<antenna_tracker_msgs::msg::EncoderFeedback> decode_encoder_frame(
  const struct can_frame & frame)
{
  if (frame.can_dlc < 5) {
    return std::nullopt;
  }

  antenna_tracker_msgs::msg::EncoderFeedback msg;
  msg.az_angle_deg = read_i16_be(&frame.data[0]) / 10.0;
  msg.el_angle_deg = read_i16_be(&frame.data[2]) / 10.0;
  const auto flags = frame.data[4];
  msg.az_valid = (flags & 0x01) != 0;
  msg.el_valid = (flags & 0x02) != 0;
  return msg;
}

std::array<uint8_t, 5> encode_motor_command_payload(
  const antenna_tracker_msgs::msg::MotorCommand & msg)
{
  const auto az_freq_raw = static_cast<int16_t>(msg.az_frequency_hz * 10.0);
  const auto el_freq_raw = static_cast<int16_t>(msg.el_frequency_hz * 10.0);

  std::array<uint8_t, 5> data{};
  std::memcpy(data.data(), &az_freq_raw, sizeof(az_freq_raw));
  std::memcpy(data.data() + 2, &el_freq_raw, sizeof(el_freq_raw));

  uint8_t flags = 0;
  if (msg.az_direction) {
    flags |= 0x01;
  }
  if (msg.el_direction) {
    flags |= 0x02;
  }
  if (msg.emergency_stop) {
    flags |= 0x04;
  }
  data[4] = flags;

  return data;
}

}  // namespace antenna_tracker_hardware
