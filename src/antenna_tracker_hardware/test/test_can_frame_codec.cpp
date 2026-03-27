#include <gtest/gtest.h>

#include <initializer_list>

#include "antenna_tracker_hardware/can_frame_codec.hpp"

namespace antenna_tracker_hardware
{
namespace
{

can_frame make_frame(uint32_t id, std::initializer_list<uint8_t> bytes)
{
  can_frame frame{};
  frame.can_id = id;
  frame.can_dlc = static_cast<__u8>(bytes.size());
  std::size_t index = 0;
  for (const auto byte : bytes) {
    frame.data[index++] = byte;
  }
  return frame;
}

}  // namespace

TEST(CanFrameCodecTest, EncodesMotorCommandPayloadWithDirectionsAndEmergencyStop)
{
  antenna_tracker_msgs::msg::MotorCommand msg;
  msg.az_frequency_hz = 12.3;
  msg.el_frequency_hz = -4.5;
  msg.az_direction = true;
  msg.el_direction = false;
  msg.emergency_stop = true;

  const auto payload = encode_motor_command_payload(msg);

  EXPECT_EQ(payload[0], 123);
  EXPECT_EQ(payload[1], 0);
  EXPECT_EQ(payload[2], 211);
  EXPECT_EQ(payload[3], 255);
  EXPECT_EQ(payload[4], 0x05);
}

TEST(CanFrameCodecTest, CombinesAccelAndGyroFramesIntoSingleImuMessage)
{
  ImuAssemblyState state;

  const auto accel = process_accel_frame(
    make_frame(0x200, {0x03, 0xE8, 0xFC, 0x18, 0x13, 0x88}),
    state);
  EXPECT_FALSE(accel.has_value());
  EXPECT_TRUE(state.accel_ready);

  const auto imu = process_gyro_frame(
    make_frame(0x201, {0x00, 0x64, 0xFF, 0x9C, 0x01, 0x2C}),
    state);

  ASSERT_TRUE(imu.has_value());
  EXPECT_NEAR(imu->linear_acceleration.x, 1.0, 1e-9);
  EXPECT_NEAR(imu->linear_acceleration.y, -1.0, 1e-9);
  EXPECT_NEAR(imu->linear_acceleration.z, 5.0, 1e-9);
  EXPECT_NEAR(imu->angular_velocity.x, 0.1, 1e-9);
  EXPECT_NEAR(imu->angular_velocity.y, -0.1, 1e-9);
  EXPECT_NEAR(imu->angular_velocity.z, 0.3, 1e-9);
  EXPECT_EQ(imu->linear_acceleration_covariance[0], -1.0);
  EXPECT_EQ(imu->angular_velocity_covariance[0], -1.0);
  EXPECT_EQ(imu->orientation_covariance[0], -1.0);
  EXPECT_FALSE(state.accel_ready);
  EXPECT_FALSE(state.gyro_ready);
}

TEST(CanFrameCodecTest, DecodesMagGpsAndEncoderFrames)
{
  const auto mag = decode_mag_frame(
    make_frame(0x202, {0x00, 0x64, 0xFF, 0x9C, 0x01, 0x2C}));
  ASSERT_TRUE(mag.has_value());
  EXPECT_NEAR(mag->magnetic_field.x, 1.0e-6, 1e-12);
  EXPECT_NEAR(mag->magnetic_field.y, -1.0e-6, 1e-12);
  EXPECT_NEAR(mag->magnetic_field.z, 3.0e-6, 1e-12);

  const auto gps = decode_gps_fix_frame(
    make_frame(0x203, {0x15, 0x96, 0x20, 0x16, 0x30, 0x40, 0xB7, 0x4B}));
  ASSERT_TRUE(gps.has_value());
  EXPECT_NEAR(gps->latitude, 37.1234325, 1e-7);
  EXPECT_NEAR(gps->longitude, 127.0300720, 1e-7);
  EXPECT_EQ(gps->status.status, sensor_msgs::msg::NavSatStatus::STATUS_FIX);

  const auto encoder = decode_encoder_frame(
    make_frame(0x204, {0x04, 0xD2, 0x01, 0xC2, 0x03}));
  ASSERT_TRUE(encoder.has_value());
  EXPECT_DOUBLE_EQ(encoder->az_angle_deg, 123.4);
  EXPECT_DOUBLE_EQ(encoder->el_angle_deg, 45.0);
  EXPECT_TRUE(encoder->az_valid);
  EXPECT_TRUE(encoder->el_valid);
}

TEST(BalloonTelemetryAssemblerTest, TargetGpsUsesStatusFieldsWhenAvailable)
{
  BalloonTelemetryAssembler assembler;

  auto target = assembler.process_target_gps(
    make_frame(0x100, {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}));
  ASSERT_TRUE(target.has_value());
  EXPECT_FLOAT_EQ(target->rssi_dbm, 0.0f);
  EXPECT_EQ(target->link_quality, 0);
  EXPECT_DOUBLE_EQ(target->altitude_m, 0.0);

  assembler.process_target_status(
    make_frame(0x101, {0x7B, 0x00, 0xD8, 0xFF, 0x07, 0x09, 0x0C, 0x22}));
  target = assembler.process_target_gps(
    make_frame(0x100, {0x15, 0x96, 0x20, 0x16, 0x30, 0x40, 0xB7, 0x4B}));

  ASSERT_TRUE(target.has_value());
  EXPECT_NEAR(target->latitude, 37.1234325, 1e-7);
  EXPECT_NEAR(target->longitude, 127.0300720, 1e-7);
  EXPECT_DOUBLE_EQ(target->altitude_m, 123.0);
  EXPECT_FLOAT_EQ(target->rssi_dbm, -40.0f);
  EXPECT_EQ(target->link_quality, 7);
}

TEST(BalloonTelemetryAssemblerTest, EmitsCompleteTelemetryAfterAllFramesArrive)
{
  BalloonTelemetryAssembler assembler;

  assembler.process_target_gps(
    make_frame(0x100, {0x15, 0x96, 0x20, 0x16, 0x30, 0x40, 0xB7, 0x4B}));
  assembler.process_target_status(
    make_frame(0x101, {0x7B, 0x00, 0xD8, 0xFF, 0x07, 0x09, 0x0C, 0x22}));
  assembler.process_balloon_utc(
    make_frame(0x102, {0x2A, 0x1B, 0x03, 0x0A, 0xEA, 0x07, 0x34, 0x12}));
  assembler.process_balloon_accel(
    make_frame(0x103, {0xF4, 0x01, 0x0C, 0xFE, 0xD0, 0x07, 0xCE, 0xFF}));
  assembler.process_balloon_gyromag(
    make_frame(0x104, {0x96, 0x00, 0xD4, 0xFE, 0x2C, 0x01, 0x70, 0xFE}));
  assembler.process_balloon_orient(
    make_frame(0x105, {0x4A, 0x01, 0xC4, 0x09, 0xDA, 0xF8, 0x41, 0x01}));
  assembler.process_balloon_env(
    make_frame(0x106, {0x39, 0x09, 0xD8, 0x07, 0x41, 0x0B, 0xF8, 0x11}));
  assembler.process_balloon_press(
    make_frame(0x107, {0xCD, 0x8B, 0x01, 0x00, 0xCF, 0x08, 0x20, 0x03}));
  assembler.process_balloon_air(
    make_frame(0x108, {0x05, 0x00, 0x0C, 0x00, 0x12, 0x00, 0x19, 0x00}));
  assembler.process_balloon_sys(
    make_frame(0x109, {0x3A, 0x00, 0xD0, 0x0E, 0x4A, 0x09, 0x1E, 0x32}));
  assembler.process_balloon_meta(
    make_frame(0x10A, {0x34, 0x12, 0xD2, 0x04, 0x00, 0x00}));

  const auto balloon = assembler.take_complete_message();
  ASSERT_TRUE(balloon.has_value());
  EXPECT_EQ(assembler.rx_mask(), 0);
  EXPECT_NEAR(balloon->latitude, 37.1234325, 1e-7);
  EXPECT_NEAR(balloon->longitude, 127.0300720, 1e-7);
  EXPECT_FLOAT_EQ(balloon->kf_altitude_m, 123.0f);
  EXPECT_EQ(balloon->gps_fix, 7);
  EXPECT_EQ(balloon->gps_sats_used, 9);
  EXPECT_EQ(balloon->utc_hour, 12);
  EXPECT_EQ(balloon->utc_min, 34);
  EXPECT_EQ(balloon->utc_sec, 42);
  EXPECT_EQ(balloon->utc_day, 27);
  EXPECT_EQ(balloon->utc_month, 3);
  EXPECT_EQ(balloon->utc_year, 2026);
  EXPECT_EQ(balloon->status_flags, 0x1234);
  EXPECT_FLOAT_EQ(balloon->accel_x_mps2, 5.0f);
  EXPECT_FLOAT_EQ(balloon->accel_y_mps2, -5.0f);
  EXPECT_FLOAT_EQ(balloon->accel_z_mps2, 20.0f);
  EXPECT_FLOAT_EQ(balloon->gyro_x_rads, -0.05f);
  EXPECT_FLOAT_EQ(balloon->gyro_y_rads, 0.15f);
  EXPECT_FLOAT_EQ(balloon->gyro_z_rads, -0.3f);
  EXPECT_FLOAT_EQ(balloon->mag_x_ut, 30.0f);
  EXPECT_FLOAT_EQ(balloon->mag_y_ut, -40.0f);
  EXPECT_FLOAT_EQ(balloon->mag_z_ut, 33.0f);
  EXPECT_FLOAT_EQ(balloon->kf_roll_deg, 25.0f);
  EXPECT_FLOAT_EQ(balloon->kf_pitch_deg, -18.3f);
  EXPECT_FLOAT_EQ(balloon->press_altitude_m, 321.0f);
  EXPECT_FLOAT_EQ(balloon->board_temp_c, 23.61f);
  EXPECT_FLOAT_EQ(balloon->external_temp_c, 20.08f);
  EXPECT_FLOAT_EQ(balloon->sht31_temp_c, 28.81f);
  EXPECT_FLOAT_EQ(balloon->sht31_rh_percent, 46.0f);
  EXPECT_EQ(balloon->ms5611_press_pa, 101325u);
  EXPECT_FLOAT_EQ(balloon->ms5611_temp_c, 22.55f);
  EXPECT_EQ(balloon->co2_ppm, 800u);
  EXPECT_EQ(balloon->pm1_ugm3, 5u);
  EXPECT_EQ(balloon->pm25_ugm3, 12u);
  EXPECT_EQ(balloon->pm10_ugm3, 18u);
  EXPECT_EQ(balloon->ozone_ppb, 25);
  EXPECT_FLOAT_EQ(balloon->gdk101_usvh, 0.58f);
  EXPECT_EQ(balloon->bat_mv, 3792u);
  EXPECT_FLOAT_EQ(balloon->bat_temp_c, 23.78f);
  EXPECT_EQ(balloon->heater_bat_duty_percent, 30);
  EXPECT_EQ(balloon->heater_board_duty_percent, 50);
  EXPECT_EQ(balloon->rssi_dbm, -40);
  EXPECT_EQ(balloon->seq, 0x1234);
  EXPECT_EQ(balloon->uptime_ms, 1234u);
  EXPECT_FALSE(assembler.take_complete_message().has_value());
}

}  // namespace antenna_tracker_hardware
