#include <gtest/gtest.h>

#include "antenna_tracker_simulation/sim_motor_model.hpp"

namespace antenna_tracker_simulation
{

TEST(SimMotorModelTest, ApplyMotorCommandStoresSignedFrequenciesAndHandlesEstop)
{
  SimMotorState state;
  antenna_tracker_msgs::msg::MotorCommand msg;
  msg.az_frequency_hz = 120.0;
  msg.el_frequency_hz = 45.0;
  msg.az_direction = false;
  msg.el_direction = true;

  apply_motor_command(msg, state);
  EXPECT_DOUBLE_EQ(state.az_command_hz, -120.0);
  EXPECT_DOUBLE_EQ(state.el_command_hz, 45.0);

  msg.emergency_stop = true;
  apply_motor_command(msg, state);
  EXPECT_DOUBLE_EQ(state.az_command_hz, 0.0);
  EXPECT_DOUBLE_EQ(state.el_command_hz, 0.0);
}

TEST(SimMotorModelTest, GearRatioAndStepperGeometryDrivePhysicalAcceleration)
{
  SimMotorConfig config;
  config.command_deadzone_hz = 0.0;
  config.command_frequency_noise_hz = 0.0;
  config.encoder_angle_noise_deg = 0.0;
  config.encoder_velocity_noise_dps = 0.0;
  config.disturbance_dps2 = 0.0;
  config.az_damping = 0.0;
  config.gravity_coeff_rads2 = 0.0;
  config.motor_max_accel_dps2 = 1000.0;
  config.az_backlash_deg = 0.0;           // 백래시 비활성화: 기구학 계산 테스트
  config.el_backlash_deg = 0.0;
  config.step_loss_accel_threshold_dps2 = 9999.0;

  SimMotorState state;
  state.az_command_hz = 100.0;

  const auto step = step_sim_motor_model(state, config);

  EXPECT_DOUBLE_EQ(pulses_per_output_revolution(config), 4000.0);
  EXPECT_NEAR(step.az_velocity_command_rps, 0.1570796327, 1e-6);
  EXPECT_NEAR(step.feedback.az_velocity_dps, 9.0, 1e-3);
  EXPECT_NEAR(step.feedback.az_angle_deg, 0.09, 1e-3);
}

TEST(SimMotorModelTest, WrapsAzimuthAndClampsElevationLimits)
{
  SimMotorConfig config;
  config.command_deadzone_hz = 0.0;
  config.command_frequency_noise_hz = 0.0;
  config.encoder_angle_noise_deg = 0.0;
  config.encoder_velocity_noise_dps = 0.0;
  config.disturbance_dps2 = 0.0;
  config.gravity_coeff_rads2 = 0.0;
  config.az_damping = 0.0;
  config.az_backlash_deg = 0.0;           // 백래시 비활성화: 경계 조건 테스트
  config.el_backlash_deg = 0.0;
  config.step_loss_accel_threshold_dps2 = 9999.0;

  SimMotorState state;
  state.az_pos_rad = kTwoPi - 0.001;
  state.az_vel_rps = 0.2;
  state.el_pos_rad = 0.001;
  state.el_vel_rps = -1.0;

  const auto step = step_sim_motor_model(state, config);

  EXPECT_GE(step.feedback.az_angle_deg, 0.0);
  EXPECT_LT(step.feedback.az_angle_deg, 1.0);
  EXPECT_DOUBLE_EQ(step.feedback.el_angle_deg, 0.0);
  EXPECT_DOUBLE_EQ(step.feedback.el_velocity_dps, 0.0);
}

TEST(SimMotorModelTest, ElevationAxisCanClimbAgainstGravityWithCompensatedCommand)
{
  // 실측 파라미터: gravity_coeff=10.77, max_accel=1800 dps²
  // 200 Hz 명령 → accel ≈ 31.4 rad/s² > gravity 10.77 rad/s² → 상승 가능
  SimMotorConfig config;
  config.command_deadzone_hz = 0.0;
  config.command_frequency_noise_hz = 0.0;
  config.encoder_angle_noise_deg = 0.0;
  config.encoder_velocity_noise_dps = 0.0;
  config.disturbance_dps2 = 0.0;
  config.az_damping = 0.0;
  config.gravity_coeff_rads2 = 10.77;
  config.motor_max_accel_dps2 = 1800.0;
  config.az_backlash_deg = 0.0;
  config.el_backlash_deg = 0.0;
  config.step_loss_accel_threshold_dps2 = 9999.0;  // step loss 비활성화

  SimMotorState state;
  state.el_command_hz = 200.0;

  const auto step = step_sim_motor_model(state, config);

  EXPECT_GT(step.el_velocity_command_rps, 0.0);
  EXPECT_GT(step.feedback.el_velocity_dps, 0.0);
  EXPECT_GT(step.feedback.el_angle_deg, 0.0);
}

TEST(SimMotorModelTest, BacklashDeadZoneOnDirectionReversal)
{
  // AC-3: 방향 전환 직후 az_backlash_deg(0.8°) 구간에서 출력 위치 변화 < 0.01°
  // 주의: 관성 우회를 위해 az_command_hz=0으로 두고 az_vel_rps를 직접 제어
  // (step에서 command_accel=0, damping=0 → vel은 외부에서 설정한 값 유지)
  SimMotorConfig config;
  config.command_deadzone_hz = 0.0;
  config.command_frequency_noise_hz = 0.0;
  config.encoder_angle_noise_deg = 0.0;
  config.encoder_velocity_noise_dps = 0.0;
  config.disturbance_dps2 = 0.0;
  config.gravity_coeff_rads2 = 0.0;
  config.az_damping = 0.0;
  config.az_backlash_deg = 0.8;
  config.el_backlash_deg = 0.0;
  config.step_loss_accel_threshold_dps2 = 9999.0;

  SimMotorState state;
  state.az_command_hz = 0.0;  // command 없이 vel 직접 제어

  // 양방향으로 충분히 이동 → last_dir=+1, 백래시 소진
  state.az_vel_rps = 0.5;
  for (int i = 0; i < 100; ++i) {
    step_sim_motor_model(state, config);
  }

  // 방향 전환: 음속도로 설정 → apply_backlash_delta가 방향 전환 감지
  state.az_vel_rps = -0.5;
  const double pos_before_reversal = state.az_pos_rad;

  // 백래시 구간 (3스텝): 위치 고정
  for (int i = 0; i < 3; ++i) {
    step_sim_motor_model(state, config);
  }
  const double pos_after_few_steps = state.az_pos_rad;

  const double delta_deg = std::abs(
    (pos_after_few_steps - pos_before_reversal) * 180.0 / 3.14159265358979);
  EXPECT_LT(delta_deg, 0.01);
}

TEST(SimMotorModelTest, BacklashReEngagesAfterZoneCleared)
{
  // AC-4: 백래시 구간(0.8°) 완전 소진 후 출력 위치가 다시 추종
  SimMotorConfig config;
  config.command_deadzone_hz = 0.0;
  config.command_frequency_noise_hz = 0.0;
  config.encoder_angle_noise_deg = 0.0;
  config.encoder_velocity_noise_dps = 0.0;
  config.disturbance_dps2 = 0.0;
  config.gravity_coeff_rads2 = 0.0;
  config.az_damping = 0.0;
  config.az_backlash_deg = 0.8;
  config.el_backlash_deg = 0.0;
  config.step_loss_accel_threshold_dps2 = 9999.0;

  SimMotorState state;
  // 양방향 이동으로 last_dir 확립
  state.az_command_hz = 100.0;
  for (int i = 0; i < 50; ++i) {
    step_sim_motor_model(state, config);
  }

  // 방향 전환 후 충분히 이동 (백래시 소진)
  state.az_command_hz = -100.0;
  for (int i = 0; i < 100; ++i) {
    step_sim_motor_model(state, config);
  }
  const double pos_a = state.az_pos_rad;

  // 추가 이동 후 위치가 변해야 함 (정상 추종)
  for (int i = 0; i < 10; ++i) {
    step_sim_motor_model(state, config);
  }
  const double pos_b = state.az_pos_rad;

  EXPECT_GT(pos_a, pos_b);  // 음방향이므로 pos 감소
}

TEST(SimMotorModelTest, StepLossAccumulatesAboveThreshold)
{
  // AC-5: 가속도 > 2232 dps² 시 step error 발생
  SimMotorConfig config;
  config.command_deadzone_hz = 0.0;
  config.command_frequency_noise_hz = 0.0;
  config.encoder_angle_noise_deg = 0.0;
  config.encoder_velocity_noise_dps = 0.0;
  config.disturbance_dps2 = 0.0;
  config.gravity_coeff_rads2 = 0.0;
  config.az_damping = 0.0;
  config.az_backlash_deg = 0.0;
  config.el_backlash_deg = 0.0;
  config.motor_max_accel_dps2 = 9999.0;  // 클램핑 없이 고가속도 허용
  config.step_loss_accel_threshold_dps2 = 2232.0;
  config.step_loss_rate = 0.002;

  SimMotorState state;
  // threshold(2232)를 초과하는 가속도 유발: 충분히 큰 hz
  state.az_command_hz = 3000.0;

  for (int i = 0; i < 10; ++i) {
    step_sim_motor_model(state, config);
  }

  EXPECT_GT(std::abs(state.az_step_error_rad), 0.0);
}

TEST(SimMotorModelTest, StepLossZeroBelowThreshold)
{
  // AC-6: 가속도 ≤ 1800 dps² 시 step error = 0
  SimMotorConfig config;
  config.command_deadzone_hz = 0.0;
  config.command_frequency_noise_hz = 0.0;
  config.encoder_angle_noise_deg = 0.0;
  config.encoder_velocity_noise_dps = 0.0;
  config.disturbance_dps2 = 0.0;
  config.gravity_coeff_rads2 = 0.0;
  config.az_damping = 0.0;
  config.az_backlash_deg = 0.0;
  config.el_backlash_deg = 0.0;
  config.motor_max_accel_dps2 = 1800.0;
  config.step_loss_accel_threshold_dps2 = 2232.0;
  config.step_loss_rate = 0.002;

  SimMotorState state;
  state.az_command_hz = 100.0;  // max_accel 1800 dps² 이내

  for (int i = 0; i < 100; ++i) {
    step_sim_motor_model(state, config);
  }

  EXPECT_DOUBLE_EQ(state.az_step_error_rad, 0.0);
}

TEST(SimMotorModelTest, StepLossGravityDependentOnElevation)
{
  // 수평(el=0°): 중력 최대 → step loss threshold 감소 → 더 쉽게 탈조
  // 고각(el=80°): 중력 작음 → threshold 유지 → 덜 탈조
  SimMotorConfig config;
  config.command_deadzone_hz = 0.0;
  config.command_frequency_noise_hz = 0.0;
  config.encoder_angle_noise_deg = 0.0;
  config.encoder_velocity_noise_dps = 0.0;
  config.disturbance_dps2 = 0.0;
  config.az_damping = 0.0;
  config.az_backlash_deg = 0.0;
  config.el_backlash_deg = 0.0;
  config.motor_max_accel_dps2 = 9999.0;
  config.gravity_coeff_rads2 = 10.77;
  config.step_loss_accel_threshold_dps2 = 2232.0;
  config.step_loss_rate = 0.002;

  // 수평 케이스: el = 0°, 중력 부하 최대
  SimMotorState state_horizontal;
  state_horizontal.el_pos_rad = 0.0;
  state_horizontal.el_command_hz = 2000.0;
  for (int i = 0; i < 20; ++i) {
    step_sim_motor_model(state_horizontal, config);
  }

  // 고각 케이스: el = 80°, 중력 부하 최소 (cos(80°) ≈ 0.17)
  SimMotorState state_high;
  state_high.el_pos_rad = 80.0 * 3.14159265358979 / 180.0;
  state_high.el_command_hz = 2000.0;
  for (int i = 0; i < 20; ++i) {
    step_sim_motor_model(state_high, config);
  }

  // 수평에서 탈조 오차가 더 크거나 같아야 함
  EXPECT_GE(
    std::abs(state_horizontal.el_step_error_rad),
    std::abs(state_high.el_step_error_rad));
}

TEST(SimMotorModelTest, DeterministicNoisePerturbsFeedbackButKeepsBounds)
{
  SimMotorConfig clean_config;
  clean_config.command_deadzone_hz = 0.0;
  clean_config.disturbance_dps2 = 0.0;
  clean_config.command_frequency_noise_hz = 0.0;
  clean_config.encoder_angle_noise_deg = 0.0;
  clean_config.encoder_velocity_noise_dps = 0.0;

  SimMotorConfig noisy_config = clean_config;
  noisy_config.command_frequency_noise_hz = 3.0;
  noisy_config.encoder_angle_noise_deg = 0.35;
  noisy_config.encoder_velocity_noise_dps = 1.5;
  noisy_config.disturbance_dps2 = 1.0;

  SimMotorState clean_state;
  SimMotorState noisy_state;
  clean_state.az_command_hz = 80.0;
  clean_state.el_command_hz = 65.0;
  noisy_state.az_command_hz = 80.0;
  noisy_state.el_command_hz = 65.0;
  noisy_state.el_pos_rad = 0.6;
  clean_state.el_pos_rad = 0.6;

  SimMotorStepResult clean_step;
  SimMotorStepResult noisy_step;
  for (int i = 0; i < 50; ++i) {
    clean_step = step_sim_motor_model(clean_state, clean_config);
    noisy_step = step_sim_motor_model(noisy_state, noisy_config);
  }

  EXPECT_NE(noisy_step.feedback.az_angle_deg, clean_step.feedback.az_angle_deg);
  EXPECT_NE(noisy_step.feedback.el_velocity_dps, clean_step.feedback.el_velocity_dps);
  EXPECT_GE(noisy_step.feedback.az_angle_deg, 0.0);
  EXPECT_LT(noisy_step.feedback.az_angle_deg, 360.0);
  EXPECT_GE(noisy_step.feedback.el_angle_deg, 0.0);
  EXPECT_LE(noisy_step.feedback.el_angle_deg, 90.0);
}

}  // namespace antenna_tracker_simulation
