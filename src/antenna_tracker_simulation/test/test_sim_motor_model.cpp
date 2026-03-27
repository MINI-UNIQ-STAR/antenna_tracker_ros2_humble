#include <gtest/gtest.h>

#include "antenna_tracker_simulation/sim_motor_model.hpp"

// NOTE: CAN delay/drop simulation (can_delay_ms, can_drop_rate) is implemented
// at the ROS2 node layer (SimMotorBridge) and is validated via E2E tests.
// The tests below cover the underlying sim_motor_model physics only.

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

TEST(SimMotorModelTest, DefaultConfigMatchesMeasuredHardwareParams)
{
  // 실측 하드웨어: NEMA23 1.2Nm + 20:1 기어박스(500g) + 야기안테나 500g @0.9m
  // 이 기본값이 변경되면 시뮬레이션이 실제 하드웨어와 달라짐
  SimMotorConfig config;

  EXPECT_DOUBLE_EQ(config.gravity_coeff_rads2, 10.77);
  EXPECT_DOUBLE_EQ(config.motor_max_accel_dps2, 1800.0);
  EXPECT_DOUBLE_EQ(config.az_backlash_deg, 0.8);
  EXPECT_DOUBLE_EQ(config.el_backlash_deg, 0.5);
  EXPECT_DOUBLE_EQ(config.step_loss_accel_threshold_dps2, 2232.0);
  EXPECT_DOUBLE_EQ(config.step_loss_rate, 0.002);
  EXPECT_DOUBLE_EQ(config.load_mass_kg, 0.5);
  EXPECT_DOUBLE_EQ(config.arm_length_m, 0.9);
  EXPECT_DOUBLE_EQ(config.load_inertia_kgm2, 0.41);
  EXPECT_DOUBLE_EQ(config.motor_holding_torque_nm, 1.2);
  EXPECT_DOUBLE_EQ(config.gearbox_efficiency, 0.85);
}

TEST(SimMotorModelTest, NoCandropNoCandDelayPassesCommandThrough) {
  // Verify that with no delay/drop, a large command is applied immediately.
  // This is the existing behavior — regression guard.
  SimMotorConfig config;
  SimMotorState state;

  antenna_tracker_msgs::msg::MotorCommand cmd;
  cmd.az_frequency_hz = 500.0;
  cmd.el_frequency_hz = 300.0;
  cmd.az_direction = true;
  cmd.el_direction = true;
  cmd.emergency_stop = false;

  apply_motor_command(cmd, state);

  EXPECT_NEAR(state.az_command_hz,  500.0, 1e-9);
  EXPECT_NEAR(state.el_command_hz,  300.0, 1e-9);
}

TEST(SimMotorModelTest, BacklashAsymmetricFwdLargerThanRev)
{
  // BL-1: az_backlash_fwd_deg(1.5) > az_backlash_rev_deg(0.3)
  // 양방향 각각 소진 후 fwd 방향이 더 긴 dead zone을 가짐
  auto make_config = [](double fwd, double rev) {
    SimMotorConfig c;
    c.command_deadzone_hz = 0.0;
    c.command_frequency_noise_hz = 0.0;
    c.encoder_angle_noise_deg = 0.0;
    c.encoder_velocity_noise_dps = 0.0;
    c.disturbance_dps2 = 0.0;
    c.gravity_coeff_rads2 = 0.0;
    c.az_damping = 0.0;
    c.az_backlash_deg = 0.8;        // symmetric fallback (불사용)
    c.az_backlash_fwd_deg = fwd;
    c.az_backlash_rev_deg = rev;
    c.az_backlash_variance_deg = 0.0;
    c.el_backlash_deg = 0.0;
    c.step_loss_accel_threshold_dps2 = 9999.0;
    return c;
  };

  // fwd 방향(양속도)으로 이동 후 방향 전환 — dead zone 스텝 수 측정
  auto count_frozen_steps = [&](double fwd_deg, double rev_deg, double vel) -> int {
    SimMotorConfig cfg = make_config(fwd_deg, rev_deg);
    SimMotorState s;
    s.az_vel_rps = -vel;  // 먼저 반대 방향으로 백래시 소진
    for (int i = 0; i < 200; ++i) step_sim_motor_model(s, cfg);
    // 방향 전환
    s.az_vel_rps = vel;
    const double pos0 = s.az_pos_rad;
    int frozen = 0;
    for (int i = 0; i < 200; ++i) {
      step_sim_motor_model(s, cfg);
      const double delta = std::abs(s.az_pos_rad - pos0);
      if (delta < 1e-9) ++frozen;
      else break;
    }
    return frozen;
  };

  const int fwd_frozen = count_frozen_steps(1.5, 0.3, 0.5);
  const int rev_frozen = count_frozen_steps(0.3, 1.5, 0.5);

  // fwd(1.5°) 방향 전환 시 dead zone이 rev(0.3°)보다 길어야 함
  EXPECT_GT(fwd_frozen, rev_frozen);
}

TEST(SimMotorModelTest, BacklashVarianceCausesPerReversalVariation)
{
  // BL-2: variance > 0 이면 연속 방향 전환마다 dead zone 크기가 달라짐
  SimMotorConfig config;
  config.command_deadzone_hz = 0.0;
  config.command_frequency_noise_hz = 0.0;
  config.encoder_angle_noise_deg = 0.0;
  config.encoder_velocity_noise_dps = 0.0;
  config.disturbance_dps2 = 0.0;
  config.gravity_coeff_rads2 = 0.0;
  config.az_damping = 0.0;
  config.az_backlash_deg = 0.5;
  config.az_backlash_variance_deg = 0.3;  // 높은 분산 → 각 전환마다 변동
  config.el_backlash_deg = 0.0;
  config.step_loss_accel_threshold_dps2 = 9999.0;

  SimMotorState state;
  state.rng = std::mt19937{12345};  // 고정 seed로 재현 가능

  // 여러 번 방향 전환 후 current_limit_rad 값이 변하는지 확인
  std::vector<double> limits;
  for (int rev = 0; rev < 6; ++rev) {
    const double vel = (rev % 2 == 0) ? 0.5 : -0.5;
    state.az_vel_rps = vel;
    for (int i = 0; i < 50; ++i) step_sim_motor_model(state, config);
    limits.push_back(state.az_backlash_current_limit_rad);
  }

  // 최소 하나의 전환에서 다른 limit이 샘플링되어야 함
  const double first = limits.front();
  bool any_different = false;
  for (size_t i = 1; i < limits.size(); ++i) {
    if (std::abs(limits[i] - first) > 1e-9) {
      any_different = true;
      break;
    }
  }
  EXPECT_TRUE(any_different);
}

TEST(SimMotorModelTest, BacklashSymmetricCompatibilityWithZeroAsymmetric)
{
  // BL-3: fwd=0, rev=0 → symmetric az_backlash_deg 폴백 → AC-3/AC-4와 동일 동작
  SimMotorConfig config;
  config.command_deadzone_hz = 0.0;
  config.command_frequency_noise_hz = 0.0;
  config.encoder_angle_noise_deg = 0.0;
  config.encoder_velocity_noise_dps = 0.0;
  config.disturbance_dps2 = 0.0;
  config.gravity_coeff_rads2 = 0.0;
  config.az_damping = 0.0;
  config.az_backlash_deg = 0.8;
  config.az_backlash_fwd_deg = 0.0;  // 폴백 → 0.8° 사용
  config.az_backlash_rev_deg = 0.0;
  config.az_backlash_variance_deg = 0.0;
  config.el_backlash_deg = 0.0;
  config.step_loss_accel_threshold_dps2 = 9999.0;

  SimMotorState state;
  state.az_vel_rps = 0.5;
  for (int i = 0; i < 100; ++i) step_sim_motor_model(state, config);

  state.az_vel_rps = -0.5;
  const double pos_before = state.az_pos_rad;
  for (int i = 0; i < 3; ++i) step_sim_motor_model(state, config);

  const double delta_deg =
    std::abs((state.az_pos_rad - pos_before) * 180.0 / M_PI);
  EXPECT_LT(delta_deg, 0.01);  // AC-3과 동일 기준
}

TEST(SimMotorModelTest, StepLossRecoveryExponentialDecay)
{
  // SL-1/SL-2: recovery_rate > 0 시 정지 구간에서 step error 지수 감소
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
  config.motor_max_accel_dps2 = 9999.0;
  config.step_loss_accel_threshold_dps2 = 500.0;  // 낮은 threshold → 쉽게 탈조
  config.step_loss_rate = 0.01;
  config.step_loss_recovery_rate = 5.0;  // 1/s: ~0.2s 후 거의 소멸

  SimMotorState state;
  // 탈조 발생
  state.az_command_hz = 3000.0;
  for (int i = 0; i < 20; ++i) step_sim_motor_model(state, config);
  const double error_after_loss = std::abs(state.az_step_error_rad);
  ASSERT_GT(error_after_loss, 0.0);

  // 명령 정지 → recovery만 작동
  state.az_command_hz = 0.0;
  for (int i = 0; i < 50; ++i) step_sim_motor_model(state, config);  // 0.5s
  const double error_after_recovery = std::abs(state.az_step_error_rad);

  // 복원 후 오차가 줄어야 함
  EXPECT_LT(error_after_recovery, error_after_loss);
  // 복원식 검증: error * exp(-5.0 * 0.5) = error * ~0.082 → 90% 이상 감소
  EXPECT_LT(error_after_recovery, error_after_loss * 0.15);
}

TEST(SimMotorModelTest, StepLossRecoveryZeroPreservesExistingBehavior)
{
  // SL-3: recovery_rate=0.0 → AC-5/AC-6 완전 보존
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
  config.motor_max_accel_dps2 = 9999.0;
  config.step_loss_accel_threshold_dps2 = 500.0;
  config.step_loss_rate = 0.01;
  config.step_loss_recovery_rate = 0.0;  // 복원 없음

  SimMotorState state;
  state.az_command_hz = 3000.0;
  for (int i = 0; i < 20; ++i) step_sim_motor_model(state, config);
  const double error_with_loss = std::abs(state.az_step_error_rad);
  ASSERT_GT(error_with_loss, 0.0);

  // 명령 정지 후에도 오차 유지 (복원 없음)
  state.az_command_hz = 0.0;
  for (int i = 0; i < 100; ++i) step_sim_motor_model(state, config);
  EXPECT_NEAR(std::abs(state.az_step_error_rad), error_with_loss, 1e-12);
}

}  // namespace antenna_tracker_simulation
