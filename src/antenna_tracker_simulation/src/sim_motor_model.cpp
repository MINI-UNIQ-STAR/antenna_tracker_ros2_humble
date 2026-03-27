#include "antenna_tracker_simulation/sim_motor_model.hpp"

#include <algorithm>
#include <cmath>

namespace antenna_tracker_simulation
{
namespace
{

double deg_to_rad(double degrees)
{
  return degrees * 3.14159265358979323846 / 180.0;
}

double rad_to_deg(double radians)
{
  return radians * 180.0 / 3.14159265358979323846;
}

double clamp_abs(double value, double limit)
{
  return std::clamp(value, -limit, limit);
}

double normalize_azimuth_deg(double degrees)
{
  auto wrapped = std::fmod(degrees, 360.0);
  if (wrapped < 0.0) {
    wrapped += 360.0;
  }
  return wrapped;
}

// Returns actual position displacement after backlash dead zone.
// On direction reversal the output is frozen until the gear backlash is cleared.
double apply_backlash_delta(
  double vel_rps,
  double dt_sec,
  double backlash_limit_rad,
  double & accum_rad,
  int & last_dir)
{
  if (vel_rps == 0.0) {
    return 0.0;
  }
  const int cur_dir = (vel_rps > 0.0) ? 1 : -1;

  if (cur_dir != last_dir) {
    // 방향 전환: 백래시 누적 초기화
    accum_rad = 0.0;
    last_dir = cur_dir;
  }

  if (accum_rad < backlash_limit_rad) {
    // 백래시 구간 — 출력 위치 고정, 누적만 증가
    accum_rad += std::abs(vel_rps) * dt_sec;
    return 0.0;
  }

  // 백래시 소진 — 정상 추종
  return vel_rps * dt_sec;
}

// Returns step error delta (rad) due to torque-limit-induced step loss.
// Elevation axis accounts for gravity load reducing effective torque margin.
double compute_step_loss_delta(
  double requested_accel_rads2,
  double el_pos_rad,
  double rad_per_pulse,
  const SimMotorConfig & config)
{
  const auto threshold_rads2 = deg_to_rad(config.step_loss_accel_threshold_dps2);
  const auto gravity_load_rads2 = config.gravity_coeff_rads2 * std::cos(el_pos_rad);
  // elevation 중력 부하만큼 실질 가용 가속도 감소
  const auto effective_threshold = std::max(0.0, threshold_rads2 - gravity_load_rads2);

  const auto excess = std::abs(requested_accel_rads2) - effective_threshold;
  if (excess <= 0.0) {
    return 0.0;
  }

  const auto overshoot_ratio = excess / threshold_rads2;
  const auto lost_steps = overshoot_ratio * config.step_loss_rate;
  return (requested_accel_rads2 > 0.0 ? 1.0 : -1.0) * lost_steps * rad_per_pulse;
}

double noise_wave(double phase, double primary, double secondary, double offset)
{
  return (std::sin(phase * primary + offset) + 0.5 * std::sin(phase * secondary + offset * 0.5)) / 1.5;
}

}  // namespace

void apply_motor_command(
  const antenna_tracker_msgs::msg::MotorCommand & msg,
  SimMotorState & state)
{
  if (msg.emergency_stop) {
    state.az_command_hz = 0.0;
    state.el_command_hz = 0.0;
    return;
  }

  state.az_command_hz = msg.az_frequency_hz * (msg.az_direction ? 1.0 : -1.0);
  state.el_command_hz = msg.el_frequency_hz * (msg.el_direction ? 1.0 : -1.0);
}

double pulses_per_output_revolution(const SimMotorConfig & config)
{
  return std::max(1.0, config.stepper_pulses_per_rev * config.stepper_microsteps * config.gear_ratio);
}

SimMotorStepResult step_sim_motor_model(
  SimMotorState & state,
  const SimMotorConfig & config)
{
  const auto pulses_per_rev = pulses_per_output_revolution(config);
  const auto rad_per_pulse = kTwoPi / pulses_per_rev;
  const auto max_speed_rps = deg_to_rad(config.motor_max_speed_dps);
  const auto max_accel_rads2 = deg_to_rad(config.motor_max_accel_dps2);
  const auto disturbance_rads2 = deg_to_rad(config.disturbance_dps2);

  state.noise_phase_sec += config.dt_sec;

  auto shaped_noise = [&](double amplitude, double phase_offset, double primary, double secondary) {
    return amplitude * noise_wave(state.noise_phase_sec, primary, secondary, phase_offset);
  };

  auto effective_command_hz = [&](double commanded_hz, double phase_offset) {
    double hz = commanded_hz;
    if (std::abs(hz) < config.command_deadzone_hz) {
      hz = 0.0;
    }
    return hz + shaped_noise(config.command_frequency_noise_hz, phase_offset, 5.7, 9.1);
  };

  const auto az_command_accel = clamp_abs(
    effective_command_hz(state.az_command_hz, 0.2) * rad_per_pulse / config.dt_sec,
    max_accel_rads2);
  const auto el_command_accel = clamp_abs(
    effective_command_hz(state.el_command_hz, 1.1) * rad_per_pulse / config.dt_sec,
    max_accel_rads2);

  state.az_vel_rps += (
    az_command_accel -
    config.az_damping * state.az_vel_rps +
    shaped_noise(disturbance_rads2, 1.7, 3.3, 6.7)) * config.dt_sec;
  state.el_vel_rps += (
    el_command_accel -
    config.gravity_coeff_rads2 * std::cos(state.el_pos_rad) +
    shaped_noise(disturbance_rads2, 2.5, 4.1, 7.9)) * config.dt_sec;

  state.az_vel_rps = clamp_abs(state.az_vel_rps, max_speed_rps);
  state.el_vel_rps = clamp_abs(state.el_vel_rps, max_speed_rps);

  // Step loss: 토크 한계 초과 시 누적 탈조 오차 (피드백에 반영, 내부 위치와 분리)
  // elevation만 중력 부하를 반영 (az는 수직축으로 중력 영향 없음)
  state.az_step_error_rad += compute_step_loss_delta(
    az_command_accel, 0.0, rad_per_pulse, config);
  state.el_step_error_rad += compute_step_loss_delta(
    el_command_accel, state.el_pos_rad, rad_per_pulse, config);

  // Backlash: 방향 전환 시 출력 위치 고정 (데드밴드)
  const auto az_backlash_limit = deg_to_rad(config.az_backlash_deg);
  const auto el_backlash_limit = deg_to_rad(config.el_backlash_deg);

  state.az_pos_rad += apply_backlash_delta(
    state.az_vel_rps, config.dt_sec, az_backlash_limit,
    state.az_backlash_accum_rad, state.az_last_dir);
  state.az_pos_rad = std::fmod(state.az_pos_rad, kTwoPi);
  if (state.az_pos_rad < 0.0) {
    state.az_pos_rad += kTwoPi;
  }

  state.el_pos_rad += apply_backlash_delta(
    state.el_vel_rps, config.dt_sec, el_backlash_limit,
    state.el_backlash_accum_rad, state.el_last_dir);
  state.el_pos_rad = std::clamp(state.el_pos_rad, 0.0, kElevationMaxRadians);
  if ((state.el_pos_rad <= 0.0 && state.el_vel_rps < 0.0) ||
      (state.el_pos_rad >= kElevationMaxRadians && state.el_vel_rps > 0.0))
  {
    state.el_vel_rps = 0.0;
  }

  SimMotorStepResult result;
  result.az_velocity_command_rps = state.az_vel_rps;
  result.el_velocity_command_rps = state.el_vel_rps;

  // 피드백: 실제 위치 + 탈조 누적 오차 + 센서 노이즈
  result.feedback.az_angle_deg = normalize_azimuth_deg(
    rad_to_deg(state.az_pos_rad + state.az_step_error_rad) +
    shaped_noise(config.encoder_angle_noise_deg, 0.6, 8.3, 13.7));
  result.feedback.el_angle_deg = std::clamp(
    rad_to_deg(state.el_pos_rad + state.el_step_error_rad) +
    shaped_noise(config.encoder_angle_noise_deg, 1.4, 10.1, 14.9),
    0.0,
    90.0);
  result.feedback.az_velocity_dps = rad_to_deg(state.az_vel_rps) +
    shaped_noise(config.encoder_velocity_noise_dps, 2.2, 11.3, 15.1);
  result.feedback.el_velocity_dps = rad_to_deg(state.el_vel_rps) +
    shaped_noise(config.encoder_velocity_noise_dps, 3.1, 12.7, 16.9);
  result.feedback.az_valid = true;
  result.feedback.el_valid = true;

  return result;
}

}  // namespace antenna_tracker_simulation
