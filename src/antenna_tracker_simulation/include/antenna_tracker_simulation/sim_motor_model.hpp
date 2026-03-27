#ifndef ANTENNA_TRACKER_SIMULATION__SIM_MOTOR_MODEL_HPP_
#define ANTENNA_TRACKER_SIMULATION__SIM_MOTOR_MODEL_HPP_

#include <antenna_tracker_msgs/msg/encoder_feedback.hpp>
#include <antenna_tracker_msgs/msg/motor_command.hpp>

namespace antenna_tracker_simulation
{

inline constexpr double kTwoPi = 2.0 * 3.14159265358979323846;
inline constexpr double kDefaultStepSeconds = 0.01;
inline constexpr double kElevationMaxRadians = 3.14159265358979323846 / 2.0;

struct SimMotorConfig
{
  double dt_sec{kDefaultStepSeconds};
  double stepper_pulses_per_rev{200.0};
  double stepper_microsteps{1.0};
  double gear_ratio{20.0};
  double motor_max_speed_dps{180.0};
  double motor_max_accel_dps2{1800.0};   // NEMA23 1.2Nm + 20:1 + 야기 500g@0.9m, 안전계수 0.8
  double command_deadzone_hz{18.0};
  double disturbance_dps2{0.25};
  double command_frequency_noise_hz{0.0};
  double encoder_angle_noise_deg{0.0};
  double encoder_velocity_noise_dps{0.0};
  double gravity_coeff_rads2{10.77};     // 4.42 Nm / 0.41 kg·m² (실측 기반)
  double az_damping{10.0};

  // 실측 하드웨어 물리 파라미터 (참조용)
  double load_mass_kg{0.5};
  double arm_length_m{0.9};
  double load_inertia_kgm2{0.41};
  double motor_holding_torque_nm{1.2};
  double gearbox_efficiency{0.85};

  // Backlash (기어 백래시)
  double az_backlash_deg{0.8};   // 20:1 기어박스 출력축 기준
  double el_backlash_deg{0.5};   // elevation은 단방향 중력 하중으로 백래시 작음

  // Step Loss (스텝 탈조)
  double step_loss_accel_threshold_dps2{2232.0};  // 토크 한계 도달 가속도
  double step_loss_rate{0.002};                    // 초과분 당 탈조 비율
};

struct SimMotorState
{
  double az_pos_rad{0.0};
  double el_pos_rad{0.0};
  double az_vel_rps{0.0};
  double el_vel_rps{0.0};
  double az_command_hz{0.0};
  double el_command_hz{0.0};
  double noise_phase_sec{0.0};

  // Backlash tracking
  double az_backlash_accum_rad{0.0};  // 현재 백래시 누적량 (항상 양수, 방향은 az_last_dir로 구분)
  double el_backlash_accum_rad{0.0};
  int    az_last_dir{0};              // 마지막 이동 방향: +1, -1, 0(정지)
  int    el_last_dir{0};

  // Step loss tracking
  double az_step_error_rad{0.0};      // 누적 탈조 오차 (피드백에 반영)
  double el_step_error_rad{0.0};
};

struct SimMotorStepResult
{
  double az_velocity_command_rps{0.0};
  double el_velocity_command_rps{0.0};
  antenna_tracker_msgs::msg::EncoderFeedback feedback;
};

void apply_motor_command(
  const antenna_tracker_msgs::msg::MotorCommand & msg,
  SimMotorState & state);

double pulses_per_output_revolution(const SimMotorConfig & config);

SimMotorStepResult step_sim_motor_model(
  SimMotorState & state,
  const SimMotorConfig & config);

}  // namespace antenna_tracker_simulation

#endif  // ANTENNA_TRACKER_SIMULATION__SIM_MOTOR_MODEL_HPP_
