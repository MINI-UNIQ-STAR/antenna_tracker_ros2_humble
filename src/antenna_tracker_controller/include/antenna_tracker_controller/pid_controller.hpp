#ifndef ANTENNA_TRACKER_CONTROLLER__PID_CONTROLLER_HPP_
#define ANTENNA_TRACKER_CONTROLLER__PID_CONTROLLER_HPP_

namespace antenna_tracker_controller
{

struct PidGains {
  double kp{0.0};
  double ki{0.0};
  double kd{0.0};
};

class CascadePid
{
public:
  CascadePid();

  void init(
    const PidGains & outer, const PidGains & inner,
    double dt, double output_min, double output_max);

  double compute(double position_target, double position_current, double velocity_current);
  void reset();

private:
  PidGains outer_;
  PidGains inner_;

  double outer_integral_;
  double outer_prev_error_;
  double outer_prev_measurement_;
  double inner_integral_;
  double inner_prev_error_;
  double inner_prev_measurement_;

  double output_min_;
  double output_max_;
  double dt_;

  static double clamp(double value, double min_val, double max_val);
};

class DualAxisCascadePid
{
public:
  DualAxisCascadePid();

  /// Initialize with default gains (backward-compatible; useful for tests).
  void init(double dt);

  /// Initialize with explicit gains loaded from configuration (preferred).
  void init(double dt,
            PidGains az_outer, PidGains az_inner,
            PidGains el_outer, PidGains el_inner,
            double az_out_min = -800.0, double az_out_max = 800.0,
            double el_out_min = -800.0, double el_out_max = 800.0);

  void compute(
    double az_target, double az_current, double az_vel,
    double el_target, double el_current, double el_vel,
    double & az_output, double & el_output);
  void reset();

  void set_az_position_gains(const PidGains & gains);
  void set_az_velocity_gains(const PidGains & gains);
  void set_el_position_gains(const PidGains & gains);
  void set_el_velocity_gains(const PidGains & gains);

private:
  CascadePid azimuth_;
  CascadePid elevation_;
  double dt_;
  double az_out_min_{-800.0};
  double az_out_max_{ 800.0};
  double el_out_min_{-800.0};
  double el_out_max_{ 800.0};
  // Stored gains — used by set_*_gains() to preserve the companion loop
  PidGains az_outer_gains_;
  PidGains az_inner_gains_;
  PidGains el_outer_gains_;
  PidGains el_inner_gains_;
};

}  // namespace antenna_tracker_controller

#endif  // ANTENNA_TRACKER_CONTROLLER__PID_CONTROLLER_HPP_
