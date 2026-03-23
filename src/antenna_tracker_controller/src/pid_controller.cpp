#include "antenna_tracker_controller/pid_controller.hpp"
#include <algorithm>
#include <cmath>

namespace antenna_tracker_controller
{

/* CascadePid */

CascadePid::CascadePid()
: outer_integral_(0.0),
  outer_prev_error_(0.0),
  outer_prev_measurement_(0.0),
  inner_integral_(0.0),
  inner_prev_error_(0.0),
  inner_prev_measurement_(0.0),
  output_min_(-1000.0),
  output_max_(1000.0),
  dt_(0.01)
{
}

void CascadePid::init(
  const PidGains & outer, const PidGains & inner,
  double dt, double output_min, double output_max)
{
  outer_ = outer;
  inner_ = inner;
  dt_ = dt;
  output_min_ = output_min;
  output_max_ = output_max;
  reset();
}

double CascadePid::compute(
  double position_target, double position_current, double velocity_current)
{
  if (dt_ <= 0.0) return 0.0;

  /* Outer loop: position -> velocity setpoint */
  double pos_error = position_target - position_current;
  outer_integral_ += pos_error * dt_;
  outer_integral_ = clamp(outer_integral_, -100.0, 100.0);
  /* Derivative on measurement: avoids kick on setpoint change */
  double pos_derivative = -(position_current - outer_prev_measurement_) / dt_;
  outer_prev_measurement_ = position_current;
  outer_prev_error_ = pos_error;

  double velocity_setpoint =
    outer_.kp * pos_error +
    outer_.ki * outer_integral_ +
    outer_.kd * pos_derivative;

  /* Inner loop: velocity -> motor command */
  double vel_error = velocity_setpoint - velocity_current;
  inner_integral_ += vel_error * dt_;
  inner_integral_ = clamp(inner_integral_, -500.0, 500.0);
  /* Derivative on measurement: avoids kick on velocity setpoint change */
  double vel_derivative = -(velocity_current - inner_prev_measurement_) / dt_;
  inner_prev_measurement_ = velocity_current;
  inner_prev_error_ = vel_error;

  double output =
    inner_.kp * vel_error +
    inner_.ki * inner_integral_ +
    inner_.kd * vel_derivative;

  return clamp(output, output_min_, output_max_);
}

void CascadePid::reset()
{
  outer_integral_ = 0.0;
  outer_prev_error_ = 0.0;
  outer_prev_measurement_ = 0.0;
  inner_integral_ = 0.0;
  inner_prev_error_ = 0.0;
  inner_prev_measurement_ = 0.0;
}

double CascadePid::clamp(double value, double min_val, double max_val)
{
  return std::max(min_val, std::min(max_val, value));
}

/* DualAxisCascadePid */

DualAxisCascadePid::DualAxisCascadePid()
: dt_(0.01)
{
}

void DualAxisCascadePid::init(double dt)
{
  dt_ = dt;

  /* Azimuth — tuned for open-loop stepper integration model */
  PidGains az_outer{5.0, 0.0, 0.1};
  PidGains az_inner{1.5, 0.01, 0.05};
  azimuth_.init(az_outer, az_inner, dt, -800.0, 800.0);

  /* Elevation — lower gains, no integral windup at limits */
  PidGains el_outer{3.0, 0.0, 0.1};
  PidGains el_inner{2.0, 0.01, 0.02};
  elevation_.init(el_outer, el_inner, dt, -800.0, 800.0);
}

void DualAxisCascadePid::compute(
  double az_target, double az_current, double az_vel,
  double el_target, double el_current, double el_vel,
  double & az_output, double & el_output)
{
  az_output = azimuth_.compute(az_target, az_current, az_vel);
  el_output = elevation_.compute(el_target, el_current, el_vel);
}

void DualAxisCascadePid::reset()
{
  azimuth_.reset();
  elevation_.reset();
}

void DualAxisCascadePid::set_az_position_gains(const PidGains & gains)
{
  PidGains inner{4.0, 0.05, 0.1};
  azimuth_.init(gains, inner, dt_, -1000.0, 1000.0);
}

void DualAxisCascadePid::set_az_velocity_gains(const PidGains & gains)
{
  PidGains outer{15.0, 0.1, 0.3};
  azimuth_.init(outer, gains, dt_, -1000.0, 1000.0);
}

void DualAxisCascadePid::set_el_position_gains(const PidGains & gains)
{
  PidGains inner{5.0, 0.15, 0.08};
  elevation_.init(gains, inner, dt_, -1000.0, 1000.0);
}

void DualAxisCascadePid::set_el_velocity_gains(const PidGains & gains)
{
  PidGains outer{8.0, 1.0, 0.2};
  elevation_.init(outer, gains, dt_, -1000.0, 1000.0);
}

}  // namespace antenna_tracker_controller
