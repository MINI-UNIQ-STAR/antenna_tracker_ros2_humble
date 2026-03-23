#ifndef ANTENNA_TRACKER_CONTROLLER__KALMAN_FILTER_HPP_
#define ANTENNA_TRACKER_CONTROLLER__KALMAN_FILTER_HPP_

#include <array>

namespace antenna_tracker_controller
{

class KalmanFilterAzEl
{
public:
  KalmanFilterAzEl();

  void init(double dt, double q_process = 0.001, double r_measurement = 2.0);
  void update(double az_meas, double el_meas);

  double azimuth() const { return state_[0]; }
  double az_velocity() const { return state_[1]; }
  double elevation() const { return state_[2]; }
  double el_velocity() const { return state_[3]; }

private:
  /* State: [az, az_vel, el, el_vel] */
  std::array<double, 4> state_;
  /* Diagonal covariance (simplified) */
  std::array<std::array<double, 4>, 4> P_;
  double Q_;
  double Q_pos_;
  double Q_vel_;
  double R_;
  double dt_;
};

}  // namespace antenna_tracker_controller

#endif  // ANTENNA_TRACKER_CONTROLLER__KALMAN_FILTER_HPP_
