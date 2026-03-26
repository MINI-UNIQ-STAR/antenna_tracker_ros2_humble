#include "antenna_tracker_controller/kalman_filter.hpp"
#include <cmath>
#include <cstring>

namespace
{

double normalize_angle_deg(double angle_deg)
{
  angle_deg = std::fmod(angle_deg, 360.0);
  if (angle_deg < 0.0) {
    angle_deg += 360.0;
  }
  return angle_deg;
}

}  // namespace

namespace antenna_tracker_controller
{

KalmanFilterAzEl::KalmanFilterAzEl()
: Q_(0.001), Q_pos_(0.001), Q_vel_(0.01), R_(2.0), dt_(0.01)
{
  state_.fill(0.0);
  for (auto & row : P_) {
    row.fill(0.0);
  }
  for (int i = 0; i < 4; i++) {
    P_[i][i] = 1.0;
  }
}

void KalmanFilterAzEl::init(double dt, double q_process, double r_measurement)
{
  dt_ = dt;
  Q_ = q_process;
  Q_pos_ = Q_;
  Q_vel_ = Q_ * 10.0;
  /* R_ must be > 0 to prevent division by zero in S_az = P_[0][0] + R_ */
  R_ = (r_measurement > 0.0) ? r_measurement : 1e-6;

  state_.fill(0.0);
  for (auto & row : P_) {
    row.fill(0.0);
  }
  for (int i = 0; i < 4; i++) {
    P_[i][i] = 1.0;
  }
}

void KalmanFilterAzEl::update(double az_meas, double el_meas)
{
  az_meas = normalize_angle_deg(az_meas);

  /* Predict: state transition [az, az_vel, el, el_vel] */
  std::array<double, 4> pred;
  pred[0] = state_[0] + state_[1] * dt_;  /* az = az + az_vel * dt */
  pred[1] = state_[1];                    /* az_vel constant */
  pred[2] = state_[2] + state_[3] * dt_;  /* el = el + el_vel * dt */
  pred[3] = state_[3];                    /* el_vel constant */
  pred[0] = normalize_angle_deg(pred[0]);

  /* K-C1: Full covariance prediction using F*P*F^T + Q for constant-velocity model */
  /* F = [[1,dt,0,0],[0,1,0,0],[0,0,1,dt],[0,0,0,1]] */
  /* Azimuth block [0][0]-[1][1] */
  double new_p00 = P_[0][0] + dt_ * (P_[0][1] + P_[1][0]) + dt_ * dt_ * P_[1][1] + Q_pos_;
  double new_p01 = P_[0][1] + dt_ * P_[1][1];
  double new_p10 = P_[1][0] + dt_ * P_[1][1];
  double new_p11 = P_[1][1] + Q_vel_;
  P_[0][0] = new_p00; P_[0][1] = new_p01;
  P_[1][0] = new_p10; P_[1][1] = new_p11;
  /* Elevation block [2][2]-[3][3] */
  double new_p22 = P_[2][2] + dt_ * (P_[2][3] + P_[3][2]) + dt_ * dt_ * P_[3][3] + Q_pos_;
  double new_p23 = P_[2][3] + dt_ * P_[3][3];
  double new_p32 = P_[3][2] + dt_ * P_[3][3];
  double new_p33 = P_[3][3] + Q_vel_;
  P_[2][2] = new_p22; P_[2][3] = new_p23;
  P_[3][2] = new_p32; P_[3][3] = new_p33;

  /* K-C2: Proper 2x2 block update with off-diagonal Kalman gain, H=[1,0] */
  /* Azimuth update */
  double S_az = P_[0][0] + R_;
  double K_az0 = P_[0][0] / S_az;
  double K_az1 = P_[1][0] / S_az;
  double az_innovation = az_meas - pred[0];
  while (az_innovation >  180.0) az_innovation -= 360.0;
  while (az_innovation < -180.0) az_innovation += 360.0;
  pred[0] += K_az0 * az_innovation;
  pred[1] += K_az1 * az_innovation;
  /* Joseph-form covariance update for az block, (I - K*H)*P, H=[1,0] */
  double p00 = (1.0 - K_az0) * P_[0][0];
  double p01 = (1.0 - K_az0) * P_[0][1];
  double p10 = -K_az1 * P_[0][0] + P_[1][0];
  double p11 = -K_az1 * P_[0][1] + P_[1][1];
  P_[0][0] = p00; P_[0][1] = p01; P_[1][0] = p10; P_[1][1] = p11;

  /* Elevation update */
  double S_el = P_[2][2] + R_;
  double K_el0 = P_[2][2] / S_el;
  double K_el1 = P_[3][2] / S_el;
  double el_innovation = el_meas - pred[2];
  pred[2] += K_el0 * el_innovation;
  pred[3] += K_el1 * el_innovation;
  /* Joseph-form covariance update for el block */
  double p22 = (1.0 - K_el0) * P_[2][2];
  double p23 = (1.0 - K_el0) * P_[2][3];
  double p32 = -K_el1 * P_[2][2] + P_[3][2];
  double p33 = -K_el1 * P_[2][3] + P_[3][3];
  P_[2][2] = p22; P_[2][3] = p23; P_[3][2] = p32; P_[3][3] = p33;

  state_ = pred;
  state_[0] = normalize_angle_deg(state_[0]);
}

}  // namespace antenna_tracker_controller
