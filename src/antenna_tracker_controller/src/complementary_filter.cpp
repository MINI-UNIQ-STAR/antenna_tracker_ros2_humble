#include "antenna_tracker_controller/complementary_filter.hpp"
#include <cmath>

namespace antenna_tracker_controller
{

static constexpr double RAD_TO_DEG = 180.0 / M_PI;
static constexpr double DEG_TO_RAD = M_PI / 180.0;

ComplementaryFilter::ComplementaryFilter()
: alpha_(0.98),
  declination_(0.0),
  last_timestamp_sec_(0.0),
  initialized_(false)
{
}

void ComplementaryFilter::set_alpha(double alpha)
{
  /* Clamp to [0, 1]: values outside invert the filter blend */
  alpha_ = (alpha < 0.0) ? 0.0 : (alpha > 1.0) ? 1.0 : alpha;
}

void ComplementaryFilter::set_declination(double declination_deg)
{
  declination_ = declination_deg;
}

void ComplementaryFilter::update(
  double accel_x, double accel_y, double accel_z,
  double gyro_x, double gyro_y, double gyro_z,
  double mag_x, double mag_y, double mag_z,
  double timestamp_sec)
{
  double dt = 0.01;
  if (initialized_ && timestamp_sec > last_timestamp_sec_) {
    dt = timestamp_sec - last_timestamp_sec_;
    /* Clamp dt to avoid gyro-integration explosion on large clock gaps */
    if (dt > 0.1) { dt = 0.1; }
  }
  last_timestamp_sec_ = timestamp_sec;
  initialized_ = true;

  /* Roll & Pitch from accelerometer */
  double accel_roll = std::atan2(accel_y, accel_z) * RAD_TO_DEG;
  double accel_pitch = std::atan2(
    -accel_x, std::sqrt(accel_y * accel_y + accel_z * accel_z)) * RAD_TO_DEG;

  /* Gyro integration (gyro in deg/s) */
  double gyro_roll = orientation_.roll + gyro_x * dt;
  double gyro_pitch = orientation_.pitch + gyro_y * dt;
  double gyro_yaw = orientation_.yaw + gyro_z * dt;

  /* Complementary filter: 98% gyro, 2% accel */
  orientation_.roll = alpha_ * gyro_roll + (1.0 - alpha_) * accel_roll;
  orientation_.pitch = alpha_ * gyro_pitch + (1.0 - alpha_) * accel_pitch;
  /* yaw is fused with magnetometer below; do not assign pure gyro_yaw here */

  /* Tilt-compensated magnetometer heading (roll + pitch 보상) */
  double roll_rad  = orientation_.roll  * DEG_TO_RAD;
  double pitch_rad = orientation_.pitch * DEG_TO_RAD;
  double mx = mag_x * std::cos(pitch_rad)
            + mag_y * std::sin(roll_rad) * std::sin(pitch_rad)
            + mag_z * std::cos(roll_rad) * std::sin(pitch_rad);
  double my = mag_y * std::cos(roll_rad) - mag_z * std::sin(roll_rad);

  /* Guard: skip heading update when magnetometer has no signal (mx==my==0).
   * atan2(0,0) is implementation-defined; retain previous azimuth instead. */
  if (mx != 0.0 || my != 0.0) {
    double heading = std::atan2(my, mx) * RAD_TO_DEG;
    heading += declination_;
    heading = std::fmod(heading, 360.0);
    if (heading < 0.0) heading += 360.0;
    orientation_.azimuth = heading;
  }

  /* Fuse gyro yaw with magnetometer heading (complementary blend) */
  double yaw_diff = heading - gyro_yaw;
  while (yaw_diff >  180.0) yaw_diff -= 360.0;
  while (yaw_diff < -180.0) yaw_diff += 360.0;
  orientation_.yaw = gyro_yaw + (1.0 - alpha_) * yaw_diff;

  orientation_.elevation = orientation_.pitch;
}

}  // namespace antenna_tracker_controller
