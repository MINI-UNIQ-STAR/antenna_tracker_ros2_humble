#include <gtest/gtest.h>
#include "antenna_tracker_controller/pid_controller.hpp"
#include <cmath>

using namespace antenna_tracker_controller;

/* === CascadePid ========================================================= */

TEST(CascadePidTest, DefaultConstructorNocrash) {
  CascadePid pid;
  (void)pid;
}

TEST(CascadePidTest, ClampLogic) {
  PidGains outer{1.0, 0.0, 0.0};
  PidGains inner{1.0, 0.0, 0.0};
  CascadePid pid;
  pid.init(outer, inner, 0.01, -10.0, 10.0);
  double out = pid.compute(100.0, 0.0, 0.0);
  EXPECT_LE(out, 10.0);
}

TEST(CascadePidTest, ClampMin) {
  PidGains outer{1.0, 0.0, 0.0};
  PidGains inner{1.0, 0.0, 0.0};
  CascadePid pid;
  pid.init(outer, inner, 0.01, -10.0, 10.0);
  double out = pid.compute(-100.0, 0.0, 0.0);
  EXPECT_GE(out, -10.0);
}

TEST(CascadePidTest, DtZeroGuard) {
  PidGains outer{1.0, 0.1, 0.1};
  PidGains inner{1.0, 0.1, 0.1};
  CascadePid pid;
  pid.init(outer, inner, 0.0, -1000.0, 1000.0);
  EXPECT_DOUBLE_EQ(pid.compute(10.0, 0.0, 0.0), 0.0);
}

TEST(CascadePidTest, DtNegativeGuard) {
  PidGains outer{1.0, 0.1, 0.1};
  PidGains inner{1.0, 0.1, 0.1};
  CascadePid pid;
  pid.init(outer, inner, -0.01, -1000.0, 1000.0);
  EXPECT_DOUBLE_EQ(pid.compute(10.0, 0.0, 0.0), 0.0);
}

TEST(CascadePidTest, PositiveErrorPositiveOutput) {
  PidGains outer{2.0, 0.0, 0.0};
  PidGains inner{1.0, 0.0, 0.0};
  CascadePid pid;
  pid.init(outer, inner, 0.01, -1000.0, 1000.0);
  EXPECT_GT(pid.compute(10.0, 0.0, 0.0), 0.0);
}

TEST(CascadePidTest, NegativeErrorNegativeOutput) {
  PidGains outer{2.0, 0.0, 0.0};
  PidGains inner{1.0, 0.0, 0.0};
  CascadePid pid;
  pid.init(outer, inner, 0.01, -1000.0, 1000.0);
  EXPECT_LT(pid.compute(-10.0, 0.0, 0.0), 0.0);
}

TEST(CascadePidTest, ResetClearsIntegral) {
  PidGains outer{1.0, 1.0, 0.0};
  PidGains inner{1.0, 1.0, 0.0};
  CascadePid pid;
  pid.init(outer, inner, 0.01, -10000.0, 10000.0);

  // Build up integral
  for (int i = 0; i < 100; ++i) pid.compute(10.0, 0.0, 0.0);
  double before_reset = pid.compute(10.0, 0.0, 0.0);

  pid.reset();
  double after_reset = pid.compute(10.0, 0.0, 0.0);

  // After reset, integral contribution is gone → output magnitude smaller
  EXPECT_LT(std::abs(after_reset), std::abs(before_reset));
}

TEST(CascadePidTest, IntegralWindupClamped) {
  PidGains outer{1.0, 100.0, 0.0};
  PidGains inner{1.0, 100.0, 0.0};
  CascadePid pid;
  pid.init(outer, inner, 0.01, -1000.0, 1000.0);
  for (int i = 0; i < 10000; ++i) pid.compute(10.0, 0.0, 0.0);
  double out = pid.compute(10.0, 0.0, 0.0);
  EXPECT_FALSE(std::isinf(out));
  EXPECT_FALSE(std::isnan(out));
  EXPECT_LE(out, 1000.0);
}

TEST(CascadePidTest, DerivativeOnMeasurementKickFree) {
  // Derivative on measurement: sudden setpoint change must NOT cause derivative kick
  PidGains outer{0.0, 0.0, 10.0};  // kd only outer loop
  PidGains inner{0.0, 0.0, 0.0};
  CascadePid pid;
  pid.init(outer, inner, 0.01, -1e6, 1e6);

  // Establish baseline with zero error
  pid.compute(0.0, 0.0, 0.0);

  // Setpoint jumps to 100 — measurement stays 0; with DoM only measurement change drives derivative
  // position unchanged (0→0) so derivative term is 0 even though error jumped
  double out_setpoint_jump = pid.compute(100.0, 0.0, 0.0);

  // Derivative = -(0 - 0)/0.01 = 0, so output = kd*0 = 0 (no kick)
  EXPECT_NEAR(out_setpoint_jump, 0.0, 1e-9);
}

TEST(CascadePidTest, MeasurementChangeTriggersDerivative) {
  PidGains outer{0.0, 0.0, 1.0};  // only kd
  PidGains inner{0.0, 0.0, 0.0};
  CascadePid pid;
  pid.init(outer, inner, 0.01, -1e6, 1e6);

  pid.compute(0.0, 0.0, 0.0);

  // Measurement changes from 0 to 5 → derivative = -(5-0)/0.01 = -500
  double out = pid.compute(0.0, 5.0, 0.0);
  EXPECT_NE(out, 0.0);
}

TEST(CascadePidTest, VelocityFeedforwardHelps) {
  PidGains outer{1.0, 0.0, 0.0};
  PidGains inner{1.0, 0.0, 0.0};
  CascadePid pid;
  pid.init(outer, inner, 0.01, -1000.0, 1000.0);

  // With a non-zero velocity_current that opposes the vel_setpoint, output is smaller
  double out_no_vel  = pid.compute(10.0, 0.0, 0.0);
  double out_with_vel = pid.compute(10.0, 0.0, 5.0);  // vel_current already at 5
  // vel_setpoint = outer_kp * 10 = 10, vel_error = 10 - 5 = 5 < 10
  EXPECT_LT(out_with_vel, out_no_vel);
}

/* === DualAxisCascadePid ================================================= */

TEST(DualAxisCascadePidTest, Initialization) {
  DualAxisCascadePid pid;
  pid.init(0.01);

  double az_out = 0.0, el_out = 0.0;
  pid.compute(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, az_out, el_out);
  EXPECT_DOUBLE_EQ(az_out, 0.0);
  /* Gravity compensation removed from PID (NMPC handles gravity via double-integrator model).
   * With zero error and zero state, output is 0. */
  EXPECT_DOUBLE_EQ(el_out, 0.0);
}

TEST(DualAxisCascadePidTest, BothAxesRespond) {
  DualAxisCascadePid pid;
  pid.init(0.01);

  double az_out = 0.0, el_out = 0.0;
  pid.compute(90.0, 0.0, 0.0, 45.0, 0.0, 0.0, az_out, el_out);
  EXPECT_NE(az_out, 0.0);
  EXPECT_NE(el_out, 0.0);
}

TEST(DualAxisCascadePidTest, Reset) {
  DualAxisCascadePid pid;
  pid.init(0.01);

  double az_out, el_out;
  for (int i = 0; i < 100; ++i) {
    pid.compute(90.0, 0.0, 0.0, 45.0, 0.0, 0.0, az_out, el_out);
  }
  double before_az = az_out;

  pid.reset();
  pid.compute(90.0, 0.0, 0.0, 45.0, 0.0, 0.0, az_out, el_out);

  // After reset integral is cleared, output is smaller than after 100 steps with integral
  EXPECT_LE(std::abs(az_out), std::abs(before_az));
  EXPECT_FALSE(std::isnan(az_out));
  EXPECT_FALSE(std::isnan(el_out));
}

TEST(DualAxisCascadePidTest, SetAzPositionGains) {
  DualAxisCascadePid pid;
  pid.init(0.01);
  PidGains gains{20.0, 0.5, 0.5};
  pid.set_az_position_gains(gains);
  double az_out, el_out;
  pid.compute(10.0, 0.0, 0.0, 0.0, 0.0, 0.0, az_out, el_out);
  EXPECT_NE(az_out, 0.0);
}

TEST(DualAxisCascadePidTest, SetAzVelocityGains) {
  DualAxisCascadePid pid;
  pid.init(0.01);
  PidGains gains{10.0, 0.1, 0.1};
  pid.set_az_velocity_gains(gains);
  double az_out, el_out;
  pid.compute(10.0, 0.0, 0.0, 0.0, 0.0, 0.0, az_out, el_out);
  EXPECT_NE(az_out, 0.0);
}

TEST(DualAxisCascadePidTest, SetElPositionGains) {
  DualAxisCascadePid pid;
  pid.init(0.01);
  PidGains gains{10.0, 1.5, 0.3};
  pid.set_el_position_gains(gains);
  double az_out, el_out;
  pid.compute(0.0, 0.0, 0.0, 45.0, 0.0, 0.0, az_out, el_out);
  EXPECT_NE(el_out, 0.0);
}

TEST(DualAxisCascadePidTest, SetElVelocityGains) {
  DualAxisCascadePid pid;
  pid.init(0.01);
  PidGains gains{8.0, 0.2, 0.1};
  pid.set_el_velocity_gains(gains);
  double az_out, el_out;
  pid.compute(0.0, 0.0, 0.0, 45.0, 0.0, 0.0, az_out, el_out);
  EXPECT_NE(el_out, 0.0);
}

TEST(DualAxisCascadePidTest, NaNFreeAfterManyIterations) {
  DualAxisCascadePid pid;
  pid.init(0.01);
  double az_out, el_out;
  for (int i = 0; i < 500; ++i) {
    pid.compute(90.0, 45.0, 1.0, 45.0, 22.0, 0.5, az_out, el_out);
  }
  EXPECT_FALSE(std::isnan(az_out));
  EXPECT_FALSE(std::isnan(el_out));
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
