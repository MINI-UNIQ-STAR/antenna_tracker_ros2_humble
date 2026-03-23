#include <gtest/gtest.h>
#include "antenna_tracker_controller/kalman_filter.hpp"
#include <cmath>

using namespace antenna_tracker_controller;

/* --- Constructor / init ------------------------------------------------- */

TEST(KalmanFilterTest, DefaultConstructorZeroState) {
  KalmanFilterAzEl filter;
  EXPECT_DOUBLE_EQ(filter.azimuth(),     0.0);
  EXPECT_DOUBLE_EQ(filter.az_velocity(), 0.0);
  EXPECT_DOUBLE_EQ(filter.elevation(),   0.0);
  EXPECT_DOUBLE_EQ(filter.el_velocity(), 0.0);
}

TEST(KalmanFilterTest, Initialization) {
  KalmanFilterAzEl filter;
  filter.init(0.01, 0.001, 2.0);
  EXPECT_DOUBLE_EQ(filter.azimuth(),     0.0);
  EXPECT_DOUBLE_EQ(filter.az_velocity(), 0.0);
  EXPECT_DOUBLE_EQ(filter.elevation(),   0.0);
  EXPECT_DOUBLE_EQ(filter.el_velocity(), 0.0);
}

TEST(KalmanFilterTest, InitWithCustomParams) {
  KalmanFilterAzEl filter;
  filter.init(0.05, 0.01, 5.0);  // different dt, q, r
  EXPECT_DOUBLE_EQ(filter.azimuth(), 0.0);
}

TEST(KalmanFilterTest, InitResetsAfterUpdates) {
  KalmanFilterAzEl filter;
  filter.init(0.01, 0.001, 2.0);
  for (int i = 0; i < 20; ++i) filter.update(90.0, 45.0);
  // Re-init must reset to zero
  filter.init(0.01, 0.001, 2.0);
  EXPECT_DOUBLE_EQ(filter.azimuth(),   0.0);
  EXPECT_DOUBLE_EQ(filter.elevation(), 0.0);
}

/* --- Single update ------------------------------------------------------ */

TEST(KalmanFilterTest, SingleUpdateMovesTowardMeasurement) {
  KalmanFilterAzEl filter;
  filter.init(0.01, 0.001, 2.0);
  filter.update(30.0, 20.0);
  EXPECT_GT(filter.azimuth(),   0.0);
  EXPECT_GT(filter.elevation(), 0.0);
}

/* --- Convergence -------------------------------------------------------- */

TEST(KalmanFilterTest, UpdateConvergence) {
  KalmanFilterAzEl filter;
  filter.init(0.01, 0.001, 2.0);
  for (int i = 0; i < 100; ++i) filter.update(90.0, 45.0);
  EXPECT_NEAR(filter.azimuth(),   90.0, 2.0);
  EXPECT_NEAR(filter.elevation(), 45.0, 2.0);
}

TEST(KalmanFilterTest, ConvergenceRemainsBounded) {
  KalmanFilterAzEl filter;
  filter.init(0.01, 0.001, 2.0);
  for (int i = 0; i < 300; ++i) filter.update(90.0, 45.0);
  EXPECT_FALSE(std::isnan(filter.azimuth()));
  EXPECT_FALSE(std::isinf(filter.azimuth()));
  EXPECT_NEAR(filter.azimuth(),   90.0, 1.0);
  EXPECT_NEAR(filter.elevation(), 45.0, 1.0);
}

/* --- Velocity estimation ------------------------------------------------ */

TEST(KalmanFilterTest, AzVelocityPositive) {
  KalmanFilterAzEl filter;
  filter.init(0.1, 0.01, 1.0);
  for (int i = 0; i < 50; ++i) filter.update(i * 1.0, 30.0);
  EXPECT_GT(filter.az_velocity(), 0.0);
}

TEST(KalmanFilterTest, ElVelocityPositive) {
  KalmanFilterAzEl filter;
  filter.init(0.1, 0.01, 1.0);
  for (int i = 0; i < 50; ++i) filter.update(0.0, i * 0.5);
  EXPECT_GT(filter.el_velocity(), 0.0);
}

/* --- Az innovation wrap-around ----------------------------------------- */

// State converged to ~1°; measurement 359° → innovation = 358 → wraps to -2°
// Azimuth should stay near 1, not jump to 359
TEST(KalmanFilterTest, AzInnovationWrapNegative) {
  KalmanFilterAzEl filter;
  filter.init(0.01, 0.001, 0.01);  // low R: fast convergence
  for (int i = 0; i < 200; ++i) filter.update(1.0, 30.0);
  EXPECT_NEAR(filter.azimuth(), 1.0, 1.0);
  filter.update(359.0, 30.0);
  // With correct wrap: innovation = -2°  → azimuth slightly decreases, stays near 1
  EXPECT_LT(filter.azimuth(), 10.0);
}

// State converged to ~359°; measurement 1° → innovation = -358 → wraps to +2°
TEST(KalmanFilterTest, AzInnovationWrapPositive) {
  KalmanFilterAzEl filter;
  filter.init(0.01, 0.001, 0.01);
  for (int i = 0; i < 200; ++i) filter.update(359.0, 30.0);
  EXPECT_NEAR(filter.azimuth(), 359.0, 1.0);
  filter.update(1.0, 30.0);
  EXPECT_GT(filter.azimuth(), 340.0);
}

/* --- Covariance stability (no NaN/Inf after many steps) ---------------- */

TEST(KalmanFilterTest, NaNFreeAfterManyUpdates) {
  KalmanFilterAzEl filter;
  filter.init(0.01, 0.001, 2.0);
  for (int i = 0; i < 500; ++i) filter.update(90.0, 45.0);
  EXPECT_FALSE(std::isnan(filter.azimuth()));
  EXPECT_FALSE(std::isnan(filter.az_velocity()));
  EXPECT_FALSE(std::isnan(filter.elevation()));
  EXPECT_FALSE(std::isnan(filter.el_velocity()));
}

/* --- Extreme / boundary measurements ----------------------------------- */

TEST(KalmanFilterTest, ZeroMeasurements) {
  KalmanFilterAzEl filter;
  filter.init(0.01, 0.001, 2.0);
  for (int i = 0; i < 50; ++i) filter.update(0.0, 0.0);
  EXPECT_NEAR(filter.azimuth(),   0.0, 0.5);
  EXPECT_NEAR(filter.elevation(), 0.0, 0.5);
}

TEST(KalmanFilterTest, FullRangeTransition) {
  KalmanFilterAzEl filter;
  filter.init(0.01, 0.001, 2.0);
  for (int i = 0; i < 50; ++i) filter.update(0.0, 0.0);
  for (int i = 0; i < 50; ++i) filter.update(180.0, 90.0);
  EXPECT_GT(filter.azimuth(),   30.0);
  EXPECT_GT(filter.elevation(), 10.0);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
