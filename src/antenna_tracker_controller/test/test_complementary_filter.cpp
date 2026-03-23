#include <gtest/gtest.h>
#include "antenna_tracker_controller/complementary_filter.hpp"
#include <cmath>

using namespace antenna_tracker_controller;

/* --- Constructor / setters --------------------------------------------- */

TEST(ComplementaryFilterTest, BasicInitialization) {
  ComplementaryFilter filter;
  filter.set_alpha(0.98);
  filter.set_declination(-8.0);
  EXPECT_DOUBLE_EQ(filter.orientation().roll,  0.0);
  EXPECT_DOUBLE_EQ(filter.orientation().pitch, 0.0);
  EXPECT_DOUBLE_EQ(filter.orientation().yaw,   0.0);
}

TEST(ComplementaryFilterTest, SetAlphaStored) {
  ComplementaryFilter filter;
  filter.set_alpha(0.5);
  filter.set_declination(0.0);
  // Two updates with different alpha should yield different pitch (smoke test)
  filter.update(0.0, 0.0, 9.81, 5.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
  filter.update(0.0, 0.0, 9.81, 5.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.0);
  EXPECT_NE(filter.orientation().pitch, 0.0);
}

/* --- First vs. subsequent updates (initialized_ branch) ---------------- */

TEST(ComplementaryFilterTest, FirstUpdateUsesDefaultDt) {
  // First call: initialized_=false → dt = 0.01 (default)
  ComplementaryFilter filter;
  filter.set_alpha(0.98);
  filter.set_declination(0.0);
  filter.update(0.0, 0.0, 9.81, 0.0, 0.0, 0.0, 30.0, 0.0, 30.0, 100.0);
  // Should not crash and orientation should be finite
  EXPECT_FALSE(std::isnan(filter.orientation().roll));
}

TEST(ComplementaryFilterTest, SecondUpdateUsesDtDiff) {
  ComplementaryFilter filter;
  filter.set_alpha(0.98);
  filter.set_declination(0.0);
  filter.update(0.0, 0.0, 9.81, 0.0, 0.0, 0.0, 30.0, 0.0, 30.0, 1.0);
  filter.update(0.0, 0.0, 9.81, 0.0, 0.0, 0.0, 30.0, 0.0, 30.0, 1.01);
  EXPECT_FALSE(std::isnan(filter.orientation().pitch));
}

/* --- Roll / pitch from accelerometer ----------------------------------- */

TEST(ComplementaryFilterTest, GravityAlignedZeroRollPitch) {
  ComplementaryFilter filter;
  filter.set_alpha(0.0);  // 100% accel
  filter.set_declination(0.0);
  filter.update(0.0, 0.0, 9.81, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
  EXPECT_NEAR(filter.orientation().roll,  0.0, 1e-9);
  EXPECT_NEAR(filter.orientation().pitch, 0.0, 1e-9);
}

TEST(ComplementaryFilterTest, AccelRollEstimation) {
  ComplementaryFilter filter;
  filter.set_alpha(0.0);  // 100% accel
  filter.set_declination(0.0);
  // accel_y = 9.81, accel_z = 0 → roll = atan2(9.81, 0) ≈ 90°
  filter.update(0.0, 9.81, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
  EXPECT_NEAR(filter.orientation().roll, 90.0, 1.0);
}

TEST(ComplementaryFilterTest, AccelPitchEstimation) {
  ComplementaryFilter filter;
  filter.set_alpha(0.0);  // 100% accel
  filter.set_declination(0.0);
  // accel_x = 9.81, accel_z = 0 → pitch = atan2(-9.81, 0) ≈ -90°
  filter.update(9.81, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
  EXPECT_NEAR(filter.orientation().pitch, -90.0, 1.0);
}

/* --- Gyro integration --------------------------------------------------- */

TEST(ComplementaryFilterTest, UpdateLogic) {
  ComplementaryFilter filter;
  filter.set_alpha(0.98);
  filter.set_declination(-8.0);
  filter.update(0.0, 0.0, 9.81, 0.0, 0.0, 0.0, 30.0, 0.0, 30.0, 1.0);
  filter.update(1.0, 0.0, 9.81, 0.1, 0.0, 0.0, 30.0, 0.0, 30.0, 1.01);
  EXPECT_NE(filter.orientation().pitch, 0.0);
}

/* --- Heading normalization --------------------------------------------- */

// Negative declination can produce negative heading_raw → +360 branch
TEST(ComplementaryFilterTest, HeadingNormalizationNegativeRaw) {
  ComplementaryFilter filter;
  filter.set_alpha(0.98);
  filter.set_declination(-170.0);  // large negative → heading_raw < 0
  filter.update(0.0, 0.0, 9.81, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0);
  EXPECT_GE(filter.orientation().azimuth, 0.0);
  EXPECT_LT(filter.orientation().azimuth, 360.0);
}

// Large positive heading goes through fmod
TEST(ComplementaryFilterTest, HeadingNormalizationFmod) {
  ComplementaryFilter filter;
  filter.set_alpha(0.98);
  filter.set_declination(350.0);  // large positive → fmod needed
  filter.update(0.0, 0.0, 9.81, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0);
  EXPECT_GE(filter.orientation().azimuth, 0.0);
  EXPECT_LT(filter.orientation().azimuth, 360.0);
}

// Declination shifts heading (compare two filters)
TEST(ComplementaryFilterTest, DeclinationAffectsAzimuth) {
  ComplementaryFilter f1, f2;
  f1.set_alpha(0.98); f1.set_declination(0.0);
  f2.set_alpha(0.98); f2.set_declination(10.0);
  f1.update(0.0, 0.0, 9.81, 0.0, 0.0, 0.0, 30.0, 0.0, 30.0, 1.0);
  f2.update(0.0, 0.0, 9.81, 0.0, 0.0, 0.0, 30.0, 0.0, 30.0, 1.0);
  EXPECT_NE(f1.orientation().azimuth, f2.orientation().azimuth);
}

/* --- Yaw wrap-around ---------------------------------------------------- */

// Accumulate positive gyro yaw until wrap needed
TEST(ComplementaryFilterTest, YawWrapPositiveGyro) {
  ComplementaryFilter filter;
  filter.set_alpha(0.98);
  filter.set_declination(0.0);
  for (int i = 0; i < 20; ++i) {
    filter.update(0.0, 0.0, 9.81, 0.0, 0.0, 20.0, 1.0, 0.0, 0.0, 1.0 + i * 0.05);
  }
  EXPECT_FALSE(std::isnan(filter.orientation().yaw));
}

// Accumulate negative gyro yaw until wrap needed
TEST(ComplementaryFilterTest, YawWrapNegativeGyro) {
  ComplementaryFilter filter;
  filter.set_alpha(0.98);
  filter.set_declination(0.0);
  for (int i = 0; i < 20; ++i) {
    filter.update(0.0, 0.0, 9.81, 0.0, 0.0, -20.0, 1.0, 0.0, 0.0, 1.0 + i * 0.05);
  }
  EXPECT_FALSE(std::isnan(filter.orientation().yaw));
}

/* --- Elevation = pitch -------------------------------------------------- */

TEST(ComplementaryFilterTest, ElevationEqualsPitch) {
  ComplementaryFilter filter;
  filter.set_alpha(0.0);
  filter.set_declination(0.0);
  filter.update(4.905, 0.0, 8.498, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
  EXPECT_DOUBLE_EQ(filter.orientation().elevation, filter.orientation().pitch);
}

/* --- NaN safety --------------------------------------------------------- */

TEST(ComplementaryFilterTest, NaNFreeAfterManyUpdates) {
  ComplementaryFilter filter;
  filter.set_alpha(0.98);
  filter.set_declination(-8.0);
  for (int i = 0; i < 200; ++i) {
    filter.update(0.0, 0.0, 9.81, 0.01, 0.0, 0.01,
                  30.0, 0.5, 30.0, 1.0 + i * 0.01);
  }
  EXPECT_FALSE(std::isnan(filter.orientation().roll));
  EXPECT_FALSE(std::isnan(filter.orientation().pitch));
  EXPECT_FALSE(std::isnan(filter.orientation().yaw));
  EXPECT_FALSE(std::isnan(filter.orientation().azimuth));
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
