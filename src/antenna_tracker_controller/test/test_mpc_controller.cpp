#include <gtest/gtest.h>
#include "antenna_tracker_controller/mpc_controller.hpp"
#include <cmath>

using namespace antenna_tracker_controller;

TEST(MpcControllerTest, BasicExecution) {
  MpcController mpc;
  mpc.init();

  // Skip test if acados solver is not available in this environment
  if (!mpc.is_initialized()) {
    GTEST_SKIP() << "acados capsule not available — skipping BasicExecution";
  }

  double az_out = 0.0;
  double el_out = 0.0;

  mpc.compute(90.0, 0.0, 0.0, 45.0, 0.0, 0.0, az_out, el_out);

  // outputs should try to move towards target
  EXPECT_NE(az_out, 0.0);
  EXPECT_NE(el_out, 0.0);
}

TEST(MpcControllerTest, ComputeWithoutInitReturnsZero) {
  MpcController mpc;

  double az_out = 123.0;
  double el_out = -456.0;

  mpc.compute(90.0, 0.0, 0.0, 45.0, 0.0, 0.0, az_out, el_out);

  EXPECT_DOUBLE_EQ(az_out, 0.0);
  EXPECT_DOUBLE_EQ(el_out, 0.0);
}

TEST(MpcControllerTest, OutputScaleAppliesAfterSolve) {
  MpcController baseline;
  baseline.init();

  double baseline_az = 0.0;
  double baseline_el = 0.0;
  baseline.compute(90.0, 0.0, 0.0, 45.0, 0.0, 0.0, baseline_az, baseline_el);

  MpcController scaled;
  scaled.set_mpc_to_hz_scale(2.0);
  scaled.init();

  double scaled_az = 0.0;
  double scaled_el = 0.0;
  scaled.compute(90.0, 0.0, 0.0, 45.0, 0.0, 0.0, scaled_az, scaled_el);

  ASSERT_NE(baseline_az, 0.0);
  ASSERT_NE(baseline_el, 0.0);
  EXPECT_NEAR(scaled_az, baseline_az * 2.0, std::abs(baseline_az) * 1e-6 + 1e-6);
  EXPECT_NEAR(scaled_el, baseline_el * 2.0, std::abs(baseline_el) * 1e-6 + 1e-6);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
