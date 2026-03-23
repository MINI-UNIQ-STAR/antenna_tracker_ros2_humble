#include <gtest/gtest.h>
#include "antenna_tracker_controller/navigation_node.hpp"
#include <cmath>

using namespace antenna_tracker_controller;

/* === haversine_distance ================================================= */

TEST(NavigationMathTest, HaversineDistanceSamePoint) {
  double dist = NavigationNode::haversine_distance(37.5665, 126.9780, 37.5665, 126.9780);
  EXPECT_DOUBLE_EQ(dist, 0.0);
}

TEST(NavigationMathTest, HaversineDistance) {
  // Seoul → Busan ≈ 325 km
  double dist = NavigationNode::haversine_distance(37.5665, 126.9780, 35.1796, 129.0756);
  EXPECT_NEAR(dist, 325000.0, 10000.0);
}

TEST(NavigationMathTest, HaversineDistanceOneDegreeLatitude) {
  // 1° latitude ≈ 111.195 km
  double dist = NavigationNode::haversine_distance(0.0, 0.0, 1.0, 0.0);
  EXPECT_NEAR(dist, 111195.0, 1000.0);
}

TEST(NavigationMathTest, HaversineDistanceSymmetric) {
  double d1 = NavigationNode::haversine_distance(10.0, 20.0, 15.0, 25.0);
  double d2 = NavigationNode::haversine_distance(15.0, 25.0, 10.0, 20.0);
  EXPECT_NEAR(d1, d2, 1e-6);
}

/* === haversine_bearing ================================================== */

TEST(NavigationMathTest, BearingNorth) {
  // Same longitude, target north → bearing ≈ 0°
  double b = NavigationNode::haversine_bearing(0.0, 0.0, 10.0, 0.0);
  EXPECT_NEAR(b, 0.0, 1.0);
}

TEST(NavigationMathTest, BearingSouth) {
  // Same longitude, target south → bearing ≈ 180°
  double b = NavigationNode::haversine_bearing(10.0, 0.0, 0.0, 0.0);
  EXPECT_NEAR(b, 180.0, 1.0);
}

TEST(NavigationMathTest, BearingEast) {
  // Same latitude at equator, target east → bearing ≈ 90°
  double b = NavigationNode::haversine_bearing(0.0, 0.0, 0.0, 10.0);
  EXPECT_NEAR(b, 90.0, 1.0);
}

TEST(NavigationMathTest, BearingWest) {
  // Same latitude at equator, target west → bearing ≈ 270°
  double b = NavigationNode::haversine_bearing(0.0, 0.0, 0.0, -10.0);
  EXPECT_NEAR(b, 270.0, 1.0);
}

TEST(NavigationMathTest, BearingInRange) {
  // Any bearing result must be in [0, 360)
  double b = NavigationNode::haversine_bearing(37.5, 127.0, 35.2, 129.1);
  EXPECT_GE(b, 0.0);
  EXPECT_LT(b, 360.0);
}

TEST(NavigationMathTest, BearingNegativeWrapsPositive) {
  // atan2 returns negative for westward-southward direction → bearing += 360
  double b = NavigationNode::haversine_bearing(0.0, 10.0, 0.0, 0.0);
  EXPECT_GE(b, 0.0);
  EXPECT_LT(b, 360.0);
}

/* === elevation_angle (static methods only — avoids ROS node context) === */

TEST(NavigationMathTest, ElevationAngleSamePoint_AbovePositive) {
  double el = NavigationNode::elevation_angle(37.0, 127.0, 100.0, 37.0, 127.0, 200.0);
  EXPECT_NEAR(el, 90.0, 1e-6);
}

TEST(NavigationMathTest, ElevationAngleSamePoint_Below) {
  double el = NavigationNode::elevation_angle(37.0, 127.0, 200.0, 37.0, 127.0, 100.0);
  EXPECT_NEAR(el, -90.0, 1e-6);
}

TEST(NavigationMathTest, ElevationAngleSamePoint_SameAlt) {
  double el = NavigationNode::elevation_angle(37.0, 127.0, 100.0, 37.0, 127.0, 100.0);
  EXPECT_NEAR(el, 0.0, 1e-6);
}

TEST(NavigationMathTest, ElevationAngle) {
  // Same point, target 100m higher → 90°
  double el = NavigationNode::elevation_angle(37.0, 127.0, 100.0, 37.0, 127.0, 200.0);
  EXPECT_NEAR(el, 90.0, 1e-6);
}

TEST(NavigationMathTest, ElevationAngleTargetAbove) {
  // ~11km away (0.1° lat), 1000m higher → positive elevation < 90°
  double el = NavigationNode::elevation_angle(37.0, 127.0, 0.0, 37.1, 127.0, 1000.0);
  EXPECT_GT(el, 0.0);
  EXPECT_LT(el, 90.0);
}

TEST(NavigationMathTest, ElevationAngleTargetBelow) {
  // ~11km away, 1000m lower → negative elevation > -90°
  double el = NavigationNode::elevation_angle(37.0, 127.0, 1000.0, 37.1, 127.0, 0.0);
  EXPECT_LT(el, 0.0);
  EXPECT_GE(el, -90.0);
}

TEST(NavigationMathTest, ElevationAngleNearLevel) {
  // Far target at same altitude → ~0° elevation
  double el = NavigationNode::elevation_angle(0.0, 0.0, 100.0, 1.0, 0.0, 100.0);
  EXPECT_NEAR(el, 0.0, 5.0);
}

TEST(NavigationMathTest, ElevationClampedAboveNeg90) {
  // Steep downward case is clamped to -90
  double el = NavigationNode::elevation_angle(0.0, 0.0, 100000.0, 10.0, 0.0, 0.0);
  EXPECT_GE(el, -90.0);
}

TEST(NavigationMathTest, ElevationClampedBelow90) {
  // Steep upward case is clamped to +90
  double el = NavigationNode::elevation_angle(0.0, 0.0, 0.0, 0.01, 0.0, 100000.0);
  EXPECT_LE(el, 90.0);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
