#include "fast_lio/point_scan_motion.hpp"

#include <array>

#include <gtest/gtest.h>

TEST(PointScanMotionTest, ComputesDistinctOriginsForDifferentPointTimes)
{
  fast_lio::PointBodyMotionInScanEnd first;
  first.relative_rotation = {{
    0.0, -1.0, 0.0,
    1.0, 0.0, 0.0,
    0.0, 0.0, 1.0}};
  first.relative_translation = {{0.5, 0.0, 0.0}};
  first.time_from_scan_end_seconds = -0.10;
  first.valid = true;

  fast_lio::PointBodyMotionInScanEnd second;
  second.relative_translation = {{0.0, 0.5, 0.0}};
  second.time_from_scan_end_seconds = -0.01;
  second.valid = true;

  const std::array<double, 3> extrinsic_translation{{1.0, 0.0, 0.0}};
  const auto first_origin = fast_lio::sensorOriginInScanEndBody(
    first, extrinsic_translation);
  const auto second_origin = fast_lio::sensorOriginInScanEndBody(
    second, extrinsic_translation);

  EXPECT_NEAR(first_origin[0], 0.5, 1.0e-12);
  EXPECT_NEAR(first_origin[1], 1.0, 1.0e-12);
  EXPECT_NEAR(second_origin[0], 1.0, 1.0e-12);
  EXPECT_NEAR(second_origin[1], 0.5, 1.0e-12);
  EXPECT_NE(first.time_from_scan_end_seconds, second.time_from_scan_end_seconds);
  EXPECT_NE(first_origin, second_origin);
}

TEST(PointScanMotionTest, UsesTheCurrentEstimatedExtrinsicTranslation)
{
  fast_lio::PointBodyMotionInScanEnd motion;
  motion.relative_rotation = {{
    0.0, -1.0, 0.0,
    1.0, 0.0, 0.0,
    0.0, 0.0, 1.0}};
  motion.relative_translation = {{0.5, 0.0, 0.0}};

  const auto initial = fast_lio::sensorOriginInScanEndBody(
    motion, {{1.0, 0.0, 0.0}});
  const auto updated = fast_lio::sensorOriginInScanEndBody(
    motion, {{0.0, 1.0, 0.0}});

  EXPECT_NEAR(initial[0], 0.5, 1.0e-12);
  EXPECT_NEAR(initial[1], 1.0, 1.0e-12);
  EXPECT_NEAR(updated[0], -0.5, 1.0e-12);
  EXPECT_NEAR(updated[1], 0.0, 1.0e-12);
}
