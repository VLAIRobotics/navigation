#include <gtest/gtest.h>

#include <Eigen/Core>

#include "GroundResidualFilter.hpp"

TEST(GroundResidualFilter, removes_nonground_points_that_match_local_ground)
{
  Eigen::MatrixX3f ground(1, 3);
  ground << 1.0F, 2.0F, -0.20F;
  Eigen::MatrixX3f nonground(2, 3);
  nonground << 1.02F, 2.01F, -0.21F,
               1.02F, 2.01F, -0.14F;

  const auto filtered = patchworkpp_ros::utils::filterGroundResiduals(
    ground, nonground, 0.10F, 0.03F);

  ASSERT_EQ(filtered.rows(), 1);
  EXPECT_FLOAT_EQ(filtered(0, 2), -0.14F);
}

TEST(GroundResidualFilter, preserves_low_obstacle_above_sloped_floor)
{
  Eigen::MatrixX3f ground(1, 3);
  ground << -0.5F, 0.8F, -0.08F;
  Eigen::MatrixX3f nonground(1, 3);
  nonground << -0.48F, 0.81F, -0.06F;

  const auto filtered = patchworkpp_ros::utils::filterGroundResiduals(
    ground, nonground, 0.10F, 0.03F);

  ASSERT_EQ(filtered.rows(), 1);
  EXPECT_FLOAT_EQ(filtered(0, 2), -0.06F);
}

TEST(GroundResidualFilter, preserves_points_when_no_local_ground_sample_exists)
{
  Eigen::MatrixX3f ground(1, 3);
  ground << 1.0F, 2.0F, -0.20F;
  Eigen::MatrixX3f nonground(1, 3);
  nonground << 1.5F, 2.0F, -0.19F;

  const auto filtered = patchworkpp_ros::utils::filterGroundResiduals(
    ground, nonground, 0.10F, 0.03F);

  EXPECT_EQ(filtered.rows(), 1);
}

TEST(GroundResidualFilter, promotes_supported_shallow_surface_returns)
{
  Eigen::MatrixX3f ground(3, 3);
  ground << 0.00F, 0.00F, 0.00F,
            0.04F, 0.00F, 0.00F,
            0.00F, 0.04F, 0.00F;
  Eigen::MatrixX3f nonground(3, 3);
  nonground << 0.00F, 0.00F, 0.02F,
               0.04F, 0.00F, 0.02F,
               0.00F, 0.04F, 0.02F;

  const auto split = patchworkpp_ros::utils::splitGroundResiduals(
    ground, nonground, 0.10F, 0.03F);

  EXPECT_EQ(split.promoted_ground_points, 3U);
  EXPECT_EQ(split.ground.rows(), 6);
  EXPECT_EQ(split.nonground.rows(), 0);
}

TEST(GroundResidualFilter, retains_isolated_and_line_supported_residuals)
{
  Eigen::MatrixX3f ground(4, 3);
  ground << 0.00F, 0.00F, 0.00F,
            0.04F, 0.00F, 0.00F,
            0.08F, 0.00F, 0.00F,
            0.50F, 0.50F, 0.00F;
  Eigen::MatrixX3f nonground(4, 3);
  nonground << 0.00F, 0.00F, 0.02F,
               0.04F, 0.00F, 0.02F,
               0.08F, 0.00F, 0.02F,
               0.50F, 0.50F, 0.02F;

  const auto split = patchworkpp_ros::utils::splitGroundResiduals(
    ground, nonground, 0.10F, 0.03F);

  EXPECT_EQ(split.promoted_ground_points, 0U);
  EXPECT_EQ(split.ground.rows(), ground.rows());
  EXPECT_EQ(split.nonground.rows(), nonground.rows());
}

TEST(GroundResidualFilter, retains_surfaces_above_the_residual_height_band)
{
  Eigen::MatrixX3f ground(3, 3);
  ground << 0.00F, 0.00F, 0.00F,
            0.04F, 0.00F, 0.00F,
            0.00F, 0.04F, 0.00F;
  Eigen::MatrixX3f nonground(3, 3);
  nonground << 0.00F, 0.00F, 0.04F,
               0.04F, 0.00F, 0.04F,
               0.00F, 0.04F, 0.04F;

  const auto split = patchworkpp_ros::utils::splitGroundResiduals(
    ground, nonground, 0.10F, 0.03F);

  EXPECT_EQ(split.promoted_ground_points, 0U);
  EXPECT_EQ(split.ground.rows(), ground.rows());
  EXPECT_EQ(split.nonground.rows(), nonground.rows());
}

TEST(GroundResidualFilter, removes_negative_residuals_using_nearby_ground_samples)
{
  Eigen::MatrixX3f ground(2, 3);
  ground << 0.00F, 0.00F, 0.00F,
            0.04F, 0.00F, 0.02F;
  Eigen::MatrixX3f nonground(1, 3);
  nonground << 0.03F, 0.00F, -0.005F;

  const auto split = patchworkpp_ros::utils::splitGroundResiduals(
    ground, nonground, 0.10F, 0.03F);

  EXPECT_EQ(split.promoted_ground_points, 0U);
  EXPECT_EQ(split.ground.rows(), ground.rows());
  EXPECT_EQ(split.nonground.rows(), 0);
}
