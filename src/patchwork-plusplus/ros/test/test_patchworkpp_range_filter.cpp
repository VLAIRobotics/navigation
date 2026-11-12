#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include <Eigen/Dense>

#include "patchwork/patchworkpp.h"

TEST(PatchworkppRangeFilter, excludes_points_outside_estimation_interval)
{
  patchwork::Params params;
  params.min_range = 0.25;
  params.max_range = 10.0;
  patchwork::PatchWorkpp patchwork(params);

  Eigen::MatrixXf cloud(2, 3);
  cloud << 0.10, 0.0, 0.0,
            11.0, 0.0, 0.0;

  patchwork.estimateGround(cloud);

  EXPECT_EQ(patchwork.getGround().rows(), 0);
  EXPECT_EQ(patchwork.getNonground().rows(), 0);
}

TEST(PatchworkppRangeFilter, retains_supported_surface_returns_inside_the_near_field)
{
  patchwork::Params params;
  params.min_range = 0.25;
  params.max_range = 10.0;
  params.sensor_height = 0.20;
  params.num_sectors_each_zone[0] = 1;
  params.num_rings_each_zone[0] = 1;
  patchwork::PatchWorkpp patchwork(params);

  std::vector<Eigen::Vector3f> points;
  for (int x_index = -25; x_index <= 25; ++x_index) {
    for (int y_index = -25; y_index <= 25; ++y_index) {
      const float x = static_cast<float>(x_index) * 0.02F;
      const float y = static_cast<float>(y_index) * 0.02F;
      const float range = std::hypot(x, y);
      if (range > 0.26F && range < 0.50F) {
        points.emplace_back(x, y, -0.20F);
      }
    }
  }
  ASSERT_GT(points.size(), 10U);
  Eigen::MatrixXf cloud(static_cast<Eigen::Index>(points.size()), 3);
  for (std::size_t index = 0; index < points.size(); ++index) {
    cloud.row(static_cast<Eigen::Index>(index)) = points[index].transpose();
  }

  patchwork.estimateGround(cloud);

  const auto ground = patchwork.getGround();
  ASSERT_GT(ground.rows(), 0);
  bool has_near_ground_return = false;
  for (Eigen::Index index = 0; index < ground.rows(); ++index) {
    const float range = std::hypot(ground(index, 0), ground(index, 1));
    has_near_ground_return = has_near_ground_return || (range > 0.25F && range < 0.50F);
  }
  EXPECT_TRUE(has_near_ground_return);
}
