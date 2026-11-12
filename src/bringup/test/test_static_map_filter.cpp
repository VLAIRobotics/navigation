#include <cmath>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <pcl/io/pcd_io.h>

#include "bringup/temporal_voxel_map.hpp"

namespace
{

bringup::TemporalVoxelParameters test_parameters()
{
  bringup::TemporalVoxelParameters parameters;
  parameters.voxel_size = 0.5;
  parameters.min_observations = 3;
  parameters.min_observation_duration = 0.2;
  parameters.candidate_timeout = 1.0;
  parameters.confirmed_miss_limit = 2;
  parameters.min_range = 0.5;
  parameters.max_range = 20.0;
  return parameters;
}

template<typename PointContainer>
bool contains_point_near(
  const PointContainer & points, const float x, const float z)
{
  for (const auto & candidate : points) {
    if (std::abs(candidate.x - x) < 0.1F && std::abs(candidate.z - z) < 0.1F) {
      return true;
    }
  }
  return false;
}

pcl::PointXYZ point(const float x, const float y = 0.0F, const float z = 0.0F)
{
  return pcl::PointXYZ{x, y, z};
}

}  // namespace

TEST(TemporalVoxelMap, ConfirmsAcrossConsecutiveFrames)
{
  bringup::TemporalVoxelMap map(test_parameters());
  map.set_origin(0.0, 0.0, 0.0);

  EXPECT_TRUE(map.process_frame({point(5.0F)}, 0.0).current_static_points.empty());
  EXPECT_TRUE(map.process_frame({point(5.0F)}, 0.1).current_static_points.empty());
  EXPECT_EQ(map.process_frame({point(5.0F)}, 0.2).current_static_points.size(), 1U);
  EXPECT_EQ(map.confirmed_count(), 1U);
}

TEST(TemporalVoxelMap, InterruptedCandidateMustStartOver)
{
  bringup::TemporalVoxelMap map(test_parameters());
  map.set_origin(0.0, 0.0, 0.0);

  map.process_frame({point(5.0F)}, 0.0);
  map.process_frame({point(5.0F)}, 0.1);
  EXPECT_TRUE(map.process_frame({}, 0.2).current_static_points.empty());
  EXPECT_TRUE(map.process_frame({point(5.0F)}, 0.3).current_static_points.empty());
  EXPECT_TRUE(map.process_frame({point(5.0F)}, 0.4).current_static_points.empty());
  EXPECT_EQ(map.process_frame({point(5.0F)}, 0.5).current_static_points.size(), 1U);
  EXPECT_EQ(map.complete_static_points().size(), 3U);
}

TEST(TemporalVoxelMap, InterruptedCandidateResetsOriginHistoryBeforeArchive)
{
  auto parameters = test_parameters();
  parameters.min_observations = 2;
  parameters.min_observation_duration = 0.1;
  parameters.archive_min_observation_duration = 0.1;
  parameters.archive_min_origin_separation = 0.5;
  bringup::TemporalVoxelMap map(parameters);
  map.set_origin(0.0, 0.0, 0.0);

  map.process_frame({point(5.0F)}, 0.0);
  map.process_frame({}, 0.1);
  map.set_origin(1.0, 0.0, 0.0);
  map.process_frame({point(5.0F)}, 0.2);
  map.process_frame({point(5.0F)}, 0.3);

  EXPECT_EQ(map.confirmed_count(), 1U);
  EXPECT_EQ(map.archived_count(), 0U);
}

TEST(TemporalVoxelMap, CandidateTimeoutRemovesUnconfirmedVoxel)
{
  bringup::TemporalVoxelMap map(test_parameters());
  map.process_frame({point(5.0F)}, 0.0);
  ASSERT_EQ(map.voxel_count(), 1U);
  map.process_frame({}, 2.0);
  map.cleanup_candidates();
  EXPECT_EQ(map.voxel_count(), 0U);
}

TEST(TemporalVoxelMap, RayMissesRemoveConfirmedVoxel)
{
  bringup::TemporalVoxelMap map(test_parameters());
  map.set_origin(0.0, 0.0, 0.0);
  map.process_frame({point(5.0F)}, 0.0);
  map.process_frame({point(5.0F)}, 0.1);
  map.process_frame({point(5.0F)}, 0.2);
  ASSERT_EQ(map.confirmed_count(), 1U);

  map.process_frame({point(10.0F)}, 0.3);
  map.process_frame({point(10.0F)}, 0.4);
  EXPECT_EQ(map.confirmed_count(), 0U);
}

TEST(TemporalVoxelMap, DefaultMissHysteresisClearsAfterTenFrames)
{
  bringup::TemporalVoxelParameters parameters;
  EXPECT_EQ(parameters.confirmed_miss_limit, 10);

  bringup::TemporalVoxelMap map(parameters);
  map.set_origin(0.0, 0.0, 0.0);
  const auto obstacle = point(5.0F);
  for (int frame = 0; frame < 5; ++frame) {
    map.process_frame({obstacle}, 0.1 * frame);
  }
  ASSERT_EQ(map.confirmed_count(), 1U);

  for (int frame = 0; frame < 9; ++frame) {
    map.process_frame({point(10.0F + 0.5F * frame)}, 0.5 + 0.1 * frame);
  }
  EXPECT_EQ(map.confirmed_count(), 1U);
  map.process_frame({point(14.5F)}, 1.4);
  EXPECT_EQ(map.confirmed_count(), 0U);
}

TEST(TemporalVoxelMap, NonAdjacentMissesDoNotAccumulate)
{
  bringup::TemporalVoxelMap map(test_parameters());
  map.set_origin(0.0, 0.0, 0.0);
  map.process_frame({point(5.0F)}, 0.0);
  map.process_frame({point(5.0F)}, 0.1);
  map.process_frame({point(5.0F)}, 0.2);
  ASSERT_EQ(map.confirmed_count(), 1U);

  map.process_frame({point(10.0F)}, 0.3);
  map.process_frame({}, 0.4);
  map.process_frame({point(10.0F)}, 0.5);
  EXPECT_EQ(map.confirmed_count(), 1U);
  map.process_frame({point(10.0F)}, 0.6);
  EXPECT_EQ(map.confirmed_count(), 0U);
}

TEST(TemporalVoxelMap, RayDdaHandlesNegativeDirection)
{
  bringup::TemporalVoxelMap map(test_parameters());
  map.set_origin(5.0, 0.0, 0.0);
  map.process_frame({point(0.0F)}, 0.0);
  map.process_frame({point(0.0F)}, 0.1);
  map.process_frame({point(0.0F)}, 0.2);
  ASSERT_EQ(map.confirmed_count(), 1U);

  map.process_frame({point(-5.0F)}, 0.3);
  map.process_frame({point(-5.0F)}, 0.4);
  EXPECT_EQ(map.confirmed_count(), 0U);
}

TEST(TemporalVoxelMap, RayDdaHandlesThreeAxisDiagonal)
{
  bringup::TemporalVoxelMap map(test_parameters());
  map.set_origin(0.0, 0.0, 0.0);
  map.process_frame({point(3.0F, 3.0F, 3.0F)}, 0.0);
  map.process_frame({point(3.0F, 3.0F, 3.0F)}, 0.1);
  map.process_frame({point(3.0F, 3.0F, 3.0F)}, 0.2);
  ASSERT_EQ(map.confirmed_count(), 1U);

  map.process_frame({point(6.0F, 6.0F, 6.0F)}, 0.3);
  map.process_frame({point(6.0F, 6.0F, 6.0F)}, 0.4);
  EXPECT_EQ(map.confirmed_count(), 0U);
}

TEST(TemporalVoxelMap, RayDdaHandlesOriginOnVoxelBoundary)
{
  bringup::TemporalVoxelMap map(test_parameters());
  map.set_origin(0.5, 0.5, 0.5);
  map.process_frame({point(1.0F, 1.0F, 1.0F)}, 0.0);
  map.process_frame({point(1.0F, 1.0F, 1.0F)}, 0.1);
  map.process_frame({point(1.0F, 1.0F, 1.0F)}, 0.2);
  ASSERT_EQ(map.confirmed_count(), 1U);

  map.process_frame({point(2.0F, 2.0F, 2.0F)}, 0.3);
  map.process_frame({point(2.0F, 2.0F, 2.0F)}, 0.4);
  EXPECT_EQ(map.confirmed_count(), 0U);
}

TEST(TemporalVoxelMap, AngularDownsamplingKeepsNearestEndpoint)
{
  bringup::TemporalVoxelMap map(test_parameters());
  map.set_origin(0.0, 0.0, 0.0);
  map.process_frame({point(5.0F)}, 0.0);
  map.process_frame({point(5.0F)}, 0.1);
  map.process_frame({point(5.0F)}, 0.2);
  ASSERT_EQ(map.confirmed_count(), 1U);

  map.process_frame({point(3.0F), point(10.0F)}, 0.3);
  map.process_frame({point(3.0F), point(10.0F)}, 0.4);
  EXPECT_EQ(map.confirmed_count(), 1U);
}

TEST(TemporalVoxelMap, TimeRollbackResetsMap)
{
  bringup::TemporalVoxelMap map(test_parameters());
  map.process_frame({point(5.0F)}, 1.0);
  map.process_frame({point(5.0F)}, 1.1);
  map.process_frame({point(5.0F)}, 1.2);
  ASSERT_EQ(map.confirmed_count(), 1U);

  map.process_frame({point(5.0F)}, 0.1);
  EXPECT_EQ(map.confirmed_count(), 0U);
}

TEST(TemporalVoxelMap, EmptyMapCannotOverwriteExistingPcd)
{
  const std::filesystem::path path =
    std::filesystem::temp_directory_path() / "static_map_filter_empty_test.pcd";
  {
    std::ofstream existing(path);
    existing << "existing";
  }

  std::string error;
  EXPECT_FALSE(bringup::save_points_to_pcd(path.string(), {}, &error));
  std::ifstream result(path);
  std::string contents;
  result >> contents;
  EXPECT_EQ(contents, "existing");
  std::filesystem::remove(path);
}

TEST(TemporalVoxelMap, AtomicPcdSaveWritesConfirmedPoints)
{
  const std::filesystem::path path =
    std::filesystem::temp_directory_path() / "static_map_filter_save_test.pcd";
  const std::vector<pcl::PointXYZ> points{point(1.0F), point(2.0F)};
  std::string error;
  ASSERT_TRUE(bringup::save_points_to_pcd(path.string(), points, &error)) << error;

  pcl::PointCloud<pcl::PointXYZ> loaded;
  ASSERT_EQ(pcl::io::loadPCDFile(path.string(), loaded), 0);
  EXPECT_EQ(loaded.size(), points.size());
  EXPECT_FALSE(std::filesystem::exists(path.string() + ".tmp"));
  std::filesystem::remove(path);
}

TEST(TemporalVoxelMap, CompleteStaticPointsKeepAllConfirmedObservations)
{
  auto parameters = test_parameters();
  parameters.min_observations = 2;
  parameters.min_observation_duration = 0.1;
  bringup::TemporalVoxelMap map(parameters);
  map.set_origin(0.0, 0.0, 0.0);

  map.process_frame({point(5.00F), point(5.10F)}, 0.0);
  map.process_frame({point(5.02F), point(5.08F)}, 0.1);

  EXPECT_EQ(map.static_points().size(), 1U);
  const auto complete_points = map.complete_static_points();
  ASSERT_EQ(complete_points.size(), 4U);
  EXPECT_TRUE(contains_point_near(complete_points, 5.00F, 0.0F));
  EXPECT_TRUE(contains_point_near(complete_points, 5.10F, 0.0F));

  const auto path =
    std::filesystem::temp_directory_path() / "static_map_filter_complete_test.pcd";
  std::string error;
  ASSERT_TRUE(bringup::save_points_to_pcd(path.string(), complete_points, &error)) << error;
  pcl::PointCloud<pcl::PointXYZ> loaded;
  ASSERT_EQ(pcl::io::loadPCDFile(path.string(), loaded), 0);
  EXPECT_EQ(loaded.size(), complete_points.size());
  std::filesystem::remove(path);
}

TEST(TemporalVoxelMap, StaticPointsExcludeCandidatesOutOfRangeAndRemovedObstacles)
{
  auto parameters = test_parameters();
  parameters.confirmed_miss_limit = 2;
  bringup::TemporalVoxelMap map(parameters);
  map.set_origin(0.0, 0.0, 0.0);

  const auto candidate = point(6.0F);
  const auto out_of_range = point(25.0F);
  const auto obstacle = point(5.0F);
  for (double stamp : {0.0, 0.1, 0.2}) {
    map.process_frame({candidate, out_of_range, obstacle}, stamp);
  }
  ASSERT_EQ(map.confirmed_count(), 2U);

  map.process_frame({candidate, out_of_range, point(10.0F)}, 0.3);
  map.process_frame({candidate, out_of_range, point(10.0F)}, 0.4);

  const auto saved_points = map.static_points();
  ASSERT_EQ(saved_points.size(), 1U);
  EXPECT_NEAR(saved_points.front().x, candidate.x, 1e-6F);

  const auto path =
    std::filesystem::temp_directory_path() / "static_map_filter_confirmed_only_test.pcd";
  std::string error;
  ASSERT_TRUE(bringup::save_points_to_pcd(path.string(), saved_points, &error)) << error;
  pcl::PointCloud<pcl::PointXYZ> loaded;
  ASSERT_EQ(pcl::io::loadPCDFile(path.string(), loaded), 0);
  ASSERT_EQ(loaded.size(), 1U);
  EXPECT_NEAR(loaded.front().x, candidate.x, 1e-6F);
  std::filesystem::remove(path);
}

TEST(TemporalVoxelMap, GroundAndObstacleAreSavedAndOnlyRayMissedObstacleClears)
{
  auto parameters = test_parameters();
  parameters.confirmed_miss_limit = 10;
  bringup::TemporalVoxelMap map(parameters);
  map.set_origin(0.0, 0.0, 0.0);

  const auto ground = point(5.0F, 0.0F, -0.2F);
  const auto obstacle = point(5.0F, 0.0F, 1.0F);
  for (int frame = 0; frame < 3; ++frame) {
    map.process_frame({ground, obstacle}, 0.1 * frame);
  }
  ASSERT_EQ(map.confirmed_count(), 2U);
  EXPECT_TRUE(contains_point_near(map.complete_static_points(), 5.0F, -0.2F));
  EXPECT_TRUE(contains_point_near(map.complete_static_points(), 5.0F, 1.0F));

  for (int frame = 0; frame < 10; ++frame) {
    map.process_frame({ground, point(10.0F, 0.0F, 2.0F)}, 0.3 + 0.1 * frame);
  }

  const auto saved_points = map.complete_static_points();
  EXPECT_TRUE(contains_point_near(saved_points, 5.0F, -0.2F));
  EXPECT_FALSE(contains_point_near(saved_points, 5.0F, 1.0F));

  const auto path =
    std::filesystem::temp_directory_path() / "static_map_filter_ground_test.pcd";
  std::string error;
  ASSERT_TRUE(bringup::save_points_to_pcd(path.string(), saved_points, &error)) << error;
  pcl::PointCloud<pcl::PointXYZ> loaded;
  ASSERT_EQ(pcl::io::loadPCDFile(path.string(), loaded), 0);
  EXPECT_TRUE(contains_point_near(loaded.points, 5.0F, -0.2F));
  EXPECT_FALSE(contains_point_near(loaded.points, 5.0F, 1.0F));
  std::filesystem::remove(path);
}

TEST(TemporalVoxelMap, LeavingDisplayWindowPreservesGlobalPoint)
{
  auto parameters = test_parameters();
  parameters.min_observations = 2;
  parameters.min_observation_duration = 0.1;
  parameters.archive_min_observation_duration = 0.2;
  parameters.archive_min_origin_separation = 0.5;
  parameters.confirmed_miss_limit = 2;
  bringup::TemporalVoxelMap map(parameters);
  map.set_origin(0.0, 0.0, 0.0);

  map.process_frame({point(5.0F)}, 0.0);
  map.process_frame({point(5.0F)}, 0.1);
  ASSERT_EQ(map.complete_static_points().size(), 2U);

  // Moving the local display center does not delete the global voxel. The
  // new endpoint is not on the ray through the existing point.
  map.set_origin(20.0, 0.0, 0.0);
  map.process_frame({point(25.0F)}, 0.2);
  const auto saved_points = map.complete_static_points();
  ASSERT_EQ(saved_points.size(), 2U);
  EXPECT_TRUE(contains_point_near(saved_points, 5.0F, 0.0F));
}

TEST(TemporalVoxelMap, ArchivedStaticVoxelIsClearedByConfirmedRayMiss)
{
  auto parameters = test_parameters();
  parameters.min_observations = 2;
  parameters.min_observation_duration = 0.1;
  parameters.archive_min_observation_duration = 0.2;
  parameters.archive_min_origin_separation = 0.5;
  parameters.confirmed_miss_limit = 2;
  bringup::TemporalVoxelMap map(parameters);
  map.set_origin(0.0, 0.0, 0.0);

  map.process_frame({point(5.0F)}, 0.0);
  map.process_frame({point(5.0F)}, 0.1);
  map.set_origin(1.0, 0.0, 0.0);
  map.process_frame({point(5.0F)}, 0.2);
  map.process_frame({point(5.0F)}, 0.3);
  ASSERT_EQ(map.complete_static_points().size(), 4U);

  // Use different, unconfirmed endpoints so the old voxel receives two
  // consecutive ray misses without confirming a replacement endpoint.
  map.process_frame({point(10.0F)}, 0.4);
  map.process_frame({point(11.0F)}, 0.5);
  EXPECT_TRUE(map.static_points().empty());
  EXPECT_TRUE(map.complete_static_points().empty());
}

TEST(TemporalVoxelMap, UnstableShortLivedVoxelIsNotArchived)
{
  auto parameters = test_parameters();
  parameters.min_observations = 2;
  parameters.min_observation_duration = 0.1;
  parameters.archive_min_observation_duration = 1.0;
  parameters.archive_min_origin_separation = 0.5;
  parameters.confirmed_miss_limit = 2;
  bringup::TemporalVoxelMap map(parameters);
  map.set_origin(0.0, 0.0, 0.0);

  map.process_frame({point(5.0F)}, 0.0);
  map.process_frame({point(5.0F)}, 0.1);
  map.set_origin(1.0, 0.0, 0.0);
  map.process_frame({point(5.0F)}, 0.2);
  map.process_frame({point(10.0F)}, 0.3);
  map.process_frame({point(11.0F)}, 0.4);
  EXPECT_TRUE(map.complete_static_points().empty());
}
