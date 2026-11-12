#include <algorithm>
#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include "bringup/consecutive_observation.hpp"
#include "bringup/footprint_cells.hpp"
#include "bringup/fresh_retry_queue.hpp"
#include "bringup/frontier_selection.hpp"
#include "bringup/map_fusion_update.hpp"
#include "bringup/no_elevation_obstacle.hpp"
#include "bringup/obstacle_evidence_sequence.hpp"
#include "bringup/occupancy_source.hpp"
#include "bringup/occupancy_grid_growth.hpp"
#include "bringup/occupancy_raytrace.hpp"

TEST(ConsecutiveObservation, RequiresAnObservationInEachFrame)
{
  bringup::ConsecutiveObservation evidence;
  EXPECT_EQ(evidence.observe(1, 1000000000, 150000000), 1);
  EXPECT_EQ(evidence.observe(2, 1100000000, 150000000), 2);
  EXPECT_EQ(evidence.observe(4, 1300000000, 150000000), 1);
}

TEST(ConsecutiveObservation, ResetsAfterTimestampGap)
{
  bringup::ConsecutiveObservation evidence;
  EXPECT_EQ(evidence.observe(1, 1000000000, 150000000), 1);
  EXPECT_EQ(evidence.observe(2, 1600000000, 150000000), 1);
}

TEST(ConsecutiveObservation, SaturatesAtCounterLimit)
{
  bringup::ConsecutiveObservation evidence;
  for (std::uint64_t frame = 1; frame <= 300; ++frame) {
    EXPECT_EQ(evidence.observe(frame, static_cast<std::int64_t>(frame) * 10000000, 150000000),
      std::min<std::uint64_t>(frame, 255));
  }
}

TEST(NongroundHeightFallback, PreservesLowBlockersWhenElevationIsUnavailable)
{
  EXPECT_TRUE(bringup::isNoElevationBlocking(0.0, 0.25, 0.05, 0.30));
  EXPECT_FALSE(bringup::isNoElevationBlocking(0.10, 0.25, 0.05, 0.30));
}

TEST(TimeWindowObservation, AccumulatesIrregularFramesWithinWindow)
{
  bringup::TimeWindowObservation evidence;
  constexpr std::int64_t window_ns = 2000000000;
  EXPECT_EQ(evidence.observe(1000000000, window_ns), 1);
  EXPECT_EQ(evidence.observe(1300000000, window_ns), 2);
  EXPECT_EQ(evidence.observe(1800000000, window_ns), 3);
  EXPECT_EQ(evidence.observe(2100000000, window_ns), 4);
  const auto count = evidence.observe(2600000000, window_ns);
  EXPECT_EQ(count, 5);
  ASSERT_TRUE(bringup::groundEvidenceUpdate(100, false, count, 3, 5));
  EXPECT_EQ(*bringup::groundEvidenceUpdate(100, false, count, 3, 5), 0);
}

TEST(TimeWindowObservation, DropsEvidenceOutsideWindow)
{
  bringup::TimeWindowObservation evidence;
  constexpr std::int64_t window_ns = 2000000000;
  EXPECT_EQ(evidence.observe(1000000000, window_ns), 1);
  EXPECT_EQ(evidence.observe(1300000000, window_ns), 2);
  EXPECT_EQ(evidence.observe(1800000000, window_ns), 3);
  EXPECT_EQ(evidence.observe(4000000000, window_ns), 1);
}

TEST(TimeWindowObservation, RejectsInvalidOrBackwardTimestamps)
{
  bringup::TimeWindowObservation evidence;
  constexpr std::int64_t window_ns = 2000000000;
  EXPECT_EQ(evidence.observe(1000000000, window_ns), 1);
  EXPECT_EQ(evidence.observe(900000000, window_ns), 1);
  EXPECT_EQ(evidence.observe(0, window_ns), 0);
}

TEST(OccupancyGridGrowth, PreservesCellsAndMovesOriginWhenExpanding)
{
  nav_msgs::msg::OccupancyGrid map;
  map.info.width = 2;
  map.info.height = 2;
  map.info.resolution = 0.1F;
  map.info.origin.orientation.w = 1.0;
  map.data = {10, 20, 30, 40};
  bringup::GridGrowthResult result;
  std::string error;

  ASSERT_TRUE(bringup::growOccupancyGridToInclude(
      map, -2, 0, 2, 1, 2, 2, 0, 0, 1.0, 100, &result, &error)) << error;

  EXPECT_TRUE(result.changed);
  EXPECT_EQ(result.old_offset_x, 2U);
  EXPECT_EQ(map.info.width, 5U);
  EXPECT_EQ(map.info.height, 2U);
  EXPECT_NEAR(map.info.origin.position.x, -0.2, 1e-6);
  EXPECT_EQ(map.data[2], 10);
  EXPECT_EQ(map.data[3], 20);
  EXPECT_EQ(map.data[7], 30);
  EXPECT_EQ(map.data[8], 40);
  EXPECT_EQ(map.data[0], -1);
}

TEST(OccupancyGridGrowth, RejectsPointsBeyondTheConfiguredExtension)
{
  nav_msgs::msg::OccupancyGrid map;
  map.info.width = 2;
  map.info.height = 2;
  map.info.resolution = 0.1F;
  map.info.origin.orientation.w = 1.0;
  map.data = {0, 0, 0, 0};
  bringup::GridGrowthResult result;
  std::string error;

  EXPECT_FALSE(bringup::growOccupancyGridToInclude(
      map, -11, 0, -11, 0, 2, 2, 0, 0, 1.0, 100, &result, &error));
  EXPECT_NE(error.find("extension limit"), std::string::npos);
  EXPECT_EQ(map.info.width, 2U);
  EXPECT_EQ(map.data.size(), 4U);
}

TEST(FootprintCells, RasterizesIntersectingCellsAndClipsAtMapBounds)
{
  const std::vector<bringup::FootprintPoint2D> square{
    {0.0, 0.0}, {2.0, 0.0}, {2.0, 2.0}, {0.0, 2.0}};
  EXPECT_EQ(
    bringup::rasterizeFootprintCells(4, 4, 1.0, square),
    (std::vector<std::size_t>{0, 1, 2, 4, 5, 6, 8, 9, 10}));

  const std::vector<bringup::FootprintPoint2D> partly_outside{
    {-1.0, -1.0}, {1.0, -1.0}, {1.0, 1.0}, {-1.0, 1.0}};
  EXPECT_EQ(
    bringup::rasterizeFootprintCells(4, 4, 1.0, partly_outside),
    (std::vector<std::size_t>{0, 1, 4, 5}));
}

TEST(FootprintCells, OptionalMetricMarginCoversSamplingRingAndPreservesClearingVetoes)
{
  const std::vector<bringup::FootprintPoint2D> footprint{
    {0.12, 0.12}, {0.18, 0.12}, {0.18, 0.18}, {0.12, 0.18}};
  const auto exact_cells = bringup::rasterizeFootprintCells(4, 4, 0.1, footprint);
  EXPECT_EQ(exact_cells, (std::vector<std::size_t>{5}));

  const auto expanded_cells = bringup::rasterizeFootprintCells(
    4, 4, 0.1, footprint,
    bringup::path_collision::kMaxSampleDisplacementCells * 0.1);
  EXPECT_EQ(expanded_cells, (std::vector<std::size_t>{0, 1, 2, 4, 5, 6, 8, 9, 10}));

  std::vector<std::int8_t> occupancy(16, -1);
  std::vector<std::uint8_t> prior_occupied(16, 0);
  prior_occupied[5] = 1;
  std::vector<bringup::ConsecutiveObservation> obstacle_evidence(16);
  std::vector<bringup::TimeWindowObservation> ground_evidence(16);
  std::vector<std::size_t> changed;
  bringup::clearFootprintCells(
    occupancy, prior_occupied, obstacle_evidence, ground_evidence,
    expanded_cells, changed, {6});

  EXPECT_EQ(occupancy[0], 0);
  EXPECT_EQ(occupancy[5], -1);
  EXPECT_EQ(occupancy[6], -1);
  EXPECT_EQ(occupancy[3], -1);
  EXPECT_EQ(changed.size(), 7U);
}

TEST(FootprintCells, RasterizesRotatedPolygonInGridCoordinates)
{
  const std::vector<bringup::FootprintPoint2D> diamond{
    {2.5, 0.5}, {4.5, 2.5}, {2.5, 4.5}, {0.5, 2.5}};
  const auto cells = bringup::rasterizeFootprintCells(5, 5, 1.0, diamond);
  EXPECT_NE(std::find(cells.begin(), cells.end(), 2 * 5 + 2), cells.end());
  EXPECT_NE(std::find(cells.begin(), cells.end(), 1 * 5 + 2), cells.end());
  EXPECT_EQ(std::find(cells.begin(), cells.end(), 0), cells.end());
  EXPECT_EQ(std::find(cells.begin(), cells.end(), 4 * 5 + 4), cells.end());
}

TEST(FootprintCells, KeepsPriorOccupiedCellsAlignedAfterMapGrowth)
{
  const std::vector<std::uint8_t> prior_occupied{1, 0, 0, 1};
  const auto remapped = bringup::remapPriorOccupiedCells(
    prior_occupied, 2, 2, 4, 3, 1, 1);
  ASSERT_EQ(remapped.size(), 12U);
  EXPECT_EQ(remapped[1 * 4 + 1], 1);
  EXPECT_EQ(remapped[2 * 4 + 2], 1);
  EXPECT_EQ(remapped[1 * 4 + 2], 0);
  EXPECT_EQ(remapped[0], 0);
}

TEST(FootprintCells, ClearsUnknownAndOnlineOccupiedButPreservesPriorOccupied)
{
  std::vector<std::int8_t> occupancy{-1, 100, 100, 0};
  const std::vector<std::uint8_t> prior_occupied{0, 1, 0, 0};
  std::vector<bringup::ConsecutiveObservation> obstacle_evidence(4);
  std::vector<bringup::TimeWindowObservation> ground_evidence(4);
  for (std::size_t i = 0; i < occupancy.size(); ++i) {
    obstacle_evidence[i].observe(1, 1000000000, 150000000);
    ground_evidence[i].observe(1000000000, 2000000000);
  }
  std::vector<std::size_t> changed;

  bringup::clearFootprintCells(
    occupancy, prior_occupied, obstacle_evidence, ground_evidence,
    {0, 1, 2, 3}, changed);

  EXPECT_EQ(occupancy, (std::vector<std::int8_t>{0, 100, 0, 0}));
  EXPECT_EQ(changed, (std::vector<std::size_t>{0, 2}));
  for (const auto index : {0U, 2U, 3U}) {
    EXPECT_EQ(obstacle_evidence[index].count, 0);
    EXPECT_FALSE(ground_evidence[index].initialized);
  }
  EXPECT_EQ(obstacle_evidence[1].count, 1);
  EXPECT_TRUE(ground_evidence[1].initialized);
}

TEST(FootprintCells, SameFrameVetoPreservesOccupancyAndEvidence)
{
  std::vector<std::int8_t> occupancy{-1, 100};
  const std::vector<std::uint8_t> prior_occupied{0, 0};
  std::vector<bringup::ConsecutiveObservation> obstacle_evidence(2);
  std::vector<bringup::TimeWindowObservation> ground_evidence(2);
  for (std::size_t i = 0; i < occupancy.size(); ++i) {
    obstacle_evidence[i].observe(1, 1000000000, 150000000);
    ground_evidence[i].observe(1000000000, 2000000000);
  }
  std::vector<std::size_t> changed;

  bringup::clearFootprintCells(
    occupancy, prior_occupied, obstacle_evidence, ground_evidence,
    {0, 1}, changed, {1});

  EXPECT_EQ(occupancy, (std::vector<std::int8_t>{0, 100}));
  EXPECT_EQ(changed, (std::vector<std::size_t>{0}));
  EXPECT_EQ(obstacle_evidence[0].count, 0);
  EXPECT_FALSE(ground_evidence[0].initialized);
  EXPECT_EQ(obstacle_evidence[1].count, 1);
  EXPECT_TRUE(ground_evidence[1].initialized);
}

TEST(FootprintCells, EmptyCellSetLeavesEvidenceUnchanged)
{
  std::vector<std::int8_t> occupancy{-1};
  const std::vector<std::uint8_t> prior_occupied{0};
  std::vector<bringup::ConsecutiveObservation> obstacle_evidence(1);
  std::vector<bringup::TimeWindowObservation> ground_evidence(1);
  obstacle_evidence[0].observe(1, 1000000000, 150000000);
  ground_evidence[0].observe(1000000000, 2000000000);
  std::vector<std::size_t> changed;

  bringup::clearFootprintCells(
    occupancy, prior_occupied, obstacle_evidence, ground_evidence, {}, changed);

  EXPECT_EQ(occupancy[0], -1);
  EXPECT_EQ(obstacle_evidence[0].count, 1);
  EXPECT_TRUE(ground_evidence[0].initialized);
  EXPECT_TRUE(changed.empty());
}

TEST(FrontierSelection, ChoosesAFreeCellAtTheNearestFrontier)
{
  nav_msgs::msg::OccupancyGrid map;
  map.info.width = 5;
  map.info.height = 5;
  map.info.resolution = 1.0F;
  map.info.origin.orientation.w = 1.0;
  map.data.assign(25, -1);
  map.data[2 * 5 + 1] = 0;
  map.data[2 * 5 + 2] = 0;
  map.data[2 * 5 + 3] = 0;

  const auto goal = bringup::selectFrontierGoal(map, 1.5, 2.5, 3);

  ASSERT_TRUE(goal);
  EXPECT_EQ(goal->frontier_cells, 3U);
  EXPECT_DOUBLE_EQ(goal->x, 1.5);
  EXPECT_DOUBLE_EQ(goal->y, 2.5);
}

TEST(FrontierSelection, ExcludesFailedApproachWithoutSkippingWholeFrontier)
{
  nav_msgs::msg::OccupancyGrid map;
  map.info.width = 7;
  map.info.height = 7;
  map.info.resolution = 1.0F;
  map.info.origin.orientation.w = 1.0;
  map.data.assign(49, -1);
  for (const auto x : {1, 2, 3}) {
    map.data[1 * 7 + x] = 0;
    map.data[5 * 7 + x] = 0;
  }
  const std::vector<std::pair<double, double>> visited{{2.5, 1.5}};

  const auto goal = bringup::selectFrontierGoal(map, 1.5, 1.5, 3, visited, 0.1);

  ASSERT_TRUE(goal);
  EXPECT_DOUBLE_EQ(goal->x, 1.5);
  EXPECT_DOUBLE_EQ(goal->y, 1.5);
}

TEST(FrontierSelection, RejectsFootprintEnteringUnknownMapCells)
{
  nav_msgs::msg::OccupancyGrid map;
  map.info.width = 6;
  map.info.height = 6;
  map.info.resolution = 1.0F;
  map.info.origin.orientation.w = 1.0;
  map.data.assign(36, 0);
  auto costmap = map;
  const std::vector<bringup::FrontierFootprintPoint> footprint{
    {0.7, 0.2}, {0.7, -0.2}, {-0.4, -0.2}, {-0.4, 0.2}};

  map.data[2 * 6 + 3] = -1;
  EXPECT_FALSE(bringup::detail::footprintIsTraversable(
      map, costmap, 2.5, 2.5, 0.0, footprint));

  map.data[2 * 6 + 3] = 0;
  EXPECT_TRUE(bringup::detail::footprintIsTraversable(
      map, costmap, 2.5, 2.5, 0.0, footprint));

  costmap.data[2 * 6 + 2] = -1;
  EXPECT_FALSE(bringup::detail::footprintIsTraversable(
      map, costmap, 2.5, 2.5, 0.0, footprint));
}

TEST(FrontierSelection, DoesNotRouteThroughUnknownMapCells)
{
  nav_msgs::msg::OccupancyGrid map;
  map.info.width = 7;
  map.info.height = 5;
  map.info.resolution = 1.0F;
  map.info.origin.orientation.w = 1.0;
  map.data.assign(35, -1);
  map.data[2 * 7 + 1] = 0;
  for (const auto x : {4, 5, 6}) {
    map.data[2 * 7 + x] = 0;
  }
  auto costmap = map;
  std::fill(costmap.data.begin(), costmap.data.end(), 0);
  const auto prior = map;

  const auto goal = bringup::selectFrontierGoal(
    map, costmap, prior, 1.5, 2.5, 3, 0.0);

  EXPECT_FALSE(goal);
}

TEST(MapFusionEvidence, RequiresRepeatedGroundForUnknownAndOccupiedCells)
{
  EXPECT_FALSE(bringup::groundEvidenceUpdate(-1, false, 2, 3, 5));
  ASSERT_TRUE(bringup::groundEvidenceUpdate(-1, false, 3, 3, 5));
  EXPECT_EQ(*bringup::groundEvidenceUpdate(-1, false, 3, 3, 5), 0);
  EXPECT_FALSE(bringup::groundEvidenceUpdate(100, false, 4, 3, 5));
  ASSERT_TRUE(bringup::groundEvidenceUpdate(100, false, 5, 3, 5));
  EXPECT_EQ(*bringup::groundEvidenceUpdate(100, false, 5, 3, 5), 0);
}

TEST(MapFusionEvidence, SameScanNongroundVetoesClearingAndKnownFreeStaysFree)
{
  EXPECT_FALSE(bringup::groundEvidenceUpdate(100, true, 5, 3, 5));
  EXPECT_FALSE(bringup::groundEvidenceUpdate(0, false, 255, 3, 5));
}

TEST(MapFusionEvidence, SameScanVetoRestartsGroundClearEvidence)
{
  constexpr int free_observations = 3;
  constexpr int clear_observations = 5;
  constexpr std::int64_t evidence_window_ns = 2000000000;
  std::int8_t occupancy = 100;
  bringup::TimeWindowObservation ground_evidence;

  for (std::uint64_t observation = 1; observation <= 4; ++observation) {
    const auto count = ground_evidence.observe(
      1000000000 + static_cast<std::int64_t>(observation) * 300000000,
      evidence_window_ns);
    EXPECT_EQ(count, observation);
    EXPECT_FALSE(bringup::groundEvidenceUpdate(
      occupancy, false, count, free_observations, clear_observations));
  }

  // groundCallback resets this cell's evidence when ground and ground_veto overlap.
  ground_evidence.reset();
  EXPECT_FALSE(bringup::groundEvidenceUpdate(
    occupancy, true, clear_observations, free_observations, clear_observations));

  for (std::uint64_t observation = 1; observation <= 4; ++observation) {
    const auto count = ground_evidence.observe(
      2500000000 + static_cast<std::int64_t>(observation) * 300000000,
      evidence_window_ns);
    EXPECT_EQ(count, observation);
    EXPECT_FALSE(bringup::groundEvidenceUpdate(
      occupancy, false, count, free_observations, clear_observations));
    EXPECT_EQ(occupancy, 100);
  }
  const auto count = ground_evidence.observe(4000000000, evidence_window_ns);
  ASSERT_EQ(count, clear_observations);
  const auto updated = bringup::groundEvidenceUpdate(
    occupancy, false, count, free_observations, clear_observations);
  ASSERT_TRUE(updated);
  EXPECT_EQ(*updated, 0);
}

TEST(MapFusionEvidence, FreeRaysExcludeReturnCellAndClipToMapBounds)
{
  EXPECT_EQ(
    bringup::rasterizeFreeRay({1, 1}, {4, 1}, 6, 3),
    (std::vector<std::size_t>{7, 8, 9}));
  EXPECT_EQ(
    bringup::rasterizeFreeRay({1, 1}, {4, 4}, 4, 4),
    (std::vector<std::size_t>{5, 10, 15}));
  EXPECT_EQ(
    bringup::rasterizeFreeRay({-2, 1}, {2, 1}, 4, 3),
    (std::vector<std::size_t>{4, 5}));
  const std::vector<std::uint8_t> online_occupied(18, 0);
  auto eligible = online_occupied;
  eligible[8] = 1;
  EXPECT_EQ(
    bringup::rasterizeFreeRay({1, 1}, {4, 1}, 6, 3, &eligible),
    (std::vector<std::size_t>{8}));
}

TEST(MapFusionEvidence, RayEvidenceClearsOnlyOnlineNonPriorOccupiedCells)
{
  bringup::ConsecutiveObservation free_ray_evidence;
  EXPECT_EQ(free_ray_evidence.observe(1, 1000000000, 150000000), 1);
  EXPECT_FALSE(bringup::onlineObstacleRayClearUpdate(
      100, false, true, free_ray_evidence.count, 3));
  EXPECT_EQ(free_ray_evidence.observe(2, 1100000000, 150000000), 2);
  EXPECT_FALSE(bringup::onlineObstacleRayClearUpdate(
      100, false, true, free_ray_evidence.count, 3));
  EXPECT_EQ(free_ray_evidence.observe(3, 1200000000, 150000000), 3);
  ASSERT_TRUE(bringup::onlineObstacleRayClearUpdate(
      100, false, true, free_ray_evidence.count, 3));
  EXPECT_EQ(*bringup::onlineObstacleRayClearUpdate(
      100, false, true, free_ray_evidence.count, 3), 0);
  EXPECT_FALSE(bringup::onlineObstacleRayClearUpdate(100, true, true, 10, 3));
  EXPECT_FALSE(bringup::onlineObstacleRayClearUpdate(100, false, false, 10, 3));
  EXPECT_FALSE(bringup::onlineObstacleRayClearUpdate(0, false, true, 10, 3));
}

TEST(MapFusionEvidence, SameFrameObstacleHitVetoesRayUntilObstacleDisappears)
{
  constexpr std::size_t target_cell = 1;
  const std::vector<std::uint8_t> online_occupied{0, 1, 0, 0};
  const auto ray_cells = bringup::rasterizeFreeRay(
    {0, 0}, {3, 0}, 4, 1, &online_occupied);
  ASSERT_EQ(ray_cells, (std::vector<std::size_t>{target_cell}));

  bringup::ConsecutiveObservation clear_evidence;
  std::int8_t occupancy = 100;
  bool online_occupied_flag = true;
  for (std::uint64_t frame = 1; frame <= 5; ++frame) {
    const auto free_cells = bringup::excludeCurrentObstacleHitsFromRayClears(
      ray_cells, {target_cell});
    EXPECT_TRUE(free_cells.empty());
    clear_evidence.reset();
    EXPECT_FALSE(bringup::onlineObstacleRayClearUpdate(
      occupancy, false, online_occupied_flag, clear_evidence.count, 5));
    EXPECT_EQ(occupancy, 100);
  }

  for (std::uint64_t frame = 6; frame <= 10; ++frame) {
    const auto free_cells = bringup::excludeCurrentObstacleHitsFromRayClears(ray_cells, {});
    ASSERT_EQ(free_cells, (std::vector<std::size_t>{target_cell}));
    const auto clear_count = clear_evidence.observe(
      frame, static_cast<std::int64_t>(frame) * 100000000, 150000000);
    const auto update = bringup::onlineObstacleRayClearUpdate(
      occupancy, false, online_occupied_flag, clear_count, 5);
    if (update) {
      occupancy = *update;
      online_occupied_flag = false;
    }
    if (frame < 10) EXPECT_FALSE(update);
    else ASSERT_TRUE(update);
  }
  EXPECT_EQ(occupancy, 0);
}

TEST(MapFusionEvidence, OlderQueuedRaysCannotClearAfterANewerObstacleHit)
{
  constexpr std::size_t target_cell = 1;
  const std::vector<std::uint8_t> online_occupied{0, 1, 0};
  const auto ray_cells = bringup::rasterizeFreeRay({0, 0}, {2, 0}, 3, 1, &online_occupied);
  ASSERT_EQ(ray_cells, (std::vector<std::size_t>{target_cell}));

  // The newer hit callback runs before the old synchronized ray frames are retried.
  constexpr std::int64_t newer_hit_stamp_ns = 1600000000;
  std::int64_t latest_obstacle_hit_stamp_ns = newer_hit_stamp_ns;
  bringup::ConsecutiveObservation ray_clear_evidence;
  std::int8_t occupancy = 100;
  bool online_occupied_flag = true;
  ray_clear_evidence.reset();

  for (std::uint64_t frame = 1; frame <= 5; ++frame) {
    const auto old_ray_stamp_ns = 1000000000 + static_cast<std::int64_t>(frame) * 100000000;
    ASSERT_LT(old_ray_stamp_ns, newer_hit_stamp_ns);
    for (const auto index : ray_cells) {
      if (bringup::obstacleHitVetoesRayClear(
          old_ray_stamp_ns, latest_obstacle_hit_stamp_ns))
      {
        continue;
      }
      ray_clear_evidence.observe(frame, old_ray_stamp_ns, 150000000);
      const auto update = bringup::onlineObstacleRayClearUpdate(
        occupancy, false, online_occupied_flag, ray_clear_evidence.count, 5);
      if (update) {
        occupancy = *update;
        online_occupied_flag = false;
      }
    }
  }

  EXPECT_EQ(ray_clear_evidence.count, 0);
  EXPECT_EQ(occupancy, 100);
  EXPECT_TRUE(online_occupied_flag);
}

TEST(MapFusionEvidence, SameFrameObstacleVetoCoversBeyondGlobalObstacleRange)
{
  constexpr double cell_resolution_m = 0.1;
  constexpr std::size_t obstacle_cell = 150;  // 15 m, beyond the 10 m marking range.
  constexpr std::size_t ray_endpoint_cell = 200;  // 20 m clearing range.
  static_assert(obstacle_cell * cell_resolution_m > 10.0);
  static_assert(ray_endpoint_cell * cell_resolution_m > 19.9 &&
    ray_endpoint_cell * cell_resolution_m < 20.1);
  std::vector<std::uint8_t> online_occupied(ray_endpoint_cell + 1, 0);
  online_occupied[obstacle_cell] = 1;
  const auto ray_cells = bringup::rasterizeFreeRay(
    {0, 0}, {static_cast<std::int64_t>(ray_endpoint_cell), 0},
    ray_endpoint_cell + 1, 1, &online_occupied);
  ASSERT_EQ(ray_cells, (std::vector<std::size_t>{obstacle_cell}));

  const auto free_cells = bringup::excludeCurrentObstacleHitsFromRayClears(
    ray_cells, {obstacle_cell});
  EXPECT_TRUE(free_cells.empty());
}

TEST(GlobalClearEventRetry, RetriesAcrossTemporaryHealthAndTransformFailures)
{
  using Queue = bringup::FreshRetryQueue<int>;
  using Clock = Queue::Clock;
  Queue events(4, std::chrono::seconds(2));
  const auto start = Clock::time_point{} + std::chrono::seconds(1);
  EXPECT_EQ(events.enqueue(42, 1000000000, 1000000000, start),
    Queue::EnqueueResult::ACCEPTED);

  int attempts = 0;
  std::int8_t occupancy = 100;
  bool online_occupied = true;
  EXPECT_EQ(events.retryAvailable(start + std::chrono::milliseconds(100), 1100000000,
      [&attempts](const int &, std::int64_t stamp_ns, Clock::time_point) {
        ++attempts;
        EXPECT_EQ(stamp_ns, 1000000000);
        return false;  // Localization is not healthy yet.
      }), 0U);
  EXPECT_EQ(events.size(), 1U);

  EXPECT_EQ(events.retryAvailable(start + std::chrono::milliseconds(200), 1200000000,
      [&attempts](const int &, std::int64_t, Clock::time_point) {
        ++attempts;
        return false;  // The transform is still unavailable.
      }), 0U);
  EXPECT_EQ(events.size(), 1U);

  EXPECT_EQ(events.retryAvailable(start + std::chrono::milliseconds(300), 1300000000,
      [&attempts, &occupancy, &online_occupied](
        const int &, std::int64_t, Clock::time_point)
      {
        ++attempts;
        const auto update = bringup::onlineObstacleClearEventUpdate(
          occupancy, false, online_occupied);
        if (update) {
          occupancy = *update;
          online_occupied = false;
        }
        return true;  // Recovery lets the clear apply.
      }), 1U);
  EXPECT_EQ(attempts, 3);
  EXPECT_EQ(events.size(), 0U);
  EXPECT_EQ(occupancy, 0);
  EXPECT_FALSE(online_occupied);
}

TEST(GlobalClearEventRetry, NewerBlockerWithoutRayPairVetoesDeferredClearAfterRecovery)
{
  using EventQueue = bringup::FreshRetryQueue<std::size_t>;
  using HitQueue = bringup::FreshRetryQueue<bool>;
  using Clock = EventQueue::Clock;
  EventQueue events(4, std::chrono::seconds(2));
  HitQueue pending_hits(4, std::chrono::seconds(2));
  const auto start = Clock::time_point{} + std::chrono::seconds(1);
  constexpr std::size_t target_cell = 3;
  EXPECT_EQ(events.enqueue(target_cell, 1000000000, 1000000000, start),
    EventQueue::EnqueueResult::ACCEPTED);
  // The blocker stream is independent of the best-effort synchronized ray pair.
  EXPECT_EQ(pending_hits.enqueue(false, 1000000000, 1000000000, start),
    HitQueue::EnqueueResult::ACCEPTED);
  EXPECT_EQ(pending_hits.enqueue(true, 1200000000, 1200000000,
      start + std::chrono::milliseconds(100)), HitQueue::EnqueueResult::ACCEPTED);

  EXPECT_EQ(pending_hits.retryAvailable(start + std::chrono::milliseconds(150), 1200000000,
      [](const bool &, std::int64_t, Clock::time_point) {
        return false;  // Blocker evidence cannot be mapped while TF is unavailable.
      }), 0U);
  EXPECT_EQ(events.size(), 1U);
  EXPECT_EQ(pending_hits.size(), 2U);

  std::int64_t latest_obstacle_hit_stamp_ns = 0;
  std::int64_t obstacle_evidence_watermark_ns = 0;
  constexpr std::int64_t confirmed_published_cutoff_ns = 1200000000;
  bool same_frame_obstacle_evidence_processed = false;
  std::int8_t occupancy = 100;
  bool online_occupied = true;
  EXPECT_EQ(pending_hits.retryAvailable(start + std::chrono::milliseconds(200), 1300000000,
      [&latest_obstacle_hit_stamp_ns, &same_frame_obstacle_evidence_processed,
        &obstacle_evidence_watermark_ns](
        const bool & contains_hit, std::int64_t hit_stamp_ns, Clock::time_point) {
        if (hit_stamp_ns == 1000000000) same_frame_obstacle_evidence_processed = true;
        obstacle_evidence_watermark_ns =
          std::max(obstacle_evidence_watermark_ns, hit_stamp_ns);
        if (contains_hit) {
          latest_obstacle_hit_stamp_ns = std::max(latest_obstacle_hit_stamp_ns, hit_stamp_ns);
        }
        return true;  // Replay blocker evidence after TF recovers.
      }), 2U);
  EXPECT_EQ(events.retryAvailable(start + std::chrono::milliseconds(200), 1300000000,
      [&latest_obstacle_hit_stamp_ns, &same_frame_obstacle_evidence_processed, &occupancy,
        &obstacle_evidence_watermark_ns, confirmed_published_cutoff_ns, &online_occupied,
        &pending_hits, target_cell](
        const std::size_t & cell, std::int64_t event_stamp_ns, Clock::time_point) {
        if (!bringup::globalClearEvidenceReady(
            event_stamp_ns, same_frame_obstacle_evidence_processed,
            confirmed_published_cutoff_ns, obstacle_evidence_watermark_ns,
            pending_hits.hasSourceStampAtOrAfter(event_stamp_ns), 0))
        {
          return false;
        }
        if (cell != target_cell || bringup::obstacleHitVetoesClearEvent(
            event_stamp_ns, latest_obstacle_hit_stamp_ns))
        {
          return true;
        }
        const auto update = bringup::onlineObstacleClearEventUpdate(
          occupancy, false, online_occupied);
        if (update) {
          occupancy = *update;
          online_occupied = false;
        }
        return true;
      }), 1U);
  EXPECT_EQ(pending_hits.size(), 0U);
  EXPECT_EQ(occupancy, 100);
  EXPECT_TRUE(online_occupied);
  EXPECT_TRUE(bringup::obstacleEvidenceFrameReady(
    1000000000, true, 1200000000, 1200000000));
  EXPECT_TRUE(bringup::obstacleHitVetoesClearEvent(1200000000, 1200000000));
}

TEST(GlobalClearEventRetry, WaitsUntilSameFrameBlockerEvidenceIsAvailable)
{
  EXPECT_FALSE(bringup::obstacleEvidenceFrameReady(1000000000, false, 1000000000, 1000000000));
  EXPECT_FALSE(bringup::obstacleEvidenceFrameReady(1000000000, true, 900000000, 900000000));
  EXPECT_FALSE(bringup::obstacleEvidenceFrameReady(1000000000, true, 1100000000, 1000000000));
  EXPECT_TRUE(bringup::obstacleEvidenceFrameReady(1000000000, true, 1000000000, 1000000000));
  EXPECT_FALSE(bringup::globalClearEvidenceReady(
      1000000000, true, 1000000000, 900000000, false, 0));
  EXPECT_FALSE(bringup::globalClearEvidenceReady(
      1000000000, true, 1100000000, 1100000000, true, 0));
  EXPECT_FALSE(bringup::globalClearEvidenceReady(
      1000000000, true, 1100000000, 1100000000, false, 1200000000));
  EXPECT_TRUE(bringup::globalClearEvidenceReady(
      1300000000, true, 1400000000, 1400000000, false, 1200000000));
}

TEST(GlobalClearEventRetry, RequiresPublishedFenceAndProcessedEvidenceThroughItsCutoff)
{
  constexpr std::int64_t clear_stamp_ns = 1000000000;
  const bool same_frame_veto_processed = true;
  constexpr std::int64_t published_cutoff_ns = 1200000000;
  std::int64_t obstacle_evidence_watermark_ns = clear_stamp_ns;

  // One processed post-event frame is insufficient when the publisher confirms a later cutoff.
  EXPECT_FALSE(bringup::globalClearEvidenceReady(
      clear_stamp_ns, same_frame_veto_processed, published_cutoff_ns,
      obstacle_evidence_watermark_ns, false, 0));
  obstacle_evidence_watermark_ns = 1100000000;
  EXPECT_FALSE(bringup::globalClearEvidenceReady(
      clear_stamp_ns, same_frame_veto_processed, published_cutoff_ns,
      obstacle_evidence_watermark_ns, false, 0));
  obstacle_evidence_watermark_ns = published_cutoff_ns;
  EXPECT_TRUE(bringup::globalClearEvidenceReady(
      clear_stamp_ns, same_frame_veto_processed, published_cutoff_ns,
      obstacle_evidence_watermark_ns, false, 0));
}

TEST(ObstacleEvidenceSequence, FenceRejectsMissingFrameUntilTheGapIsFilled)
{
  bringup::ObstacleEvidenceSequenceTracker tracker;
  const bringup::ObstacleEvidenceFrameMetadata event_frame{17, 40};
  const bringup::ObstacleEvidenceFrameMetadata later_frame{17, 42};
  EXPECT_EQ(tracker.observe(event_frame),
    bringup::ObstacleEvidenceSequenceTracker::ObserveResult::PUBLISHER_CHANGED);
  EXPECT_TRUE(tracker.markProcessed(event_frame));
  EXPECT_EQ(tracker.observe(later_frame),
    bringup::ObstacleEvidenceSequenceTracker::ObserveResult::ACCEPTED);
  EXPECT_TRUE(tracker.markProcessed(later_frame));
  const bringup::ObstacleEvidenceFence fence{17, 42, 1200000000};
  ASSERT_TRUE(tracker.establishFence(fence));

  // A timestamp watermark could pass here, but sequence 41 was overwritten or lost.
  EXPECT_FALSE(tracker.sequenceRangeProcessed(17, 40, 42));
  const bringup::ObstacleEvidenceFrameMetadata recovered_frame{17, 41};
  EXPECT_EQ(tracker.observe(recovered_frame),
    bringup::ObstacleEvidenceSequenceTracker::ObserveResult::ACCEPTED);
  EXPECT_TRUE(tracker.markProcessed(recovered_frame));
  EXPECT_TRUE(tracker.sequenceRangeProcessed(17, 40, 42));
}

TEST(ObstacleEvidenceSequence, PublisherRestartInvalidatesOldFenceAndSequenceRange)
{
  bringup::ObstacleEvidenceSequenceTracker tracker;
  const bringup::ObstacleEvidenceFrameMetadata old_frame{101, 8};
  EXPECT_EQ(tracker.observe(old_frame),
    bringup::ObstacleEvidenceSequenceTracker::ObserveResult::PUBLISHER_CHANGED);
  ASSERT_TRUE(tracker.markProcessed(old_frame));
  ASSERT_TRUE(tracker.establishFence({101, 8, 1000000000}));

  const bringup::ObstacleEvidenceFrameMetadata new_frame{202, 1};
  EXPECT_EQ(tracker.observe(new_frame),
    bringup::ObstacleEvidenceSequenceTracker::ObserveResult::PUBLISHER_CHANGED);
  ASSERT_TRUE(tracker.markProcessed(new_frame));
  EXPECT_FALSE(tracker.establishFence({101, 9, 1100000000}));
  EXPECT_FALSE(tracker.sequenceRangeProcessed(101, 8, 9));
  ASSERT_TRUE(tracker.establishFence({202, 1, 1200000000}));
  EXPECT_TRUE(tracker.sequenceRangeProcessed(202, 1, 1));
}

TEST(GlobalClearEventRetry, RefreshesFenceAfterWaitingSequenceRangeRecovers)
{
  bringup::ObstacleEvidenceSequenceTracker sequence_tracker;
  bringup::ClearEventFenceRefreshTracker refresh_tracker;
  const bringup::ObstacleEvidenceFrameMetadata event_frame{303, 1};
  const bringup::ObstacleEvidenceFrameMetadata later_frame{303, 3};
  ASSERT_EQ(sequence_tracker.observe(event_frame),
    bringup::ObstacleEvidenceSequenceTracker::ObserveResult::PUBLISHER_CHANGED);
  ASSERT_TRUE(sequence_tracker.markProcessed(event_frame));
  ASSERT_EQ(sequence_tracker.observe(later_frame),
    bringup::ObstacleEvidenceSequenceTracker::ObserveResult::ACCEPTED);
  ASSERT_TRUE(sequence_tracker.markProcessed(later_frame));
  ASSERT_TRUE(sequence_tracker.establishFence({303, 3, 1300000000}));

  // Sequence 2 is delayed, so the first fence cannot be used.
  EXPECT_FALSE(sequence_tracker.sequenceRangeProcessed(303, 1, 3));
  EXPECT_FALSE(refresh_tracker.sequenceRangeReady(7, false));

  const bringup::ObstacleEvidenceFrameMetadata recovered_frame{303, 2};
  ASSERT_EQ(sequence_tracker.observe(recovered_frame),
    bringup::ObstacleEvidenceSequenceTracker::ObserveResult::ACCEPTED);
  ASSERT_TRUE(sequence_tracker.markProcessed(recovered_frame));
  ASSERT_TRUE(sequence_tracker.sequenceRangeProcessed(303, 1, 3));
  // Recovery of the old range still requires a newer publisher boundary.
  EXPECT_FALSE(refresh_tracker.sequenceRangeReady(7, true));
  ASSERT_TRUE(refresh_tracker.hasEventsNeedingFreshFence());
  const auto refresh_ids = refresh_tracker.eventsNeedingFreshFenceThrough(7);
  ASSERT_EQ(refresh_ids.size(), 1U);
  EXPECT_EQ(refresh_ids.front(), 7U);

  // Obstacle sequence 4 was published while sequence 2 was delayed. The refresh
  // fence sees it even though the independent subscriber has not handled it yet.
  ASSERT_TRUE(sequence_tracker.establishFence({303, 4, 1400000000}));
  refresh_tracker.acceptFreshFence(refresh_ids);
  EXPECT_FALSE(sequence_tracker.sequenceRangeProcessed(303, 1, 4));
  EXPECT_FALSE(refresh_tracker.sequenceRangeReady(7, false));
  EXPECT_FALSE(bringup::clearEventCoveredByFence(8, 7));

  const bringup::ObstacleEvidenceFrameMetadata published_but_unprocessed{303, 4};
  ASSERT_EQ(sequence_tracker.observe(published_but_unprocessed),
    bringup::ObstacleEvidenceSequenceTracker::ObserveResult::ACCEPTED);
  ASSERT_TRUE(sequence_tracker.markProcessed(published_but_unprocessed));
  ASSERT_TRUE(sequence_tracker.sequenceRangeProcessed(303, 1, 4));
  // Sequence 5 is published during the refreshed-fence wait. Recovery of sequence
  // 4 must request another boundary instead of releasing the older cutoff.
  EXPECT_FALSE(refresh_tracker.sequenceRangeReady(7, true));
  ASSERT_TRUE(refresh_tracker.hasEventsNeedingFreshFence());
  const auto second_refresh_ids = refresh_tracker.eventsNeedingFreshFenceThrough(7);
  ASSERT_EQ(second_refresh_ids.size(), 1U);
  ASSERT_TRUE(sequence_tracker.establishFence({303, 5, 1500000000}));
  refresh_tracker.acceptFreshFence(second_refresh_ids);
  EXPECT_FALSE(sequence_tracker.sequenceRangeProcessed(303, 1, 5));
  EXPECT_FALSE(refresh_tracker.sequenceRangeReady(7, false));

  const bringup::ObstacleEvidenceFrameMetadata second_published_but_unprocessed{303, 5};
  ASSERT_EQ(sequence_tracker.observe(second_published_but_unprocessed),
    bringup::ObstacleEvidenceSequenceTracker::ObserveResult::ACCEPTED);
  ASSERT_TRUE(sequence_tracker.markProcessed(second_published_but_unprocessed));
  ASSERT_TRUE(sequence_tracker.sequenceRangeProcessed(303, 1, 5));
  // The second wait also requires a boundary after it closes. A request with no
  // newer blocker frame can then confirm the complete interval and release it.
  EXPECT_FALSE(refresh_tracker.sequenceRangeReady(7, true));
  ASSERT_TRUE(refresh_tracker.hasEventsNeedingFreshFence());
  const auto final_refresh_ids = refresh_tracker.eventsNeedingFreshFenceThrough(7);
  ASSERT_EQ(final_refresh_ids.size(), 1U);
  ASSERT_TRUE(sequence_tracker.establishFence({303, 5, 1500000000}));
  refresh_tracker.acceptFreshFence(final_refresh_ids);
  EXPECT_TRUE(sequence_tracker.sequenceRangeProcessed(303, 1, 5));
  EXPECT_TRUE(refresh_tracker.sequenceRangeReady(7, true));
}

TEST(GlobalClearEventRetry, DelayedFenceResponseOnlyConfirmsItsCapturedBatch)
{
  std::uint64_t latest_event_id = 1;
  const auto request_batch_id = latest_event_id;
  // A later event arrives while the service response is delayed.
  ++latest_event_id;
  std::uint64_t confirmed_batch_id = request_batch_id;
  EXPECT_TRUE(bringup::clearEventCoveredByFence(1, confirmed_batch_id));
  EXPECT_FALSE(bringup::clearEventCoveredByFence(2, confirmed_batch_id));

  const auto next_request_batch_id = latest_event_id;
  confirmed_batch_id = next_request_batch_id;
  EXPECT_TRUE(bringup::clearEventCoveredByFence(2, confirmed_batch_id));
}

TEST(GlobalClearEventRetry, FenceIsRequiredWhenBlockerCallbackRunsFirst)
{
  constexpr std::int64_t clear_stamp_ns = 1000000000;
  constexpr std::int64_t blocker_stamp_ns = 1200000000;
  const bool same_frame_veto_processed = true;
  const std::int64_t obstacle_evidence_watermark_ns = blocker_stamp_ns;

  // Processing the blocker first does not itself prove that the source has finished publishing.
  EXPECT_FALSE(bringup::globalClearEvidenceReady(
      clear_stamp_ns, same_frame_veto_processed, 0,
      obstacle_evidence_watermark_ns, false, 0));
  // The later service response fences the stream at a boundary already processed by fusion.
  EXPECT_TRUE(bringup::globalClearEvidenceReady(
      clear_stamp_ns, same_frame_veto_processed, blocker_stamp_ns,
      obstacle_evidence_watermark_ns, false, 0));
}

TEST(GlobalClearEventRetry, TimerWaitsForPublishedButUnprocessedNewerBlocker)
{
  using Queue = bringup::FreshRetryQueue<int>;
  using Clock = Queue::Clock;
  Queue events(2, std::chrono::seconds(2));
  const auto start = Clock::time_point{} + std::chrono::seconds(1);
  constexpr std::int64_t clear_stamp_ns = 1000000000;
  constexpr std::int64_t first_newer_frame_stamp_ns = 1100000000;
  constexpr std::int64_t newer_blocker_stamp_ns = 1200000000;
  EXPECT_EQ(events.enqueue(1, clear_stamp_ns, clear_stamp_ns, start),
    Queue::EnqueueResult::ACCEPTED);

  std::int64_t obstacle_evidence_watermark_ns = clear_stamp_ns;
  std::int64_t confirmed_published_cutoff_ns = 0;
  std::int64_t latest_obstacle_hit_stamp_ns = 0;
  std::int8_t occupancy = 100;
  bool online_occupied = true;
  EXPECT_EQ(events.retryAvailable(start + std::chrono::milliseconds(100), 1100000000,
      [&confirmed_published_cutoff_ns, &obstacle_evidence_watermark_ns,
        &latest_obstacle_hit_stamp_ns, &occupancy,
        &online_occupied](const int &, std::int64_t event_stamp_ns, Clock::time_point) {
        if (!bringup::globalClearEvidenceReady(
            event_stamp_ns, true, confirmed_published_cutoff_ns,
            obstacle_evidence_watermark_ns, false, 0)) {
          return false;
        }
        if (!bringup::obstacleHitVetoesClearEvent(
            event_stamp_ns, latest_obstacle_hit_stamp_ns)) {
          const auto update = bringup::onlineObstacleClearEventUpdate(
            occupancy, false, online_occupied);
          if (update) {
            occupancy = *update;
            online_occupied = false;
          }
        }
        return true;
      }), 0U);
  EXPECT_EQ(events.size(), 1U);
  EXPECT_EQ(occupancy, 100);

  // A first later frame was processed, but the source-side fence also reports a second,
  // newer obstacle frame whose fusion callback has not run yet.
  obstacle_evidence_watermark_ns = first_newer_frame_stamp_ns;
  confirmed_published_cutoff_ns = newer_blocker_stamp_ns;
  EXPECT_EQ(events.retryAvailable(start + std::chrono::milliseconds(150), 1150000000,
      [&confirmed_published_cutoff_ns, &obstacle_evidence_watermark_ns,
        &latest_obstacle_hit_stamp_ns, &occupancy, &online_occupied](
        const int &, std::int64_t event_stamp_ns, Clock::time_point) {
        if (!bringup::globalClearEvidenceReady(
            event_stamp_ns, true, confirmed_published_cutoff_ns,
            obstacle_evidence_watermark_ns, false, 0)) {
          return false;
        }
        if (!bringup::obstacleHitVetoesClearEvent(
            event_stamp_ns, latest_obstacle_hit_stamp_ns)) {
          const auto update = bringup::onlineObstacleClearEventUpdate(
            occupancy, false, online_occupied);
          if (update) {
            occupancy = *update;
            online_occupied = false;
          }
        }
        return true;
      }), 0U);
  EXPECT_EQ(events.size(), 1U);
  EXPECT_EQ(occupancy, 100);

  // The timer is retried after the blocker callback, then the updated hit vetoes the clear.
  latest_obstacle_hit_stamp_ns = newer_blocker_stamp_ns;
  obstacle_evidence_watermark_ns = newer_blocker_stamp_ns;
  EXPECT_EQ(events.retryAvailable(start + std::chrono::milliseconds(200), 1200000000,
      [&confirmed_published_cutoff_ns, &obstacle_evidence_watermark_ns,
        &latest_obstacle_hit_stamp_ns, &occupancy,
        &online_occupied](const int &, std::int64_t event_stamp_ns, Clock::time_point) {
        if (!bringup::globalClearEvidenceReady(
            event_stamp_ns, true, confirmed_published_cutoff_ns,
            obstacle_evidence_watermark_ns, false, 0)) {
          return false;
        }
        if (!bringup::obstacleHitVetoesClearEvent(
            event_stamp_ns, latest_obstacle_hit_stamp_ns)) {
          const auto update = bringup::onlineObstacleClearEventUpdate(
            occupancy, false, online_occupied);
          if (update) {
            occupancy = *update;
            online_occupied = false;
          }
        }
        return true;
      }), 1U);
  EXPECT_EQ(events.size(), 0U);
  EXPECT_EQ(occupancy, 100);
  EXPECT_TRUE(online_occupied);
}

TEST(GlobalClearEventRetry, OlderUnavailableBlockerDoesNotBlockLaterCompleteClear)
{
  using Queue = bringup::FreshRetryQueue<int>;
  using Clock = Queue::Clock;
  Queue pending_hits(4, std::chrono::seconds(2));
  Queue pending_clears(4, std::chrono::seconds(2));
  const auto start = Clock::time_point{} + std::chrono::seconds(1);
  EXPECT_EQ(pending_hits.enqueue(1, 1000000000, 1000000000, start),
    Queue::EnqueueResult::ACCEPTED);
  EXPECT_EQ(pending_clears.enqueue(2, 2000000000, 2000000000, start),
    Queue::EnqueueResult::ACCEPTED);

  EXPECT_EQ(pending_hits.retryAvailable(start + std::chrono::milliseconds(100), 2100000000,
      [](const int &, std::int64_t, Clock::time_point) {
        return false;  // The older blocker frame still has no TF.
      }), 0U);
  EXPECT_TRUE(pending_hits.hasSourceStampAtOrAfter(1000000000));
  EXPECT_FALSE(pending_hits.hasSourceStampAtOrAfter(2000000000));

  bool clear_applied = false;
  const bool same_frame_veto_processed = true;
  const std::int64_t obstacle_evidence_watermark_ns = 2100000000;
  const std::int64_t confirmed_published_cutoff_ns = 2100000000;
  EXPECT_EQ(pending_clears.retryAvailable(start + std::chrono::milliseconds(100), 2100000000,
      [&pending_hits, &clear_applied, same_frame_veto_processed,
        obstacle_evidence_watermark_ns, confirmed_published_cutoff_ns](
        const int &, std::int64_t clear_stamp_ns, Clock::time_point) {
        if (!bringup::globalClearEvidenceReady(
            clear_stamp_ns, same_frame_veto_processed, confirmed_published_cutoff_ns,
            obstacle_evidence_watermark_ns,
            pending_hits.hasSourceStampAtOrAfter(clear_stamp_ns), 0)) {
          return false;
        }
        clear_applied = true;
        return true;
      }), 1U);
  EXPECT_TRUE(clear_applied);
  EXPECT_EQ(pending_hits.size(), 1U);
}

TEST(FreshRetryQueue, RetriesNewestClearEventBeforeOlderPendingEvents)
{
  using Queue = bringup::FreshRetryQueue<int>;
  using Clock = Queue::Clock;
  Queue events(4, std::chrono::seconds(2));
  const auto start = Clock::time_point{} + std::chrono::seconds(1);
  EXPECT_EQ(events.enqueue(1, 1000000000, 1000000000, start),
    Queue::EnqueueResult::ACCEPTED);
  EXPECT_EQ(events.enqueue(2, 1200000000, 1200000000,
      start + std::chrono::milliseconds(100)), Queue::EnqueueResult::ACCEPTED);

  std::vector<int> attempted;
  EXPECT_EQ(events.retryAvailableNewestFirst(
      start + std::chrono::milliseconds(150), 1250000000,
      [&attempted](const int & event, std::int64_t, Clock::time_point) {
        attempted.push_back(event);
        return true;
      }), 2U);
  EXPECT_EQ(attempted, (std::vector<int>{2, 1}));
}

TEST(FreshRetryQueue, NewestFirstRetryKeepsOldestFirstCapacityEviction)
{
  using Queue = bringup::FreshRetryQueue<int>;
  using Clock = Queue::Clock;
  Queue events(2, std::chrono::seconds(2), true);
  const auto start = Clock::time_point{} + std::chrono::seconds(1);
  EXPECT_EQ(events.enqueue(1, 1000000000, 1000000000, start),
    Queue::EnqueueResult::ACCEPTED);
  EXPECT_EQ(events.enqueue(2, 1200000000, 1200000000,
      start + std::chrono::milliseconds(100)), Queue::EnqueueResult::ACCEPTED);
  EXPECT_EQ(events.retryAvailableNewestFirst(
      start + std::chrono::milliseconds(150), 1250000000,
      [](const int &, std::int64_t, Clock::time_point) {return false;}), 0U);

  EXPECT_EQ(events.enqueue(3, 1300000000, 1300000000,
      start + std::chrono::milliseconds(200)), Queue::EnqueueResult::DROPPED_OLDEST);
  const auto discarded = events.takeDiscardedPayloads();
  EXPECT_EQ(discarded, (std::vector<int>{1}));
}

TEST(GlobalClearEventRetry, DiscardedBlockerEvidenceKeepsItsSourceTimeBoundary)
{
  using Queue = bringup::FreshRetryQueue<int>;
  using Clock = Queue::Clock;
  const auto start = Clock::time_point{} + std::chrono::seconds(1);
  Queue pending_hits(2, std::chrono::seconds(2));
  EXPECT_EQ(pending_hits.enqueue(1, 1100000000, 1000000000, start),
    Queue::EnqueueResult::ACCEPTED);
  EXPECT_EQ(pending_hits.enqueue(2, 1200000000, 1000000000,
      start + std::chrono::milliseconds(100)), Queue::EnqueueResult::ACCEPTED);

  EXPECT_EQ(pending_hits.retryAvailable(start + std::chrono::milliseconds(2100), 1500000000,
      [](const int &, std::int64_t, Clock::time_point) {
        return false;
      }), 0U);
  const auto incomplete_through_stamp_ns = pending_hits.takeDiscardedSourceStampNs();
  EXPECT_EQ(incomplete_through_stamp_ns, 1200000000);
  EXPECT_FALSE(bringup::globalClearEvidenceReady(
      1100000000, true, 1300000000, 1300000000, false, incomplete_through_stamp_ns));
  EXPECT_TRUE(bringup::globalClearEvidenceReady(
      1300000000, true, 1400000000, 1400000000, false, incomplete_through_stamp_ns));

  Queue overflow_hits(1, std::chrono::seconds(2));
  EXPECT_EQ(overflow_hits.enqueue(3, 1400000000, 1400000000, start),
    Queue::EnqueueResult::ACCEPTED);
  EXPECT_EQ(overflow_hits.enqueue(4, 1500000000, 1500000000,
      start + std::chrono::milliseconds(100)), Queue::EnqueueResult::DROPPED_OLDEST);
  const auto overflow_incomplete_through_stamp_ns =
    overflow_hits.takeDiscardedSourceStampNs();
  EXPECT_EQ(overflow_incomplete_through_stamp_ns, 1400000000);
  EXPECT_FALSE(bringup::globalClearEvidenceReady(
      1400000000, true, 1500000000, 1500000000, false,
      overflow_incomplete_through_stamp_ns));
  EXPECT_TRUE(bringup::globalClearEvidenceReady(
      1500000000, true, 1600000000, 1600000000, false,
      overflow_incomplete_through_stamp_ns));
}

TEST(GlobalRayRetry, HandlesEitherBlockerAndRayCallbackOrder)
{
  using Queue = bringup::FreshRetryQueue<int>;
  using Clock = Queue::Clock;
  constexpr std::int64_t frame_stamp_ns = 2000000000;
  const auto start = Clock::time_point{} + std::chrono::seconds(1);

  bool blocker_processed = false;
  int processed_rays = 0;
  const auto process_ray = [&blocker_processed, &processed_rays, frame_stamp_ns]() {
      if (!bringup::rayClearEvidenceReady(frame_stamp_ns, blocker_processed)) return false;
      ++processed_rays;
      return true;
    };

  // If the independent blocker callback runs first, the synchronized ray is usable immediately.
  blocker_processed = true;
  EXPECT_TRUE(process_ray());
  EXPECT_EQ(processed_rays, 1);

  // If the synchronized ray arrives first, it stays queued until the blocker callback completes.
  blocker_processed = false;
  Queue pending_rays(2, std::chrono::seconds(2));
  EXPECT_FALSE(process_ray());
  EXPECT_EQ(pending_rays.enqueue(1, frame_stamp_ns, frame_stamp_ns, start),
    Queue::EnqueueResult::ACCEPTED);
  blocker_processed = true;
  EXPECT_EQ(pending_rays.retryAvailable(start + std::chrono::milliseconds(100), frame_stamp_ns,
      [&process_ray](const int &, std::int64_t, Clock::time_point) {
        return process_ray();
      }), 1U);
  EXPECT_EQ(processed_rays, 2);
  EXPECT_EQ(pending_rays.size(), 0U);
}

TEST(GlobalClearEventRetry, ContinuesPastUnavailableQueueEntryInArrivalOrder)
{
  using Queue = bringup::FreshRetryQueue<int>;
  using Clock = Queue::Clock;
  Queue events(4, std::chrono::seconds(2));
  const auto start = Clock::time_point{} + std::chrono::seconds(1);
  EXPECT_EQ(events.enqueue(1, 1000000000, 1000000000, start),
    Queue::EnqueueResult::ACCEPTED);
  EXPECT_EQ(events.enqueue(2, 1100000000, 1100000000,
      start + std::chrono::milliseconds(50)), Queue::EnqueueResult::ACCEPTED);

  std::vector<int> attempted;
  EXPECT_EQ(events.retryAvailable(start + std::chrono::milliseconds(100), 1200000000,
      [&attempted](const int & event, std::int64_t, Clock::time_point) {
        attempted.push_back(event);
        return event == 2;  // Event 1's TF is unavailable; event 2 can apply.
      }), 1U);
  EXPECT_EQ(attempted, (std::vector<int>{1, 2}));
  EXPECT_EQ(events.size(), 1U);

  EXPECT_EQ(events.retryAvailable(start + std::chrono::milliseconds(200), 1300000000,
      [&attempted](const int & event, std::int64_t, Clock::time_point) {
        attempted.push_back(event);
        return true;
      }), 1U);
  EXPECT_EQ(attempted, (std::vector<int>{1, 2, 1}));
  EXPECT_EQ(events.size(), 0U);
}

TEST(GlobalClearEventRetry, DropsEvidenceWhenEitherSourceOrReceiveTimeExpires)
{
  using Queue = bringup::FreshRetryQueue<int>;
  using Clock = Queue::Clock;
  Queue events(2, std::chrono::seconds(2));
  const auto start = Clock::time_point{} + std::chrono::seconds(1);
  EXPECT_EQ(events.enqueue(17, 1000000000, 2800000000, start),
    Queue::EnqueueResult::ACCEPTED);

  int attempts = 0;
  EXPECT_EQ(events.retryAvailable(start + std::chrono::milliseconds(100), 3100000000,
      [&attempts](const int &, std::int64_t, Clock::time_point) {
        ++attempts;
        return true;
      }), 0U);
  EXPECT_EQ(attempts, 0);
  EXPECT_EQ(events.size(), 0U);

  EXPECT_EQ(events.enqueue(18, 1000000000, 1000000000,
      start + std::chrono::seconds(3)), Queue::EnqueueResult::ACCEPTED);
  EXPECT_EQ(events.retryAvailable(start + std::chrono::seconds(5), 1100000000,
      [&attempts](const int &, std::int64_t, Clock::time_point) {
        ++attempts;
        return true;
      }), 0U);
  EXPECT_EQ(attempts, 0);
  EXPECT_EQ(events.size(), 0U);
}

TEST(MapFusionEvidence, RestoresOnlineObstacleSourceWithoutPromotingItToPrior)
{
  const auto online_pixel = bringup::decodeOccupancySourcePixel(170U);
  ASSERT_TRUE(online_pixel);
  const auto online_occupied = bringup::restoreOccupancySourceFlags(*online_pixel, 100);
  ASSERT_TRUE(online_occupied);
  EXPECT_FALSE(online_occupied->prior);
  EXPECT_TRUE(online_occupied->online);

  const auto prior_pixel = bringup::decodeOccupancySourcePixel(85U);
  ASSERT_TRUE(prior_pixel);
  const auto prior_occupied = bringup::restoreOccupancySourceFlags(*prior_pixel, 100);
  ASSERT_TRUE(prior_occupied);
  EXPECT_TRUE(prior_occupied->prior);
  EXPECT_FALSE(prior_occupied->online);

  const auto both_pixel = bringup::decodeOccupancySourcePixel(255U);
  ASSERT_TRUE(both_pixel);
  const auto both_sources = bringup::restoreOccupancySourceFlags(*both_pixel, 100);
  ASSERT_TRUE(both_sources);
  EXPECT_TRUE(both_sources->prior);
  EXPECT_TRUE(both_sources->online);

  const auto cleared_online = bringup::restoreOccupancySourceFlags(*online_pixel, 0);
  ASSERT_TRUE(cleared_online);
  EXPECT_FALSE(cleared_online->prior);
  EXPECT_FALSE(cleared_online->online);
  EXPECT_FALSE(bringup::decodeOccupancySourcePixel(1U));
  EXPECT_FALSE(bringup::restoreOccupancySourceFlags(4U, 100));
}

TEST(MapFusionEvidence, ClearEventsRemoveOnlineOccupancyButPreservePriorOccupancy)
{
  const auto cleared = bringup::onlineObstacleClearEventUpdate(100, false, true);
  ASSERT_TRUE(cleared);
  EXPECT_EQ(*cleared, 0);
  EXPECT_FALSE(bringup::onlineObstacleClearEventUpdate(100, true, true));
  EXPECT_FALSE(bringup::onlineObstacleClearEventUpdate(100, false, false));
  EXPECT_FALSE(bringup::onlineObstacleClearEventUpdate(0, false, true));
}
