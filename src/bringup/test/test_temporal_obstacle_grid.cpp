#include <vector>

#include <gtest/gtest.h>

#include "bringup/temporal_obstacle_grid.hpp"

namespace
{
using bringup::ObstacleGridPoint;
using bringup::TemporalObstacleGrid;
using bringup::TemporalObstacleGridParameters;

const ObstacleGridPoint kOrigin{0.05, 0.05, 0.5};
const ObstacleGridPoint kHit{1.05, 0.05, 0.5};

TemporalObstacleGrid makeGrid(int clear_ray_frames = 2)
{
  TemporalObstacleGridParameters parameters;
  parameters.confirmation_window_frames = 5;
  parameters.confirmation_hits = 2;
  parameters.clear_ray_frames = clear_ray_frames;
  return TemporalObstacleGrid(parameters);
}

TEST(TemporalObstacleGrid, ConfirmsTwoHitsWithinSlidingWindow)
{
  auto grid = makeGrid();
  EXPECT_TRUE(grid.processFrame({kHit}, {}, kOrigin).empty());
  EXPECT_EQ(grid.processFrame({}, {}, kOrigin).size(), 0U);
  const auto confirmed = grid.processFrame({kHit}, {}, kOrigin);
  ASSERT_EQ(confirmed.size(), 1U);
  EXPECT_DOUBLE_EQ(confirmed.front().x, 1.05);
  EXPECT_DOUBLE_EQ(confirmed.front().y, 0.05);
}

TEST(TemporalObstacleGrid, CandidateExpiresOutsideHitWindow)
{
  auto grid = makeGrid();
  EXPECT_TRUE(grid.processFrame({kHit}, {}, kOrigin).empty());
  for (int i = 0; i < 5; ++i) {
    EXPECT_TRUE(grid.processFrame({}, {}, kOrigin).empty());
  }
  EXPECT_TRUE(grid.processFrame({kHit}, {}, kOrigin).empty());
}

TEST(TemporalObstacleGrid, ConfirmedCellPersistsWithoutRayEvidence)
{
  auto grid = makeGrid();
  grid.processFrame({kHit}, {}, kOrigin);
  ASSERT_EQ(grid.processFrame({kHit}, {}, kOrigin).size(), 1U);
  for (int i = 0; i < 8; ++i) {
    EXPECT_EQ(grid.processFrame({}, {}, kOrigin).size(), 1U);
  }
}

TEST(TemporalObstacleGrid, ExpiresConfirmedCellAfterTenFramesWithoutHits)
{
  auto grid = makeGrid();
  grid.processFrame({kHit}, {}, kOrigin);
  ASSERT_EQ(grid.processFrame({kHit}, {}, kOrigin).size(), 1U);

  for (int frame = 0; frame < 9; ++frame) {
    EXPECT_EQ(grid.processFrame({}, {}, kOrigin).size(), 1U);
  }
  EXPECT_TRUE(grid.processFrame({}, {}, kOrigin).empty());
  EXPECT_EQ(grid.diagnostics().stale_cleared_cells, 1U);
}

TEST(TemporalObstacleGrid, NewHitRestartsConfirmedCellExpiry)
{
  auto grid = makeGrid();
  grid.processFrame({kHit}, {}, kOrigin);
  ASSERT_EQ(grid.processFrame({kHit}, {}, kOrigin).size(), 1U);
  for (int frame = 0; frame < 9; ++frame) {
    ASSERT_EQ(grid.processFrame({}, {}, kOrigin).size(), 1U);
  }

  ASSERT_EQ(grid.processFrame({kHit}, {}, kOrigin).size(), 1U);
  for (int frame = 0; frame < 9; ++frame) {
    EXPECT_EQ(grid.processFrame({}, {}, kOrigin).size(), 1U);
  }
  EXPECT_TRUE(grid.processFrame({}, {}, kOrigin).empty());
}

TEST(TemporalObstacleGrid, AgesConfirmedCellsWithoutTrackingTransform)
{
  auto grid = makeGrid();
  grid.processFrame({kHit}, {}, kOrigin);
  ASSERT_EQ(grid.processFrame({kHit}, {}, kOrigin).size(), 1U);

  for (int frame = 0; frame < 9; ++frame) {
    EXPECT_EQ(grid.ageWithoutObservation().size(), 1U);
  }
  EXPECT_TRUE(grid.ageWithoutObservation().empty());
  EXPECT_EQ(grid.diagnostics().stale_cleared_cells, 1U);
}

TEST(TemporalObstacleGrid, RequiresConsecutiveRayMissFramesToClear)
{
  auto grid = makeGrid();
  const std::vector<ObstacleGridPoint> rays{{2.05, 0.05, 0.0}};
  grid.processFrame({kHit}, {}, kOrigin);
  ASSERT_EQ(grid.processFrame({kHit}, {}, kOrigin).size(), 1U);
  EXPECT_EQ(grid.processFrame({}, rays, kOrigin).size(), 1U);
  EXPECT_TRUE(grid.takeRayClearedPoints().empty());
  EXPECT_TRUE(grid.processFrame({}, rays, kOrigin).empty());
  const auto cleared = grid.takeRayClearedPoints();
  ASSERT_EQ(cleared.size(), 1U);
  EXPECT_DOUBLE_EQ(cleared.front().x, kHit.x);
  EXPECT_DOUBLE_EQ(cleared.front().y, kHit.y);
}

TEST(TemporalObstacleGrid, StaleExpiryDoesNotCreateRayClearEvidence)
{
  auto grid = makeGrid();
  grid.processFrame({kHit}, {}, kOrigin);
  ASSERT_EQ(grid.processFrame({kHit}, {}, kOrigin).size(), 1U);
  for (int frame = 0; frame < 10; ++frame) {
    grid.processFrame({}, {}, kOrigin);
  }
  EXPECT_EQ(grid.diagnostics().stale_cleared_cells, 1U);
  EXPECT_TRUE(grid.takeRayClearedPoints().empty());
}

TEST(TemporalObstacleGrid, EndpointInCellDoesNotCountAsRayMiss)
{
  auto grid = makeGrid(1);
  grid.processFrame({kHit}, {}, kOrigin);
  ASSERT_EQ(grid.processFrame({kHit}, {}, kOrigin).size(), 1U);
  EXPECT_EQ(grid.processFrame({}, {kHit}, kOrigin).size(), 1U);
}

TEST(TemporalObstacleGrid, RayAboveObstacleHeightDoesNotCountAsMiss)
{
  auto grid = makeGrid(1);
  grid.processFrame({kHit}, {}, kOrigin);
  ASSERT_EQ(grid.processFrame({kHit}, {}, kOrigin).size(), 1U);
  const std::vector<ObstacleGridPoint> high_ray{{2.05, 0.05, 2.5}};
  EXPECT_EQ(grid.processFrame({}, high_ray, kOrigin).size(), 1U);
}

TEST(TemporalObstacleGrid, CurrentHitWinsOverCrossingRay)
{
  auto grid = makeGrid(1);
  const std::vector<ObstacleGridPoint> rays{{2.05, 0.05, 0.0}};
  grid.processFrame({kHit}, {}, kOrigin);
  ASSERT_EQ(grid.processFrame({kHit}, {}, kOrigin).size(), 1U);
  EXPECT_EQ(grid.processFrame({kHit}, rays, kOrigin).size(), 1U);
}

TEST(TemporalObstacleGrid, PrunesCellsOutsideLocalWindow)
{
  auto grid = makeGrid();
  grid.processFrame({kHit}, {}, kOrigin);
  ASSERT_EQ(grid.processFrame({kHit}, {}, kOrigin).size(), 1U);
  EXPECT_TRUE(grid.processFrame({}, {}, {5.0, 0.0, 0.0}).empty());
}

TEST(TemporalObstacleGrid, GlobalWindowConfirmsFallbackObstacleOutsideLocalWindow)
{
  TemporalObstacleGridParameters local_parameters;
  local_parameters.confirmation_window_frames = 5;
  local_parameters.confirmation_hits = 2;
  local_parameters.local_window_size = 6.0;
  TemporalObstacleGrid local_grid(local_parameters);

  auto global_parameters = local_parameters;
  global_parameters.local_window_size = 20.0;
  TemporalObstacleGrid global_grid(global_parameters);

  const ObstacleGridPoint fallback_obstacle{8.05, 0.05, 0.35};
  EXPECT_TRUE(local_grid.processFrame({fallback_obstacle}, {}, kOrigin, true).empty());
  EXPECT_TRUE(local_grid.processFrame({fallback_obstacle}, {}, kOrigin, true).empty());
  EXPECT_TRUE(global_grid.processFrame({fallback_obstacle}, {}, kOrigin, true).empty());
  const auto globally_confirmed =
    global_grid.processFrame({fallback_obstacle}, {}, kOrigin, true);
  const auto snapshot_without_a_new_hit =
    global_grid.processFrame({}, {}, kOrigin, false);

  EXPECT_TRUE(local_grid.confirmedPoints().empty());
  ASSERT_EQ(globally_confirmed.size(), 1U);
  EXPECT_NEAR(globally_confirmed.front().x, fallback_obstacle.x, 0.10);
  ASSERT_EQ(snapshot_without_a_new_hit.size(), 1U);
  EXPECT_NEAR(snapshot_without_a_new_hit.front().x, fallback_obstacle.x, 0.10);
}
}  // namespace
