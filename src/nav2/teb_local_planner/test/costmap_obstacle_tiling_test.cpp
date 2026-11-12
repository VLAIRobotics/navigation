#include <cstdint>
#include <vector>

#include <gtest/gtest.h>

#include "teb_local_planner/costmap_obstacle_tiling.hpp"
#include "teb_local_planner/distance_calculations.h"

namespace
{

using teb_local_planner::detail::CostmapObstacleTile;
using teb_local_planner::detail::tileLethalCells;

std::vector<std::uint8_t> rasterizeTiles(
  const std::vector<CostmapObstacleTile> & tiles,
  unsigned int width,
  unsigned int height)
{
  std::vector<std::uint8_t> covered(static_cast<std::size_t>(width) * height, 0);
  for (const auto & tile : tiles) {
    for (unsigned int y = tile.min_y; y <= tile.max_y; ++y) {
      for (unsigned int x = tile.min_x; x <= tile.max_x; ++x) {
        auto & cell = covered[static_cast<std::size_t>(y) * width + x];
        ++cell;
      }
    }
  }
  return covered;
}

TEST(CostmapObstacleTiling, IsolatedCellProducesAOneCellTile)
{
  const std::vector<std::uint8_t> lethal{0, 0, 0, 0, 1, 0, 0, 0, 0};
  const auto tiles = tileLethalCells(lethal, 3, 3, 5);

  ASSERT_EQ(tiles.size(), 1U);
  EXPECT_EQ(tiles.front().min_x, 1U);
  EXPECT_EQ(tiles.front().max_x, 1U);
  EXPECT_EQ(tiles.front().min_y, 1U);
  EXPECT_EQ(tiles.front().max_y, 1U);
  EXPECT_EQ(rasterizeTiles(tiles, 3, 3), lethal);
}

TEST(CostmapObstacleTiling, PreservesLethalCellsAndFreeGaps)
{
  const auto expect_exact_coverage = [](
    unsigned int width, unsigned int height, const std::vector<std::uint8_t> & lethal,
    unsigned int max_cells_per_side) {
      const auto tiles = tileLethalCells(lethal, width, height, max_cells_per_side);
      EXPECT_EQ(rasterizeTiles(tiles, width, height), lethal);
      for (const auto & tile : tiles) {
        EXPECT_LE(tile.max_x - tile.min_x + 1, max_cells_per_side);
        EXPECT_LE(tile.max_y - tile.min_y + 1, max_cells_per_side);
      }
    };

  // Open-bottom U shape.
  expect_exact_coverage(4, 3, {
      1, 1, 1, 1,
      1, 0, 0, 1,
      1, 0, 0, 1}, 3);
  // Ring with a free center.
  expect_exact_coverage(3, 3, {
      1, 1, 1,
      1, 0, 1,
      1, 1, 1}, 3);
  // Diagonally touching cells must remain separate occupied cells.
  expect_exact_coverage(3, 3, {
      1, 0, 0,
      0, 1, 0,
      0, 0, 1}, 3);
  // Solid regions are split by the side cap without losing or adding cells.
  expect_exact_coverage(4, 4, std::vector<std::uint8_t>(16, 1), 2);
}

TEST(CostmapObstacleTiling, RejectsMalformedGridInput)
{
  EXPECT_TRUE(tileLethalCells({}, 0, 1, 1).empty());
  EXPECT_TRUE(tileLethalCells({1}, 2, 1, 1).empty());
  EXPECT_TRUE(tileLethalCells({1}, 1, 1, 0).empty());
}

TEST(ObstaclePolygonDistance, ReturnsZeroForContainedPointAndPolygon)
{
  const teb_local_planner::Point2dContainer footprint{
    {-0.35, -0.225}, {0.275, -0.225}, {0.275, 0.225}, {-0.35, 0.225}};
  const teb_local_planner::Point2dContainer cell{
    {-0.05, -0.05}, {0.05, -0.05}, {0.05, 0.05}, {-0.05, 0.05}};

  EXPECT_DOUBLE_EQ(
    teb_local_planner::distance_point_to_polygon_2d(Eigen::Vector2d::Zero(), cell), 0.0);
  EXPECT_DOUBLE_EQ(
    teb_local_planner::distance_polygon_to_polygon_2d(footprint, cell), 0.0);
  EXPECT_DOUBLE_EQ(
    teb_local_planner::distance_polygon_to_polygon_2d(cell, footprint), 0.0);
}

TEST(ObstaclePolygonDistance, KeepsTouchingAndSeparatedPolygonsDistinct)
{
  const teb_local_planner::Point2dContainer first{
    {0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}, {0.0, 1.0}};
  const teb_local_planner::Point2dContainer touching{
    {1.0, 0.25}, {2.0, 0.25}, {2.0, 0.75}, {1.0, 0.75}};
  const teb_local_planner::Point2dContainer separated{
    {2.0, 0.25}, {3.0, 0.25}, {3.0, 0.75}, {2.0, 0.75}};

  EXPECT_DOUBLE_EQ(
    teb_local_planner::distance_polygon_to_polygon_2d(first, touching), 0.0);
  EXPECT_NEAR(
    teb_local_planner::distance_polygon_to_polygon_2d(first, separated), 1.0, 1.0e-12);
}

}  // namespace
