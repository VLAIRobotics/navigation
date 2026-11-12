#include "bringup/inflation_surface.hpp"

#include <algorithm>

#include <gtest/gtest.h>

namespace
{

nav_msgs::msg::OccupancyGrid makeGrid(std::uint32_t width, std::uint32_t height)
{
  nav_msgs::msg::OccupancyGrid grid;
  grid.info.width = width;
  grid.info.height = height;
  grid.info.resolution = 1.0F;
  grid.info.origin.orientation.w = 1.0;
  grid.data.assign(static_cast<std::size_t>(width) * height, 0);
  return grid;
}

TEST(InflationSurfaceTest, ReturnsEachInflatedGridCellWithItsOwnCost)
{
  auto grid = makeGrid(3, 3);
  grid.data[4] = 100;
  grid.data[3] = 60;
  grid.data[8] = 35;
  const auto cells = bringup::inflationCells(grid, 1.0);
  ASSERT_EQ(cells.size(), 2U);
  EXPECT_EQ(cells[0].x, 0U);
  EXPECT_EQ(cells[0].y, 1U);
  EXPECT_EQ(cells[0].cost, 60);
  EXPECT_EQ(cells[1].x, 1U);
  EXPECT_EQ(cells[1].y, 1U);
  EXPECT_EQ(cells[1].cost, 100);
}

TEST(InflationSurfaceTest, DrawsOnlySoftCostsNearAnObstacleCore)
{
  auto grid = makeGrid(6, 6);
  grid.data[1 + 1 * 6] = 100;
  grid.data[2 + 1 * 6] = 40;
  grid.data[5 + 5 * 6] = 40;
  const auto cells = bringup::inflationCells(grid, 1.1);
  EXPECT_TRUE(std::any_of(cells.begin(), cells.end(), [](const auto & cell) {
      return cell.cost == 40;
    }));
  EXPECT_TRUE(std::none_of(cells.begin(), cells.end(), [](const auto & cell) {
      return cell.x >= 4 && cell.y >= 4;
    }));
  grid.data[1 + 1 * 6] = 0;
  EXPECT_TRUE(bringup::inflationCells(grid, 1.1).empty());
}

TEST(InflationSurfaceTest, MatchesRvizCostmapPaletteWithOpaqueCostCells)
{
  const auto free = bringup::inflationColor(0);
  const auto outer = bringup::inflationColor(1);
  const auto middle = bringup::inflationColor(50);
  const auto inner = bringup::inflationColor(98);
  const auto inscribed = bringup::inflationColor(99);
  const auto obstacle = bringup::inflationColor(100);
  EXPECT_EQ(free.a, 0.0F);
  EXPECT_EQ(outer.a, 1.0F);
  EXPECT_EQ(middle.a, 1.0F);
  EXPECT_FLOAT_EQ(middle.r, 127.0F / 255.0F);
  EXPECT_FLOAT_EQ(middle.g, 0.0F);
  EXPECT_FLOAT_EQ(middle.b, 128.0F / 255.0F);
  EXPECT_FLOAT_EQ(inscribed.r, 0.0F);
  EXPECT_FLOAT_EQ(inscribed.g, 1.0F);
  EXPECT_FLOAT_EQ(inscribed.b, 1.0F);
  EXPECT_FLOAT_EQ(obstacle.r, 1.0F);
  EXPECT_FLOAT_EQ(obstacle.g, 0.0F);
  EXPECT_FLOAT_EQ(obstacle.b, 1.0F);
  EXPECT_GT(outer.b, outer.r);
  EXPECT_GT(inner.r, inner.b);
  EXPECT_EQ(bringup::inflationColor(-1).a, 0.0F);
}

TEST(InflationSurfaceTest, IgnoresUnknownAndEmptyAreas)
{
  auto grid = makeGrid(2, 2);
  grid.data = {-1, -1, 0, 0};
  EXPECT_TRUE(bringup::inflationCells(grid, 1.0).empty());
  grid.data = {-1, 100, 0, 0};
  EXPECT_EQ(bringup::inflationCells(grid, 1.0).size(), 1U);
  EXPECT_EQ(bringup::inflationColor(-1).a, 0.0F);
}

TEST(InflationSurfaceTest, IncludesCostCellsAtGridEdges)
{
  auto grid = makeGrid(2, 2);
  grid.data[0] = 100;
  const auto cells = bringup::inflationCells(grid, 0.0);
  ASSERT_EQ(cells.size(), 1U);
  EXPECT_EQ(cells.front().x, 0U);
  EXPECT_EQ(cells.front().y, 0U);
  EXPECT_EQ(cells.front().cost, 100);
}

TEST(InflationSurfaceTest, AppliesIncrementalUpdatesAndClearsSurface)
{
  auto grid = makeGrid(3, 3);
  map_msgs::msg::OccupancyGridUpdate update;
  update.x = 1;
  update.y = 1;
  update.width = 1;
  update.height = 1;
  update.data = {100};
  bool changed = false;
  ASSERT_TRUE(bringup::applyCostmapUpdate(grid, update, &changed));
  EXPECT_TRUE(changed);
  EXPECT_EQ(bringup::inflationCells(grid, 1.0).size(), 1U);
  ASSERT_TRUE(bringup::applyCostmapUpdate(grid, update, &changed));
  EXPECT_FALSE(changed);
  update.data = {0};
  ASSERT_TRUE(bringup::applyCostmapUpdate(grid, update, &changed));
  EXPECT_TRUE(changed);
  EXPECT_TRUE(bringup::inflationCells(grid, 1.0).empty());
}

TEST(InflationSurfaceTest, RejectsOutOfBoundsUpdateWithoutChangingMap)
{
  auto grid = makeGrid(3, 3);
  map_msgs::msg::OccupancyGridUpdate update;
  update.x = 2;
  update.y = 2;
  update.width = 2;
  update.height = 1;
  update.data = {100, 100};
  EXPECT_FALSE(bringup::applyCostmapUpdate(grid, update));
  EXPECT_EQ(grid.data[8], 0);
}

}  // namespace
