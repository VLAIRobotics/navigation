#include <cstddef>
#include <cstdint>
#include <vector>

#include <gtest/gtest.h>
#include <nav2_costmap_2d/cost_values.hpp>

#include "bringup/path_collision_escape.hpp"

namespace
{
constexpr std::size_t kWidth = 9;
constexpr std::size_t kHeight = 9;

std::size_t cell(int x, int y)
{
  return static_cast<std::size_t>(y) * kWidth + static_cast<std::size_t>(x);
}
}  // namespace

TEST(PathCollisionEscape, FreeStartAllowsDisconnectedUnknownInsideStartCentredDisk)
{
  std::vector<std::uint8_t> costs(kWidth * kHeight, nav2_costmap_2d::FREE_SPACE);
  costs[cell(5, 4)] = nav2_costmap_2d::NO_INFORMATION;
  costs[cell(7, 4)] = nav2_costmap_2d::NO_INFORMATION;
  costs[cell(8, 8)] = nav2_costmap_2d::NO_INFORMATION;
  costs[cell(4, 5)] = nav2_costmap_2d::LETHAL_OBSTACLE;

  const std::vector<bringup::path_collision::Point2D> footprint{
    {4.15, 4.20}, {5.15, 4.20}, {5.15, 4.80}, {4.15, 4.80}};
  std::vector<std::size_t> initial_footprint_unknown;
  ASSERT_TRUE(bringup::path_collision::collectUnknownCells(
      costs, kWidth, kHeight, footprint, 0.0, 100,
      initial_footprint_unknown));
  ASSERT_EQ(initial_footprint_unknown, (std::vector<std::size_t>{cell(5, 4)}));
  bringup::path_collision::StartEscapeMask mask(
    costs, kWidth, kHeight, 0.1, 4.5, 4.5, 0.45, initial_footprint_unknown);

  EXPECT_TRUE(mask.allows_unknown(cell(5, 4)));
  // Free cells separate this unknown cell from the initial footprint unknown.
  EXPECT_TRUE(mask.allows_unknown(cell(7, 4)));
  EXPECT_FALSE(mask.allows_unknown(cell(8, 8)));
  EXPECT_TRUE(bringup::path_collision::isBlockedCell(
      costs[cell(4, 5)], mask.allows_unknown(cell(4, 5))));

  mask.finish_pose(true);
  EXPECT_TRUE(mask.allows_unknown(cell(7, 4)));
  mask.finish_pose(false);
  EXPECT_FALSE(mask.allows_unknown(cell(5, 4)));
  EXPECT_FALSE(mask.allows_unknown(cell(7, 4)));
}

TEST(PathCollisionEscape, SoftInflatedStartAllowsFootprintUnknownInsideDisk)
{
  std::vector<std::uint8_t> costs(kWidth * kHeight, nav2_costmap_2d::FREE_SPACE);
  costs[cell(4, 4)] = 100;
  costs[cell(5, 4)] = nav2_costmap_2d::NO_INFORMATION;

  bringup::path_collision::StartEscapeMask mask(
    costs, kWidth, kHeight, 0.1, 4.5, 4.5, 0.45,
    std::vector<std::size_t>{cell(5, 4)});

  EXPECT_TRUE(mask.allows_unknown(cell(5, 4)));
  EXPECT_FALSE(bringup::path_collision::isBlockedCell(
      costs[cell(5, 4)], mask.allows_unknown(cell(5, 4))));
}

TEST(PathCollisionEscape, SoftInflatedStartDoesNotEscapeForUnknownOutsideFootprint)
{
  std::vector<std::uint8_t> costs(kWidth * kHeight, nav2_costmap_2d::FREE_SPACE);
  costs[cell(4, 4)] = 100;
  costs[cell(7, 4)] = nav2_costmap_2d::NO_INFORMATION;

  bringup::path_collision::StartEscapeMask mask(
    costs, kWidth, kHeight, 0.1, 4.5, 4.5, 0.35);

  EXPECT_FALSE(mask.allows_unknown(cell(7, 4)));
  EXPECT_TRUE(bringup::path_collision::isBlockedCell(
      costs[cell(7, 4)], mask.allows_unknown(cell(7, 4))));
}

TEST(PathCollisionEscape, InitialFootprintRasterizationUsesCellGeometry)
{
  std::vector<std::uint8_t> costs(kWidth * kHeight, nav2_costmap_2d::FREE_SPACE);
  costs[cell(4, 4)] = nav2_costmap_2d::NO_INFORMATION;
  costs[cell(6, 4)] = nav2_costmap_2d::NO_INFORMATION;
  const std::vector<bringup::path_collision::Point2D> footprint = {
    {4.2, 4.2}, {4.8, 4.2}, {4.8, 4.8}, {4.2, 4.8}};
  std::vector<std::size_t> initial_unknown_cells;

  ASSERT_TRUE(bringup::path_collision::collectUnknownCells(
      costs, kWidth, kHeight, footprint, 0.0, 100, initial_unknown_cells));

  ASSERT_EQ(initial_unknown_cells.size(), 1U);
  EXPECT_EQ(initial_unknown_cells.front(), cell(4, 4));
}

TEST(PathCollisionEscape, UnknownInsideSamplingMarginActivatesBoundedStartEscape)
{
  std::vector<std::uint8_t> costs(kWidth * kHeight, nav2_costmap_2d::FREE_SPACE);
  costs[cell(5, 4)] = nav2_costmap_2d::NO_INFORMATION;
  costs[cell(8, 8)] = nav2_costmap_2d::NO_INFORMATION;
  costs[cell(4, 5)] = nav2_costmap_2d::LETHAL_OBSTACLE;
  const std::vector<bringup::path_collision::Point2D> footprint{
    {4.15, 4.20}, {4.85, 4.20}, {4.85, 4.80}, {4.15, 4.80}};
  std::vector<std::size_t> exact_footprint_unknown_cells;
  std::vector<std::size_t> initial_unknown_cells;

  ASSERT_TRUE(bringup::path_collision::collectUnknownCells(
      costs, kWidth, kHeight, footprint, 0.0, 100,
      exact_footprint_unknown_cells));
  EXPECT_TRUE(exact_footprint_unknown_cells.empty());

  ASSERT_TRUE(bringup::path_collision::collectUnknownCells(
      costs, kWidth, kHeight, footprint,
      bringup::path_collision::kMaxSampleDisplacementCells, 100, initial_unknown_cells));
  ASSERT_EQ(initial_unknown_cells, (std::vector<std::size_t>{cell(5, 4)}));

  bringup::path_collision::StartEscapeMask mask(
    costs, kWidth, kHeight, 0.1, 4.5, 4.5, 0.45, initial_unknown_cells);
  EXPECT_TRUE(mask.allows_unknown(cell(5, 4)));
  EXPECT_FALSE(bringup::path_collision::isBlockedCell(
      costs[cell(5, 4)], mask.allows_unknown(cell(5, 4))));
  EXPECT_FALSE(mask.allows_unknown(cell(8, 8)));
  EXPECT_TRUE(bringup::path_collision::isBlockedCell(
      costs[cell(8, 8)], mask.allows_unknown(cell(8, 8))));
  EXPECT_TRUE(bringup::path_collision::isBlockedCell(
      costs[cell(4, 5)], mask.allows_unknown(cell(4, 5))));
}

TEST(PathCollisionEscape, InitialFootprintUsesCurrentStartTranslationAndYaw)
{
  std::vector<std::uint8_t> costs(kWidth * kHeight, nav2_costmap_2d::FREE_SPACE);
  costs[cell(6, 4)] = nav2_costmap_2d::NO_INFORMATION;
  costs[cell(7, 4)] = nav2_costmap_2d::NO_INFORMATION;
  costs[cell(4, 6)] = nav2_costmap_2d::NO_INFORMATION;
  const std::vector<bringup::path_collision::Point2D> footprint_in_base = {
    {0.24, 0.02}, {0.24, -0.02}, {0.16, -0.02}, {0.16, 0.02}};
  std::vector<bringup::path_collision::Point2D> footprint_in_grid;
  std::vector<std::size_t> initial_unknown_cells;

  ASSERT_TRUE(bringup::path_collision::transformFootprintToGrid(
      footprint_in_base, 4.5, 4.5, 0.0, 0.1, footprint_in_grid));
  ASSERT_TRUE(bringup::path_collision::collectUnknownCells(
      costs, kWidth, kHeight, footprint_in_grid, 0.0, 100, initial_unknown_cells));
  ASSERT_EQ(initial_unknown_cells.size(), 1U);
  EXPECT_EQ(initial_unknown_cells.front(), cell(6, 4));

  ASSERT_TRUE(bringup::path_collision::transformFootprintToGrid(
      footprint_in_base, 5.5, 4.5, 0.0, 0.1, footprint_in_grid));
  initial_unknown_cells.clear();
  ASSERT_TRUE(bringup::path_collision::collectUnknownCells(
      costs, kWidth, kHeight, footprint_in_grid, 0.0, 100, initial_unknown_cells));
  ASSERT_EQ(initial_unknown_cells.size(), 1U);
  EXPECT_EQ(initial_unknown_cells.front(), cell(7, 4));

  ASSERT_TRUE(bringup::path_collision::transformFootprintToGrid(
      footprint_in_base, 4.5, 4.5, 1.5707963267948966, 0.1, footprint_in_grid));
  initial_unknown_cells.clear();
  ASSERT_TRUE(bringup::path_collision::collectUnknownCells(
      costs, kWidth, kHeight, footprint_in_grid, 0.0, 100, initial_unknown_cells));
  ASSERT_EQ(initial_unknown_cells.size(), 1U);
  EXPECT_EQ(initial_unknown_cells.front(), cell(4, 6));
}

TEST(PathCollisionEscape, UnknownStartUsesSameBoundedGeometricDisk)
{
  std::vector<std::uint8_t> costs(kWidth * kHeight, nav2_costmap_2d::FREE_SPACE);
  costs[cell(4, 4)] = nav2_costmap_2d::NO_INFORMATION;
  costs[cell(6, 5)] = nav2_costmap_2d::NO_INFORMATION;
  costs[cell(8, 8)] = nav2_costmap_2d::NO_INFORMATION;
  const std::vector<std::size_t> no_initial_unknown{};

  bringup::path_collision::StartEscapeMask mask(
    costs, kWidth, kHeight, 0.1, 4.5, 4.5, 0.45, no_initial_unknown);
  EXPECT_TRUE(mask.allows_unknown(cell(4, 4)));
  EXPECT_TRUE(mask.allows_unknown(cell(6, 5)));
  EXPECT_FALSE(mask.allows_unknown(cell(8, 8)));
}

TEST(PathCollisionEscape, InitialFootprintUnknownOutsideConfiguredRadiusIsRejected)
{
  std::vector<std::uint8_t> costs(kWidth * kHeight, nav2_costmap_2d::FREE_SPACE);
  costs[cell(7, 4)] = nav2_costmap_2d::NO_INFORMATION;
  const std::vector<std::size_t> initial_footprint_unknown{cell(7, 4)};

  bringup::path_collision::StartEscapeMask mask(
    costs, kWidth, kHeight, 0.1, 4.5, 4.5, 0.20, initial_footprint_unknown);
  EXPECT_FALSE(mask.allows_unknown(cell(7, 4)));
  EXPECT_TRUE(bringup::path_collision::isBlockedCell(
      costs[cell(7, 4)], mask.allows_unknown(cell(7, 4))));
}

TEST(PathCollisionEscape, ZeroRadiusDoesNotPermitUnknown)
{
  std::vector<std::uint8_t> costs(kWidth * kHeight, nav2_costmap_2d::FREE_SPACE);
  costs[cell(4, 4)] = nav2_costmap_2d::NO_INFORMATION;
  const std::vector<std::size_t> initial_footprint_unknown{cell(4, 4)};

  bringup::path_collision::StartEscapeMask mask(
    costs, kWidth, kHeight, 0.1, 4.5, 4.5, 0.0, initial_footprint_unknown);
  EXPECT_FALSE(mask.allows_unknown(cell(4, 4)));
}

TEST(PathCollisionEscape, LethalStartDoesNotActivateUnknownEscape)
{
  std::vector<std::uint8_t> costs(kWidth * kHeight, nav2_costmap_2d::FREE_SPACE);
  costs[cell(4, 4)] = nav2_costmap_2d::LETHAL_OBSTACLE;
  costs[cell(5, 4)] = nav2_costmap_2d::NO_INFORMATION;

  bringup::path_collision::StartEscapeMask mask(
    costs, kWidth, kHeight, 0.1, 4.5, 4.5, 0.45);
  EXPECT_FALSE(mask.allows_unknown(cell(5, 4)));
  EXPECT_TRUE(bringup::path_collision::isBlockedCell(
      costs[cell(4, 4)], mask.allows_unknown(cell(4, 4))));
}
