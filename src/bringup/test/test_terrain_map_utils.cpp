#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <vector>

#include "bringup/terrain_map_utils.hpp"

TEST(TerrainMapUtils, OccupiedRadiusOverridesKnownFreeCells)
{
  std::vector<std::uint8_t> occupancy(3, 254);
  std::vector<bool> occupied(3, false);
  occupied[1] = true;
  bringup::expand_occupied_cells(occupancy, occupied, 1, 3, 1);
  EXPECT_EQ(occupancy, (std::vector<std::uint8_t>{0, 0, 0}));
}

TEST(TerrainMapUtils, SingleHeightReturnIsNotObstacleEvidence)
{
  const std::vector<bringup::HeightEvidenceBin> bins{{0, 1, 0.051F}};
  EXPECT_TRUE(std::isnan(bringup::supported_height_above(bins, 1, 0.0F, 0.05F, 8, 0.05)));
}

TEST(TerrainMapUtils, RetainsLowObstacleInThresholdBoundaryBin)
{
  const std::vector<bringup::HeightEvidenceBin> bins{{0, 80, 0.0F}, {1, 18, 0.18F}};
  EXPECT_NEAR(bringup::supported_height_above(bins, 98, 0.0F, 0.15F, 8, 0.05), 0.18F, 1e-6F);
}

TEST(TerrainMapUtils, RejectsReturnExactlyAtHeightGap)
{
  const std::vector<bringup::HeightEvidenceBin> bins{{1, 18, 0.15F}};
  EXPECT_TRUE(std::isnan(bringup::supported_height_above(bins, 18, 0.0F, 0.15F, 8, 0.05)));
}

TEST(TerrainMapUtils, HeightEvidenceIsInvariantToWorldHeightTranslation)
{
  const std::vector<bringup::HeightEvidenceBin> bins{{100, 80, 10.0F}, {101, 18, 10.18F}};
  EXPECT_NEAR(bringup::supported_height_above(bins, 98, 10.0F, 0.15F, 8, 0.05), 10.18F, 1e-6F);
}

TEST(TerrainMapUtils, SupportedLowObstacleIsRetained)
{
  const std::vector<bringup::HeightEvidenceBin> bins{{0, 80, 0.0F}, {1, 18, 0.12F}};
  EXPECT_NEAR(bringup::supported_height_above(bins, 98, 0.0F, 0.05F, 8, 0.05), 0.12F, 1e-6F);
}

TEST(TerrainMapUtils, RejectedSurfaceWithoutObstacleEvidenceRemainsUnknown)
{
  const auto terrain_class = bringup::classify_terrain_cell(false, 0.0F, 0.8F, 0.50F);
  EXPECT_EQ(terrain_class, bringup::TerrainCellClass::Unknown);
  EXPECT_EQ(bringup::terrain_cell_occupancy(terrain_class), 205);
}

TEST(TerrainMapUtils, RejectedHighPlatformRemainsUnknownInPgmOutput)
{
  const auto terrain_class = bringup::classify_terrain_cell(false, 0.0F, 3.7F, 0.50F);
  EXPECT_EQ(terrain_class, bringup::TerrainCellClass::Unknown);
  EXPECT_EQ(bringup::terrain_cell_occupancy(terrain_class), 205);
}

TEST(TerrainMapUtils, IsolatedSurfaceRemainsUnknownInPgmOutput)
{
  const auto terrain_class = bringup::classify_terrain_cell(false, 0.0F, NAN, 0.50F);
  EXPECT_EQ(terrain_class, bringup::TerrainCellClass::Unknown);
  EXPECT_EQ(bringup::terrain_cell_occupancy(terrain_class), 205);
}

TEST(TerrainMapUtils, RejectedSurfaceWithNeighbourEvidenceRemainsOccupied)
{
  const auto terrain_class = bringup::classify_terrain_cell(false, 0.0F, 0.18F, 0.50F);
  EXPECT_EQ(terrain_class, bringup::TerrainCellClass::Obstacle);
  EXPECT_EQ(bringup::terrain_cell_occupancy(terrain_class), 0);
}

TEST(TerrainMapUtils, ValidGroundWithoutObstacleRemainsFree)
{
  const auto terrain_class = bringup::classify_terrain_cell(true, 0.0F, NAN, 0.50F);
  EXPECT_EQ(terrain_class, bringup::TerrainCellClass::Ground);
  EXPECT_EQ(bringup::terrain_cell_occupancy(terrain_class), 254);
}

TEST(TerrainMapUtils, ClearanceThresholdControlsObstacleAndPreservesUnknown)
{
  using bringup::TerrainCellClass;
  EXPECT_EQ(bringup::classify_terrain_cell(true, 0.0F, 0.8F, 1.30F), TerrainCellClass::Obstacle);
  EXPECT_EQ(bringup::classify_terrain_cell(true, 0.0F, 0.8F, 0.50F), TerrainCellClass::Ground);
  EXPECT_EQ(bringup::classify_terrain_cell(true, 0.0F, 1.30F, 1.30F), TerrainCellClass::Ground);
  EXPECT_EQ(bringup::classify_terrain_cell(false, 0.0F, 0.8F, 1.30F), TerrainCellClass::Obstacle);
  EXPECT_EQ(bringup::classify_terrain_cell(false, 0.0F, NAN, 1.30F), TerrainCellClass::Unknown);
  EXPECT_EQ(bringup::terrain_cell_occupancy(TerrainCellClass::Obstacle), 0);
}

TEST(TerrainMapUtils, FloatClearanceBoundaryMatchesStaticTerrainLayer)
{
  constexpr float threshold = 1.30F;
  const float just_below = std::nextafter(threshold, 0.0F);
  EXPECT_EQ(
    bringup::classify_terrain_cell(true, 0.0F, threshold, threshold),
    bringup::TerrainCellClass::Ground);
  EXPECT_EQ(
    bringup::classify_terrain_cell(true, 0.0F, just_below, threshold),
    bringup::TerrainCellClass::Obstacle);
}

TEST(TerrainMapUtils, OccupancyPgmConversionPreservesNonSquareWorldAxes)
{
  // GridMap rows are X (width in the PGM), columns are Y (height in the PGM).
  // Encode every cell with a unique value so a transpose or vertical flip fails.
  constexpr std::size_t rows = 3;
  constexpr std::size_t cols = 2;
  std::vector<std::uint8_t> occupancy{10, 11, 20, 21, 30, 31};
  const auto pixels = bringup::occupancy_to_pgm(occupancy, rows, cols);
  ASSERT_EQ(pixels.size(), rows * cols);
  // Image rows are y=high to y=low; image columns are x=low to x=high.
  EXPECT_EQ(pixels, (std::vector<std::uint8_t>{30, 20, 10, 31, 21, 11}));
}

TEST(TerrainMapUtils, OccupancyPgmConversionRejectsInvalidGeometry)
{
  EXPECT_THROW(
    bringup::occupancy_to_pgm(std::vector<std::uint8_t>{0}, 2, 2), std::invalid_argument);
}

TEST(TerrainMapUtils, RejectsOversizedMapBeforeAllocation)
{
  EXPECT_THROW(bringup::checked_terrain_cell_count(100.0, 100.0, 1.0, 1000), std::runtime_error);
  EXPECT_EQ(bringup::checked_terrain_cell_count(10.0, 10.0, 1.0, 100), 100U);
}

TEST(TerrainMapUtils, RejectsDisconnectedHighSurface)
{
  const float nan = std::numeric_limits<float>::quiet_NaN();
  const std::vector<float> elevation{
    0.0F, 0.0F, nan, 3.7F, 3.7F,
    0.0F, 0.1F, nan, 3.7F, 3.7F,
    0.1F, 0.1F, nan, nan, nan,
  };
  std::vector<bool> candidate(elevation.size(), true);
  for (std::size_t i = 0; i < elevation.size(); ++i) {
    candidate[i] = std::isfinite(elevation[i]);
  }
  const auto connected = bringup::retain_ground_largest_connected_component(
    elevation, candidate, 3, 5, 0.5);
  EXPECT_TRUE(connected[0]);
  EXPECT_TRUE(connected[11]);
  EXPECT_FALSE(connected[3]);
  EXPECT_FALSE(connected[9]);
}

TEST(TerrainMapUtils, KeepsContinuouslyRisingGround)
{
  const std::vector<float> elevation{0.0F, 0.4F, 0.8F, 1.2F, 1.6F};
  const std::vector<bool> candidate(elevation.size(), true);
  const auto connected = bringup::retain_ground_largest_connected_component(
    elevation, candidate, 1, 5, 0.5);
  EXPECT_EQ(connected, candidate);
}

TEST(TerrainMapUtils, SelectsLargestSurfaceInsteadOfLowestNoiseIsland)
{
  const float nan = std::numeric_limits<float>::quiet_NaN();
  const std::vector<float> elevation{
    0.0F, nan, 1.0F, 1.0F, 1.0F,
    nan, nan, 1.0F, 1.0F, 1.0F,
    nan, nan, nan, nan, nan,
  };
  std::vector<bool> candidate(elevation.size(), false);
  for (std::size_t i = 0; i < elevation.size(); ++i) candidate[i] = std::isfinite(elevation[i]);

  const auto connected = bringup::retain_ground_largest_connected_component(
    elevation, candidate, 3, 5, 0.5);
  EXPECT_FALSE(connected[0]);
  for (std::size_t i : {2U, 3U, 4U, 7U, 8U, 9U}) EXPECT_TRUE(connected[i]);
}

TEST(TerrainMapUtils, RetainsLowGroundFragmentsButRejectsHighSurfaces)
{
  const std::vector<float> elevation{
    0.0F, 0.0F, NAN, 3.7F, 3.7F,
    0.0F, 0.1F, NAN, 3.7F, 3.7F,
    NAN, NAN, NAN, NAN, NAN,
  };
  std::vector<bool> candidate(elevation.size(), false);
  for (std::size_t i = 0; i < elevation.size(); ++i) candidate[i] = std::isfinite(elevation[i]);
  const auto retained = bringup::retain_ground_components_by_area(
    elevation, candidate, 3, 5, 0.5, 2, 0.75);
  EXPECT_TRUE(retained[0]);
  EXPECT_TRUE(retained[6]);
  EXPECT_FALSE(retained[3]);
  EXPECT_FALSE(retained[9]);
}
