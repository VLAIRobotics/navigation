#include <cmath>

#include <gtest/gtest.h>

#include "nav2_terrain_layer/terrain_cost.hpp"

using nav2_terrain_layer::TerrainCostConfig;
using nav2_terrain_layer::featureCost;
using nav2_terrain_layer::terrainCost;

TEST(TerrainCost, FlatTerrainHasNoCost)
{
  EXPECT_EQ(terrainCost(2.0, 0.005, 0.005, TerrainCostConfig{}), 0);
}

TEST(TerrainCost, SoftCostUsesWorstFeature)
{
  EXPECT_GT(terrainCost(15.0, 0.0, 0.0, TerrainCostConfig{}), 0);
  EXPECT_EQ(terrainCost(20.0, 0.0, 0.0, TerrainCostConfig{}), 254);
  EXPECT_GT(terrainCost(2.0, 0.07, 0.0, TerrainCostConfig{}), 0);
  EXPECT_EQ(featureCost(0.03F, 0.015, 0.03), 254);
}

TEST(TerrainCost, InvalidFeaturesDoNotCreateCost)
{
  EXPECT_EQ(terrainCost(NAN, NAN, NAN, TerrainCostConfig{}), 0);
}

TEST(TerrainCost, SoftResidualStillKeepsSlopeAndRoughnessLethal)
{
  TerrainCostConfig config;
  EXPECT_EQ(terrainCost(2.0, 0.12, 0.01, config, false), 252);
  EXPECT_EQ(terrainCost(12.0, 0.12, 0.01, config, false), 254);
  EXPECT_EQ(terrainCost(2.0, 0.12, 0.12, config, false), 254);
  EXPECT_EQ(terrainCost(2.0, 0.12, 0.01, config, true), 254);
}

TEST(TerrainCost, StaticClassAndClearanceUseRuntimeCapability)
{
  TerrainCostConfig config;
  EXPECT_EQ(terrainCost(0.0, NAN, 0.0, 0.0, 0.0, config), 255);
  EXPECT_EQ(terrainCost(2.0, 2.0, 0.0, 0.0, 0.0, config), 254);
  EXPECT_EQ(terrainCost(1.0, 0.49, 0.0, 0.0, 0.0, config), 254);
  EXPECT_EQ(terrainCost(1.0, 1.0, 0.0, 0.05, 0.0, config), 254);
  EXPECT_EQ(terrainCost(1.0, 1.0, 10.0, 0.0, 0.0, config), 254);
}

TEST(TerrainCost, FloatStoredClearanceMatchesConverterThresholdAtBoundary)
{
  TerrainCostConfig config;
  config.robot_height = 1.20;
  config.clearance_margin = 0.10;
  const float boundary = static_cast<float>(config.robot_height + config.clearance_margin);
  const float just_below = std::nextafter(boundary, 0.0F);

  EXPECT_NE(terrainCost(1.0, static_cast<double>(boundary), 0.0, 0.0, 0.0, config), 254);
  EXPECT_EQ(terrainCost(1.0, static_cast<double>(just_below), 0.0, 0.0, 0.0, config), 254);
}

TEST(TerrainCost, SoftFeatureModeKeepsDerivedFeaturesBelowLethal)
{
  TerrainCostConfig config;
  EXPECT_EQ(terrainCost(1.0, 1.0, 35.0, 0.20, 0.20, config, false), 252);
  EXPECT_EQ(terrainCost(2.0, 1.0, 35.0, 0.20, 0.20, config, false), 254);
  EXPECT_EQ(terrainCost(1.0, 0.49, 35.0, 0.20, 0.20, config, false), 254);
}

TEST(TerrainCost, RecordedGoalResidualReachesConfiguredNavigationGateCost)
{
  TerrainCostConfig config;
  config.residual_step_soft = 0.015;
  config.residual_step_lethal = 0.05;

  // The recorded goal cell has this residual; the global static layer uses the
  // soft-feature mode, which caps lethal-derived feature costs at 252.
  EXPECT_EQ(terrainCost(
      1.0, 3.29359722, 2.64234328, 0.0371047258, 0.015057466,
      config, false), 159);
}

TEST(TerrainCost, HardFeatureModePreservesCapabilityLimits)
{
  TerrainCostConfig config;
  EXPECT_EQ(terrainCost(1.0, 1.0, 35.0, 0.02, 0.0, config, true), 254);
  EXPECT_EQ(terrainCost(1.0, 1.0, 9.0, 0.04, 0.0, config, true), 254);
}
