#ifndef NAV2_TERRAIN_LAYER__TERRAIN_COST_HPP_
#define NAV2_TERRAIN_LAYER__TERRAIN_COST_HPP_

#include <algorithm>
#include <cmath>

namespace nav2_terrain_layer
{
struct TerrainCostConfig
{
  double robot_height{0.40};
  double clearance_margin{0.10};
  double max_climbable_step{0.03};
  double max_traversable_slope{10.0};
  double slope_soft{5.0};
  double slope_lethal{10.0};
  double residual_step_soft{0.015};
  double residual_step_lethal{0.05};
  double roughness_soft{0.03};
  double roughness_lethal{0.10};
  double soft_cost_scale{1.0};
};

// Scale only terrain soft costs; inflation, lethal obstacles, and unknown cells
// keep their Nav2 cost semantics.
inline unsigned char scaleSoftTerrainCost(
  const unsigned char cost, const double scale)
{
  if (cost > 252) {
    return cost;
  }
  return static_cast<unsigned char>(std::clamp(std::lround(cost * scale), 0L, 252L));
}

inline unsigned char featureCost(
  const double value, const double soft_threshold, const double lethal_threshold)
{
  if (!std::isfinite(value)) {
    return 0;
  }
  constexpr double kThresholdTolerance = 1.0e-9;
  if (lethal_threshold <= soft_threshold || value >= lethal_threshold - kThresholdTolerance) {
    return value >= lethal_threshold - kThresholdTolerance ? 254 : 0;
  }
  if (value <= soft_threshold) {
    return 0;
  }
  const double ratio = (value - soft_threshold) / (lethal_threshold - soft_threshold);
  return static_cast<unsigned char>(std::clamp(std::lround(ratio * 252.0), 0L, 252L));
}

inline unsigned char terrainCost(
  const double slope, const double residual_step, const double roughness,
  const TerrainCostConfig & config, const bool hard_residual_step)
{
  return std::max({
    featureCost(slope, config.slope_soft, config.slope_lethal),
    hard_residual_step ?
    featureCost(residual_step, config.residual_step_soft, config.residual_step_lethal) :
    std::min<unsigned char>(
      featureCost(residual_step, config.residual_step_soft, config.residual_step_lethal), 252),
    featureCost(roughness, config.roughness_soft, config.roughness_lethal)});
}

inline unsigned char terrainCost(
  const double slope, const double residual_step, const double roughness,
  const TerrainCostConfig & config, const bool hard_slope, const bool hard_residual_step,
  const bool hard_roughness)
{
  const auto effective_cost = [](const unsigned char cost, const bool hard_constraint) {
      return cost == 254 && !hard_constraint ? static_cast<unsigned char>(252) : cost;
    };
  return std::max({
    effective_cost(featureCost(slope, config.slope_soft, config.slope_lethal), hard_slope),
    effective_cost(
      featureCost(residual_step, config.residual_step_soft, config.residual_step_lethal),
      hard_residual_step),
    effective_cost(
      featureCost(roughness, config.roughness_soft, config.roughness_lethal), hard_roughness)});
}

inline unsigned char terrainCost(
  const double slope, const double residual_step, const double roughness,
  const TerrainCostConfig & config)
{
  return terrainCost(slope, residual_step, roughness, config, true);
}

// Static maps carry geometry separately from the robot capability.  Keep the
// class values numeric because ElevationMap layers are float matrices and NPZ
// serialization is float-based.
inline unsigned char terrainCost(
  const double terrain_class, const double clearance, const double slope,
  const double residual_step, const double roughness, const TerrainCostConfig & config,
  const bool hard_slope, const bool hard_residual_step, const bool hard_roughness)
{
  if (!std::isfinite(terrain_class) || terrain_class < 0.5) return 255;
  if (terrain_class >= 1.5) return 254;
  if (std::isfinite(clearance)) {
    // The static map stores clearance layers as float. Compare the loaded value
    // against the same float-quantized capability threshold used by the converter.
    const float stored_clearance = static_cast<float>(clearance);
    const float required_clearance = static_cast<float>(
      config.robot_height + config.clearance_margin);
    if (std::isfinite(stored_clearance) && std::isfinite(required_clearance) &&
      stored_clearance < required_clearance)
    {
      return 254;
    }
  }
  constexpr double kThresholdTolerance = 1.0e-9;
  if (hard_residual_step && std::isfinite(residual_step) &&
    residual_step >= config.max_climbable_step - kThresholdTolerance)
  {
    return 254;
  }
  if (hard_slope && std::isfinite(slope) &&
    slope >= config.max_traversable_slope - kThresholdTolerance)
  {
    return 254;
  }
  const auto effective_cost = [](const unsigned char cost, const bool hard_constraint) {
      return cost == 254 && !hard_constraint ? static_cast<unsigned char>(252) : cost;
    };
  return std::max({
    effective_cost(featureCost(slope, config.slope_soft, config.slope_lethal), hard_slope),
    effective_cost(
      featureCost(residual_step, config.residual_step_soft, config.residual_step_lethal),
      hard_residual_step),
    effective_cost(
      featureCost(roughness, config.roughness_soft, config.roughness_lethal), hard_roughness)});
}

inline unsigned char terrainCost(
  const double terrain_class, const double clearance, const double slope,
  const double residual_step, const double roughness, const TerrainCostConfig & config,
  const bool hard_feature_constraints)
{
  return terrainCost(
    terrain_class, clearance, slope, residual_step, roughness, config,
    hard_feature_constraints, hard_feature_constraints, hard_feature_constraints);
}

inline unsigned char terrainCost(
  const double terrain_class, const double clearance, const double slope,
  const double residual_step, const double roughness, const TerrainCostConfig & config)
{
  return terrainCost(
    terrain_class, clearance, slope, residual_step, roughness, config, true);
}
}  // namespace nav2_terrain_layer

#endif  // NAV2_TERRAIN_LAYER__TERRAIN_COST_HPP_
