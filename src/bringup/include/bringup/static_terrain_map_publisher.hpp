#ifndef BRINGUP__STATIC_TERRAIN_MAP_PUBLISHER_HPP_
#define BRINGUP__STATIC_TERRAIN_MAP_PUBLISHER_HPP_

#include <array>
#include <cstdint>
#include <optional>
#include <cstddef>
#include <limits>
#include <string>

#include <fastdem/elevation_map.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <tf2/LinearMath/Transform.h>

namespace bringup
{

/**
 * Load and validate the static terrain map used by the global costmap.
 *
 * Keeping this validation shared with the visualization node prevents RViz
 * from silently showing a different or incomplete map than Nav2 consumes.
 */
bool loadStaticTerrainMap(
  const std::string & path, const std::string & expected_frame,
  fastdem::ElevationMap & map, std::string * error = nullptr);

// Filter the display PCD without changing the navigation map.
bool keepStaticDisplayPoint(
  const fastdem::ElevationMap & map, float x, float y, float z,
  float max_height_above_ground);

std::optional<float> staticDisplayHeightAboveGround(
  const fastdem::ElevationMap & map, float x, float y, float z,
  float fallback_ground = std::numeric_limits<float>::quiet_NaN());

std::array<std::uint8_t, 3> staticDisplayColor(float height_above_ground);

// Build the display-only elevation and relief layers; returns isolated spikes removed.
std::size_t prepareStaticDisplayMap(fastdem::ElevationMap & map);

// Validate grid alignment and expand the display only when occupancy bounds are not covered.
bool extendDisplayMapToOccupancyGrid(
  fastdem::ElevationMap & terrain_map,
  const nav_msgs::msg::OccupancyGrid & occupancy_map,
  bool * changed = nullptr,
  std::string * error = nullptr);

// Overlay fresh, observed FastDEM terrain cells in the display map's frame.
std::size_t mergeOnlineDisplayTerrain(
  fastdem::ElevationMap & display_map, const nanogrid::GridMap & online_map,
  const tf2::Transform & display_from_online,
  int min_observations = 3, int max_observation_age = 3);

}  // namespace bringup

#endif  // BRINGUP__STATIC_TERRAIN_MAP_PUBLISHER_HPP_
