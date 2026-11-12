#ifndef BRINGUP__GROUND_PROVENANCE_HPP_
#define BRINGUP__GROUND_PROVENANCE_HPP_

#include <cmath>
#include <cstdint>
#include <optional>
#include <string>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace bringup
{

enum class GroundSource : std::int8_t
{
  UNKNOWN = 0,
  MEASURED = 1
};

inline bool terrainGroundSourceMatches(
  const std::int8_t terrain_class, const std::int8_t ground_source,
  const std::int8_t map_source)
{
  if (terrain_class < 0 || terrain_class > 2 || ground_source < 0 || ground_source > 1 ||
    map_source < 0 || map_source > 1)
  {
    return false;
  }
  const auto expected_source = terrain_class == 1 ? 1 : 0;
  return ground_source == expected_source && map_source == expected_source;
}

inline std::optional<GroundSource> groundSourceAtWorldPoint(
  const nav_msgs::msg::OccupancyGrid & provenance,
  const double world_x,
  const double world_y,
  const std::string & expected_frame)
{
  const auto width = provenance.info.width;
  const auto height = provenance.info.height;
  const auto count = static_cast<std::uint64_t>(width) * height;
  if (provenance.header.frame_id != expected_frame || width == 0 || height == 0 ||
    !std::isfinite(provenance.info.resolution) || provenance.info.resolution <= 0.0F ||
    provenance.data.size() != count || !std::isfinite(world_x) || !std::isfinite(world_y))
  {
    return std::nullopt;
  }
  tf2::Transform world_from_grid;
  tf2::fromMsg(provenance.info.origin, world_from_grid);
  const auto grid = world_from_grid.inverse() * tf2::Vector3(world_x, world_y, 0.0);
  if (!std::isfinite(grid.x()) || !std::isfinite(grid.y()) || grid.x() < 0.0 || grid.y() < 0.0) {
    return std::nullopt;
  }
  const auto x = static_cast<std::uint64_t>(std::floor(grid.x() / provenance.info.resolution));
  const auto y = static_cast<std::uint64_t>(std::floor(grid.y() / provenance.info.resolution));
  if (x >= width || y >= height) return std::nullopt;
  const auto value = provenance.data[static_cast<std::size_t>(y) * width + x];
  if (value == static_cast<std::int8_t>(GroundSource::MEASURED)) return GroundSource::MEASURED;
  return GroundSource::UNKNOWN;
}

}  // namespace bringup

#endif  // BRINGUP__GROUND_PROVENANCE_HPP_
