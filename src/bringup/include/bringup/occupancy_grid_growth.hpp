#ifndef BRINGUP__OCCUPANCY_GRID_GROWTH_HPP_
#define BRINGUP__OCCUPANCY_GRID_GROWTH_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace bringup
{

struct GridGrowthResult
{
  bool changed{false};
  std::uint32_t old_offset_x{0};
  std::uint32_t old_offset_y{0};
};

inline bool growOccupancyGridToInclude(
  nav_msgs::msg::OccupancyGrid & map,
  std::int64_t min_cell_x, std::int64_t min_cell_y,
  std::int64_t max_cell_x, std::int64_t max_cell_y,
  std::uint32_t original_width, std::uint32_t original_height,
  std::int64_t original_offset_x, std::int64_t original_offset_y,
  double max_extension_m, std::size_t max_cells,
  GridGrowthResult * result, std::string * error = nullptr)
{
  const auto fail = [error](const std::string & reason) {
      if (error != nullptr) *error = reason;
      return false;
    };
  if (map.info.width == 0 || map.info.height == 0 || original_width == 0 ||
    original_height == 0 || !std::isfinite(map.info.resolution) ||
    map.info.resolution <= 0.0F || !std::isfinite(max_extension_m) ||
    max_extension_m < 0.0 || max_cells == 0 ||
    map.data.size() != static_cast<std::size_t>(map.info.width) * map.info.height ||
    min_cell_x > max_cell_x || min_cell_y > max_cell_y)
  {
    return fail("invalid occupancy map growth input");
  }

  const auto extension_cells = static_cast<std::int64_t>(
    std::floor(max_extension_m / static_cast<double>(map.info.resolution)));
  if (min_cell_x < original_offset_x - extension_cells ||
    min_cell_y < original_offset_y - extension_cells ||
    max_cell_x >= original_offset_x + static_cast<std::int64_t>(original_width) + extension_cells ||
    max_cell_y >= original_offset_y + static_cast<std::int64_t>(original_height) + extension_cells)
  {
    return fail("observed ground lies beyond configured map extension limit");
  }

  const auto new_min_x = std::min<std::int64_t>(0, min_cell_x);
  const auto new_min_y = std::min<std::int64_t>(0, min_cell_y);
  const auto new_max_x = std::max<std::int64_t>(map.info.width - 1, max_cell_x);
  const auto new_max_y = std::max<std::int64_t>(map.info.height - 1, max_cell_y);
  const auto new_width = static_cast<std::uint64_t>(new_max_x - new_min_x + 1);
  const auto new_height = static_cast<std::uint64_t>(new_max_y - new_min_y + 1);
  if (new_width == map.info.width && new_height == map.info.height) {
    if (result != nullptr) *result = {};
    return true;
  }
  if (new_width > std::numeric_limits<std::uint32_t>::max() ||
    new_height > std::numeric_limits<std::uint32_t>::max() ||
    new_width > max_cells / new_height)
  {
    return fail("expanded occupancy map exceeds configured cell limit");
  }

  const auto old_width = map.info.width;
  const auto old_height = map.info.height;
  const auto offset_x = static_cast<std::uint32_t>(-new_min_x);
  const auto offset_y = static_cast<std::uint32_t>(-new_min_y);
  std::vector<std::int8_t> expanded(static_cast<std::size_t>(new_width * new_height), -1);
  for (std::uint32_t y = 0; y < old_height; ++y) {
    const auto old_begin = map.data.begin() + static_cast<std::ptrdiff_t>(y) * old_width;
    const auto new_begin = expanded.begin() +
      static_cast<std::ptrdiff_t>(y + offset_y) * new_width + offset_x;
    std::copy_n(old_begin, old_width, new_begin);
  }

  tf2::Transform origin;
  tf2::fromMsg(map.info.origin, origin);
  const auto local_shift = tf2::Vector3(
    static_cast<double>(new_min_x) * map.info.resolution,
    static_cast<double>(new_min_y) * map.info.resolution, 0.0);
  origin.setOrigin(origin.getOrigin() + origin.getBasis() * local_shift);
  const auto shifted_origin = tf2::toMsg(origin);
  map.info.origin.position.x = shifted_origin.translation.x;
  map.info.origin.position.y = shifted_origin.translation.y;
  map.info.origin.position.z = shifted_origin.translation.z;
  map.info.origin.orientation = shifted_origin.rotation;
  map.info.width = static_cast<std::uint32_t>(new_width);
  map.info.height = static_cast<std::uint32_t>(new_height);
  map.data = std::move(expanded);
  if (result != nullptr) {
    result->changed = true;
    result->old_offset_x = offset_x;
    result->old_offset_y = offset_y;
  }
  return true;
}

}  // namespace bringup

#endif  // BRINGUP__OCCUPANCY_GRID_GROWTH_HPP_
