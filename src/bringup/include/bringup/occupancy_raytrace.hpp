#ifndef BRINGUP__OCCUPANCY_RAYTRACE_HPP_
#define BRINGUP__OCCUPANCY_RAYTRACE_HPP_

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <iterator>
#include <vector>

namespace bringup
{

struct OccupancyGridCell
{
  std::int64_t x;
  std::int64_t y;
};

inline std::vector<std::size_t> excludeCurrentObstacleHitsFromRayClears(
  std::vector<std::size_t> ray_clear_cells,
  std::vector<std::size_t> same_frame_hit_cells)
{
  const auto unique_cells = [](std::vector<std::size_t> & cells) {
      std::sort(cells.begin(), cells.end());
      cells.erase(std::unique(cells.begin(), cells.end()), cells.end());
    };
  unique_cells(ray_clear_cells);
  unique_cells(same_frame_hit_cells);
  std::vector<std::size_t> result;
  result.reserve(ray_clear_cells.size());
  std::set_difference(
    ray_clear_cells.begin(), ray_clear_cells.end(),
    same_frame_hit_cells.begin(), same_frame_hit_cells.end(),
    std::back_inserter(result));
  return result;
}

// Return the cells crossed before the return endpoint. The endpoint contains
// the observed return and therefore is not free-space evidence.
inline std::vector<std::size_t> rasterizeFreeRay(
  OccupancyGridCell start, OccupancyGridCell endpoint,
  std::uint32_t width, std::uint32_t height,
  const std::vector<std::uint8_t>* eligible_cells = nullptr)
{
  std::vector<std::size_t> cells;
  if (width == 0 || height == 0 ||
    (start.x == endpoint.x && start.y == endpoint.y))
  {
    return cells;
  }

  const auto dx = std::llabs(endpoint.x - start.x);
  const auto sx = start.x < endpoint.x ? 1 : -1;
  const auto dy = -std::llabs(endpoint.y - start.y);
  const auto sy = start.y < endpoint.y ? 1 : -1;
  auto error = dx + dy;
  auto x = start.x;
  auto y = start.y;

  while (x != endpoint.x || y != endpoint.y) {
    if (x >= 0 && y >= 0 && x < width && y < height) {
      const auto index = static_cast<std::size_t>(y) * width + static_cast<std::size_t>(x);
      if (eligible_cells == nullptr ||
        (index < eligible_cells->size() && (*eligible_cells)[index] != 0))
      {
        cells.push_back(index);
      }
    }
    const auto doubled_error = 2 * error;
    if (doubled_error >= dy) {
      error += dy;
      x += sx;
    }
    if (doubled_error <= dx) {
      error += dx;
      y += sy;
    }
  }
  return cells;
}

}  // namespace bringup

#endif  // BRINGUP__OCCUPANCY_RAYTRACE_HPP_
