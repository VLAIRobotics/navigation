#include "bringup/inflation_surface.hpp"

#include <algorithm>
#include <cmath>

namespace bringup
{

std::vector<InflationCell> inflationCells(
  const nav_msgs::msg::OccupancyGrid & grid, double radius)
{
  std::vector<InflationCell> cells;
  const auto width = grid.info.width;
  const auto height = grid.info.height;
  if (width == 0 || height == 0 || grid.data.size() != static_cast<std::size_t>(width) * height ||
    !std::isfinite(grid.info.resolution) || grid.info.resolution <= 0.0F ||
    !std::isfinite(radius) || radius < 0.0)
  {
    return cells;
  }

  std::vector<std::uint8_t> near_obstacle(grid.data.size(), 0);
  const int reach = static_cast<int>(std::ceil(radius / grid.info.resolution));
  const double reach_squared = radius * radius /
    (static_cast<double>(grid.info.resolution) * grid.info.resolution);
  for (std::uint32_t y = 0; y < height; ++y) {
    for (std::uint32_t x = 0; x < width; ++x) {
      if (grid.data[static_cast<std::size_t>(y) * width + x] < 100) continue;
      for (int dy = -reach; dy <= reach; ++dy) {
        for (int dx = -reach; dx <= reach; ++dx) {
          if (dx * dx + dy * dy > reach_squared) continue;
          const auto nx = static_cast<int>(x) + dx;
          const auto ny = static_cast<int>(y) + dy;
          if (nx < 0 || ny < 0 || nx >= static_cast<int>(width) || ny >= static_cast<int>(height)) {
            continue;
          }
          near_obstacle[static_cast<std::size_t>(ny) * width + nx] = 1;
        }
      }
    }
  }
  for (std::uint32_t y = 0; y < height; ++y) {
    for (std::uint32_t x = 0; x < width; ++x) {
      const auto index = static_cast<std::size_t>(y) * width + x;
      if (near_obstacle[index] && grid.data[index] > 0) {
        cells.push_back({x, y, grid.data[index]});
      }
    }
  }
  return cells;
}

InflationColor inflationColor(std::int8_t cost)
{
  if (cost <= 0) return {0.0F, 0.0F, 0.0F, 0.0F};
  if (cost == 99) return {0.0F, 1.0F, 1.0F, 1.0F};
  if (cost >= 100) return {1.0F, 0.0F, 1.0F, 1.0F};
  const auto red = (255 * static_cast<int>(cost)) / 100;
  const float value = static_cast<float>(red) / 255.0F;
  return {value, 0.0F, 1.0F - value, 1.0F};
}

bool applyCostmapUpdate(
  nav_msgs::msg::OccupancyGrid & grid, const map_msgs::msg::OccupancyGridUpdate & update,
  bool * changed)
{
  if (changed) *changed = false;
  const auto width = grid.info.width;
  const auto height = grid.info.height;
  if (grid.data.size() != static_cast<std::size_t>(width) * height ||
    update.x > width || update.y > height ||
    update.width > width - update.x || update.height > height - update.y ||
    update.data.size() != static_cast<std::size_t>(update.width) * update.height)
  {
    return false;
  }
  for (std::uint32_t y = 0; y < update.height; ++y) {
    for (std::uint32_t x = 0; x < update.width; ++x) {
      auto & target = grid.data[static_cast<std::size_t>(update.y + y) * width + update.x + x];
      const auto incoming = update.data[static_cast<std::size_t>(y) * update.width + x];
      if (changed && target != incoming) *changed = true;
      target = incoming;
    }
  }
  return true;
}

}  // namespace bringup
