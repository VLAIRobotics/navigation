#ifndef TEB_LOCAL_PLANNER__COSTMAP_OBSTACLE_TILING_HPP_
#define TEB_LOCAL_PLANNER__COSTMAP_OBSTACLE_TILING_HPP_

#include <cstddef>
#include <cstdint>
#include <vector>

namespace teb_local_planner
{
namespace detail
{

struct CostmapObstacleTile
{
  // Cell indices are inclusive at both ends.
  unsigned int min_x;
  unsigned int min_y;
  unsigned int max_x;
  unsigned int max_y;
};

inline std::vector<CostmapObstacleTile> tileLethalCells(
  const std::vector<std::uint8_t> & lethal_cells,
  unsigned int width,
  unsigned int height,
  unsigned int max_cells_per_side)
{
  const auto cell_count = static_cast<std::size_t>(width) * height;
  if (width == 0 || height == 0 || max_cells_per_side == 0 ||
    lethal_cells.size() != cell_count)
  {
    return {};
  }

  std::vector<std::uint8_t> represented(cell_count, 0);
  std::vector<CostmapObstacleTile> tiles;
  for (unsigned int y = 0; y < height; ++y) {
    for (unsigned int x = 0; x < width; ++x) {
      const auto seed = static_cast<std::size_t>(y) * width + x;
      if (!lethal_cells[seed] || represented[seed]) {
        continue;
      }

      unsigned int max_x = x;
      while (max_x + 1 < width && max_x - x + 1 < max_cells_per_side) {
        const auto next = static_cast<std::size_t>(y) * width + max_x + 1;
        if (!lethal_cells[next] || represented[next]) {
          break;
        }
        ++max_x;
      }

      unsigned int max_y = y;
      while (max_y + 1 < height && max_y - y + 1 < max_cells_per_side) {
        bool full_row = true;
        for (unsigned int column = x; column <= max_x; ++column) {
          const auto next = static_cast<std::size_t>(max_y + 1) * width + column;
          if (!lethal_cells[next] || represented[next]) {
            full_row = false;
            break;
          }
        }
        if (!full_row) {
          break;
        }
        ++max_y;
      }

      for (unsigned int row = y; row <= max_y; ++row) {
        for (unsigned int column = x; column <= max_x; ++column) {
          represented[static_cast<std::size_t>(row) * width + column] = 1;
        }
      }
      tiles.push_back({x, y, max_x, max_y});
    }
  }
  return tiles;
}

}  // namespace detail
}  // namespace teb_local_planner

#endif  // TEB_LOCAL_PLANNER__COSTMAP_OBSTACLE_TILING_HPP_
