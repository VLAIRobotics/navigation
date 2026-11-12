#ifndef BRINGUP__FOOTPRINT_CELLS_HPP_
#define BRINGUP__FOOTPRINT_CELLS_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "bringup/consecutive_observation.hpp"
#include "bringup/path_collision_escape.hpp"

namespace bringup
{

struct FootprintPoint2D
{
  double x{0.0};
  double y{0.0};
};

// The polygon and optional expansion margin use meters in the occupancy grid's local frame.
inline std::vector<std::size_t> rasterizeFootprintCells(
  std::uint32_t width, std::uint32_t height, double resolution,
  const std::vector<FootprintPoint2D> & polygon,
  double margin_meters = 0.0)
{
  std::vector<std::size_t> cells;
  if (width == 0 || height == 0 || !std::isfinite(resolution) || resolution <= 0.0 ||
    !std::isfinite(margin_meters) || margin_meters < 0.0 || polygon.size() < 3)
  {
    return cells;
  }
  const double margin_cells = margin_meters / resolution;
  if (!std::isfinite(margin_cells)) {
    return cells;
  }

  double min_x = polygon.front().x;
  double max_x = min_x;
  double min_y = polygon.front().y;
  double max_y = min_y;
  for (const auto & point : polygon) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y)) return cells;
    min_x = std::min(min_x, point.x);
    max_x = std::max(max_x, point.x);
    min_y = std::min(min_y, point.y);
    max_y = std::max(max_y, point.y);
  }

  const double expanded_min_x = min_x - margin_meters;
  const double expanded_max_x = max_x + margin_meters;
  const double expanded_min_y = min_y - margin_meters;
  const double expanded_max_y = max_y + margin_meters;
  if (!std::isfinite(expanded_min_x) || !std::isfinite(expanded_max_x) ||
    !std::isfinite(expanded_min_y) || !std::isfinite(expanded_max_y))
  {
    return cells;
  }

  const double map_width = static_cast<double>(width) * resolution;
  const double map_height = static_cast<double>(height) * resolution;
  if (expanded_max_x < 0.0 || expanded_max_y < 0.0 ||
    expanded_min_x > map_width || expanded_min_y > map_height)
  {
    return cells;
  }

  std::vector<path_collision::Point2D> polygon_cells;
  polygon_cells.reserve(polygon.size());
  for (const auto & point : polygon) {
    polygon_cells.push_back({point.x / resolution, point.y / resolution});
  }
  const auto first_cell = [resolution](double value, std::uint32_t limit) {
      const double index = std::floor(value / resolution) - 1.0;
      return static_cast<std::uint32_t>(std::clamp(index, 0.0, static_cast<double>(limit - 1)));
    };
  const auto last_cell = [resolution](double value, std::uint32_t limit) {
      const double index = std::floor(value / resolution);
      return static_cast<std::uint32_t>(std::clamp(index, 0.0, static_cast<double>(limit - 1)));
    };
  const auto min_cell_x = first_cell(expanded_min_x, width);
  const auto max_cell_x = last_cell(expanded_max_x, width);
  const auto min_cell_y = first_cell(expanded_min_y, height);
  const auto max_cell_y = last_cell(expanded_max_y, height);
  if (min_cell_x > max_cell_x || min_cell_y > max_cell_y) return cells;

  for (std::uint32_t y = min_cell_y; y <= max_cell_y; ++y) {
    for (std::uint32_t x = min_cell_x; x <= max_cell_x; ++x) {
      if (path_collision::polygonIntersectsCell(polygon_cells, x, y, margin_cells)) {
        cells.push_back(static_cast<std::size_t>(y) * width + x);
      }
    }
  }
  return cells;
}

inline std::vector<std::uint8_t> remapPriorOccupiedCells(
  const std::vector<std::uint8_t> & prior_occupied,
  std::uint32_t old_width, std::uint32_t old_height,
  std::uint32_t new_width, std::uint32_t new_height,
  std::uint32_t old_offset_x, std::uint32_t old_offset_y)
{
  std::vector<std::uint8_t> remapped(static_cast<std::size_t>(new_width) * new_height, 0);
  for (std::uint32_t y = 0; y < old_height; ++y) {
    for (std::uint32_t x = 0; x < old_width; ++x) {
      remapped[static_cast<std::size_t>(y + old_offset_y) * new_width + x + old_offset_x] =
        prior_occupied[static_cast<std::size_t>(y) * old_width + x];
    }
  }
  return remapped;
}

inline void clearFootprintCells(
  std::vector<std::int8_t> & occupancy,
  const std::vector<std::uint8_t> & prior_occupied,
  std::vector<ConsecutiveObservation> & obstacle_evidence,
  std::vector<TimeWindowObservation> & ground_evidence,
  const std::vector<std::size_t> & footprint_cells,
  std::vector<std::size_t> & changed_cells,
  const std::vector<std::size_t> & veto_cells = {})
{
  for (const auto index : footprint_cells) {
    if (prior_occupied[index] ||
      std::binary_search(veto_cells.begin(), veto_cells.end(), index))
    {
      continue;
    }
    ground_evidence[index].reset();
    obstacle_evidence[index].reset();
    if (occupancy[index] != 0) {
      occupancy[index] = 0;
      changed_cells.push_back(index);
    }
  }
}

}  // namespace bringup

#endif  // BRINGUP__FOOTPRINT_CELLS_HPP_
