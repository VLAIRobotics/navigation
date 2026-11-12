#ifndef BRINGUP__TERRAIN_MAP_UTILS_HPP_
#define BRINGUP__TERRAIN_MAP_UTILS_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

namespace bringup
{

struct HeightEvidenceBin
{
  int index{0};
  std::uint32_t count{0};
  float min_z{std::numeric_limits<float>::infinity()};
};

enum class TerrainCellClass : std::uint8_t
{
  Unknown = 0,
  Ground = 1,
  Obstacle = 2,
};

// An estimated height borrowed from a neighbouring ground cell is sufficient
// to confirm a solid obstacle, but it must not make the source cell free.
inline TerrainCellClass classify_terrain_cell(
  bool ground_valid, float estimated_ground, float obstacle_point,
  float solid_obstacle_clearance)
{
  if (!std::isfinite(estimated_ground) || !std::isfinite(solid_obstacle_clearance)) {
    return TerrainCellClass::Unknown;
  }
  if (std::isfinite(obstacle_point) &&
    obstacle_point - estimated_ground < solid_obstacle_clearance)
  {
    return TerrainCellClass::Obstacle;
  }
  return ground_valid ? TerrainCellClass::Ground : TerrainCellClass::Unknown;
}

inline std::uint8_t terrain_cell_occupancy(TerrainCellClass terrain_class)
{
  switch (terrain_class) {
    case TerrainCellClass::Ground:
      return 254;
    case TerrainCellClass::Obstacle:
      return 0;
    case TerrainCellClass::Unknown:
    default:
      return 205;
  }
}

// Return the first supported return above a ground estimate.  A single range
// return (the common 5 cm speckle seen in the map) is deliberately not enough
// evidence to make a lethal cell; the count and fraction guards are applied to
// the complete per-cell histogram, not to a bounded reservoir sample.
inline float supported_height_above(
  const std::vector<HeightEvidenceBin> & bins, std::uint64_t total_points,
  float ground, float minimum_gap, std::uint32_t minimum_points,
  double minimum_fraction)
{
  if (!std::isfinite(ground) || !std::isfinite(minimum_gap) || minimum_gap < 0.0F ||
    total_points == 0 || minimum_points == 0 || !std::isfinite(minimum_fraction) ||
    minimum_fraction <= 0.0 || minimum_fraction > 1.0)
  {
    return std::numeric_limits<float>::quiet_NaN();
  }
  const float threshold = ground + minimum_gap;
  if (!std::isfinite(threshold)) return std::numeric_limits<float>::quiet_NaN();
  const auto required = std::max<std::uint64_t>(
    minimum_points, static_cast<std::uint64_t>(std::ceil(total_points * minimum_fraction)));
  float result = std::numeric_limits<float>::infinity();
  for (const auto & bin : bins) {
    if (bin.count < required || !std::isfinite(bin.min_z) || bin.min_z <= threshold) continue;
    result = std::min(result, bin.min_z);
  }
  return std::isfinite(result) ? result : std::numeric_limits<float>::quiet_NaN();
}

inline std::size_t checked_terrain_cell_count(
  double length_x, double length_y, double resolution, std::size_t max_cells)
{
  if (!std::isfinite(length_x) || !std::isfinite(length_y) ||
    !std::isfinite(resolution) || length_x <= 0.0 || length_y <= 0.0 ||
    resolution <= 0.0 || max_cells == 0)
  {
    throw std::invalid_argument("invalid terrain map geometry");
  }
  const long double rows = std::ceil(static_cast<long double>(length_x) / resolution);
  const long double cols = std::ceil(static_cast<long double>(length_y) / resolution);
  if (rows < 1.0L || cols < 1.0L ||
    rows > static_cast<long double>(std::numeric_limits<std::size_t>::max()) ||
    cols > static_cast<long double>(std::numeric_limits<std::size_t>::max()) ||
    rows * cols > static_cast<long double>(max_cells))
  {
    throw std::runtime_error("terrain map exceeds max_cells");
  }
  return static_cast<std::size_t>(rows) * static_cast<std::size_t>(cols);
}

inline void expand_occupied_cells(
  std::vector<std::uint8_t> & occupancy, const std::vector<bool> & occupied,
  std::size_t rows, std::size_t cols, int radius_cells)
{
  if (rows == 0 || cols == 0 || occupancy.size() != rows * cols ||
    occupied.size() != rows * cols || radius_cells < 0)
  {
    throw std::invalid_argument("invalid occupancy grid for expansion");
  }
  for (std::size_t i = 0; i < rows; ++i) {
    for (std::size_t j = 0; j < cols; ++j) {
      if (!occupied[i * cols + j]) continue;
      for (int di = -radius_cells; di <= radius_cells; ++di) {
        for (int dj = -radius_cells; dj <= radius_cells; ++dj) {
          if (di * di + dj * dj > radius_cells * radius_cells) continue;
          const auto ni = static_cast<long long>(i) + di;
          const auto nj = static_cast<long long>(j) + dj;
          if (ni < 0 || nj < 0 || ni >= static_cast<long long>(rows) ||
            nj >= static_cast<long long>(cols)) continue;
          occupancy[static_cast<std::size_t>(ni) * cols + static_cast<std::size_t>(nj)] = 0;
        }
      }
    }
  }
}

// GridMap stores cells as [x_index, y_index], while a PGM stores pixels as
// [image_row, image_column].  The image column is world +X and the image row
// is world +Y, so non-square maps require an explicit transpose and X flip.
inline std::vector<std::uint8_t> occupancy_to_pgm(
  const std::vector<std::uint8_t> & occupancy, std::size_t rows, std::size_t cols)
{
  if (rows == 0 || cols == 0 || occupancy.size() != rows * cols) {
    throw std::invalid_argument("invalid occupancy grid for PGM conversion");
  }
  std::vector<std::uint8_t> pixels(rows * cols);
  for (std::size_t image_row = 0; image_row < cols; ++image_row) {
    for (std::size_t image_col = 0; image_col < rows; ++image_col) {
      const std::size_t grid_row = rows - 1 - image_col;
      const std::size_t grid_col = image_row;
      pixels[image_row * rows + image_col] = occupancy[grid_row * cols + grid_col];
    }
  }
  return pixels;
}

inline std::vector<bool> retain_ground_largest_connected_component(
  const std::vector<float> & elevation, const std::vector<bool> & candidate,
  std::size_t rows, std::size_t cols, double max_step)
{
  if (rows == 0 || cols == 0 || elevation.size() != rows * cols ||
    candidate.size() != rows * cols || !std::isfinite(max_step) || max_step <= 0.0)
  {
    throw std::invalid_argument("invalid ground connectivity grid");
  }

  std::vector<bool> connected(elevation.size(), false);
  std::vector<int> component(elevation.size(), -1);
  std::size_t best_size = 0;
  int best_component = -1;
  float best_min_elevation = std::numeric_limits<float>::infinity();
  int component_id = 0;

  for (std::size_t start = 0; start < elevation.size(); ++start) {
    if (!candidate[start] || !std::isfinite(elevation[start]) || component[start] >= 0) continue;
    std::deque<std::size_t> queue{start};
    component[start] = component_id;
    std::size_t size = 0;
    float min_elevation = std::numeric_limits<float>::infinity();
    while (!queue.empty()) {
      const auto flat = queue.front();
      queue.pop_front();
      ++size;
      min_elevation = std::min(min_elevation, elevation[flat]);
      const auto row = flat / cols;
      const auto col = flat % cols;
      for (int drow = -1; drow <= 1; ++drow) {
        for (int dcol = -1; dcol <= 1; ++dcol) {
          if (drow == 0 && dcol == 0) continue;
          const auto next_row = static_cast<long long>(row) + drow;
          const auto next_col = static_cast<long long>(col) + dcol;
          if (next_row < 0 || next_col < 0 ||
            next_row >= static_cast<long long>(rows) ||
            next_col >= static_cast<long long>(cols))
          {
            continue;
          }
          const auto next = static_cast<std::size_t>(next_row) * cols +
            static_cast<std::size_t>(next_col);
          if (component[next] >= 0 || !candidate[next] || !std::isfinite(elevation[next]) ||
            std::abs(elevation[next] - elevation[flat]) > max_step)
          {
            continue;
          }
          component[next] = component_id;
          queue.push_back(next);
        }
      }
    }
    if (size > best_size || (size == best_size && min_elevation < best_min_elevation)) {
      best_size = size;
      best_component = component_id;
      best_min_elevation = min_elevation;
    }
    ++component_id;
  }

  if (best_component >= 0) {
    for (std::size_t i = 0; i < component.size(); ++i) {
      connected[i] = component[i] == best_component;
    }
  }
  return connected;
}

inline std::vector<bool> retain_ground_components_by_area(
  const std::vector<float> & elevation, const std::vector<bool> & candidate,
  std::size_t rows, std::size_t cols, double max_step, std::size_t min_component_size,
  double max_component_height_delta = std::numeric_limits<double>::infinity())
{
  if (rows == 0 || cols == 0 || elevation.size() != rows * cols ||
    candidate.size() != rows * cols || !std::isfinite(max_step) || max_step <= 0.0 ||
    min_component_size == 0 || std::isnan(max_component_height_delta) ||
    max_component_height_delta < 0.0)
  {
    throw std::invalid_argument("invalid ground connectivity grid");
  }

  std::vector<bool> retained(elevation.size(), false);
  std::vector<bool> visited(elevation.size(), false);
  std::vector<std::vector<std::size_t>> components;
  std::vector<float> component_min_elevations;
  std::size_t largest_component_size = 0;
  float largest_component_min_elevation = std::numeric_limits<float>::infinity();
  for (std::size_t start = 0; start < elevation.size(); ++start) {
    if (!candidate[start] || visited[start] || !std::isfinite(elevation[start])) continue;
    std::deque<std::size_t> queue{start};
    std::vector<std::size_t> component;
    float component_min_elevation = std::numeric_limits<float>::infinity();
    visited[start] = true;
    while (!queue.empty()) {
      const auto flat = queue.front();
      queue.pop_front();
      component.push_back(flat);
      component_min_elevation = std::min(component_min_elevation, elevation[flat]);
      const auto row = flat / cols;
      const auto col = flat % cols;
      for (int drow = -1; drow <= 1; ++drow) {
        for (int dcol = -1; dcol <= 1; ++dcol) {
          if (drow == 0 && dcol == 0) continue;
          const auto next_row = static_cast<long long>(row) + drow;
          const auto next_col = static_cast<long long>(col) + dcol;
          if (next_row < 0 || next_col < 0 ||
            next_row >= static_cast<long long>(rows) ||
            next_col >= static_cast<long long>(cols)) continue;
          const auto next = static_cast<std::size_t>(next_row) * cols +
            static_cast<std::size_t>(next_col);
          if (visited[next] || !candidate[next] || !std::isfinite(elevation[next]) ||
            std::abs(elevation[next] - elevation[flat]) > max_step) continue;
          visited[next] = true;
          queue.push_back(next);
        }
      }
    }
    if (component.size() > largest_component_size ||
      (component.size() == largest_component_size &&
      component_min_elevation < largest_component_min_elevation))
    {
      largest_component_size = component.size();
      largest_component_min_elevation = component_min_elevation;
    }
    components.push_back(std::move(component));
    component_min_elevations.push_back(component_min_elevation);
  }
  if (!std::isfinite(largest_component_min_elevation)) return retained;
  for (std::size_t component_index = 0; component_index < components.size(); ++component_index) {
    const auto & component = components[component_index];
    if (component.size() < min_component_size ||
      static_cast<double>(component_min_elevations[component_index]) >
      static_cast<double>(largest_component_min_elevation) + max_component_height_delta)
    {
      continue;
    }
    for (const auto flat : component) retained[flat] = true;
  }
  return retained;
}

}  // namespace bringup

#endif  // BRINGUP__TERRAIN_MAP_UTILS_HPP_
