#ifndef BRINGUP__TEMPORAL_OBSTACLE_GRID_HPP_
#define BRINGUP__TEMPORAL_OBSTACLE_GRID_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace bringup
{
struct ObstacleGridPoint
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct TemporalObstacleGridParameters
{
  double resolution{0.10};
  int confirmation_window_frames{5};
  int confirmation_hits{2};
  int clear_ray_frames{2};
  int max_unobserved_frames{10};
  double local_window_size{6.0};
  double raytrace_height_tolerance{0.25};
  double raytrace_min_range{0.5};
  double raytrace_max_range{5.0};
};

struct TemporalObstacleGridDiagnostics
{
  std::size_t confirmed_cells{0};
  std::size_t newly_confirmed_cells{0};
  std::size_t ray_cleared_cells{0};
  std::size_t stale_cleared_cells{0};
  std::uint64_t newly_confirmed_cells_total{0};
  std::uint64_t ray_cleared_cells_total{0};
  std::uint64_t stale_cleared_cells_total{0};
  std::uint64_t oldest_confirmed_age_frames{0};
};

class TemporalObstacleGrid
{
public:
  explicit TemporalObstacleGrid(const TemporalObstacleGridParameters & parameters)
  : parameters_(parameters)
  {
    if (!std::isfinite(parameters_.resolution) || parameters_.resolution <= 0.0 ||
      parameters_.confirmation_window_frames < 1 ||
      parameters_.confirmation_window_frames > 64 || parameters_.confirmation_hits < 1 ||
      parameters_.confirmation_hits > parameters_.confirmation_window_frames ||
      parameters_.clear_ray_frames < 1 || parameters_.max_unobserved_frames < 1 ||
      !std::isfinite(parameters_.local_window_size) ||
      parameters_.local_window_size <= 0.0 ||
      !std::isfinite(parameters_.raytrace_height_tolerance) ||
      parameters_.raytrace_height_tolerance < 0.0 ||
      !std::isfinite(parameters_.raytrace_min_range) || parameters_.raytrace_min_range < 0.0 ||
      !std::isfinite(parameters_.raytrace_max_range) ||
      parameters_.raytrace_max_range <= parameters_.raytrace_min_range)
    {
      throw std::invalid_argument("invalid temporal obstacle grid parameter");
    }
    history_mask_ = parameters_.confirmation_window_frames == 64 ?
      std::numeric_limits<std::uint64_t>::max() :
      (std::uint64_t{1} << parameters_.confirmation_window_frames) - 1;
  }

  std::vector<ObstacleGridPoint> processFrame(
    const std::vector<ObstacleGridPoint> & hits,
    const std::vector<ObstacleGridPoint> & ray_endpoints,
    const ObstacleGridPoint & sensor_origin,
    bool current_hits_only = false)
  {
    diagnostics_ = {};
    cleared_points_.clear();
    ray_cleared_points_.clear();
    last_sensor_origin_ = sensor_origin;
    have_sensor_origin_ = true;
    pruneOutsideWindow(sensor_origin);

    for (auto it = cells_.begin(); it != cells_.end();) {
      it->second.hit_history = (it->second.hit_history << 1) & history_mask_;
      if (it->second.confirmed) {
        ++it->second.confirmed_age_frames;
        ++it->second.frames_since_last_hit;
      }
      if (!it->second.confirmed && it->second.hit_history == 0) {
        it = cells_.erase(it);
      } else {
        ++it;
      }
    }

    std::unordered_map<CellKey, ObstacleGridPoint, CellKeyHash> current_hits;
    current_hits.reserve(hits.size());
    for (const auto & point : hits) {
      if (!finite(point) || !withinWindow(point, sensor_origin)) {
        continue;
      }
      current_hits[keyFor(point)] = point;
    }
    for (const auto & [key, point] : current_hits) {
      auto & cell = cells_[key];
      cell.point = point;
      cell.hit_history |= 1;
      cell.consecutive_ray_misses = 0;
      cell.frames_since_last_hit = 0;
      if (!cell.confirmed && popcount(cell.hit_history) >= parameters_.confirmation_hits) {
        cell.confirmed = true;
        cell.confirmed_age_frames = 1;
        ++diagnostics_.newly_confirmed_cells;
        ++newly_confirmed_cells_total_;
      }
    }

    const auto missed_cells = rayMisses(ray_endpoints, sensor_origin);
    for (auto it = cells_.begin(); it != cells_.end();) {
      auto & cell = it->second;
      if (cell.confirmed && current_hits.count(it->first) == 0) {
        if (missed_cells.count(it->first) != 0) {
          ++cell.consecutive_ray_misses;
        } else {
          cell.consecutive_ray_misses = 0;
        }
        if (cell.consecutive_ray_misses >= parameters_.clear_ray_frames) {
          cleared_points_.push_back({
              (static_cast<double>(it->first.x) + 0.5) * parameters_.resolution,
              (static_cast<double>(it->first.y) + 0.5) * parameters_.resolution,
              cell.point.z});
          ray_cleared_points_.push_back(cleared_points_.back());
          ++diagnostics_.ray_cleared_cells;
          ++ray_cleared_cells_total_;
          it = cells_.erase(it);
          continue;
        }
      }
      if (cell.confirmed &&
        cell.frames_since_last_hit >= static_cast<std::uint64_t>(parameters_.max_unobserved_frames))
      {
        cleared_points_.push_back({
            (static_cast<double>(it->first.x) + 0.5) * parameters_.resolution,
            (static_cast<double>(it->first.y) + 0.5) * parameters_.resolution,
            cell.point.z});
        ++diagnostics_.stale_cleared_cells;
        ++stale_cleared_cells_total_;
        it = cells_.erase(it);
        continue;
      }
      ++it;
    }

    for (const auto & [key, cell] : cells_) {
      if (!cell.confirmed) {
        continue;
      }
      ++diagnostics_.confirmed_cells;
      diagnostics_.oldest_confirmed_age_frames = std::max(
        diagnostics_.oldest_confirmed_age_frames, cell.confirmed_age_frames);
    }
    diagnostics_.newly_confirmed_cells_total = newly_confirmed_cells_total_;
    diagnostics_.ray_cleared_cells_total = ray_cleared_cells_total_;
    diagnostics_.stale_cleared_cells_total = stale_cleared_cells_total_;
    if (current_hits_only) {
      // Downstream obstacle layers treat every published cell as a fresh observation.
      std::vector<ObstacleGridPoint> current_confirmed;
      current_confirmed.reserve(current_hits.size());
      for (const auto & hit : current_hits) {
        const auto & key = hit.first;
        const auto cell = cells_.find(key);
        if (cell == cells_.end() || !cell->second.confirmed) {
          continue;
        }
        current_confirmed.push_back({
          (static_cast<double>(key.x) + 0.5) * parameters_.resolution,
          (static_cast<double>(key.y) + 0.5) * parameters_.resolution,
          cell->second.point.z});
      }
      return current_confirmed;
    }
    return confirmedPoints();
  }

  std::vector<ObstacleGridPoint> ageWithoutObservation()
  {
    if (!have_sensor_origin_) {
      return {};
    }
    return processFrame({}, {}, last_sensor_origin_);
  }

  void reset()
  {
    cells_.clear();
    cleared_points_.clear();
    ray_cleared_points_.clear();
    diagnostics_ = {};
    diagnostics_.newly_confirmed_cells_total = newly_confirmed_cells_total_;
    diagnostics_.ray_cleared_cells_total = ray_cleared_cells_total_;
    diagnostics_.stale_cleared_cells_total = stale_cleared_cells_total_;
  }

  const TemporalObstacleGridDiagnostics & diagnostics() const
  {
    return diagnostics_;
  }

  std::vector<ObstacleGridPoint> confirmedPoints() const
  {
    std::vector<ObstacleGridPoint> result;
    result.reserve(cells_.size());
    for (const auto & [key, cell] : cells_) {
      if (!cell.confirmed) {
        continue;
      }
      result.push_back({
        (static_cast<double>(key.x) + 0.5) * parameters_.resolution,
        (static_cast<double>(key.y) + 0.5) * parameters_.resolution,
        cell.point.z});
    }
    return result;
  }

  std::vector<ObstacleGridPoint> takeClearedPoints()
  {
    auto result = std::move(cleared_points_);
    cleared_points_.clear();
    return result;
  }

  std::vector<ObstacleGridPoint> takeRayClearedPoints()
  {
    auto result = std::move(ray_cleared_points_);
    ray_cleared_points_.clear();
    return result;
  }

private:
  struct CellKey
  {
    std::int64_t x;
    std::int64_t y;

    bool operator==(const CellKey & other) const
    {
      return x == other.x && y == other.y;
    }
  };

  struct CellKeyHash
  {
    std::size_t operator()(const CellKey & key) const
    {
      const auto hx = std::hash<std::int64_t>{}(key.x);
      const auto hy = std::hash<std::int64_t>{}(key.y);
      return hx ^ (hy + static_cast<std::size_t>(0x9e3779b9) + (hx << 6) + (hx >> 2));
    }
  };

  struct Cell
  {
    ObstacleGridPoint point;
    std::uint64_t hit_history{0};
    int consecutive_ray_misses{0};
    bool confirmed{false};
    std::uint64_t confirmed_age_frames{0};
    std::uint64_t frames_since_last_hit{0};
  };

  static bool finite(const ObstacleGridPoint & point)
  {
    return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
  }

  bool withinWindow(
    const ObstacleGridPoint & point, const ObstacleGridPoint & sensor_origin) const
  {
    const double half_window = parameters_.local_window_size / 2.0 + parameters_.resolution;
    return std::abs(point.x - sensor_origin.x) <= half_window &&
           std::abs(point.y - sensor_origin.y) <= half_window;
  }

  CellKey keyFor(const ObstacleGridPoint & point) const
  {
    return {
      static_cast<std::int64_t>(std::floor(point.x / parameters_.resolution)),
      static_cast<std::int64_t>(std::floor(point.y / parameters_.resolution))};
  }

  void pruneOutsideWindow(const ObstacleGridPoint & sensor_origin)
  {
    for (auto it = cells_.begin(); it != cells_.end();) {
      const double x = (static_cast<double>(it->first.x) + 0.5) * parameters_.resolution;
      const double y = (static_cast<double>(it->first.y) + 0.5) * parameters_.resolution;
      const ObstacleGridPoint center{x, y, it->second.point.z};
      if (!withinWindow(center, sensor_origin)) {
        if (it->second.confirmed) {
          cleared_points_.push_back({x, y, it->second.point.z});
        }
        it = cells_.erase(it);
      } else {
        ++it;
      }
    }
  }

  static int popcount(std::uint64_t bits)
  {
    int count = 0;
    while (bits != 0) {
      bits &= bits - 1;
      ++count;
    }
    return count;
  }

  std::unordered_set<CellKey, CellKeyHash> rayMisses(
    const std::vector<ObstacleGridPoint> & endpoints,
    const ObstacleGridPoint & sensor_origin) const
  {
    std::unordered_set<CellKey, CellKeyHash> result;
    for (const auto & endpoint : endpoints) {
      if (!finite(endpoint)) {
        continue;
      }
      const double dx = endpoint.x - sensor_origin.x;
      const double dy = endpoint.y - sensor_origin.y;
      const double dz = endpoint.z - sensor_origin.z;
      const double range_squared = dx * dx + dy * dy + dz * dz;
      if (range_squared < parameters_.raytrace_min_range * parameters_.raytrace_min_range) {
        continue;
      }
      const double range = std::sqrt(range_squared);
      if (range <= std::numeric_limits<double>::epsilon()) {
        continue;
      }
      const double clip_fraction = std::min(parameters_.raytrace_max_range / range, 1.0);
      const double clipped_dx = dx * clip_fraction;
      const double clipped_dy = dy * clip_fraction;
      const double clipped_dz = dz * clip_fraction;
      const ObstacleGridPoint clipped_endpoint{
        sensor_origin.x + clipped_dx,
        sensor_origin.y + clipped_dy,
        sensor_origin.z + clipped_dz};
      const CellKey end = keyFor(clipped_endpoint);
      auto current = keyFor(sensor_origin);
      const int step_x = clipped_dx > 0.0 ? 1 : (clipped_dx < 0.0 ? -1 : 0);
      const int step_y = clipped_dy > 0.0 ? 1 : (clipped_dy < 0.0 ? -1 : 0);
      const double inf = std::numeric_limits<double>::infinity();
      const double delta_x = step_x == 0 ? inf : parameters_.resolution / std::abs(clipped_dx);
      const double delta_y = step_y == 0 ? inf : parameters_.resolution / std::abs(clipped_dy);
      const double boundary_x = step_x > 0 ?
        (static_cast<double>(current.x) + 1.0) * parameters_.resolution :
        static_cast<double>(current.x) * parameters_.resolution;
      const double boundary_y = step_y > 0 ?
        (static_cast<double>(current.y) + 1.0) * parameters_.resolution :
        static_cast<double>(current.y) * parameters_.resolution;
      double next_x = step_x == 0 ? inf : (boundary_x - sensor_origin.x) / clipped_dx;
      double next_y = step_y == 0 ? inf : (boundary_y - sensor_origin.y) / clipped_dy;
      std::size_t steps = 0;
      while (!(current == end) && steps++ < 512) {
        double ray_fraction;
        if (next_x < next_y) {
          ray_fraction = next_x;
          current.x += step_x;
          next_x += delta_x;
        } else if (next_y < next_x) {
          ray_fraction = next_y;
          current.y += step_y;
          next_y += delta_y;
        } else {
          ray_fraction = next_x;
          current.x += step_x;
          current.y += step_y;
          next_x += delta_x;
          next_y += delta_y;
        }
        if (!(current == end)) {
          const auto cell = cells_.find(current);
          if (cell != cells_.end()) {
            const double ray_height = sensor_origin.z + ray_fraction * clipped_dz;
            if (std::abs(ray_height - cell->second.point.z) <=
              parameters_.raytrace_height_tolerance)
            {
              result.insert(current);
            }
          }
        }
      }
    }
    return result;
  }

  TemporalObstacleGridParameters parameters_;
  std::uint64_t history_mask_{0};
  std::unordered_map<CellKey, Cell, CellKeyHash> cells_;
  std::vector<ObstacleGridPoint> cleared_points_;
  std::vector<ObstacleGridPoint> ray_cleared_points_;
  TemporalObstacleGridDiagnostics diagnostics_;
  std::uint64_t newly_confirmed_cells_total_{0};
  std::uint64_t ray_cleared_cells_total_{0};
  std::uint64_t stale_cleared_cells_total_{0};
  ObstacleGridPoint last_sensor_origin_;
  bool have_sensor_origin_{false};
};
}  // namespace bringup

#endif  // BRINGUP__TEMPORAL_OBSTACLE_GRID_HPP_
