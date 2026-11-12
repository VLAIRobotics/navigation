#ifndef BRINGUP__PATH_COLLISION_ESCAPE_HPP_
#define BRINGUP__PATH_COLLISION_ESCAPE_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

#include <nav2_costmap_2d/cost_values.hpp>

namespace bringup
{
namespace path_collision
{
struct Point2D
{
  double x;
  double y;
};

inline constexpr double kGeometryEpsilon = 1.0e-9;
inline constexpr double kMaxSampleDisplacementCells = 0.25;

inline double cross(const Point2D & a, const Point2D & b, const Point2D & c)
{
  return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}

inline bool pointOnSegment(const Point2D & point, const Point2D & a, const Point2D & b)
{
  return std::abs(cross(a, b, point)) <= kGeometryEpsilon &&
         point.x >= std::min(a.x, b.x) - kGeometryEpsilon &&
         point.x <= std::max(a.x, b.x) + kGeometryEpsilon &&
         point.y >= std::min(a.y, b.y) - kGeometryEpsilon &&
         point.y <= std::max(a.y, b.y) + kGeometryEpsilon;
}

inline bool segmentsIntersect(
  const Point2D & a, const Point2D & b, const Point2D & c, const Point2D & d)
{
  const double ab_c = cross(a, b, c);
  const double ab_d = cross(a, b, d);
  const double cd_a = cross(c, d, a);
  const double cd_b = cross(c, d, b);
  if (((ab_c > kGeometryEpsilon && ab_d < -kGeometryEpsilon) ||
    (ab_c < -kGeometryEpsilon && ab_d > kGeometryEpsilon)) &&
    ((cd_a > kGeometryEpsilon && cd_b < -kGeometryEpsilon) ||
    (cd_a < -kGeometryEpsilon && cd_b > kGeometryEpsilon)))
  {
    return true;
  }
  return pointOnSegment(c, a, b) || pointOnSegment(d, a, b) ||
         pointOnSegment(a, c, d) || pointOnSegment(b, c, d);
}

inline bool pointInPolygon(const Point2D & point, const std::vector<Point2D> & polygon)
{
  bool inside = false;
  for (std::size_t i = 0, previous = polygon.size() - 1; i < polygon.size(); previous = i++) {
    const auto & a = polygon[previous];
    const auto & b = polygon[i];
    if (pointOnSegment(point, a, b)) {
      return true;
    }
    const bool crosses_y = (a.y > point.y) != (b.y > point.y);
    if (crosses_y) {
      const double crossing_x = a.x + (point.y - a.y) * (b.x - a.x) / (b.y - a.y);
      if (point.x < crossing_x) {
        inside = !inside;
      }
    }
  }
  return inside;
}

inline bool polygonIntersectsRect(
  const std::vector<Point2D> & polygon,
  double left, double bottom, double right, double top)
{
  for (const auto & point : polygon) {
    if (point.x >= left - kGeometryEpsilon && point.x <= right + kGeometryEpsilon &&
      point.y >= bottom - kGeometryEpsilon && point.y <= top + kGeometryEpsilon)
    {
      return true;
    }
  }

  const Point2D corners[] = {
    {left, bottom}, {right, bottom}, {right, top}, {left, top}};
  for (const auto & corner : corners) {
    if (pointInPolygon(corner, polygon)) {
      return true;
    }
  }

  for (std::size_t i = 0; i < polygon.size(); ++i) {
    const auto & a = polygon[i];
    const auto & b = polygon[(i + 1) % polygon.size()];
    for (std::size_t edge = 0; edge < 4; ++edge) {
      if (segmentsIntersect(a, b, corners[edge], corners[(edge + 1) % 4])) {
        return true;
      }
    }
  }
  return false;
}

inline bool polygonIntersectsCell(
  const std::vector<Point2D> & polygon,
  std::int64_t x, std::int64_t y, double margin_cells)
{
  return polygonIntersectsRect(
    polygon, static_cast<double>(x) - margin_cells,
    static_cast<double>(y) - margin_cells,
    static_cast<double>(x + 1) + margin_cells,
    static_cast<double>(y + 1) + margin_cells);
}

inline bool hasUsableArea(const std::vector<Point2D> & polygon)
{
  if (polygon.size() < 3) {
    return false;
  }
  double twice_area = 0.0;
  for (std::size_t i = 0; i < polygon.size(); ++i) {
    const auto & a = polygon[i];
    const auto & b = polygon[(i + 1) % polygon.size()];
    if (std::hypot(b.x - a.x, b.y - a.y) <= kGeometryEpsilon) {
      return false;
    }
    twice_area += a.x * b.y - b.x * a.y;
  }
  if (!std::isfinite(twice_area) || std::abs(twice_area) <= kGeometryEpsilon) {
    return false;
  }
  for (std::size_t first = 0; first < polygon.size(); ++first) {
    const std::size_t first_next = (first + 1) % polygon.size();
    for (std::size_t second = first + 1; second < polygon.size(); ++second) {
      const std::size_t second_next = (second + 1) % polygon.size();
      if (first_next == second || second_next == first) {
        continue;
      }
      if (segmentsIntersect(
          polygon[first], polygon[first_next], polygon[second], polygon[second_next]))
      {
        return false;
      }
    }
  }
  return true;
}

inline bool transformFootprintToGrid(
  const std::vector<Point2D> & footprint_in_base,
  double start_cell_x,
  double start_cell_y,
  double yaw,
  double resolution,
  std::vector<Point2D> & footprint_in_grid)
{
  if (footprint_in_base.size() < 3 || !std::isfinite(start_cell_x) ||
    !std::isfinite(start_cell_y) || !std::isfinite(yaw) ||
    !std::isfinite(resolution) || resolution <= 0.0)
  {
    return false;
  }
  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);
  footprint_in_grid.clear();
  footprint_in_grid.reserve(footprint_in_base.size());
  for (const auto & point : footprint_in_base) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y)) {
      footprint_in_grid.clear();
      return false;
    }
    footprint_in_grid.push_back({
      start_cell_x + (cos_yaw * point.x - sin_yaw * point.y) / resolution,
      start_cell_y + (sin_yaw * point.x + cos_yaw * point.y) / resolution});
  }
  return true;
}

inline bool collectUnknownCells(
  const std::vector<std::uint8_t> & costs,
  std::size_t width,
  std::size_t height,
  const std::vector<Point2D> & polygon,
  double margin_cells,
  std::size_t max_candidate_cells,
  std::vector<std::size_t> & cells)
{
  if (width == 0 || height == 0 ||
    width > std::numeric_limits<std::size_t>::max() / height ||
    width > static_cast<std::size_t>(std::numeric_limits<std::int64_t>::max()) ||
    height > static_cast<std::size_t>(std::numeric_limits<std::int64_t>::max()) ||
    costs.size() != width * height || !std::isfinite(margin_cells) || margin_cells < 0.0 ||
    polygon.size() < 3 || max_candidate_cells == 0)
  {
    return false;
  }

  double min_x = polygon.front().x;
  double max_x = polygon.front().x;
  double min_y = polygon.front().y;
  double max_y = polygon.front().y;
  for (const auto & point : polygon) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y)) {
      return false;
    }
    min_x = std::min(min_x, point.x);
    max_x = std::max(max_x, point.x);
    min_y = std::min(min_y, point.y);
    max_y = std::max(max_y, point.y);
  }

  if (max_x + margin_cells < 0.0 || max_y + margin_cells < 0.0 ||
    min_x - margin_cells >= static_cast<double>(width) ||
    min_y - margin_cells >= static_cast<double>(height))
  {
    return true;
  }
  const auto first_x = static_cast<std::int64_t>(std::max(
      0.0, std::floor(min_x - 1.0 - margin_cells)));
  const auto last_x = static_cast<std::int64_t>(std::min(
      static_cast<double>(width - 1), std::ceil(max_x + margin_cells)));
  const auto first_y = static_cast<std::int64_t>(std::max(
      0.0, std::floor(min_y - 1.0 - margin_cells)));
  const auto last_y = static_cast<std::int64_t>(std::min(
      static_cast<double>(height - 1), std::ceil(max_y + margin_cells)));
  if (last_x < first_x || last_y < first_y) {
    return true;
  }

  const auto candidate_cells = static_cast<std::uint64_t>(last_x - first_x + 1) *
    static_cast<std::uint64_t>(last_y - first_y + 1);
  if (candidate_cells > max_candidate_cells) {
    return false;
  }

  for (auto y = first_y; y <= last_y; ++y) {
    for (auto x = first_x; x <= last_x; ++x) {
      if (!polygonIntersectsCell(polygon, x, y, margin_cells)) {
        continue;
      }
      const auto index = static_cast<std::size_t>(y) * width + static_cast<std::size_t>(x);
      if (costs[index] == nav2_costmap_2d::NO_INFORMATION) {
        cells.push_back(index);
      }
    }
  }
  return true;
}

inline bool isBlockedCell(std::uint8_t cost, bool allow_unknown)
{
  return cost >= nav2_costmap_2d::LETHAL_OBSTACLE &&
         !(cost == nav2_costmap_2d::NO_INFORMATION && allow_unknown);
}

class StartEscapeMask
{
public:
  StartEscapeMask(
    const std::vector<std::uint8_t> & costs,
    std::size_t width,
    std::size_t height,
    double resolution,
    double start_x,
    double start_y,
    double unknown_escape_radius,
    const std::vector<std::size_t> & initial_unknown_cells = {})
  : cells_(costs.size(), 0)
  {
    if (width == 0 || height == 0 || costs.size() != width * height ||
      !std::isfinite(resolution) || resolution <= 0.0 || !std::isfinite(start_x) ||
      !std::isfinite(start_y) || !std::isfinite(unknown_escape_radius) ||
      unknown_escape_radius <= 0.0 || start_x < 0.0 || start_y < 0.0 ||
      start_x >= static_cast<double>(width) || start_y >= static_cast<double>(height))
    {
      return;
    }

    const auto start_cell_x = static_cast<std::size_t>(std::floor(start_x));
    const auto start_cell_y = static_cast<std::size_t>(std::floor(start_y));
    const auto start_index = start_cell_y * width + start_cell_x;
    const auto start_cost = costs[start_index];
    if (start_cost == nav2_costmap_2d::LETHAL_OBSTACLE) {
      return;
    }

    bool initial_footprint_has_unknown = false;
    for (const auto index : initial_unknown_cells) {
      if (index < costs.size() && costs[index] == nav2_costmap_2d::NO_INFORMATION) {
        initial_footprint_has_unknown = true;
        break;
      }
    }

    if (start_cost != nav2_costmap_2d::NO_INFORMATION && !initial_footprint_has_unknown) {
      return;
    }

    const double radius_cells = unknown_escape_radius / resolution;
    if (!std::isfinite(radius_cells)) {
      return;
    }
    const auto first_x = static_cast<std::size_t>(std::max(
      0.0, std::floor(start_x - radius_cells) - 1.0));
    const auto first_y = static_cast<std::size_t>(std::max(
      0.0, std::floor(start_y - radius_cells) - 1.0));
    const auto last_x = static_cast<std::size_t>(std::min(
      static_cast<double>(width - 1), std::ceil(start_x + radius_cells)));
    const auto last_y = static_cast<std::size_t>(std::min(
      static_cast<double>(height - 1), std::ceil(start_y + radius_cells)));

    bool has_unknown_in_disk = false;
    for (std::size_t y = first_y; y <= last_y; ++y) {
      for (std::size_t x = first_x; x <= last_x; ++x) {
        // Costmap cells represent their full square. Include a cell when that
        // square intersects the start-centred escape disk; do not require an
        // unknown/inscribed connectivity path from the robot's start cell.
        const double dx = std::max({0.0, static_cast<double>(x) - start_x,
          start_x - static_cast<double>(x + 1)}) * resolution;
        const double dy = std::max({0.0, static_cast<double>(y) - start_y,
          start_y - static_cast<double>(y + 1)}) * resolution;
        if (dx * dx + dy * dy > unknown_escape_radius * unknown_escape_radius + 1.0e-12) {
          continue;
        }
        const auto index = y * width + x;
        cells_[index] |= kStartEscape;
        has_unknown_in_disk = has_unknown_in_disk ||
          costs[index] == nav2_costmap_2d::NO_INFORMATION;
      }
    }
    start_escape_active_ = has_unknown_in_disk;
  }

  bool contains_start_escape(std::size_t index) const
  {
    return start_escape_active_ && index < cells_.size() && (cells_[index] & kStartEscape) != 0;
  }

  bool allows_unknown(std::size_t index) const
  {
    return contains_start_escape(index);
  }

  void finish_pose(bool footprint_overlaps_start_escape)
  {
    if (start_escape_active_ && !footprint_overlaps_start_escape) {
      start_escape_active_ = false;
    }
  }

private:
  static constexpr std::uint8_t kStartEscape = 1U;

  std::vector<std::uint8_t> cells_;
  bool start_escape_active_{false};
};
}  // namespace path_collision
}  // namespace bringup

#endif  // BRINGUP__PATH_COLLISION_ESCAPE_HPP_
