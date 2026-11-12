#ifndef BRINGUP__ONLINE_GOAL_ROUTE_HPP_
#define BRINGUP__ONLINE_GOAL_ROUTE_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <queue>
#include <utility>
#include <vector>

#include <nav_msgs/msg/occupancy_grid.hpp>

#include "bringup/frontier_selection.hpp"

namespace bringup
{
namespace online_goal_route
{

struct Route
{
  std::vector<std::pair<double, double>> points;
  double length{0.0};
};

struct FrontierRoute
{
  Route route;
  FrontierGoal frontier;
  double score{0.0};
};

struct DispatchTarget
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  bool is_final{false};
};

inline std::optional<DispatchTarget> validatedDispatchTarget(
  const nav_msgs::msg::OccupancyGrid & map,
  const nav_msgs::msg::OccupancyGrid & global_costmap,
  const double waypoint_x, const double waypoint_y, const double waypoint_yaw,
  const double final_x, const double final_y, const double final_yaw,
  const bool dispatch_final,
  const std::vector<FrontierFootprintPoint> & footprint)
{
  const DispatchTarget target{
    dispatch_final ? final_x : waypoint_x,
    dispatch_final ? final_y : waypoint_y,
    dispatch_final ? final_yaw : waypoint_yaw,
    dispatch_final};
  if (!detail::footprintIsTraversable(
      map, global_costmap, target.x, target.y, target.yaw, footprint))
  {
    return std::nullopt;
  }
  return target;
}

inline bool validGrid(const nav_msgs::msg::OccupancyGrid & grid)
{
  return grid.info.width > 0 && grid.info.height > 0 &&
         std::isfinite(grid.info.resolution) && grid.info.resolution > 0.0F &&
         grid.data.size() == static_cast<std::size_t>(grid.info.width) * grid.info.height;
}

inline bool knownFree(const nav_msgs::msg::OccupancyGrid & grid, std::size_t index)
{
  return index < grid.data.size() && grid.data[index] >= 0 && grid.data[index] <= 50;
}

enum class FootprintBlocker
{
  NONE,
  INVALID_GRID,
  INVALID_ARGUMENT,
  OUT_OF_BOUNDS,
  UNKNOWN,
  OCCUPIED
};

struct FootprintAssessment
{
  bool clear{false};
  FootprintBlocker blocker{FootprintBlocker::INVALID_GRID};
  std::int64_t cell_x{-1};
  std::int64_t cell_y{-1};
  std::int8_t value{-2};
};

struct StartEscapeMask
{
  bool enabled{false};
  bool valid{true};
  double start_cell_x{0.0};
  double start_cell_y{0.0};
  double resolution{0.0};
  double radius_squared{0.0};
};

inline StartEscapeMask makeStartEscapeMask(
  const nav_msgs::msg::OccupancyGrid & grid, const double start_x,
  const double start_y, const double escape_radius)
{
  StartEscapeMask mask;
  if (!std::isfinite(escape_radius) || escape_radius < 0.0 ||
    (escape_radius > 0.0 && (!std::isfinite(start_x) || !std::isfinite(start_y))))
  {
    mask.valid = false;
    return mask;
  }
  if (escape_radius == 0.0) return mask;
  if (!validGrid(grid)) {
    mask.valid = false;
    return mask;
  }
  const auto grid_from_world = detail::mapFromGrid(grid).inverse();
  const auto start_local = grid_from_world * tf2::Vector3(start_x, start_y, 0.0);
  mask.enabled = true;
  mask.start_cell_x = start_local.x() / grid.info.resolution;
  mask.start_cell_y = start_local.y() / grid.info.resolution;
  mask.resolution = grid.info.resolution;
  mask.radius_squared = escape_radius * escape_radius;
  mask.valid = std::isfinite(mask.start_cell_x) && std::isfinite(mask.start_cell_y);
  return mask;
}

inline bool cellInsideStartEscape(
  const StartEscapeMask & mask, const std::int64_t cell_x, const std::int64_t cell_y)
{
  if (!mask.enabled || !mask.valid) return false;
  const double dx = std::max({0.0, static_cast<double>(cell_x) - mask.start_cell_x,
    mask.start_cell_x - static_cast<double>(cell_x + 1)}) * mask.resolution;
  const double dy = std::max({0.0, static_cast<double>(cell_y) - mask.start_cell_y,
    mask.start_cell_y - static_cast<double>(cell_y + 1)}) * mask.resolution;
  return dx * dx + dy * dy <= mask.radius_squared + 1.0e-12;
}

inline bool cellInsideStartEscape(
  const nav_msgs::msg::OccupancyGrid & grid, const std::int64_t cell_x,
  const std::int64_t cell_y, const double start_x, const double start_y,
  const double escape_radius)
{
  return cellInsideStartEscape(
    makeStartEscapeMask(grid, start_x, start_y, escape_radius), cell_x, cell_y);
}

inline bool footprintTouchesUnknown(
  const nav_msgs::msg::OccupancyGrid & grid, const double world_x,
  const double world_y, const double yaw,
  const std::vector<FrontierFootprintPoint> & footprint)
{
  if (!validGrid(grid)) return false;
  const auto center = detail::gridCellAtWorld(grid, world_x, world_y);
  if (center && detail::inside(grid, *center) &&
    grid.data[static_cast<std::size_t>(center->second) * grid.info.width +
    static_cast<std::size_t>(center->first)] < 0)
  {
    return true;
  }
  const auto cells = detail::footprintCellsAtPose(grid, world_x, world_y, yaw, footprint);
  return cells && std::any_of(cells->begin(), cells->end(), [&grid](const std::size_t index) {
      return grid.data[index] < 0;
    });
}

inline FootprintAssessment assessCircularFootprintWithMask(
  const nav_msgs::msg::OccupancyGrid & grid, const std::int64_t x,
  const std::int64_t y, const double radius, const StartEscapeMask & start_escape)
{
  if (!validGrid(grid)) return {false, FootprintBlocker::INVALID_GRID};
  if (!std::isfinite(radius) || radius < 0.0 || !start_escape.valid)
  {
    return {false, FootprintBlocker::INVALID_ARGUMENT};
  }
  if (x < 0 || y < 0 || x >= grid.info.width || y >= grid.info.height) {
    return {false, FootprintBlocker::OUT_OF_BOUNDS, x, y, -2};
  }
  const auto cells = static_cast<std::int64_t>(std::ceil(
    (radius + 0.5 * std::sqrt(2.0) * grid.info.resolution) / grid.info.resolution));
  for (auto dy = -cells; dy <= cells; ++dy) {
    for (auto dx = -cells; dx <= cells; ++dx) {
      const double center_distance = std::hypot(
        static_cast<double>(dx) * grid.info.resolution,
        static_cast<double>(dy) * grid.info.resolution);
      if (center_distance > radius + 0.5 * std::sqrt(2.0) * grid.info.resolution) continue;
      const auto cell_x = x + dx;
      const auto cell_y = y + dy;
      if (cell_x < 0 || cell_y < 0 || cell_x >= grid.info.width ||
        cell_y >= grid.info.height)
      {
        return {false, FootprintBlocker::OUT_OF_BOUNDS, cell_x, cell_y, -2};
      }
      const auto index = static_cast<std::size_t>(cell_y) * grid.info.width +
        static_cast<std::size_t>(cell_x);
      const auto value = grid.data[index];
      if (value >= 0 && value <= 50) continue;
      if (value < 0 && cellInsideStartEscape(start_escape, cell_x, cell_y))
      {
        continue;
      }
      return {false, value < 0 ? FootprintBlocker::UNKNOWN : FootprintBlocker::OCCUPIED,
        cell_x, cell_y, value};
    }
  }
  return {true, FootprintBlocker::NONE};
}

inline FootprintAssessment assessCircularOccupiedClearance(
  const nav_msgs::msg::OccupancyGrid & grid, const std::int64_t x,
  const std::int64_t y, const double radius, const double pose_x,
  const double pose_y)
{
  if (!validGrid(grid)) return {false, FootprintBlocker::INVALID_GRID};
  if (!std::isfinite(radius) || radius < 0.0 || !std::isfinite(pose_x) ||
    !std::isfinite(pose_y))
  {
    return {false, FootprintBlocker::INVALID_ARGUMENT};
  }
  if (x < 0 || y < 0 || x >= grid.info.width || y >= grid.info.height) {
    return {false, FootprintBlocker::OUT_OF_BOUNDS, x, y, -2};
  }

  const double cell_radius = std::ceil(radius / grid.info.resolution) + 1.0;
  const auto search_cells = static_cast<std::int64_t>(cell_radius);
  const auto grid_from_world = detail::mapFromGrid(grid).inverse();
  const auto pose_grid = grid_from_world * tf2::Vector3(pose_x, pose_y, 0.0);
  const double pose_cell_x = pose_grid.x() / grid.info.resolution;
  const double pose_cell_y = pose_grid.y() / grid.info.resolution;
  for (auto cell_y = y - search_cells; cell_y <= y + search_cells; ++cell_y) {
    for (auto cell_x = x - search_cells; cell_x <= x + search_cells; ++cell_x) {
      if (cell_x < 0 || cell_y < 0 || cell_x >= grid.info.width ||
        cell_y >= grid.info.height)
      {
        continue;
      }
      const double dx = std::max({0.0, static_cast<double>(cell_x) - pose_cell_x,
        pose_cell_x - static_cast<double>(cell_x + 1)}) * grid.info.resolution;
      const double dy = std::max({0.0, static_cast<double>(cell_y) - pose_cell_y,
        pose_cell_y - static_cast<double>(cell_y + 1)}) * grid.info.resolution;
      if (std::hypot(dx, dy) > radius + 1.0e-12) continue;
      const auto index = static_cast<std::size_t>(cell_y) * grid.info.width +
        static_cast<std::size_t>(cell_x);
      const auto value = grid.data[index];
      if (value >= 0 && value <= 50) continue;
      if (value < 0) continue;
      return {false, FootprintBlocker::OCCUPIED, cell_x, cell_y, value};
    }
  }
  return {true, FootprintBlocker::NONE};
}

inline FootprintAssessment assessFootprintCellsWithMask(
  const nav_msgs::msg::OccupancyGrid & grid, const double pose_x,
  const double pose_y, const double yaw,
  const std::vector<FrontierFootprintPoint> & footprint,
  const StartEscapeMask & start_escape)
{
  if (!validGrid(grid)) return {false, FootprintBlocker::INVALID_GRID};
  if (!std::isfinite(pose_x) || !std::isfinite(pose_y) || !std::isfinite(yaw) ||
    !start_escape.valid)
  {
    return {false, FootprintBlocker::INVALID_ARGUMENT};
  }
  const auto body_cells = detail::footprintCellsAtPose(
    grid, pose_x, pose_y, yaw, footprint);
  if (!body_cells) return {false, FootprintBlocker::OUT_OF_BOUNDS};
  for (const auto index : *body_cells) {
    const auto cell_x = static_cast<std::int64_t>(index % grid.info.width);
    const auto cell_y = static_cast<std::int64_t>(index / grid.info.width);
    const auto value = grid.data[index];
    if (value >= 0 && value <= 50) continue;
    if (value < 0 && cellInsideStartEscape(start_escape, cell_x, cell_y)) continue;
    return {false, value < 0 ? FootprintBlocker::UNKNOWN : FootprintBlocker::OCCUPIED,
      cell_x, cell_y, value};
  }
  return {true, FootprintBlocker::NONE};
}

using FootprintRotationSample = std::pair<double, double>;
using FootprintCellOffset = std::pair<std::int64_t, std::int64_t>;

inline std::optional<std::vector<FootprintRotationSample>> footprintRotationSamples(
  const nav_msgs::msg::OccupancyGrid & grid,
  const std::vector<FrontierFootprintPoint> & footprint,
  const double from_yaw, const double to_yaw)
{
  if (!validGrid(grid) || !std::isfinite(from_yaw) || !std::isfinite(to_yaw) ||
    (!footprint.empty() && footprint.size() < 3))
  {
    return std::nullopt;
  }

  double circumradius = 0.0;
  for (const auto & point : footprint) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y)) {
      return std::nullopt;
    }
    circumradius = std::max(circumradius, std::hypot(point.x, point.y));
  }
  if (!std::isfinite(circumradius)) return std::nullopt;

  constexpr double kPi = 3.14159265358979323846;
  const double yaw_difference = to_yaw - from_yaw;
  if (!std::isfinite(yaw_difference)) return std::nullopt;
  const double angle_delta = std::remainder(yaw_difference, 2.0 * kPi);
  if (!std::isfinite(angle_delta)) return std::nullopt;
  const double max_vertex_margin = 0.25 * grid.info.resolution;
  const double max_sample_delta = circumradius > 1.0e-12 ? std::min(
    kPi / 18.0,
    4.0 * std::asin(std::min(1.0, max_vertex_margin / (2.0 * circumradius)))) : kPi;
  const double safe_sample_delta = std::max(max_sample_delta, 1.0e-6);
  std::vector<FootprintRotationSample> samples;
  const auto append_sweep = [&](const double sweep_delta) {
      const double required_samples = std::ceil(
        std::abs(sweep_delta) / safe_sample_delta);
      const auto sample_count = static_cast<std::size_t>(std::clamp(
          required_samples, 1.0, 4096.0));
      const double sample_delta = sweep_delta / static_cast<double>(sample_count);
      // Inflate each sampled polygon by the maximum vertex motion to either
      // neighboring angle. This closes raster gaps between sampled poses.
      const double sample_margin =
        2.0 * circumradius * std::sin(std::abs(sample_delta) / 4.0);
      for (std::size_t sample = 0; sample <= sample_count; ++sample) {
        const double yaw = from_yaw + sample_delta * static_cast<double>(sample);
        samples.emplace_back(yaw, sample_margin);
      }
    };

  append_sweep(angle_delta);
  if (std::abs(std::abs(angle_delta) - kPi) <= 1.0e-9) {
    append_sweep(-angle_delta);
  }
  return samples;
}

inline std::optional<std::vector<FootprintCellOffset>> footprintRotationCellOffsets(
  const nav_msgs::msg::OccupancyGrid & reference_grid,
  const std::vector<FrontierFootprintPoint> & footprint,
  const double from_yaw, const double to_yaw)
{
  const auto samples = footprintRotationSamples(
    reference_grid, footprint, from_yaw, to_yaw);
  if (!samples) return std::nullopt;

  double circumradius = 0.0;
  for (const auto & point : footprint) {
    circumradius = std::max(circumradius, std::hypot(point.x, point.y));
  }
  double maximum_margin = 0.0;
  for (const auto & sample : *samples) maximum_margin = std::max(maximum_margin, sample.second);
  const double half_extent = std::ceil(
    (circumradius + maximum_margin) / reference_grid.info.resolution) + 3.0;
  if (!std::isfinite(half_extent) || half_extent >
    (static_cast<double>(std::numeric_limits<std::uint32_t>::max()) - 1.0) / 2.0)
  {
    return std::nullopt;
  }

  nav_msgs::msg::OccupancyGrid geometry_grid;
  geometry_grid.info.resolution = reference_grid.info.resolution;
  geometry_grid.info.width = static_cast<std::uint32_t>(2.0 * half_extent + 1.0);
  geometry_grid.info.height = geometry_grid.info.width;
  geometry_grid.info.origin = reference_grid.info.origin;
  geometry_grid.data.assign(
    static_cast<std::size_t>(geometry_grid.info.width) * geometry_grid.info.height, 0);
  const auto center_x = static_cast<std::int64_t>(geometry_grid.info.width / 2U);
  const auto center_y = static_cast<std::int64_t>(geometry_grid.info.height / 2U);
  const auto pose = detail::gridCellCenter(geometry_grid, center_x, center_y);
  std::vector<FootprintCellOffset> offsets;
  for (const auto & sample : *samples) {
    const auto cells = detail::footprintCellsAtPose(
      geometry_grid, pose.x(), pose.y(), sample.first, footprint, sample.second);
    if (!cells) return std::nullopt;
    for (const auto index : *cells) {
      const auto cell_x = static_cast<std::int64_t>(index % geometry_grid.info.width);
      const auto cell_y = static_cast<std::int64_t>(index / geometry_grid.info.width);
      offsets.emplace_back(cell_x - center_x, cell_y - center_y);
    }
  }
  std::sort(offsets.begin(), offsets.end());
  offsets.erase(std::unique(offsets.begin(), offsets.end()), offsets.end());
  return offsets;
}

inline std::size_t flatIndex(
  const nav_msgs::msg::OccupancyGrid & grid, std::int64_t x, std::int64_t y);

template<typename IsTraversable>
inline FootprintAssessment assessFootprintRotationSweepWithOffsets(
  const nav_msgs::msg::OccupancyGrid & grid,
  const std::int64_t center_x, const std::int64_t center_y,
  const std::vector<FootprintCellOffset> & offsets,
  const StartEscapeMask & start_escape, IsTraversable is_traversable)
{
  if (!validGrid(grid)) return {false, FootprintBlocker::INVALID_GRID};
  if (!start_escape.valid || offsets.empty()) {
    return {false, FootprintBlocker::INVALID_ARGUMENT};
  }
  for (const auto & offset : offsets) {
    const auto cell_x = center_x + offset.first;
    const auto cell_y = center_y + offset.second;
    if (cell_x < 0 || cell_y < 0 || cell_x >= grid.info.width ||
      cell_y >= grid.info.height)
    {
      return {false, FootprintBlocker::OUT_OF_BOUNDS, cell_x, cell_y, -2};
    }
    const auto index = flatIndex(grid, cell_x, cell_y);
    const auto value = grid.data[index];
    if (is_traversable(value)) continue;
    if (value < 0 && cellInsideStartEscape(start_escape, cell_x, cell_y)) continue;
    return {false, value < 0 ? FootprintBlocker::UNKNOWN : FootprintBlocker::OCCUPIED,
      cell_x, cell_y, value};
  }
  return {true, FootprintBlocker::NONE};
}

template<typename IsTraversable>
inline FootprintAssessment assessFootprintRotationSweepWithMask(
  const nav_msgs::msg::OccupancyGrid & grid, const double pose_x,
  const double pose_y, const double from_yaw, const double to_yaw,
  const std::vector<FrontierFootprintPoint> & footprint,
  const StartEscapeMask & start_escape, IsTraversable is_traversable)
{
  if (!validGrid(grid)) return {false, FootprintBlocker::INVALID_GRID};
  if (!std::isfinite(pose_x) || !std::isfinite(pose_y) || !start_escape.valid) {
    return {false, FootprintBlocker::INVALID_ARGUMENT};
  }
  const auto samples = footprintRotationSamples(grid, footprint, from_yaw, to_yaw);
  if (!samples) return {false, FootprintBlocker::INVALID_ARGUMENT};
  for (const auto & sample : *samples) {
    const auto cells = detail::footprintCellsAtPose(
      grid, pose_x, pose_y, sample.first, footprint, sample.second);
    if (!cells) return {false, FootprintBlocker::OUT_OF_BOUNDS};
    for (const auto index : *cells) {
      const auto value = grid.data[index];
      if (is_traversable(value)) continue;
      const auto cell_x = static_cast<std::int64_t>(index % grid.info.width);
      const auto cell_y = static_cast<std::int64_t>(index / grid.info.width);
      if (value < 0 && cellInsideStartEscape(start_escape, cell_x, cell_y)) continue;
      return {false, value < 0 ? FootprintBlocker::UNKNOWN : FootprintBlocker::OCCUPIED,
        cell_x, cell_y, value};
    }
  }
  return {true, FootprintBlocker::NONE};
}

inline FootprintAssessment assessFootprintRotationSweepWithStartEscape(
  const nav_msgs::msg::OccupancyGrid & map,
  const nav_msgs::msg::OccupancyGrid & global_costmap,
  const double pose_x, const double pose_y,
  const double from_yaw, const double to_yaw,
  const std::vector<FrontierFootprintPoint> & footprint,
  const double start_x, const double start_y,
  const double map_start_unknown_escape_radius,
  const double costmap_start_unknown_escape_radius)
{
  if (!validGrid(map) || !validGrid(global_costmap)) {
    return {false, FootprintBlocker::INVALID_GRID};
  }
  if (!std::isfinite(pose_x) || !std::isfinite(pose_y) ||
    !std::isfinite(from_yaw) || !std::isfinite(to_yaw) ||
    !std::isfinite(start_x) || !std::isfinite(start_y) ||
    !std::isfinite(map_start_unknown_escape_radius) ||
    map_start_unknown_escape_radius < 0.0 ||
    !std::isfinite(costmap_start_unknown_escape_radius) ||
    costmap_start_unknown_escape_radius < 0.0)
  {
    return {false, FootprintBlocker::INVALID_ARGUMENT};
  }
  const auto map_escape = makeStartEscapeMask(
    map, start_x, start_y, map_start_unknown_escape_radius);
  const auto costmap_escape = makeStartEscapeMask(
    global_costmap, start_x, start_y, costmap_start_unknown_escape_radius);
  if (!map_escape.valid || !costmap_escape.valid) {
    return {false, FootprintBlocker::INVALID_ARGUMENT};
  }

  const auto map_assessment = assessFootprintRotationSweepWithMask(
    map, pose_x, pose_y, from_yaw, to_yaw, footprint, map_escape,
    [](const std::int8_t value) { return value >= 0 && value <= 50; });
  if (!map_assessment.clear) return map_assessment;
  return assessFootprintRotationSweepWithMask(
    global_costmap, pose_x, pose_y, from_yaw, to_yaw, footprint, costmap_escape,
    [](const std::int8_t value) { return detail::isTraversableCost(value); });
}

inline FootprintAssessment assessRouteClearanceWithMask(
  const nav_msgs::msg::OccupancyGrid & grid, const std::int64_t x,
  const std::int64_t y, const double radius, const double pose_x,
  const double pose_y, const double yaw,
  const std::vector<FrontierFootprintPoint> & footprint,
  const StartEscapeMask & start_escape)
{
  const auto circular_assessment = assessCircularOccupiedClearance(
    grid, x, y, radius, pose_x, pose_y);
  if (!circular_assessment.clear) return circular_assessment;
  return assessFootprintCellsWithMask(
    grid, pose_x, pose_y, yaw, footprint, start_escape);
}

inline FootprintAssessment assessCircularFootprint(
  const nav_msgs::msg::OccupancyGrid & grid, const std::int64_t x,
  const std::int64_t y, const double radius, const double start_x = 0.0,
  const double start_y = 0.0, const double start_unknown_escape_radius = 0.0)
{
  if (!std::isfinite(start_unknown_escape_radius) || start_unknown_escape_radius < 0.0 ||
    (start_unknown_escape_radius > 0.0 &&
    (!std::isfinite(start_x) || !std::isfinite(start_y))))
  {
    return {false, FootprintBlocker::INVALID_ARGUMENT};
  }
  const auto mask = makeStartEscapeMask(
    grid, start_x, start_y, start_unknown_escape_radius);
  return assessCircularFootprintWithMask(grid, x, y, radius, mask);
}

inline FootprintAssessment assessRouteClearance(
  const nav_msgs::msg::OccupancyGrid & grid, const std::int64_t x,
  const std::int64_t y, const double radius, const double world_x,
  const double world_y, const double yaw,
  const std::vector<FrontierFootprintPoint> & footprint,
  const double start_unknown_escape_radius = 0.0)
{
  if (!std::isfinite(world_x) || !std::isfinite(world_y) ||
    !std::isfinite(start_unknown_escape_radius) || start_unknown_escape_radius < 0.0)
  {
    return {false, FootprintBlocker::INVALID_ARGUMENT};
  }
  const auto mask = makeStartEscapeMask(
    grid, world_x, world_y, start_unknown_escape_radius);
  return assessRouteClearanceWithMask(
    grid, x, y, radius, world_x, world_y, yaw, footprint, mask);
}

inline bool circularFootprintClear(
  const nav_msgs::msg::OccupancyGrid & grid, std::int64_t x, std::int64_t y,
  const double radius)
{
  return assessCircularFootprint(grid, x, y, radius).clear;
}

inline std::size_t flatIndex(
  const nav_msgs::msg::OccupancyGrid & grid, std::int64_t x, std::int64_t y)
{
  return static_cast<std::size_t>(y) * grid.info.width + static_cast<std::size_t>(x);
}

inline FootprintAssessment assessFootprintWithStartEscape(
  const nav_msgs::msg::OccupancyGrid & map,
  const nav_msgs::msg::OccupancyGrid & global_costmap,
  const double world_x, const double world_y, const double yaw,
  const std::vector<FrontierFootprintPoint> & footprint,
  const double start_x, const double start_y,
  const double map_start_unknown_escape_radius,
  const double costmap_start_unknown_escape_radius)
{
  if (!validGrid(map) || !validGrid(global_costmap)) {
    return {false, FootprintBlocker::INVALID_GRID};
  }
  if (!std::isfinite(world_x) || !std::isfinite(world_y) || !std::isfinite(yaw) ||
    !std::isfinite(start_x) || !std::isfinite(start_y) ||
    !std::isfinite(map_start_unknown_escape_radius) ||
    map_start_unknown_escape_radius < 0.0 ||
    !std::isfinite(costmap_start_unknown_escape_radius) ||
    costmap_start_unknown_escape_radius < 0.0)
  {
    return {false, FootprintBlocker::INVALID_ARGUMENT};
  }
  const auto map_escape = makeStartEscapeMask(
    map, start_x, start_y, map_start_unknown_escape_radius);
  const auto costmap_escape = makeStartEscapeMask(
    global_costmap, start_x, start_y, costmap_start_unknown_escape_radius);
  if (!map_escape.valid || !costmap_escape.valid) {
    return {false, FootprintBlocker::INVALID_ARGUMENT};
  }
  const auto map_cells = detail::footprintCellsAtPose(map, world_x, world_y, yaw, footprint);
  if (!map_cells) return {false, FootprintBlocker::OUT_OF_BOUNDS};
  for (const auto index : *map_cells) {
    const auto cell_x = static_cast<std::int64_t>(index % map.info.width);
    const auto cell_y = static_cast<std::int64_t>(index / map.info.width);
    const auto value = map.data[index];
    if (value >= 0 && value <= 50) continue;
    if (value < 0 && cellInsideStartEscape(map_escape, cell_x, cell_y))
    {
      continue;
    }
    return {false, value < 0 ? FootprintBlocker::UNKNOWN : FootprintBlocker::OCCUPIED,
      cell_x, cell_y, value};
  }

  const auto costmap_cells = detail::footprintCellsAtPose(
    global_costmap, world_x, world_y, yaw, footprint);
  if (!costmap_cells) return {false, FootprintBlocker::OUT_OF_BOUNDS};
  for (const auto index : *costmap_cells) {
    const auto cell_x = static_cast<std::int64_t>(index % global_costmap.info.width);
    const auto cell_y = static_cast<std::int64_t>(index / global_costmap.info.width);
    const auto value = global_costmap.data[index];
    if (detail::isTraversableCost(value)) continue;
    if (value < 0 && cellInsideStartEscape(costmap_escape, cell_x, cell_y))
    {
      continue;
    }
    return {false, value < 0 ? FootprintBlocker::UNKNOWN : FootprintBlocker::OCCUPIED,
      cell_x, cell_y, value};
  }
  return {true, FootprintBlocker::NONE};
}

inline FootprintAssessment assessKnownFreeFootprint(
  const nav_msgs::msg::OccupancyGrid & map, const double world_x,
  const double world_y, const double yaw,
  const std::vector<FrontierFootprintPoint> & footprint)
{
  if (!validGrid(map)) return {false, FootprintBlocker::INVALID_GRID};
  if (!std::isfinite(world_x) || !std::isfinite(world_y) || !std::isfinite(yaw)) {
    return {false, FootprintBlocker::INVALID_ARGUMENT};
  }
  const auto cells = detail::footprintCellsAtPose(map, world_x, world_y, yaw, footprint);
  if (!cells) return {false, FootprintBlocker::OUT_OF_BOUNDS};
  for (const auto index : *cells) {
    const auto value = map.data[index];
    if (value >= 0 && value <= 50) continue;
    const auto cell_x = static_cast<std::int64_t>(index % map.info.width);
    const auto cell_y = static_cast<std::int64_t>(index / map.info.width);
    return {false, value < 0 ? FootprintBlocker::UNKNOWN : FootprintBlocker::OCCUPIED,
      cell_x, cell_y, value};
  }
  return {true, FootprintBlocker::NONE};
}

inline std::pair<double, double> cellCenter(
  const nav_msgs::msg::OccupancyGrid & grid, std::int64_t x, std::int64_t y)
{
  const auto world = detail::gridCellCenter(grid, x, y);
  return {world.x(), world.y()};
}

inline std::vector<std::pair<double, double>> unknownCellsAhead(
  const nav_msgs::msg::OccupancyGrid & grid, const double observer_x,
  const double observer_y, const double observer_yaw, const double radius = 2.5,
  const double half_angle = 0.7853981633974483)
{
  std::vector<std::pair<double, double>> cells;
  if (!validGrid(grid) || !std::isfinite(observer_x) || !std::isfinite(observer_y) ||
    !std::isfinite(observer_yaw) || !std::isfinite(radius) || radius <= 0.0 ||
    !std::isfinite(half_angle) || half_angle <= 0.0 || half_angle >= 1.5707963267948966)
  {
    return cells;
  }
  const auto observer = detail::gridCellAtWorld(grid, observer_x, observer_y);
  if (!observer || !detail::inside(grid, *observer)) return cells;

  const auto search_cells = static_cast<std::int64_t>(
    std::ceil(radius / grid.info.resolution)) + 1;
  // Grid cells beyond the current map edge are unknown too; retain them so
  // later map expansion can count as useful gain at this observation pose.
  const double forward_x = std::cos(observer_yaw);
  const double forward_y = std::sin(observer_yaw);
  const double cone_slope = std::tan(half_angle);
  const auto min_x = observer->first - search_cells;
  const auto max_x = observer->first + search_cells;
  const auto min_y = observer->second - search_cells;
  const auto max_y = observer->second + search_cells;
  for (auto y = min_y; y <= max_y; ++y) {
    for (auto x = min_x; x <= max_x; ++x) {
      if (detail::inside(grid, {x, y}) && grid.data[flatIndex(grid, x, y)] >= 0) continue;
      const auto world = cellCenter(grid, x, y);
      const double dx = world.first - observer_x;
      const double dy = world.second - observer_y;
      const double forward = dx * forward_x + dy * forward_y;
      const double lateral = std::abs(-dx * forward_y + dy * forward_x);
      if (forward > 0.0 && forward <= radius && lateral <= forward * cone_slope) {
        cells.push_back(world);
      }
    }
  }
  return cells;
}

inline bool frontierObservationGained(
  const nav_msgs::msg::OccupancyGrid & grid,
  const std::vector<std::pair<double, double>> & unknown_before)
{
  if (!validGrid(grid)) return false;
  for (const auto & world : unknown_before) {
    const auto cell = detail::gridCellAtWorld(grid, world.first, world.second);
    if (cell && detail::inside(grid, *cell) &&
      grid.data[flatIndex(grid, cell->first, cell->second)] >= 0)
    {
      return true;
    }
  }
  return false;
}

struct QueueEntry
{
  double score{0.0};
  double cost{0.0};
  std::size_t index{0};
};

struct QueueEntryGreater
{
  bool operator()(const QueueEntry & lhs, const QueueEntry & rhs) const
  {
    return lhs.score > rhs.score;
  }
};

// Eight movement directions use their 3x3-grid index; the center index is the start pose.
inline constexpr std::size_t kRouteHeadingCount = 9U;
inline constexpr std::size_t kStationaryHeading = 4U;
inline constexpr std::size_t kRouteRotationPairCount =
  kRouteHeadingCount * (kRouteHeadingCount + 1U) / 2U;

inline std::size_t routeRotationPairIndex(
  std::size_t first_heading, std::size_t second_heading)
{
  if (first_heading > second_heading) std::swap(first_heading, second_heading);
  const auto preceding_pairs = first_heading *
    (first_heading == 0U ? 0U : first_heading - 1U) / 2U;
  return first_heading * kRouteHeadingCount - preceding_pairs +
         second_heading - first_heading;
}

inline std::size_t routeHeadingFromStep(const int dx, const int dy)
{
  return static_cast<std::size_t>((dy + 1) * 3 + dx + 1);
}

inline double routeHeadingYaw(
  const nav_msgs::msg::OccupancyGrid & map, const std::size_t heading)
{
  const int dx = static_cast<int>(heading % 3U) - 1;
  const int dy = static_cast<int>(heading / 3U) - 1;
  const auto world_direction = detail::mapFromGrid(map).getBasis() *
    tf2::Vector3(static_cast<double>(dx), static_cast<double>(dy), 0.0);
  return std::atan2(world_direction.y(), world_direction.x());
}

inline std::array<double, kRouteHeadingCount> routeHeadingYaws(
  const nav_msgs::msg::OccupancyGrid & map)
{
  std::array<double, kRouteHeadingCount> yaws{};
  for (std::size_t heading = 0; heading < kRouteHeadingCount; ++heading) {
    if (heading != kStationaryHeading) yaws[heading] = routeHeadingYaw(map, heading);
  }
  return yaws;
}

inline std::optional<Route> planKnownPath(
  const nav_msgs::msg::OccupancyGrid & map, const double start_x, const double start_y,
  const double goal_x, const double goal_y, const double footprint_radius,
  const std::vector<FrontierFootprintPoint> & footprint,
  const std::size_t max_expansions = 500000,
  const double start_unknown_escape_radius = 0.0,
  const double start_yaw = 0.0,
  const double goal_yaw = 0.0,
  FootprintAssessment * first_blocker = nullptr)
{
  if (!validGrid(map) || !std::isfinite(start_x) || !std::isfinite(start_y) ||
    !std::isfinite(goal_x) || !std::isfinite(goal_y) || !std::isfinite(start_yaw) ||
    !std::isfinite(goal_yaw) || max_expansions == 0 ||
    !std::isfinite(start_unknown_escape_radius) || start_unknown_escape_radius < 0.0)
  {
    return std::nullopt;
  }
  if (first_blocker) *first_blocker = {true, FootprintBlocker::NONE};
  const auto start = detail::gridCellAtWorld(map, start_x, start_y);
  const auto goal = detail::gridCellAtWorld(map, goal_x, goal_y);
  if (!start || !goal || !detail::inside(map, *start) || !detail::inside(map, *goal)) {
    return std::nullopt;
  }

  const auto cell_count = map.data.size();
  const auto start_escape = makeStartEscapeMask(
    map, start_x, start_y, start_unknown_escape_radius);
  if (!start_escape.valid) return std::nullopt;
  const auto heading_yaws = routeHeadingYaws(map);
  std::array<std::array<std::vector<FootprintCellOffset>, kRouteHeadingCount>,
    kRouteHeadingCount> rotation_offsets;
  for (std::size_t from_heading = 0; from_heading < kRouteHeadingCount; ++from_heading) {
    if (from_heading == kStationaryHeading) continue;
    for (std::size_t to_heading = from_heading + 1U;
      to_heading < kRouteHeadingCount; ++to_heading)
    {
      if (to_heading == kStationaryHeading) continue;
      const auto offsets = footprintRotationCellOffsets(
        map, footprint, heading_yaws[from_heading], heading_yaws[to_heading]);
      if (!offsets) return std::nullopt;
      rotation_offsets[from_heading][to_heading] = *offsets;
      rotation_offsets[to_heading][from_heading] = *offsets;
    }
  }
  const auto start_cell_index = flatIndex(map, start->first, start->second);
  const auto goal_cell_index = flatIndex(map, goal->first, goal->second);
  const auto record_blocker = [first_blocker](const FootprintAssessment & assessment) {
      if (first_blocker && first_blocker->blocker == FootprintBlocker::NONE &&
        !assessment.clear)
      {
        *first_blocker = assessment;
      }
    };
  const auto start_assessment = assessRouteClearanceWithMask(
    map, start->first, start->second, footprint_radius, start_x, start_y,
    start_yaw, footprint, start_escape);
  if (!start_assessment.clear) {
    record_blocker(start_assessment);
    return std::nullopt;
  }
  const auto goal_assessment = assessRouteClearanceWithMask(
    map, goal->first, goal->second, footprint_radius, goal_x, goal_y,
    goal_yaw, footprint, start_escape);
  if (!goal_assessment.clear) {
    record_blocker(goal_assessment);
    return std::nullopt;
  }

  const auto state_count = cell_count * kRouteHeadingCount;
  std::vector<double> cost_so_far(state_count, std::numeric_limits<double>::infinity());
  std::vector<std::int64_t> previous(state_count, -1);
  std::vector<std::int8_t> circular_clearance(cell_count, -1);
  // A cell can be clear for one movement direction and blocked for another.
  std::vector<std::int8_t> clearance(state_count, -1);
  std::vector<std::int8_t> rotation_clearance(
    cell_count * kRouteRotationPairCount, -1);
  const auto is_clear = [&map, &circular_clearance, &clearance, footprint_radius,
      &footprint, &start_escape, start, start_x, start_y, start_yaw,
      &heading_yaws, record_blocker](
      std::int64_t x, std::int64_t y, const std::size_t heading) {
      if (!detail::inside(map, {x, y})) return false;
      if (heading >= kRouteHeadingCount) return false;
      const auto cell_index = flatIndex(map, x, y);
      const auto cache_index = cell_index * kRouteHeadingCount + heading;
      if (clearance[cache_index] < 0) {
        const auto world = detail::gridCellCenter(map, x, y);
        const bool at_start = heading == kStationaryHeading &&
          x == start->first && y == start->second;
        const double pose_x = at_start ? start_x : world.x();
        const double pose_y = at_start ? start_y : world.y();
        const double yaw = heading == kStationaryHeading ? start_yaw : heading_yaws[heading];
        if (circular_clearance[cell_index] < 0) {
          const auto circular_assessment = assessCircularOccupiedClearance(
            map, x, y, footprint_radius, pose_x, pose_y);
          circular_clearance[cell_index] = static_cast<std::int8_t>(
            circular_assessment.clear ? 1 : 0);
          record_blocker(circular_assessment);
        }
        if (circular_clearance[cell_index] == 0) {
          clearance[cache_index] = 0;
          return false;
        }
        const auto assessment = assessFootprintCellsWithMask(
          map, pose_x, pose_y, yaw, footprint, start_escape);
        clearance[cache_index] = static_cast<std::int8_t>(assessment.clear ? 1 : 0);
        record_blocker(assessment);
      }
      return clearance[cache_index] == 1;
    };
  const auto is_rotation_clear = [&map, &rotation_clearance, &rotation_offsets, &start_escape,
      &footprint, start, start_x, start_y, start_yaw, &heading_yaws, record_blocker](
      const std::int64_t x, const std::int64_t y,
      const std::size_t from_heading, const std::size_t to_heading) {
      if (from_heading >= kRouteHeadingCount || to_heading >= kRouteHeadingCount) {
        return false;
      }
      if (from_heading == to_heading) return true;
      const auto cell_index = flatIndex(map, x, y);
      const auto cache_index = cell_index * kRouteRotationPairCount +
        routeRotationPairIndex(from_heading, to_heading);
      if (rotation_clearance[cache_index] < 0) {
        const auto world = detail::gridCellCenter(map, x, y);
        const bool at_start = from_heading == kStationaryHeading &&
          x == start->first && y == start->second;
        const double pose_x = at_start ? start_x : world.x();
        const double pose_y = at_start ? start_y : world.y();
        const double from_yaw = from_heading == kStationaryHeading ?
          start_yaw : heading_yaws[from_heading];
        const double to_yaw = to_heading == kStationaryHeading ?
          start_yaw : heading_yaws[to_heading];
        const auto assessment = at_start ? assessFootprintRotationSweepWithMask(
          map, pose_x, pose_y, from_yaw, to_yaw, footprint, start_escape,
          [](const std::int8_t value) { return value >= 0 && value <= 50; }) :
          assessFootprintRotationSweepWithOffsets(
          map, x, y, rotation_offsets[from_heading][to_heading], start_escape,
          [](const std::int8_t value) { return value >= 0 && value <= 50; });
        rotation_clearance[cache_index] = static_cast<std::int8_t>(assessment.clear ? 1 : 0);
        record_blocker(assessment);
      }
      return rotation_clearance[cache_index] == 1;
    };

  const auto start_state = start_cell_index * kRouteHeadingCount + kStationaryHeading;
  const auto start_clear = is_clear(
    start->first, start->second, kStationaryHeading);
  if (!start_clear) return std::nullopt;
  const auto heuristic = [goal](std::int64_t x, std::int64_t y) {
      return std::hypot(static_cast<double>(goal->first - x),
        static_cast<double>(goal->second - y));
    };
  std::priority_queue<QueueEntry, std::vector<QueueEntry>, QueueEntryGreater> open;
  cost_so_far[start_state] = 0.0;
  open.push({heuristic(start->first, start->second), 0.0, start_state});
  std::size_t expansions = 0;
  std::optional<std::size_t> goal_state;
  while (!open.empty() && expansions < max_expansions) {
    const auto current = open.top();
    open.pop();
    if (current.cost != cost_so_far[current.index]) continue;
    const auto cell_index = current.index / kRouteHeadingCount;
    const auto incoming_heading = current.index % kRouteHeadingCount;
    const auto x = static_cast<std::int64_t>(cell_index % map.info.width);
    const auto y = static_cast<std::int64_t>(cell_index / map.info.width);
    if (cell_index == goal_cell_index) {
      const double incoming_yaw = incoming_heading == kStationaryHeading ?
        start_yaw : heading_yaws[incoming_heading];
      const auto goal_sweep = assessFootprintRotationSweepWithMask(
        map, goal_x, goal_y, incoming_yaw, goal_yaw, footprint, start_escape,
        [](const std::int8_t value) { return value >= 0 && value <= 50; });
      if (goal_sweep.clear) {
        goal_state = current.index;
        break;
      }
      record_blocker(goal_sweep);
    }
    ++expansions;
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dx = -1; dx <= 1; ++dx) {
        if (dx == 0 && dy == 0) continue;
        const auto next_x = x + dx;
        const auto next_y = y + dy;
        const auto heading = routeHeadingFromStep(dx, dy);
        const bool current_clear = incoming_heading == kStationaryHeading ||
          is_clear(x, y, heading);
        if (!is_rotation_clear(x, y, incoming_heading, heading) ||
          !current_clear || !is_clear(next_x, next_y, heading))
        {
          continue;
        }
        if (dx != 0 && dy != 0 &&
          (!is_clear(x + dx, y, heading) || !is_clear(x, y + dy, heading)))
        {
          continue;
        }
        const auto next_index = flatIndex(map, next_x, next_y);
        const auto next_state = next_index * kRouteHeadingCount + heading;
        const double next_cost = current.cost + (dx == 0 || dy == 0 ? 1.0 : 1.41421356237);
        if (next_cost >= cost_so_far[next_state]) continue;
        cost_so_far[next_state] = next_cost;
        previous[next_state] = static_cast<std::int64_t>(current.index);
        open.push({next_cost + heuristic(next_x, next_y), next_cost, next_state});
      }
    }
  }
  if (!goal_state) return std::nullopt;

  std::vector<std::size_t> reversed;
  for (auto index = static_cast<std::int64_t>(*goal_state); index >= 0;
    index = previous[static_cast<std::size_t>(index)])
  {
    reversed.push_back(static_cast<std::size_t>(index));
    if (static_cast<std::size_t>(index) == start_state) break;
  }
  if (reversed.empty() || reversed.back() != start_state) return std::nullopt;
  Route route;
  route.points.reserve(reversed.size());
  for (auto it = reversed.rbegin(); it != reversed.rend(); ++it) {
    const auto cell_index = *it / kRouteHeadingCount;
    const auto x = static_cast<std::int64_t>(cell_index % map.info.width);
    const auto y = static_cast<std::int64_t>(cell_index / map.info.width);
    route.points.push_back(cellCenter(map, x, y));
  }
  route.length = cost_so_far[*goal_state] * map.info.resolution;
  return route;
}

inline std::optional<FrontierRoute> selectFrontierRoute(
  const nav_msgs::msg::OccupancyGrid & map, const double start_x, const double start_y,
  const double target_x, const double target_y, const double footprint_radius,
  const std::vector<FrontierFootprintPoint> & footprint,
  const std::vector<std::pair<double, double>> & excluded,
  const double exclusion_radius, const std::size_t max_expansions = 500000,
  const double start_unknown_escape_radius = 0.0,
  const double start_yaw = 0.0,
  FootprintAssessment * first_blocker = nullptr)
{
  if (!validGrid(map) || !std::isfinite(start_x) || !std::isfinite(start_y) ||
    !std::isfinite(target_x) || !std::isfinite(target_y) || !std::isfinite(start_yaw) ||
    !std::isfinite(exclusion_radius) || exclusion_radius < 0.0 || max_expansions == 0 ||
    !std::isfinite(start_unknown_escape_radius) || start_unknown_escape_radius < 0.0)
  {
    return std::nullopt;
  }
  if (first_blocker) *first_blocker = {true, FootprintBlocker::NONE};
  const auto start = detail::gridCellAtWorld(map, start_x, start_y);
  if (!start || !detail::inside(map, *start)) return std::nullopt;
  const auto start_escape = makeStartEscapeMask(
    map, start_x, start_y, start_unknown_escape_radius);
  if (!start_escape.valid) return std::nullopt;
  const auto heading_yaws = routeHeadingYaws(map);
  std::array<std::array<std::vector<FootprintCellOffset>, kRouteHeadingCount>,
    kRouteHeadingCount> rotation_offsets;
  for (std::size_t from_heading = 0; from_heading < kRouteHeadingCount; ++from_heading) {
    if (from_heading == kStationaryHeading) continue;
    for (std::size_t to_heading = from_heading + 1U;
      to_heading < kRouteHeadingCount; ++to_heading)
    {
      if (to_heading == kStationaryHeading) continue;
      const auto offsets = footprintRotationCellOffsets(
        map, footprint, heading_yaws[from_heading], heading_yaws[to_heading]);
      if (!offsets) return std::nullopt;
      rotation_offsets[from_heading][to_heading] = *offsets;
      rotation_offsets[to_heading][from_heading] = *offsets;
    }
  }
  const auto record_blocker = [first_blocker](const FootprintAssessment & assessment) {
      if (first_blocker && first_blocker->blocker == FootprintBlocker::NONE &&
        !assessment.clear)
      {
        *first_blocker = assessment;
      }
    };
  const auto cell_count = map.data.size();
  const auto state_count = cell_count * kRouteHeadingCount;
  std::vector<double> distance(state_count, std::numeric_limits<double>::infinity());
  std::vector<std::int64_t> previous(state_count, -1);
  std::vector<std::int8_t> circular_clearance(cell_count, -1);
  // Intermediate clearance is direction-specific; observation poses use their own yaw below.
  std::vector<std::int8_t> clearance(state_count, -1);
  std::vector<std::int8_t> rotation_clearance(
    cell_count * kRouteRotationPairCount, -1);
  const auto is_clear = [&map, &circular_clearance, &clearance, footprint_radius,
      &footprint, &start_escape, start, start_x, start_y, start_yaw,
      &heading_yaws, record_blocker](
      std::int64_t x, std::int64_t y, const std::size_t heading) {
      if (!detail::inside(map, {x, y})) return false;
      if (heading >= kRouteHeadingCount) return false;
      const auto cell_index = flatIndex(map, x, y);
      const auto cache_index = cell_index * kRouteHeadingCount + heading;
      if (clearance[cache_index] < 0) {
        const auto world = detail::gridCellCenter(map, x, y);
        const bool at_start = heading == kStationaryHeading &&
          x == start->first && y == start->second;
        const double pose_x = at_start ? start_x : world.x();
        const double pose_y = at_start ? start_y : world.y();
        const double yaw = heading == kStationaryHeading ? start_yaw : heading_yaws[heading];
        if (circular_clearance[cell_index] < 0) {
          const auto circular_assessment = assessCircularOccupiedClearance(
            map, x, y, footprint_radius, pose_x, pose_y);
          circular_clearance[cell_index] = static_cast<std::int8_t>(
            circular_assessment.clear ? 1 : 0);
          record_blocker(circular_assessment);
        }
        if (circular_clearance[cell_index] == 0) {
          clearance[cache_index] = 0;
          return false;
        }
        const auto assessment = assessFootprintCellsWithMask(
          map, pose_x, pose_y, yaw, footprint, start_escape);
        clearance[cache_index] = static_cast<std::int8_t>(assessment.clear ? 1 : 0);
        record_blocker(assessment);
      }
      return clearance[cache_index] == 1;
    };
  const auto is_rotation_clear = [&map, &rotation_clearance, &rotation_offsets, &start_escape,
      &footprint, start, start_x, start_y, start_yaw, &heading_yaws, record_blocker](
      const std::int64_t x, const std::int64_t y,
      const std::size_t from_heading, const std::size_t to_heading) {
      if (from_heading >= kRouteHeadingCount || to_heading >= kRouteHeadingCount) {
        return false;
      }
      if (from_heading == to_heading) return true;
      const auto cell_index = flatIndex(map, x, y);
      const auto cache_index = cell_index * kRouteRotationPairCount +
        routeRotationPairIndex(from_heading, to_heading);
      if (rotation_clearance[cache_index] < 0) {
        const auto world = detail::gridCellCenter(map, x, y);
        const bool at_start = from_heading == kStationaryHeading &&
          x == start->first && y == start->second;
        const double pose_x = at_start ? start_x : world.x();
        const double pose_y = at_start ? start_y : world.y();
        const double from_yaw = from_heading == kStationaryHeading ?
          start_yaw : heading_yaws[from_heading];
        const double to_yaw = to_heading == kStationaryHeading ?
          start_yaw : heading_yaws[to_heading];
        const auto assessment = at_start ? assessFootprintRotationSweepWithMask(
          map, pose_x, pose_y, from_yaw, to_yaw, footprint, start_escape,
          [](const std::int8_t value) { return value >= 0 && value <= 50; }) :
          assessFootprintRotationSweepWithOffsets(
          map, x, y, rotation_offsets[from_heading][to_heading], start_escape,
          [](const std::int8_t value) { return value >= 0 && value <= 50; });
        rotation_clearance[cache_index] = static_cast<std::int8_t>(assessment.clear ? 1 : 0);
        record_blocker(assessment);
      }
      return rotation_clearance[cache_index] == 1;
    };
  if (!is_clear(start->first, start->second, kStationaryHeading)) return std::nullopt;

  // Extract free/unknown boundaries first, then retreat into a known-free
  // observation pose.  A pose whose full footprint is clear cannot also have
  // unknown cells in its immediately adjacent 8-neighborhood.
  std::vector<float> observation_yaw(cell_count, 0.0F);
  std::vector<std::uint8_t> observation_frontier_cells(cell_count, 0U);
  std::vector<float> observation_quality(
    cell_count, -std::numeric_limits<float>::infinity());
  const auto map_from_grid = detail::mapFromGrid(map);
  const double desired_setback = footprint_radius + 2.0 * map.info.resolution;
  const double minimum_setback = footprint_radius + map.info.resolution;
  const double maximum_setback = footprint_radius + 4.0 * map.info.resolution;
  const auto search_cells = static_cast<std::int64_t>(std::ceil(0.35 / map.info.resolution));
  for (std::int64_t y = 0; y < map.info.height; ++y) {
    for (std::int64_t x = 0; x < map.info.width; ++x) {
      const auto frontier_index = flatIndex(map, x, y);
      if (!knownFree(map, frontier_index)) continue;
      int unknown_count = 0;
      double unknown_dx = 0.0;
      double unknown_dy = 0.0;
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
          if (dx == 0 && dy == 0) continue;
          const auto neighbor_x = x + dx;
          const auto neighbor_y = y + dy;
          if (!detail::inside(map, {neighbor_x, neighbor_y}) ||
            map.data[flatIndex(map, neighbor_x, neighbor_y)] < 0)
          {
            ++unknown_count;
            unknown_dx += dx;
            unknown_dy += dy;
          }
        }
      }
      if (unknown_count == 0 || std::hypot(unknown_dx, unknown_dy) <= 1.0e-9) continue;

      const auto frontier_world = cellCenter(map, x, y);
      const auto world_direction = map_from_grid.getBasis() *
        tf2::Vector3(unknown_dx, unknown_dy, 0.0);
      const double direction_length = std::hypot(
        world_direction.x(), world_direction.y());
      if (!std::isfinite(direction_length) || direction_length <= 1.0e-9) continue;
      const double unit_x = world_direction.x() / direction_length;
      const double unit_y = world_direction.y() / direction_length;
      const double yaw = std::atan2(unit_y, unit_x);
      const auto desired_cell = detail::gridCellAtWorld(
        map, frontier_world.first - unit_x * desired_setback,
        frontier_world.second - unit_y * desired_setback);
      if (!desired_cell) continue;

      for (auto candidate_y = desired_cell->second - search_cells;
        candidate_y <= desired_cell->second + search_cells; ++candidate_y)
      {
        for (auto candidate_x = desired_cell->first - search_cells;
          candidate_x <= desired_cell->first + search_cells; ++candidate_x)
        {
          if (!detail::inside(map, {candidate_x, candidate_y})) continue;
          const auto candidate_world = cellCenter(map, candidate_x, candidate_y);
          const double offset_x = frontier_world.first - candidate_world.first;
          const double offset_y = frontier_world.second - candidate_world.second;
          const double setback = offset_x * unit_x + offset_y * unit_y;
          const double lateral = std::abs(offset_x * unit_y - offset_y * unit_x);
          if (setback < minimum_setback || setback > maximum_setback || lateral > 0.35) {
            continue;
          }
          const auto observation_assessment = assessKnownFreeFootprint(
            map, candidate_world.first, candidate_world.second, yaw, footprint);
          if (!observation_assessment.clear) {
            record_blocker(observation_assessment);
            continue;
          }
          const auto clearance_assessment = assessRouteClearance(
            map, candidate_x, candidate_y, footprint_radius,
            candidate_world.first, candidate_world.second, yaw, footprint);
          if (!clearance_assessment.clear) {
            record_blocker(clearance_assessment);
            continue;
          }
          const auto candidate_index = flatIndex(map, candidate_x, candidate_y);
          const float quality = static_cast<float>(
            0.12 * std::min(unknown_count, 8) -
            std::abs(setback - desired_setback) - lateral);
          if (quality <= observation_quality[candidate_index]) continue;
          observation_quality[candidate_index] = quality;
          observation_yaw[candidate_index] = static_cast<float>(yaw);
          observation_frontier_cells[candidate_index] =
            static_cast<std::uint8_t>(unknown_count);
        }
      }
    }
  }

  const auto strict_observation_escape = [start_escape]() {
      auto mask = start_escape;
      mask.enabled = false;
      return mask;
    }();
  std::vector<std::int8_t> observation_sweep_clearance(
    cell_count * kRouteHeadingCount, -1);
  const auto is_observation_rotation_clear = [
      &map, &observation_sweep_clearance, &observation_yaw,
      &observation_frontier_cells, &heading_yaws, &footprint,
      &strict_observation_escape, start_yaw, record_blocker](
      const std::int64_t x, const std::int64_t y,
      const std::size_t incoming_heading) {
      if (incoming_heading >= kRouteHeadingCount) return false;
      const auto cell_index = flatIndex(map, x, y);
      const auto cache_index = cell_index * kRouteHeadingCount + incoming_heading;
      if (observation_sweep_clearance[cache_index] < 0) {
        const auto world = cellCenter(map, x, y);
        const double from_yaw = incoming_heading == kStationaryHeading ?
          start_yaw : heading_yaws[incoming_heading];
        const double to_yaw = static_cast<double>(observation_yaw[cell_index]);
        const auto assessment = assessFootprintRotationSweepWithMask(
          map, world.first, world.second, from_yaw, to_yaw, footprint,
          strict_observation_escape,
          [](const std::int8_t value) { return value >= 0 && value <= 50; });
        observation_sweep_clearance[cache_index] = static_cast<std::int8_t>(
          assessment.clear ? 1 : 0);
        record_blocker(assessment);
      }
      return observation_sweep_clearance[cache_index] == 1;
    };

  const auto start_cell_index = flatIndex(map, start->first, start->second);
  const auto start_state = start_cell_index * kRouteHeadingCount + kStationaryHeading;
  std::priority_queue<QueueEntry, std::vector<QueueEntry>, QueueEntryGreater> open;
  distance[start_state] = 0.0;
  open.push({0.0, 0.0, start_state});
  std::optional<std::size_t> best_state;
  double best_score = std::numeric_limits<double>::infinity();
  FrontierGoal best_frontier;
  std::size_t expansions = 0;
  while (!open.empty() && expansions < max_expansions) {
    const auto current = open.top();
    open.pop();
    if (current.cost != distance[current.index]) continue;
    ++expansions;
    const auto cell_index = current.index / kRouteHeadingCount;
    const auto incoming_heading = current.index % kRouteHeadingCount;
    const auto x = static_cast<std::int64_t>(cell_index % map.info.width);
    const auto y = static_cast<std::int64_t>(cell_index / map.info.width);
    if (observation_frontier_cells[cell_index] != 0U &&
      is_observation_rotation_clear(x, y, incoming_heading))
    {
      const auto observer_world = cellCenter(map, x, y);
      const FrontierGoal observer{
        observer_world.first, observer_world.second,
        static_cast<double>(observation_yaw[cell_index]),
        observation_frontier_cells[cell_index]};
      const bool excluded_candidate = std::any_of(
        excluded.begin(), excluded.end(), [&](const auto & point) {
          return std::hypot(observer.x - point.first, observer.y - point.second) <
                 exclusion_radius;
        });
      if (!excluded_candidate) {
        const double start_distance = std::hypot(target_x - start_x, target_y - start_y);
        const double target_distance = std::hypot(
          target_x - observer.x, target_y - observer.y);
        const double score = current.cost * map.info.resolution +
          0.35 * (target_distance - start_distance) -
          0.12 * std::min(static_cast<int>(observer.frontier_cells), 8);
        if (score < best_score) {
          best_score = score;
          best_state = current.index;
          best_frontier = observer;
        }
      }
    }

    for (int dy = -1; dy <= 1; ++dy) {
      for (int dx = -1; dx <= 1; ++dx) {
        if (dx == 0 && dy == 0) continue;
        const auto next_x = x + dx;
        const auto next_y = y + dy;
        const auto heading = routeHeadingFromStep(dx, dy);
        const bool current_clear = incoming_heading == kStationaryHeading ||
          is_clear(x, y, heading);
        if (!is_rotation_clear(x, y, incoming_heading, heading) ||
          !current_clear || !is_clear(next_x, next_y, heading))
        {
          continue;
        }
        if (dx != 0 && dy != 0 &&
          (!is_clear(x + dx, y, heading) || !is_clear(x, y + dy, heading)))
        {
          continue;
        }
        const auto next_index = flatIndex(map, next_x, next_y);
        const auto next_state = next_index * kRouteHeadingCount + heading;
        const double next_cost = current.cost + (dx == 0 || dy == 0 ? 1.0 : 1.41421356237);
        if (next_cost >= distance[next_state]) continue;
        distance[next_state] = next_cost;
        previous[next_state] = static_cast<std::int64_t>(current.index);
        open.push({next_cost, next_cost, next_state});
      }
    }
  }
  if (!best_state) return std::nullopt;
  std::vector<std::size_t> reversed;
  for (auto index = static_cast<std::int64_t>(*best_state); index >= 0;
    index = previous[static_cast<std::size_t>(index)])
  {
    reversed.push_back(static_cast<std::size_t>(index));
    if (static_cast<std::size_t>(index) == start_state) break;
  }
  if (reversed.empty() || reversed.back() != start_state) return std::nullopt;
  FrontierRoute route;
  route.frontier = best_frontier;
  route.score = best_score;
  route.route.points.reserve(reversed.size());
  for (auto it = reversed.rbegin(); it != reversed.rend(); ++it) {
    const auto cell_index = *it / kRouteHeadingCount;
    route.route.points.push_back(cellCenter(map,
      static_cast<std::int64_t>(cell_index % map.info.width),
      static_cast<std::int64_t>(cell_index / map.info.width)));
  }
  route.route.length = distance[*best_state] * map.info.resolution;
  return route;
}

}  // namespace online_goal_route
}  // namespace bringup

#endif  // BRINGUP__ONLINE_GOAL_ROUTE_HPP_
