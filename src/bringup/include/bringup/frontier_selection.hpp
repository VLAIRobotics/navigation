#ifndef BRINGUP__FRONTIER_SELECTION_HPP_
#define BRINGUP__FRONTIER_SELECTION_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <queue>
#include <utility>
#include <vector>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "bringup/path_collision_escape.hpp"

namespace bringup
{

struct FrontierGoal
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  std::size_t frontier_cells{0};
};

struct FrontierFootprintPoint
{
  double x{0.0};
  double y{0.0};
};

namespace detail
{

using GridCell = std::pair<std::int64_t, std::int64_t>;

inline tf2::Transform mapFromGrid(const nav_msgs::msg::OccupancyGrid & grid)
{
  tf2::Transform transform;
  tf2::fromMsg(grid.info.origin, transform);
  return transform;
}

inline tf2::Vector3 gridCellCenter(
  const nav_msgs::msg::OccupancyGrid & grid, std::int64_t x, std::int64_t y)
{
  return mapFromGrid(grid) * tf2::Vector3(
    (static_cast<double>(x) + 0.5) * grid.info.resolution,
    (static_cast<double>(y) + 0.5) * grid.info.resolution, 0.0);
}

inline std::optional<GridCell> gridCellAtWorld(
  const nav_msgs::msg::OccupancyGrid & grid, double world_x, double world_y)
{
  tf2::Transform grid_from_world = mapFromGrid(grid).inverse();
  const auto point = grid_from_world * tf2::Vector3(world_x, world_y, 0.0);
  const double x = point.x() / grid.info.resolution;
  const double y = point.y() / grid.info.resolution;
  if (!std::isfinite(x) || !std::isfinite(y) ||
    x < static_cast<double>(std::numeric_limits<std::int64_t>::min()) ||
    x > static_cast<double>(std::numeric_limits<std::int64_t>::max()) ||
    y < static_cast<double>(std::numeric_limits<std::int64_t>::min()) ||
    y > static_cast<double>(std::numeric_limits<std::int64_t>::max()))
  {
    return std::nullopt;
  }
  return GridCell(
    static_cast<std::int64_t>(std::floor(x)),
    static_cast<std::int64_t>(std::floor(y)));
}

inline bool inside(
  const nav_msgs::msg::OccupancyGrid & grid, const GridCell & cell)
{
  return cell.first >= 0 && cell.second >= 0 &&
         cell.first < grid.info.width && cell.second < grid.info.height;
}

inline bool growthAllowedAt(
  const nav_msgs::msg::OccupancyGrid & prior_map,
  double world_x, double world_y, double max_extension_m)
{
  const auto prior_cell = gridCellAtWorld(prior_map, world_x, world_y);
  if (!prior_cell) return false;
  const auto extension = static_cast<std::int64_t>(
    std::floor(max_extension_m / prior_map.info.resolution));
  return prior_cell->first >= -extension && prior_cell->second >= -extension &&
         prior_cell->first < static_cast<std::int64_t>(prior_map.info.width) + extension &&
         prior_cell->second < static_cast<std::int64_t>(prior_map.info.height) + extension;
}

inline bool isMapFrontier(
  const nav_msgs::msg::OccupancyGrid & map,
  const nav_msgs::msg::OccupancyGrid & prior_map,
  std::int64_t x, std::int64_t y, double max_extension_m)
{
  if (!inside(map, {x, y})) return false;
  const auto current_index = static_cast<std::size_t>(y) * map.info.width +
    static_cast<std::size_t>(x);
  const auto current_value = map.data[current_index];
  if (current_value < 0 || current_value > 50) return false;
  for (int dy = -1; dy <= 1; ++dy) {
    for (int dx = -1; dx <= 1; ++dx) {
      if (dx == 0 && dy == 0) continue;
      const GridCell neighbor{x + dx, y + dy};
      if (inside(map, neighbor)) {
        const auto index = static_cast<std::size_t>(neighbor.second) * map.info.width +
          static_cast<std::size_t>(neighbor.first);
        if (map.data[index] < 0) return true;
        continue;
      }
      const auto world = gridCellCenter(map, neighbor.first, neighbor.second);
      if (growthAllowedAt(prior_map, world.x(), world.y(), max_extension_m)) return true;
    }
  }
  return false;
}

inline bool isTraversableCost(std::int8_t cost)
{
  // OccupancyGrid represents Nav2's inscribed and lethal costs as 99 and 100.
  return cost >= 0 && cost < 99;
}

inline std::optional<std::vector<std::size_t>> footprintCellsAtPose(
  const nav_msgs::msg::OccupancyGrid & grid,
  double world_x, double world_y, double yaw,
  const std::vector<FrontierFootprintPoint> & footprint,
  const double expansion_margin_meters = 0.0)
{
  if (!std::isfinite(expansion_margin_meters) || expansion_margin_meters < 0.0) {
    return std::nullopt;
  }
  const auto center = gridCellAtWorld(grid, world_x, world_y);
  if (!center || !inside(grid, *center)) return std::nullopt;
  if (footprint.empty()) {
    return std::vector<std::size_t>{
      static_cast<std::size_t>(center->second) * grid.info.width +
      static_cast<std::size_t>(center->first)};
  }
  if (footprint.size() < 3 || !std::isfinite(yaw)) return std::nullopt;

  const auto grid_from_world = mapFromGrid(grid).inverse();
  const double cosine = std::cos(yaw);
  const double sine = std::sin(yaw);
  const double margin_cells = expansion_margin_meters / grid.info.resolution;
  if (!std::isfinite(margin_cells)) return std::nullopt;
  std::vector<path_collision::Point2D> polygon;
  polygon.reserve(footprint.size());
  for (const auto & point : footprint) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y)) return std::nullopt;
    const double x = world_x + cosine * point.x - sine * point.y;
    const double y = world_y + sine * point.x + cosine * point.y;
    const auto local = grid_from_world * tf2::Vector3(x, y, 0.0);
    polygon.push_back({local.x() / grid.info.resolution, local.y() / grid.info.resolution});
  }

  double min_x = polygon.front().x;
  double max_x = min_x;
  double min_y = polygon.front().y;
  double max_y = min_y;
  for (const auto & point : polygon) {
    min_x = std::min(min_x, point.x);
    max_x = std::max(max_x, point.x);
    min_y = std::min(min_y, point.y);
    max_y = std::max(max_y, point.y);
  }
  if (min_x - margin_cells < 0.0 || min_y - margin_cells < 0.0 ||
    max_x + margin_cells > grid.info.width ||
    max_y + margin_cells > grid.info.height)
  {
    return std::nullopt;
  }

  const auto first_x = static_cast<std::int64_t>(std::max(
      0.0, std::floor(min_x - margin_cells) - 1.0));
  const auto last_x = static_cast<std::int64_t>(std::min(
      static_cast<double>(grid.info.width - 1), std::floor(max_x + margin_cells)));
  const auto first_y = static_cast<std::int64_t>(std::max(
      0.0, std::floor(min_y - margin_cells) - 1.0));
  const auto last_y = static_cast<std::int64_t>(std::min(
      static_cast<double>(grid.info.height - 1), std::floor(max_y + margin_cells)));
  if (last_x < first_x || last_y < first_y) return std::nullopt;

  std::vector<std::size_t> cells;
  for (auto y = first_y; y <= last_y; ++y) {
    for (auto x = first_x; x <= last_x; ++x) {
      if (path_collision::polygonIntersectsCell(polygon, x, y, margin_cells)) {
        cells.push_back(static_cast<std::size_t>(y) * grid.info.width +
          static_cast<std::size_t>(x));
      }
    }
  }
  if (cells.empty()) return std::nullopt;
  return cells;
}

inline bool footprintIsTraversable(
  const nav_msgs::msg::OccupancyGrid & map,
  const nav_msgs::msg::OccupancyGrid & global_costmap,
  double world_x, double world_y, double yaw,
  const std::vector<FrontierFootprintPoint> & footprint)
{
  const auto map_cells = footprintCellsAtPose(map, world_x, world_y, yaw, footprint);
  const auto costmap_cells = footprintCellsAtPose(
    global_costmap, world_x, world_y, yaw, footprint);
  if (!map_cells || !costmap_cells) return false;
  return std::all_of(map_cells->begin(), map_cells->end(), [&map](std::size_t index) {
      return map.data[index] >= 0 && map.data[index] <= 50;
    }) && std::all_of(
    costmap_cells->begin(), costmap_cells->end(), [&global_costmap](std::size_t index) {
      return isTraversableCost(global_costmap.data[index]);
    });
}

}  // namespace detail

inline std::optional<FrontierGoal> selectFrontierGoal(
  const nav_msgs::msg::OccupancyGrid & map,
  const nav_msgs::msg::OccupancyGrid & global_costmap,
  const nav_msgs::msg::OccupancyGrid & prior_map,
  double robot_x, double robot_y,
  std::size_t min_frontier_cells,
  double max_map_extension_m,
  const std::vector<std::pair<double, double>> & excluded_goals = {},
  double exclusion_radius = 0.6,
  const std::vector<FrontierFootprintPoint> & footprint = {})
{
  const auto valid_grid = [](const nav_msgs::msg::OccupancyGrid & grid) {
      return grid.info.width > 0 && grid.info.height > 0 &&
             std::isfinite(grid.info.resolution) && grid.info.resolution > 0.0F &&
             grid.data.size() == static_cast<std::size_t>(grid.info.width) * grid.info.height;
    };
  if (!valid_grid(map) || !valid_grid(global_costmap) || !valid_grid(prior_map) ||
    map.header.frame_id != global_costmap.header.frame_id ||
    map.header.frame_id != prior_map.header.frame_id ||
    std::abs(map.info.resolution - prior_map.info.resolution) > 1e-5 ||
    min_frontier_cells == 0 || !std::isfinite(robot_x) || !std::isfinite(robot_y) ||
    !std::isfinite(max_map_extension_m) || max_map_extension_m < 0.0 ||
    !std::isfinite(exclusion_radius) || exclusion_radius < 0.0)
  {
    return std::nullopt;
  }

  const auto costmap_size = static_cast<std::size_t>(global_costmap.info.width) *
    global_costmap.info.height;
  std::vector<bool> traversable_costmap_cells(costmap_size, false);
  for (std::uint32_t y = 0; y < global_costmap.info.height; ++y) {
    for (std::uint32_t x = 0; x < global_costmap.info.width; ++x) {
      const auto index = static_cast<std::size_t>(y) * global_costmap.info.width + x;
      if (!detail::isTraversableCost(global_costmap.data[index])) continue;
      const auto world = detail::gridCellCenter(global_costmap, x, y);
      const auto map_cell = detail::gridCellAtWorld(map, world.x(), world.y());
      if (!map_cell || !detail::inside(map, *map_cell)) continue;
      const auto map_index = static_cast<std::size_t>(map_cell->second) * map.info.width +
        static_cast<std::size_t>(map_cell->first);
      traversable_costmap_cells[index] = map.data[map_index] >= 0 && map.data[map_index] <= 50;
    }
  }
  const auto traversable = [&traversable_costmap_cells](std::size_t index) {
      return traversable_costmap_cells[index];
    };
  const auto robot_cell = detail::gridCellAtWorld(global_costmap, robot_x, robot_y);
  if (!robot_cell || !detail::inside(global_costmap, *robot_cell)) return std::nullopt;
  const auto flat_cost = [&global_costmap](const detail::GridCell & cell) {
      return static_cast<std::size_t>(cell.second) * global_costmap.info.width +
             static_cast<std::size_t>(cell.first);
    };
  const auto start = flat_cost(*robot_cell);
  if (!traversable(start)) return std::nullopt;

  std::vector<bool> reachable(costmap_size, false);
  std::queue<detail::GridCell> pending_costmap;
  pending_costmap.push(*robot_cell);
  reachable[start] = true;
  while (!pending_costmap.empty()) {
    const auto cell = pending_costmap.front();
    pending_costmap.pop();
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dx = -1; dx <= 1; ++dx) {
        if (dx == 0 && dy == 0) continue;
        const detail::GridCell next{cell.first + dx, cell.second + dy};
        if (!detail::inside(global_costmap, next)) continue;
        const auto next_index = flat_cost(next);
        if (reachable[next_index] || !traversable(next_index)) continue;
        if (dx != 0 && dy != 0) {
          const detail::GridCell side_x{cell.first + dx, cell.second};
          const detail::GridCell side_y{cell.first, cell.second + dy};
          if (!traversable(flat_cost(side_x)) || !traversable(flat_cost(side_y))) continue;
        }
        reachable[next_index] = true;
        pending_costmap.push(next);
      }
    }
  }

  const auto flat_map = [&map](std::uint32_t x, std::uint32_t y) {
      return static_cast<std::size_t>(y) * map.info.width + x;
    };
  std::vector<bool> frontier(static_cast<std::size_t>(map.info.width) * map.info.height, false);
  std::vector<bool> visited(frontier.size(), false);
  for (std::uint32_t y = 0; y < map.info.height; ++y) {
    for (std::uint32_t x = 0; x < map.info.width; ++x) {
      frontier[flat_map(x, y)] = detail::isMapFrontier(
        map, prior_map, x, y, max_map_extension_m);
    }
  }

  const auto map_from_grid = detail::mapFromGrid(map);
  std::optional<FrontierGoal> best;
  double best_distance_squared = std::numeric_limits<double>::infinity();
  for (std::uint32_t y = 0; y < map.info.height; ++y) {
    for (std::uint32_t x = 0; x < map.info.width; ++x) {
      const auto start_index = flat_map(x, y);
      if (!frontier[start_index] || visited[start_index]) continue;
      std::vector<detail::GridCell> component;
      std::queue<detail::GridCell> pending_frontier;
      pending_frontier.emplace(x, y);
      visited[start_index] = true;
      while (!pending_frontier.empty()) {
        const auto cell = pending_frontier.front();
        pending_frontier.pop();
        component.push_back(cell);
        for (int dy = -1; dy <= 1; ++dy) {
          for (int dx = -1; dx <= 1; ++dx) {
            if (dx == 0 && dy == 0) continue;
            const detail::GridCell next{cell.first + dx, cell.second + dy};
            if (!detail::inside(map, next)) continue;
            const auto next_index = flat_map(
              static_cast<std::uint32_t>(next.first), static_cast<std::uint32_t>(next.second));
            if (frontier[next_index] && !visited[next_index]) {
              visited[next_index] = true;
              pending_frontier.push(next);
            }
          }
        }
      }
      if (component.size() < min_frontier_cells) continue;

      double center_x = 0.0;
      double center_y = 0.0;
      for (const auto & cell : component) {
        center_x += static_cast<double>(cell.first);
        center_y += static_cast<double>(cell.second);
      }
      center_x /= component.size();
      center_y /= component.size();
      const auto cluster_center = map_from_grid * tf2::Vector3(
        (center_x + 0.5) * map.info.resolution,
        (center_y + 0.5) * map.info.resolution, 0.0);

      bool have_approach = false;
      double closest_to_center = std::numeric_limits<double>::infinity();
      tf2::Vector3 approach_world;
      double approach_yaw = 0.0;
      for (const auto & cell : component) {
        const auto world = detail::gridCellCenter(map, cell.first, cell.second);
        double unknown_dx = 0.0;
        double unknown_dy = 0.0;
        for (int neighbor_dy = -1; neighbor_dy <= 1; ++neighbor_dy) {
          for (int neighbor_dx = -1; neighbor_dx <= 1; ++neighbor_dx) {
            if (neighbor_dx == 0 && neighbor_dy == 0) continue;
            const detail::GridCell neighbor{
              cell.first + neighbor_dx, cell.second + neighbor_dy};
            if (detail::inside(map, neighbor)) {
              const auto index = flat_map(
                static_cast<std::uint32_t>(neighbor.first),
                static_cast<std::uint32_t>(neighbor.second));
              if (map.data[index] < 0) {
                unknown_dx += neighbor_dx;
                unknown_dy += neighbor_dy;
              }
            } else {
              const auto neighbor_world = detail::gridCellCenter(
                map, neighbor.first, neighbor.second);
              if (detail::growthAllowedAt(
                  prior_map, neighbor_world.x(), neighbor_world.y(), max_map_extension_m))
              {
                unknown_dx += neighbor_dx;
                unknown_dy += neighbor_dy;
              }
            }
          }
        }
        if (unknown_dx == 0.0 && unknown_dy == 0.0) continue;

        const auto world_direction = map_from_grid.getBasis() *
          tf2::Vector3(unknown_dx, unknown_dy, 0.0);
        const double direction_length = std::hypot(world_direction.x(), world_direction.y());
        if (!std::isfinite(direction_length) || direction_length <= 1.0e-9) continue;
        const double unit_x = world_direction.x() / direction_length;
        const double unit_y = world_direction.y() / direction_length;
        const double candidate_yaw = std::atan2(unit_y, unit_x);
        double front_extent = 0.0;
        for (const auto & point : footprint) {
          // candidate_yaw points the robot's local +X axis toward the frontier.
          // Project in that local frame; unit_x/unit_y are already world-frame.
          front_extent = std::max(front_extent, point.x);
        }
        const double setback = footprint.empty() ? 0.0 :
          std::max(0.0, front_extent - 0.5 * map.info.resolution) +
          global_costmap.info.resolution;
        const auto desired_x = world.x() - unit_x * setback;
        const auto desired_y = world.y() - unit_y * setback;
        const auto cost_cell = detail::gridCellAtWorld(global_costmap, desired_x, desired_y);
        if (!cost_cell || !detail::inside(global_costmap, *cost_cell)) continue;
        const auto candidate_index = flat_cost(*cost_cell);
        if (!reachable[candidate_index]) continue;
        const auto candidate_world = detail::gridCellCenter(
          global_costmap, cost_cell->first, cost_cell->second);
        const bool excluded = std::any_of(
          excluded_goals.begin(), excluded_goals.end(), [&](const auto & previous) {
            const double dx = candidate_world.x() - previous.first;
            const double dy = candidate_world.y() - previous.second;
            return dx * dx + dy * dy < exclusion_radius * exclusion_radius;
          });
        if (excluded || !detail::footprintIsTraversable(
            map, global_costmap, candidate_world.x(), candidate_world.y(),
            candidate_yaw, footprint))
        {
          continue;
        }

        const double dx = candidate_world.x() - cluster_center.x();
        const double dy = candidate_world.y() - cluster_center.y();
        const double distance = dx * dx + dy * dy;
        if (distance < closest_to_center) {
          closest_to_center = distance;
          have_approach = true;
          approach_world = candidate_world;
          approach_yaw = candidate_yaw;
        }
      }
      if (!have_approach) continue;

      const double robot_dx = approach_world.x() - robot_x;
      const double robot_dy = approach_world.y() - robot_y;
      const double distance_squared = robot_dx * robot_dx + robot_dy * robot_dy;
      if (distance_squared >= best_distance_squared) continue;
      best_distance_squared = distance_squared;
      best = FrontierGoal{
        approach_world.x(), approach_world.y(), approach_yaw, component.size()};
    }
  }
  return best;
}

// Retain the map-only helper for callers that only need geometric frontier selection.
inline std::optional<FrontierGoal> selectFrontierGoal(
  const nav_msgs::msg::OccupancyGrid & map, double robot_x, double robot_y,
  std::size_t min_frontier_cells,
  const std::vector<std::pair<double, double>> & excluded_goals = {},
  double exclusion_radius = 0.6)
{
  auto open_costmap = map;
  for (auto & cell : open_costmap.data) {
    if (cell < 0) cell = 0;
  }
  return selectFrontierGoal(
    map, open_costmap, map, robot_x, robot_y, min_frontier_cells, 0.0,
    excluded_goals, exclusion_radius, {});
}

}  // namespace bringup

#endif  // BRINGUP__FRONTIER_SELECTION_HPP_
