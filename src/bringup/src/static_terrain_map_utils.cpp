#include "bringup/static_terrain_map_publisher.hpp"
#include "bringup/display_cloud_color.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <utility>
#include <vector>

#include <fastdem/io/npz.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>

namespace bringup
{

namespace
{

void rebuildDisplayElevation(fastdem::ElevationMap & map)
{
  const auto source = map.get(fastdem::layer::elevation).eval();
  if (!map.exists("display_elevation")) {
    map.add("display_elevation", std::numeric_limits<float>::quiet_NaN());
  }
  auto & display_elevation = map.get("display_elevation");
  display_elevation = source;

  const auto * classes = map.exists("class") ? &map.get("class") : nullptr;
  constexpr float kMaximumNeighborSpread = 0.18F;
  constexpr std::size_t kMinimumNeighbors = 5;
  constexpr int kMinimumCardinalNeighbors = 3;
  for (Eigen::Index row = 0; row < source.rows(); ++row) {
    for (Eigen::Index col = 0; col < source.cols(); ++col) {
      if (std::isfinite(source(row, col))) continue;
      const bool known_obstacle = classes != nullptr &&
        std::isfinite((*classes)(row, col)) && (*classes)(row, col) >= 1.5F;

      // The map converter classifies an obstacle using a nearby ground estimate
      // even when the obstacle cell itself has no ground return. Reuse the
      // nearest original ground only in the display layer.
      if (known_obstacle) {
        constexpr int radius = 3;
        std::array<float, 48> nearest_heights{};
        std::size_t nearest_count = 0;
        int nearest_distance_squared = std::numeric_limits<int>::max();
        for (int dr = -radius; dr <= radius; ++dr) {
          for (int dc = -radius; dc <= radius; ++dc) {
            if (dr == 0 && dc == 0) continue;
            const auto neighbor_row = row + dr;
            const auto neighbor_col = col + dc;
            if (neighbor_row < 0 || neighbor_col < 0 ||
              neighbor_row >= source.rows() || neighbor_col >= source.cols())
            {
              continue;
            }
            const float height = source(neighbor_row, neighbor_col);
            if (!std::isfinite(height)) continue;
            const int distance_squared = dr * dr + dc * dc;
            if (distance_squared < nearest_distance_squared) {
              nearest_distance_squared = distance_squared;
              nearest_count = 0;
            }
            if (distance_squared == nearest_distance_squared) {
              nearest_heights[nearest_count++] = height;
            }
          }
        }
        if (nearest_count > 0) {
          const auto middle = nearest_heights.begin() + nearest_count / 2;
          std::nth_element(
            nearest_heights.begin(), middle, nearest_heights.begin() + nearest_count);
          display_elevation(row, col) = *middle;
        }
        continue;
      }

      constexpr int radius = 1;
      std::array<float, 48> neighbors{};
      std::size_t neighbor_count = 0;
      int cardinal_neighbors = 0;
      float minimum = std::numeric_limits<float>::infinity();
      float maximum = -std::numeric_limits<float>::infinity();
      for (int dr = -radius; dr <= radius; ++dr) {
        for (int dc = -radius; dc <= radius; ++dc) {
          if (dr == 0 && dc == 0) continue;
          const auto neighbor_row = row + dr;
          const auto neighbor_col = col + dc;
          if (neighbor_row < 0 || neighbor_col < 0 ||
            neighbor_row >= source.rows() || neighbor_col >= source.cols())
          {
            continue;
          }
          const float height = source(neighbor_row, neighbor_col);
          if (!std::isfinite(height)) continue;
          neighbors[neighbor_count++] = height;
          minimum = std::min(minimum, height);
          maximum = std::max(maximum, height);
          if (dr == 0 || dc == 0) ++cardinal_neighbors;
        }
      }
      if (neighbor_count < kMinimumNeighbors ||
        cardinal_neighbors < kMinimumCardinalNeighbors ||
        maximum - minimum > kMaximumNeighborSpread)
      {
        continue;
      }
      const auto middle = neighbors.begin() + neighbor_count / 2;
      std::nth_element(neighbors.begin(), middle, neighbors.begin() + neighbor_count);
      display_elevation(row, col) = *middle;
    }
  }

  if (classes == nullptr) return;

  // Fill small unknown components enclosed by mapped cells for visualization.
  // Keep boundary-connected unknown space open and leave larger voids untouched.
  constexpr std::size_t kMaximumEnclosedUnknownHoleCells = 64;
  constexpr int kMaximumUnknownHoleFillRadius = 3;
  const auto rows = static_cast<std::size_t>(source.rows());
  const auto cols = static_cast<std::size_t>(source.cols());
  const auto display_seeds = display_elevation.eval();
  std::vector<std::uint8_t> visited(rows * cols, 0);
  std::vector<std::size_t> component;
  for (std::size_t row = 0; row < rows; ++row) {
    for (std::size_t col = 0; col < cols; ++col) {
      const auto flat_index = row * cols + col;
      if (visited[flat_index] || std::isfinite(display_elevation(row, col)) ||
        !std::isfinite((*classes)(row, col)) || (*classes)(row, col) != 0.0F)
      {
        continue;
      }

      component.clear();
      component.push_back(flat_index);
      visited[flat_index] = 1;
      bool touches_map_boundary = false;
      for (std::size_t head = 0; head < component.size(); ++head) {
        const auto cell = component[head];
        const auto cell_row = cell / cols;
        const auto cell_col = cell % cols;
        touches_map_boundary = touches_map_boundary || cell_row == 0 || cell_col == 0 ||
          cell_row + 1 == rows || cell_col + 1 == cols;
        for (int dr = -1; dr <= 1; ++dr) {
          for (int dc = -1; dc <= 1; ++dc) {
            if (dr == 0 && dc == 0) continue;
            const auto neighbor_row = static_cast<std::ptrdiff_t>(cell_row) + dr;
            const auto neighbor_col = static_cast<std::ptrdiff_t>(cell_col) + dc;
            if (neighbor_row < 0 || neighbor_col < 0 ||
              neighbor_row >= static_cast<std::ptrdiff_t>(rows) ||
              neighbor_col >= static_cast<std::ptrdiff_t>(cols))
            {
              continue;
            }
            const auto neighbor_flat = static_cast<std::size_t>(neighbor_row) * cols +
              static_cast<std::size_t>(neighbor_col);
            if (visited[neighbor_flat] ||
              std::isfinite(display_elevation(neighbor_row, neighbor_col)) ||
              !std::isfinite((*classes)(neighbor_row, neighbor_col)) ||
              (*classes)(neighbor_row, neighbor_col) != 0.0F)
            {
              continue;
            }
            visited[neighbor_flat] = 1;
            component.push_back(neighbor_flat);
          }
        }
      }
      if (touches_map_boundary || component.size() > kMaximumEnclosedUnknownHoleCells) {
        continue;
      }

      std::vector<float> fill_heights;
      fill_heights.reserve(component.size());
      bool can_fill_component = true;
      for (const auto cell : component) {
        const auto cell_row = static_cast<Eigen::Index>(cell / cols);
        const auto cell_col = static_cast<Eigen::Index>(cell % cols);
        std::array<float, 48> nearest_heights{};
        std::size_t nearest_count = 0;
        int nearest_distance_squared = std::numeric_limits<int>::max();
        for (int dr = -kMaximumUnknownHoleFillRadius;
          dr <= kMaximumUnknownHoleFillRadius; ++dr)
        {
          for (int dc = -kMaximumUnknownHoleFillRadius;
            dc <= kMaximumUnknownHoleFillRadius; ++dc)
          {
            const auto neighbor_row = cell_row + dr;
            const auto neighbor_col = cell_col + dc;
            if (neighbor_row < 0 || neighbor_col < 0 ||
              neighbor_row >= source.rows() || neighbor_col >= source.cols())
            {
              continue;
            }
            const float height = display_seeds(neighbor_row, neighbor_col);
            if (!std::isfinite(height)) continue;
            const int distance_squared = dr * dr + dc * dc;
            if (distance_squared < nearest_distance_squared) {
              nearest_distance_squared = distance_squared;
              nearest_count = 0;
            }
            if (distance_squared == nearest_distance_squared) {
              nearest_heights[nearest_count++] = height;
            }
          }
        }
        if (nearest_count == 0) {
          can_fill_component = false;
          break;
        }
        const auto middle = nearest_heights.begin() + nearest_count / 2;
        std::nth_element(
          nearest_heights.begin(), middle, nearest_heights.begin() + nearest_count);
        fill_heights.push_back(*middle);
      }
      if (!can_fill_component) continue;
      for (std::size_t i = 0; i < component.size(); ++i) {
        display_elevation(
          static_cast<Eigen::Index>(component[i] / cols),
          static_cast<Eigen::Index>(component[i] % cols)) = fill_heights[i];
      }
    }
  }
}

void rebuildDisplayRelief(fastdem::ElevationMap & map)
{
  if (!map.exists("display_relief")) map.add("display_relief", 0.0F);
  auto & relief = map.get("display_relief");
  const auto & elevation = map.get("display_elevation");
  const auto & slope = map.get("slope");
  const auto & step = map.get("step_residual");
  const auto unit = [](float value) {
      return std::clamp(value, 0.0F, 1.0F);
    };
  relief.setZero();
  for (Eigen::Index row = 0; row < elevation.rows(); ++row) {
    for (Eigen::Index col = 0; col < elevation.cols(); ++col) {
      const float height = elevation(row, col);
      if (!std::isfinite(height)) continue;
      const float height_tone = 0.60F * unit((height + 0.20F) / 0.80F);
      const float slope_tone = std::isfinite(slope(row, col)) ?
        0.65F * std::sqrt(unit(slope(row, col) / 25.0F)) : 0.0F;
      const float step_tone = std::isfinite(step(row, col)) ?
        0.80F * unit(step(row, col) / 0.20F) : 0.0F;
      relief(row, col) = std::max({height_tone, slope_tone, step_tone});
    }
  }
}

}  // namespace

bool loadStaticTerrainMap(
  const std::string & path, const std::string & expected_frame,
  fastdem::ElevationMap & map, std::string * error)
{
  const auto fail = [error](const std::string & message) {
      if (error != nullptr) *error = message;
      return false;
    };

  if (path.empty()) return fail("terrain_map_file is empty");
  std::string load_error;
  if (!fastdem::io::loadNpz(path, map, &load_error)) {
    return fail("failed to load terrain map '" + path + "': " + load_error);
  }
  if (!map.isInitialized()) return fail("terrain map is not initialized: " + path);
  const auto size = map.getSize();
  const auto start_index = map.getStartIndex();
  if (size.x() <= 0 || size.y() <= 0 || start_index.x() < 0 || start_index.y() < 0 ||
    start_index.x() >= size.x() || start_index.y() >= size.y() ||
    !std::isfinite(map.getResolution()) || map.getResolution() <= 0.0F ||
    !std::isfinite(map.getPosition().x()) || !std::isfinite(map.getPosition().y()) ||
    !std::isfinite(map.getLength().x()) || !std::isfinite(map.getLength().y()))
  {
    return fail("terrain map has invalid dimensions, geometry, or start index: " + path);
  }
  if (map.getFrameId() != expected_frame) {
    return fail(
      "terrain map frame '" + map.getFrameId() + "' does not match expected frame '" +
      expected_frame + "'");
  }
  for (const auto * layer : std::array<const char *, 7>{
      fastdem::layer::elevation, "slope", "step_residual", "roughness", fastdem::layer::n_points,
      "class", "clearance"})
  {
    if (!map.exists(layer)) return fail("terrain map is missing layer: " + std::string(layer));
    const auto & matrix = map.get(layer);
    if (matrix.rows() != size.x() || matrix.cols() != size.y() ||
      static_cast<std::size_t>(matrix.size()) !=
      static_cast<std::size_t>(size.x()) * static_cast<std::size_t>(size.y()))
    {
      return fail("terrain map layer has invalid dimensions: " + std::string(layer));
    }
  }
  return true;
}

std::optional<float> staticDisplayHeightAboveGround(
  const fastdem::ElevationMap & map, float x, float y, float z, float fallback_ground)
{
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) return std::nullopt;
  const auto index = map.index(nanogrid::Position(x, y));
  if (!index) return std::nullopt;

  float ground = map.elevationAt(*index);
  if (!std::isfinite(ground)) {
    float nearest_distance_squared = std::numeric_limits<float>::infinity();
    const auto size = map.getSize();
    for (int dx = -3; dx <= 3; ++dx) {
      for (int dy = -3; dy <= 3; ++dy) {
        const int row = index->x() + dx;
        const int col = index->y() + dy;
        if (row < 0 || col < 0 || row >= size.x() || col >= size.y()) continue;
        const float candidate = map.elevationAt(nanogrid::Index(row, col));
        const float distance_squared = static_cast<float>(dx * dx + dy * dy);
        if (std::isfinite(candidate) && distance_squared < nearest_distance_squared) {
          ground = candidate;
          nearest_distance_squared = distance_squared;
        }
      }
    }
  }
  if (!std::isfinite(ground)) ground = fallback_ground;
  if (!std::isfinite(ground)) return std::nullopt;
  return z - ground;
}

bool keepStaticDisplayPoint(
  const fastdem::ElevationMap & map, float x, float y, float z,
  float max_height_above_ground)
{
  const auto height = staticDisplayHeightAboveGround(map, x, y, z);
  return height && *height <= max_height_above_ground;
}

std::array<std::uint8_t, 3> staticDisplayColor(float height_above_ground)
{
  return heightDisplayColor(height_above_ground);
}

std::size_t prepareStaticDisplayMap(fastdem::ElevationMap & map)
{
  const auto original = map.get(fastdem::layer::elevation).eval();
  auto & elevation = map.get(fastdem::layer::elevation);
  const auto & classes = map.get("class");
  std::size_t removed = 0;
  for (Eigen::Index row = 0; row < original.rows(); ++row) {
    for (Eigen::Index col = 0; col < original.cols(); ++col) {
      const float current = original(row, col);
      if (!std::isfinite(current) || classes(row, col) != 1.0F) continue;
      std::vector<float> neighbors;
      int nearby_support = 0;
      for (Eigen::Index dr = -1; dr <= 1; ++dr) {
        for (Eigen::Index dc = -1; dc <= 1; ++dc) {
          if (dr == 0 && dc == 0) continue;
          const auto nr = row + dr;
          const auto nc = col + dc;
          if (nr < 0 || nc < 0 || nr >= original.rows() || nc >= original.cols()) continue;
          const float neighbor = original(nr, nc);
          if (!std::isfinite(neighbor)) continue;
          neighbors.push_back(neighbor);
          if (std::abs(neighbor - current) <= 0.08F) ++nearby_support;
        }
      }
      if (neighbors.size() < 3 || nearby_support >= 2) continue;
      const auto middle = neighbors.begin() + neighbors.size() / 2;
      std::nth_element(neighbors.begin(), middle, neighbors.end());
      if (current > *middle + 0.18F) {
        elevation(row, col) = *middle;
        ++removed;
      }
    }
  }

  rebuildDisplayElevation(map);
  rebuildDisplayRelief(map);
  return removed;
}

bool extendDisplayMapToOccupancyGrid(
  fastdem::ElevationMap & terrain_map,
  const nav_msgs::msg::OccupancyGrid & occupancy_map,
  bool * changed,
  std::string * error)
{
  const auto fail = [error](const std::string & message) {
      if (error != nullptr) *error = message;
      return false;
    };
  if (changed != nullptr) *changed = false;
  if (!terrain_map.isInitialized() || occupancy_map.header.frame_id != terrain_map.getFrameId() ||
    occupancy_map.info.width == 0 || occupancy_map.info.height == 0 ||
    !std::isfinite(occupancy_map.info.resolution) || occupancy_map.info.resolution <= 0.0F ||
    occupancy_map.data.size() != static_cast<std::size_t>(occupancy_map.info.width) *
    occupancy_map.info.height ||
    std::abs(terrain_map.getResolution() - occupancy_map.info.resolution) > 1e-5)
  {
    return fail("occupancy map and terrain display frame/resolution do not match");
  }
  tf2::Quaternion orientation;
  tf2::fromMsg(occupancy_map.info.origin.orientation, orientation);
  double roll;
  double pitch;
  double yaw;
  tf2::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
  if (std::abs(roll) > 1e-5 || std::abs(pitch) > 1e-5 || std::abs(yaw) > 1e-5) {
    return fail("rotated occupancy maps cannot be aligned to the axis-aligned terrain display");
  }

  constexpr double kAlignmentToleranceMeters = 1e-5;
  constexpr double kMaxAlignmentToleranceCells = 1e-4;
  const double resolution = occupancy_map.info.resolution;
  const double alignment_tolerance_cells = std::min(
    kAlignmentToleranceMeters / resolution, kMaxAlignmentToleranceCells);
  const auto getAlignedOffsets = [&](double & offset_x, double & offset_y) {
      const double terrain_origin_x =
        terrain_map.getPosition().x() - terrain_map.getLength().x() / 2.0;
      const double terrain_origin_y =
        terrain_map.getPosition().y() - terrain_map.getLength().y() / 2.0;
      offset_x = (occupancy_map.info.origin.position.x - terrain_origin_x) / resolution;
      offset_y = (occupancy_map.info.origin.position.y - terrain_origin_y) / resolution;
      return std::isfinite(offset_x) && std::isfinite(offset_y) &&
             std::abs(offset_x - std::round(offset_x)) <= alignment_tolerance_cells &&
             std::abs(offset_y - std::round(offset_y)) <= alignment_tolerance_cells;
    };
  double offset_x = 0.0;
  double offset_y = 0.0;
  if (!getAlignedOffsets(offset_x, offset_y)) {
    return fail("occupancy and terrain grids are not aligned to the same cell centers");
  }

  const auto current_size = terrain_map.getSize();
  constexpr double kMaxGridDimension = static_cast<double>(std::numeric_limits<int>::max());
  const double rounded_offset_x = std::round(offset_x);
  const double rounded_offset_y = std::round(offset_y);
  if (rounded_offset_x < -kMaxGridDimension || rounded_offset_x > kMaxGridDimension ||
    rounded_offset_y < -kMaxGridDimension || rounded_offset_y > kMaxGridDimension)
  {
    return fail("occupancy map origin is outside the expandable terrain grid range");
  }

  const std::int64_t occupancy_start_x = static_cast<std::int64_t>(rounded_offset_x);
  const std::int64_t occupancy_start_y = static_cast<std::int64_t>(rounded_offset_y);
  const std::int64_t occupancy_end_x = occupancy_start_x + occupancy_map.info.width;
  const std::int64_t occupancy_end_y = occupancy_start_y + occupancy_map.info.height;
  const std::int64_t current_width = current_size(0);
  const std::int64_t current_height = current_size(1);
  if (occupancy_start_x >= 0 && occupancy_start_y >= 0 &&
    occupancy_end_x <= current_width && occupancy_end_y <= current_height)
  {
    return true;
  }

  const std::int64_t minimum_x = std::min<std::int64_t>(0, occupancy_start_x);
  const std::int64_t minimum_y = std::min<std::int64_t>(0, occupancy_start_y);
  const std::int64_t maximum_x = std::max(current_width, occupancy_end_x);
  const std::int64_t maximum_y = std::max(current_height, occupancy_end_y);
  const std::int64_t expanded_width = maximum_x - minimum_x;
  const std::int64_t expanded_height = maximum_y - minimum_y;
  if (expanded_width <= 0 || expanded_height <= 0 ||
    expanded_width > std::numeric_limits<int>::max() ||
    expanded_height > std::numeric_limits<int>::max())
  {
    return fail("expanded terrain grid dimensions exceed the supported range");
  }

  const double terrain_origin_x =
    terrain_map.getPosition().x() - terrain_map.getLength().x() / 2.0;
  const double terrain_origin_y =
    terrain_map.getPosition().y() - terrain_map.getLength().y() / 2.0;
  const double expanded_origin_x = terrain_origin_x + minimum_x * resolution;
  const double expanded_origin_y = terrain_origin_y + minimum_y * resolution;
  const nanogrid::Length expanded_length(
    static_cast<double>(expanded_width) * resolution,
    static_cast<double>(expanded_height) * resolution);
  const nanogrid::Position expanded_center(
    expanded_origin_x + expanded_length.x() / 2.0,
    expanded_origin_y + expanded_length.y() / 2.0);

  fastdem::ElevationMap expanded_map;
  static_cast<nanogrid::GridMap &>(expanded_map).setGeometry(
    expanded_length, resolution, expanded_center);
  expanded_map.setFrameId(terrain_map.getFrameId());
  expanded_map.setTimestamp(terrain_map.getTimestamp());
  for (const auto & layer : terrain_map.getLayers()) {
    if (!expanded_map.exists(layer)) expanded_map.add(layer, NAN);
  }

  const auto & layers = terrain_map.getLayers();
  const double cell_position_tolerance = resolution * alignment_tolerance_cells;
  for (const auto cell : terrain_map.cells()) {
    const auto old_position = terrain_map.position(cell);
    if (!old_position) return fail("failed to read an existing terrain cell position");
    const auto new_index = expanded_map.index(*old_position);
    if (!new_index) return fail("expanded terrain grid does not cover an existing terrain cell");
    const auto new_position = expanded_map.position(*new_index);
    if (!new_position ||
      std::abs(new_position->x() - old_position->x()) > cell_position_tolerance ||
      std::abs(new_position->y() - old_position->y()) > cell_position_tolerance)
    {
      return fail("expanded terrain grid would move an existing terrain cell");
    }
    for (const auto & layer : layers) {
      const float value = terrain_map.at(layer, cell);
      expanded_map.at(layer, *new_index) = value;
      const float copied_value = expanded_map.at(layer, *new_index);
      if (copied_value != value && !(std::isnan(copied_value) && std::isnan(value))) {
        return fail("failed to preserve an existing terrain layer value");
      }
    }
  }

  const auto expanded_size = expanded_map.getSize();
  const double actual_origin_x =
    expanded_map.getPosition().x() - expanded_map.getLength().x() / 2.0;
  const double actual_origin_y =
    expanded_map.getPosition().y() - expanded_map.getLength().y() / 2.0;
  const double actual_offset_x =
    (occupancy_map.info.origin.position.x - actual_origin_x) / resolution;
  const double actual_offset_y =
    (occupancy_map.info.origin.position.y - actual_origin_y) / resolution;
  if (expanded_size(0) != expanded_width || expanded_size(1) != expanded_height ||
    std::abs((actual_origin_x - expanded_origin_x) / resolution) > alignment_tolerance_cells ||
    std::abs((actual_origin_y - expanded_origin_y) / resolution) > alignment_tolerance_cells ||
    !std::isfinite(actual_offset_x) || !std::isfinite(actual_offset_y) ||
    std::abs(actual_offset_x - std::round(actual_offset_x)) > alignment_tolerance_cells ||
    std::abs(actual_offset_y - std::round(actual_offset_y)) > alignment_tolerance_cells ||
    std::round(actual_offset_x) < 0.0 || std::round(actual_offset_y) < 0.0 ||
    std::round(actual_offset_x) + occupancy_map.info.width > expanded_size(0) ||
    std::round(actual_offset_y) + occupancy_map.info.height > expanded_size(1))
  {
    return fail("expanded terrain display does not cover occupancy bounds and preserve cell alignment");
  }

  terrain_map = std::move(expanded_map);
  if (changed != nullptr) *changed = true;
  return true;
}

std::size_t mergeOnlineDisplayTerrain(
  fastdem::ElevationMap & display_map, const nanogrid::GridMap & online_map,
  const tf2::Transform & display_from_online,
  int min_observations, int max_observation_age)
{
  if (!display_map.isInitialized() || !display_map.exists("display_relief") ||
    !online_map.exists(fastdem::layer::elevation) || min_observations < 1 ||
    max_observation_age < 0)
  {
    return 0;
  }
  const bool has_observations = online_map.exists(fastdem::layer::n_points);
  const bool has_age = online_map.exists(fastdem::layer::observation_age);
  if (!has_observations || !has_age) return 0;

  const auto & online_elevation = online_map.get(fastdem::layer::elevation);
  const auto & observations = online_map.get(fastdem::layer::n_points);
  const auto & observation_age = online_map.get(fastdem::layer::observation_age);
  const auto * online_slope = online_map.exists("slope") ? &online_map.get("slope") : nullptr;
  const auto * online_step = online_map.exists("step_residual") ?
    &online_map.get("step_residual") : nullptr;
  const auto * online_roughness = online_map.exists("roughness") ?
    &online_map.get("roughness") : nullptr;
  std::size_t merged = 0;
  for (const auto cell : online_map.cells()) {
    const float elevation = online_elevation(cell.index);
    if (!std::isfinite(elevation) || !std::isfinite(observations(cell.index)) ||
      observations(cell.index) < static_cast<float>(min_observations) ||
      !std::isfinite(observation_age(cell.index)) ||
      observation_age(cell.index) > static_cast<float>(max_observation_age))
    {
      continue;
    }
    const auto position = online_map.position(cell);
    if (!position) continue;
    const auto transformed = display_from_online * tf2::Vector3(
      position->x(), position->y(), elevation);
    const float transformed_height = static_cast<float>(transformed.z());
    const auto display_index = display_map.index(
      nanogrid::Position(transformed.x(), transformed.y()));
    if (!display_index) continue;

    display_map.at(fastdem::layer::elevation, *display_index) =
      transformed_height;
    if (online_slope && std::isfinite((*online_slope)(cell.index))) {
      display_map.at("slope", *display_index) = (*online_slope)(cell.index);
    }
    if (online_step && std::isfinite((*online_step)(cell.index))) {
      display_map.at("step_residual", *display_index) = (*online_step)(cell.index);
    }
    if (online_roughness && std::isfinite((*online_roughness)(cell.index))) {
      display_map.at("roughness", *display_index) = (*online_roughness)(cell.index);
    }
    if (display_map.exists(fastdem::layer::n_points)) {
      display_map.at(fastdem::layer::n_points, *display_index) = observations(cell.index);
    }
    ++merged;
  }
  if (merged > 0) {
    rebuildDisplayElevation(display_map);
    rebuildDisplayRelief(display_map);
  }
  return merged;
}

}  // namespace bringup
