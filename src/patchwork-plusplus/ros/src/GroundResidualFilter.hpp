#ifndef PATCHWORKPP_ROS__GROUND_RESIDUAL_FILTER_HPP_
#define PATCHWORKPP_ROS__GROUND_RESIDUAL_FILTER_HPP_

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Core>

namespace patchworkpp_ros::utils
{

struct GroundResidualSplit
{
  Eigen::MatrixX3f ground;
  Eigen::MatrixX3f nonground;
  std::size_t promoted_ground_points{0};
};

// Route spatially supported small positive surfaces into the terrain stream.
// FastDEM retains their elevation; narrow or isolated returns remain available
// to obstacle processing.
inline GroundResidualSplit splitGroundResiduals(
  const Eigen::MatrixX3f & ground,
  const Eigen::MatrixX3f & nonground,
  const float xy_tolerance,
  const float z_tolerance)
{
  GroundResidualSplit result;
  result.ground = ground;
  result.nonground = nonground;
  if (xy_tolerance <= 0.0F || z_tolerance < 0.0F || ground.rows() == 0) {
    return result;
  }

  const auto cell_coordinates = [xy_tolerance](const Eigen::Vector3f & point) {
      const auto x = static_cast<std::int32_t>(std::floor(point.x() / xy_tolerance));
      const auto y = static_cast<std::int32_t>(std::floor(point.y() / xy_tolerance));
      return std::pair{x, y};
    };
  const auto cell_key = [](const std::int32_t x, const std::int32_t y) {
      const auto high = static_cast<std::uint64_t>(static_cast<std::uint32_t>(x)) << 32;
      return static_cast<std::int64_t>(high | static_cast<std::uint32_t>(y));
    };

  std::unordered_map<std::int64_t, std::vector<Eigen::Vector3f>> ground_cells;
  ground_cells.reserve(static_cast<std::size_t>(ground.rows()));
  for (Eigen::Index i = 0; i < ground.rows(); ++i) {
    const Eigen::Vector3f point = ground.row(i).transpose();
    if (!point.allFinite()) {
      continue;
    }
    const auto [x, y] = cell_coordinates(point);
    ground_cells[cell_key(x, y)].push_back(point);
  }

  const float xy_tolerance_squared = xy_tolerance * xy_tolerance;
  std::vector<Eigen::Vector3f> positive_residuals;
  std::vector<Eigen::Vector3f> retained;
  positive_residuals.reserve(static_cast<std::size_t>(nonground.rows()));
  retained.reserve(static_cast<std::size_t>(nonground.rows()));
  for (Eigen::Index i = 0; i < nonground.rows(); ++i) {
    const Eigen::Vector3f point = nonground.row(i).transpose();
    if (!point.allFinite()) {
      continue;
    }
    const auto [cell_x, cell_y] = cell_coordinates(point);
    float best_xy_distance = std::numeric_limits<float>::infinity();
    float best_height_delta = std::numeric_limits<float>::infinity();
    float best_height_error = std::numeric_limits<float>::infinity();
    bool has_matching_below_ground_sample = false;
    for (std::int32_t offset_x = -1; offset_x <= 1; ++offset_x) {
      for (std::int32_t offset_y = -1; offset_y <= 1; ++offset_y) {
        const auto found = ground_cells.find(cell_key(cell_x + offset_x, cell_y + offset_y));
        if (found == ground_cells.end()) {
          continue;
        }
        for (const auto & ground_point : found->second) {
          const float dx = point.x() - ground_point.x();
          const float dy = point.y() - ground_point.y();
          const float xy_distance = dx * dx + dy * dy;
          const float height_delta = point.z() - ground_point.z();
          const float height_error = std::abs(height_delta);
          if (xy_distance <= xy_tolerance_squared && height_delta <= 0.0F &&
            height_delta >= -z_tolerance)
          {
            has_matching_below_ground_sample = true;
          }
          const bool closer_ground = xy_distance + 1.0e-6F < best_xy_distance;
          const bool same_xy_support = std::abs(xy_distance - best_xy_distance) <= 1.0e-6F;
          if (xy_distance <= xy_tolerance_squared &&
            (closer_ground || (same_xy_support && height_error < best_height_error)))
          {
            best_xy_distance = xy_distance;
            best_height_delta = height_delta;
            best_height_error = height_error;
          }
        }
      }
    }
    if (best_height_delta <= 0.0F && has_matching_below_ground_sample) {
      continue;
    }
    if (best_height_delta > 0.0F && best_height_delta < z_tolerance) {
      positive_residuals.push_back(point);
    } else if (!std::isfinite(best_height_error) || best_height_delta > 0.0F ||
      best_height_delta < -z_tolerance)
    {
      retained.push_back(point);
    }
  }

  std::unordered_map<std::int64_t, std::vector<std::size_t>> residual_cells;
  residual_cells.reserve(positive_residuals.size());
  for (std::size_t i = 0; i < positive_residuals.size(); ++i) {
    const auto [x, y] = cell_coordinates(positive_residuals[i]);
    residual_cells[cell_key(x, y)].push_back(i);
  }

  std::vector<Eigen::Vector3f> promoted;
  promoted.reserve(positive_residuals.size());
  constexpr std::size_t kMinSurfaceSupport = 3;
  const float min_surface_variance = xy_tolerance_squared * 0.01F;
  for (std::size_t i = 0; i < positive_residuals.size(); ++i) {
    const auto & point = positive_residuals[i];
    const auto [cell_x, cell_y] = cell_coordinates(point);
    std::size_t support = 0;
    double sum_x = 0.0;
    double sum_y = 0.0;
    double sum_xx = 0.0;
    double sum_yy = 0.0;
    double sum_xy = 0.0;
    for (std::int32_t offset_x = -1; offset_x <= 1; ++offset_x) {
      for (std::int32_t offset_y = -1; offset_y <= 1; ++offset_y) {
        const auto found = residual_cells.find(cell_key(cell_x + offset_x, cell_y + offset_y));
        if (found == residual_cells.end()) {
          continue;
        }
        for (const auto index : found->second) {
          const auto & neighbor = positive_residuals[index];
          const double dx = neighbor.x() - point.x();
          const double dy = neighbor.y() - point.y();
          if (dx * dx + dy * dy > xy_tolerance_squared ||
            std::abs(neighbor.z() - point.z()) >= z_tolerance)
          {
            continue;
          }
          ++support;
          sum_x += dx;
          sum_y += dy;
          sum_xx += dx * dx;
          sum_yy += dy * dy;
          sum_xy += dx * dy;
        }
      }
    }
    if (support < kMinSurfaceSupport) {
      retained.push_back(point);
      continue;
    }
    const double inverse_support = 1.0 / static_cast<double>(support);
    const double variance_x = sum_xx * inverse_support -
      sum_x * sum_x * inverse_support * inverse_support;
    const double variance_y = sum_yy * inverse_support -
      sum_y * sum_y * inverse_support * inverse_support;
    const double covariance_xy = sum_xy * inverse_support -
      sum_x * sum_y * inverse_support * inverse_support;
    const double trace = variance_x + variance_y;
    const double discriminant = std::hypot(variance_x - variance_y, 2.0 * covariance_xy);
    const double min_variance = 0.5 * (trace - discriminant);
    if (min_variance > min_surface_variance) {
      promoted.push_back(point);
    } else {
      retained.push_back(point);
    }
  }

  result.ground.resize(ground.rows() + static_cast<Eigen::Index>(promoted.size()), 3);
  result.ground.topRows(ground.rows()) = ground;
  result.nonground.resize(static_cast<Eigen::Index>(retained.size()), 3);
  for (std::size_t i = 0; i < promoted.size(); ++i) {
    result.ground.row(ground.rows() + static_cast<Eigen::Index>(i)) = promoted[i].transpose();
  }
  for (std::size_t i = 0; i < retained.size(); ++i) {
    result.nonground.row(static_cast<Eigen::Index>(i)) = retained[i].transpose();
  }
  result.promoted_ground_points = promoted.size();
  return result;
}

// Removes only nonground points at or below a nearby measured ground sample. Positive
// height above the local ground is preserved so low obstacles are not treated as noise.
inline Eigen::MatrixX3f filterGroundResiduals(
  const Eigen::MatrixX3f & ground,
  const Eigen::MatrixX3f & nonground,
  const float xy_tolerance,
  const float z_tolerance)
{
  if (xy_tolerance <= 0.0F || z_tolerance < 0.0F || ground.rows() == 0) {
    return nonground;
  }

  const auto cell_coordinates = [xy_tolerance](const Eigen::Vector3f & point) {
      const auto x = static_cast<std::int32_t>(std::floor(point.x() / xy_tolerance));
      const auto y = static_cast<std::int32_t>(std::floor(point.y() / xy_tolerance));
      return std::pair{x, y};
    };
  const auto cell_key = [](const std::int32_t x, const std::int32_t y) {
      const auto high = static_cast<std::uint64_t>(static_cast<std::uint32_t>(x)) << 32;
      return static_cast<std::int64_t>(high | static_cast<std::uint32_t>(y));
    };

  std::unordered_map<std::int64_t, std::vector<Eigen::Vector3f>> ground_cells;
  ground_cells.reserve(static_cast<std::size_t>(ground.rows()));
  for (Eigen::Index i = 0; i < ground.rows(); ++i) {
    const Eigen::Vector3f point = ground.row(i).transpose();
    if (!point.allFinite()) {
      continue;
    }
    const auto [x, y] = cell_coordinates(point);
    ground_cells[cell_key(x, y)].push_back(point);
  }

  const float xy_tolerance_squared = xy_tolerance * xy_tolerance;
  std::vector<Eigen::Vector3f> retained;
  retained.reserve(static_cast<std::size_t>(nonground.rows()));
  for (Eigen::Index i = 0; i < nonground.rows(); ++i) {
    const Eigen::Vector3f point = nonground.row(i).transpose();
    if (!point.allFinite()) {
      continue;
    }
    const auto [cell_x, cell_y] = cell_coordinates(point);
    bool ground_residual = false;
    for (std::int32_t offset_x = -1; offset_x <= 1 && !ground_residual; ++offset_x) {
      for (std::int32_t offset_y = -1; offset_y <= 1 && !ground_residual; ++offset_y) {
        const auto found = ground_cells.find(cell_key(cell_x + offset_x, cell_y + offset_y));
        if (found == ground_cells.end()) {
          continue;
        }
        for (const auto & ground_point : found->second) {
          const float dx = point.x() - ground_point.x();
          const float dy = point.y() - ground_point.y();
          const float height_delta = point.z() - ground_point.z();
          // Preserve positive protrusions above the local ground estimate. Only points
          // at or below the ground plane within sensor-noise tolerance are residuals.
          ground_residual = dx * dx + dy * dy <= xy_tolerance_squared &&
            height_delta <= 0.0F && height_delta >= -z_tolerance;
          if (ground_residual) {
            break;
          }
        }
      }
    }
    if (!ground_residual) {
      retained.push_back(point);
    }
  }

  Eigen::MatrixX3f filtered(static_cast<Eigen::Index>(retained.size()), 3);
  for (std::size_t i = 0; i < retained.size(); ++i) {
    filtered.row(static_cast<Eigen::Index>(i)) = retained[i].transpose();
  }
  return filtered;
}

}  // namespace patchworkpp_ros::utils

#endif  // PATCHWORKPP_ROS__GROUND_RESIDUAL_FILTER_HPP_
