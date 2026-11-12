#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <limits>
#include <stdexcept>
#include <system_error>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

namespace bringup
{

struct TemporalVoxelKey
{
  std::int64_t x;
  std::int64_t y;
  std::int64_t z;

  bool operator==(const TemporalVoxelKey & other) const
  {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct TemporalVoxelKeyHash
{
  std::size_t operator()(const TemporalVoxelKey & key) const
  {
    std::size_t hash = std::hash<std::int64_t>{}(key.x);
    hash ^= std::hash<std::int64_t>{}(key.y) + 0x9e3779b9 + (hash << 6) + (hash >> 2);
    hash ^= std::hash<std::int64_t>{}(key.z) + 0x9e3779b9 + (hash << 6) + (hash >> 2);
    return hash;
  }
};

struct TemporalVoxelParameters
{
  double voxel_size{0.18};
  int min_observations{4};
  double min_observation_duration{0.4};
  double candidate_timeout{1.5};
  // MID360 runs at about 10 Hz. The active map uses a one-second miss
  // hysteresis for dynamic clearing; the display window only filters output
  // and never removes a voxel from the global map.
  int confirmed_miss_limit{10};
  double archive_min_observation_duration{1.0};
  double archive_min_origin_separation{0.5};
  double ray_angular_resolution{0.03490658503988659};
  double min_range{0.5};
  double max_range{40.0};
};

struct TemporalVoxel
{
  std::uint32_t observations{0};
  std::uint64_t last_seen_frame{0};
  std::uint64_t last_miss_frame{0};
  double first_seen{0.0};
  double last_seen{0.0};
  double sum_x{0.0};
  double sum_y{0.0};
  double sum_z{0.0};
  double last_origin_x{0.0};
  double last_origin_y{0.0};
  double last_origin_z{0.0};
  std::uint32_t consecutive_misses{0};
  std::uint32_t distinct_origins{0};
  bool has_observation_origin{false};
  std::vector<pcl::PointXYZ> complete_points;
  bool confirmed{false};
  bool archived{false};

  pcl::PointXYZ representative() const
  {
    const double count = static_cast<double>(std::max<std::uint32_t>(observations, 1));
    return pcl::PointXYZ{
      static_cast<float>(sum_x / count),
      static_cast<float>(sum_y / count),
      static_cast<float>(sum_z / count)};
  }
};

struct TemporalVoxelFrameResult
{
  std::vector<pcl::PointXYZ> current_static_points;
  std::vector<pcl::PointXYZ> rejected_points;
  bool map_changed{false};
};

class TemporalVoxelMap
{
public:
  explicit TemporalVoxelMap(const TemporalVoxelParameters & parameters)
  : parameters_(parameters)
  {
    if (parameters_.voxel_size <= 0.0 || parameters_.min_observations < 1 ||
      parameters_.min_observation_duration < 0.0 || parameters_.candidate_timeout <= 0.0 ||
      parameters_.confirmed_miss_limit < 1 ||
      parameters_.archive_min_observation_duration < 0.0 ||
      parameters_.archive_min_origin_separation < 0.0 || parameters_.min_range < 0.0 ||
      parameters_.max_range <= parameters_.min_range ||
      parameters_.ray_angular_resolution <= 0.0)
    {
      throw std::invalid_argument("invalid temporal voxel parameter");
    }
  }

  void set_origin(const double x, const double y, const double z)
  {
    origin_x_ = x;
    origin_y_ = y;
    origin_z_ = z;
    have_origin_ = true;
  }

  TemporalVoxelFrameResult process_frame(
    const std::vector<pcl::PointXYZ> & points, const double stamp)
  {
    TemporalVoxelFrameResult result;
    if (last_stamp_ > 0.0 && stamp < last_stamp_) {
      voxels_.clear();
      historical_static_points_.clear();
      confirmed_voxel_count_ = 0;
      result.map_changed = true;
      frame_sequence_ = 0;
    }
    last_stamp_ = stamp;
    ++frame_sequence_;

    std::unordered_set<TemporalVoxelKey, TemporalVoxelKeyHash> touched;
    std::unordered_set<TemporalVoxelKey, TemporalVoxelKeyHash> free_keys;
    std::vector<pcl::PointXYZ> frame_points;
    frame_points.reserve(points.size());
    result.rejected_points.reserve(points.size());

    for (const pcl::PointXYZ & point : points) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
        continue;
      }
      if (!in_range(point)) {
        result.rejected_points.push_back(point);
        continue;
      }

      const TemporalVoxelKey key = key_for(point);
      auto voxel_it = voxels_.find(key);
      if (voxel_it == voxels_.end()) {
        voxel_it = voxels_.emplace(key, TemporalVoxel{}).first;
      }
      if (touched.insert(key).second) {
        frame_points.push_back(point);
        TemporalVoxel & voxel = voxel_it->second;
        if (!voxel.confirmed && voxel.observations > 0 &&
          voxel.last_seen_frame + 1 != frame_sequence_)
        {
          voxel.observations = 0;
          voxel.sum_x = 0.0;
          voxel.sum_y = 0.0;
          voxel.sum_z = 0.0;
          voxel.first_seen = 0.0;
          voxel.last_seen = 0.0;
          voxel.last_seen_frame = 0;
          voxel.last_miss_frame = 0;
          voxel.consecutive_misses = 0;
          voxel.last_origin_x = 0.0;
          voxel.last_origin_y = 0.0;
          voxel.last_origin_z = 0.0;
          voxel.distinct_origins = 0;
          voxel.has_observation_origin = false;
          voxel.complete_points.clear();
          voxel.archived = false;
        }
        ++voxel.observations;
        voxel.last_seen_frame = frame_sequence_;
        if (!voxel.has_observation_origin) {
          voxel.last_origin_x = origin_x_;
          voxel.last_origin_y = origin_y_;
          voxel.last_origin_z = origin_z_;
          voxel.distinct_origins = 1;
          voxel.has_observation_origin = have_origin_;
        } else if (have_origin_) {
          const double dx = origin_x_ - voxel.last_origin_x;
          const double dy = origin_y_ - voxel.last_origin_y;
          const double dz = origin_z_ - voxel.last_origin_z;
          if (dx * dx + dy * dy + dz * dz >=
            parameters_.archive_min_origin_separation * parameters_.archive_min_origin_separation)
          {
            ++voxel.distinct_origins;
            voxel.last_origin_x = origin_x_;
            voxel.last_origin_y = origin_y_;
            voxel.last_origin_z = origin_z_;
          }
        }
        if (voxel.observations == 1) {
          voxel.first_seen = stamp;
        }
        voxel.last_seen = stamp;
        voxel.sum_x += point.x;
        voxel.sum_y += point.y;
        voxel.sum_z += point.z;
        voxel.consecutive_misses = 0;
        voxel.last_miss_frame = 0;
        if (!voxel.confirmed &&
          voxel.observations >= static_cast<std::uint32_t>(parameters_.min_observations) &&
          voxel.last_seen - voxel.first_seen + 1e-9 >= parameters_.min_observation_duration)
        {
          voxel.confirmed = true;
          ++confirmed_voxel_count_;
          result.map_changed = true;
        }
        if (voxel.confirmed) {
          result.map_changed = true;
          if (!voxel.archived &&
            voxel.last_seen - voxel.first_seen + 1e-9 >=
            parameters_.archive_min_observation_duration &&
            voxel.distinct_origins >= 2)
          {
            voxel.archived = true;
            historical_static_points_[key] = std::move(voxel.complete_points);
          }
        }
      }
      if (voxel_it->second.archived) {
        historical_static_points_[key].push_back(point);
      } else {
        voxel_it->second.complete_points.push_back(point);
      }
      if (!voxel_it->second.confirmed) {
        result.rejected_points.push_back(point);
      }
    }

    if (confirmed_voxel_count_ > 0 && have_origin_) {
      std::unordered_map<std::uint64_t, std::pair<double, pcl::PointXYZ>> ray_endpoints;
      for (const pcl::PointXYZ & point : frame_points) {
        const double dx = static_cast<double>(point.x) - origin_x_;
        const double dy = static_cast<double>(point.y) - origin_y_;
        const double dz = static_cast<double>(point.z) - origin_z_;
        const double horizontal = std::sqrt(dx * dx + dy * dy);
        const double range = std::sqrt(horizontal * horizontal + dz * dz);
        if (range <= 0.0) {
          continue;
        }
        constexpr double pi = 3.14159265358979323846;
        const double azimuth = std::atan2(dy, dx) + pi;
        const double elevation = std::atan2(dz, horizontal) + pi / 2.0;
        const auto azimuth_bin = static_cast<std::uint64_t>(
          azimuth / parameters_.ray_angular_resolution);
        const auto elevation_bin = static_cast<std::uint64_t>(
          elevation / parameters_.ray_angular_resolution);
        const std::uint64_t bin = (azimuth_bin << 32U) | elevation_bin;
        auto endpoint_it = ray_endpoints.find(bin);
        if (endpoint_it == ray_endpoints.end() || range < endpoint_it->second.first) {
          ray_endpoints[bin] = {range, point};
        }
      }
      for (const auto & endpoint : ray_endpoints) {
        trace_ray(endpoint.second.second, free_keys);
      }
    }
    for (const TemporalVoxelKey & key : free_keys) {
      if (touched.find(key) != touched.end()) {
        continue;
      }
      auto voxel_it = voxels_.find(key);
      if (voxel_it == voxels_.end() || !voxel_it->second.confirmed) {
        continue;
      }
      TemporalVoxel & voxel = voxel_it->second;
      if (voxel.last_miss_frame + 1 == frame_sequence_) {
        ++voxel.consecutive_misses;
      } else {
        voxel.consecutive_misses = 1;
      }
      voxel.last_miss_frame = frame_sequence_;
      if (voxel.consecutive_misses >=
        static_cast<std::uint32_t>(parameters_.confirmed_miss_limit))
      {
        historical_static_points_.erase(key);
        voxels_.erase(voxel_it);
        --confirmed_voxel_count_;
        result.map_changed = true;
      }
    }

    result.current_static_points.reserve(frame_points.size());
    for (const pcl::PointXYZ & point : frame_points) {
      const TemporalVoxel & voxel = voxels_.at(key_for(point));
      if (voxel.confirmed) {
        result.current_static_points.push_back(voxel.representative());
      }
    }
    return result;
  }

  void cleanup_candidates()
  {
    if (last_stamp_ <= 0.0) {
      return;
    }
    for (auto voxel_it = voxels_.begin(); voxel_it != voxels_.end();) {
      if (!voxel_it->second.confirmed &&
        last_stamp_ - voxel_it->second.last_seen > parameters_.candidate_timeout)
      {
        voxel_it = voxels_.erase(voxel_it);
      } else {
        ++voxel_it;
      }
    }
  }

  std::vector<pcl::PointXYZ> static_points() const
  {
    std::vector<pcl::PointXYZ> points;
    points.reserve(voxels_.size());
    for (const auto & entry : voxels_) {
      if (entry.second.confirmed) {
        points.push_back(entry.second.representative());
      }
    }
    return points;
  }

  std::vector<pcl::PointXYZ> complete_static_points() const
  {
    std::vector<pcl::PointXYZ> points;
    std::size_t point_count = 0;
    for (const auto & entry : historical_static_points_) {
      point_count += entry.second.size();
    }
    for (const auto & entry : voxels_) {
      if (entry.second.confirmed && !entry.second.archived) {
        point_count += entry.second.complete_points.size();
      }
    }
    points.reserve(point_count);
    for (const auto & entry : historical_static_points_) {
      points.insert(points.end(), entry.second.begin(), entry.second.end());
    }
    for (const auto & entry : voxels_) {
      if (entry.second.confirmed && !entry.second.archived) {
        points.insert(
          points.end(), entry.second.complete_points.begin(), entry.second.complete_points.end());
      }
    }
    return points;
  }

  std::size_t voxel_count() const
  {
    return voxels_.size();
  }

  std::size_t confirmed_count() const
  {
    return confirmed_voxel_count_;
  }

  std::size_t archived_count() const
  {
    return historical_static_points_.size();
  }

private:
  TemporalVoxelKey key_for(const pcl::PointXYZ & point) const
  {
    return {
      static_cast<std::int64_t>(std::floor(static_cast<double>(point.x) / parameters_.voxel_size)),
      static_cast<std::int64_t>(std::floor(static_cast<double>(point.y) / parameters_.voxel_size)),
      static_cast<std::int64_t>(std::floor(static_cast<double>(point.z) / parameters_.voxel_size))};
  }

  bool in_range(const pcl::PointXYZ & point) const
  {
    if (!have_origin_) {
      return true;
    }
    const double dx = static_cast<double>(point.x) - origin_x_;
    const double dy = static_cast<double>(point.y) - origin_y_;
    const double dz = static_cast<double>(point.z) - origin_z_;
    const double range_squared = dx * dx + dy * dy + dz * dz;
    return range_squared >= parameters_.min_range * parameters_.min_range &&
      range_squared <= parameters_.max_range * parameters_.max_range;
  }

  void trace_ray(
    const pcl::PointXYZ & endpoint,
    std::unordered_set<TemporalVoxelKey, TemporalVoxelKeyHash> & free_keys) const
  {
    if (!have_origin_) {
      return;
    }
    const double dx = static_cast<double>(endpoint.x) - origin_x_;
    const double dy = static_cast<double>(endpoint.y) - origin_y_;
    const double dz = static_cast<double>(endpoint.z) - origin_z_;
    const double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
    TemporalVoxelKey current = {
      static_cast<std::int64_t>(std::floor(origin_x_ / parameters_.voxel_size)),
      static_cast<std::int64_t>(std::floor(origin_y_ / parameters_.voxel_size)),
      static_cast<std::int64_t>(std::floor(origin_z_ / parameters_.voxel_size))};
    const TemporalVoxelKey endpoint_key = key_for(endpoint);
    if (current == endpoint_key) {
      return;
    }

    const auto step_x = dx > 0.0 ? 1 : -1;
    const auto step_y = dy > 0.0 ? 1 : -1;
    const auto step_z = dz > 0.0 ? 1 : -1;
    const double infinity = std::numeric_limits<double>::infinity();
    const double delta_x = dx == 0.0 ? infinity : parameters_.voxel_size / std::abs(dx);
    const double delta_y = dy == 0.0 ? infinity : parameters_.voxel_size / std::abs(dy);
    const double delta_z = dz == 0.0 ? infinity : parameters_.voxel_size / std::abs(dz);
    const double next_x = dx == 0.0 ? infinity : (dx > 0.0 ?
      (static_cast<double>(current.x + 1) * parameters_.voxel_size - origin_x_) / dx :
      (origin_x_ - static_cast<double>(current.x) * parameters_.voxel_size) / -dx);
    const double next_y = dy == 0.0 ? infinity : (dy > 0.0 ?
      (static_cast<double>(current.y + 1) * parameters_.voxel_size - origin_y_) / dy :
      (origin_y_ - static_cast<double>(current.y) * parameters_.voxel_size) / -dy);
    const double next_z = dz == 0.0 ? infinity : (dz > 0.0 ?
      (static_cast<double>(current.z + 1) * parameters_.voxel_size - origin_z_) / dz :
      (origin_z_ - static_cast<double>(current.z) * parameters_.voxel_size) / -dz);
    double max_x = dx == 0.0 ? infinity : next_x;
    double max_y = dy == 0.0 ? infinity : next_y;
    double max_z = dz == 0.0 ? infinity : next_z;
    while (!(current == endpoint_key)) {
      const double minimum = std::min({max_x, max_y, max_z});
      if (max_x <= minimum) {
        current.x += step_x;
        max_x += delta_x;
      }
      if (max_y <= minimum) {
        current.y += step_y;
        max_y += delta_y;
      }
      if (max_z <= minimum) {
        current.z += step_z;
        max_z += delta_z;
      }
      if (current == endpoint_key) {
        break;
      }
      const auto voxel_it = voxels_.find(current);
      if (voxel_it != voxels_.end() && voxel_it->second.confirmed) {
        free_keys.insert(current);
      }
    }
  }

  TemporalVoxelParameters parameters_;
  std::unordered_map<TemporalVoxelKey, TemporalVoxel, TemporalVoxelKeyHash> voxels_;
  std::unordered_map<TemporalVoxelKey, std::vector<pcl::PointXYZ>, TemporalVoxelKeyHash>
    historical_static_points_;
  std::size_t confirmed_voxel_count_{0};
  std::uint64_t frame_sequence_{0};
  double last_stamp_{0.0};
  double origin_x_{0.0};
  double origin_y_{0.0};
  double origin_z_{0.0};
  bool have_origin_{false};
};

inline bool save_points_to_pcd(
  const std::string & output_path,
  const std::vector<pcl::PointXYZ> & points,
  std::string * error_message = nullptr)
{
  if (points.empty()) {
    if (error_message != nullptr) {
      *error_message = "No confirmed static points to save";
    }
    return false;
  }

  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.reserve(points.size());
  for (const auto & point : points) {
    cloud.push_back(point);
  }
  cloud.width = static_cast<std::uint32_t>(cloud.size());
  cloud.height = 1;
  cloud.is_dense = true;

  const std::string temporary_path = output_path + ".tmp";
  try {
    const std::filesystem::path path(output_path);
    if (path.has_parent_path()) {
      std::filesystem::create_directories(path.parent_path());
    }
    if (pcl::io::savePCDFileBinary(temporary_path, cloud) != 0) {
      std::error_code cleanup_error;
      std::filesystem::remove(temporary_path, cleanup_error);
      if (error_message != nullptr) {
        *error_message = "Failed to write " + temporary_path;
      }
      return false;
    }
    std::filesystem::rename(temporary_path, output_path);
    return true;
  } catch (const std::exception & error) {
    std::error_code cleanup_error;
    std::filesystem::remove(temporary_path, cleanup_error);
    if (error_message != nullptr) {
      *error_message = error.what();
    }
    return false;
  }
}

}  // namespace bringup
