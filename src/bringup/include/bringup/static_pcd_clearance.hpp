#ifndef BRINGUP__STATIC_PCD_CLEARANCE_HPP_
#define BRINGUP__STATIC_PCD_CLEARANCE_HPP_

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <functional>
#include <limits>
#include <optional>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include <map_msgs/msg/occupancy_grid_update.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include "bringup/map_geometry_event.hpp"

namespace bringup
{

struct StaticPcdCell
{
  std::int64_t x;
  std::int64_t y;

  bool operator==(const StaticPcdCell & other) const noexcept
  {
    return x == other.x && y == other.y;
  }
};

struct StaticPcdCellHash
{
  std::size_t operator()(const StaticPcdCell & cell) const noexcept
  {
    auto seed = std::hash<std::int64_t>{}(cell.x);
    seed ^= std::hash<std::int64_t>{}(cell.y) + 0x9e3779b9U + (seed << 6U) + (seed >> 2U);
    return seed;
  }
};

struct StaticPcdRay3D
{
  double start_x;
  double start_y;
  double start_z;
  double end_x;
  double end_y;
  double end_z;
};

inline bool staticPcdRayClearsPoint(
  const StaticPcdRay3D & ray, double point_x, double point_y, double point_z,
  double xy_resolution, double z_resolution)
{
  if (!std::isfinite(ray.start_x) || !std::isfinite(ray.start_y) ||
    !std::isfinite(ray.start_z) || !std::isfinite(ray.end_x) ||
    !std::isfinite(ray.end_y) || !std::isfinite(ray.end_z) ||
    !std::isfinite(point_x) || !std::isfinite(point_y) || !std::isfinite(point_z) ||
    !std::isfinite(xy_resolution) || xy_resolution <= 0.0 ||
    !std::isfinite(z_resolution) || z_resolution <= 0.0)
  {
    return false;
  }

  const double dx = ray.end_x - ray.start_x;
  const double dy = ray.end_y - ray.start_y;
  const double dz = ray.end_z - ray.start_z;
  const double length_squared = dx * dx + dy * dy + dz * dz;
  if (length_squared <= 1.0e-12) return false;

  const double ray_length = std::sqrt(length_squared);
  const double projection = ((point_x - ray.start_x) * dx +
    (point_y - ray.start_y) * dy + (point_z - ray.start_z) * dz) / length_squared;
  if (projection <= 0.0 || projection >= 1.0 ||
    (1.0 - projection) * ray_length < std::max(xy_resolution, z_resolution))
  {
    // The endpoint is a measured return, so its voxel is not free space.
    return false;
  }

  const double offset_x = ray.start_x + projection * dx - point_x;
  const double offset_y = ray.start_y + projection * dy - point_y;
  const double offset_z = ray.start_z + projection * dz - point_z;
  const double half_xy = xy_resolution * 0.5;
  const double half_z = z_resolution * 0.5;
  return (offset_x * offset_x + offset_y * offset_y) / (half_xy * half_xy) +
         offset_z * offset_z / (half_z * half_z) <= 1.0;
}

class StaticPcdHeightClearanceEvidence
{
public:
  bool observe(std::uint64_t stamp_ns, std::uint64_t max_gap_ns, std::uint32_t required)
  {
    if (stamp_ns == 0 || stamp_ns <= last_stamp_ns_ || required == 0) return false;
    if (last_stamp_ns_ == 0 || stamp_ns - last_stamp_ns_ > max_gap_ns) {
      observations_ = 1;
    } else {
      ++observations_;
    }
    last_stamp_ns_ = stamp_ns;
    if (observations_ >= required && !cleared_) {
      cleared_ = true;
      return true;
    }
    return false;
  }

  bool cleared() const noexcept {return cleared_;}
  std::uint32_t observations() const noexcept {return observations_;}

  void reset() noexcept
  {
    last_stamp_ns_ = 0;
    observations_ = 0;
    cleared_ = false;
  }

private:
  std::uint64_t last_stamp_ns_{0};
  std::uint32_t observations_{0};
  bool cleared_{false};
};

template<typename Callback>
inline void forEachStaticPcdRayCellBeforeEndpoint(
  StaticPcdCell start, StaticPcdCell endpoint, Callback && callback)
{
  const auto dx = std::llabs(endpoint.x - start.x);
  const auto sx = start.x < endpoint.x ? 1 : -1;
  const auto dy = -std::llabs(endpoint.y - start.y);
  const auto sy = start.y < endpoint.y ? 1 : -1;
  auto error = dx + dy;
  auto x = start.x;
  auto y = start.y;
  while (x != endpoint.x || y != endpoint.y) {
    callback(StaticPcdCell{x, y});
    const auto doubled_error = 2 * error;
    if (doubled_error >= dy) {
      error += dy;
      x += sx;
    }
    if (doubled_error <= dx) {
      error += dx;
      y += sy;
    }
  }
}

enum class StaticPcdSnapshotResult
{
  INITIALIZED,
  UPDATED,
  STALE,
  INVALID
};

inline bool staticPcdSameSnapshot(
  const nav_msgs::msg::OccupancyGrid & map,
  const nav_msgs::msg::OccupancyGrid & occupancy_source)
{
  constexpr double tolerance = 1.0e-5;
  const auto & map_info = map.info;
  const auto & source_info = occupancy_source.info;
  const auto & map_origin = map_info.origin;
  const auto & source_origin = source_info.origin;
  return map.header.stamp.sec == occupancy_source.header.stamp.sec &&
         map.header.stamp.nanosec == occupancy_source.header.stamp.nanosec &&
         map.header.frame_id == occupancy_source.header.frame_id &&
         map_info.width == source_info.width && map_info.height == source_info.height &&
         std::abs(static_cast<double>(map_info.resolution) - source_info.resolution) <= tolerance &&
         std::abs(map_origin.position.x - source_origin.position.x) <= tolerance &&
         std::abs(map_origin.position.y - source_origin.position.y) <= tolerance &&
         std::abs(map_origin.position.z - source_origin.position.z) <= tolerance &&
         std::abs(map_origin.orientation.x - source_origin.orientation.x) <= tolerance &&
         std::abs(map_origin.orientation.y - source_origin.orientation.y) <= tolerance &&
         std::abs(map_origin.orientation.z - source_origin.orientation.z) <= tolerance &&
         std::abs(map_origin.orientation.w - source_origin.orientation.w) <= tolerance;
}

inline bool shouldDiscardStaticPcdOccupancySource(
  const nav_msgs::msg::OccupancyGrid & map,
  const nav_msgs::msg::OccupancyGrid & occupancy_source)
{
  if (staticPcdSameSnapshot(map, occupancy_source)) return false;
  return mapStampNanoseconds(occupancy_source.header.stamp) <=
         mapStampNanoseconds(map.header.stamp);
}

inline bool staticPcdMapUpdateCoveredByState(
  std::uint64_t update_stamp_ns, std::uint64_t latest_full_map_stamp_ns,
  std::uint64_t latest_state_stamp_ns)
{
  return update_stamp_ns != 0 &&
         ((latest_full_map_stamp_ns != 0 && update_stamp_ns <= latest_full_map_stamp_ns) ||
         (latest_state_stamp_ns != 0 && update_stamp_ns <= latest_state_stamp_ns));
}

class StaticPcdClearanceTracker
{
public:
  StaticPcdClearanceTracker(
    std::string frame_id, double reference_origin_x, double reference_origin_y,
    double resolution)
  : frame_id_(std::move(frame_id)), reference_origin_x_(reference_origin_x),
    reference_origin_y_(reference_origin_y), resolution_(resolution)
  {
  }

  void reset() noexcept
  {
    current_map_.reset();
    last_stamp_ns_ = 0;
  }

  StaticPcdSnapshotResult observe(
    const nav_msgs::msg::OccupancyGrid & snapshot,
    std::vector<StaticPcdCell> & cleared_cells, std::string * error = nullptr)
  {
    cleared_cells.clear();
    if (!validSnapshot(snapshot, error)) return StaticPcdSnapshotResult::INVALID;
    std::int64_t new_offset_x = 0;
    std::int64_t new_offset_y = 0;
    if (!alignedOffset(
        snapshot.info.origin.position.x, reference_origin_x_, resolution_, new_offset_x) ||
      !alignedOffset(
        snapshot.info.origin.position.y, reference_origin_y_, resolution_, new_offset_y))
    {
      if (error != nullptr) *error = "map origin is not aligned to the PCD reference grid";
      return StaticPcdSnapshotResult::INVALID;
    }
    if (!offsetFitsExtent(new_offset_x, snapshot.info.width) ||
      !offsetFitsExtent(new_offset_y, snapshot.info.height))
    {
      if (error != nullptr) *error = "map dimensions overflow the PCD reference grid";
      return StaticPcdSnapshotResult::INVALID;
    }

    const auto stamp_ns = toNanoseconds(snapshot.header.stamp.sec, snapshot.header.stamp.nanosec);
    if (!current_map_) {
      current_map_ = snapshot;
      last_stamp_ns_ = stamp_ns;
      return StaticPcdSnapshotResult::INITIALIZED;
    }
    if (stamp_ns <= last_stamp_ns_) return StaticPcdSnapshotResult::STALE;
    std::int64_t old_offset_x = 0;
    std::int64_t old_offset_y = 0;
    if (!alignedOffset(
        current_map_->info.origin.position.x, reference_origin_x_, resolution_, old_offset_x) ||
      !alignedOffset(
        current_map_->info.origin.position.y, reference_origin_y_, resolution_, old_offset_y))
    {
      if (error != nullptr) *error = "cached map origin is not aligned to the PCD reference grid";
      return StaticPcdSnapshotResult::INVALID;
    }
    if (!offsetFitsExtent(old_offset_x, current_map_->info.width) ||
      !offsetFitsExtent(old_offset_y, current_map_->info.height))
    {
      if (error != nullptr) *error = "cached map dimensions overflow the PCD reference grid";
      return StaticPcdSnapshotResult::INVALID;
    }
    collectClearTransitions(
      *current_map_, snapshot, old_offset_x, old_offset_y, new_offset_x, new_offset_y,
      cleared_cells);

    current_map_ = snapshot;
    last_stamp_ns_ = stamp_ns;
    return StaticPcdSnapshotResult::UPDATED;
  }

  bool collectPreviouslyClearedPriorCells(
    const nav_msgs::msg::OccupancyGrid & map,
    const nav_msgs::msg::OccupancyGrid & occupancy_source,
    std::vector<StaticPcdCell> & cleared_cells, std::string * error = nullptr) const
  {
    cleared_cells.clear();
    if (!validSnapshot(map, error) || !validSnapshot(occupancy_source, error)) return false;
    const auto & map_info = map.info;
    if (!staticPcdSameSnapshot(map, occupancy_source))
    {
      if (error != nullptr) {
        *error = "map and occupancy source do not describe the same snapshot";
      }
      return false;
    }

    std::int64_t offset_x = 0;
    std::int64_t offset_y = 0;
    if (!alignedOffset(
        map_info.origin.position.x, reference_origin_x_, resolution_, offset_x) ||
      !alignedOffset(
        map_info.origin.position.y, reference_origin_y_, resolution_, offset_y))
    {
      if (error != nullptr) *error = "map origin is not aligned to the PCD reference grid";
      return false;
    }
    if (!offsetFitsExtent(offset_x, map_info.width) ||
      !offsetFitsExtent(offset_y, map_info.height))
    {
      if (error != nullptr) *error = "map dimensions overflow the PCD reference grid";
      return false;
    }

    for (std::uint32_t y = 0; y < map_info.height; ++y) {
      for (std::uint32_t x = 0; x < map_info.width; ++x) {
        const auto index = static_cast<std::size_t>(y) * map_info.width + x;
        const auto source_mask = occupancy_source.data[index];
        if (source_mask < 0 || source_mask > 3) {
          if (error != nullptr) *error = "occupancy source snapshot contains an invalid mask";
          cleared_cells.clear();
          return false;
        }
        if (map.data[index] == 0 &&
          (static_cast<std::uint8_t>(source_mask) & 1U) != 0U)
        {
          cleared_cells.push_back({
            offset_x + static_cast<std::int64_t>(x),
            offset_y + static_cast<std::int64_t>(y)});
        }
      }
    }
    return true;
  }

  bool collectCandidatesNoLongerFree(
    const nav_msgs::msg::OccupancyGrid & snapshot,
    const std::vector<StaticPcdCell> & candidates,
    std::vector<StaticPcdCell> & revoked_cells, std::string * error = nullptr) const
  {
    revoked_cells.clear();
    if (!validSnapshot(snapshot, error)) return false;

    std::int64_t offset_x = 0;
    std::int64_t offset_y = 0;
    if (!alignedOffset(
        snapshot.info.origin.position.x, reference_origin_x_, resolution_, offset_x) ||
      !alignedOffset(
        snapshot.info.origin.position.y, reference_origin_y_, resolution_, offset_y))
    {
      if (error != nullptr) *error = "map origin is not aligned to the PCD reference grid";
      return false;
    }
    if (!offsetFitsExtent(offset_x, snapshot.info.width) ||
      !offsetFitsExtent(offset_y, snapshot.info.height))
    {
      if (error != nullptr) *error = "map dimensions overflow the PCD reference grid";
      return false;
    }

    const auto max_x = offset_x + static_cast<std::int64_t>(snapshot.info.width);
    const auto max_y = offset_y + static_cast<std::int64_t>(snapshot.info.height);
    for (const auto & cell : candidates) {
      if (cell.x < offset_x || cell.x >= max_x || cell.y < offset_y || cell.y >= max_y) {
        revoked_cells.push_back(cell);
        continue;
      }
      const auto local_x = static_cast<std::uint32_t>(cell.x - offset_x);
      const auto local_y = static_cast<std::uint32_t>(cell.y - offset_y);
      const auto index = static_cast<std::size_t>(local_y) * snapshot.info.width + local_x;
      if (snapshot.data[index] != 0) {
        // Only an explicit free value can continue to support a clearance candidate.
        revoked_cells.push_back(cell);
      }
    }
    return true;
  }

  bool collectStartupPriorClearanceCandidates(
    const nav_msgs::msg::OccupancyGrid & full_map_baseline,
    const nav_msgs::msg::OccupancyGrid & occupancy_source,
    const nav_msgs::msg::OccupancyGrid & current_map,
    std::vector<StaticPcdCell> & candidates, std::string * error = nullptr) const
  {
    candidates.clear();
    std::vector<StaticPcdCell> prior_clearance_cells;
    if (!collectPreviouslyClearedPriorCells(
        full_map_baseline, occupancy_source, prior_clearance_cells, error))
    {
      return false;
    }
    if (mapStampNanoseconds(current_map.header.stamp) <
      mapStampNanoseconds(full_map_baseline.header.stamp))
    {
      if (error != nullptr) *error = "current map predates the full-map startup baseline";
      return false;
    }

    std::vector<StaticPcdCell> no_longer_free;
    if (!collectCandidatesNoLongerFree(current_map, prior_clearance_cells, no_longer_free, error)) {
      return false;
    }
    const std::unordered_set<StaticPcdCell, StaticPcdCellHash> blocked_cells(
      no_longer_free.begin(), no_longer_free.end());
    candidates.reserve(prior_clearance_cells.size());
    for (const auto & cell : prior_clearance_cells) {
      if (blocked_cells.find(cell) == blocked_cells.end()) candidates.push_back(cell);
    }
    return true;
  }

private:
  static std::uint64_t toNanoseconds(std::int32_t sec, std::uint32_t nanosec)
  {
    return static_cast<std::uint64_t>(sec) * 1000000000ULL + nanosec;
  }

  bool validSnapshot(const nav_msgs::msg::OccupancyGrid & snapshot, std::string * error) const
  {
    const auto & orientation = snapshot.info.origin.orientation;
    if (frame_id_.empty() || snapshot.header.frame_id != frame_id_ ||
      snapshot.header.stamp.sec < 0 || snapshot.header.stamp.nanosec >= 1000000000U ||
      snapshot.info.width == 0 || snapshot.info.height == 0 ||
      !std::isfinite(snapshot.info.resolution) ||
      std::abs(static_cast<double>(snapshot.info.resolution) - resolution_) > 1.0e-5 ||
      !std::isfinite(snapshot.info.origin.position.x) ||
      !std::isfinite(snapshot.info.origin.position.y) ||
      !std::isfinite(snapshot.info.origin.position.z) ||
      std::abs(snapshot.info.origin.position.z) > 1.0e-5 ||
      std::abs(orientation.x) > 1.0e-5 || std::abs(orientation.y) > 1.0e-5 ||
      std::abs(orientation.z) > 1.0e-5 ||
      std::abs(std::abs(orientation.w) - 1.0) > 1.0e-5 ||
      snapshot.data.size() != static_cast<std::size_t>(snapshot.info.width) * snapshot.info.height)
    {
      if (error != nullptr) *error = "full map snapshot has invalid frame, geometry, or data";
      return false;
    }
    return true;
  }

  static bool alignedOffset(
    double origin, double reference_origin, double resolution, std::int64_t & offset)
  {
    if (!std::isfinite(origin) || !std::isfinite(reference_origin) ||
      !std::isfinite(resolution) || resolution <= 0.0)
    {
      return false;
    }
    const double cells = (origin - reference_origin) / resolution;
    const double rounded = std::round(cells);
    if (!std::isfinite(cells) || std::abs(cells - rounded) > 1.0e-4 ||
      rounded < static_cast<double>(std::numeric_limits<std::int64_t>::min()) ||
      rounded >= static_cast<double>(std::numeric_limits<std::int64_t>::max()))
    {
      return false;
    }
    offset = static_cast<std::int64_t>(rounded);
    return true;
  }

  static bool offsetFitsExtent(std::int64_t offset, std::uint32_t extent)
  {
    return offset <= std::numeric_limits<std::int64_t>::max() -
      static_cast<std::int64_t>(extent);
  }

  static void collectClearTransitions(
    const nav_msgs::msg::OccupancyGrid & previous,
    const nav_msgs::msg::OccupancyGrid & next,
    std::int64_t previous_offset_x, std::int64_t previous_offset_y,
    std::int64_t next_offset_x, std::int64_t next_offset_y,
    std::vector<StaticPcdCell> & cleared_cells)
  {
    const auto old_min_x = previous_offset_x;
    const auto old_min_y = previous_offset_y;
    const auto old_max_x = old_min_x + static_cast<std::int64_t>(previous.info.width);
    const auto old_max_y = old_min_y + static_cast<std::int64_t>(previous.info.height);
    for (std::uint32_t y = 0; y < next.info.height; ++y) {
      const auto world_y = next_offset_y + static_cast<std::int64_t>(y);
      if (world_y < old_min_y || world_y >= old_max_y) continue;
      for (std::uint32_t x = 0; x < next.info.width; ++x) {
        const auto world_x = next_offset_x + static_cast<std::int64_t>(x);
        if (world_x < old_min_x || world_x >= old_max_x) continue;
        const auto old_x = static_cast<std::uint32_t>(world_x - old_min_x);
        const auto old_y = static_cast<std::uint32_t>(world_y - old_min_y);
        const auto old_index = static_cast<std::size_t>(old_y) * previous.info.width + old_x;
        const auto new_index = static_cast<std::size_t>(y) * next.info.width + x;
        if (previous.data[old_index] != 0 && next.data[new_index] == 0) {
          cleared_cells.push_back({world_x, world_y});
        }
      }
    }
  }

  std::string frame_id_;
  double reference_origin_x_;
  double reference_origin_y_;
  double resolution_;
  std::optional<nav_msgs::msg::OccupancyGrid> current_map_;
  std::uint64_t last_stamp_ns_{0};
};

enum class StaticPcdFullMapDisposition
{
  ACCEPT,
  ACCEPT_NEW_EPOCH,
  ACCEPT_NEW_GEOMETRY,
  ADVANCE_WATERMARK_ONLY,
  REJECT_STALE,
  REJECT_MISMATCH
};

inline StaticPcdFullMapDisposition assessStaticPcdFullMap(
  const nav_msgs::msg::OccupancyGrid & map, const MapGeometryEvent & event,
  std::uint64_t current_epoch, std::uint64_t current_geometry_version,
  std::uint64_t latest_full_stamp_ns, std::uint64_t latest_state_stamp_ns,
  const nav_msgs::msg::OccupancyGrid * current_map = nullptr)
{
  if (map.header.stamp.sec < 0 || map.header.stamp.nanosec >= 1000000000U ||
    event.stamp_ns == 0 || event.publisher_epoch == 0 || event.geometry_version == 0 ||
    event.kind != MapGeometryEventKind::FULL_MAP ||
    event.stamp_ns != mapStampNanoseconds(map.header.stamp) ||
    !mapGeometryEventMatches(event, map))
  {
    return StaticPcdFullMapDisposition::REJECT_MISMATCH;
  }
  if (event.publisher_epoch < current_epoch ||
    (event.publisher_epoch == current_epoch &&
    event.geometry_version < current_geometry_version))
  {
    return StaticPcdFullMapDisposition::REJECT_STALE;
  }
  if (event.publisher_epoch > current_epoch) {
    return StaticPcdFullMapDisposition::ACCEPT_NEW_EPOCH;
  }
  if (event.geometry_version > current_geometry_version) {
    return StaticPcdFullMapDisposition::ACCEPT_NEW_GEOMETRY;
  }
  if (current_map != nullptr && !mapGeometryEventMatches(event, *current_map)) {
    return StaticPcdFullMapDisposition::REJECT_MISMATCH;
  }
  if (event.stamp_ns <= latest_full_stamp_ns) {
    return StaticPcdFullMapDisposition::REJECT_STALE;
  }
  if (event.stamp_ns <= latest_state_stamp_ns) {
    return StaticPcdFullMapDisposition::ADVANCE_WATERMARK_ONLY;
  }
  return StaticPcdFullMapDisposition::ACCEPT;
}

enum class StaticPcdMapUpdateDisposition
{
  READY,
  WAIT_FOR_FULL_MAP,
  REJECT_STALE,
  REJECT_MISMATCH,
  REJECT_INVALID
};

inline StaticPcdMapUpdateDisposition assessStaticPcdMapUpdate(
  const nav_msgs::msg::OccupancyGrid & map,
  const map_msgs::msg::OccupancyGridUpdate & update,
  const MapGeometryEvent & event,
  std::uint64_t current_epoch, std::uint64_t current_geometry_version,
  std::uint64_t latest_state_stamp_ns)
{
  if (update.header.stamp.sec < 0 || update.header.stamp.nanosec >= 1000000000U ||
    event.publisher_epoch == 0 || event.geometry_version == 0 || event.stamp_ns == 0)
  {
    return StaticPcdMapUpdateDisposition::REJECT_INVALID;
  }
  if (event.kind != MapGeometryEventKind::INCREMENTAL_UPDATE ||
    event.stamp_ns != mapStampNanoseconds(update.header.stamp))
  {
    return StaticPcdMapUpdateDisposition::REJECT_INVALID;
  }
  if (event.publisher_epoch > current_epoch ||
    (event.publisher_epoch == current_epoch &&
    event.geometry_version > current_geometry_version))
  {
    return StaticPcdMapUpdateDisposition::WAIT_FOR_FULL_MAP;
  }
  if (event.publisher_epoch < current_epoch || event.geometry_version < current_geometry_version ||
    event.stamp_ns <= latest_state_stamp_ns)
  {
    return StaticPcdMapUpdateDisposition::REJECT_STALE;
  }
  if (event.publisher_epoch != current_epoch ||
    event.geometry_version != current_geometry_version ||
    update.header.frame_id != map.header.frame_id || !mapGeometryEventMatches(event, map))
  {
    return StaticPcdMapUpdateDisposition::REJECT_MISMATCH;
  }
  if (map.info.width == 0 || map.info.height == 0 ||
    map.data.size() != static_cast<std::size_t>(map.info.width) * map.info.height ||
    update.width == 0 || update.height == 0 ||
    update.x > map.info.width || update.y > map.info.height ||
    update.width > map.info.width - update.x ||
    update.height > map.info.height - update.y ||
    update.data.size() != static_cast<std::size_t>(update.width) * update.height)
  {
    return StaticPcdMapUpdateDisposition::REJECT_INVALID;
  }
  return StaticPcdMapUpdateDisposition::READY;
}

inline bool applyStaticPcdMapUpdate(
  nav_msgs::msg::OccupancyGrid & map,
  const map_msgs::msg::OccupancyGridUpdate & update,
  const MapGeometryEvent & event,
  std::uint64_t current_epoch, std::uint64_t current_geometry_version)
{
  if (assessStaticPcdMapUpdate(
      map, update, event, current_epoch, current_geometry_version,
      mapStampNanoseconds(map.header.stamp)) != StaticPcdMapUpdateDisposition::READY)
  {
    return false;
  }

  for (std::uint32_t y = 0; y < update.height; ++y) {
    const auto destination = static_cast<std::size_t>(update.y + y) * map.info.width + update.x;
    const auto source = static_cast<std::size_t>(y) * update.width;
    std::copy_n(update.data.begin() + static_cast<std::ptrdiff_t>(source), update.width,
      map.data.begin() + static_cast<std::ptrdiff_t>(destination));
  }
  map.header.stamp = update.header.stamp;
  return true;
}

struct StaticPcdRefreshPlan
{
  bool should_publish{false};
  bool full_refresh{false};
};

class StaticPcdRefreshState
{
public:
  static constexpr auto kIncrementalPublishPeriod = std::chrono::milliseconds(200);
  static constexpr auto kFullSnapshotPeriod = std::chrono::seconds(1);

  void markDirty() {dirty_ = true;}

  StaticPcdRefreshPlan observeSubscribers(
    const std::vector<std::string> & identifiers,
    std::chrono::steady_clock::time_point now)
  {
    auto current_identifiers = identifiers;
    std::sort(current_identifiers.begin(), current_identifiers.end());
    current_identifiers.erase(
      std::unique(current_identifiers.begin(), current_identifiers.end()),
      current_identifiers.end());
    if (current_identifiers.empty()) {
      had_subscriber_ = false;
      subscriber_identifiers_.clear();
      drawn_ = false;
      return {};
    }
    if (!had_subscriber_ || current_identifiers != subscriber_identifiers_) {
      full_refresh_pending_ = true;
      had_subscriber_ = true;
    }
    subscriber_identifiers_ = std::move(current_identifiers);
    const bool full_refresh_required = full_refresh_pending_ || !drawn_ ||
      last_full_publish_ == std::chrono::steady_clock::time_point{} ||
      now - last_full_publish_ >= kFullSnapshotPeriod;
    const bool full_rate_limit_elapsed = last_full_publish_ ==
      std::chrono::steady_clock::time_point{} ||
      now - last_full_publish_ >= kFullSnapshotPeriod;
    const bool incremental_rate_limit_elapsed = last_publish_ ==
      std::chrono::steady_clock::time_point{} ||
      now - last_publish_ >= kIncrementalPublishPeriod;
    const bool publish_full = full_refresh_required && full_rate_limit_elapsed;
    const bool publish_incremental = dirty_ && incremental_rate_limit_elapsed;
    return {publish_full || publish_incremental, publish_full};
  }

  void markPublished(std::chrono::steady_clock::time_point now, bool full_refresh = true)
  {
    dirty_ = false;
    drawn_ = true;
    last_publish_ = now;
    if (full_refresh) {
      full_refresh_pending_ = false;
      last_full_publish_ = now;
    }
  }

private:
  bool full_refresh_pending_{true};
  bool dirty_{false};
  bool drawn_{false};
  bool had_subscriber_{false};
  std::chrono::steady_clock::time_point last_full_publish_{};
  std::chrono::steady_clock::time_point last_publish_{};
  std::vector<std::string> subscriber_identifiers_;
};

}  // namespace bringup

#endif  // BRINGUP__STATIC_PCD_CLEARANCE_HPP_
