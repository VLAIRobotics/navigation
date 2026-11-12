#ifndef BRINGUP__ONLINE_MAP_STATE_HPP_
#define BRINGUP__ONLINE_MAP_STATE_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <map>
#include <optional>
#include <set>
#include <vector>

#include <builtin_interfaces/msg/time.hpp>
#include <map_msgs/msg/occupancy_grid_update.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <std_msgs/msg/u_int64_multi_array.hpp>

#include "bringup/map_geometry_event.hpp"

namespace bringup
{

class OnlineMapState
{
public:
  static constexpr std::size_t kMaximumPendingFullMaps = 4;
  static constexpr std::size_t kMaximumPendingUpdates = 32;
  static constexpr std::size_t kMaximumPendingEvents = 128;
  static constexpr std::size_t kMaximumCoveredUpdateEvents = 256;

  bool receiveFullMap(const nav_msgs::msg::OccupancyGrid & map)
  {
    map_traffic_seen_ = true;
    const auto previous_revision = revision_;
    const auto stamp_ns = mapStampNanoseconds(map.header.stamp);
    if (!validMap(map) || stamp_ns == 0) {
      invalidateDeltaChain();
      return false;
    }
    const auto [entry, inserted] = pending_full_maps_.emplace(stamp_ns, map);
    if (!inserted && !sameOccupancyGrid(entry->second, map)) {
      pending_full_maps_.erase(entry);
      ambiguous_full_stamps_.insert(stamp_ns);
    }
    drain();
    enforceQueueBounds();
    return revision_ != previous_revision;
  }

  bool receiveUpdate(const map_msgs::msg::OccupancyGridUpdate & update)
  {
    map_traffic_seen_ = true;
    const auto previous_revision = revision_;
    const auto stamp_ns = mapStampNanoseconds(update.header.stamp);
    if (stamp_ns == 0) {
      invalidateDeltaChain();
      return false;
    }
    // Source timestamps may reset when the publisher restarts.  A timestamp
    // alone cannot prove that this body was superseded by the current map.
    if (pending_update_events_.find(stamp_ns) == pending_update_events_.end() &&
      covered_update_events_.find(stamp_ns) != covered_update_events_.end())
    {
      quarantineUpdate(update);
      drain();
      enforceQueueBounds();
      return revision_ != previous_revision;
    }
    const auto [entry, inserted] = pending_updates_.emplace(stamp_ns, update);
    if (!inserted && !sameUpdate(entry->second, update)) {
      pending_updates_.erase(entry);
      ambiguous_update_stamps_.insert(stamp_ns);
    }
    drain();
    enforceQueueBounds();
    return revision_ != previous_revision;
  }

  bool receiveGeometryEvent(const std_msgs::msg::UInt64MultiArray & message)
  {
    map_traffic_seen_ = true;
    const auto previous_revision = revision_;
    MapGeometryEvent event;
    if (!parseMapGeometryEvent(message, event)) {
      invalidateDeltaChain();
      return false;
    }
    if (event.kind == MapGeometryEventKind::INCREMENTAL_UPDATE) {
      const auto covered = covered_update_events_.find(event.stamp_ns);
      if (covered != covered_update_events_.end()) {
        if (sameGeometryEvent(covered->second, event)) {
          pending_updates_.erase(event.stamp_ns);
          quarantined_updates_.erase(event.stamp_ns);
          ambiguous_update_stamps_.erase(event.stamp_ns);
          drain();
          enforceQueueBounds();
          return revision_ != previous_revision;
        }
        covered_update_events_.erase(covered);
        restoreQuarantinedUpdate(event.stamp_ns);
      }
      if (isStale(event)) {
        rememberCoveredUpdateEvent(event);
        pending_updates_.erase(event.stamp_ns);
        quarantined_updates_.erase(event.stamp_ns);
        ambiguous_update_stamps_.erase(event.stamp_ns);
        drain();
        enforceQueueBounds();
        return revision_ != previous_revision;
      }
      restoreQuarantinedUpdate(event.stamp_ns);
    }
    auto & events = event.kind == MapGeometryEventKind::FULL_MAP ?
      pending_full_events_ : pending_update_events_;
    const auto [entry, inserted] = events.emplace(event.stamp_ns, event);
    if (!inserted && serializeMapGeometryEvent(entry->second).data != message.data) {
      events.erase(entry);
      if (event.kind == MapGeometryEventKind::FULL_MAP) {
        ambiguous_full_stamps_.insert(event.stamp_ns);
      } else {
        ambiguous_update_stamps_.insert(event.stamp_ns);
      }
    }
    drain();
    enforceQueueBounds();
    return revision_ != previous_revision;
  }

  const std::optional<nav_msgs::msg::OccupancyGrid> & map() const
  {
    return map_;
  }

  bool ready() const
  {
    return map_.has_value() && delta_chain_valid_ && pending_full_maps_.empty() &&
           pending_full_events_.empty() && pending_updates_.empty() &&
           pending_update_events_.empty() && ambiguous_full_stamps_.empty() &&
           ambiguous_update_stamps_.empty();
  }

  bool hasPendingMessages() const
  {
    return !pending_full_maps_.empty() || !pending_full_events_.empty() ||
           !pending_updates_.empty() || !pending_update_events_.empty() ||
           !ambiguous_full_stamps_.empty() || !ambiguous_update_stamps_.empty();
  }

  bool synchronizationPending() const
  {
    return map_traffic_seen_ && !ready();
  }

  std::uint64_t revision() const
  {
    return revision_;
  }

private:
  enum class UpdateDisposition
  {
    APPLIED,
    STALE,
    WAIT_FOR_FULL_MAP,
    INVALID
  };

  static bool validMap(const nav_msgs::msg::OccupancyGrid & map)
  {
    return map.header.stamp.sec >= 0 && map.header.stamp.nanosec < 1000000000U &&
           map.info.width > 0 && map.info.height > 0 &&
           std::isfinite(map.info.resolution) && map.info.resolution > 0.0F &&
           map.data.size() == static_cast<std::size_t>(map.info.width) * map.info.height;
  }

  static bool sameGeometry(
    const nav_msgs::msg::OccupancyGrid & lhs,
    const nav_msgs::msg::OccupancyGrid & rhs)
  {
    return lhs.header.frame_id == rhs.header.frame_id &&
           lhs.info.width == rhs.info.width && lhs.info.height == rhs.info.height &&
           mapGeometryValueBits(lhs.info.resolution) ==
           mapGeometryValueBits(rhs.info.resolution) &&
           mapOriginBits(lhs) == mapOriginBits(rhs);
  }

  static bool sameStamp(
    const builtin_interfaces::msg::Time & lhs,
    const builtin_interfaces::msg::Time & rhs)
  {
    return lhs.sec == rhs.sec && lhs.nanosec == rhs.nanosec;
  }

  static bool sameOccupancyGrid(
    const nav_msgs::msg::OccupancyGrid & lhs,
    const nav_msgs::msg::OccupancyGrid & rhs)
  {
    return lhs.header.frame_id == rhs.header.frame_id &&
           sameStamp(lhs.header.stamp, rhs.header.stamp) && sameGeometry(lhs, rhs) &&
           lhs.data == rhs.data;
  }

  static bool sameUpdate(
    const map_msgs::msg::OccupancyGridUpdate & lhs,
    const map_msgs::msg::OccupancyGridUpdate & rhs)
  {
    return lhs.header.frame_id == rhs.header.frame_id &&
           sameStamp(lhs.header.stamp, rhs.header.stamp) && lhs.x == rhs.x && lhs.y == rhs.y &&
           lhs.width == rhs.width && lhs.height == rhs.height && lhs.data == rhs.data;
  }

  static bool sameGeometryEvent(const MapGeometryEvent & lhs, const MapGeometryEvent & rhs)
  {
    return serializeMapGeometryEvent(lhs).data == serializeMapGeometryEvent(rhs).data;
  }

  void rememberCoveredUpdateEvent(const MapGeometryEvent & event)
  {
    covered_update_events_.insert_or_assign(event.stamp_ns, event);
  }

  void quarantineUpdate(const map_msgs::msg::OccupancyGridUpdate & update)
  {
    const auto stamp_ns = mapStampNanoseconds(update.header.stamp);
    const auto [entry, inserted] = quarantined_updates_.emplace(stamp_ns, update);
    if (!inserted && !sameUpdate(entry->second, update)) {
      quarantined_updates_.erase(entry);
    }
  }

  void restoreQuarantinedUpdate(const std::uint64_t stamp_ns)
  {
    const auto quarantined = quarantined_updates_.find(stamp_ns);
    if (quarantined == quarantined_updates_.end()) return;
    const auto [entry, inserted] = pending_updates_.emplace(stamp_ns, quarantined->second);
    if (!inserted && !sameUpdate(entry->second, quarantined->second)) {
      pending_updates_.erase(entry);
      ambiguous_update_stamps_.insert(stamp_ns);
      invalidateDeltaChain();
    }
    quarantined_updates_.erase(quarantined);
  }

  void invalidateDeltaChain()
  {
    delta_chain_valid_ = false;
  }

  void drain()
  {
    processFullMapPairs();
    processUpdatePairs();
    discardStaleFullEvents();
  }

  void processFullMapPairs()
  {
    for (auto ambiguous = ambiguous_full_stamps_.begin();
      ambiguous != ambiguous_full_stamps_.end();)
    {
      const auto stamp_ns = *ambiguous;
      const bool has_map = pending_full_maps_.erase(stamp_ns) > 0;
      const bool has_event = pending_full_events_.erase(stamp_ns) > 0;
      if (has_map || has_event) {
        invalidateDeltaChain();
        ambiguous = ambiguous_full_stamps_.erase(ambiguous);
      } else {
        ++ambiguous;
      }
    }
    for (auto map_entry = pending_full_maps_.begin();
      map_entry != pending_full_maps_.end();)
    {
      const auto stamp_ns = map_entry->first;
      const auto event_entry = pending_full_events_.find(stamp_ns);
      if (event_entry == pending_full_events_.end()) {
        ++map_entry;
        continue;
      }
      if (ambiguous_full_stamps_.erase(stamp_ns) > 0) {
        invalidateDeltaChain();
      } else {
        processFullMapPair(map_entry->second, event_entry->second);
      }
      pending_full_events_.erase(event_entry);
      map_entry = pending_full_maps_.erase(map_entry);
      discardStaleFullEvents();
    }
  }

  void processFullMapPair(
    const nav_msgs::msg::OccupancyGrid & map, const MapGeometryEvent & event)
  {
    if (!validMap(map) || event.kind != MapGeometryEventKind::FULL_MAP ||
      event.stamp_ns != mapStampNanoseconds(map.header.stamp) ||
      !mapGeometryEventMatches(event, map))
    {
      invalidateDeltaChain();
      return;
    }
    if (map_ && (event.publisher_epoch < publisher_epoch_ ||
      (event.publisher_epoch == publisher_epoch_ &&
      event.geometry_version < geometry_version_)))
    {
      return;
    }

    const bool new_publisher = !map_ || event.publisher_epoch > publisher_epoch_;
    const bool new_geometry = new_publisher || event.geometry_version > geometry_version_;
    if (!new_geometry && !mapGeometryEventMatches(event, *map_)) {
      invalidateDeltaChain();
      return;
    }
    if (new_geometry) {
      const bool changed = !map_ || event.publisher_epoch != publisher_epoch_ ||
        event.geometry_version != geometry_version_ || !sameGeometry(*map_, map) ||
        map_->data != map.data;
      map_ = map;
      publisher_epoch_ = event.publisher_epoch;
      geometry_version_ = event.geometry_version;
      latest_full_stamp_ns_ = event.stamp_ns;
      latest_state_stamp_ns_ = event.stamp_ns;
      delta_chain_valid_ = true;
      if (changed) ++revision_;
      discardUpdatesCoveredByFullMap(event.publisher_epoch, event.geometry_version,
        event.stamp_ns);
      return;
    }

    if (event.stamp_ns <= latest_full_stamp_ns_) return;
    latest_full_stamp_ns_ = event.stamp_ns;
    if (event.stamp_ns < latest_state_stamp_ns_) return;
    if (event.stamp_ns == latest_state_stamp_ns_ && map_->data != map.data) {
      invalidateDeltaChain();
      return;
    }
    const bool changed = map_->data != map.data;
    map_ = map;
    latest_state_stamp_ns_ = event.stamp_ns;
    delta_chain_valid_ = true;
    if (changed) ++revision_;
    discardUpdatesCoveredByFullMap(event.publisher_epoch, event.geometry_version,
      event.stamp_ns);
  }

  void processUpdatePairs()
  {
    for (auto ambiguous = ambiguous_update_stamps_.begin();
      ambiguous != ambiguous_update_stamps_.end();)
    {
      const auto stamp_ns = *ambiguous;
      const bool has_update = pending_updates_.erase(stamp_ns) > 0;
      const bool has_event = pending_update_events_.erase(stamp_ns) > 0;
      if (has_update || has_event) {
        invalidateDeltaChain();
        ambiguous = ambiguous_update_stamps_.erase(ambiguous);
      } else {
        ++ambiguous;
      }
    }
    while (!pending_updates_.empty() || !pending_update_events_.empty()) {
      std::uint64_t stamp_ns = 0;
      if (pending_updates_.empty()) {
        stamp_ns = pending_update_events_.begin()->first;
      } else if (pending_update_events_.empty()) {
        stamp_ns = pending_updates_.begin()->first;
      } else {
        stamp_ns = std::min(
          pending_updates_.begin()->first, pending_update_events_.begin()->first);
      }

      const auto update = pending_updates_.find(stamp_ns);
      const auto event = pending_update_events_.find(stamp_ns);
      if (ambiguous_update_stamps_.erase(stamp_ns) > 0) {
        pending_updates_.erase(stamp_ns);
        pending_update_events_.erase(stamp_ns);
        invalidateDeltaChain();
        break;
      }
      if (update == pending_updates_.end() || event == pending_update_events_.end()) {
        if (event != pending_update_events_.end() && isStale(event->second)) {
          rememberCoveredUpdateEvent(event->second);
          pending_updates_.erase(stamp_ns);
          pending_update_events_.erase(event);
          continue;
        }
        break;
      }

      const auto disposition = processUpdatePair(update->second, event->second);
      if (disposition == UpdateDisposition::WAIT_FOR_FULL_MAP) break;
      if (disposition == UpdateDisposition::STALE) {
        rememberCoveredUpdateEvent(event->second);
      }
      pending_updates_.erase(update);
      pending_update_events_.erase(event);
      if (disposition == UpdateDisposition::INVALID) break;
    }
  }

  bool isStale(const MapGeometryEvent & event) const
  {
    return map_ && (event.publisher_epoch < publisher_epoch_ ||
      (event.publisher_epoch == publisher_epoch_ &&
      (event.geometry_version < geometry_version_ ||
      (event.geometry_version == geometry_version_ &&
      event.stamp_ns <= latest_state_stamp_ns_))));
  }

  UpdateDisposition processUpdatePair(
    const map_msgs::msg::OccupancyGridUpdate & update, const MapGeometryEvent & event)
  {
    if (update.header.stamp.sec < 0 || update.header.stamp.nanosec >= 1000000000U ||
      event.kind != MapGeometryEventKind::INCREMENTAL_UPDATE ||
      event.stamp_ns != mapStampNanoseconds(update.header.stamp) ||
      event.publisher_epoch == 0 || event.geometry_version == 0)
    {
      invalidateDeltaChain();
      return UpdateDisposition::INVALID;
    }
    if (map_ && (event.publisher_epoch < publisher_epoch_ ||
      (event.publisher_epoch == publisher_epoch_ &&
      (event.geometry_version < geometry_version_ ||
      (event.geometry_version == geometry_version_ &&
      event.stamp_ns <= latest_state_stamp_ns_)))))
    {
      return UpdateDisposition::STALE;
    }
    if (!map_ || event.publisher_epoch > publisher_epoch_ ||
      event.geometry_version > geometry_version_)
    {
      return UpdateDisposition::WAIT_FOR_FULL_MAP;
    }
    if (event.publisher_epoch != publisher_epoch_ ||
      event.geometry_version != geometry_version_ ||
      !mapGeometryEventMatches(event, *map_) || update.header.frame_id != map_->header.frame_id)
    {
      invalidateDeltaChain();
      return UpdateDisposition::INVALID;
    }
    if (!delta_chain_valid_) return UpdateDisposition::WAIT_FOR_FULL_MAP;
    if (update.width == 0 || update.height == 0 || update.x > map_->info.width ||
      update.y > map_->info.height || update.width > map_->info.width - update.x ||
      update.height > map_->info.height - update.y ||
      update.data.size() != static_cast<std::size_t>(update.width) * update.height)
    {
      invalidateDeltaChain();
      return UpdateDisposition::INVALID;
    }

    bool changed = false;
    for (std::uint32_t y = 0; y < update.height; ++y) {
      for (std::uint32_t x = 0; x < update.width; ++x) {
        const auto source = static_cast<std::size_t>(y) * update.width + x;
        const auto target = static_cast<std::size_t>(update.y + y) * map_->info.width +
          update.x + x;
        if (map_->data[target] != update.data[source]) {
          map_->data[target] = update.data[source];
          changed = true;
        }
      }
    }
    map_->header.stamp = update.header.stamp;
    latest_state_stamp_ns_ = event.stamp_ns;
    if (changed) ++revision_;
    return UpdateDisposition::APPLIED;
  }

  void discardUpdatesCoveredByFullMap(
    const std::uint64_t epoch, const std::uint64_t version, const std::uint64_t stamp_ns)
  {
    for (auto entry = pending_updates_.begin(); entry != pending_updates_.end();) {
      const auto event = pending_update_events_.find(entry->first);
      const bool covered = event != pending_update_events_.end() &&
        (event->second.publisher_epoch < epoch ||
        (event->second.publisher_epoch == epoch &&
        (event->second.geometry_version < version ||
        (event->second.geometry_version == version && entry->first <= stamp_ns))));
      if (covered) {
        if (event != pending_update_events_.end()) {
          rememberCoveredUpdateEvent(event->second);
        }
        pending_update_events_.erase(entry->first);
        ambiguous_update_stamps_.erase(entry->first);
        entry = pending_updates_.erase(entry);
      } else {
        ++entry;
      }
    }
    for (auto entry = pending_update_events_.begin();
      entry != pending_update_events_.end();)
    {
      const bool covered = entry->second.publisher_epoch < epoch ||
        (entry->second.publisher_epoch == epoch &&
        (entry->second.geometry_version < version ||
        (entry->second.geometry_version == version && entry->first <= stamp_ns)));
      if (covered) {
        rememberCoveredUpdateEvent(entry->second);
        pending_updates_.erase(entry->first);
        ambiguous_update_stamps_.erase(entry->first);
        entry = pending_update_events_.erase(entry);
      } else {
        ++entry;
      }
    }
  }

  void discardStaleFullEvents()
  {
    if (!map_) return;
    for (auto entry = pending_full_events_.begin(); entry != pending_full_events_.end();) {
      const auto & event = entry->second;
      const bool stale = event.publisher_epoch < publisher_epoch_ ||
        (event.publisher_epoch == publisher_epoch_ &&
        (event.geometry_version < geometry_version_ ||
        (event.geometry_version == geometry_version_ &&
        event.stamp_ns <= latest_full_stamp_ns_)));
      if (stale) {
        pending_full_maps_.erase(entry->first);
        ambiguous_full_stamps_.erase(entry->first);
        entry = pending_full_events_.erase(entry);
      } else {
        ++entry;
      }
    }
  }

  void enforceQueueBounds()
  {
    const bool required_queue_overflow =
      pending_full_maps_.size() > kMaximumPendingFullMaps ||
      pending_full_events_.size() > kMaximumPendingEvents ||
      pending_updates_.size() > kMaximumPendingUpdates ||
      pending_update_events_.size() > kMaximumPendingEvents ||
      ambiguous_full_stamps_.size() > kMaximumPendingEvents ||
      ambiguous_update_stamps_.size() > kMaximumPendingEvents;
    if (!required_queue_overflow) {
      // These are bounded history/candidate caches. Eviction is safe until a
      // later unmatched event proves that a discarded body was required.
      while (covered_update_events_.size() > kMaximumCoveredUpdateEvents) {
        covered_update_events_.erase(covered_update_events_.begin());
      }
      while (quarantined_updates_.size() > kMaximumPendingUpdates) {
        quarantined_updates_.erase(quarantined_updates_.begin());
      }
      return;
    }
    invalidateDeltaChain();
    while (pending_full_maps_.size() > kMaximumPendingFullMaps) {
      const auto stamp_ns = pending_full_maps_.begin()->first;
      pending_full_maps_.erase(pending_full_maps_.begin());
      pending_full_events_.erase(stamp_ns);
      ambiguous_full_stamps_.erase(stamp_ns);
    }
    while (pending_full_events_.size() > kMaximumPendingEvents) {
      const auto stamp_ns = pending_full_events_.begin()->first;
      pending_full_events_.erase(pending_full_events_.begin());
      pending_full_maps_.erase(stamp_ns);
      ambiguous_full_stamps_.erase(stamp_ns);
    }
    while (pending_updates_.size() > kMaximumPendingUpdates) {
      const auto stamp_ns = pending_updates_.begin()->first;
      pending_updates_.erase(pending_updates_.begin());
      pending_update_events_.erase(stamp_ns);
      ambiguous_update_stamps_.erase(stamp_ns);
    }
    while (pending_update_events_.size() > kMaximumPendingEvents) {
      const auto stamp_ns = pending_update_events_.begin()->first;
      pending_update_events_.erase(pending_update_events_.begin());
      pending_updates_.erase(stamp_ns);
      ambiguous_update_stamps_.erase(stamp_ns);
    }
    while (quarantined_updates_.size() > kMaximumPendingUpdates) {
      quarantined_updates_.erase(quarantined_updates_.begin());
    }
    while (covered_update_events_.size() > kMaximumCoveredUpdateEvents) {
      covered_update_events_.erase(covered_update_events_.begin());
    }
    while (ambiguous_full_stamps_.size() > kMaximumPendingEvents) {
      ambiguous_full_stamps_.erase(ambiguous_full_stamps_.begin());
    }
    while (ambiguous_update_stamps_.size() > kMaximumPendingEvents) {
      ambiguous_update_stamps_.erase(ambiguous_update_stamps_.begin());
    }
  }

  std::optional<nav_msgs::msg::OccupancyGrid> map_;
  std::map<std::uint64_t, nav_msgs::msg::OccupancyGrid> pending_full_maps_;
  std::map<std::uint64_t, MapGeometryEvent> pending_full_events_;
  std::map<std::uint64_t, map_msgs::msg::OccupancyGridUpdate> pending_updates_;
  std::map<std::uint64_t, map_msgs::msg::OccupancyGridUpdate> quarantined_updates_;
  std::map<std::uint64_t, MapGeometryEvent> pending_update_events_;
  std::map<std::uint64_t, MapGeometryEvent> covered_update_events_;
  std::set<std::uint64_t> ambiguous_full_stamps_;
  std::set<std::uint64_t> ambiguous_update_stamps_;
  std::uint64_t publisher_epoch_{0};
  std::uint64_t geometry_version_{0};
  std::uint64_t latest_full_stamp_ns_{0};
  std::uint64_t latest_state_stamp_ns_{0};
  std::uint64_t revision_{0};
  bool delta_chain_valid_{false};
  bool map_traffic_seen_{false};
};

}  // namespace bringup

#endif  // BRINGUP__ONLINE_MAP_STATE_HPP_
