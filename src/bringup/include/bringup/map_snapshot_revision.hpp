#ifndef BRINGUP__MAP_SNAPSHOT_REVISION_HPP_
#define BRINGUP__MAP_SNAPSHOT_REVISION_HPP_

#include <cstdint>
#include <limits>
#include <stdexcept>

#include <builtin_interfaces/msg/time.hpp>

namespace bringup
{

inline builtin_interfaces::msg::Time nextMapSnapshotRevision(
  const builtin_interfaces::msg::Time & previous,
  const builtin_interfaces::msg::Time & candidate)
{
  if (candidate.sec < 0 || candidate.nanosec >= 1000000000U) {
    throw std::invalid_argument("map snapshot candidate revision is invalid");
  }
  if (candidate.sec > previous.sec ||
    (candidate.sec == previous.sec && candidate.nanosec > previous.nanosec))
  {
    return candidate;
  }

  auto revision = previous;
  if (revision.nanosec < 999999999U) {
    ++revision.nanosec;
  } else {
    if (revision.sec == std::numeric_limits<std::int32_t>::max()) {
      throw std::overflow_error("map snapshot revision space is exhausted");
    }
    ++revision.sec;
    revision.nanosec = 0U;
  }
  return revision;
}

inline builtin_interfaces::msg::Time mapSnapshotTimeFromNanoseconds(
  const std::int64_t stamp_nanoseconds)
{
  constexpr std::int64_t kNanosecondsPerSecond = 1000000000LL;
  if (stamp_nanoseconds < 0) {
    throw std::invalid_argument("map snapshot time cannot be negative");
  }
  const auto seconds = stamp_nanoseconds / kNanosecondsPerSecond;
  if (seconds > std::numeric_limits<std::int32_t>::max()) {
    throw std::overflow_error("map snapshot time exceeds builtin_interfaces/Time range");
  }
  builtin_interfaces::msg::Time stamp;
  stamp.sec = static_cast<std::int32_t>(seconds);
  stamp.nanosec = static_cast<std::uint32_t>(stamp_nanoseconds % kNanosecondsPerSecond);
  return stamp;
}

inline bool sameMapSnapshotRevision(
  const builtin_interfaces::msg::Time & map,
  const builtin_interfaces::msg::Time & provenance,
  const builtin_interfaces::msg::Time & occupancy_source,
  const builtin_interfaces::msg::Time & terrain)
{
  if (map.sec < 0 || map.nanosec >= 1000000000U ||
    (map.sec == 0 && map.nanosec == 0U))
  {
    return false;
  }
  const auto matches_map = [&map](const builtin_interfaces::msg::Time & stamp) {
      return stamp.sec == map.sec && stamp.nanosec == map.nanosec;
    };
  return matches_map(provenance) && matches_map(occupancy_source) && matches_map(terrain);
}

}  // namespace bringup

#endif  // BRINGUP__MAP_SNAPSHOT_REVISION_HPP_
