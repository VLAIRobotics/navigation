#ifndef BRINGUP__MAP_GEOMETRY_EVENT_HPP_
#define BRINGUP__MAP_GEOMETRY_EVENT_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <string>
#include <vector>

#include <builtin_interfaces/msg/time.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <std_msgs/msg/u_int64_multi_array.hpp>

namespace bringup
{

enum class MapGeometryEventKind : std::uint64_t
{
  FULL_MAP = 0,
  INCREMENTAL_UPDATE = 1
};

struct MapGeometryEvent
{
  MapGeometryEventKind kind{MapGeometryEventKind::FULL_MAP};
  std::uint64_t stamp_ns{0};
  std::uint64_t publisher_epoch{0};
  std::uint64_t geometry_version{0};
  std::uint32_t width{0};
  std::uint32_t height{0};
  std::uint32_t resolution_bits{0};
  std::array<std::uint64_t, 7> origin_bits{};
  std::uint64_t frame_hash{0};
};

inline std::uint64_t mapStampNanoseconds(const builtin_interfaces::msg::Time & stamp)
{
  if (stamp.sec < 0) return 0;
  return static_cast<std::uint64_t>(stamp.sec) * 1000000000ULL + stamp.nanosec;
}

template<typename T>
inline std::uint64_t mapGeometryValueBits(const T & value)
{
  static_assert(sizeof(T) <= sizeof(std::uint64_t));
  std::uint64_t bits = 0;
  std::memcpy(&bits, &value, sizeof(T));
  return bits;
}

inline std::uint64_t mapGeometryFrameHash(const std::string & frame)
{
  std::uint64_t hash = 14695981039346656037ULL;
  for (const unsigned char character : frame) {
    hash ^= character;
    hash *= 1099511628211ULL;
  }
  return hash;
}

inline std::array<std::uint64_t, 7> mapOriginBits(const nav_msgs::msg::OccupancyGrid & map)
{
  const auto & origin = map.info.origin;
  return {
    mapGeometryValueBits(origin.position.x), mapGeometryValueBits(origin.position.y),
    mapGeometryValueBits(origin.position.z), mapGeometryValueBits(origin.orientation.x),
    mapGeometryValueBits(origin.orientation.y), mapGeometryValueBits(origin.orientation.z),
    mapGeometryValueBits(origin.orientation.w)};
}

inline MapGeometryEvent makeMapGeometryEvent(
  const nav_msgs::msg::OccupancyGrid & map,
  const builtin_interfaces::msg::Time & stamp,
  std::uint64_t publisher_epoch, std::uint64_t geometry_version,
  MapGeometryEventKind kind)
{
  MapGeometryEvent event;
  event.kind = kind;
  event.stamp_ns = mapStampNanoseconds(stamp);
  event.publisher_epoch = publisher_epoch;
  event.geometry_version = geometry_version;
  event.width = map.info.width;
  event.height = map.info.height;
  event.resolution_bits = static_cast<std::uint32_t>(
    mapGeometryValueBits(map.info.resolution));
  event.origin_bits = mapOriginBits(map);
  event.frame_hash = mapGeometryFrameHash(map.header.frame_id);
  return event;
}

inline std_msgs::msg::UInt64MultiArray serializeMapGeometryEvent(
  const MapGeometryEvent & event)
{
  std_msgs::msg::UInt64MultiArray message;
  message.data.reserve(15);
  message.data.push_back(static_cast<std::uint64_t>(event.kind));
  message.data.push_back(event.stamp_ns);
  message.data.push_back(event.publisher_epoch);
  message.data.push_back(event.geometry_version);
  message.data.push_back(event.width);
  message.data.push_back(event.height);
  message.data.push_back(event.resolution_bits);
  message.data.insert(message.data.end(), event.origin_bits.begin(), event.origin_bits.end());
  message.data.push_back(event.frame_hash);
  return message;
}

inline bool parseMapGeometryEvent(
  const std_msgs::msg::UInt64MultiArray & message, MapGeometryEvent & event)
{
  if (message.data.size() != 15 || message.data[0] >
    static_cast<std::uint64_t>(MapGeometryEventKind::INCREMENTAL_UPDATE))
  {
    return false;
  }
  event.kind = static_cast<MapGeometryEventKind>(message.data[0]);
  event.stamp_ns = message.data[1];
  event.publisher_epoch = message.data[2];
  event.geometry_version = message.data[3];
  if (event.stamp_ns == 0 || event.publisher_epoch == 0 || event.geometry_version == 0 ||
    message.data[4] == 0 || message.data[4] > std::numeric_limits<std::uint32_t>::max() ||
    message.data[5] == 0 || message.data[5] > std::numeric_limits<std::uint32_t>::max() ||
    message.data[6] > std::numeric_limits<std::uint32_t>::max())
  {
    return false;
  }
  event.width = static_cast<std::uint32_t>(message.data[4]);
  event.height = static_cast<std::uint32_t>(message.data[5]);
  event.resolution_bits = static_cast<std::uint32_t>(message.data[6]);
  for (std::size_t index = 0; index < event.origin_bits.size(); ++index) {
    event.origin_bits[index] = message.data[7 + index];
  }
  event.frame_hash = message.data[14];
  return true;
}

inline bool mapGeometryEventMatches(
  const MapGeometryEvent & event, const nav_msgs::msg::OccupancyGrid & map)
{
  return event.width == map.info.width && event.height == map.info.height &&
         event.resolution_bits == static_cast<std::uint32_t>(
           mapGeometryValueBits(map.info.resolution)) &&
         event.origin_bits == mapOriginBits(map) &&
         event.frame_hash == mapGeometryFrameHash(map.header.frame_id);
}

}  // namespace bringup

#endif  // BRINGUP__MAP_GEOMETRY_EVENT_HPP_
