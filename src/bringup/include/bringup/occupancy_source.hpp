#ifndef BRINGUP__OCCUPANCY_SOURCE_HPP_
#define BRINGUP__OCCUPANCY_SOURCE_HPP_

#include <cstdint>
#include <optional>

namespace bringup
{

struct OccupancySourceFlags
{
  bool prior;
  bool online;
};

inline std::optional<std::uint8_t> decodeOccupancySourcePixel(const std::uint8_t pixel)
{
  if (pixel == 0U) return 0U;
  if (pixel == 85U) return 1U;
  if (pixel == 170U) return 2U;
  if (pixel == 255U) return 3U;
  return std::nullopt;
}

inline std::optional<OccupancySourceFlags> restoreOccupancySourceFlags(
  const std::uint8_t mask, const std::int8_t occupancy)
{
  if (mask > 3U) return std::nullopt;
  return OccupancySourceFlags{
    (mask & 1U) != 0U,
    (mask & 2U) != 0U && occupancy >= 65};
}

}  // namespace bringup

#endif  // BRINGUP__OCCUPANCY_SOURCE_HPP_
