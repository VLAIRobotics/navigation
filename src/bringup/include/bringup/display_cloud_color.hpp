#ifndef BRINGUP__DISPLAY_CLOUD_COLOR_HPP_
#define BRINGUP__DISPLAY_CLOUD_COLOR_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>

namespace bringup
{

inline std::array<std::uint8_t, 3> heightDisplayColor(float height_above_ground)
{
  constexpr std::array<std::array<std::uint8_t, 3>, 5> colors{{
      {255, 0, 0}, {255, 255, 0}, {0, 255, 0}, {0, 255, 255}, {0, 0, 255}}};
  const float height = std::isfinite(height_above_ground) ? height_above_ground : 0.0F;
  const float normalized = std::clamp(height / 1.8F, 0.0F, 1.0F);
  const float scaled = normalized * 4.0F;
  const auto segment = std::min(static_cast<std::size_t>(scaled), colors.size() - 2);
  const float ratio = scaled - static_cast<float>(segment);
  std::array<std::uint8_t, 3> result{};
  for (std::size_t i = 0; i < result.size(); ++i) {
    result[i] = static_cast<std::uint8_t>(std::lround(
      colors[segment][i] + ratio * (colors[segment + 1][i] - colors[segment][i])));
  }
  return result;
}

}  // namespace bringup

#endif  // BRINGUP__DISPLAY_CLOUD_COLOR_HPP_
