#pragma once

#include <cmath>

namespace small_gicp_relocalization
{

inline bool fastlioHealthIsUsable(
  bool has_health, bool healthy, double age_sec, double timeout_sec)
{
  return has_health && healthy && std::isfinite(age_sec) && age_sec >= 0.0 &&
         std::isfinite(timeout_sec) && timeout_sec >= 0.0 && age_sec <= timeout_sec;
}

}  // namespace small_gicp_relocalization
