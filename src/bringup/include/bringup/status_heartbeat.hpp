#pragma once

#include <chrono>
#include <cmath>

namespace bringup
{

inline constexpr std::chrono::milliseconds kNavigationReadyHeartbeatPeriod{250};
inline constexpr double kNavigationReadyHeartbeatTimeoutSec = 1.0;

inline bool localizationStatusHeartbeatIsFresh(
  bool received, double age_sec, double timeout_sec)
{
  return received && std::isfinite(age_sec) && age_sec >= 0.0 &&
         std::isfinite(timeout_sec) && timeout_sec > 0.0 && age_sec <= timeout_sec;
}

inline bool navigationReadyHeartbeatIsFresh(bool received, double age_sec)
{
  return localizationStatusHeartbeatIsFresh(
    received, age_sec, kNavigationReadyHeartbeatTimeoutSec);
}

}  // namespace bringup
