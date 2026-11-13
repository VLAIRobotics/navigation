#pragma once

#include <cmath>

namespace small_gicp_relocalization
{

enum class ManualSeedDeadlineState
{
  INACTIVE,
  WAITING_FOR_INPUT,
  VALIDATING,
  EXPIRED,
};

inline ManualSeedDeadlineState manualSeedDeadlineState(
  bool active, bool has_usable_cloud,
  double elapsed_since_received_sec, double elapsed_since_usable_cloud_sec,
  double input_timeout_sec, double validation_timeout_sec)
{
  if (!active) return ManualSeedDeadlineState::INACTIVE;
  if (!has_usable_cloud) {
    if (!std::isfinite(elapsed_since_received_sec) ||
      !std::isfinite(input_timeout_sec) || elapsed_since_received_sec >= input_timeout_sec)
    {
      return ManualSeedDeadlineState::EXPIRED;
    }
    return ManualSeedDeadlineState::WAITING_FOR_INPUT;
  }
  if (!std::isfinite(elapsed_since_usable_cloud_sec) ||
    !std::isfinite(validation_timeout_sec) ||
    elapsed_since_usable_cloud_sec >= validation_timeout_sec)
  {
    return ManualSeedDeadlineState::EXPIRED;
  }
  return ManualSeedDeadlineState::VALIDATING;
}

}  // namespace small_gicp_relocalization
