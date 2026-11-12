#pragma once

#include <cstdint>

namespace small_gicp_relocalization
{

inline bool localizedFailureKeepsValidity(
  std::uint32_t consecutive_failures, std::uint32_t max_consecutive_failures,
  double elapsed_since_success, double validity_timeout)
{
  return consecutive_failures < max_consecutive_failures &&
         elapsed_since_success <= validity_timeout;
}

}  // namespace small_gicp_relocalization
