#pragma once

#include <cstddef>

namespace small_gicp_relocalization
{

// Hitting the iteration cap is a usable optimizer termination signal only when
// the caller also applies geometric, coverage, multi-start, and temporal gates.
inline bool registrationTerminationAccepted(
  const bool converged, const std::size_t iterations_completed,
  const std::size_t max_iterations)
{
  return converged || (max_iterations > 0U && iterations_completed >= max_iterations);
}

}  // namespace small_gicp_relocalization
