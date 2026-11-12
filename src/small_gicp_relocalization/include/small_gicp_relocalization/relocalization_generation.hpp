#pragma once

#include <cstdint>

namespace small_gicp_relocalization
{

inline bool resultBelongsToInitialization(
  std::uint64_t result_generation, std::uint64_t current_generation)
{
  return result_generation == current_generation;
}

inline bool resultBelongsToAppliedInitialization(
  std::uint64_t result_generation, std::uint64_t current_generation,
  std::uint64_t applied_generation)
{
  return result_generation != 0 && result_generation == current_generation &&
    result_generation == applied_generation;
}

}  // namespace small_gicp_relocalization
