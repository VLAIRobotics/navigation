#pragma once

#include <cstdint>

namespace fastdem::ros2 {

inline bool postprocessSnapshotCanPublish(
    std::uint64_t snapshot_reset_generation,
    std::uint64_t current_reset_generation,
    bool clock_rollback_pending) noexcept {
  return !clock_rollback_pending &&
         snapshot_reset_generation == current_reset_generation;
}

// The caller serializes these operations with its publication mutex.
class PostprocessPublicationOrder {
 public:
  bool accepts(std::uint64_t source_stamp) const noexcept {
    return !has_published_ || source_stamp >= last_source_stamp_;
  }

  bool canClearStaleSnapshot(
      std::uint64_t source_stamp,
      std::uint64_t snapshot_map_generation,
      std::uint64_t current_map_generation) const noexcept {
    return accepts(source_stamp) &&
           snapshot_map_generation == current_map_generation;
  }

  void recordPublished(std::uint64_t source_stamp) noexcept {
    last_source_stamp_ = source_stamp;
    has_published_ = true;
  }

  void reset() noexcept {
    last_source_stamp_ = 0;
    has_published_ = false;
  }

 private:
  std::uint64_t last_source_stamp_{0};
  bool has_published_{false};
};

}  // namespace fastdem::ros2
