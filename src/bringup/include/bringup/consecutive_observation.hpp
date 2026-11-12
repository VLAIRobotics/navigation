#ifndef BRINGUP__CONSECUTIVE_OBSERVATION_HPP_
#define BRINGUP__CONSECUTIVE_OBSERVATION_HPP_

#include <cstdint>
#include <limits>

namespace bringup
{

struct ConsecutiveObservation
{
  std::uint8_t count{0};
  std::uint64_t last_frame{0};
  std::int64_t last_stamp_ns{0};

  void reset()
  {
    count = 0;
    last_frame = 0;
    last_stamp_ns = 0;
  }

  std::uint8_t observe(
    std::uint64_t frame, std::int64_t stamp_ns, std::int64_t max_gap_ns)
  {
    const bool consecutive_frame =
      count > 0 && last_frame != std::numeric_limits<std::uint64_t>::max() &&
      frame == last_frame + 1;
    const bool consecutive_time = stamp_ns > 0 && last_stamp_ns > 0 &&
      stamp_ns > last_stamp_ns && stamp_ns - last_stamp_ns <= max_gap_ns;
    if (!consecutive_frame || !consecutive_time) {
      count = 1;
    } else if (count < std::numeric_limits<std::uint8_t>::max()) {
      ++count;
    }
    last_frame = frame;
    last_stamp_ns = stamp_ns;
    return count;
  }
};

struct TimeWindowObservation
{
  // Sixty intervals plus the current bucket retain up to 61 timestamp-aligned buckets.
  // For a 2.0 s ground window, ceil(2.0 s / 60) gives 33,333,334 ns buckets and a
  // maximum retained span of about 2.033 s. Observations in one bucket count once,
  // so this representation assumes ground evidence below roughly 30 Hz (currently 3.3 Hz).
  static constexpr std::uint64_t kWindowBucketCount = 60;
  static constexpr std::uint64_t kWindowMask = (std::uint64_t{1} << 61) - 1;

  std::uint64_t buckets{0};
  std::uint64_t last_bucket{0};
  bool initialized{false};

  std::uint8_t count() const noexcept
  {
    std::uint8_t result = 0;
    for (auto remaining = buckets; remaining != 0; remaining &= remaining - 1) {
      ++result;
    }
    return result;
  }

  void reset()
  {
    buckets = 0;
    last_bucket = 0;
    initialized = false;
  }

  std::uint8_t observe(std::int64_t stamp_ns, std::int64_t window_ns)
  {
    if (stamp_ns <= 0 || window_ns <= 0) {
      reset();
      return 0;
    }

    const auto bucket_count = static_cast<std::int64_t>(kWindowBucketCount);
    const auto bucket_width_ns = static_cast<std::uint64_t>(
      window_ns / bucket_count + (window_ns % bucket_count == 0 ? 0 : 1));
    const auto bucket = static_cast<std::uint64_t>(stamp_ns) / bucket_width_ns;
    if (!initialized || bucket < last_bucket) {
      buckets = 0;
      initialized = true;
    } else {
      const auto elapsed = bucket - last_bucket;
      if (elapsed > kWindowBucketCount) {
        buckets = 0;
      } else if (elapsed > 0) {
        buckets <<= elapsed;
      }
    }

    last_bucket = bucket;
    buckets = (buckets | 1U) & kWindowMask;
    return count();
  }
};

}  // namespace bringup

#endif  // BRINGUP__CONSECUTIVE_OBSERVATION_HPP_
