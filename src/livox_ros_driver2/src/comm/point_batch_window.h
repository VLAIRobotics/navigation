#ifndef LIVOX_DRIVER_POINT_BATCH_WINDOW_H_
#define LIVOX_DRIVER_POINT_BATCH_WINDOW_H_

#include <chrono>

namespace livox_ros
{

class PointBatchWindow
{
public:
  using Clock = std::chrono::steady_clock;
  using TimePoint = Clock::time_point;
  using Duration = Clock::duration;

  void reset(const TimePoint now)
  {
    have_batch_ = false;
    first_packet_ = TimePoint();
    last_packet_ = TimePoint();
    last_publish_ = now;
    have_last_publish_ = true;
  }

  bool shouldResetBeforePacket(
    const TimePoint received_at, const TimePoint processed_at,
    const Duration maximum_packet_gap, const Duration maximum_batch_wait) const
  {
    if (!have_batch_) {
      return false;
    }
    if (received_at < last_packet_ || processed_at < first_packet_) {
      return true;
    }
    return received_at - last_packet_ > maximum_packet_gap ||
           processed_at - first_packet_ >= maximum_batch_wait;
  }

  void observePacket(const TimePoint received_at)
  {
    if (!have_batch_) {
      first_packet_ = received_at;
      last_packet_ = received_at;
      have_batch_ = true;
      if (!have_last_publish_) {
        last_publish_ = received_at;
        have_last_publish_ = true;
      }
      return;
    }
    last_packet_ = received_at;
  }

  bool canPublish(
    const TimePoint now, const Duration minimum_coverage,
    const Duration minimum_period, const Duration maximum_batch_wait) const
  {
    return have_batch_ && have_last_publish_ && now >= first_packet_ &&
           now >= last_publish_ && last_packet_ >= first_packet_ &&
           now - first_packet_ < maximum_batch_wait &&
           last_packet_ - first_packet_ >= minimum_coverage &&
           now - last_publish_ >= minimum_period;
  }

  void markPublished(const TimePoint now)
  {
    reset(now);
  }

private:
  TimePoint first_packet_;
  TimePoint last_packet_;
  TimePoint last_publish_;
  bool have_batch_{false};
  bool have_last_publish_{false};
};

}  // namespace livox_ros

#endif  // LIVOX_DRIVER_POINT_BATCH_WINDOW_H_
