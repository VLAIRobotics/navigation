#ifndef BRINGUP__FRESH_RETRY_QUEUE_HPP_
#define BRINGUP__FRESH_RETRY_QUEUE_HPP_

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <stdexcept>
#include <utility>
#include <vector>

namespace bringup
{
template<typename Payload>
class FreshRetryQueue
{
public:
  using Clock = std::chrono::steady_clock;

  struct Entry
  {
    Payload payload;
    std::int64_t source_stamp_ns{0};
    Clock::time_point received_at;
  };

  enum class EnqueueResult
  {
    ACCEPTED,
    DROPPED_OLDEST,
    REJECTED_EXPIRED
  };

  FreshRetryQueue(std::size_t capacity, Clock::duration max_age,
    bool track_discarded_payloads = false)
  : capacity_(capacity), max_age_(max_age), track_discarded_payloads_(track_discarded_payloads)
  {
    if (capacity_ == 0 || max_age_ <= Clock::duration::zero()) {
      throw std::invalid_argument("fresh retry queue bounds must be positive");
    }
  }

  EnqueueResult enqueue(
    Payload payload, std::int64_t source_stamp_ns, std::int64_t source_now_ns,
    Clock::time_point received_at = Clock::now())
  {
    if (source_stamp_ns <= 0 || sourceTimestampExpired(source_stamp_ns, source_now_ns)) {
      return EnqueueResult::REJECTED_EXPIRED;
    }
    pruneExpired(received_at, source_now_ns);
    const bool dropped_oldest = entries_.size() >= capacity_;
    if (dropped_oldest) {
      discarded_source_stamp_ns_ = std::max(
        discarded_source_stamp_ns_, entries_.front().source_stamp_ns);
      if (track_discarded_payloads_) {
        discarded_payloads_.push_back(std::move(entries_.front().payload));
      }
      entries_.pop_front();
    }
    entries_.push_back({std::move(payload), source_stamp_ns, received_at});
    return dropped_oldest ? EnqueueResult::DROPPED_OLDEST : EnqueueResult::ACCEPTED;
  }

  template<typename Processor>
  std::size_t retryAvailable(
    Clock::time_point now, std::int64_t source_now_ns, Processor processor)
  {
    pruneExpired(now, source_now_ns);
    std::size_t processed = 0;
    for (auto it = entries_.begin(); it != entries_.end();) {
      if (processor(it->payload, it->source_stamp_ns, it->received_at)) {
        it = entries_.erase(it);
        ++processed;
      } else {
        ++it;
      }
    }
    return processed;
  }

  template<typename Processor>
  std::size_t retryAvailableNewestFirst(
    Clock::time_point now, std::int64_t source_now_ns, Processor processor)
  {
    std::stable_sort(entries_.begin(), entries_.end(), [](const Entry & lhs, const Entry & rhs) {
      if (lhs.source_stamp_ns != rhs.source_stamp_ns) {
        return lhs.source_stamp_ns > rhs.source_stamp_ns;
      }
      return lhs.received_at > rhs.received_at;
    });
    const auto processed = retryAvailable(now, source_now_ns, processor);
    // enqueue() removes entries_.front() when full, so restore oldest-first storage
    // after this priority pass to keep capacity eviction conservative.
    std::stable_sort(entries_.begin(), entries_.end(), [](const Entry & lhs, const Entry & rhs) {
      if (lhs.source_stamp_ns != rhs.source_stamp_ns) {
        return lhs.source_stamp_ns < rhs.source_stamp_ns;
      }
      return lhs.received_at < rhs.received_at;
    });
    return processed;
  }

  std::size_t size() const
  {
    return entries_.size();
  }

  bool empty() const
  {
    return entries_.empty();
  }

  bool hasSourceStampAtOrAfter(std::int64_t source_stamp_ns) const
  {
    for (const auto & entry : entries_) {
      if (entry.source_stamp_ns >= source_stamp_ns) return true;
    }
    return false;
  }

  // Return the newest source timestamp discarded by expiry or capacity pressure.
  std::int64_t takeDiscardedSourceStampNs()
  {
    const auto discarded_source_stamp_ns = discarded_source_stamp_ns_;
    discarded_source_stamp_ns_ = 0;
    return discarded_source_stamp_ns;
  }

  std::vector<Payload> takeDiscardedPayloads()
  {
    std::vector<Payload> discarded;
    discarded.swap(discarded_payloads_);
    return discarded;
  }

private:
  std::int64_t maxAgeNs() const
  {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(max_age_).count();
  }

  bool sourceTimestampExpired(std::int64_t stamp_ns, std::int64_t now_ns) const
  {
    return now_ns > 0 && stamp_ns > 0 && now_ns >= stamp_ns &&
           now_ns - stamp_ns >= maxAgeNs();
  }

  void pruneExpired(Clock::time_point now, std::int64_t source_now_ns)
  {
    for (auto it = entries_.begin(); it != entries_.end();) {
      const bool receive_time_expired = now >= it->received_at &&
        now - it->received_at >= max_age_;
      if (receive_time_expired ||
        sourceTimestampExpired(it->source_stamp_ns, source_now_ns))
      {
        discarded_source_stamp_ns_ = std::max(
          discarded_source_stamp_ns_, it->source_stamp_ns);
        if (track_discarded_payloads_) {
          discarded_payloads_.push_back(std::move(it->payload));
        }
        it = entries_.erase(it);
      } else {
        ++it;
      }
    }
  }

  std::size_t capacity_;
  Clock::duration max_age_;
  std::deque<Entry> entries_;
  std::int64_t discarded_source_stamp_ns_{0};
  std::vector<Payload> discarded_payloads_;
  bool track_discarded_payloads_{false};
};
}  // namespace bringup

#endif  // BRINGUP__FRESH_RETRY_QUEUE_HPP_
