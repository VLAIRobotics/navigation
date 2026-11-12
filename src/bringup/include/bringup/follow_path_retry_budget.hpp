#ifndef BRINGUP__FOLLOW_PATH_RETRY_BUDGET_HPP_
#define BRINGUP__FOLLOW_PATH_RETRY_BUDGET_HPP_

#include <chrono>
#include <cstdint>

namespace bringup
{

class FollowPathRetryBudget
{
public:
  using Clock = std::chrono::steady_clock;

  void recordFailure(Clock::time_point now)
  {
    if (!active_) {
      started_ = now;
      active_ = true;
    }
  }

  bool releaseOnProgress(
    bool waiting, std::uint64_t active_attempt, std::uint64_t progress_attempt,
    Clock::time_point now, Clock::duration budget)
  {
    if (!active_ || expired(now, budget) || waiting || active_attempt == 0 ||
      active_attempt != progress_attempt)
    {
      return false;
    }
    active_ = false;
    return true;
  }

  bool expired(Clock::time_point now, Clock::duration budget) const
  {
    return active_ && now - started_ >= budget;
  }

  Clock::duration remaining(Clock::time_point now, Clock::duration budget) const
  {
    if (!active_ || expired(now, budget)) {
      return Clock::duration::zero();
    }
    return budget - (now - started_);
  }

  bool active() const
  {
    return active_;
  }

  void reset()
  {
    active_ = false;
  }

private:
  bool active_{false};
  Clock::time_point started_{};
};

}  // namespace bringup

#endif  // BRINGUP__FOLLOW_PATH_RETRY_BUDGET_HPP_
