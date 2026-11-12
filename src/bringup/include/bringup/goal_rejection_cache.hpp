#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace bringup
{

struct GoalRejectionDiagnostic
{
  std::int32_t stamp_sec{0};
  std::uint32_t stamp_nanosec{0};
  std::string message;
};

class GoalRejectionCache
{
public:
  explicit GoalRejectionCache(std::size_t capacity = 8)
  : capacity_(capacity)
  {
  }

  bool remember(std::int32_t stamp_sec, std::uint32_t stamp_nanosec, std::string message)
  {
    if (capacity_ == 0 || message.empty() || stamp_nanosec >= 1000000000U) return false;
    for (const auto & entry : entries_) {
      if (entry.stamp_sec == stamp_sec && entry.stamp_nanosec == stamp_nanosec &&
        entry.message == message)
      {
        return true;
      }
    }
    if (entries_.size() == capacity_) entries_.erase(entries_.begin());
    entries_.push_back({stamp_sec, stamp_nanosec, std::move(message)});
    return true;
  }

  std::vector<GoalRejectionDiagnostic> takeForGoal(
    std::int32_t stamp_sec, std::uint32_t stamp_nanosec)
  {
    std::vector<GoalRejectionDiagnostic> matches;
    std::vector<GoalRejectionDiagnostic> unmatched;
    matches.reserve(entries_.size());
    unmatched.reserve(entries_.size());
    for (auto & entry : entries_) {
      if (entry.stamp_sec == stamp_sec && entry.stamp_nanosec == stamp_nanosec) {
        matches.push_back(std::move(entry));
      } else {
        unmatched.push_back(std::move(entry));
      }
    }
    entries_ = std::move(unmatched);
    return matches;
  }

  std::size_t size() const
  {
    return entries_.size();
  }

private:
  std::size_t capacity_;
  std::vector<GoalRejectionDiagnostic> entries_;
};

}  // namespace bringup
