#ifndef BRINGUP__OBSTACLE_EVIDENCE_SEQUENCE_HPP_
#define BRINGUP__OBSTACLE_EVIDENCE_SEQUENCE_HPP_

#include <algorithm>
#include <charconv>
#include <cstddef>
#include <cstdint>
#include <map>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>
#include <vector>

namespace bringup
{

struct ObstacleEvidenceFrameMetadata
{
  std::uint64_t publisher_id{0};
  std::uint64_t sequence{0};
};

struct ObstacleEvidenceFence
{
  std::uint64_t publisher_id{0};
  std::uint64_t sequence{0};
  std::int64_t source_stamp_ns{0};
};

inline std::string serializeObstacleEvidenceFence(const ObstacleEvidenceFence & fence)
{
  return std::to_string(fence.publisher_id) + ":" + std::to_string(fence.sequence) + ":" +
         std::to_string(fence.source_stamp_ns);
}

inline bool parseObstacleEvidenceFence(
  const std::string & text, ObstacleEvidenceFence & fence)
{
  const auto first_separator = text.find(':');
  if (first_separator == std::string::npos) return false;
  const auto second_separator = text.find(':', first_separator + 1);
  if (second_separator == std::string::npos) return false;

  const auto parse_unsigned = [](std::string_view value, std::uint64_t & parsed) {
      const auto result = std::from_chars(value.data(), value.data() + value.size(), parsed);
      return result.ec == std::errc{} && result.ptr == value.data() + value.size();
    };
  const auto parse_signed = [](std::string_view value, std::int64_t & parsed) {
      const auto result = std::from_chars(value.data(), value.data() + value.size(), parsed);
      return result.ec == std::errc{} && result.ptr == value.data() + value.size();
    };

  ObstacleEvidenceFence parsed;
  if (!parse_unsigned(std::string_view(text).substr(0, first_separator), parsed.publisher_id) ||
    !parse_unsigned(std::string_view(text).substr(
      first_separator + 1, second_separator - first_separator - 1), parsed.sequence) ||
    !parse_signed(std::string_view(text).substr(second_separator + 1), parsed.source_stamp_ns) ||
    parsed.publisher_id == 0 || parsed.sequence == 0 || parsed.source_stamp_ns <= 0)
  {
    return false;
  }
  fence = parsed;
  return true;
}

inline std::string tagObstacleClearEventFrameId(
  const std::string & frame_id, const ObstacleEvidenceFrameMetadata & metadata)
{
  return frame_id + "|bringup_evidence=" + std::to_string(metadata.publisher_id) + ":" +
         std::to_string(metadata.sequence);
}

inline bool parseObstacleClearEventFrameId(
  const std::string & tagged_frame_id, std::string & frame_id,
  ObstacleEvidenceFrameMetadata & metadata)
{
  constexpr std::string_view marker{"|bringup_evidence="};
  const auto marker_position = tagged_frame_id.rfind(marker);
  if (marker_position == std::string::npos || marker_position == 0) return false;
  const auto values = std::string_view(tagged_frame_id).substr(marker_position + marker.size());
  const auto separator = values.find(':');
  if (separator == std::string_view::npos) return false;

  ObstacleEvidenceFrameMetadata parsed;
  const auto publisher_text = values.substr(0, separator);
  const auto sequence_text = values.substr(separator + 1);
  const auto publisher_result = std::from_chars(
    publisher_text.data(), publisher_text.data() + publisher_text.size(), parsed.publisher_id);
  const auto sequence_result = std::from_chars(
    sequence_text.data(), sequence_text.data() + sequence_text.size(), parsed.sequence);
  if (publisher_result.ec != std::errc{} ||
    publisher_result.ptr != publisher_text.data() + publisher_text.size() ||
    sequence_result.ec != std::errc{} ||
    sequence_result.ptr != sequence_text.data() + sequence_text.size() ||
    parsed.publisher_id == 0 || parsed.sequence == 0)
  {
    return false;
  }
  frame_id = tagged_frame_id.substr(0, marker_position);
  metadata = parsed;
  return !frame_id.empty();
}

class ObstacleEvidenceSequenceTracker
{
public:
  enum class ObserveResult
  {
    ACCEPTED,
    PUBLISHER_CHANGED,
    INVALID
  };

  static constexpr std::size_t kSequenceHistoryCapacity = 512;

  ObserveResult observe(const ObstacleEvidenceFrameMetadata & metadata)
  {
    if (metadata.publisher_id == 0 || metadata.sequence == 0) {
      return ObserveResult::INVALID;
    }

    const bool publisher_changed = publisher_id_ != metadata.publisher_id;
    if (publisher_changed) reset(metadata.publisher_id);

    const auto existing = sequence_processed_.find(metadata.sequence);
    if (existing == sequence_processed_.end()) {
      sequence_processed_.emplace(metadata.sequence, false);
      latest_received_sequence_ = std::max(latest_received_sequence_, metadata.sequence);
      if (sequence_processed_.size() > kSequenceHistoryCapacity) {
        sequence_processed_.erase(sequence_processed_.begin());
      }
    }
    return publisher_changed ? ObserveResult::PUBLISHER_CHANGED : ObserveResult::ACCEPTED;
  }

  bool establishFence(const ObstacleEvidenceFence & fence)
  {
    if (fence.publisher_id == 0 || fence.sequence == 0 || fence.source_stamp_ns <= 0) {
      return false;
    }
    if (publisher_id_ == 0) {
      reset(fence.publisher_id);
    } else if (publisher_id_ != fence.publisher_id) {
      // A delayed response from an old publisher must not roll the stream back.
      return false;
    }
    latest_received_sequence_ = std::max(latest_received_sequence_, fence.sequence);
    return true;
  }

  bool markProcessed(const ObstacleEvidenceFrameMetadata & metadata)
  {
    if (metadata.publisher_id != publisher_id_) return false;
    const auto found = sequence_processed_.find(metadata.sequence);
    if (found == sequence_processed_.end()) return false;
    found->second = true;
    return true;
  }

  bool isProcessed(const ObstacleEvidenceFrameMetadata & metadata) const
  {
    if (metadata.publisher_id != publisher_id_) return false;
    const auto found = sequence_processed_.find(metadata.sequence);
    return found != sequence_processed_.end() && found->second;
  }

  bool sequenceRangeProcessed(
    std::uint64_t publisher_id, std::uint64_t first_sequence,
    std::uint64_t last_sequence) const
  {
    if (publisher_id == 0 || publisher_id != publisher_id_ ||
      first_sequence == 0 || last_sequence < first_sequence ||
      last_sequence - first_sequence >= kSequenceHistoryCapacity)
    {
      return false;
    }
    for (auto sequence = first_sequence;; ++sequence) {
      const auto found = sequence_processed_.find(sequence);
      if (found == sequence_processed_.end() || !found->second) return false;
      if (sequence == last_sequence) return true;
    }
  }

  std::uint64_t publisherId() const
  {
    return publisher_id_;
  }

private:
  void reset(std::uint64_t publisher_id)
  {
    publisher_id_ = publisher_id;
    latest_received_sequence_ = 0;
    sequence_processed_.clear();
  }

  std::uint64_t publisher_id_{0};
  std::uint64_t latest_received_sequence_{0};
  std::map<std::uint64_t, bool> sequence_processed_;
};

class ClearEventFenceRefreshTracker
{
public:
  enum class State
  {
    WAITING_FOR_INITIAL_RANGE,
    NEEDS_FRESH_FENCE,
    CHECKING_REFRESHED_RANGE,
    WAITING_FOR_REFRESHED_RANGE
  };

  bool sequenceRangeReady(std::uint64_t event_id, bool range_processed)
  {
    const auto found = states_.find(event_id);
    if (found == states_.end()) {
      if (range_processed) return true;
      states_.emplace(event_id, State::WAITING_FOR_INITIAL_RANGE);
      return false;
    }

    switch (found->second) {
      case State::WAITING_FOR_INITIAL_RANGE:
        if (!range_processed) return false;
        found->second = State::NEEDS_FRESH_FENCE;
        return false;
      case State::NEEDS_FRESH_FENCE:
        return false;
      case State::CHECKING_REFRESHED_RANGE:
        if (!range_processed) {
          found->second = State::WAITING_FOR_REFRESHED_RANGE;
          return false;
        }
        states_.erase(found);
        return true;
      case State::WAITING_FOR_REFRESHED_RANGE:
        if (!range_processed) return false;
        found->second = State::NEEDS_FRESH_FENCE;
        return false;
    }
    return false;
  }

  std::vector<std::uint64_t> eventsNeedingFreshFenceThrough(std::uint64_t event_id) const
  {
    std::vector<std::uint64_t> events;
    for (const auto & entry : states_) {
      if (entry.first > event_id) break;
      if (entry.second == State::NEEDS_FRESH_FENCE) events.push_back(entry.first);
    }
    return events;
  }

  void acceptFreshFence(const std::vector<std::uint64_t> & event_ids)
  {
    for (const auto event_id : event_ids) {
      const auto found = states_.find(event_id);
      if (found != states_.end() && found->second == State::NEEDS_FRESH_FENCE) {
        found->second = State::CHECKING_REFRESHED_RANGE;
      }
    }
  }

  bool hasEventsNeedingFreshFence() const
  {
    for (const auto & entry : states_) {
      if (entry.second == State::NEEDS_FRESH_FENCE) return true;
    }
    return false;
  }

  void forget(std::uint64_t event_id)
  {
    states_.erase(event_id);
  }

  void reset()
  {
    states_.clear();
  }

private:
  std::map<std::uint64_t, State> states_;
};

inline bool clearEventCoveredByFence(std::uint64_t event_id, std::uint64_t fence_event_id)
{
  return event_id > 0 && event_id <= fence_event_id;
}

}  // namespace bringup

#endif  // BRINGUP__OBSTACLE_EVIDENCE_SEQUENCE_HPP_
