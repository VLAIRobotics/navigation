#ifndef BRINGUP__PATH_DISPLAY_STATE_HPP_
#define BRINGUP__PATH_DISPLAY_STATE_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <map>
#include <set>
#include <string>

#include <action_msgs/msg/goal_status.hpp>
#include <action_msgs/msg/goal_status_array.hpp>
#include <builtin_interfaces/msg/time.hpp>

namespace bringup
{

enum class NavigationAction
{
  ComputePath,
  FollowPath,
  NavigateToPose
};

class DirectNav2PathAssociation
{
public:
  void observeNavigationGoal(const action_msgs::msg::GoalStatus & status)
  {
    const auto & goal_id = status.goal_info.goal_id.uuid;
    if (!have_navigation_goal_identity_ || navigation_goal_id_ != goal_id) {
      navigation_goal_id_ = goal_id;
      navigation_goal_stamp_ = status.goal_info.stamp;
      have_navigation_goal_identity_ = true;
    }
    navigation_goal_active_ = isActive(status.status);
    refresh();
  }

  void observeActionGoal(
    NavigationAction action, const action_msgs::msg::GoalStatus & status)
  {
    if (action == NavigationAction::ComputePath) {
      updateLatestGoal(have_compute_status_, latest_compute_status_, status);
    } else if (action == NavigationAction::FollowPath) {
      updateLatestGoal(have_follow_status_, latest_follow_status_, status);
    }
    refresh();
  }

  bool globalPathMatchesCurrentGoal(const builtin_interfaces::msg::Time & path_stamp) const
  {
    return navigation_goal_active_ && current_compute_goal_valid_ &&
           stampAtOrAfter(current_compute_goal_.goal_info.stamp, navigation_goal_stamp_) &&
           stampAtOrAfter(path_stamp, current_compute_goal_.goal_info.stamp);
  }

  bool localPathMatchesCurrentGoal(const builtin_interfaces::msg::Time & path_stamp) const
  {
    return globalPathMatchesCurrentGoal(path_stamp) && current_follow_goal_valid_ &&
           stampAtOrAfter(current_follow_goal_.goal_info.stamp,
           current_compute_goal_.goal_info.stamp) &&
           stampAtOrAfter(path_stamp, current_follow_goal_.goal_info.stamp);
  }

  const action_msgs::msg::GoalStatus * currentComputeGoal() const
  {
    return current_compute_goal_valid_ ? &current_compute_goal_ : nullptr;
  }

  const action_msgs::msg::GoalStatus * latestComputeGoal() const
  {
    return have_compute_status_ ? &latest_compute_status_ : nullptr;
  }

  const action_msgs::msg::GoalStatus * currentFollowGoal() const
  {
    return current_follow_goal_valid_ ? &current_follow_goal_ : nullptr;
  }

private:
  static bool isActive(std::uint8_t status)
  {
    return status == action_msgs::msg::GoalStatus::STATUS_ACCEPTED ||
           status == action_msgs::msg::GoalStatus::STATUS_EXECUTING;
  }

  static bool isUsableComputeGoal(std::uint8_t status)
  {
    return isActive(status) || status == action_msgs::msg::GoalStatus::STATUS_SUCCEEDED;
  }

  static bool isExecuting(std::uint8_t status)
  {
    return status == action_msgs::msg::GoalStatus::STATUS_EXECUTING;
  }

  static bool stampAtOrAfter(
    const builtin_interfaces::msg::Time & lhs,
    const builtin_interfaces::msg::Time & rhs)
  {
    return lhs.sec > rhs.sec ||
           (lhs.sec == rhs.sec && lhs.nanosec >= rhs.nanosec);
  }

  static bool sameGoal(
    const action_msgs::msg::GoalStatus & lhs,
    const action_msgs::msg::GoalStatus & rhs)
  {
    return lhs.goal_info.goal_id.uuid == rhs.goal_info.goal_id.uuid;
  }

  static bool isNewer(
    const action_msgs::msg::GoalStatus & candidate,
    const action_msgs::msg::GoalStatus & current)
  {
    const auto & candidate_stamp = candidate.goal_info.stamp;
    const auto & current_stamp = current.goal_info.stamp;
    if (candidate_stamp.sec != current_stamp.sec) {
      return candidate_stamp.sec > current_stamp.sec;
    }
    if (candidate_stamp.nanosec != current_stamp.nanosec) {
      return candidate_stamp.nanosec > current_stamp.nanosec;
    }
    return candidate.goal_info.goal_id.uuid > current.goal_info.goal_id.uuid;
  }

  static void updateLatestGoal(
    bool & initialized, action_msgs::msg::GoalStatus & current,
    const action_msgs::msg::GoalStatus & candidate)
  {
    if (!initialized || sameGoal(candidate, current) || isNewer(candidate, current)) {
      current = candidate;
      initialized = true;
    }
  }

  void refresh()
  {
    current_compute_goal_valid_ = navigation_goal_active_ && have_compute_status_ &&
      isUsableComputeGoal(latest_compute_status_.status) &&
      stampAtOrAfter(latest_compute_status_.goal_info.stamp, navigation_goal_stamp_);
    if (!current_compute_goal_valid_) {
      current_follow_goal_valid_ = false;
      return;
    }
    current_compute_goal_ = latest_compute_status_;

    const bool same_follow_goal = current_follow_goal_valid_ && have_follow_status_ &&
      sameGoal(current_follow_goal_, latest_follow_status_) &&
      sameGoal(current_follow_compute_goal_, current_compute_goal_);
    const bool follow_goal_is_new = !have_associated_follow_goal_ ||
      !sameGoal(last_associated_follow_goal_, latest_follow_status_);
    current_follow_goal_valid_ = have_follow_status_ &&
      isExecuting(latest_follow_status_.status) &&
      stampAtOrAfter(latest_follow_status_.goal_info.stamp,
      current_compute_goal_.goal_info.stamp) &&
      (follow_goal_is_new || same_follow_goal);
    if (current_follow_goal_valid_) {
      current_follow_goal_ = latest_follow_status_;
      current_follow_compute_goal_ = current_compute_goal_;
      last_associated_follow_goal_ = latest_follow_status_;
      have_associated_follow_goal_ = true;
    }
  }

  std::array<std::uint8_t, 16> navigation_goal_id_{};
  builtin_interfaces::msg::Time navigation_goal_stamp_;
  action_msgs::msg::GoalStatus latest_compute_status_;
  action_msgs::msg::GoalStatus latest_follow_status_;
  action_msgs::msg::GoalStatus current_compute_goal_;
  action_msgs::msg::GoalStatus current_follow_goal_;
  action_msgs::msg::GoalStatus current_follow_compute_goal_;
  action_msgs::msg::GoalStatus last_associated_follow_goal_;
  bool have_navigation_goal_identity_{false};
  bool navigation_goal_active_{false};
  bool have_compute_status_{false};
  bool have_follow_status_{false};
  bool current_compute_goal_valid_{false};
  bool current_follow_goal_valid_{false};
  bool have_associated_follow_goal_{false};
};

struct PathDisplayClear
{
  bool global{false};
  bool local{false};
};

enum class GlobalPathTerminalDisposition
{
  IGNORE,
  RETAIN,
  CLEAR,
};

inline GlobalPathTerminalDisposition globalPathTerminalDisposition(
  bool public_goal_active, const std::string & displayed_public_goal_id,
  const std::string & current_public_goal_id, const std::string & displayed_execution_id,
  const std::string & terminal_execution_id, const std::string & current_execution_id)
{
  if (terminal_execution_id.empty() || current_execution_id != terminal_execution_id ||
    displayed_execution_id != terminal_execution_id)
  {
    return GlobalPathTerminalDisposition::IGNORE;
  }
  if (public_goal_active && !current_public_goal_id.empty() &&
    displayed_public_goal_id == current_public_goal_id)
  {
    return GlobalPathTerminalDisposition::RETAIN;
  }
  return GlobalPathTerminalDisposition::CLEAR;
}

class PathDisplayState
{
public:
  void observeClock(std::int64_t now_nanoseconds)
  {
    if (have_clock_ && now_nanoseconds < last_clock_nanoseconds_) {
      ++timeline_generation_;
    }
    last_clock_nanoseconds_ = now_nanoseconds;
    have_clock_ = true;
  }

  PathDisplayClear observeStatuses(
    NavigationAction action, const action_msgs::msg::GoalStatusArray & message)
  {
    const auto action_index = index(action);
    auto & goal_generations = goal_generations_[action_index];
    std::set<GoalUuid> current_goal_ids;
    const action_msgs::msg::GoalStatus * latest = nullptr;
    std::uint64_t latest_generation = 0;
    for (const auto & status : message.status_list) {
      const auto & goal_id = status.goal_info.goal_id.uuid;
      current_goal_ids.insert(goal_id);
      const auto inserted = goal_generations.emplace(goal_id, timeline_generation_);
      const auto generation = inserted.first->second;
      if (latest == nullptr || generation > latest_generation ||
        (generation == latest_generation &&
        (isNewer(status, *latest) ||
        (sameGoal(status, *latest) && isTerminal(status.status) &&
        !isTerminal(latest->status)))))
      {
        latest = &status;
        latest_generation = generation;
      }
    }

    // Keep only goal IDs still reported by the action server. Their generation
    // remains stable across clock rollbacks while they are in its status list.
    for (auto it = goal_generations.begin(); it != goal_generations.end();) {
      if (current_goal_ids.count(it->first) == 0) {
        it = goal_generations.erase(it);
      } else {
        ++it;
      }
    }
    if (latest == nullptr) {
      return {};
    }

    auto & action_state = states_[action_index];
    if (!action_state.initialized ||
      latest_generation > action_state.latest_goal_generation ||
      (latest_generation == action_state.latest_goal_generation &&
      isNewer(*latest, action_state.latest_goal)))
    {
      action_state.latest_goal = *latest;
      action_state.initialized = true;
      action_state.terminal_processed = false;
      action_state.latest_goal_generation = latest_generation;
    } else if (!sameGoal(*latest, action_state.latest_goal)) {
      return {};
    }

    if (!isTerminal(latest->status) || action_state.terminal_processed) {
      return {};
    }
    action_state.terminal_processed = true;
    return clearFor(action, latest->status);
  }

private:
  using GoalUuid = std::array<std::uint8_t, 16>;

  struct ActionGoalState
  {
    bool initialized{false};
    bool terminal_processed{false};
    std::uint64_t latest_goal_generation{0};
    action_msgs::msg::GoalStatus latest_goal;
  };

  static constexpr std::size_t index(NavigationAction action)
  {
    return static_cast<std::size_t>(action);
  }

  static bool isTerminal(std::uint8_t status)
  {
    using GoalStatus = action_msgs::msg::GoalStatus;
    return status == GoalStatus::STATUS_SUCCEEDED ||
           status == GoalStatus::STATUS_CANCELED ||
           status == GoalStatus::STATUS_ABORTED;
  }

  static bool isNewer(
    const action_msgs::msg::GoalStatus & candidate,
    const action_msgs::msg::GoalStatus & current)
  {
    const auto & candidate_stamp = candidate.goal_info.stamp;
    const auto & current_stamp = current.goal_info.stamp;
    if (candidate_stamp.sec != current_stamp.sec) {
      return candidate_stamp.sec > current_stamp.sec;
    }
    if (candidate_stamp.nanosec != current_stamp.nanosec) {
      return candidate_stamp.nanosec > current_stamp.nanosec;
    }
    return candidate.goal_info.goal_id.uuid > current.goal_info.goal_id.uuid;
  }

  static bool sameGoal(
    const action_msgs::msg::GoalStatus & lhs,
    const action_msgs::msg::GoalStatus & rhs)
  {
    return lhs.goal_info.goal_id.uuid == rhs.goal_info.goal_id.uuid;
  }

  static PathDisplayClear clearFor(NavigationAction action, std::uint8_t status)
  {
    using GoalStatus = action_msgs::msg::GoalStatus;
    if (action == NavigationAction::NavigateToPose) {
      return {true, true};
    }
    if (action == NavigationAction::ComputePath && status == GoalStatus::STATUS_ABORTED) {
      return {false, true};
    }
    if (action == NavigationAction::ComputePath && status == GoalStatus::STATUS_CANCELED) {
      return {true, true};
    }
    if (action == NavigationAction::FollowPath && status == GoalStatus::STATUS_ABORTED) {
      return {false, true};
    }
    return {};
  }

  std::array<ActionGoalState, 3> states_;
  std::array<std::map<GoalUuid, std::uint64_t>, 3> goal_generations_;
  std::int64_t last_clock_nanoseconds_{0};
  std::uint64_t timeline_generation_{0};
  bool have_clock_{false};
};

}  // namespace bringup

#endif  // BRINGUP__PATH_DISPLAY_STATE_HPP_
