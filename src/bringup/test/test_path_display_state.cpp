#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <action_msgs/msg/goal_status.hpp>
#include <action_msgs/msg/goal_status_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/qos.hpp>

#include "bringup/path_display_state.hpp"

using action_msgs::msg::GoalStatus;
using bringup::DirectNav2PathAssociation;
using bringup::GlobalPathTerminalDisposition;
using bringup::NavigationAction;
using bringup::PathDisplayClear;
using bringup::PathDisplayState;

namespace
{

action_msgs::msg::GoalStatus status(
  const std::string & id, std::uint8_t value, std::int32_t accepted_sec = 0,
  std::uint32_t accepted_nanosec = 0)
{
  action_msgs::msg::GoalStatus result;
  result.goal_info.goal_id.uuid.fill(0);
  std::copy_n(id.begin(), std::min(id.size(), result.goal_info.goal_id.uuid.size()),
    result.goal_info.goal_id.uuid.begin());
  result.goal_info.stamp.sec = accepted_sec;
  result.goal_info.stamp.nanosec = accepted_nanosec;
  result.status = value;
  return result;
}

}  // namespace

TEST(PathDisplayState, RetainsRouteWhenInternalCancelArrivesBeforePause)
{
  EXPECT_EQ(
    bringup::globalPathTerminalDisposition(
      true, "public_a", "public_a", "execution_a", "execution_a", "execution_a"),
    GlobalPathTerminalDisposition::RETAIN);
}

TEST(PathDisplayState, KeepsPriorRouteWhileResumedExecutionWaitsForANewPlan)
{
  EXPECT_EQ(
    bringup::globalPathTerminalDisposition(
      true, "public_a", "public_a", "execution_a", "execution_b", "execution_b"),
    GlobalPathTerminalDisposition::IGNORE);
}

TEST(PathDisplayState, IgnoresLateCancelAfterNewExecutionTakesOver)
{
  EXPECT_EQ(
    bringup::globalPathTerminalDisposition(
      true, "public_a", "public_a", "execution_b", "execution_a", "execution_b"),
    GlobalPathTerminalDisposition::IGNORE);
}

TEST(PathDisplayState, NewPublicGoalDoesNotRetainOldRoute)
{
  EXPECT_EQ(
    bringup::globalPathTerminalDisposition(
      true, "public_a", "public_b", "execution_a", "execution_a", "execution_a"),
    GlobalPathTerminalDisposition::CLEAR);
  EXPECT_EQ(
    bringup::globalPathTerminalDisposition(
      false, "public_a", "public_a", "execution_a", "execution_a", "execution_a"),
    GlobalPathTerminalDisposition::CLEAR);
}

TEST(PathDisplayState, ignoresOldGoalResultsAndPreservesGlobalOnPlanningFailure)
{
  PathDisplayState state;

  action_msgs::msg::GoalStatusArray active;
  active.status_list = {
    status("old", GoalStatus::STATUS_ABORTED, 1),
    status("current", GoalStatus::STATUS_EXECUTING, 2)};
  const auto no_clear = state.observeStatuses(NavigationAction::ComputePath, active);
  EXPECT_FALSE(no_clear.global);
  EXPECT_FALSE(no_clear.local);

  action_msgs::msg::GoalStatusArray terminal;
  terminal.status_list = {
    status("old", GoalStatus::STATUS_ABORTED, 1),
    status("current", GoalStatus::STATUS_ABORTED, 2)};
  const auto clear = state.observeStatuses(NavigationAction::ComputePath, terminal);
  EXPECT_FALSE(clear.global);
  EXPECT_TRUE(clear.local);
  const auto repeated = state.observeStatuses(NavigationAction::ComputePath, terminal);
  EXPECT_FALSE(repeated.global);
  EXPECT_FALSE(repeated.local);
}

TEST(DirectNav2PathAssociation, rejectsLateOldPathUntilFollowPathHandoff)
{
  DirectNav2PathAssociation association;
  association.observeNavigationGoal(status("nav_a", GoalStatus::STATUS_EXECUTING, 10));
  association.observeActionGoal(
    NavigationAction::ComputePath, status("compute_a", GoalStatus::STATUS_EXECUTING, 11));
  association.observeActionGoal(
    NavigationAction::FollowPath, status("follow_a", GoalStatus::STATUS_EXECUTING, 12));

  builtin_interfaces::msg::Time path_stamp;
  path_stamp.sec = 13;
  EXPECT_TRUE(association.globalPathMatchesCurrentGoal(path_stamp));
  EXPECT_TRUE(association.localPathMatchesCurrentGoal(path_stamp));

  association.observeNavigationGoal(status("nav_b", GoalStatus::STATUS_ACCEPTED, 20));
  path_stamp.sec = 21;
  EXPECT_FALSE(association.globalPathMatchesCurrentGoal(path_stamp));
  EXPECT_FALSE(association.localPathMatchesCurrentGoal(path_stamp));

  // Goal A can publish a fresh local-plan stamp while goal B is still waiting
  // for its ComputePathToPose and FollowPath actions to take over.
  association.observeActionGoal(
    NavigationAction::FollowPath, status("follow_a", GoalStatus::STATUS_EXECUTING, 12));
  path_stamp.sec = 30;
  EXPECT_FALSE(association.localPathMatchesCurrentGoal(path_stamp));

  association.observeActionGoal(
    NavigationAction::ComputePath, status("compute_b", GoalStatus::STATUS_EXECUTING, 21));
  path_stamp.sec = 22;
  EXPECT_TRUE(association.globalPathMatchesCurrentGoal(path_stamp));
  EXPECT_FALSE(association.localPathMatchesCurrentGoal(path_stamp));

  association.observeActionGoal(
    NavigationAction::FollowPath, status("follow_b", GoalStatus::STATUS_EXECUTING, 22));
  path_stamp.sec = 23;
  EXPECT_TRUE(association.localPathMatchesCurrentGoal(path_stamp));
}

TEST(DirectNav2PathAssociation, rapidGoalReplacementWaitsForLatestRouteHandoff)
{
  DirectNav2PathAssociation association;
  association.observeNavigationGoal(status("nav_a", GoalStatus::STATUS_EXECUTING, 10));
  association.observeActionGoal(
    NavigationAction::ComputePath, status("compute_a", GoalStatus::STATUS_EXECUTING, 11));
  association.observeActionGoal(
    NavigationAction::FollowPath, status("follow_a", GoalStatus::STATUS_EXECUTING, 12));

  association.observeNavigationGoal(status("nav_b", GoalStatus::STATUS_ACCEPTED, 20));
  association.observeNavigationGoal(status("nav_c", GoalStatus::STATUS_ACCEPTED, 30));
  association.observeActionGoal(
    NavigationAction::ComputePath, status("compute_b", GoalStatus::STATUS_EXECUTING, 21));
  association.observeActionGoal(
    NavigationAction::FollowPath, status("follow_b", GoalStatus::STATUS_EXECUTING, 22));

  builtin_interfaces::msg::Time path_stamp;
  path_stamp.sec = 31;
  EXPECT_FALSE(association.globalPathMatchesCurrentGoal(path_stamp));
  EXPECT_FALSE(association.localPathMatchesCurrentGoal(path_stamp));

  association.observeActionGoal(
    NavigationAction::ComputePath, status("compute_c", GoalStatus::STATUS_EXECUTING, 31));
  path_stamp.sec = 32;
  EXPECT_TRUE(association.globalPathMatchesCurrentGoal(path_stamp));
  EXPECT_FALSE(association.localPathMatchesCurrentGoal(path_stamp));

  association.observeActionGoal(
    NavigationAction::FollowPath, status("follow_c", GoalStatus::STATUS_EXECUTING, 32));
  path_stamp.sec = 33;
  EXPECT_TRUE(association.localPathMatchesCurrentGoal(path_stamp));
}

TEST(DirectNav2PathAssociation, cancellationDisablesPathsUntilNewGoalRouteHandoff)
{
  DirectNav2PathAssociation association;
  association.observeNavigationGoal(status("nav_a", GoalStatus::STATUS_EXECUTING, 10));
  association.observeActionGoal(
    NavigationAction::ComputePath, status("compute_a", GoalStatus::STATUS_EXECUTING, 11));
  association.observeActionGoal(
    NavigationAction::FollowPath, status("follow_a", GoalStatus::STATUS_EXECUTING, 12));

  association.observeNavigationGoal(status("nav_a", GoalStatus::STATUS_CANCELED, 10));
  builtin_interfaces::msg::Time path_stamp;
  path_stamp.sec = 100;
  EXPECT_FALSE(association.globalPathMatchesCurrentGoal(path_stamp));
  EXPECT_FALSE(association.localPathMatchesCurrentGoal(path_stamp));

  association.observeNavigationGoal(status("nav_b", GoalStatus::STATUS_ACCEPTED, 20));
  EXPECT_FALSE(association.globalPathMatchesCurrentGoal(path_stamp));
  EXPECT_FALSE(association.localPathMatchesCurrentGoal(path_stamp));
}

TEST(PathDisplayState, olderCanceledGoalCannotClearNewGoalRegardlessOfArrayOrder)
{
  const auto verify_order = [](bool old_first) {
      PathDisplayState state;
      action_msgs::msg::GoalStatusArray executing;
      const auto old_active = status("old", GoalStatus::STATUS_EXECUTING, 10);
      const auto new_active = status("new", GoalStatus::STATUS_EXECUTING, 20);
      executing.status_list = old_first ?
        std::vector<GoalStatus>{old_active, new_active} :
        std::vector<GoalStatus>{new_active, old_active};
      EXPECT_FALSE(state.observeStatuses(NavigationAction::ComputePath, executing).global);

      action_msgs::msg::GoalStatusArray old_canceled;
      const auto old_terminal = status("old", GoalStatus::STATUS_CANCELED, 10);
      old_canceled.status_list = old_first ?
        std::vector<GoalStatus>{old_terminal, new_active} :
        std::vector<GoalStatus>{new_active, old_terminal};
      const auto stale_result = state.observeStatuses(
        NavigationAction::ComputePath, old_canceled);
      EXPECT_FALSE(stale_result.global);
      EXPECT_FALSE(stale_result.local);

      action_msgs::msg::GoalStatusArray new_failed;
      new_failed.status_list = {status("new", GoalStatus::STATUS_ABORTED, 20)};
      const auto current_result = state.observeStatuses(
        NavigationAction::ComputePath, new_failed);
      EXPECT_TRUE(current_result.global);
      EXPECT_TRUE(current_result.local);
    };

  verify_order(true);
  verify_order(false);
}

TEST(PathDisplayState, equalAcceptanceTimesUseStableGoalIdTieBreak)
{
  PathDisplayState first_state;
  PathDisplayState second_state;
  const auto low_id = status("alpha", GoalStatus::STATUS_EXECUTING, 4, 100);
  const auto high_id = status("zeta", GoalStatus::STATUS_EXECUTING, 4, 100);

  action_msgs::msg::GoalStatusArray first_order;
  first_order.status_list = {low_id, high_id};
  action_msgs::msg::GoalStatusArray second_order;
  second_order.status_list = {high_id, low_id};
  first_state.observeStatuses(NavigationAction::NavigateToPose, first_order);
  second_state.observeStatuses(NavigationAction::NavigateToPose, second_order);

  action_msgs::msg::GoalStatusArray terminal;
  terminal.status_list = {status("zeta", GoalStatus::STATUS_CANCELED, 4, 100)};
  const auto first_clear = first_state.observeStatuses(
    NavigationAction::NavigateToPose, terminal);
  const auto second_clear = second_state.observeStatuses(
    NavigationAction::NavigateToPose, terminal);
  EXPECT_TRUE(first_clear.global);
  EXPECT_TRUE(first_clear.local);
  EXPECT_TRUE(second_clear.global);
  EXPECT_TRUE(second_clear.local);
}

TEST(PathDisplayState, acceptsGoalStatusSlightlyAheadOfObservedClock)
{
  PathDisplayState state;
  constexpr std::int64_t kNanosecondsPerSecond = 1000000000LL;
  state.observeClock(10 * kNanosecondsPerSecond);

  action_msgs::msg::GoalStatusArray active;
  active.status_list = {status("future", GoalStatus::STATUS_EXECUTING, 11)};
  const auto no_clear = state.observeStatuses(NavigationAction::ComputePath, active);
  EXPECT_FALSE(no_clear.global);
  EXPECT_FALSE(no_clear.local);

  action_msgs::msg::GoalStatusArray terminal;
  terminal.status_list = {status("future", GoalStatus::STATUS_ABORTED, 11)};
  const auto clear = state.observeStatuses(NavigationAction::ComputePath, terminal);
  EXPECT_TRUE(clear.global);
  EXPECT_TRUE(clear.local);
}

TEST(PathDisplayState, oldGoalCanEndAfterClockRollbackWithoutReplacement)
{
  PathDisplayState state;
  constexpr std::int64_t kNanosecondsPerSecond = 1000000000LL;

  state.observeClock(100 * kNanosecondsPerSecond);
  action_msgs::msg::GoalStatusArray before_rollback;
  before_rollback.status_list = {
    status("old", GoalStatus::STATUS_EXECUTING, 100)};
  state.observeStatuses(NavigationAction::ComputePath, before_rollback);

  state.observeClock(10 * kNanosecondsPerSecond);
  action_msgs::msg::GoalStatusArray old_goal_failed;
  old_goal_failed.status_list = {
    status("old", GoalStatus::STATUS_ABORTED, 100)};
  const auto clear = state.observeStatuses(
    NavigationAction::ComputePath, old_goal_failed);
  EXPECT_TRUE(clear.global);
  EXPECT_TRUE(clear.local);
}

TEST(PathDisplayState, newTimelineGoalSupersedesOldStatusAfterClockRollback)
{
  PathDisplayState state;
  constexpr std::int64_t kNanosecondsPerSecond = 1000000000LL;

  state.observeClock(100 * kNanosecondsPerSecond);
  action_msgs::msg::GoalStatusArray before_rollback;
  before_rollback.status_list = {
    status("old", GoalStatus::STATUS_EXECUTING, 100)};
  state.observeStatuses(NavigationAction::ComputePath, before_rollback);

  state.observeClock(10 * kNanosecondsPerSecond);
  action_msgs::msg::GoalStatusArray after_rollback;
  after_rollback.status_list = {
    status("old", GoalStatus::STATUS_EXECUTING, 100),
    status("new", GoalStatus::STATUS_EXECUTING, 10)};
  const auto stale_status = state.observeStatuses(
    NavigationAction::ComputePath, after_rollback);
  EXPECT_FALSE(stale_status.global);
  EXPECT_FALSE(stale_status.local);

  state.observeClock(101 * kNanosecondsPerSecond);
  action_msgs::msg::GoalStatusArray clock_caught_up;
  clock_caught_up.status_list = {
    status("old", GoalStatus::STATUS_CANCELED, 100),
    status("new", GoalStatus::STATUS_EXECUTING, 10)};
  const auto caught_up_stale_status = state.observeStatuses(
    NavigationAction::ComputePath, clock_caught_up);
  EXPECT_FALSE(caught_up_stale_status.global);
  EXPECT_FALSE(caught_up_stale_status.local);

  action_msgs::msg::GoalStatusArray new_goal_failed;
  new_goal_failed.status_list = {
    status("old", GoalStatus::STATUS_CANCELED, 100),
    status("new", GoalStatus::STATUS_ABORTED, 10)};
  const auto clear = state.observeStatuses(
    NavigationAction::ComputePath, new_goal_failed);
  EXPECT_TRUE(clear.global);
  EXPECT_TRUE(clear.local);
}

TEST(PathDisplayState, keepsGoalGenerationsDistinctAcrossTwoClockRollbacks)
{
  PathDisplayState state;
  constexpr std::int64_t kNanosecondsPerSecond = 1000000000LL;

  state.observeClock(100 * kNanosecondsPerSecond);
  action_msgs::msg::GoalStatusArray first_timeline;
  first_timeline.status_list = {
    status("old", GoalStatus::STATUS_EXECUTING, 100)};
  state.observeStatuses(NavigationAction::ComputePath, first_timeline);

  state.observeClock(10 * kNanosecondsPerSecond);
  action_msgs::msg::GoalStatusArray second_timeline;
  second_timeline.status_list = {
    status("old", GoalStatus::STATUS_EXECUTING, 100),
    status("new", GoalStatus::STATUS_EXECUTING, 20)};
  const auto first_rollback = state.observeStatuses(
    NavigationAction::ComputePath, second_timeline);
  EXPECT_FALSE(first_rollback.global);
  EXPECT_FALSE(first_rollback.local);

  // Let the clock advance, then loop the bag a second time while both goals
  // remain visible in the action status array.
  state.observeClock(30 * kNanosecondsPerSecond);
  state.observeClock(5 * kNanosecondsPerSecond);
  action_msgs::msg::GoalStatusArray old_goal_finished;
  old_goal_finished.status_list = {
    status("old", GoalStatus::STATUS_ABORTED, 100),
    status("new", GoalStatus::STATUS_EXECUTING, 20)};
  const auto stale_terminal = state.observeStatuses(
    NavigationAction::ComputePath, old_goal_finished);
  EXPECT_FALSE(stale_terminal.global);
  EXPECT_FALSE(stale_terminal.local);

  action_msgs::msg::GoalStatusArray new_goal_failed;
  new_goal_failed.status_list = {
    status("old", GoalStatus::STATUS_ABORTED, 100),
    status("new", GoalStatus::STATUS_ABORTED, 20)};
  const auto clear = state.observeStatuses(
    NavigationAction::ComputePath, new_goal_failed);
  EXPECT_TRUE(clear.global);
  EXPECT_TRUE(clear.local);
}

TEST(PathDisplayState, tracksLatestGoalIndependentlyForEachAction)
{
  PathDisplayState state;
  action_msgs::msg::GoalStatusArray compute_active;
  compute_active.status_list = {status("compute", GoalStatus::STATUS_EXECUTING, 10)};
  state.observeStatuses(NavigationAction::ComputePath, compute_active);

  action_msgs::msg::GoalStatusArray follow_active;
  follow_active.status_list = {status("follow", GoalStatus::STATUS_EXECUTING, 20)};
  state.observeStatuses(NavigationAction::FollowPath, follow_active);

  action_msgs::msg::GoalStatusArray compute_failed;
  compute_failed.status_list = {status("compute", GoalStatus::STATUS_ABORTED, 10)};
  const auto clear = state.observeStatuses(NavigationAction::ComputePath, compute_failed);
  EXPECT_TRUE(clear.global);
  EXPECT_TRUE(clear.local);
}

TEST(PathDisplayState, clearsOnlyLocalPathWhenFollowPathAborts)
{
  PathDisplayState state;
  action_msgs::msg::GoalStatusArray active;
  active.status_list = {status("follow", GoalStatus::STATUS_EXECUTING)};
  state.observeStatuses(NavigationAction::FollowPath, active);

  action_msgs::msg::GoalStatusArray terminal;
  terminal.status_list = {status("follow", GoalStatus::STATUS_ABORTED)};
  const auto clear = state.observeStatuses(NavigationAction::FollowPath, terminal);
  EXPECT_FALSE(clear.global);
  EXPECT_TRUE(clear.local);
}

TEST(PathDisplayState, clearsBothPathsWhenNavigationEnds)
{
  PathDisplayState state;
  action_msgs::msg::GoalStatusArray active;
  active.status_list = {status("navigate", GoalStatus::STATUS_EXECUTING)};
  state.observeStatuses(NavigationAction::NavigateToPose, active);

  action_msgs::msg::GoalStatusArray terminal;
  terminal.status_list = {status("navigate", GoalStatus::STATUS_CANCELED)};
  const auto clear = state.observeStatuses(NavigationAction::NavigateToPose, terminal);
  EXPECT_TRUE(clear.global);
  EXPECT_TRUE(clear.local);
}

TEST(PathDisplayState, clearsBothPathsWhenNavigationAborts)
{
  PathDisplayState state;
  action_msgs::msg::GoalStatusArray active;
  active.status_list = {status("navigate", GoalStatus::STATUS_EXECUTING)};
  state.observeStatuses(NavigationAction::NavigateToPose, active);

  action_msgs::msg::GoalStatusArray terminal;
  terminal.status_list = {status("navigate", GoalStatus::STATUS_ABORTED)};
  const auto clear = state.observeStatuses(NavigationAction::NavigateToPose, terminal);
  EXPECT_TRUE(clear.global);
  EXPECT_TRUE(clear.local);
  const auto repeated = state.observeStatuses(NavigationAction::NavigateToPose, terminal);
  EXPECT_FALSE(repeated.global);
  EXPECT_FALSE(repeated.local);
}

TEST(PathDisplayState, clearsBothPathsWhenNavigationSucceeds)
{
  PathDisplayState state;
  action_msgs::msg::GoalStatusArray active;
  active.status_list = {status("navigate", GoalStatus::STATUS_EXECUTING, 10)};
  state.observeStatuses(NavigationAction::NavigateToPose, active);

  action_msgs::msg::GoalStatusArray terminal;
  terminal.status_list = {status("navigate", GoalStatus::STATUS_SUCCEEDED, 10)};
  const auto clear = state.observeStatuses(NavigationAction::NavigateToPose, terminal);
  EXPECT_TRUE(clear.global);
  EXPECT_TRUE(clear.local);
}

TEST(PathDisplayState, lateSubscriberReceivesCachedActiveGoalAndKeepsItOnAbort)
{
  rclcpp::init(0, nullptr);
  auto publisher_node = std::make_shared<rclcpp::Node>("path_status_publisher_test");
  auto subscriber_node = std::make_shared<rclcpp::Node>("path_status_subscriber_test");
  auto publisher = publisher_node->create_publisher<action_msgs::msg::GoalStatusArray>(
    "/path_display_state_test/status", rclcpp_action::DefaultActionStatusQoS());

  action_msgs::msg::GoalStatusArray active;
  active.status_list = {status("current", GoalStatus::STATUS_EXECUTING)};
  publisher->publish(active);

  PathDisplayState state;
  std::vector<PathDisplayClear> clears;
  auto subscription = subscriber_node->create_subscription<action_msgs::msg::GoalStatusArray>(
    "/path_display_state_test/status", rclcpp_action::DefaultActionStatusQoS(),
    [&state, &clears](action_msgs::msg::GoalStatusArray::ConstSharedPtr message) {
      clears.push_back(state.observeStatuses(NavigationAction::ComputePath, *message));
    });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(publisher_node);
  executor.add_node(subscriber_node);
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
  while (clears.empty() && std::chrono::steady_clock::now() < deadline) {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  ASSERT_FALSE(clears.empty()) << "late subscriber did not receive cached active status";
  EXPECT_FALSE(clears.back().global);
  EXPECT_FALSE(clears.back().local);

  action_msgs::msg::GoalStatusArray terminal;
  terminal.status_list = {status("current", GoalStatus::STATUS_ABORTED)};
  publisher->publish(terminal);
  const auto terminal_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
  while (clears.size() < 2 && std::chrono::steady_clock::now() < terminal_deadline) {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  ASSERT_GE(clears.size(), 2U) << "subscriber did not receive terminal status";
  EXPECT_FALSE(clears.back().global);
  EXPECT_TRUE(clears.back().local);

  executor.remove_node(subscriber_node);
  executor.remove_node(publisher_node);
  subscription.reset();
  publisher.reset();
  subscriber_node.reset();
  publisher_node.reset();
  rclcpp::shutdown();
}
