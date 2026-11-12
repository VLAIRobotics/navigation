#include <algorithm>
#include <array>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "bringup/goal_rejection_cache.hpp"
#include "bringup/navigation_goal_status.hpp"

TEST(GoalRejectionCache, RetainsEarlyLogAcrossStatusAndIdentityArrivalOrders)
{
  enum class Event { LOG, STATUS, IDENTITY };
  std::array<Event, 3> events{Event::LOG, Event::STATUS, Event::IDENTITY};
  constexpr std::int32_t goal_stamp_sec = 172;
  constexpr std::uint32_t goal_stamp_nanosec = 440000000;
  constexpr const char * diagnostic = "category=static_goal_unknown goal_stamp=172.440000000";

  do {
    bringup::GoalRejectionCache cache;
    bool status_received = false;
    bool identity_received = false;
    std::vector<std::string> displayed;
    for (const auto event : events) {
      if (event == Event::LOG) {
        ASSERT_TRUE(cache.remember(goal_stamp_sec, goal_stamp_nanosec, diagnostic));
      } else if (event == Event::STATUS) {
        status_received = true;
      } else {
        identity_received = true;
      }
      if (status_received && identity_received) {
        for (auto & entry : cache.takeForGoal(goal_stamp_sec, goal_stamp_nanosec)) {
          displayed.push_back(std::move(entry.message));
        }
      }
    }
    ASSERT_EQ(displayed.size(), 1U);
    EXPECT_EQ(displayed.front(), diagnostic);
    EXPECT_EQ(cache.size(), 0U);
  } while (std::next_permutation(events.begin(), events.end(), [](Event lhs, Event rhs) {
    return static_cast<int>(lhs) < static_cast<int>(rhs);
  }));
}

TEST(GoalRejectionCache, KeepsOtherGoalsPendingAndBoundsStorage)
{
  bringup::GoalRejectionCache cache(2);
  ASSERT_TRUE(cache.remember(1, 10, "old goal"));
  ASSERT_TRUE(cache.remember(2, 20, "current goal"));
  auto current = cache.takeForGoal(2, 20);
  ASSERT_EQ(current.size(), 1U);
  EXPECT_EQ(current.front().message, "current goal");
  EXPECT_EQ(cache.size(), 1U);

  ASSERT_TRUE(cache.remember(3, 30, "later goal"));
  ASSERT_TRUE(cache.remember(4, 40, "newest goal"));
  EXPECT_EQ(cache.size(), 2U);
  EXPECT_TRUE(cache.takeForGoal(1, 10).empty());
  auto newest = cache.takeForGoal(4, 40);
  ASSERT_EQ(newest.size(), 1U);
  EXPECT_EQ(newest.front().message, "newest goal");
}

TEST(GoalRejectionCache, LateGoalStatusAndAbortPreserveCachedRejectionText)
{
  constexpr std::int32_t goal_stamp_sec = 410;
  constexpr std::uint32_t goal_stamp_nanosec = 120000000;
  constexpr const char * diagnostic =
    "category=static_goal_unknown goal_stamp=410.120000000";
  constexpr const char * rejection_text =
    "目标 (4.330, 3.442)：目标位于未知区域；未知区域限制保持生效。";

  bringup::GoalRejectionCache cache;
  ASSERT_TRUE(cache.remember(goal_stamp_sec, goal_stamp_nanosec, diagnostic));
  bool goal_status_received = false;
  bool identity_received = false;
  bringup::NavigationGoalStatusText display;
  const auto replayIfReady = [&]() {
      if (!goal_status_received || !identity_received) return;
      for (const auto & entry : cache.takeForGoal(goal_stamp_sec, goal_stamp_nanosec)) {
        ASSERT_EQ(entry.message, diagnostic);
        display.showRejection(rejection_text);
      }
    };

  identity_received = true;
  replayIfReady();
  EXPECT_EQ(cache.size(), 1U);

  goal_status_received = true;
  display.beginGoal("导航目标已更新，等待本目标规划结果");
  replayIfReady();
  EXPECT_FALSE(display.finishIfWaiting(
      "本目标在规划结果返回前失败；请查看本目标拒绝原因"));
  EXPECT_EQ(display.text(), rejection_text);
}
