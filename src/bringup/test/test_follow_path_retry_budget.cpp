#include <chrono>
#include <cstdint>

#include <gtest/gtest.h>

#include "bringup/follow_path_retry_budget.hpp"

using bringup::FollowPathRetryBudget;

TEST(FollowPathRetryBudget, MeasuredProgressReleasesBudgetAndLaterBlockStartsFreshBudget)
{
  using namespace std::chrono_literals;
  const auto start = FollowPathRetryBudget::Clock::time_point{};
  FollowPathRetryBudget budget;
  budget.recordFailure(start);
  EXPECT_EQ(budget.remaining(start, 20s), 20s);
  EXPECT_EQ(budget.remaining(start + 7s, 20s), 13s);
  EXPECT_EQ(budget.remaining(start + 20s, 20s), 0s);

  EXPECT_FALSE(budget.releaseOnProgress(false, 12, 11, start + 3s, 20s));
  EXPECT_FALSE(budget.releaseOnProgress(true, 12, 12, start + 3s, 20s));
  EXPECT_TRUE(budget.releaseOnProgress(false, 12, 12, start + 3s, 20s));
  EXPECT_FALSE(budget.expired(start + 30s, 20s));

  budget.recordFailure(start + 30s);
  EXPECT_FALSE(budget.expired(start + 49s, 20s));
  EXPECT_TRUE(budget.expired(start + 50s, 20s));
  EXPECT_EQ(budget.remaining(start + 50s, 20s), 0s);
}

TEST(FollowPathRetryBudget, ProgressAfterDeadlineDoesNotReleasePermanentBlock)
{
  using namespace std::chrono_literals;
  const auto start = FollowPathRetryBudget::Clock::time_point{};
  FollowPathRetryBudget budget;
  budget.recordFailure(start);

  EXPECT_FALSE(budget.releaseOnProgress(false, 12, 12, start + 20s, 20s));
  EXPECT_TRUE(budget.expired(start + 20s, 20s));
}

TEST(FollowPathRetryBudget, PermanentBlockExpiresWithoutProgress)
{
  using namespace std::chrono_literals;
  const auto start = FollowPathRetryBudget::Clock::time_point{};
  FollowPathRetryBudget budget;
  budget.recordFailure(start);

  EXPECT_FALSE(budget.expired(start + 19999ms, 20s));
  EXPECT_TRUE(budget.expired(start + 20s, 20s));
}
