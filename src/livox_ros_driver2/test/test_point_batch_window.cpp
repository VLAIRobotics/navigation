#include <chrono>

#include <gtest/gtest.h>

#include "point_batch_window.h"

using livox_ros::PointBatchWindow;

TEST(PointBatchWindow, RequiresCoverageAndPeriodAfterQueueReset)
{
  using namespace std::chrono;
  const auto start = PointBatchWindow::TimePoint{} + seconds(1);
  const auto coverage = milliseconds(100);
  const auto max_wait = milliseconds(350);
  PointBatchWindow window;

  window.reset(start);
  window.observePacket(start);
  EXPECT_FALSE(window.canPublish(start + milliseconds(200), coverage, coverage, max_wait));

  window.observePacket(start + coverage);
  EXPECT_TRUE(window.canPublish(start + coverage, coverage, coverage, max_wait));

  window.markPublished(start + coverage);
  window.observePacket(start + seconds(1));
  EXPECT_FALSE(window.canPublish(start + seconds(1), coverage, coverage, max_wait));
  EXPECT_FALSE(window.canPublish(start + seconds(1) + milliseconds(99),
      coverage, coverage, max_wait));
  window.observePacket(start + seconds(1) + coverage);
  EXPECT_TRUE(window.canPublish(start + seconds(1) + coverage,
      coverage, coverage, max_wait));
}

TEST(PointBatchWindow, StreamGapResetsTheFirstRecoveryBatch)
{
  using namespace std::chrono;
  const auto start = PointBatchWindow::TimePoint{} + seconds(2);
  const auto max_gap = milliseconds(250);
  const auto max_wait = milliseconds(350);
  const auto coverage = milliseconds(100);
  PointBatchWindow window;

  window.reset(start);
  window.observePacket(start);
  EXPECT_TRUE(window.shouldResetBeforePacket(
      start + milliseconds(300), start + milliseconds(300), max_gap, max_wait));

  window.reset(start + milliseconds(300));
  window.observePacket(start + milliseconds(300));
  EXPECT_FALSE(window.canPublish(
      start + milliseconds(300), coverage, coverage, max_wait));
}

TEST(PointBatchWindow, IncompleteBatchExpiresWithoutPublishing)
{
  using namespace std::chrono;
  const auto start = PointBatchWindow::TimePoint{} + seconds(3);
  const auto max_gap = milliseconds(250);
  const auto max_wait = milliseconds(350);
  const auto coverage = milliseconds(100);
  PointBatchWindow window;

  window.reset(start);
  window.observePacket(start);
  window.observePacket(start + milliseconds(30));
  EXPECT_FALSE(window.canPublish(
      start + max_wait, coverage, coverage, max_wait));
  EXPECT_TRUE(window.shouldResetBeforePacket(
      start + max_wait, start + max_wait, max_gap, max_wait));

  window.reset(start + max_wait);
  window.observePacket(start + max_wait);
  EXPECT_FALSE(window.canPublish(
      start + max_wait, coverage, coverage, max_wait));
}
