#include <gtest/gtest.h>

#include <future>
#include <mutex>
#include <thread>
#include <vector>

#include "fastdem_ros/postprocess_publication_guard.hpp"

TEST(PostprocessPublicationGuard, AllowsSnapshotWhileResetEpochIsUnchanged) {
  EXPECT_TRUE(fastdem::ros2::postprocessSnapshotCanPublish(7, 7, false));
}

TEST(PostprocessPublicationGuard, RejectsSnapshotAfterReset) {
  EXPECT_FALSE(fastdem::ros2::postprocessSnapshotCanPublish(7, 8, false));
}

TEST(PostprocessPublicationGuard, RejectsSnapshotDuringClockRollback) {
  EXPECT_FALSE(fastdem::ros2::postprocessSnapshotCanPublish(7, 7, true));
}

TEST(PostprocessPublicationOrder, NewerResultWinsWhenOlderCalculationFinishesLater) {
  fastdem::ros2::PostprocessPublicationOrder order;
  std::mutex publish_mutex;
  std::promise<void> old_calculation_started;
  auto old_started = old_calculation_started.get_future();
  std::promise<void> release_old_calculation;
  auto release_old = release_old_calculation.get_future();
  std::vector<std::uint64_t> published;

  std::thread older_calculation([&]() {
    old_calculation_started.set_value();
    release_old.wait();
    std::lock_guard lock(publish_mutex);
    if (order.accepts(100)) {
      order.recordPublished(100);
      published.push_back(100);
    }
  });

  old_started.wait();
  {
    std::lock_guard lock(publish_mutex);
    if (order.accepts(200)) {
      order.recordPublished(200);
      published.push_back(200);
    }
  }
  release_old_calculation.set_value();
  older_calculation.join();

  EXPECT_EQ(published, (std::vector<std::uint64_t>{200}));
}

TEST(PostprocessPublicationOrder, OlderStaleSnapshotCannotClearNewerOutput) {
  fastdem::ros2::PostprocessPublicationOrder order;
  order.recordPublished(200);

  EXPECT_FALSE(order.canClearStaleSnapshot(100, 4, 4));
}

TEST(PostprocessPublicationOrder, AllowsRepeatedSourceStamp) {
  fastdem::ros2::PostprocessPublicationOrder order;
  order.recordPublished(200);

  EXPECT_TRUE(order.accepts(200));
}

TEST(PostprocessPublicationOrder, StaleSnapshotCannotClearAfterMapAdvanced) {
  fastdem::ros2::PostprocessPublicationOrder order;

  EXPECT_FALSE(order.canClearStaleSnapshot(100, 4, 5));
}
