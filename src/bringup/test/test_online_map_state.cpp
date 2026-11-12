#include <cstddef>
#include <cstdint>
#include <vector>

#include <gtest/gtest.h>
#include <map_msgs/msg/occupancy_grid_update.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <std_msgs/msg/u_int64_multi_array.hpp>

#include "bringup/map_geometry_event.hpp"
#include "bringup/online_map_state.hpp"

namespace
{
nav_msgs::msg::OccupancyGrid makeGrid(
  const std::uint32_t width, const std::uint32_t height, const float resolution,
  const double origin_x, const std::int32_t seconds, const std::uint32_t nanoseconds = 0)
{
  nav_msgs::msg::OccupancyGrid map;
  map.header.frame_id = "odom";
  map.header.stamp.sec = seconds;
  map.header.stamp.nanosec = nanoseconds;
  map.info.width = width;
  map.info.height = height;
  map.info.resolution = resolution;
  map.info.origin.position.x = origin_x;
  map.info.origin.orientation.w = 1.0;
  map.data.assign(static_cast<std::size_t>(width) * height, 0);
  return map;
}

std_msgs::msg::UInt64MultiArray makeEvent(
  const nav_msgs::msg::OccupancyGrid & map, const std::uint64_t epoch,
  const std::uint64_t version, const bringup::MapGeometryEventKind kind)
{
  return bringup::serializeMapGeometryEvent(bringup::makeMapGeometryEvent(
    map, map.header.stamp, epoch, version, kind));
}

void sendFullMap(
  bringup::OnlineMapState & state, const nav_msgs::msg::OccupancyGrid & map,
  const std::uint64_t epoch, const std::uint64_t version)
{
  state.receiveGeometryEvent(makeEvent(map, epoch, version,
      bringup::MapGeometryEventKind::FULL_MAP));
  state.receiveFullMap(map);
}

map_msgs::msg::OccupancyGridUpdate makeUpdate(
  const std::uint32_t x, const std::uint32_t y, const std::uint32_t value,
  const std::int32_t seconds)
{
  map_msgs::msg::OccupancyGridUpdate update;
  update.header.frame_id = "odom";
  update.header.stamp.sec = seconds;
  update.x = x;
  update.y = y;
  update.width = 1;
  update.height = 1;
  update.data = {static_cast<std::int8_t>(value)};
  return update;
}

void sendUpdate(
  bringup::OnlineMapState & state, const map_msgs::msg::OccupancyGridUpdate & update,
  const nav_msgs::msg::OccupancyGrid & geometry_map, const std::uint64_t epoch,
  const std::uint64_t version)
{
  const auto event = bringup::serializeMapGeometryEvent(bringup::makeMapGeometryEvent(
      geometry_map, update.header.stamp, epoch, version,
      bringup::MapGeometryEventKind::INCREMENTAL_UPDATE));
  state.receiveGeometryEvent(event);
  state.receiveUpdate(update);
}
}  // namespace

TEST(OnlineMapState, ExpansionUpdateWaitsForMatchingFullMapAndReplaysAtNewOrigin)
{
  bringup::OnlineMapState state;
  const auto original = makeGrid(100, 20, 0.1F, 0.0, 1);
  sendFullMap(state, original, 7, 1);
  ASSERT_TRUE(state.ready());

  const auto expanded = makeGrid(120, 20, 0.1F, -1.0, 2);
  const auto update = makeUpdate(20, 4, 100, 3);
  sendUpdate(state, update, expanded, 7, 2);

  ASSERT_FALSE(state.ready());
  ASSERT_TRUE(state.map());
  EXPECT_DOUBLE_EQ(state.map()->info.origin.position.x, 0.0);
  EXPECT_EQ(state.map()->data[4U * original.info.width + 20U], 0);

  // The full-map body may arrive before its geometry event on the other topic.
  state.receiveFullMap(expanded);
  EXPECT_FALSE(state.ready());
  state.receiveGeometryEvent(makeEvent(
      expanded, 7, 2, bringup::MapGeometryEventKind::FULL_MAP));

  ASSERT_TRUE(state.ready());
  ASSERT_TRUE(state.map());
  EXPECT_DOUBLE_EQ(state.map()->info.origin.position.x, -1.0);
  EXPECT_EQ(state.map()->data[4U * expanded.info.width + 20U], 100);
  const double cell_center_x = state.map()->info.origin.position.x +
    (20.5 * state.map()->info.resolution);
  EXPECT_NEAR(cell_center_x, 1.05, 1.0e-6);
}

TEST(OnlineMapState, FullMapBodyWaitsForItsGeometryMetadata)
{
  bringup::OnlineMapState state;
  const auto map = makeGrid(12, 8, 0.1F, -0.4, 4);

  state.receiveFullMap(map);

  EXPECT_FALSE(state.ready());
  EXPECT_TRUE(state.hasPendingMessages());
  EXPECT_TRUE(state.synchronizationPending());
  EXPECT_FALSE(state.map());
  state.receiveGeometryEvent(makeEvent(
      map, 3, 1, bringup::MapGeometryEventKind::FULL_MAP));

  ASSERT_TRUE(state.ready());
  ASSERT_TRUE(state.map());
  EXPECT_DOUBLE_EQ(state.map()->info.origin.position.x, -0.4);
}

TEST(OnlineMapState, LateFullMapCannotEraseNewerAppliedUpdate)
{
  bringup::OnlineMapState state;
  const auto baseline = makeGrid(20, 12, 0.1F, -1.0, 10);
  sendFullMap(state, baseline, 4, 2);
  ASSERT_TRUE(state.ready());

  sendUpdate(state, makeUpdate(8, 3, 100, 12), baseline, 4, 2);
  ASSERT_TRUE(state.ready());
  ASSERT_TRUE(state.map());
  ASSERT_EQ(state.map()->data[3U * baseline.info.width + 8U], 100);

  const auto delayed_snapshot = makeGrid(20, 12, 0.1F, -1.0, 11, 500000000U);
  sendFullMap(state, delayed_snapshot, 4, 2);

  ASSERT_TRUE(state.ready());
  ASSERT_TRUE(state.map());
  EXPECT_EQ(state.map()->data[3U * baseline.info.width + 8U], 100);
  EXPECT_EQ(state.map()->header.stamp.sec, 12);
}

TEST(OnlineMapState, FullSnapshotAndLateMetadataDiscardCoveredUpdateBody)
{
  bringup::OnlineMapState state;
  const auto baseline = makeGrid(20, 12, 0.1F, -1.0, 10);
  sendFullMap(state, baseline, 4, 2);
  ASSERT_TRUE(state.ready());

  const auto update = makeUpdate(8, 3, 100, 11);
  state.receiveUpdate(update);
  EXPECT_FALSE(state.ready());

  auto newer_snapshot = makeGrid(20, 12, 0.1F, -1.0, 12);
  newer_snapshot.data[3U * newer_snapshot.info.width + 8U] = 100;
  sendFullMap(state, newer_snapshot, 4, 2);
  EXPECT_FALSE(state.ready());

  const auto update_event = bringup::serializeMapGeometryEvent(
    bringup::makeMapGeometryEvent(newer_snapshot, update.header.stamp, 4, 2,
    bringup::MapGeometryEventKind::INCREMENTAL_UPDATE));
  state.receiveGeometryEvent(update_event);

  ASSERT_TRUE(state.ready());
  ASSERT_TRUE(state.map());
  EXPECT_EQ(state.map()->data[3U * state.map()->info.width + 8U], 100);
}

TEST(OnlineMapState, LateCoveredBodyAfterNewSnapshotDoesNotBlockFutureSnapshots)
{
  bringup::OnlineMapState state;
  const auto baseline = makeGrid(20, 12, 0.1F, -1.0, 10);
  sendFullMap(state, baseline, 4, 2);
  ASSERT_TRUE(state.ready());

  const auto update = makeUpdate(8, 3, 100, 11);
  const auto update_event = bringup::serializeMapGeometryEvent(
    bringup::makeMapGeometryEvent(baseline, update.header.stamp, 4, 2,
    bringup::MapGeometryEventKind::INCREMENTAL_UPDATE));
  state.receiveGeometryEvent(update_event);
  EXPECT_FALSE(state.ready());

  auto newer_snapshot = makeGrid(20, 12, 0.1F, -1.0, 12);
  newer_snapshot.data[3U * newer_snapshot.info.width + 8U] = 100;
  state.receiveFullMap(newer_snapshot);
  EXPECT_FALSE(state.ready());
  state.receiveGeometryEvent(makeEvent(
      newer_snapshot, 4, 2, bringup::MapGeometryEventKind::FULL_MAP));
  ASSERT_TRUE(state.ready());

  // The update body can arrive after its metadata has been covered by F12.
  state.receiveUpdate(update);
  ASSERT_TRUE(state.ready());
  ASSERT_TRUE(state.map());
  EXPECT_EQ(state.map()->header.stamp.sec, 12);
  EXPECT_EQ(state.map()->data[3U * state.map()->info.width + 8U], 100);

  auto latest_snapshot = makeGrid(20, 12, 0.1F, -1.0, 13);
  latest_snapshot.data[3U * latest_snapshot.info.width + 8U] = 100;
  sendFullMap(state, latest_snapshot, 4, 2);
  ASSERT_TRUE(state.ready());
  ASSERT_TRUE(state.map());
  EXPECT_EQ(state.map()->header.stamp.sec, 13);
  EXPECT_EQ(state.map()->data[3U * state.map()->info.width + 8U], 100);
}

TEST(OnlineMapState, QuarantinedUpdateCapacityRetainsResetPublisherBody)
{
  bringup::OnlineMapState state;
  const auto baseline = makeGrid(20, 12, 0.1F, -1.0, 1000);
  sendFullMap(state, baseline, 4, 2);
  ASSERT_TRUE(state.ready());

  std::vector<map_msgs::msg::OccupancyGridUpdate> late_updates;
  late_updates.reserve(bringup::OnlineMapState::kMaximumPendingUpdates);
  for (std::size_t i = 0; i < bringup::OnlineMapState::kMaximumPendingUpdates; ++i) {
    const auto stamp = static_cast<std::int32_t>(900 + i);
    const auto update = makeUpdate(5, 4, 100, stamp);
    state.receiveGeometryEvent(bringup::serializeMapGeometryEvent(
        bringup::makeMapGeometryEvent(baseline, update.header.stamp, 4, 2,
        bringup::MapGeometryEventKind::INCREMENTAL_UPDATE)));
    late_updates.push_back(update);
  }
  for (const auto & update : late_updates) {
    state.receiveUpdate(update);
  }
  ASSERT_TRUE(state.ready());

  const auto restarted_update = late_updates.front();
  state.receiveGeometryEvent(bringup::serializeMapGeometryEvent(
      bringup::makeMapGeometryEvent(baseline, restarted_update.header.stamp, 5, 1,
      bringup::MapGeometryEventKind::INCREMENTAL_UPDATE)));
  EXPECT_FALSE(state.ready());
  EXPECT_TRUE(state.hasPendingMessages());

  const auto restarted_map = makeGrid(20, 12, 0.1F, -1.0, 899);
  sendFullMap(state, restarted_map, 5, 1);
  ASSERT_TRUE(state.ready());
  ASSERT_TRUE(state.map());
  EXPECT_EQ(state.map()->data[4U * state.map()->info.width + 5U], 100);
}

TEST(OnlineMapState, QuarantinedUpdateEvictionWaitsOnlyWhenMetadataConfirmsNewInstance)
{
  bringup::OnlineMapState state;
  const auto baseline = makeGrid(20, 12, 0.1F, -1.0, 1000);
  sendFullMap(state, baseline, 4, 2);
  ASSERT_TRUE(state.ready());

  for (std::int32_t stamp = 900; stamp <= 932; ++stamp) {
    const auto update = makeUpdate(5, 4, 100, stamp);
    state.receiveGeometryEvent(bringup::serializeMapGeometryEvent(
        bringup::makeMapGeometryEvent(baseline, update.header.stamp, 4, 2,
        bringup::MapGeometryEventKind::INCREMENTAL_UPDATE)));
  }
  for (std::int32_t stamp = 900; stamp <= 932; ++stamp) {
    state.receiveUpdate(makeUpdate(5, 4, 100, stamp));
  }
  EXPECT_TRUE(state.ready());

  // The oldest body was evicted from the bounded quarantine. A new epoch's
  // matching metadata makes the missing body relevant and closes the gate.
  const auto restarted_update = makeUpdate(5, 4, 100, 900);
  state.receiveGeometryEvent(bringup::serializeMapGeometryEvent(
      bringup::makeMapGeometryEvent(baseline, restarted_update.header.stamp, 5, 1,
      bringup::MapGeometryEventKind::INCREMENTAL_UPDATE)));
  EXPECT_FALSE(state.ready());

  const auto restarted_snapshot = makeGrid(20, 12, 0.1F, -1.0, 933);
  sendFullMap(state, restarted_snapshot, 5, 1);
  ASSERT_TRUE(state.ready());
  ASSERT_TRUE(state.map());
  EXPECT_EQ(state.map()->header.stamp.sec, 933);
  EXPECT_EQ(state.map()->data[4U * state.map()->info.width + 5U], 0);
}

TEST(OnlineMapState, CoveredMetadataCapacityEvictionDoesNotInvalidateMap)
{
  bringup::OnlineMapState state;
  const auto baseline = makeGrid(20, 12, 0.1F, -1.0, 1000);
  sendFullMap(state, baseline, 4, 2);
  ASSERT_TRUE(state.ready());

  for (std::int32_t stamp = 1; stamp <= 257; ++stamp) {
    const auto update = makeUpdate(1, 1, 100, stamp);
    state.receiveGeometryEvent(bringup::serializeMapGeometryEvent(
        bringup::makeMapGeometryEvent(baseline, update.header.stamp, 4, 2,
        bringup::MapGeometryEventKind::INCREMENTAL_UPDATE)));
  }
  ASSERT_TRUE(state.ready());

  // The newest tombstone is retained; the oldest has been evicted. Only the
  // latter waits for metadata because its publisher instance is unconfirmed.
  state.receiveUpdate(makeUpdate(1, 1, 100, 257));
  EXPECT_TRUE(state.ready());
  state.receiveUpdate(makeUpdate(2, 1, 100, 1));
  EXPECT_FALSE(state.ready());
  EXPECT_TRUE(state.hasPendingMessages());

  const auto evicted_update = makeUpdate(2, 1, 100, 1);
  state.receiveGeometryEvent(bringup::serializeMapGeometryEvent(
      bringup::makeMapGeometryEvent(baseline, evicted_update.header.stamp, 4, 2,
      bringup::MapGeometryEventKind::INCREMENTAL_UPDATE)));
  ASSERT_TRUE(state.ready());
  ASSERT_TRUE(state.map());
  EXPECT_EQ(state.map()->header.stamp.sec, 1000);
  EXPECT_EQ(state.map()->data[1U * state.map()->info.width + 1U], 0);
}

TEST(OnlineMapState, PublisherSwitchAcceptsNewEpochAndRejectsLateOldEpoch)
{
  bringup::OnlineMapState state;
  const auto old_map = makeGrid(16, 10, 0.1F, 0.0, 20);
  sendFullMap(state, old_map, 8, 5);
  sendUpdate(state, makeUpdate(5, 2, 100, 21), old_map, 8, 5);
  ASSERT_TRUE(state.ready());

  // The restarted publisher has a new epoch and a reset source clock.
  const auto restarted_map = makeGrid(18, 10, 0.1F, -0.2, 1);
  const auto restarted_update = makeUpdate(6, 2, 100, 2);
  state.receiveUpdate(restarted_update);
  EXPECT_FALSE(state.ready());
  EXPECT_TRUE(state.hasPendingMessages());

  // A late old-epoch full map must not discard a reset-clock update before
  // that update's geometry event arrives.
  auto latest_old_map = makeGrid(16, 10, 0.1F, 0.0, 22);
  latest_old_map.data[2U * latest_old_map.info.width + 5U] = 100;
  sendFullMap(state, latest_old_map, 8, 5);
  EXPECT_FALSE(state.ready());
  EXPECT_TRUE(state.hasPendingMessages());

  sendFullMap(state, restarted_map, 9, 1);
  ASSERT_TRUE(state.map());
  EXPECT_DOUBLE_EQ(state.map()->info.origin.position.x, -0.2);
  EXPECT_FALSE(state.ready());
  const auto restarted_update_event = bringup::serializeMapGeometryEvent(
    bringup::makeMapGeometryEvent(restarted_map, restarted_update.header.stamp, 9, 1,
    bringup::MapGeometryEventKind::INCREMENTAL_UPDATE));
  state.receiveGeometryEvent(restarted_update_event);
  ASSERT_TRUE(state.ready());
  ASSERT_TRUE(state.map());
  EXPECT_EQ(state.map()->header.stamp.sec, 2);
  EXPECT_EQ(state.map()->data[2U * state.map()->info.width + 6U], 100);

  auto late_old_map = makeGrid(16, 10, 0.1F, 0.0, 24);
  late_old_map.data[2U * late_old_map.info.width + 6U] = 0;
  state.receiveFullMap(late_old_map);
  state.receiveGeometryEvent(makeEvent(
      late_old_map, 8, 5, bringup::MapGeometryEventKind::FULL_MAP));
  ASSERT_TRUE(state.ready());
  ASSERT_TRUE(state.map());
  EXPECT_DOUBLE_EQ(state.map()->info.origin.position.x, -0.2);
  EXPECT_EQ(state.map()->header.stamp.sec, 2);
  EXPECT_EQ(state.map()->data[2U * state.map()->info.width + 6U], 100);

  sendUpdate(state, makeUpdate(4, 4, 100, 25), late_old_map, 8, 5);
  ASSERT_TRUE(state.ready());
  EXPECT_EQ(state.map()->data[4U * state.map()->info.width + 4U], 0);
}

TEST(OnlineMapState, UnpairedStateIsExplicitlyPending)
{
  bringup::OnlineMapState state;
  const auto map = makeGrid(10, 10, 0.1F, 0.0, 30);

  state.receiveFullMap(map);

  EXPECT_FALSE(state.ready());
  EXPECT_TRUE(state.hasPendingMessages());
  EXPECT_TRUE(state.synchronizationPending());
}
