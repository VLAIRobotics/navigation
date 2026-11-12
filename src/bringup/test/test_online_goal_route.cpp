#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

#include <gtest/gtest.h>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include "bringup/online_goal_route.hpp"

namespace
{
nav_msgs::msg::OccupancyGrid makeGrid(
  const std::uint32_t width, const std::uint32_t height, const float resolution,
  const double origin_x, const double origin_y, const std::int8_t value)
{
  nav_msgs::msg::OccupancyGrid grid;
  grid.header.frame_id = "odom";
  grid.info.width = width;
  grid.info.height = height;
  grid.info.resolution = resolution;
  grid.info.origin.position.x = origin_x;
  grid.info.origin.position.y = origin_y;
  grid.info.origin.orientation.w = 1.0;
  grid.data.assign(static_cast<std::size_t>(width) * height, value);
  return grid;
}

std::vector<bringup::FrontierFootprintPoint> testFootprint()
{
  return {
    {0.275, 0.225}, {0.275, -0.225}, {-0.35, -0.225}, {-0.35, 0.225}};
}
}  // namespace

TEST(OnlineGoalRoute, OpenMapBoundaryProducesSafeKnownFreeObservationPose)
{
  auto map = makeGrid(60, 60, 0.1F, -3.0, -3.0, 0);
  constexpr double footprint_radius = 0.416;
  const double start_x = 0.05;
  const double start_y = 0.05;

  const auto selected = bringup::online_goal_route::selectFrontierRoute(
    map, start_x, start_y, 10.0, start_y, footprint_radius, testFootprint(), {}, 0.8);

  ASSERT_TRUE(selected);
  ASSERT_FALSE(selected->route.points.empty());
  const auto & observer = selected->frontier;
  EXPECT_GT(observer.x, 2.0);
  EXPECT_LT(observer.x, 2.8);
  EXPECT_NEAR(observer.y, start_y, 0.25);
  EXPECT_NEAR(observer.yaw, 0.0, 0.1);
  const auto cell = bringup::detail::gridCellAtWorld(map, observer.x, observer.y);
  ASSERT_TRUE(cell);
  EXPECT_TRUE(bringup::online_goal_route::circularFootprintClear(
      map, cell->first, cell->second, footprint_radius));
  EXPECT_NEAR(selected->route.points.back().first, observer.x, 1.0e-9);
  EXPECT_NEAR(selected->route.points.back().second, observer.y, 1.0e-9);
}

TEST(OnlineGoalRoute, ClosedOccupiedWallDoesNotExposeUnreachableFrontier)
{
  auto map = makeGrid(80, 80, 0.1F, -4.0, -4.0, -1);
  for (std::int64_t y = 24; y <= 55; ++y) {
    for (std::int64_t x = 24; x <= 55; ++x) {
      const bool wall = x == 24 || x == 55 || y == 24 || y == 55;
      map.data[static_cast<std::size_t>(y) * map.info.width +
        static_cast<std::size_t>(x)] = wall ? 100 : 0;
    }
  }

  const auto selected = bringup::online_goal_route::selectFrontierRoute(
    map, 0.05, 0.05, 3.0, 0.05, 0.416, testFootprint(), {}, 0.8);

  EXPECT_FALSE(selected);
}

TEST(OnlineGoalRoute, StartEscapeAllowsBoundedUnknownWithoutChangingMap)
{
  auto map = makeGrid(60, 60, 0.1F, -3.0, -3.0, 0);
  constexpr double start_x = 0.05;
  constexpr double start_y = 0.05;
  const auto start = bringup::detail::gridCellAtWorld(map, start_x, start_y);
  ASSERT_TRUE(start);
  for (std::int64_t y = 0; y < map.info.height; ++y) {
    for (std::int64_t x = 0; x < map.info.width; ++x) {
      if (bringup::online_goal_route::cellInsideStartEscape(
          map, x, y, start_x, start_y, 0.30))
      {
        map.data[bringup::online_goal_route::flatIndex(map, x, y)] = -1;
      }
    }
  }

  EXPECT_FALSE(bringup::online_goal_route::circularFootprintClear(
      map, start->first, start->second, 0.20));
  const auto route = bringup::online_goal_route::planKnownPath(
    map, start_x, start_y, 1.05, start_y, 0.416, testFootprint(), 10000, 0.50, 0.0);
  ASSERT_TRUE(route);
  EXPECT_EQ(map.data[bringup::online_goal_route::flatIndex(
      map, start->first, start->second)], -1);

  const auto frontier = bringup::online_goal_route::selectFrontierRoute(
    map, start_x, start_y, 10.0, start_y, 0.416, testFootprint(), {}, 0.80,
    10000, 0.50, 0.0);
  ASSERT_TRUE(frontier);
  ASSERT_FALSE(frontier->route.points.empty());
  const auto observation_cell = bringup::detail::gridCellAtWorld(
    map, frontier->frontier.x, frontier->frontier.y);
  ASSERT_TRUE(observation_cell);
  EXPECT_TRUE(bringup::online_goal_route::knownFree(map,
      bringup::online_goal_route::flatIndex(
        map, observation_cell->first, observation_cell->second)));
  EXPECT_TRUE(bringup::online_goal_route::circularFootprintClear(
      map, observation_cell->first, observation_cell->second, 0.20));
  EXPECT_EQ(map.data[bringup::online_goal_route::flatIndex(
      map, start->first, start->second)], -1);

  auto occupied_at_start = map;
  occupied_at_start.data[bringup::online_goal_route::flatIndex(
      occupied_at_start, start->first, start->second)] = 100;
  const auto occupied_check = bringup::online_goal_route::assessCircularFootprint(
    occupied_at_start, start->first, start->second, 0.20,
    start_x, start_y, 0.50);
  EXPECT_FALSE(occupied_check.clear);
  EXPECT_EQ(occupied_check.blocker, bringup::online_goal_route::FootprintBlocker::OCCUPIED);

  auto remote_unknown = makeGrid(60, 60, 0.1F, -3.0, -3.0, 0);
  const auto remote = bringup::detail::gridCellAtWorld(remote_unknown, 1.05, start_y);
  ASSERT_TRUE(remote);
  remote_unknown.data[bringup::online_goal_route::flatIndex(
      remote_unknown, remote->first, remote->second)] = -1;
  const auto remote_check = bringup::online_goal_route::assessCircularFootprint(
    remote_unknown, remote->first, remote->second, 0.10,
    start_x, start_y, 0.50);
  EXPECT_FALSE(remote_check.clear);
  EXPECT_EQ(remote_check.blocker, bringup::online_goal_route::FootprintBlocker::UNKNOWN);
  EXPECT_EQ(remote_check.cell_x, remote->first);
  EXPECT_EQ(remote_check.cell_y, remote->second);
}

TEST(OnlineGoalRoute, ExactFootprintEscapeKeepsOccupiedCellsBlocking)
{
  auto known_map = makeGrid(60, 60, 0.1F, -3.0, -3.0, 0);
  auto known_costmap = known_map;
  const auto start_cell = bringup::detail::gridCellAtWorld(known_map, 0.05, 0.05);
  ASSERT_TRUE(start_cell);
  const auto footprint = testFootprint();
  EXPECT_FALSE(bringup::online_goal_route::footprintTouchesUnknown(
      known_map, 0.05, 0.05, 0.0, footprint));
  EXPECT_FALSE(bringup::online_goal_route::footprintTouchesUnknown(
      known_costmap, 0.05, 0.05, 0.0, footprint));
  known_costmap.data[bringup::online_goal_route::flatIndex(
      known_costmap, start_cell->first, start_cell->second)] = -1;
  EXPECT_TRUE(bringup::online_goal_route::footprintTouchesUnknown(
      known_costmap, 0.05, 0.05, 0.0, footprint));

  auto map = makeGrid(60, 60, 0.1F, -3.0, -3.0, -1);
  auto costmap = map;
  constexpr double start_x = 0.05;
  constexpr double start_y = 0.05;

  const auto escaped = bringup::online_goal_route::assessFootprintWithStartEscape(
    map, costmap, start_x, start_y, 0.0, footprint,
    start_x, start_y, 0.50, 0.50);
  EXPECT_TRUE(escaped.clear);

  const auto start = bringup::detail::gridCellAtWorld(map, start_x, start_y);
  ASSERT_TRUE(start);
  map.data[bringup::online_goal_route::flatIndex(map, start->first, start->second)] = 100;
  const auto blocked = bringup::online_goal_route::assessFootprintWithStartEscape(
    map, costmap, start_x, start_y, 0.0, footprint,
    start_x, start_y, 0.50, 0.50);
  EXPECT_FALSE(blocked.clear);
  EXPECT_EQ(blocked.blocker, bringup::online_goal_route::FootprintBlocker::OCCUPIED);
  EXPECT_EQ(blocked.cell_x, start->first);
  EXPECT_EQ(blocked.cell_y, start->second);

  auto occupied_costmap = costmap;
  occupied_costmap.data[bringup::online_goal_route::flatIndex(
      occupied_costmap, start->first, start->second)] = 100;
  const auto costmap_blocked =
    bringup::online_goal_route::assessFootprintWithStartEscape(
    makeGrid(60, 60, 0.1F, -3.0, -3.0, -1), occupied_costmap,
    start_x, start_y, 0.0, footprint, start_x, start_y, 0.50, 0.50);
  EXPECT_FALSE(costmap_blocked.clear);
  EXPECT_EQ(costmap_blocked.blocker, bringup::online_goal_route::FootprintBlocker::OCCUPIED);

  const auto outside_escape = bringup::online_goal_route::assessFootprintWithStartEscape(
    costmap, costmap, start_x, start_y, 0.0, footprint,
    start_x, start_y, 0.30, 0.30);
  EXPECT_FALSE(outside_escape.clear);
  EXPECT_EQ(outside_escape.blocker, bringup::online_goal_route::FootprintBlocker::UNKNOWN);
}

TEST(OnlineGoalRoute, UnknownOutsideBodyDoesNotBlockRouteClearanceButOccupiedStillDoes)
{
  auto map = makeGrid(60, 60, 0.1F, -3.0, -3.0, 0);
  constexpr double start_x = 0.05;
  constexpr double start_y = 0.05;
  const auto outside_body = bringup::detail::gridCellAtWorld(map, 0.25, 0.35);
  const auto start = bringup::detail::gridCellAtWorld(map, start_x, start_y);
  ASSERT_TRUE(outside_body);
  ASSERT_TRUE(start);
  const auto footprint = testFootprint();
  const auto outside_index = bringup::online_goal_route::flatIndex(
    map, outside_body->first, outside_body->second);
  map.data[outside_index] = -1;

  EXPECT_FALSE(bringup::online_goal_route::footprintTouchesUnknown(
      map, start_x, start_y, 0.0, footprint));
  EXPECT_FALSE(bringup::online_goal_route::assessCircularFootprint(
      map, start->first, start->second, 0.416).clear);
  EXPECT_TRUE(bringup::online_goal_route::assessRouteClearance(
      map, start->first, start->second, 0.416, start_x, start_y, 0.0,
      footprint).clear);

  const auto route = bringup::online_goal_route::planKnownPath(
    map, start_x, start_y, 1.05, start_y, 0.416, footprint, 10000, 0.0, 0.0);
  ASSERT_TRUE(route);

  auto occupied_map = map;
  occupied_map.data[outside_index] = 100;
  const auto occupied_clearance = bringup::online_goal_route::assessRouteClearance(
    occupied_map, start->first, start->second, 0.416, start_x, start_y, 0.0,
    footprint);
  EXPECT_FALSE(occupied_clearance.clear);
  EXPECT_EQ(occupied_clearance.blocker, bringup::online_goal_route::FootprintBlocker::OCCUPIED);
  EXPECT_FALSE(bringup::online_goal_route::planKnownPath(
      occupied_map, start_x, start_y, 1.05, start_y, 0.416, footprint,
      10000, 0.0, 0.0));
}

TEST(OnlineGoalRoute, FrontierSelectionSkipsEscapeUnknownObservationFootprint)
{
  auto map = makeGrid(100, 80, 0.1F, -5.0, -4.0, 0);
  constexpr double start_x = 0.05;
  constexpr double start_y = 0.05;
  const auto start_unknown = bringup::detail::gridCellAtWorld(map, start_x, start_y);
  const auto east_unknown = bringup::detail::gridCellAtWorld(map, 1.05, start_y);
  ASSERT_TRUE(start_unknown);
  ASSERT_TRUE(east_unknown);
  map.data[bringup::online_goal_route::flatIndex(
      map, start_unknown->first, start_unknown->second)] = -1;
  map.data[bringup::online_goal_route::flatIndex(
      map, east_unknown->first, east_unknown->second)] = -1;

  const auto footprint = testFootprint();
  const auto unsafe_candidate = bringup::online_goal_route::assessKnownFreeFootprint(
    map, 0.25, start_y, 0.0, footprint);
  EXPECT_FALSE(unsafe_candidate.clear);
  EXPECT_EQ(unsafe_candidate.blocker, bringup::online_goal_route::FootprintBlocker::UNKNOWN);

  const auto selected = bringup::online_goal_route::selectFrontierRoute(
    map, start_x, start_y, 3.05, start_y, 0.416, footprint, {}, 0.8,
    100000, 0.50, 0.0);
  ASSERT_TRUE(selected);
  const auto selected_assessment = bringup::online_goal_route::assessKnownFreeFootprint(
    map, selected->frontier.x, selected->frontier.y, selected->frontier.yaw, footprint);
  EXPECT_TRUE(selected_assessment.clear);
  const bool selected_the_unknown_candidate =
    selected->frontier.x < 0.50 &&
    std::abs(selected->frontier.y - start_y) < 0.15 &&
    std::abs(std::remainder(selected->frontier.yaw, 6.283185307179586)) < 0.10;
  EXPECT_FALSE(selected_the_unknown_candidate);
}

TEST(OnlineGoalRoute, ExactNonzeroGoalYawIsUsedAtCellCenter)
{
  auto map = makeGrid(80, 80, 0.1F, -3.0, -3.0, 0);
  constexpr double start_x = 0.05;
  constexpr double start_y = 0.05;
  constexpr double goal_x = 0.05;
  constexpr double goal_y = 1.05;
  constexpr double north_yaw = 1.5707963267948966;
  const auto goal = bringup::detail::gridCellAtWorld(map, goal_x, goal_y);
  const auto outside_north_footprint = bringup::detail::gridCellAtWorld(
    map, 0.35, goal_y);
  ASSERT_TRUE(goal);
  ASSERT_TRUE(outside_north_footprint);
  map.data[bringup::online_goal_route::flatIndex(
      map, outside_north_footprint->first, outside_north_footprint->second)] = -1;
  const auto footprint = testFootprint();

  EXPECT_TRUE(bringup::online_goal_route::assessRouteClearance(
      map, goal->first, goal->second, 0.416, goal_x, goal_y, north_yaw,
      footprint).clear);
  EXPECT_FALSE(bringup::online_goal_route::assessRouteClearance(
      map, goal->first, goal->second, 0.416, goal_x, goal_y, 0.0,
      footprint).clear);

  const auto route = bringup::online_goal_route::planKnownPath(
    map, start_x, start_y, goal_x, goal_y, 0.416, footprint,
    10000, 0.0, north_yaw, north_yaw);
  ASSERT_TRUE(route);
  EXPECT_FALSE(bringup::online_goal_route::planKnownPath(
      map, start_x, start_y, goal_x, goal_y, 0.416, footprint,
      10000, 0.0, north_yaw, 0.0));
}

TEST(OnlineGoalRoute, RotationSweepFindsUnknownBetweenSafeEndpointOrientations)
{
  auto map = makeGrid(80, 80, 0.1F, -3.0, -3.0, 0);
  auto costmap = makeGrid(80, 80, 0.1F, -3.0, -3.0, 0);
  constexpr double pose_x = 0.05;
  constexpr double pose_y = 0.05;
  constexpr double east_yaw = 0.0;
  constexpr double north_yaw = 1.5707963267948966;
  const auto swept_unknown = bringup::detail::gridCellAtWorld(map, 0.45, pose_y);
  ASSERT_TRUE(swept_unknown);
  const auto unknown_index = bringup::online_goal_route::flatIndex(
    map, swept_unknown->first, swept_unknown->second);
  map.data[unknown_index] = -1;
  const auto footprint = testFootprint();

  EXPECT_TRUE(bringup::online_goal_route::assessKnownFreeFootprint(
      map, pose_x, pose_y, east_yaw, footprint).clear);
  EXPECT_TRUE(bringup::online_goal_route::assessKnownFreeFootprint(
      map, pose_x, pose_y, north_yaw, footprint).clear);
  const auto blocked_sweep =
    bringup::online_goal_route::assessFootprintRotationSweepWithStartEscape(
    map, costmap, pose_x, pose_y, east_yaw, north_yaw, footprint,
    pose_x, pose_y, 0.0, 0.0);
  EXPECT_FALSE(blocked_sweep.clear);
  EXPECT_EQ(blocked_sweep.blocker, bringup::online_goal_route::FootprintBlocker::UNKNOWN);

  const auto pose_cell = bringup::detail::gridCellAtWorld(map, pose_x, pose_y);
  ASSERT_TRUE(pose_cell);
  const auto offsets = bringup::online_goal_route::footprintRotationCellOffsets(
    map, footprint, east_yaw, north_yaw);
  ASSERT_TRUE(offsets);
  const auto escape = bringup::online_goal_route::makeStartEscapeMask(
    map, pose_x, pose_y, 0.0);
  const auto cached_sweep =
    bringup::online_goal_route::assessFootprintRotationSweepWithOffsets(
    map, pose_cell->first, pose_cell->second, *offsets, escape,
    [](const std::int8_t value) { return value >= 0 && value <= 50; });
  EXPECT_FALSE(cached_sweep.clear);
  EXPECT_EQ(cached_sweep.blocker, bringup::online_goal_route::FootprintBlocker::UNKNOWN);

  const auto escaped_sweep =
    bringup::online_goal_route::assessFootprintRotationSweepWithStartEscape(
    map, costmap, pose_x, pose_y, east_yaw, north_yaw, footprint,
    pose_x, pose_y, 0.5, 0.0);
  EXPECT_TRUE(escaped_sweep.clear);

  map.data[unknown_index] = 100;
  const auto occupied_sweep =
    bringup::online_goal_route::assessFootprintRotationSweepWithStartEscape(
    map, costmap, pose_x, pose_y, east_yaw, north_yaw, footprint,
    pose_x, pose_y, 0.5, 0.0);
  EXPECT_FALSE(occupied_sweep.clear);
  EXPECT_EQ(occupied_sweep.blocker, bringup::online_goal_route::FootprintBlocker::OCCUPIED);
}

TEST(OnlineGoalRoute, CostmapOnlyRotationBlockIsRejectedAtStart)
{
  auto map = makeGrid(80, 80, 0.1F, -3.0, -3.0, 0);
  auto costmap = makeGrid(80, 80, 0.1F, -3.0, -3.0, 0);
  constexpr double pose_x = 0.05;
  constexpr double pose_y = 0.05;
  constexpr double east_yaw = 0.0;
  constexpr double north_yaw = 1.5707963267948966;
  const auto swept_cell = bringup::detail::gridCellAtWorld(costmap, 0.45, pose_y);
  ASSERT_TRUE(swept_cell);
  const auto index = bringup::online_goal_route::flatIndex(
    costmap, swept_cell->first, swept_cell->second);
  const auto footprint = testFootprint();

  EXPECT_TRUE(bringup::online_goal_route::assessKnownFreeFootprint(
      map, pose_x, pose_y, east_yaw, footprint).clear);
  EXPECT_TRUE(bringup::online_goal_route::assessKnownFreeFootprint(
      map, pose_x, pose_y, north_yaw, footprint).clear);
  EXPECT_TRUE(bringup::online_goal_route::assessKnownFreeFootprint(
      costmap, pose_x, pose_y, east_yaw, footprint).clear);
  EXPECT_TRUE(bringup::online_goal_route::assessKnownFreeFootprint(
      costmap, pose_x, pose_y, north_yaw, footprint).clear);
  const auto free_sweep =
    bringup::online_goal_route::assessFootprintRotationSweepWithStartEscape(
    map, costmap, pose_x, pose_y, east_yaw, north_yaw, footprint,
    pose_x, pose_y, 0.0, 0.0);
  EXPECT_TRUE(free_sweep.clear);

  // Keep the persistent map free; only the rolling costmap blocks the sweep.
  costmap.data[index] = -1;
  EXPECT_TRUE(bringup::online_goal_route::assessKnownFreeFootprint(
      map, pose_x, pose_y, east_yaw, footprint).clear);
  EXPECT_TRUE(bringup::online_goal_route::assessKnownFreeFootprint(
      map, pose_x, pose_y, north_yaw, footprint).clear);
  EXPECT_TRUE(bringup::online_goal_route::assessKnownFreeFootprint(
      costmap, pose_x, pose_y, east_yaw, footprint).clear);
  EXPECT_TRUE(bringup::online_goal_route::assessKnownFreeFootprint(
      costmap, pose_x, pose_y, north_yaw, footprint).clear);
  const auto unknown_sweep =
    bringup::online_goal_route::assessFootprintRotationSweepWithStartEscape(
    map, costmap, pose_x, pose_y, east_yaw, north_yaw, footprint,
    pose_x, pose_y, 0.0, 0.0);
  EXPECT_FALSE(unknown_sweep.clear);
  EXPECT_EQ(unknown_sweep.blocker, bringup::online_goal_route::FootprintBlocker::UNKNOWN);

  costmap.data[index] = 100;
  EXPECT_TRUE(bringup::online_goal_route::assessKnownFreeFootprint(
      map, pose_x, pose_y, east_yaw, footprint).clear);
  EXPECT_TRUE(bringup::online_goal_route::assessKnownFreeFootprint(
      map, pose_x, pose_y, north_yaw, footprint).clear);
  EXPECT_TRUE(bringup::online_goal_route::assessKnownFreeFootprint(
      costmap, pose_x, pose_y, east_yaw, footprint).clear);
  EXPECT_TRUE(bringup::online_goal_route::assessKnownFreeFootprint(
      costmap, pose_x, pose_y, north_yaw, footprint).clear);
  const auto occupied_sweep =
    bringup::online_goal_route::assessFootprintRotationSweepWithStartEscape(
    map, costmap, pose_x, pose_y, east_yaw, north_yaw, footprint,
    pose_x, pose_y, 0.5, 0.5);
  EXPECT_FALSE(occupied_sweep.clear);
  EXPECT_EQ(occupied_sweep.blocker, bringup::online_goal_route::FootprintBlocker::OCCUPIED);
}

TEST(OnlineGoalRoute, TurnClearanceUsesSegmentHeadingInsteadOfGoalBearing)
{
  auto map = makeGrid(120, 100, 0.1F, -5.0, -5.0, 0);
  for (double wall_y = -0.25; wall_y <= 0.3501; wall_y += 0.1) {
    for (const double wall_x : {-0.05, 0.05, 0.15}) {
      const auto wall = bringup::detail::gridCellAtWorld(map, wall_x, wall_y);
      ASSERT_TRUE(wall);
      map.data[bringup::online_goal_route::flatIndex(
          map, wall->first, wall->second)] = 100;
    }
  }
  const auto unknown = bringup::detail::gridCellAtWorld(map, -0.25, 0.45);
  ASSERT_TRUE(unknown);
  map.data[bringup::online_goal_route::flatIndex(
      map, unknown->first, unknown->second)] = -1;
  const auto footprint = testFootprint();
  constexpr double turn_x = -0.55;
  constexpr double turn_y = 0.45;
  constexpr double goal_x = 2.05;
  constexpr double goal_y = 0.05;
  constexpr double north_yaw = 1.5707963267948966;
  const auto turn_cell = bringup::detail::gridCellAtWorld(map, turn_x, turn_y);
  ASSERT_TRUE(turn_cell);
  const double goal_bearing = std::atan2(goal_y - turn_y, goal_x - turn_x);

  EXPECT_TRUE(bringup::online_goal_route::assessRouteClearance(
      map, turn_cell->first, turn_cell->second, 0.416, turn_x, turn_y,
      north_yaw, footprint).clear);
  EXPECT_FALSE(bringup::online_goal_route::assessRouteClearance(
      map, turn_cell->first, turn_cell->second, 0.416, turn_x, turn_y,
      goal_bearing, footprint).clear);

  const auto route = bringup::online_goal_route::planKnownPath(
    map, -2.05, 0.05, goal_x, goal_y, 0.416, footprint,
    100000, 0.0, 0.0, 0.0);
  ASSERT_TRUE(route);
  bool has_safe_northbound_turn = false;
  double maximum_y = -std::numeric_limits<double>::infinity();
  for (const auto & point : route->points) {
    maximum_y = std::max(maximum_y, point.second);
    if (std::abs(point.first - turn_x) < 1.0e-6 &&
      point.second >= 0.35 && point.second <= 0.55)
    {
      has_safe_northbound_turn = true;
    }
  }
  EXPECT_TRUE(has_safe_northbound_turn);
  EXPECT_GT(maximum_y, 0.6);
}

TEST(OnlineGoalRoute, FrontierObservationYawIsIndependentOfFinalGoalBearing)
{
  auto map = makeGrid(100, 100, 0.1F, -5.0, -5.0, 0);
  constexpr double start_x = 0.05;
  constexpr double start_y = 0.05;
  constexpr double observation_x = 0.35;
  constexpr double observation_y = 0.05;
  constexpr double target_x = 0.35;
  constexpr double target_y = 3.05;
  const auto east_frontier = bringup::detail::gridCellAtWorld(map, 1.05, start_y);
  const auto north_unknown = bringup::detail::gridCellAtWorld(map, observation_x, 0.35);
  const auto observation = bringup::detail::gridCellAtWorld(
    map, observation_x, observation_y);
  ASSERT_TRUE(east_frontier);
  ASSERT_TRUE(north_unknown);
  ASSERT_TRUE(observation);
  map.data[bringup::online_goal_route::flatIndex(
      map, east_frontier->first, east_frontier->second)] = -1;
  map.data[bringup::online_goal_route::flatIndex(
      map, north_unknown->first, north_unknown->second)] = -1;
  const auto footprint = testFootprint();
  const double final_bearing = std::atan2(
    target_y - observation_y, target_x - observation_x);

  EXPECT_TRUE(bringup::online_goal_route::assessKnownFreeFootprint(
      map, observation_x, observation_y, 0.0, footprint).clear);
  EXPECT_FALSE(bringup::online_goal_route::assessKnownFreeFootprint(
      map, observation_x, observation_y, final_bearing, footprint).clear);

  const auto selected = bringup::online_goal_route::selectFrontierRoute(
    map, start_x, start_y, target_x, target_y, 0.416, footprint, {}, 0.8,
    100000, 0.0, 0.0);
  ASSERT_TRUE(selected);
  EXPECT_NEAR(selected->frontier.yaw, 0.0, 0.1);
  EXPECT_LT(selected->frontier.x, 0.8);
  EXPECT_NEAR(selected->frontier.y, start_y, 0.2);
  EXPECT_TRUE(bringup::online_goal_route::assessKnownFreeFootprint(
      map, selected->frontier.x, selected->frontier.y,
      selected->frontier.yaw, footprint).clear);
}

TEST(OnlineGoalRoute, FrontierProgressRequiresUnknownCellsNearTheSelectedView)
{
  auto before = makeGrid(100, 80, 0.1F, -5.0, -4.0, 0);
  const auto near_cell = bringup::detail::gridCellAtWorld(before, 0.8, 0.0);
  const auto distant_cell = bringup::detail::gridCellAtWorld(before, -3.5, 2.5);
  ASSERT_TRUE(near_cell);
  ASSERT_TRUE(distant_cell);
  before.data[bringup::online_goal_route::flatIndex(
      before, near_cell->first, near_cell->second)] = -1;
  before.data[bringup::online_goal_route::flatIndex(
      before, distant_cell->first, distant_cell->second)] = -1;

  const auto unknown_before = bringup::online_goal_route::unknownCellsAhead(
    before, 0.0, 0.0, 0.0);
  ASSERT_FALSE(unknown_before.empty());

  auto unrelated_change = before;
  unrelated_change.data[bringup::online_goal_route::flatIndex(
      unrelated_change, distant_cell->first, distant_cell->second)] = 0;
  EXPECT_FALSE(bringup::online_goal_route::frontierObservationGained(
      unrelated_change, unknown_before));

  auto local_gain = unrelated_change;
  local_gain.data[bringup::online_goal_route::flatIndex(
      local_gain, near_cell->first, near_cell->second)] = 0;
  EXPECT_TRUE(bringup::online_goal_route::frontierObservationGained(
      local_gain, unknown_before));
}

TEST(OnlineGoalRoute, InvalidPreciseFinalPoseIsNotSelectedOverSafeWaypoint)
{
  auto map = makeGrid(20, 20, 0.5F, 0.0, 0.0, 0);
  auto costmap = makeGrid(100, 100, 0.1F, 0.0, 0.0, 0);
  costmap.data[7U * costmap.info.width + 12U] = 100;
  const auto footprint = testFootprint();

  EXPECT_TRUE(bringup::detail::footprintIsTraversable(
      map, costmap, 0.75, 0.75, 0.0, footprint));
  EXPECT_FALSE(bringup::detail::footprintIsTraversable(
      map, costmap, 0.99, 0.75, 0.0, footprint));
  const auto waypoint_cell = bringup::detail::gridCellAtWorld(map, 0.75, 0.75);
  const auto final_cell = bringup::detail::gridCellAtWorld(map, 0.99, 0.75);
  ASSERT_TRUE(waypoint_cell);
  ASSERT_TRUE(final_cell);
  EXPECT_EQ(waypoint_cell->first, final_cell->first);
  EXPECT_EQ(waypoint_cell->second, final_cell->second);

  const auto final_target = bringup::online_goal_route::validatedDispatchTarget(
    map, costmap, 0.75, 0.75, 0.0, 0.99, 0.75, 0.0, true, footprint);
  EXPECT_FALSE(final_target);

  const auto safe_waypoint = bringup::online_goal_route::validatedDispatchTarget(
    map, costmap, 0.75, 0.75, 0.0, 0.99, 0.75, 0.0, false, footprint);
  ASSERT_TRUE(safe_waypoint);
  EXPECT_FALSE(safe_waypoint->is_final);
  EXPECT_DOUBLE_EQ(safe_waypoint->x, 0.75);
  EXPECT_DOUBLE_EQ(safe_waypoint->y, 0.75);
}
