#include "bringup/static_terrain_map_publisher.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iterator>
#include <filesystem>
#include <string>
#include <vector>

#include <fastdem/elevation_map.hpp>
#include <fastdem/bridge/ros2.hpp>
#include <fastdem/io/npz.hpp>
#include <gtest/gtest.h>
#include <grid_map_ros/GridMapRosConverter.hpp>

#include "bringup/static_pcd_clearance.hpp"

namespace
{

fastdem::ElevationMap makeMap(const std::string & frame)
{
  fastdem::ElevationMap map(1.0F, 1.0F, 0.5F, frame);
  map.add("slope", 0.0F);
  map.add("step_residual", 0.0F);
  map.add("roughness", 0.0F);
  map.add(fastdem::layer::n_points, 0.0F);
  map.add("class", 0.0F);
  map.add("clearance", 0.0F);
  map.get(fastdem::layer::elevation).setConstant(0.1F);
  map.get("slope").setConstant(0.0F);
  map.get("step_residual").setConstant(0.0F);
  map.get("roughness").setConstant(0.0F);
  map.get(fastdem::layer::n_points).setConstant(3.0F);
  map.get("class").setConstant(1.0F);
  map.get("clearance").setConstant(1.0F);
  return map;
}

void setDistinctLayerValues(fastdem::ElevationMap & map)
{
  const auto layers = map.getLayers();
  for (const auto cell : map.cells()) {
    const float cell_value = static_cast<float>(cell.row * map.getSize()(1) + cell.col + 1);
    for (std::size_t layer_index = 0; layer_index < layers.size(); ++layer_index) {
      map.at(layers[layer_index], cell) = cell_value * 100.0F +
        static_cast<float>(layer_index);
    }
  }
}

void expectCellValuesPreserved(
  const fastdem::ElevationMap & expected, const fastdem::ElevationMap & actual)
{
  for (const auto cell : expected.cells()) {
    const auto position = expected.position(cell);
    ASSERT_TRUE(position);
    const auto actual_index = actual.index(*position);
    ASSERT_TRUE(actual_index);
    const auto actual_position = actual.position(*actual_index);
    ASSERT_TRUE(actual_position);
    EXPECT_NEAR(actual_position->x(), position->x(), 1e-8);
    EXPECT_NEAR(actual_position->y(), position->y(), 1e-8);
    for (const auto & layer : expected.getLayers()) {
      const float expected_value = expected.at(layer, cell);
      const float actual_value = actual.at(layer, *actual_index);
      EXPECT_TRUE(actual_value == expected_value ||
        (std::isnan(actual_value) && std::isnan(expected_value))) << layer;
    }
  }
}

nav_msgs::msg::OccupancyGrid makeOccupancyGrid(
  std::uint32_t width, std::uint32_t height, double origin_x, double origin_y)
{
  nav_msgs::msg::OccupancyGrid occupancy;
  occupancy.header.frame_id = "map";
  occupancy.info.width = width;
  occupancy.info.height = height;
  occupancy.info.resolution = 0.5F;
  occupancy.info.origin.position.x = origin_x;
  occupancy.info.origin.position.y = origin_y;
  occupancy.info.origin.orientation.w = 1.0;
  occupancy.data.assign(static_cast<std::size_t>(width) * height, -1);
  return occupancy;
}

void setMapStamp(nav_msgs::msg::OccupancyGrid & map, std::int32_t seconds)
{
  map.header.stamp.sec = seconds;
}

nav_msgs::msg::OccupancyGrid makeExpandedSnapshot(
  double origin_x, double origin_y, std::uint32_t width, std::uint32_t height,
  const nav_msgs::msg::OccupancyGrid & previous)
{
  auto expanded = makeOccupancyGrid(width, height, origin_x, origin_y);
  setMapStamp(expanded, 2);
  for (std::uint32_t y = 0; y < height; ++y) {
    for (std::uint32_t x = 0; x < width; ++x) {
      const auto world_x = static_cast<std::int64_t>(std::llround(origin_x / 0.5)) + x;
      const auto world_y = static_cast<std::int64_t>(std::llround(origin_y / 0.5)) + y;
      const auto index = static_cast<std::size_t>(y) * width + x;
      if (world_x >= 0 && world_x < 2 && world_y >= 0 && world_y < 2) {
        const auto old_index = static_cast<std::size_t>(world_y) * 2 +
          static_cast<std::size_t>(world_x);
        expanded.data[index] = previous.data[old_index];
        if (world_x == 1 && world_y == 1) expanded.data[index] = 0;
      } else {
        // Newly added cells may already be free, but they have no prior full
        // map value and therefore cannot prove a historical PCD point cleared.
        expanded.data[index] = 0;
      }
    }
  }
  return expanded;
}

class StaticTerrainMapPublisherTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    path_ = std::filesystem::temp_directory_path() / "bringup_static_terrain_test.npz";
  }

  void TearDown() override
  {
    std::error_code error;
    std::filesystem::remove(path_, error);
  }

  std::filesystem::path path_;
};

TEST_F(StaticTerrainMapPublisherTest, LoadsRequiredLayersAndFrame)
{
  ASSERT_TRUE(fastdem::io::saveNpz(path_.string(), makeMap("map")));
  fastdem::ElevationMap loaded;
  std::string error;
  EXPECT_TRUE(bringup::loadStaticTerrainMap(path_.string(), "map", loaded, &error)) << error;
  EXPECT_EQ(loaded.getFrameId(), "map");
  EXPECT_TRUE(loaded.exists("slope"));
  EXPECT_TRUE(loaded.exists("step_residual"));
  EXPECT_TRUE(loaded.exists("roughness"));
}

TEST_F(StaticTerrainMapPublisherTest, RejectsWrongFrame)
{
  ASSERT_TRUE(fastdem::io::saveNpz(path_.string(), makeMap("odom")));
  fastdem::ElevationMap loaded;
  std::string error;
  EXPECT_FALSE(bringup::loadStaticTerrainMap(path_.string(), "map", loaded, &error));
  EXPECT_NE(error.find("does not match"), std::string::npos);
}

TEST_F(StaticTerrainMapPublisherTest, RejectsMissingFile)
{
  fastdem::ElevationMap loaded;
  std::string error;
  EXPECT_FALSE(bringup::loadStaticTerrainMap(path_.string(), "map", loaded, &error));
  EXPECT_NE(error.find("failed to load"), std::string::npos);
}

TEST_F(StaticTerrainMapPublisherTest, RejectsMissingRequiredLayer)
{
  fastdem::ElevationMap map(1.0F, 1.0F, 0.5F, "map");
  map.add("slope", 0.0F);
  map.add("step_residual", 0.0F);
  map.add(fastdem::layer::n_points, 3.0F);
  map.get(fastdem::layer::elevation).setConstant(0.1F);
  ASSERT_TRUE(fastdem::io::saveNpz(path_.string(), map));
  fastdem::ElevationMap loaded;
  std::string error;
  EXPECT_FALSE(bringup::loadStaticTerrainMap(path_.string(), "map", loaded, &error));
  EXPECT_NE(error.find("missing layer"), std::string::npos);
}

TEST(StaticPcdClearanceTrackerTest, FullSnapshotsClearOnlyObservedFreeTransitions)
{
  bringup::StaticPcdClearanceTracker tracker("map", 0.0, 0.0, 0.5);
  auto previous = makeOccupancyGrid(2, 2, 0.0, 0.0);
  setMapStamp(previous, 1);
  previous.data[0] = 100;
  previous.data[3] = 100;
  std::vector<bringup::StaticPcdCell> cleared;
  EXPECT_EQ(
    tracker.observe(previous, cleared), bringup::StaticPcdSnapshotResult::INITIALIZED);
  EXPECT_TRUE(cleared.empty());

  auto next = previous;
  setMapStamp(next, 2);
  next.data[3] = 0;
  EXPECT_EQ(tracker.observe(next, cleared), bringup::StaticPcdSnapshotResult::UPDATED);
  ASSERT_EQ(cleared.size(), 1U);
  EXPECT_EQ(cleared.front(), (bringup::StaticPcdCell{1, 1}));
}

TEST(StaticPcdClearanceTrackerTest, StartupEvidenceRequiresMatchedSnapshotAndExplicitFreePriorCell)
{
  bringup::StaticPcdClearanceTracker tracker("map", 0.0, 0.0, 0.5);
  auto map = makeOccupancyGrid(2, 2, 0.0, 0.0);
  auto source = map;
  setMapStamp(map, 8);
  setMapStamp(source, 8);
  map.data = {0, 0, 50, 100};
  source.data = {1, 3, 1, 1};

  std::vector<bringup::StaticPcdCell> cleared;
  std::string error;
  ASSERT_TRUE(tracker.collectPreviouslyClearedPriorCells(map, source, cleared, &error)) << error;
  ASSERT_EQ(cleared.size(), 2U);
  EXPECT_EQ(cleared[0], (bringup::StaticPcdCell{0, 0}));
  EXPECT_EQ(cleared[1], (bringup::StaticPcdCell{1, 0}));

  setMapStamp(source, 9);
  EXPECT_FALSE(tracker.collectPreviouslyClearedPriorCells(map, source, cleared, &error));
  EXPECT_TRUE(cleared.empty());
  EXPECT_NE(error.find("same snapshot"), std::string::npos);

  setMapStamp(source, 8);
  source.info.origin.position.x = 0.5;
  EXPECT_FALSE(tracker.collectPreviouslyClearedPriorCells(map, source, cleared, &error));
  EXPECT_TRUE(cleared.empty());
}

TEST(StaticPcdClearanceTrackerTest, StartupSourcePairsWhenItArrivesBeforeOrAfterTheMap)
{
  bringup::StaticPcdClearanceTracker tracker("map", 0.0, 0.0, 0.5);
  auto map = makeOccupancyGrid(2, 2, 0.0, 0.0);
  setMapStamp(map, 8);
  map.data = {0, 0, 0, 0};
  auto source = map;
  source.data = {1, 0, 0, 0};

  std::vector<bringup::StaticPcdCell> cleared;
  std::string error;

  // Source first: accepting its matching map must retain it for startup evidence.
  auto source_first_cache = source;
  EXPECT_FALSE(bringup::shouldDiscardStaticPcdOccupancySource(map, source_first_cache));
  ASSERT_TRUE(tracker.collectPreviouslyClearedPriorCells(
      map, source_first_cache, cleared, &error)) << error;
  ASSERT_EQ(cleared.size(), 1U);
  EXPECT_EQ(cleared.front(), (bringup::StaticPcdCell{0, 0}));

  // Map first: the subsequently received matching source is accepted by the same pairing rule.
  auto source_after_map = source;
  EXPECT_TRUE(bringup::staticPcdSameSnapshot(map, source_after_map));
  ASSERT_TRUE(tracker.collectPreviouslyClearedPriorCells(
      map, source_after_map, cleared, &error)) << error;
  ASSERT_EQ(cleared.size(), 1U);
  EXPECT_EQ(cleared.front(), (bringup::StaticPcdCell{0, 0}));

  auto old_source = source;
  setMapStamp(old_source, 7);
  EXPECT_TRUE(bringup::shouldDiscardStaticPcdOccupancySource(map, old_source));
}

TEST(StaticPcdClearanceTrackerTest, StartupEvidenceUsesFullBaselineAndChecksCurrentMap)
{
  bringup::StaticPcdClearanceTracker tracker("map", 0.0, 0.0, 0.5);
  auto full_map_20 = makeOccupancyGrid(2, 2, 0.0, 0.0);
  setMapStamp(full_map_20, 20);
  full_map_20.data = {0, 100, 100, 100};
  auto source_20 = full_map_20;
  source_20.data = {1, 0, 0, 0};

  std::vector<bringup::StaticPcdCell> changed_cells;
  ASSERT_EQ(
    tracker.observe(full_map_20, changed_cells),
    bringup::StaticPcdSnapshotResult::INITIALIZED);

  // Simulate /map 20 -> /map_updates 21 -> occupancy source 20.
  auto current_map_21 = full_map_20;
  map_msgs::msg::OccupancyGridUpdate update_21;
  update_21.header.frame_id = "map";
  update_21.header.stamp.sec = 21;
  update_21.x = 0;
  update_21.y = 0;
  update_21.width = 1;
  update_21.height = 1;
  update_21.data = {0};
  const auto update_event_21 = bringup::makeMapGeometryEvent(
    full_map_20, update_21.header.stamp, 22, 1,
    bringup::MapGeometryEventKind::INCREMENTAL_UPDATE);
  ASSERT_TRUE(bringup::applyStaticPcdMapUpdate(
      current_map_21, update_21, update_event_21, 22, 1));
  ASSERT_EQ(
    tracker.observe(current_map_21, changed_cells),
    bringup::StaticPcdSnapshotResult::UPDATED);

  std::vector<bringup::StaticPcdCell> candidates;
  std::string error;
  ASSERT_TRUE(tracker.collectStartupPriorClearanceCandidates(
      full_map_20, source_20, current_map_21, candidates, &error)) << error;
  ASSERT_EQ(candidates.size(), 1U);
  EXPECT_EQ(candidates.front(), (bringup::StaticPcdCell{0, 0}));

  auto occupied_current_map = current_map_21;
  occupied_current_map.data[0] = 100;
  ASSERT_TRUE(tracker.collectStartupPriorClearanceCandidates(
      full_map_20, source_20, occupied_current_map, candidates, &error)) << error;
  EXPECT_TRUE(candidates.empty());
}

TEST(StaticPcdHeightClearanceTest, ClearsOnlyTheHeightVoxelCrossedByObservedRay)
{
  const bringup::StaticPcdRay3D ray{0.0, 0.0, 0.0, 2.0, 0.0, 2.0};
  EXPECT_TRUE(bringup::staticPcdRayClearsPoint(ray, 1.0, 0.0, 1.0, 0.10, 0.10));
  EXPECT_FALSE(bringup::staticPcdRayClearsPoint(ray, 1.0, 0.0, 1.2, 0.10, 0.10));
  EXPECT_FALSE(bringup::staticPcdRayClearsPoint(ray, 2.0, 0.0, 2.0, 0.10, 0.10));
}

TEST(StaticPcdHeightClearanceTest, RequiresDistinctRecentScansBeforeHidingAPoint)
{
  bringup::StaticPcdHeightClearanceEvidence evidence;
  constexpr std::uint64_t kGap = 500000000ULL;
  EXPECT_FALSE(evidence.observe(1000000000ULL, kGap, 3));
  EXPECT_FALSE(evidence.observe(1000000000ULL, kGap, 3));
  EXPECT_FALSE(evidence.observe(1100000000ULL, kGap, 3));
  EXPECT_TRUE(evidence.observe(1200000000ULL, kGap, 3));
  EXPECT_TRUE(evidence.cleared());

  bringup::StaticPcdHeightClearanceEvidence interrupted;
  EXPECT_FALSE(interrupted.observe(1000000000ULL, kGap, 3));
  EXPECT_FALSE(interrupted.observe(2000000000ULL, kGap, 3));
  EXPECT_EQ(interrupted.observations(), 1U);
}

TEST(StaticPcdHeightClearanceTest, ResetRestoresPreviouslyHiddenPointEvidence)
{
  bringup::StaticPcdHeightClearanceEvidence evidence;
  constexpr std::uint64_t kGap = 500000000ULL;
  EXPECT_FALSE(evidence.observe(1000000000ULL, kGap, 2));
  EXPECT_TRUE(evidence.observe(1100000000ULL, kGap, 2));
  ASSERT_TRUE(evidence.cleared());

  evidence.reset();
  EXPECT_FALSE(evidence.cleared());
  EXPECT_EQ(evidence.observations(), 0U);
  EXPECT_FALSE(evidence.observe(1200000000ULL, kGap, 2));
  EXPECT_TRUE(evidence.observe(1300000000ULL, kGap, 2));
}

TEST(StaticPcdClearanceTrackerTest, RevokesCandidatesOutsideMapOrInNonFreeCells)
{
  bringup::StaticPcdClearanceTracker tracker("map", 0.0, 0.0, 0.5);
  auto snapshot = makeOccupancyGrid(2, 2, 0.0, 0.0);
  setMapStamp(snapshot, 1);
  snapshot.data = {0, -1, 100, 0};
  const std::vector<bringup::StaticPcdCell> candidates{
    {0, 0}, {1, 0}, {0, 1}, {5, 5}};
  std::vector<bringup::StaticPcdCell> revoked;
  std::string error;

  ASSERT_TRUE(tracker.collectCandidatesNoLongerFree(snapshot, candidates, revoked, &error))
    << error;
  ASSERT_EQ(revoked.size(), 3U);
  EXPECT_EQ(revoked[0], (bringup::StaticPcdCell{1, 0}));
  EXPECT_EQ(revoked[1], (bringup::StaticPcdCell{0, 1}));
  EXPECT_EQ(revoked[2], (bringup::StaticPcdCell{5, 5}));
}

TEST(StaticPcdClearanceTrackerTest, RequiresFreshEvidenceAfterFreeUnknownOccupiedFree)
{
  bringup::StaticPcdClearanceTracker tracker("map", 0.0, 0.0, 0.5);
  auto snapshot = makeOccupancyGrid(1, 1, 0.0, 0.0);
  snapshot.data[0] = 100;
  setMapStamp(snapshot, 1);
  std::vector<bringup::StaticPcdCell> transitions;
  std::vector<bringup::StaticPcdCell> candidates;
  std::vector<bringup::StaticPcdCell> revoked;
  ASSERT_EQ(
    tracker.observe(snapshot, transitions), bringup::StaticPcdSnapshotResult::INITIALIZED);

  snapshot.data[0] = 0;
  setMapStamp(snapshot, 2);
  ASSERT_EQ(
    tracker.observe(snapshot, transitions), bringup::StaticPcdSnapshotResult::UPDATED);
  ASSERT_EQ(transitions, (std::vector<bringup::StaticPcdCell>{{0, 0}}));
  candidates = transitions;
  ASSERT_TRUE(tracker.collectCandidatesNoLongerFree(snapshot, candidates, revoked));
  EXPECT_TRUE(revoked.empty());

  bringup::StaticPcdHeightClearanceEvidence evidence;
  constexpr std::uint64_t kGap = 500000000ULL;
  EXPECT_FALSE(evidence.observe(1000000000ULL, kGap, 3));
  EXPECT_FALSE(evidence.observe(1100000000ULL, kGap, 3));

  snapshot.data[0] = -1;
  setMapStamp(snapshot, 3);
  ASSERT_EQ(
    tracker.observe(snapshot, transitions), bringup::StaticPcdSnapshotResult::UPDATED);
  ASSERT_TRUE(tracker.collectCandidatesNoLongerFree(snapshot, candidates, revoked));
  ASSERT_EQ(revoked, candidates);
  evidence.reset();
  EXPECT_EQ(evidence.observations(), 0U);
  EXPECT_FALSE(evidence.cleared());

  snapshot.data[0] = 0;
  setMapStamp(snapshot, 4);
  ASSERT_EQ(
    tracker.observe(snapshot, transitions), bringup::StaticPcdSnapshotResult::UPDATED);
  ASSERT_EQ(transitions, (std::vector<bringup::StaticPcdCell>{{0, 0}}));
  candidates = transitions;
  EXPECT_FALSE(evidence.observe(1200000000ULL, kGap, 3));
  EXPECT_FALSE(evidence.observe(1300000000ULL, kGap, 3));
  EXPECT_TRUE(evidence.observe(1400000000ULL, kGap, 3));

  snapshot.data[0] = 100;
  setMapStamp(snapshot, 5);
  ASSERT_EQ(
    tracker.observe(snapshot, transitions), bringup::StaticPcdSnapshotResult::UPDATED);
  ASSERT_TRUE(tracker.collectCandidatesNoLongerFree(snapshot, candidates, revoked));
  ASSERT_EQ(revoked, candidates);
  evidence.reset();
  EXPECT_FALSE(evidence.cleared());

  snapshot.data[0] = 0;
  setMapStamp(snapshot, 6);
  ASSERT_EQ(
    tracker.observe(snapshot, transitions), bringup::StaticPcdSnapshotResult::UPDATED);
  ASSERT_EQ(transitions, (std::vector<bringup::StaticPcdCell>{{0, 0}}));
  EXPECT_FALSE(evidence.observe(1500000000ULL, kGap, 3));
  EXPECT_FALSE(evidence.observe(1600000000ULL, kGap, 3));
  EXPECT_TRUE(evidence.observe(1700000000ULL, kGap, 3));
}

TEST(StaticPcdClearanceTrackerTest, IgnoresOutOfOrderFullSnapshots)
{
  bringup::StaticPcdClearanceTracker tracker("map", 0.0, 0.0, 0.5);
  auto first = makeOccupancyGrid(2, 2, 0.0, 0.0);
  setMapStamp(first, 10);
  first.data[3] = 100;
  std::vector<bringup::StaticPcdCell> cleared;
  ASSERT_EQ(
    tracker.observe(first, cleared), bringup::StaticPcdSnapshotResult::INITIALIZED);

  auto current = first;
  setMapStamp(current, 20);
  current.data[3] = 0;
  ASSERT_EQ(tracker.observe(current, cleared), bringup::StaticPcdSnapshotResult::UPDATED);
  ASSERT_EQ(cleared.size(), 1U);
  EXPECT_EQ(cleared.front(), (bringup::StaticPcdCell{1, 1}));

  auto stale = makeOccupancyGrid(3, 2, -0.5, 0.0);
  setMapStamp(stale, 15);
  stale.data[2] = 100;
  EXPECT_EQ(tracker.observe(stale, cleared), bringup::StaticPcdSnapshotResult::STALE);
  EXPECT_TRUE(cleared.empty());

  setMapStamp(current, 30);
  EXPECT_EQ(tracker.observe(current, cleared), bringup::StaticPcdSnapshotResult::UPDATED);
  EXPECT_TRUE(cleared.empty());
}

TEST(StaticPcdClearanceTrackerTest, ExpansionInEveryDirectionPreservesWorldCellIdentity)
{
  struct Geometry
  {
    double origin_x;
    double origin_y;
    std::uint32_t width;
    std::uint32_t height;
  };
  const std::vector<Geometry> expansions{
    {-0.5, 0.0, 3, 2}, {0.0, 0.0, 3, 2}, {0.0, -0.5, 2, 3}, {0.0, 0.0, 2, 3}};
  for (const auto & geometry : expansions) {
    SCOPED_TRACE(geometry.origin_x);
    bringup::StaticPcdClearanceTracker tracker("map", 0.0, 0.0, 0.5);
    auto previous = makeOccupancyGrid(2, 2, 0.0, 0.0);
    setMapStamp(previous, 1);
    std::fill(previous.data.begin(), previous.data.end(), 0);
    previous.data[3] = 100;
    std::vector<bringup::StaticPcdCell> cleared;
    ASSERT_EQ(
      tracker.observe(previous, cleared), bringup::StaticPcdSnapshotResult::INITIALIZED);

    auto expanded = makeExpandedSnapshot(
      geometry.origin_x, geometry.origin_y, geometry.width, geometry.height, previous);
    ASSERT_EQ(
      tracker.observe(expanded, cleared), bringup::StaticPcdSnapshotResult::UPDATED);
    ASSERT_EQ(cleared.size(), 1U);
    EXPECT_EQ(cleared.front(), (bringup::StaticPcdCell{1, 1}));
  }
}

TEST(StaticPcdRefreshStateTest, ReplaysFullStateForLateAndAdditionalSubscribers)
{
  using Clock = std::chrono::steady_clock;
  const auto t0 = Clock::time_point{} + std::chrono::seconds(1);
  bringup::StaticPcdRefreshState state;
  EXPECT_FALSE(state.observeSubscribers({}, t0).should_publish);

  auto plan = state.observeSubscribers({"rviz-a"}, t0);
  EXPECT_TRUE(plan.should_publish);
  EXPECT_TRUE(plan.full_refresh);
  state.markPublished(t0);
  EXPECT_FALSE(state.observeSubscribers(
      {"rviz-a"}, t0 + std::chrono::milliseconds(100)).should_publish);

  state.markDirty();
  plan = state.observeSubscribers({"rviz-a"}, t0 + std::chrono::seconds(1));
  EXPECT_TRUE(plan.should_publish);
  EXPECT_TRUE(plan.full_refresh);
  state.markPublished(t0 + std::chrono::seconds(1));

  EXPECT_FALSE(state.observeSubscribers(
      {}, t0 + std::chrono::milliseconds(1100)).should_publish);
  plan = state.observeSubscribers({"rviz-a"}, t0 + std::chrono::seconds(2));
  EXPECT_TRUE(plan.should_publish);
  EXPECT_TRUE(plan.full_refresh);
  state.markPublished(t0 + std::chrono::seconds(2));

  plan = state.observeSubscribers({"rviz-a", "rviz-b"}, t0 + std::chrono::seconds(3));
  EXPECT_TRUE(plan.should_publish);
  EXPECT_TRUE(plan.full_refresh);
  state.markPublished(t0 + std::chrono::seconds(3));
  EXPECT_FALSE(state.observeSubscribers(
      {"rviz-b", "rviz-a"}, t0 + std::chrono::milliseconds(3100)).should_publish);
}

TEST(StaticPcdRefreshStateTest, PublishesAffectedTilesAtFiveHzAndFullStateAtOneHz)
{
  using Clock = std::chrono::steady_clock;
  const auto t0 = Clock::time_point{} + std::chrono::seconds(1);
  bringup::StaticPcdRefreshState state;
  auto plan = state.observeSubscribers({"rviz"}, t0);
  ASSERT_TRUE(plan.should_publish);
  ASSERT_TRUE(plan.full_refresh);
  state.markPublished(t0);

  state.markDirty();
  plan = state.observeSubscribers({"rviz"}, t0 + std::chrono::milliseconds(199));
  EXPECT_FALSE(plan.should_publish);
  plan = state.observeSubscribers({"rviz"}, t0 + std::chrono::milliseconds(200));
  EXPECT_TRUE(plan.should_publish);
  EXPECT_FALSE(plan.full_refresh);
  state.markPublished(t0 + std::chrono::milliseconds(200), false);

  plan = state.observeSubscribers({"rviz"}, t0 + std::chrono::milliseconds(400));
  EXPECT_FALSE(plan.should_publish);
  plan = state.observeSubscribers({"rviz"}, t0 + std::chrono::seconds(1));
  EXPECT_TRUE(plan.should_publish);
  EXPECT_TRUE(plan.full_refresh);
}

TEST(StaticPcdMapUpdate, AppliesBoundedIncrementAndRejectsMismatchedFrame)
{
  auto map = makeOccupancyGrid(2, 2, 0.0, 0.0);
  map.data = {100, 100, 100, 100};
  map.header.stamp.sec = 1;
  map_msgs::msg::OccupancyGridUpdate update;
  update.header.frame_id = "map";
  update.header.stamp.sec = 2;
  update.x = 1;
  update.y = 0;
  update.width = 1;
  update.height = 2;
  update.data = {0, 0};
  const auto event = bringup::makeMapGeometryEvent(
    map, update.header.stamp, 9, 1,
    bringup::MapGeometryEventKind::INCREMENTAL_UPDATE);

  ASSERT_TRUE(bringup::applyStaticPcdMapUpdate(map, update, event, 9, 1));
  EXPECT_EQ(map.data, (std::vector<std::int8_t>{100, 0, 100, 0}));
  EXPECT_EQ(map.header.stamp.sec, 2);

  update.header.frame_id = "odom";
  EXPECT_FALSE(bringup::applyStaticPcdMapUpdate(map, update, event, 9, 1));
  EXPECT_EQ(map.data, (std::vector<std::int8_t>{100, 0, 100, 0}));
}

TEST(StaticPcdMapUpdate, RequiresMatchingGeometryVersionAndFullMapBaseline)
{
  auto map = makeOccupancyGrid(2, 2, 0.0, 0.0);
  map.data = {100, 100, 100, 100};
  map.header.stamp.sec = 10;
  auto expanded = makeOccupancyGrid(4, 2, -0.5, 0.0);
  expanded.header.stamp.sec = 12;
  map_msgs::msg::OccupancyGridUpdate update;
  update.header.frame_id = "map";
  update.header.stamp.sec = 11;
  update.x = 1;
  update.y = 0;
  update.width = 1;
  update.height = 1;
  update.data = {0};

  const auto future_geometry_event = bringup::makeMapGeometryEvent(
    expanded, update.header.stamp, 9, 2,
    bringup::MapGeometryEventKind::INCREMENTAL_UPDATE);
  EXPECT_EQ(
    bringup::assessStaticPcdMapUpdate(map, update, future_geometry_event, 9, 1, 10ULL * 1000000000ULL),
    bringup::StaticPcdMapUpdateDisposition::WAIT_FOR_FULL_MAP);
  EXPECT_FALSE(bringup::applyStaticPcdMapUpdate(map, update, future_geometry_event, 9, 1));
  EXPECT_EQ(map.data, (std::vector<std::int8_t>{100, 100, 100, 100}));

  auto shifted_map = map;
  shifted_map.info.origin.position.x = -0.5;
  const auto wrong_origin_event = bringup::makeMapGeometryEvent(
    shifted_map, update.header.stamp, 9, 1,
    bringup::MapGeometryEventKind::INCREMENTAL_UPDATE);
  EXPECT_EQ(
    bringup::assessStaticPcdMapUpdate(map, update, wrong_origin_event, 9, 1,
      10ULL * 1000000000ULL),
    bringup::StaticPcdMapUpdateDisposition::REJECT_MISMATCH);

  const auto expanded_full_event = bringup::makeMapGeometryEvent(
    expanded, expanded.header.stamp, 9, 2, bringup::MapGeometryEventKind::FULL_MAP);
  EXPECT_EQ(
    bringup::assessStaticPcdFullMap(expanded, expanded_full_event, 9, 1, 0, 0),
    bringup::StaticPcdFullMapDisposition::ACCEPT_NEW_GEOMETRY);

  update.header.stamp.sec = 13;
  const auto old_geometry_event = bringup::makeMapGeometryEvent(
    map, update.header.stamp, 9, 1,
    bringup::MapGeometryEventKind::INCREMENTAL_UPDATE);
  EXPECT_EQ(
    bringup::assessStaticPcdMapUpdate(
      expanded, update, old_geometry_event, 9, 2, 12ULL * 1000000000ULL),
    bringup::StaticPcdMapUpdateDisposition::REJECT_STALE);
}

TEST(StaticPcdMapUpdate, DelayedFullSnapshotCannotReplaceNewerDeltaState)
{
  auto delayed_full = makeOccupancyGrid(2, 2, 0.0, 0.0);
  setMapStamp(delayed_full, 11);
  const auto delayed_event = bringup::makeMapGeometryEvent(
    delayed_full, delayed_full.header.stamp, 15, 1,
    bringup::MapGeometryEventKind::FULL_MAP);
  EXPECT_EQ(
    bringup::assessStaticPcdFullMap(
      delayed_full, delayed_event, 15, 1, 10ULL * 1000000000ULL,
      12ULL * 1000000000ULL),
    bringup::StaticPcdFullMapDisposition::ADVANCE_WATERMARK_ONLY);

  auto changed_geometry_full = delayed_full;
  changed_geometry_full.info.origin.position.x = -0.5;
  setMapStamp(changed_geometry_full, 13);
  const auto changed_same_version_event = bringup::makeMapGeometryEvent(
    changed_geometry_full, changed_geometry_full.header.stamp, 15, 1,
    bringup::MapGeometryEventKind::FULL_MAP);
  EXPECT_EQ(
    bringup::assessStaticPcdFullMap(
      changed_geometry_full, changed_same_version_event, 15, 1,
      10ULL * 1000000000ULL, 12ULL * 1000000000ULL, &delayed_full),
    bringup::StaticPcdFullMapDisposition::REJECT_MISMATCH);

  auto stale_full = delayed_full;
  setMapStamp(stale_full, 9);
  const auto stale_event = bringup::makeMapGeometryEvent(
    stale_full, stale_full.header.stamp, 15, 1,
    bringup::MapGeometryEventKind::FULL_MAP);
  EXPECT_EQ(
    bringup::assessStaticPcdFullMap(
      stale_full, stale_event, 15, 1, 10ULL * 1000000000ULL,
      12ULL * 1000000000ULL),
    bringup::StaticPcdFullMapDisposition::REJECT_STALE);
}

TEST(StaticPcdMapUpdate, LateCoveredDeltaCannotBlockUpdateAfterNewFullMap)
{
  constexpr std::uint64_t kOldBaseline = 10ULL * 1000000000ULL;
  constexpr std::uint64_t kOldUpdate = 11ULL * 1000000000ULL;
  constexpr std::uint64_t kNewFullMap = 20ULL * 1000000000ULL;
  constexpr std::uint64_t kNewUpdate = 21ULL * 1000000000ULL;

  // Old update metadata arrives first and waits, then a full snapshot covers it.
  EXPECT_FALSE(bringup::staticPcdMapUpdateCoveredByState(
      kOldUpdate, kOldBaseline, kOldBaseline));
  EXPECT_TRUE(bringup::staticPcdMapUpdateCoveredByState(
      kOldUpdate, kNewFullMap, kNewFullMap));
  // Its late body is dropped before the queue waits for the now-discarded metadata.
  EXPECT_TRUE(bringup::staticPcdMapUpdateCoveredByState(
      kOldUpdate, kNewFullMap, kNewFullMap));

  auto map = makeOccupancyGrid(2, 2, 0.0, 0.0);
  setMapStamp(map, 20);
  map.data = {100, 100, 100, 100};
  map_msgs::msg::OccupancyGridUpdate update;
  update.header.frame_id = "map";
  update.header.stamp.sec = 21;
  update.x = 1;
  update.y = 0;
  update.width = 1;
  update.height = 1;
  update.data = {0};
  const auto event = bringup::makeMapGeometryEvent(
    map, update.header.stamp, 22, 1,
    bringup::MapGeometryEventKind::INCREMENTAL_UPDATE);
  EXPECT_FALSE(bringup::staticPcdMapUpdateCoveredByState(
      kNewUpdate, kNewFullMap, kNewFullMap));
  EXPECT_EQ(
    bringup::assessStaticPcdMapUpdate(map, update, event, 22, 1, kNewFullMap),
    bringup::StaticPcdMapUpdateDisposition::READY);
}

TEST(
  StaticPcdRefreshStateTest,
  KeepsLastStateCompleteWhenGraphDiscoveryPrecedesMatching)
{
  using Clock = std::chrono::steady_clock;
  const auto t0 = Clock::time_point{} + std::chrono::seconds(1);
  bringup::StaticPcdRefreshState state;
  auto plan = state.observeSubscribers({"rviz-old-gid"}, t0);
  ASSERT_TRUE(plan.should_publish);
  ASSERT_TRUE(plan.full_refresh);
  state.markPublished(t0);

  state.markDirty();
  plan = state.observeSubscribers({"rviz-old-gid"}, t0 + std::chrono::seconds(1));
  ASSERT_TRUE(plan.should_publish);
  ASSERT_TRUE(plan.full_refresh);
  state.markPublished(t0 + std::chrono::seconds(1));

  // Graph discovery can expose the new GID before DDS has matched the reader.
  plan = state.observeSubscribers(
    {"rviz-new-gid"}, t0 + std::chrono::milliseconds(1500));
  EXPECT_FALSE(plan.should_publish);
  EXPECT_FALSE(plan.full_refresh);
  plan = state.observeSubscribers(
    {"rviz-new-gid"}, t0 + std::chrono::milliseconds(1999));
  EXPECT_FALSE(plan.should_publish);
  EXPECT_FALSE(plan.full_refresh);
  plan = state.observeSubscribers({"rviz-new-gid"}, t0 + std::chrono::seconds(2));
  EXPECT_TRUE(plan.should_publish);
  EXPECT_TRUE(plan.full_refresh);
  // The retained history contains a full baseline and recent tile updates.
  state.markPublished(t0 + std::chrono::seconds(2));
  state.markDirty();
  plan = state.observeSubscribers(
    {"rviz-new-gid"}, t0 + std::chrono::milliseconds(2100));
  EXPECT_FALSE(plan.should_publish);
  EXPECT_FALSE(plan.full_refresh);
  plan = state.observeSubscribers({"rviz-new-gid"}, t0 + std::chrono::seconds(3));
  EXPECT_TRUE(plan.should_publish);
  EXPECT_TRUE(plan.full_refresh);
  state.markPublished(t0 + std::chrono::seconds(3));
  EXPECT_FALSE(state.observeSubscribers(
      {"rviz-new-gid"}, t0 + std::chrono::milliseconds(3100)).should_publish);

  bringup::StaticPcdRefreshState two_subscribers;
  plan = two_subscribers.observeSubscribers({"rviz-a", "rviz-c"}, t0);
  ASSERT_TRUE(plan.should_publish);
  ASSERT_TRUE(plan.full_refresh);
  two_subscribers.markPublished(t0);
  plan = two_subscribers.observeSubscribers(
    {"rviz-b", "rviz-c"}, t0 + std::chrono::seconds(1));
  EXPECT_TRUE(plan.should_publish);
  EXPECT_TRUE(plan.full_refresh);
}

TEST_F(StaticTerrainMapPublisherTest, PublishesStandardColumnMajorLayout)
{
  auto map = makeMap("map");
  map.setGeometry(1.5F, 1.0F, 0.5F);
  auto & elevation = map.get(fastdem::layer::elevation);
  for (Eigen::Index col = 0; col < elevation.cols(); ++col) {
    for (Eigen::Index row = 0; row < elevation.rows(); ++row) {
      elevation(row, col) = static_cast<float>(100 * col + row);
    }
  }
  const auto message = fastdem::ros2::toGridMap(map);
  ASSERT_FALSE(message.data.empty());
  const auto layer_index = static_cast<std::size_t>(std::distance(
    message.layers.begin(),
    std::find(message.layers.begin(), message.layers.end(), fastdem::layer::elevation)));
  ASSERT_LT(layer_index, message.layers.size());
  const auto & matrix = map.get(fastdem::layer::elevation);
  const auto & layout = message.data[layer_index].layout;
  ASSERT_EQ(layout.dim.size(), 2U);
  EXPECT_EQ(layout.dim[0].label, "column_index");
  EXPECT_EQ(layout.dim[0].size, static_cast<std::uint32_t>(matrix.cols()));
  EXPECT_EQ(layout.dim[0].stride, static_cast<std::uint32_t>(matrix.size()));
  EXPECT_EQ(layout.dim[1].label, "row_index");
  EXPECT_EQ(layout.dim[1].size, static_cast<std::uint32_t>(matrix.rows()));
  EXPECT_EQ(layout.dim[1].stride, static_cast<std::uint32_t>(matrix.rows()));
  ASSERT_NE(layout.dim[0].size, layout.dim[1].size);
  ASSERT_EQ(message.data[layer_index].data.size(), static_cast<std::size_t>(matrix.size()));
  for (Eigen::Index index = 0; index < matrix.size(); ++index) {
    EXPECT_FLOAT_EQ(message.data[layer_index].data[static_cast<std::size_t>(index)], matrix.data()[index]);
  }

  const auto round_trip = nanogrid::ros2::fromMsg(message);
  EXPECT_EQ(round_trip.getSize()(0), map.getSize()(0));
  EXPECT_EQ(round_trip.getSize()(1), map.getSize()(1));
  EXPECT_TRUE(round_trip.get(fastdem::layer::elevation).isApprox(matrix));

  grid_map::GridMap official_round_trip;
  ASSERT_TRUE(grid_map::GridMapRosConverter::fromMessage(message, official_round_trip));
  ASSERT_EQ(official_round_trip.getSize()(0), matrix.rows());
  ASSERT_EQ(official_round_trip.getSize()(1), matrix.cols());
  EXPECT_TRUE(official_round_trip.get(fastdem::layer::elevation).isApprox(matrix));
}

TEST(StaticTerrainDisplayMapTest, SuppressesSingleSpikeAndRetainsStepPlateau)
{
  auto map = makeMap("map");
  map.setGeometry(2.5F, 2.5F, 0.5F);
  map.get(fastdem::layer::elevation).setConstant(0.1F);
  map.get("class").setConstant(1.0F);
  map.get(fastdem::layer::elevation)(1, 1) = 0.7F;
  for (int row = 2; row <= 3; ++row) {
    for (int col = 2; col <= 3; ++col) {
      map.get(fastdem::layer::elevation)(row, col) = 0.6F;
    }
  }
  EXPECT_EQ(bringup::prepareStaticDisplayMap(map), 1U);
  EXPECT_FLOAT_EQ(map.get(fastdem::layer::elevation)(1, 1), 0.1F);
  EXPECT_FLOAT_EQ(map.get(fastdem::layer::elevation)(2, 2), 0.6F);
  EXPECT_TRUE(map.exists("display_relief"));
}

TEST(StaticTerrainDisplayMapTest, FillsObstacleHoleFromNearestGroundWithoutChangingMapData)
{
  auto map = makeMap("map");
  map.setGeometry(4.0F, 4.0F, 0.5F);
  auto & elevation = map.get(fastdem::layer::elevation);
  auto & classes = map.get("class");
  for (Eigen::Index row = 0; row < elevation.rows(); ++row) {
    for (Eigen::Index col = 0; col < elevation.cols(); ++col) {
      elevation(row, col) = row <= 4 ? 0.1F : 0.9F;
      classes(row, col) = 1.0F;
    }
  }
  elevation(4, 4) = NAN;
  classes(4, 4) = 2.0F;

  EXPECT_EQ(bringup::prepareStaticDisplayMap(map), 0U);

  ASSERT_TRUE(map.exists("display_elevation"));
  EXPECT_FLOAT_EQ(map.get("display_elevation")(4, 4), 0.1F);
  EXPECT_TRUE(std::isnan(map.get(fastdem::layer::elevation)(4, 4)));
  EXPECT_FLOAT_EQ(map.get("class")(4, 4), 2.0F);
}

TEST(StaticTerrainDisplayMapTest, FillsUnknownHoleAtAHeightDiscontinuityForDisplayOnly)
{
  auto map = makeMap("map");
  map.setGeometry(4.0F, 4.0F, 0.5F);
  auto & elevation = map.get(fastdem::layer::elevation);
  auto & classes = map.get("class");
  for (Eigen::Index row = 0; row < elevation.rows(); ++row) {
    for (Eigen::Index col = 0; col < elevation.cols(); ++col) {
      elevation(row, col) = row <= 4 ? 0.1F : 0.9F;
      classes(row, col) = 1.0F;
    }
  }
  elevation(4, 4) = NAN;
  classes(4, 4) = 0.0F;

  EXPECT_EQ(bringup::prepareStaticDisplayMap(map), 0U);

  EXPECT_TRUE(std::isfinite(map.get("display_elevation")(4, 4)));
  EXPECT_TRUE(std::isnan(map.get(fastdem::layer::elevation)(4, 4)));
  EXPECT_FLOAT_EQ(map.get("class")(4, 4), 0.0F);
}

TEST(StaticTerrainDisplayMapTest, HighlightsSlopeAndStepWithoutChangingHeight)
{
  auto map = makeMap("map");
  map.setGeometry(1.5F, 1.5F, 0.5F);
  map.get(fastdem::layer::elevation).setConstant(0.1F);
  map.get("slope")(1, 0) = 20.0F;
  map.get("step_residual")(1, 1) = 0.18F;
  EXPECT_EQ(bringup::prepareStaticDisplayMap(map), 0U);
  const auto & relief = map.get("display_relief");
  EXPECT_GT(relief(1, 0), relief(0, 0));
  EXPECT_GT(relief(1, 1), relief(0, 0));
  EXPECT_FLOAT_EQ(map.get(fastdem::layer::elevation)(1, 0), 0.1F);
  EXPECT_FLOAT_EQ(map.get(fastdem::layer::elevation)(1, 1), 0.1F);
}

TEST(StaticTerrainDisplayMapTest, ExpandsToFusedMapWithoutMovingPriorCells)
{
  auto terrain = makeMap("map");
  ASSERT_EQ(bringup::prepareStaticDisplayMap(terrain), 0U);
  nav_msgs::msg::OccupancyGrid occupancy;
  occupancy.header.frame_id = "map";
  occupancy.info.width = 4;
  occupancy.info.height = 2;
  occupancy.info.resolution = 0.5F;
  occupancy.info.origin.position.x = -1.0;
  occupancy.info.origin.position.y = -0.5;
  occupancy.info.origin.orientation.w = 1.0;
  occupancy.data.assign(8, -1);
  bool changed = false;
  std::string error;

  ASSERT_TRUE(bringup::extendDisplayMapToOccupancyGrid(terrain, occupancy, &changed, &error))
    << error;

  EXPECT_TRUE(changed);
  EXPECT_EQ(terrain.getSize()(0), 4);
  EXPECT_EQ(terrain.getSize()(1), 2);
  ASSERT_TRUE(terrain.index(nanogrid::Position(0.0, 0.0)));
  EXPECT_FLOAT_EQ(terrain.elevationAt(nanogrid::Position(0.0, 0.0)), 0.1F);
  EXPECT_FALSE(std::isfinite(terrain.elevationAt(nanogrid::Position(0.75, 0.0))));
}

TEST(StaticTerrainDisplayMapTest, ExpandsOneCellOnNegativeXAndPreservesAllLayers)
{
  auto terrain = makeMap("map");
  ASSERT_EQ(bringup::prepareStaticDisplayMap(terrain), 0U);
  setDistinctLayerValues(terrain);
  const auto before = terrain;
  const auto occupancy = makeOccupancyGrid(3, 2, -1.0, -0.5);
  bool changed = false;
  std::string error;

  ASSERT_TRUE(bringup::extendDisplayMapToOccupancyGrid(terrain, occupancy, &changed, &error))
    << error;

  EXPECT_TRUE(changed);
  EXPECT_EQ(terrain.getSize()(0), 3);
  EXPECT_EQ(terrain.getSize()(1), 2);
  EXPECT_NEAR(terrain.getPosition().x() - terrain.getLength().x() / 2.0, -1.0, 1e-8);
  EXPECT_NEAR(terrain.getPosition().y() - terrain.getLength().y() / 2.0, -0.5, 1e-8);
  expectCellValuesPreserved(before, terrain);
  const auto new_cell = terrain.index(nanogrid::Position(-0.75, -0.25));
  ASSERT_TRUE(new_cell);
  EXPECT_TRUE(std::isnan(terrain.at(fastdem::layer::elevation, *new_cell)));
}

TEST(StaticTerrainDisplayMapTest, ExpandsOneCellOnNegativeYAndPreservesAllLayers)
{
  auto terrain = makeMap("map");
  ASSERT_EQ(bringup::prepareStaticDisplayMap(terrain), 0U);
  setDistinctLayerValues(terrain);
  const auto before = terrain;
  const auto occupancy = makeOccupancyGrid(2, 3, -0.5, -1.0);
  bool changed = false;
  std::string error;

  ASSERT_TRUE(bringup::extendDisplayMapToOccupancyGrid(terrain, occupancy, &changed, &error))
    << error;

  EXPECT_TRUE(changed);
  EXPECT_EQ(terrain.getSize()(0), 2);
  EXPECT_EQ(terrain.getSize()(1), 3);
  EXPECT_NEAR(terrain.getPosition().x() - terrain.getLength().x() / 2.0, -0.5, 1e-8);
  EXPECT_NEAR(terrain.getPosition().y() - terrain.getLength().y() / 2.0, -1.0, 1e-8);
  expectCellValuesPreserved(before, terrain);
  const auto new_cell = terrain.index(nanogrid::Position(-0.25, -0.75));
  ASSERT_TRUE(new_cell);
  EXPECT_TRUE(std::isnan(terrain.at(fastdem::layer::elevation, *new_cell)));
}

TEST(StaticTerrainDisplayMapTest, RepeatedExpansionInputDoesNotChangeGeometryOrData)
{
  auto terrain = makeMap("map");
  ASSERT_EQ(bringup::prepareStaticDisplayMap(terrain), 0U);
  setDistinctLayerValues(terrain);
  const auto occupancy = makeOccupancyGrid(3, 2, -1.0, -0.5);
  bool changed = false;
  std::string error;
  ASSERT_TRUE(bringup::extendDisplayMapToOccupancyGrid(terrain, occupancy, &changed, &error))
    << error;
  ASSERT_TRUE(changed);
  const auto before_repeat = terrain;

  changed = true;
  ASSERT_TRUE(bringup::extendDisplayMapToOccupancyGrid(terrain, occupancy, &changed, &error))
    << error;

  EXPECT_FALSE(changed);
  EXPECT_EQ(terrain.getSize()(0), before_repeat.getSize()(0));
  EXPECT_EQ(terrain.getSize()(1), before_repeat.getSize()(1));
  EXPECT_DOUBLE_EQ(terrain.getPosition().x(), before_repeat.getPosition().x());
  EXPECT_DOUBLE_EQ(terrain.getPosition().y(), before_repeat.getPosition().y());
  EXPECT_DOUBLE_EQ(terrain.getLength().x(), before_repeat.getLength().x());
  EXPECT_DOUBLE_EQ(terrain.getLength().y(), before_repeat.getLength().y());
  expectCellValuesPreserved(before_repeat, terrain);
}

TEST(StaticTerrainDisplayMapTest, MergesFreshOnlineElevationInDisplayFrame)
{
  auto display = makeMap("map");
  ASSERT_EQ(bringup::prepareStaticDisplayMap(display), 0U);
  nanogrid::GridMap online({
      fastdem::layer::elevation, fastdem::layer::n_points,
      fastdem::layer::observation_age, "slope", "step_residual", "roughness"});
  online.setGeometry(nanogrid::Length(0.5, 0.5), 0.5, nanogrid::Position(0.0, 0.0));
  const auto index = online.index(nanogrid::Position(0.0, 0.0));
  ASSERT_TRUE(index);
  online.at(fastdem::layer::elevation, *index) = 0.35F;
  online.at(fastdem::layer::n_points, *index) = 4.0F;
  online.at(fastdem::layer::observation_age, *index) = 1.0F;
  online.at("slope", *index) = 10.0F;
  online.at("step_residual", *index) = 0.02F;
  online.at("roughness", *index) = 0.01F;
  tf2::Transform identity;
  identity.setIdentity();

  EXPECT_EQ(bringup::mergeOnlineDisplayTerrain(display, online, identity), 1U);
  EXPECT_FLOAT_EQ(display.elevationAt(nanogrid::Position(0.0, 0.0)), 0.35F);
  const auto display_index = display.index(nanogrid::Position(0.0, 0.0));
  ASSERT_TRUE(display_index);
  EXPECT_GT(display.at("display_relief", *display_index), 0.4F);
}

TEST(StaticTerrainDisplayFilterTest, KeepsWallsAndRemovesCeiling)
{
  auto map = makeMap("map");
  EXPECT_TRUE(bringup::keepStaticDisplayPoint(map, 0.0F, 0.0F, 1.5F, 1.8F));
  EXPECT_FALSE(bringup::keepStaticDisplayPoint(map, 0.0F, 0.0F, 2.1F, 1.8F));
  EXPECT_FALSE(bringup::keepStaticDisplayPoint(map, 3.0F, 0.0F, 0.5F, 1.8F));
}

TEST(StaticTerrainDisplayColorTest, UsesRainbowHeightPalette)
{
  EXPECT_EQ(bringup::staticDisplayColor(0.0F), (std::array<std::uint8_t, 3>{255, 0, 0}));
  EXPECT_EQ(bringup::staticDisplayColor(0.55F), (std::array<std::uint8_t, 3>{198, 255, 0}));
  EXPECT_EQ(bringup::staticDisplayColor(1.8F), (std::array<std::uint8_t, 3>{0, 0, 255}));
  EXPECT_EQ(bringup::staticDisplayColor(3.0F), bringup::staticDisplayColor(1.8F));
}

TEST(StaticTerrainDisplayFilterTest, UsesNearbyGroundForWallCells)
{
  auto map = makeMap("map");
  map.setGeometry(1.5F, 1.5F, 0.5F);
  map.get(fastdem::layer::elevation).setConstant(NAN);
  const auto index = map.index(nanogrid::Position(0.0, 0.0));
  ASSERT_TRUE(index.has_value());
  map.at(fastdem::layer::elevation, nanogrid::Index(index->x() + 1, index->y())) = 0.2F;
  EXPECT_TRUE(bringup::keepStaticDisplayPoint(map, 0.0F, 0.0F, 1.0F, 1.8F));
  ASSERT_TRUE(bringup::staticDisplayHeightAboveGround(map, 0.0F, 0.0F, 1.0F));
  EXPECT_FLOAT_EQ(*bringup::staticDisplayHeightAboveGround(map, 0.0F, 0.0F, 1.0F), 0.8F);
  EXPECT_FALSE(bringup::keepStaticDisplayPoint(map, 0.0F, 0.0F, 2.2F, 1.8F));
  map.get(fastdem::layer::elevation).setConstant(NAN);
  EXPECT_FALSE(bringup::keepStaticDisplayPoint(map, 0.0F, 0.0F, 1.0F, 1.8F));
}

}  // namespace
