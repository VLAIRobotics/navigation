#include <gtest/gtest.h>

#include <stdexcept>

#include "bringup/ground_provenance.hpp"
#include "bringup/map_snapshot_revision.hpp"

TEST(GroundProvenance, UsesOccupancyGridGeometryAndPreservesUnknownSource)
{
  nav_msgs::msg::OccupancyGrid map;
  map.header.frame_id = "map";
  map.info.width = 3;
  map.info.height = 2;
  map.info.resolution = 0.5F;
  map.info.origin.position.x = -1.0;
  map.info.origin.position.y = 2.0;
  map.info.origin.orientation.w = 1.0;
  map.data = {0, 1, 2, 0, 2, 1};

  EXPECT_EQ(
    bringup::groundSourceAtWorldPoint(map, -0.75, 2.25, "map"),
    bringup::GroundSource::UNKNOWN);
  EXPECT_EQ(
    bringup::groundSourceAtWorldPoint(map, -0.25, 2.25, "map"),
    bringup::GroundSource::MEASURED);
  EXPECT_EQ(
    bringup::groundSourceAtWorldPoint(map, 0.25, 2.25, "map"),
    bringup::GroundSource::UNKNOWN);
  EXPECT_EQ(
    bringup::groundSourceAtWorldPoint(map, -0.75, 2.75, "map"),
    bringup::GroundSource::UNKNOWN);
  EXPECT_FALSE(bringup::groundSourceAtWorldPoint(map, -0.75, 2.25, "odom"));
  EXPECT_FALSE(bringup::groundSourceAtWorldPoint(map, 4.0, 4.0, "map"));
}

TEST(GroundProvenance, RejectsInconsistentTerrainAndMapSources)
{
  EXPECT_TRUE(bringup::terrainGroundSourceMatches(1, 1, 1));
  EXPECT_TRUE(bringup::terrainGroundSourceMatches(2, 0, 0));
  EXPECT_TRUE(bringup::terrainGroundSourceMatches(0, 0, 0));
  EXPECT_FALSE(bringup::terrainGroundSourceMatches(0, 0, 1));
  EXPECT_FALSE(bringup::terrainGroundSourceMatches(1, 0, 1));
  EXPECT_FALSE(bringup::terrainGroundSourceMatches(3, 2, 2));
  EXPECT_FALSE(bringup::terrainGroundSourceMatches(3, 0, 0));
  EXPECT_FALSE(bringup::terrainGroundSourceMatches(1, 1, 0));
  EXPECT_FALSE(bringup::terrainGroundSourceMatches(4, 0, 0));
  EXPECT_FALSE(bringup::terrainGroundSourceMatches(2, 3, 0));
}

TEST(MapSnapshotRevision, WaitsForDelayedTerrainRevisionBeforeAcceptingSnapshot)
{
  builtin_interfaces::msg::Time map_stamp;
  builtin_interfaces::msg::Time provenance_stamp;
  builtin_interfaces::msg::Time occupancy_source_stamp;
  builtin_interfaces::msg::Time terrain_stamp;
  map_stamp.sec = 10;
  provenance_stamp.sec = 10;
  occupancy_source_stamp.sec = 10;
  terrain_stamp.sec = 9;
  EXPECT_FALSE(bringup::sameMapSnapshotRevision(
      map_stamp, provenance_stamp, occupancy_source_stamp, terrain_stamp));

  terrain_stamp = map_stamp;
  EXPECT_TRUE(bringup::sameMapSnapshotRevision(
      map_stamp, provenance_stamp, occupancy_source_stamp, terrain_stamp));

  map_stamp.sec = 11;
  provenance_stamp.sec = 11;
  occupancy_source_stamp.sec = 11;
  EXPECT_FALSE(bringup::sameMapSnapshotRevision(
      map_stamp, provenance_stamp, occupancy_source_stamp, terrain_stamp));
  terrain_stamp = map_stamp;
  EXPECT_TRUE(bringup::sameMapSnapshotRevision(
      map_stamp, provenance_stamp, occupancy_source_stamp, terrain_stamp));
}

TEST(MapSnapshotRevision, AdvancesWhenClockDoesNotAdvance)
{
  builtin_interfaces::msg::Time previous;
  previous.sec = 12;
  previous.nanosec = 999999999U;
  const auto revision = bringup::nextMapSnapshotRevision(previous, previous);
  EXPECT_EQ(revision.sec, 13);
  EXPECT_EQ(revision.nanosec, 0U);

  builtin_interfaces::msg::Time later;
  later.sec = 14;
  later.nanosec = 1U;
  const auto later_revision = bringup::nextMapSnapshotRevision(previous, later);
  EXPECT_EQ(later_revision.sec, later.sec);
  EXPECT_EQ(later_revision.nanosec, later.nanosec);
}

TEST(MapSnapshotRevision, ConvertsNanosecondsToHumbleBuiltinTime)
{
  const auto stamp = bringup::mapSnapshotTimeFromNanoseconds(12000000034LL);
  EXPECT_EQ(stamp.sec, 12);
  EXPECT_EQ(stamp.nanosec, 34U);
  EXPECT_THROW(bringup::mapSnapshotTimeFromNanoseconds(-1), std::invalid_argument);
}
