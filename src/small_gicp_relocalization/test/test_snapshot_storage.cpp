#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>
#include <system_error>

#include <small_gicp_relocalization/snapshot_storage.hpp>

namespace {

bool writeTextFile(const std::filesystem::path & path, const std::string & contents)
{
  std::ofstream output(path, std::ios::binary | std::ios::trunc);
  output << contents;
  return output.good();
}

std::string readTextFile(const std::filesystem::path & path)
{
  std::ifstream input(path, std::ios::binary);
  return std::string(std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>());
}

class SnapshotStorageTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    const auto unique = std::chrono::steady_clock::now().time_since_epoch().count();
    root_ = std::filesystem::temp_directory_path() /
      ("small_gicp_snapshot_storage_" + std::to_string(unique));
    std::filesystem::create_directories(root_);
  }

  void TearDown() override
  {
    std::error_code error;
    std::filesystem::remove_all(root_, error);
  }

  std::filesystem::path root_;
};

TEST_F(SnapshotStorageTest, EventCacheSuppressesRecentKeysAndStaysBounded)
{
  small_gicp_relocalization::RecentSnapshotEventCache cache(2);

  EXPECT_TRUE(cache.insert("first"));
  EXPECT_TRUE(cache.insert("second"));
  EXPECT_FALSE(cache.insert("first"));
  EXPECT_TRUE(cache.insert("third"));
  EXPECT_FALSE(cache.insert("first"));
  EXPECT_TRUE(cache.insert("second"));
  EXPECT_EQ(cache.size(), 2U);
  EXPECT_TRUE(cache.erase("first"));
  EXPECT_TRUE(cache.insert("first"));
  EXPECT_EQ(cache.size(), 2U);

  for (int index = 0; index < 1000; ++index) {
    EXPECT_TRUE(cache.insert("event_" + std::to_string(index)));
    EXPECT_LE(cache.size(), 2U);
  }
}

TEST_F(SnapshotStorageTest, QueueCapacityAndRateLimitAreBounded)
{
  EXPECT_TRUE(small_gicp_relocalization::snapshotQueueHasCapacity(0));
  EXPECT_TRUE(small_gicp_relocalization::snapshotQueueHasCapacity(1));
  EXPECT_FALSE(small_gicp_relocalization::snapshotQueueHasCapacity(2));
  EXPECT_FALSE(small_gicp_relocalization::isRateLimitedSnapshotEvent("lost"));
  EXPECT_FALSE(small_gicp_relocalization::isRateLimitedSnapshotEvent("normal_localized"));
  EXPECT_TRUE(small_gicp_relocalization::isRateLimitedSnapshotEvent("match_rejected"));

  small_gicp_relocalization::SnapshotRateLimiter limiter;
  const auto start = std::chrono::steady_clock::time_point{};
  EXPECT_TRUE(limiter.admit(start, std::chrono::seconds(5)));
  EXPECT_FALSE(limiter.admit(start + std::chrono::seconds(4), std::chrono::seconds(5)));
  EXPECT_TRUE(limiter.admit(start + std::chrono::seconds(5), std::chrono::seconds(5)));
}

TEST_F(SnapshotStorageTest, PrunesOldestSamplesAcrossSessionDirectories)
{
  const auto old_sample = root_ / "session_old" / "sample_000001";
  const auto middle_sample = root_ / "session_middle" / "sample_000002";
  const auto newest_sample = root_ / "sample_000003";
  std::filesystem::create_directories(old_sample);
  std::filesystem::create_directories(middle_sample);
  std::filesystem::create_directories(newest_sample);

  const auto base_time = std::filesystem::file_time_type::clock::now();
  std::filesystem::last_write_time(old_sample, base_time - std::chrono::seconds(30));
  std::filesystem::last_write_time(middle_sample, base_time - std::chrono::seconds(20));
  std::filesystem::last_write_time(newest_sample, base_time - std::chrono::seconds(10));

  const auto removed = small_gicp_relocalization::pruneSnapshotDirectories(root_, 2);

  ASSERT_EQ(removed.size(), 1U);
  EXPECT_EQ(removed.front(), old_sample);
  EXPECT_FALSE(std::filesystem::exists(old_sample));
  EXPECT_FALSE(std::filesystem::exists(root_ / "session_old"));
  EXPECT_TRUE(std::filesystem::exists(middle_sample));
  EXPECT_TRUE(std::filesystem::exists(newest_sample));
}

TEST_F(SnapshotStorageTest, CreatesMissingCacheAssetAndReusesExistingFile)
{
  const auto source = root_ / "source_map.pcd";
  const auto cached = root_ / "maps" / "version_one" / "map.pcd";
  ASSERT_TRUE(writeTextFile(source, "first-map-content"));
  ASSERT_FALSE(std::filesystem::exists(cached));

  ASSERT_NO_THROW(small_gicp_relocalization::cacheSnapshotAsset(source, cached, "first"));
  ASSERT_TRUE(std::filesystem::is_regular_file(cached));
  EXPECT_EQ(readTextFile(cached), "first-map-content");

  ASSERT_TRUE(writeTextFile(source, "replacement-source-content"));
  ASSERT_NO_THROW(small_gicp_relocalization::cacheSnapshotAsset(source, cached, "second"));
  EXPECT_EQ(readTextFile(cached), "first-map-content");
}

TEST_F(SnapshotStorageTest, ReusesMapAssetsForOneVersionAndPrunesOldVersions)
{
  const auto source = root_ / "source_map.pcd";
  ASSERT_TRUE(writeTextFile(source, "map-version-one"));
  const auto first_version = small_gicp_relocalization::snapshotMapVersionKey(source, {});
  const auto cache_root = root_ / "snapshots" / "maps";
  const auto first_cache_dir = cache_root / ("version_" + first_version);
  const auto first_cached_map = first_cache_dir / "map.pcd";
  const auto first_sample = root_ / "snapshots" / "sample_one";
  const auto second_sample = root_ / "snapshots" / "sample_two";
  std::filesystem::create_directories(first_sample);
  std::filesystem::create_directories(second_sample);

  small_gicp_relocalization::cacheSnapshotAsset(source, first_cached_map, "test");
  small_gicp_relocalization::cacheSnapshotAsset(source, first_cached_map, "test");
  small_gicp_relocalization::linkOrCopySnapshotAsset(
    first_cached_map, first_sample / "map.pcd");
  small_gicp_relocalization::linkOrCopySnapshotAsset(
    first_cached_map, second_sample / "map.pcd");
  EXPECT_EQ(readTextFile(first_sample / "map.pcd"), "map-version-one");
  EXPECT_EQ(readTextFile(second_sample / "map.pcd"), "map-version-one");

  ASSERT_TRUE(writeTextFile(source, "map-version-two-with-new-size"));
  const auto second_version = small_gicp_relocalization::snapshotMapVersionKey(source, {});
  ASSERT_NE(first_version, second_version);
  const auto second_cache_dir = cache_root / ("version_" + second_version);
  std::filesystem::create_directories(second_cache_dir);
  small_gicp_relocalization::cacheSnapshotAsset(
    source, second_cache_dir / "map.pcd", "test");

  small_gicp_relocalization::pruneSnapshotMapVersions(root_ / "snapshots", second_version);
  EXPECT_FALSE(std::filesystem::exists(first_cache_dir));
  EXPECT_TRUE(std::filesystem::exists(second_cache_dir / "map.pcd"));
  EXPECT_EQ(readTextFile(first_sample / "map.pcd"), "map-version-one");
}

}  // namespace
