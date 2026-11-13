#pragma once

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>
#include <system_error>
#include <unordered_set>
#include <vector>

namespace small_gicp_relocalization {

inline constexpr std::size_t kSnapshotEventCacheCapacity = 256;
inline constexpr std::size_t kSnapshotQueueCapacity = 2;
inline constexpr auto kSnapshotEventMinimumInterval = std::chrono::seconds(5);

inline bool isRateLimitedSnapshotEvent(const std::string & event)
{
  return event == "match_rejected" || event == "tracking_candidate_pending" ||
         event == "registration_failure";
}

inline bool snapshotQueueHasCapacity(std::size_t pending_count)
{
  return pending_count < kSnapshotQueueCapacity;
}

class SnapshotRateLimiter
{
public:
  bool admit(
    std::chrono::steady_clock::time_point now,
    std::chrono::steady_clock::duration minimum_interval)
  {
    if (has_last_admitted_ && now - last_admitted_ < minimum_interval) {
      return false;
    }
    last_admitted_ = now;
    has_last_admitted_ = true;
    return true;
  }

private:
  bool has_last_admitted_{false};
  std::chrono::steady_clock::time_point last_admitted_{};
};

class RecentSnapshotEventCache
{
public:
  explicit RecentSnapshotEventCache(std::size_t capacity)
  : capacity_(capacity)
  {
  }

  bool insert(const std::string & key)
  {
    if (capacity_ == 0) {
      return true;
    }
    const auto existing = keys_.find(key);
    if (existing != keys_.end()) {
      const auto queued = std::find(order_.begin(), order_.end(), key);
      if (queued != order_.end()) {
        order_.erase(queued);
      }
      order_.push_back(key);
      return false;
    }

    keys_.insert(key);
    order_.push_back(key);
    while (order_.size() > capacity_) {
      keys_.erase(order_.front());
      order_.pop_front();
    }
    return true;
  }

  bool erase(const std::string & key)
  {
    if (keys_.erase(key) == 0) {
      return false;
    }
    const auto queued = std::find(order_.begin(), order_.end(), key);
    if (queued != order_.end()) {
      order_.erase(queued);
    }
    return true;
  }

  std::size_t size() const
  {
    return keys_.size();
  }

private:
  const std::size_t capacity_;
  std::deque<std::string> order_;
  std::unordered_set<std::string> keys_;
};

inline std::vector<std::filesystem::path> pruneSnapshotDirectories(
  const std::filesystem::path & root, std::size_t max_count)
{
  struct SnapshotDirectory
  {
    std::filesystem::path path;
    std::filesystem::file_time_type modified;
  };

  std::error_code error;
  if (!std::filesystem::exists(root, error)) {
    if (error) {
      throw std::filesystem::filesystem_error("cannot inspect snapshot root", root, error);
    }
    return {};
  }

  std::vector<SnapshotDirectory> snapshots;
  std::filesystem::recursive_directory_iterator iterator(
    root, std::filesystem::directory_options::skip_permission_denied, error);
  if (error) {
    throw std::filesystem::filesystem_error("cannot scan snapshot root", root, error);
  }
  const std::filesystem::recursive_directory_iterator end;
  while (iterator != end) {
    const auto path = iterator->path();
    const bool is_directory = iterator->is_directory(error);
    if (error) {
      throw std::filesystem::filesystem_error("cannot inspect snapshot entry", path, error);
    }
    if (is_directory && path.filename().string().rfind("sample_", 0) == 0) {
      const auto modified = std::filesystem::last_write_time(path, error);
      if (error) {
        throw std::filesystem::filesystem_error("cannot read snapshot timestamp", path, error);
      }
      snapshots.push_back({path, modified});
      iterator.disable_recursion_pending();
    }
    iterator.increment(error);
    if (error) {
      throw std::filesystem::filesystem_error("cannot continue snapshot scan", root, error);
    }
  }

  std::sort(snapshots.begin(), snapshots.end(), [](const auto & lhs, const auto & rhs) {
      if (lhs.modified != rhs.modified) {
        return lhs.modified < rhs.modified;
      }
      return lhs.path < rhs.path;
    });

  std::vector<std::filesystem::path> removed;
  const std::size_t remove_count = snapshots.size() > max_count ? snapshots.size() - max_count : 0;
  for (std::size_t index = 0; index < remove_count; ++index) {
    std::filesystem::remove_all(snapshots[index].path, error);
    if (error) {
      throw std::filesystem::filesystem_error(
              "cannot remove old snapshot", snapshots[index].path, error);
    }
    removed.push_back(snapshots[index].path);

    auto parent_relative = snapshots[index].path.lexically_relative(root).parent_path();
    while (!parent_relative.empty() && parent_relative != ".") {
      const auto parent = root / parent_relative;
      if (!std::filesystem::is_empty(parent, error)) {
        if (error) {
          throw std::filesystem::filesystem_error("cannot inspect snapshot parent", parent, error);
        }
        break;
      }
      std::filesystem::remove(parent, error);
      if (error) {
        throw std::filesystem::filesystem_error(
                "cannot remove empty snapshot parent", parent, error);
      }
      parent_relative = parent_relative.parent_path();
    }
  }
  return removed;
}

inline std::string snapshotMapVersionKey(
  const std::filesystem::path & localization_map,
  const std::filesystem::path & navigation_map)
{
  std::uint64_t hash = 14695981039346656037ULL;
  const auto mix = [&hash](const std::string & value) {
      for (const unsigned char byte : value) {
        hash ^= byte;
        hash *= 1099511628211ULL;
      }
      hash ^= 0xffU;
      hash *= 1099511628211ULL;
    };
  const auto append_file_identity = [&mix](const std::filesystem::path & path) {
      if (path.empty()) {
        mix("<empty>");
        return;
      }
      const auto absolute_path = std::filesystem::absolute(path).lexically_normal();
      const auto size = std::filesystem::file_size(absolute_path);
      const auto modified = std::filesystem::last_write_time(absolute_path).time_since_epoch().count();
      mix(absolute_path.string() + ":" + std::to_string(size) + ":" +
        std::to_string(modified));
    };
  append_file_identity(localization_map);
  append_file_identity(navigation_map);

  std::ostringstream key;
  key << std::hex << std::setw(16) << std::setfill('0') << hash;
  return key.str();
}

inline void cacheSnapshotAsset(
  const std::filesystem::path & source,
  const std::filesystem::path & cached,
  const std::string & temporary_suffix)
{
  std::error_code error;
  const auto cached_status = std::filesystem::status(cached, error);
  if (error == std::errc::no_such_file_or_directory) {
    error.clear();
  } else if (error) {
    throw std::filesystem::filesystem_error("cannot inspect cached map asset", cached, error);
  } else if (std::filesystem::is_regular_file(cached_status)) {
    return;
  } else if (cached_status.type() != std::filesystem::file_type::not_found) {
    throw std::filesystem::filesystem_error(
            "cached map asset is not a regular file", cached,
            std::make_error_code(std::errc::invalid_argument));
  }

  std::filesystem::create_directories(cached.parent_path());
  auto temporary = cached;
  temporary += ".tmp_" + temporary_suffix;
  std::filesystem::remove(temporary, error);
  if (error) {
    throw std::filesystem::filesystem_error("cannot remove stale map asset temporary", temporary, error);
  }
  std::filesystem::copy_file(source, temporary, std::filesystem::copy_options::none, error);
  if (error) {
    std::filesystem::remove(temporary);
    throw std::filesystem::filesystem_error("cannot cache map asset", source, temporary, error);
  }
  std::filesystem::rename(temporary, cached, error);
  if (error) {
    std::error_code inspect_error;
    const bool another_writer_finished = std::filesystem::is_regular_file(cached, inspect_error);
    std::error_code cleanup_error;
    std::filesystem::remove(temporary, cleanup_error);
    if (another_writer_finished && !inspect_error) {
      return;
    }
    throw std::filesystem::filesystem_error("cannot publish cached map asset", temporary, cached, error);
  }
}

inline void linkOrCopySnapshotAsset(
  const std::filesystem::path & cached,
  const std::filesystem::path & snapshot_asset)
{
  std::error_code error;
  std::filesystem::create_hard_link(cached, snapshot_asset, error);
  if (!error) {
    return;
  }
  error.clear();
  std::filesystem::copy_file(cached, snapshot_asset, std::filesystem::copy_options::none, error);
  if (error) {
    throw std::filesystem::filesystem_error(
            "cannot link or copy cached map asset", cached, snapshot_asset, error);
  }
}

inline void pruneSnapshotMapVersions(
  const std::filesystem::path & root, const std::string & keep_version)
{
  const auto map_root = root / "maps";
  std::error_code error;
  if (!std::filesystem::exists(map_root, error)) {
    if (error) {
      throw std::filesystem::filesystem_error("cannot inspect snapshot map cache", map_root, error);
    }
    return;
  }

  const std::string keep_name = "version_" + keep_version;
  for (const auto & entry : std::filesystem::directory_iterator(map_root)) {
    if (!entry.is_directory(error)) {
      if (error) {
        throw std::filesystem::filesystem_error("cannot inspect snapshot map cache entry", entry.path(), error);
      }
      continue;
    }
    if (entry.path().filename().string().rfind("version_", 0) == 0 &&
      entry.path().filename().string() != keep_name)
    {
      std::filesystem::remove_all(entry.path(), error);
      if (error) {
        throw std::filesystem::filesystem_error(
                "cannot remove stale snapshot map version", entry.path(), error);
      }
    }
  }
}

}  // namespace small_gicp_relocalization
