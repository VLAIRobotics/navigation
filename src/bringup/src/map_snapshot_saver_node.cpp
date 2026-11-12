#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <fastdem/elevation_map.hpp>
#include <fastdem/io/npz.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nanogrid/bridge/ros2.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rmw/qos_profiles.h>
#include <std_srvs/srv/trigger.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "bringup/ground_provenance.hpp"
#include "bringup/map_snapshot_revision.hpp"

namespace
{
using OccupancyGrid = nav_msgs::msg::OccupancyGrid;
using Trigger = std_srvs::srv::Trigger;

void validateGrid(const OccupancyGrid & map)
{
  const auto cells = static_cast<std::uint64_t>(map.info.width) * map.info.height;
  const auto & orientation = map.info.origin.orientation;
  const double quaternion_norm_squared = orientation.x * orientation.x +
    orientation.y * orientation.y + orientation.z * orientation.z + orientation.w * orientation.w;
  if (map.header.frame_id.empty() || map.info.width == 0 || map.info.height == 0 ||
    !std::isfinite(map.info.resolution) || map.info.resolution <= 0.0F ||
    !std::isfinite(map.info.origin.position.x) ||
    !std::isfinite(map.info.origin.position.y) ||
    !std::isfinite(map.info.origin.position.z) ||
    !std::isfinite(map.info.origin.orientation.x) ||
    !std::isfinite(map.info.origin.orientation.y) ||
    !std::isfinite(map.info.origin.orientation.z) ||
    !std::isfinite(map.info.origin.orientation.w) ||
    !std::isfinite(quaternion_norm_squared) ||
    std::abs(quaternion_norm_squared - 1.0) > 1.0e-5 ||
    map.data.size() != cells)
  {
    throw std::runtime_error("fused OccupancyGrid is incomplete or has invalid geometry");
  }
}

bool sameGridGeometry(const OccupancyGrid & lhs, const OccupancyGrid & rhs)
{
  constexpr double kTolerance = 1.0e-5;
  return lhs.header.frame_id == rhs.header.frame_id &&
    lhs.info.width == rhs.info.width && lhs.info.height == rhs.info.height &&
    std::abs(lhs.info.resolution - rhs.info.resolution) <= kTolerance &&
    std::abs(lhs.info.origin.position.x - rhs.info.origin.position.x) <= kTolerance &&
    std::abs(lhs.info.origin.position.y - rhs.info.origin.position.y) <= kTolerance &&
    std::abs(lhs.info.origin.position.z - rhs.info.origin.position.z) <= kTolerance &&
    std::abs(lhs.info.origin.orientation.x - rhs.info.origin.orientation.x) <= kTolerance &&
    std::abs(lhs.info.origin.orientation.y - rhs.info.origin.orientation.y) <= kTolerance &&
    std::abs(lhs.info.origin.orientation.z - rhs.info.origin.orientation.z) <= kTolerance &&
    std::abs(lhs.info.origin.orientation.w - rhs.info.origin.orientation.w) <= kTolerance &&
    std::abs(tf2::getYaw(lhs.info.origin.orientation) -
    tf2::getYaw(rhs.info.origin.orientation)) <= kTolerance;
}

bool sameStamp(const builtin_interfaces::msg::Time & lhs, const builtin_interfaces::msg::Time & rhs)
{
  return lhs.sec == rhs.sec && lhs.nanosec == rhs.nanosec;
}

std::uint8_t occupancyPixel(const std::int8_t value)
{
  if (value < 0) return 205U;
  if (value >= 65) return 0U;
  return 254U;
}

void writeMapPgm(const std::filesystem::path & path, const OccupancyGrid & map)
{
  std::ofstream output(path, std::ios::binary);
  if (!output) throw std::runtime_error("cannot create map.pgm");
  output << "P5\n" << map.info.width << " " << map.info.height << "\n255\n";
  std::vector<std::uint8_t> row(map.info.width);
  for (std::uint32_t image_row = 0; image_row < map.info.height; ++image_row) {
    const auto grid_y = map.info.height - 1U - image_row;
    for (std::uint32_t x = 0; x < map.info.width; ++x) {
      row[x] = occupancyPixel(map.data[static_cast<std::size_t>(grid_y) * map.info.width + x]);
    }
    output.write(reinterpret_cast<const char *>(row.data()),
      static_cast<std::streamsize>(row.size()));
  }
  output.flush();
  if (!output) throw std::runtime_error("failed while writing map.pgm");
  output.close();
  if (!output) throw std::runtime_error("failed to close map.pgm");
}

void writeProvenancePgm(
  const std::filesystem::path & path, const OccupancyGrid & provenance)
{
  std::ofstream output(path, std::ios::binary);
  if (!output) throw std::runtime_error("cannot create ground_source.pgm");
  output << "P5\n" << provenance.info.width << " " << provenance.info.height << "\n255\n";
  std::vector<std::uint8_t> row(provenance.info.width);
  for (std::uint32_t image_row = 0; image_row < provenance.info.height; ++image_row) {
    const auto grid_y = provenance.info.height - 1U - image_row;
    for (std::uint32_t x = 0; x < provenance.info.width; ++x) {
      const auto source = provenance.data[
        static_cast<std::size_t>(grid_y) * provenance.info.width + x];
      if (source < 0 || source > 1) {
        throw std::runtime_error("ground provenance contains an unsupported source value");
      }
      row[x] = source == 1 ? 255U : 0U;
    }
    output.write(reinterpret_cast<const char *>(row.data()),
      static_cast<std::streamsize>(row.size()));
  }
  output.flush();
  if (!output) throw std::runtime_error("failed while writing ground_source.pgm");
  output.close();
  if (!output) throw std::runtime_error("failed to close ground_source.pgm");
}

void writeOccupancySourcePgm(
  const std::filesystem::path & path, const OccupancyGrid & source)
{
  std::ofstream output(path, std::ios::binary);
  if (!output) throw std::runtime_error("cannot create occupancy_source.pgm");
  output << "P5\n" << source.info.width << " " << source.info.height << "\n255\n";
  std::vector<std::uint8_t> row(source.info.width);
  for (std::uint32_t image_row = 0; image_row < source.info.height; ++image_row) {
    const auto grid_y = source.info.height - 1U - image_row;
    for (std::uint32_t x = 0; x < source.info.width; ++x) {
      const auto value = source.data[
        static_cast<std::size_t>(grid_y) * source.info.width + x];
      if (value < 0 || value > 3) {
        throw std::runtime_error("occupancy source contains an unsupported bit mask");
      }
      row[x] = static_cast<std::uint8_t>(value) * 85U;
    }
    output.write(reinterpret_cast<const char *>(row.data()),
      static_cast<std::streamsize>(row.size()));
  }
  output.flush();
  if (!output) throw std::runtime_error("failed while writing occupancy_source.pgm");
  output.close();
  if (!output) throw std::runtime_error("failed to close occupancy_source.pgm");
}

void writeMapYaml(const std::filesystem::path & path, const OccupancyGrid & map)
{
  std::ofstream output(path);
  if (!output) throw std::runtime_error("cannot create map.yaml");
  output << std::setprecision(10)
         << "image: map.pgm\n"
         << "resolution: " << map.info.resolution << "\n"
         << "origin: [" << map.info.origin.position.x << ", "
         << map.info.origin.position.y << ", "
         << tf2::getYaw(map.info.origin.orientation) << "]\n"
         << "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\nmode: trinary\n";
  output.flush();
  if (!output) throw std::runtime_error("failed while writing map.yaml");
  output.close();
  if (!output) throw std::runtime_error("failed to close map.yaml");
}

void writeProvenanceYaml(
  const std::filesystem::path & path, const OccupancyGrid & provenance)
{
  std::ofstream output(path);
  if (!output) throw std::runtime_error("cannot create ground_source.yaml");
  output << std::setprecision(10)
         << "image: ground_source.pgm\n"
         << "frame: " << provenance.header.frame_id << "\n"
         << "resolution: " << provenance.info.resolution << "\n"
         << "origin: [" << provenance.info.origin.position.x << ", "
         << provenance.info.origin.position.y << ", "
         << tf2::getYaw(provenance.info.origin.orientation) << "]\n"
         << "encoding: 0=unknown, 255=measured_ground\n";
  output.flush();
  if (!output) throw std::runtime_error("failed while writing ground_source.yaml");
  output.close();
  if (!output) throw std::runtime_error("failed to close ground_source.yaml");
}

void writeOccupancySourceYaml(
  const std::filesystem::path & path, const OccupancyGrid & source)
{
  std::ofstream output(path);
  if (!output) throw std::runtime_error("cannot create occupancy_source.yaml");
  output << std::setprecision(10)
         << "image: occupancy_source.pgm\n"
         << "frame: " << source.header.frame_id << "\n"
         << "resolution: " << source.info.resolution << "\n"
         << "origin: [" << source.info.origin.position.x << ", "
         << source.info.origin.position.y << ", "
         << tf2::getYaw(source.info.origin.orientation) << "]\n"
         << "encoding: 0=none, 85=prior, 170=online, 255=prior_and_online\n";
  output.flush();
  if (!output) throw std::runtime_error("failed while writing occupancy_source.yaml");
  output.close();
  if (!output) throw std::runtime_error("failed to close occupancy_source.yaml");
}

fastdem::ElevationMap copyTerrainMap(const grid_map_msgs::msg::GridMap & message)
{
  auto source = nanogrid::ros2::fromMsg(message);
  fastdem::ElevationMap result(
    static_cast<float>(source.getLength()(0)), static_cast<float>(source.getLength()(1)),
    static_cast<float>(source.getResolution()), source.getFrameId());
  result.setPosition(source.getPosition());
  result.setStartIndex(source.getStartIndex());
  result.setTimestamp(source.getTimestamp());
  for (const auto & layer : source.getLayers()) {
    if (result.exists(layer)) result.get(layer) = source.get(layer);
    else result.add(layer, source.get(layer));
  }
  if (!result.exists("class")) {
    throw std::runtime_error("global terrain snapshot has no terrain class layer");
  }
  return result;
}

void validateTerrainVersion(
  const fastdem::ElevationMap & terrain, const grid_map_msgs::msg::GridMap & message,
  const OccupancyGrid & map, const OccupancyGrid & provenance)
{
  constexpr double kTolerance = 1.0e-5;
  const bool valid_stamp = message.header.stamp.sec >= 0 &&
    message.header.stamp.nanosec < 1000000000U;
  const auto expected_terrain_stamp = valid_stamp ?
    static_cast<std::uint64_t>(message.header.stamp.sec) * 1000000000ULL +
    message.header.stamp.nanosec : 0U;
  if (message.header.frame_id != map.header.frame_id ||
    !sameStamp(message.header.stamp, map.header.stamp) ||
    !valid_stamp || terrain.getTimestamp() != expected_terrain_stamp ||
    terrain.getFrameId() != map.header.frame_id ||
    std::abs(terrain.getResolution() - map.info.resolution) > kTolerance ||
    terrain.getSize()(0) != static_cast<int>(map.info.width) ||
    terrain.getSize()(1) != static_cast<int>(map.info.height) ||
    std::abs(terrain.getLength()(0) - map.info.width * map.info.resolution) > kTolerance ||
    std::abs(terrain.getLength()(1) - map.info.height * map.info.resolution) > kTolerance ||
    std::abs(map.info.origin.position.z) > kTolerance ||
    std::abs(map.info.origin.orientation.x) > kTolerance ||
    std::abs(map.info.origin.orientation.y) > kTolerance ||
    std::abs(map.info.origin.orientation.z) > kTolerance ||
    std::abs(std::abs(map.info.origin.orientation.w) - 1.0) > kTolerance ||
    std::abs(message.info.pose.position.z) > kTolerance ||
    std::abs(message.info.pose.orientation.x) > kTolerance ||
    std::abs(message.info.pose.orientation.y) > kTolerance ||
    std::abs(message.info.pose.orientation.z) > kTolerance ||
    std::abs(std::abs(message.info.pose.orientation.w) - 1.0) > kTolerance ||
    std::abs(terrain.getPosition().x() -
      (map.info.origin.position.x + map.info.width * map.info.resolution * 0.5)) > kTolerance ||
    std::abs(terrain.getPosition().y() -
      (map.info.origin.position.y + map.info.height * map.info.resolution * 0.5)) > kTolerance)
  {
    throw std::runtime_error("map and global terrain version/geometry do not match");
  }

  tf2::Transform map_from_grid;
  tf2::fromMsg(map.info.origin, map_from_grid);
  const auto grid_from_map = map_from_grid.inverse();
  for (const auto cell : terrain.cells()) {
    const auto position = terrain.position(cell);
    if (!position) continue;
    const auto grid_position = grid_from_map * tf2::Vector3(position->x(), position->y(), 0.0);
    const auto x = static_cast<std::int64_t>(std::floor(grid_position.x() / map.info.resolution));
    const auto y = static_cast<std::int64_t>(std::floor(grid_position.y() / map.info.resolution));
    if (x < 0 || y < 0 || x >= map.info.width || y >= map.info.height) {
      throw std::runtime_error("global terrain cell lies outside the fused map snapshot");
    }
    const auto index = static_cast<std::size_t>(y) * map.info.width +
      static_cast<std::size_t>(x);
    const auto terrain_class = terrain.get("class")(cell.index);
    const auto provenance_value = provenance.data[index];
    if (provenance_value < 0 || provenance_value > 1 || !std::isfinite(terrain_class) ||
      terrain_class < -0.5F || terrain_class > 2.5F)
    {
      throw std::runtime_error("terrain or ground provenance contains an invalid source class");
    }
    const auto class_code = static_cast<std::int8_t>(std::lround(terrain_class));
    if (std::abs(terrain_class - class_code) > 1.0e-4F) {
      throw std::runtime_error("terrain class layer contains a non-integral class value");
    }
    std::int8_t source = class_code == 1 ? 1 : 0;
    if (terrain.exists("ground_source")) {
      const auto raw_source = terrain.get("ground_source")(cell.index);
      if (!std::isfinite(raw_source) || raw_source < -0.5F || raw_source > 1.5F) {
        throw std::runtime_error("terrain ground_source layer contains an invalid source value");
      }
      source = static_cast<std::int8_t>(std::lround(raw_source));
      if (std::abs(raw_source - source) > 1.0e-4F) {
        throw std::runtime_error("terrain ground_source layer contains a non-integral value");
      }
    }
    if (!bringup::terrainGroundSourceMatches(class_code, source, provenance_value)) {
      throw std::runtime_error(
        "terrain class, ground_source and map provenance layers disagree");
    }
  }
}

std::string makeVersionName()
{
  const auto now = std::chrono::system_clock::now();
  const auto epoch_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
    now.time_since_epoch()).count();
  const auto time = std::chrono::system_clock::to_time_t(now);
  std::tm local{};
  if (localtime_r(&time, &local) == nullptr) {
    throw std::runtime_error("cannot format saved map timestamp");
  }
  std::ostringstream output;
  output << std::put_time(&local, "%Y%m%dT%H%M%S") << "_" << (epoch_ms % 1000);
  return output.str();
}

class MapSnapshotSaver : public rclcpp::Node
{
public:
  MapSnapshotSaver()
  : Node("map_snapshot_saver")
  {
    output_root_ = declare_parameter<std::string>("output_root", "");
    source_id_ = declare_parameter<std::string>("source_id", "");
    if (output_root_.empty() || source_id_.empty()) {
      throw std::invalid_argument("output_root and source_id are required");
    }
    if (source_id_.find_first_not_of(
        "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-") !=
      std::string::npos)
    {
      throw std::invalid_argument("source_id may contain only letters, digits, '_' and '-'");
    }
    map_sub_ = create_subscription<OccupancyGrid>(
      "/map", rclcpp::QoS(1).reliable().transient_local(),
      [this](OccupancyGrid::ConstSharedPtr message) {
        std::lock_guard<std::mutex> lock(snapshot_mutex_);
        map_ = *message;
        snapshot_condition_.notify_all();
      });
    provenance_sub_ = create_subscription<OccupancyGrid>(
      "/navigation/ground_provenance", rclcpp::QoS(1).reliable().transient_local(),
      [this](OccupancyGrid::ConstSharedPtr message) {
        std::lock_guard<std::mutex> lock(snapshot_mutex_);
        provenance_ = *message;
        snapshot_condition_.notify_all();
      });
    occupancy_source_sub_ = create_subscription<OccupancyGrid>(
      "/navigation/occupancy_source", rclcpp::QoS(1).reliable().transient_local(),
      [this](OccupancyGrid::ConstSharedPtr message) {
        std::lock_guard<std::mutex> lock(snapshot_mutex_);
        occupancy_source_ = *message;
        snapshot_condition_.notify_all();
      });
    terrain_sub_ = create_subscription<grid_map_msgs::msg::GridMap>(
      "/navigation/global_terrain_map", rclcpp::QoS(1).reliable().transient_local(),
      [this](grid_map_msgs::msg::GridMap::ConstSharedPtr message) {
        std::lock_guard<std::mutex> lock(snapshot_mutex_);
        terrain_ = *message;
        snapshot_condition_.notify_all();
      });
    save_service_callback_group_ = create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
    save_service_ = create_service<Trigger>(
      "/navigation/save_map",
      [this](std::shared_ptr<Trigger::Request>,
        std::shared_ptr<Trigger::Response> response) {saveSnapshot(response);},
      rmw_qos_profile_services_default, save_service_callback_group_);
  }

private:
  void saveSnapshot(const std::shared_ptr<Trigger::Response> & response)
  {
    OccupancyGrid map;
    OccupancyGrid provenance;
    OccupancyGrid occupancy_source;
    grid_map_msgs::msg::GridMap terrain_message;
    {
      std::unique_lock<std::mutex> lock(snapshot_mutex_);
      if (saving_) {
        response->success = false;
        response->message = "a map snapshot is already being written";
        return;
      }
      saving_ = true;
      const auto coherent_snapshot_available = [this]() {
          return map_ && provenance_ && occupancy_source_ && terrain_ &&
                 bringup::sameMapSnapshotRevision(
                   map_->header.stamp, provenance_->header.stamp,
                   occupancy_source_->header.stamp, terrain_->header.stamp);
        };
      if (!snapshot_condition_.wait_for(
          lock, std::chrono::seconds(3), coherent_snapshot_available))
      {
        saving_ = false;
        response->success = false;
        response->message = "timed out waiting for one coherent map snapshot revision";
        return;
      }
      map = *map_;
      provenance = *provenance_;
      occupancy_source = *occupancy_source_;
      terrain_message = *terrain_;
    }
    try {
      validateGrid(map);
      validateGrid(provenance);
      validateGrid(occupancy_source);
      if (!sameGridGeometry(map, provenance)) {
        throw std::runtime_error("map and ground provenance geometry do not match");
      }
      if (!sameGridGeometry(map, occupancy_source)) {
        throw std::runtime_error("map and occupancy source geometry do not match");
      }
      if (!sameStamp(map.header.stamp, provenance.header.stamp)) {
        throw std::runtime_error("map and ground provenance are from different map versions");
      }
      if (!sameStamp(map.header.stamp, occupancy_source.header.stamp)) {
        throw std::runtime_error("map and occupancy source are from different map versions");
      }
      auto terrain = copyTerrainMap(terrain_message);
      validateTerrainVersion(terrain, terrain_message, map, provenance);
      const auto root = std::filesystem::path(output_root_) / source_id_;
      std::filesystem::create_directories(root);
      const auto version = makeVersionName();
      const auto staging = root / (".tmp_" + version);
      const auto final = root / version;
      std::filesystem::create_directory(staging);
      try {
        writeMapPgm(staging / "map.pgm", map);
        writeMapYaml(staging / "map.yaml", map);
        writeProvenancePgm(staging / "ground_source.pgm", provenance);
        writeProvenanceYaml(staging / "ground_source.yaml", provenance);
        writeOccupancySourcePgm(staging / "occupancy_source.pgm", occupancy_source);
        writeOccupancySourceYaml(staging / "occupancy_source.yaml", occupancy_source);
        if (!fastdem::io::saveNpz((staging / "terrain_map.npz").string(), terrain)) {
          throw std::runtime_error("failed to write terrain_map.npz");
        }
        {
          std::ofstream source(staging / ".source_id");
          source << source_id_ << "\n";
          source.flush();
          if (!source) throw std::runtime_error("failed to write source identity");
          source.close();
          if (!source) throw std::runtime_error("failed to close source identity");
        }
        {
          std::ofstream manifest(staging / "snapshot.yaml");
          manifest << "version: " << version << "\n"
                   << "source_id: " << source_id_ << "\n"
                   << "frame: " << map.header.frame_id << "\n"
                   << "map_stamp: [" << map.header.stamp.sec << ", "
                   << map.header.stamp.nanosec << "]\n"
                   << "provenance_stamp: [" << provenance.header.stamp.sec << ", "
                   << provenance.header.stamp.nanosec << "]\n"
                   << "occupancy_source_stamp: [" << occupancy_source.header.stamp.sec << ", "
                   << occupancy_source.header.stamp.nanosec << "]\n"
                   << "terrain_stamp: [" << terrain_message.header.stamp.sec << ", "
                   << terrain_message.header.stamp.nanosec << "]\n"
                   << "width: " << map.info.width << "\nheight: " << map.info.height << "\n"
                   << "resolution: " << map.info.resolution << "\n"
                   << "origin: [" << map.info.origin.position.x << ", "
                   << map.info.origin.position.y << ", "
                   << tf2::getYaw(map.info.origin.orientation) << "]\n";
          manifest.flush();
          if (!manifest) throw std::runtime_error("failed to write snapshot manifest");
          manifest.close();
          if (!manifest) throw std::runtime_error("failed to close snapshot manifest");
        }
        std::filesystem::rename(staging, final);
        const auto temporary_latest = root / (".latest_" + version);
        const auto latest = root / "latest";
        std::filesystem::create_directory_symlink(version, temporary_latest);
        std::filesystem::rename(temporary_latest, latest);
      } catch (...) {
        std::filesystem::remove_all(staging);
        throw;
      }
      response->success = true;
      response->message = final.string();
      RCLCPP_INFO(get_logger(), "Saved fused map snapshot version=%s source_id=%s",
        version.c_str(), source_id_.c_str());
    } catch (const std::exception & error) {
      response->success = false;
      response->message = error.what();
      RCLCPP_ERROR(get_logger(), "Map snapshot save failed: %s", error.what());
    }
    {
      std::lock_guard<std::mutex> lock(snapshot_mutex_);
      saving_ = false;
    }
  }

  std::string output_root_;
  std::string source_id_;
  std::mutex snapshot_mutex_;
  std::condition_variable snapshot_condition_;
  bool saving_{false};
  std::optional<OccupancyGrid> map_;
  std::optional<OccupancyGrid> provenance_;
  std::optional<OccupancyGrid> occupancy_source_;
  std::optional<grid_map_msgs::msg::GridMap> terrain_;
  rclcpp::Subscription<OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<OccupancyGrid>::SharedPtr provenance_sub_;
  rclcpp::Subscription<OccupancyGrid>::SharedPtr occupancy_source_sub_;
  rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr terrain_sub_;
  rclcpp::CallbackGroup::SharedPtr save_service_callback_group_;
  rclcpp::Service<Trigger>::SharedPtr save_service_;
};
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<MapSnapshotSaver>();
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
    executor.add_node(node);
    executor.spin();
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("map_snapshot_saver"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
