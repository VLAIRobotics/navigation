#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <unordered_map>
#include <utility>
#include <vector>

#include <grid_map_core/GridMap.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "bringup/display_cloud_color.hpp"

namespace
{

struct DisplayPoint
{
  float x;
  float y;
  float z;
  float height_above_ground;
};

std::uint64_t groundVoxelKey(float x, float y, double voxel_size)
{
  const auto ix = static_cast<std::uint32_t>(
    static_cast<std::int32_t>(std::floor(x / voxel_size)));
  const auto iy = static_cast<std::uint32_t>(
    static_cast<std::int32_t>(std::floor(y / voxel_size)));
  return (static_cast<std::uint64_t>(ix) << 32U) | iy;
}

struct StructureVoxelKey
{
  std::int64_t x{0};
  std::int64_t y{0};
  std::int64_t z{0};

  bool operator==(const StructureVoxelKey & other) const
  {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct StructureVoxelKeyHash
{
  std::size_t operator()(const StructureVoxelKey & key) const
  {
    auto seed = std::hash<std::int64_t>{}(key.x);
    seed ^= std::hash<std::int64_t>{}(key.y) + 0x9e3779b9U + (seed << 6U) + (seed >> 2U);
    seed ^= std::hash<std::int64_t>{}(key.z) + 0x9e3779b9U + (seed << 6U) + (seed >> 2U);
    return seed;
  }
};

struct HistoryCandidate
{
  DisplayPoint point{};
  std::uint16_t observations{0};
  std::uint16_t required_observations{0};
  std::int64_t last_stamp_ns{0};
  std::chrono::steady_clock::time_point last_received;
};

StructureVoxelKey structureVoxelKey(const DisplayPoint & point, double voxel_size)
{
  return {
    static_cast<std::int64_t>(std::floor(point.x / voxel_size)),
    static_cast<std::int64_t>(std::floor(point.y / voxel_size)),
    static_cast<std::int64_t>(std::floor(point.z / voxel_size))};
}

sensor_msgs::msg::PointCloud2 makeColoredCloud(
  const std::vector<DisplayPoint> & points,
  const std::string & frame_id,
  const rclcpp::Time & stamp)
{
  sensor_msgs::msg::PointCloud2 message;
  message.header.frame_id = frame_id;
  message.header.stamp = stamp;
  sensor_msgs::PointCloud2Modifier modifier(message);
  modifier.setPointCloud2Fields(
    4,
    "x", 1, sensor_msgs::msg::PointField::FLOAT32,
    "y", 1, sensor_msgs::msg::PointField::FLOAT32,
    "z", 1, sensor_msgs::msg::PointField::FLOAT32,
    "rgb", 1, sensor_msgs::msg::PointField::FLOAT32);
  modifier.resize(points.size());
  sensor_msgs::PointCloud2Iterator<float> x(message, "x");
  sensor_msgs::PointCloud2Iterator<float> y(message, "y");
  sensor_msgs::PointCloud2Iterator<float> z(message, "z");
  sensor_msgs::PointCloud2Iterator<std::uint8_t> r(message, "r");
  sensor_msgs::PointCloud2Iterator<std::uint8_t> g(message, "g");
  sensor_msgs::PointCloud2Iterator<std::uint8_t> b(message, "b");
  for (const auto & point : points) {
    *x = point.x;
    *y = point.y;
    *z = point.z;
    const auto color = bringup::heightDisplayColor(point.height_above_ground);
    *r = color[0];
    *g = color[1];
    *b = color[2];
    ++x;
    ++y;
    ++z;
    ++r;
    ++g;
    ++b;
  }
  return message;
}

class NavigationCloudDisplayNode : public rclcpp::Node
{
  using PointCloud = sensor_msgs::msg::PointCloud2;

  struct GroundSnapshot
  {
    grid_map::GridMap map;
    std::int64_t stamp_ns{0};
    std::chrono::steady_clock::time_point received_at;
    bool has_required_layers{false};
  };

public:
  NavigationCloudDisplayNode()
  : Node("navigation_cloud_display"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
  {
    input_topic_ = declare_parameter("input_topic", std::string("/cloud_registered_body"));
    terrain_topic_ = declare_parameter(
      "terrain_topic", std::string("/fastdem/postprocess/gridmap"));
    ground_topic_ = declare_parameter(
      "ground_display_topic", std::string("/navigation_display/local_ground_cloud"));
    structure_topic_ = declare_parameter(
      "structure_display_topic", std::string("/navigation_display/local_structure_cloud"));
    history_ground_topic_ = declare_parameter(
      "history_ground_topic", std::string("/navigation_display/history_ground_cloud"));
    history_structure_topic_ = declare_parameter(
      "history_structure_topic", std::string("/navigation_display/history_structure_cloud"));
    trajectory_topic_ = declare_parameter(
      "trajectory_topic", std::string("/navigation_display/traveled_path"));
    odom_frame_ = declare_parameter("odom_frame", std::string("odom"));
    sensor_height_ = declare_parameter("sensor_height", 0.25);
    max_height_above_ground_ = declare_parameter("max_height_above_ground", 1.2);
    display_radius_ = declare_parameter("display_radius", 5.0);
    ground_voxel_size_ = declare_parameter("ground_voxel_size", 0.12);
    structure_voxel_size_ = declare_parameter("structure_voxel_size", 0.12);
    history_confirmation_frames_ = declare_parameter("history_confirmation_frames", 2);
    fallback_history_confirmation_frames_ = declare_parameter(
      "fallback_history_confirmation_frames", 5);
    history_confirmation_window_ = declare_parameter("history_confirmation_window", 0.5);
    history_max_ground_voxels_ = declare_parameter("history_max_ground_voxels", 300000);
    history_max_structure_voxels_ = declare_parameter("history_max_structure_voxels", 300000);
    history_candidate_limit_ = declare_parameter("history_candidate_limit", 30000);
    history_publish_period_ = declare_parameter("history_publish_period", 2.0);
    trajectory_max_poses_ = declare_parameter("trajectory_max_poses", 20000);
    trajectory_min_distance_ = declare_parameter("trajectory_min_distance", 0.05);
    history_health_topic_ = declare_parameter("history_health_topic", "/fastlio/health");
    history_health_timeout_ = declare_parameter("history_health_timeout", 0.5);
    ground_band_ = declare_parameter("ground_band", 0.10);
    terrain_max_age_ = declare_parameter("terrain_max_age", 1.0);
    future_stamp_tolerance_ = declare_parameter("future_stamp_tolerance", 0.1);
    cloud_max_age_ = declare_parameter("cloud_max_age", 1.0);
    point_retention_timeout_ = declare_parameter("point_retention_timeout", 1.0);
    min_terrain_observations_ = declare_parameter("min_terrain_observations", 3);
    max_terrain_observation_age_ = declare_parameter("max_terrain_observation_age", 3);
    if (input_topic_.empty() || terrain_topic_.empty() || ground_topic_.empty() ||
      structure_topic_.empty() || history_ground_topic_.empty() ||
      history_structure_topic_.empty() || trajectory_topic_.empty() || odom_frame_.empty() ||
      !std::isfinite(sensor_height_) || sensor_height_ <= 0.0 ||
      !std::isfinite(max_height_above_ground_) || max_height_above_ground_ <= 0.0 ||
      !std::isfinite(display_radius_) || display_radius_ <= 0.0 ||
      !std::isfinite(ground_voxel_size_) || ground_voxel_size_ <= 0.0 ||
      !std::isfinite(structure_voxel_size_) || structure_voxel_size_ <= 0.0 ||
      history_confirmation_frames_ < 1 || history_confirmation_frames_ > 255 ||
      fallback_history_confirmation_frames_ < history_confirmation_frames_ ||
      fallback_history_confirmation_frames_ > 255 ||
      !std::isfinite(history_confirmation_window_) || history_confirmation_window_ <= 0.0 ||
      history_confirmation_window_ > 10.0 ||
      history_max_ground_voxels_ < 1 || history_max_structure_voxels_ < 1 ||
      history_candidate_limit_ < 1 || !std::isfinite(history_publish_period_) ||
      history_publish_period_ < 0.5 || trajectory_max_poses_ < 2 ||
      !std::isfinite(trajectory_min_distance_) || trajectory_min_distance_ <= 0.0 ||
      history_health_topic_.empty() || !std::isfinite(history_health_timeout_) ||
      history_health_timeout_ <= 0.0 ||
      !std::isfinite(ground_band_) || ground_band_ < 0.0 ||
      !std::isfinite(terrain_max_age_) || terrain_max_age_ <= 0.0 ||
      !std::isfinite(future_stamp_tolerance_) || future_stamp_tolerance_ < 0.0 ||
      !std::isfinite(cloud_max_age_) || cloud_max_age_ <= 0.0 ||
      !std::isfinite(point_retention_timeout_) || point_retention_timeout_ <= 0.0 ||
      min_terrain_observations_ < 1 || max_terrain_observation_age_ < 0)
    {
      throw std::invalid_argument("invalid navigation cloud display parameters");
    }
    history_confirmation_window_ns_ = static_cast<std::int64_t>(
      history_confirmation_window_ * 1.0e9);

    const auto output_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
    ground_publisher_ = create_publisher<PointCloud>(ground_topic_, output_qos);
    structure_publisher_ = create_publisher<PointCloud>(structure_topic_, output_qos);
    const auto history_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    history_ground_publisher_ = create_publisher<PointCloud>(history_ground_topic_, history_qos);
    history_structure_publisher_ = create_publisher<PointCloud>(
      history_structure_topic_, history_qos);
    trajectory_publisher_ = create_publisher<nav_msgs::msg::Path>(trajectory_topic_, history_qos);
    cloud_subscription_ = create_subscription<PointCloud>(
      input_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
      std::bind(&NavigationCloudDisplayNode::cloudCallback, this, std::placeholders::_1));
    terrain_subscription_ = create_subscription<grid_map_msgs::msg::GridMap>(
      terrain_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
      std::bind(&NavigationCloudDisplayNode::terrainCallback, this, std::placeholders::_1));
    odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      "/Odometry", rclcpp::QoS(10),
      std::bind(&NavigationCloudDisplayNode::odometryCallback, this, std::placeholders::_1));
    history_health_subscription_ = create_subscription<std_msgs::msg::Bool>(
      history_health_topic_, rclcpp::QoS(1),
      [this](std_msgs::msg::Bool::ConstSharedPtr message) {
        history_health_received_at_ = std::chrono::steady_clock::now();
        history_health_known_ = true;
        history_health_ok_ = message->data;
      });
    clear_timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&NavigationCloudDisplayNode::clearExpiredDisplay, this));
    history_publish_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(history_publish_period_)),
      std::bind(&NavigationCloudDisplayNode::publishHistory, this));
  }

private:
  void terrainCallback(const grid_map_msgs::msg::GridMap::ConstSharedPtr message)
  {
    if (!message) return;
    grid_map::GridMap map;
    if (!grid_map::GridMapRosConverter::fromMessage(*message, map)) {
      std::lock_guard<std::mutex> lock(terrain_mutex_);
      terrain_snapshot_.reset();
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Cannot decode FastDEM display gridmap");
      return;
    }

    auto snapshot = std::make_shared<GroundSnapshot>();
    snapshot->stamp_ns = rclcpp::Time(
      message->header.stamp, get_clock()->get_clock_type()).nanoseconds();
    snapshot->received_at = std::chrono::steady_clock::now();
    snapshot->has_required_layers = map.exists("elevation") && map.exists("n_points") &&
      map.exists("observation_age");
    snapshot->map = std::move(map);
    std::lock_guard<std::mutex> lock(terrain_mutex_);
    terrain_snapshot_ = std::move(snapshot);
  }

  std::shared_ptr<const GroundSnapshot> groundSnapshot() const
  {
    std::lock_guard<std::mutex> lock(terrain_mutex_);
    return terrain_snapshot_;
  }

  bool terrainSnapshotFresh(
    const std::shared_ptr<const GroundSnapshot> & snapshot,
    const rclcpp::Time & now) const
  {
    if (!snapshot || !snapshot->has_required_layers || snapshot->stamp_ns <= 0) return false;
    const auto stamp = rclcpp::Time(snapshot->stamp_ns, get_clock()->get_clock_type());
    const double source_age = (now - stamp).seconds();
    const double receive_age = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - snapshot->received_at).count();
    return std::isfinite(source_age) && source_age <= terrain_max_age_ &&
           source_age >= -future_stamp_tolerance_ && std::isfinite(receive_age) &&
           receive_age <= terrain_max_age_;
  }

  bool groundElevationAt(
    const GroundSnapshot & snapshot,
    double x,
    double y,
    float & elevation) const
  {
    grid_map::Index index;
    if (!snapshot.map.getIndex(grid_map::Position(x, y), index)) return false;
    const float value = snapshot.map.at("elevation", index);
    const float observation_count = snapshot.map.at("n_points", index);
    const float observation_age = snapshot.map.at("observation_age", index);
    if (!std::isfinite(value) || !std::isfinite(observation_count) ||
      observation_count < static_cast<float>(min_terrain_observations_) ||
      !std::isfinite(observation_age) ||
      observation_age > static_cast<float>(max_terrain_observation_age_))
    {
      return false;
    }
    elevation = value;
    return true;
  }

  bool historyWritesAllowed() const
  {
    if (!history_health_known_ || !history_health_ok_) return false;
    const double age = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - history_health_received_at_).count();
    return std::isfinite(age) && age >= 0.0 && age <= history_health_timeout_;
  }

  template<typename Key, typename Hash>
  void observeHistoryVoxel(
    std::unordered_map<Key, DisplayPoint, Hash> & history,
    std::unordered_map<Key, HistoryCandidate, Hash> & candidates,
    const Key & key, const DisplayPoint & point, const std::int64_t stamp_ns,
    const std::uint16_t required_observations, const std::size_t history_limit,
    const bool trusted_ground_reference)
  {
    std::lock_guard<std::mutex> lock(history_mutex_);
    const auto confirmed = history.find(key);
    if (confirmed != history.end()) {
      if (trusted_ground_reference) {
        auto & previous = confirmed->second;
        previous.x = previous.x * 0.8F + point.x * 0.2F;
        previous.y = previous.y * 0.8F + point.y * 0.2F;
        previous.z = previous.z * 0.8F + point.z * 0.2F;
        previous.height_above_ground = previous.height_above_ground * 0.8F +
          point.height_above_ground * 0.2F;
      }
      return;
    }

    auto candidate = candidates.find(key);
    if (candidate == candidates.end()) {
      if (candidates.size() >= history_candidate_limit_) return;
      HistoryCandidate initial;
      initial.point = point;
      initial.observations = 1;
      initial.required_observations = required_observations;
      initial.last_stamp_ns = stamp_ns;
      initial.last_received = std::chrono::steady_clock::now();
      candidate = candidates.emplace(key, initial).first;
    } else {
      auto & evidence = candidate->second;
      if (stamp_ns <= evidence.last_stamp_ns) return;
      const auto gap_ns = stamp_ns - evidence.last_stamp_ns;
      if (gap_ns > history_confirmation_window_ns_) {
        evidence.point = point;
        evidence.observations = 1;
        evidence.required_observations = required_observations;
      } else {
        if (evidence.observations < std::numeric_limits<std::uint16_t>::max()) {
          ++evidence.observations;
        }
        evidence.required_observations = std::max(
          evidence.required_observations, required_observations);
        evidence.point.x = evidence.point.x * 0.8F + point.x * 0.2F;
        evidence.point.y = evidence.point.y * 0.8F + point.y * 0.2F;
        evidence.point.z = evidence.point.z * 0.8F + point.z * 0.2F;
        evidence.point.height_above_ground = evidence.point.height_above_ground * 0.8F +
          point.height_above_ground * 0.2F;
      }
      evidence.last_stamp_ns = stamp_ns;
      evidence.last_received = std::chrono::steady_clock::now();
    }

    const auto evidence = candidate->second;
    if (evidence.observations < evidence.required_observations) return;
    if (history.size() < history_limit) {
      history.emplace(key, evidence.point);
    }
    candidates.erase(candidate);
  }

  void pruneHistoryCandidatesLocked(const std::chrono::steady_clock::time_point now)
  {
    const auto expire_after = std::chrono::duration<double>(
      std::max(1.0, history_confirmation_window_ * 2.0));
    const auto prune = [now, expire_after](auto & candidates) {
        for (auto it = candidates.begin(); it != candidates.end();) {
          if (now >= it->second.last_received && now - it->second.last_received > expire_after) {
            it = candidates.erase(it);
          } else {
            ++it;
          }
        }
      };
    prune(ground_candidates_);
    prune(structure_candidates_);
  }

  void odometryCallback(nav_msgs::msg::Odometry::ConstSharedPtr message)
  {
    if (!message || message->header.frame_id.empty() || !historyWritesAllowed()) return;
    const auto stamp = rclcpp::Time(message->header.stamp, get_clock()->get_clock_type());
    const double source_age = (now() - stamp).seconds();
    if (stamp.nanoseconds() <= 0 || !std::isfinite(source_age) || source_age < -0.1 ||
      source_age > history_health_timeout_ ||
      stamp.nanoseconds() <= last_trajectory_source_stamp_ns_)
    {
      return;
    }
    geometry_msgs::msg::PoseStamped pose;
    pose.header = message->header;
    pose.pose = message->pose.pose;
    if (pose.header.frame_id != odom_frame_) {
      try {
        pose = tf_buffer_.transform(pose, odom_frame_, tf2::durationFromSec(0.05));
      } catch (const tf2::TransformException & error) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 3000,
          "Cannot transform odometry into trajectory frame '%s': %s",
          odom_frame_.c_str(), error.what());
        return;
      }
    }
    pose.header.frame_id = odom_frame_;
    last_trajectory_source_stamp_ns_ = stamp.nanoseconds();
    std::lock_guard<std::mutex> lock(history_mutex_);
    if (!trajectory_.empty()) {
      const auto & previous = trajectory_.back().pose.position;
      const auto & current = pose.pose.position;
      const double dx = current.x - previous.x;
      const double dy = current.y - previous.y;
      if (dx * dx + dy * dy < trajectory_min_distance_ * trajectory_min_distance_) return;
    }
    trajectory_.push_back(std::move(pose));
    while (trajectory_.size() > trajectory_max_poses_) trajectory_.pop_front();
  }

  void publishHistory()
  {
    std::vector<DisplayPoint> ground_history;
    std::vector<DisplayPoint> structure_history;
    nav_msgs::msg::Path trajectory;
    const auto steady_now = std::chrono::steady_clock::now();
    {
      std::lock_guard<std::mutex> lock(history_mutex_);
      pruneHistoryCandidatesLocked(steady_now);
      ground_history.reserve(ground_history_.size());
      structure_history.reserve(structure_history_.size());
      for (const auto & entry : ground_history_) ground_history.push_back(entry.second);
      for (const auto & entry : structure_history_) structure_history.push_back(entry.second);
      trajectory.header.frame_id = odom_frame_;
      trajectory.header.stamp = get_clock()->now();
      trajectory.poses.assign(trajectory_.begin(), trajectory_.end());
    }
    const auto stamp = get_clock()->now();
    history_ground_publisher_->publish(makeColoredCloud(ground_history, odom_frame_, stamp));
    history_structure_publisher_->publish(makeColoredCloud(
      structure_history, odom_frame_, stamp));
    trajectory_publisher_->publish(trajectory);
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 10000,
      "Persistent display history: ground=%zu/%zu structure=%zu/%zu trajectory=%zu",
      ground_history.size(), history_max_ground_voxels_, structure_history.size(),
      history_max_structure_voxels_, trajectory.poses.size());
  }

  void observeRosClock(const rclcpp::Time & now)
  {
    const auto now_ns = now.nanoseconds();
    if (have_last_ros_clock_time_ && now_ns < last_ros_clock_time_ns_) {
      last_accepted_source_stamp_ns_ = 0;
      clearDisplay();
      RCLCPP_WARN(get_logger(), "ROS clock moved backward; reset live display timestamp guard");
    }
    have_last_ros_clock_time_ = true;
    last_ros_clock_time_ns_ = now_ns;
  }

  void cloudCallback(const PointCloud::ConstSharedPtr message)
  {
    const bool want_ground = ground_publisher_->get_subscription_count() > 0;
    const bool want_structure = structure_publisher_->get_subscription_count() > 0;
    if (!message || message->header.frame_id.empty()) return;

    const auto now = get_clock()->now();
    observeRosClock(now);
    const auto received_at = std::chrono::steady_clock::now();
    const auto source_stamp = rclcpp::Time(
      message->header.stamp, get_clock()->get_clock_type());
    const double source_age = (now - source_stamp).seconds();
    if (source_stamp.nanoseconds() <= 0 || !std::isfinite(source_age) ||
      source_age > cloud_max_age_ || source_age < -future_stamp_tolerance_)
    {
      clearDisplay();
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Clearing stale/future live cloud (age %.3f s)", source_age);
      return;
    }
    if (last_accepted_source_stamp_ns_ > 0 &&
      source_stamp.nanoseconds() <= last_accepted_source_stamp_ns_)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Ignoring duplicate/out-of-order live cloud (stamp=%ld last=%ld)",
        static_cast<long>(source_stamp.nanoseconds()),
        static_cast<long>(last_accepted_source_stamp_ns_));
      return;
    }

    tf2::Transform odom_from_cloud;
    odom_from_cloud.setIdentity();
    try {
      if (message->header.frame_id != odom_frame_) {
        const auto stamped = tf_buffer_.lookupTransform(
          odom_frame_, message->header.frame_id, source_stamp,
          rclcpp::Duration::from_seconds(0.05));
        tf2::fromMsg(stamped.transform, odom_from_cloud);
      }
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "Cannot transform live cloud to '%s' at source time: %s",
        odom_frame_.c_str(), error.what());
      return;
    }

    const auto snapshot = groundSnapshot();
    const bool terrain_fresh = terrainSnapshotFresh(snapshot, now);
    const bool history_enabled = historyWritesAllowed();
    tf2::Transform terrain_from_cloud;
    terrain_from_cloud.setIdentity();
    bool terrain_transform_valid = terrain_fresh;
    if (terrain_fresh) {
      const auto & terrain_frame = snapshot->map.getFrameId();
      try {
        if (terrain_frame == odom_frame_) {
          terrain_from_cloud = odom_from_cloud;
        } else if (terrain_frame != message->header.frame_id) {
          const auto stamped = tf_buffer_.lookupTransform(
            terrain_frame, message->header.frame_id, source_stamp,
            rclcpp::Duration::from_seconds(0.05));
          tf2::fromMsg(stamped.transform, terrain_from_cloud);
        }
      } catch (const tf2::TransformException & error) {
        terrain_transform_valid = false;
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 3000,
          "Cannot transform live cloud to FastDEM frame '%s': %s",
          terrain_frame.c_str(), error.what());
      }
    }

    const auto sensor_origin = odom_from_cloud.getOrigin();
    const double fallback_ground_odom = sensor_origin.z() - sensor_height_;
    const double radius_squared = display_radius_ * display_radius_;
    const auto point_count = static_cast<std::size_t>(message->width) * message->height;
    std::vector<DisplayPoint> ground_points;
    std::vector<DisplayPoint> structure_points;
    ground_points.reserve(want_ground ? point_count / 8 : 0);
    structure_points.reserve(want_structure ? point_count : 0);
    std::unordered_set<std::uint64_t> ground_voxels;
    ground_voxels.reserve(want_ground ? point_count / 8 : 0);
    std::size_t fallback_count = 0;
    std::size_t clipped_count = 0;
    try {
      sensor_msgs::PointCloud2ConstIterator<float> in_x(*message, "x");
      sensor_msgs::PointCloud2ConstIterator<float> in_y(*message, "y");
      sensor_msgs::PointCloud2ConstIterator<float> in_z(*message, "z");
      for (std::size_t i = 0; i < point_count; ++i, ++in_x, ++in_y, ++in_z) {
        if (!std::isfinite(*in_x) || !std::isfinite(*in_y) || !std::isfinite(*in_z)) continue;
        const tf2::Vector3 point_cloud(*in_x, *in_y, *in_z);
        const auto point_odom = odom_from_cloud * point_cloud;
        const double dx = point_odom.x() - sensor_origin.x();
        const double dy = point_odom.y() - sensor_origin.y();
        if (dx * dx + dy * dy > radius_squared) continue;

        float ground_elevation = 0.0F;
        double height_above_ground;
        bool used_fallback = true;
        if (terrain_transform_valid) {
          const auto point_terrain = terrain_from_cloud * point_cloud;
          if (groundElevationAt(
              *snapshot, point_terrain.x(), point_terrain.y(), ground_elevation))
          {
            height_above_ground = point_terrain.z() - ground_elevation;
            used_fallback = false;
          }
        }
        if (used_fallback) {
          height_above_ground = point_odom.z() - fallback_ground_odom;
          ++fallback_count;
        }
        if (!std::isfinite(height_above_ground)) continue;
        if (height_above_ground > max_height_above_ground_) {
          ++clipped_count;
          continue;
        }

        const DisplayPoint display_point{
          static_cast<float>(point_odom.x()), static_cast<float>(point_odom.y()),
          static_cast<float>(point_odom.z()), static_cast<float>(height_above_ground)};
        if (std::abs(height_above_ground) <= ground_band_) {
          const auto voxel_key = groundVoxelKey(
            display_point.x, display_point.y, ground_voxel_size_);
          if (history_enabled) {
            observeHistoryVoxel(
              ground_history_, ground_candidates_, voxel_key, display_point,
              source_stamp.nanoseconds(), static_cast<std::uint16_t>(used_fallback ?
              fallback_history_confirmation_frames_ : history_confirmation_frames_),
              static_cast<std::size_t>(history_max_ground_voxels_), !used_fallback);
          }
          if (want_ground && ground_voxels.insert(voxel_key).second) {
            ground_points.push_back(display_point);
          }
        } else {
          if (history_enabled) {
            observeHistoryVoxel(
              structure_history_, structure_candidates_,
              structureVoxelKey(display_point, structure_voxel_size_), display_point,
              source_stamp.nanoseconds(), static_cast<std::uint16_t>(used_fallback ?
              fallback_history_confirmation_frames_ : history_confirmation_frames_),
              static_cast<std::size_t>(history_max_structure_voxels_), !used_fallback);
          }
          if (want_structure) structure_points.push_back(display_point);
        }
      }
    } catch (const std::runtime_error & error) {
      clearDisplay();
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "Clearing live display cloud without x/y/z fields: %s", error.what());
      return;
    }

    last_accepted_source_stamp_ns_ = source_stamp.nanoseconds();
    if (!want_ground && !want_structure) return;
    if (want_ground) {
      ground_publisher_->publish(makeColoredCloud(ground_points, odom_frame_, source_stamp));
    }
    if (want_structure) {
      structure_publisher_->publish(makeColoredCloud(
        structure_points, odom_frame_, source_stamp));
    }
    last_display_source_stamp_ns_ = source_stamp.nanoseconds();
    last_display_received_at_ = received_at;
    display_visible_ = true;
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Live display cloud: input=%zu ground=%zu structure=%zu clipped_above=%.2f m:%zu "
      "terrain_fallback=%zu terrain_fresh=%s radius=%.1f m",
      point_count, ground_points.size(), structure_points.size(),
      max_height_above_ground_, clipped_count, fallback_count,
      terrain_fresh ? "true" : "false", display_radius_);
  }

  void publishEmptyClouds()
  {
    const std::vector<DisplayPoint> no_points;
    const auto stamp = get_clock()->now();
    ground_publisher_->publish(makeColoredCloud(no_points, odom_frame_, stamp));
    structure_publisher_->publish(makeColoredCloud(no_points, odom_frame_, stamp));
    display_visible_ = false;
  }

  void clearDisplay()
  {
    if (!display_visible_) return;
    publishEmptyClouds();
    RCLCPP_INFO(get_logger(), "Cleared stale live ground/structure display");
  }

  void clearExpiredDisplay()
  {
    const auto now = get_clock()->now();
    observeRosClock(now);
    if (!display_visible_) return;
    const auto source_stamp = rclcpp::Time(
      last_display_source_stamp_ns_, get_clock()->get_clock_type());
    const double source_age = (now - source_stamp).seconds();
    const double receive_age = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - last_display_received_at_).count();
    if (!std::isfinite(source_age) || source_age > cloud_max_age_ ||
      source_age < -future_stamp_tolerance_ || !std::isfinite(receive_age) ||
      receive_age > point_retention_timeout_)
    {
      clearDisplay();
      RCLCPP_INFO(
        get_logger(), "Expired live display (source_age=%.3f s receive_age=%.3f s)",
        source_age, receive_age);
    }
  }

  std::string input_topic_;
  std::string terrain_topic_;
  std::string ground_topic_;
  std::string structure_topic_;
  std::string history_ground_topic_;
  std::string history_structure_topic_;
  std::string trajectory_topic_;
  std::string history_health_topic_;
  std::string odom_frame_;
  double sensor_height_{0.25};
  double max_height_above_ground_{1.2};
  double display_radius_{5.0};
  double ground_voxel_size_{0.12};
  double structure_voxel_size_{0.12};
  double ground_band_{0.10};
  double terrain_max_age_{1.0};
  double future_stamp_tolerance_{0.1};
  double cloud_max_age_{1.0};
  double point_retention_timeout_{1.0};
  double history_confirmation_window_{0.5};
  double history_publish_period_{2.0};
  double trajectory_min_distance_{0.05};
  double history_health_timeout_{0.5};
  std::int64_t history_confirmation_window_ns_{500000000};
  std::int64_t last_trajectory_source_stamp_ns_{0};
  int min_terrain_observations_{3};
  int max_terrain_observation_age_{3};
  int history_confirmation_frames_{2};
  int fallback_history_confirmation_frames_{5};
  int history_max_ground_voxels_{300000};
  int history_max_structure_voxels_{300000};
  int history_candidate_limit_{30000};
  int trajectory_max_poses_{20000};
  mutable std::mutex terrain_mutex_;
  std::shared_ptr<const GroundSnapshot> terrain_snapshot_;
  std::mutex history_mutex_;
  std::unordered_map<std::uint64_t, DisplayPoint> ground_history_;
  std::unordered_map<std::uint64_t, HistoryCandidate> ground_candidates_;
  std::unordered_map<StructureVoxelKey, DisplayPoint, StructureVoxelKeyHash> structure_history_;
  std::unordered_map<StructureVoxelKey, HistoryCandidate, StructureVoxelKeyHash>
    structure_candidates_;
  std::deque<geometry_msgs::msg::PoseStamped> trajectory_;
  std::int64_t last_accepted_source_stamp_ns_{0};
  std::int64_t last_display_source_stamp_ns_{0};
  std::int64_t last_ros_clock_time_ns_{0};
  std::chrono::steady_clock::time_point last_display_received_at_;
  bool have_last_ros_clock_time_{false};
  bool display_visible_{false};
  bool history_health_known_{false};
  bool history_health_ok_{false};
  std::chrono::steady_clock::time_point history_health_received_at_{};
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<PointCloud>::SharedPtr cloud_subscription_;
  rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr terrain_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr history_health_subscription_;
  rclcpp::Publisher<PointCloud>::SharedPtr ground_publisher_;
  rclcpp::Publisher<PointCloud>::SharedPtr structure_publisher_;
  rclcpp::Publisher<PointCloud>::SharedPtr history_ground_publisher_;
  rclcpp::Publisher<PointCloud>::SharedPtr history_structure_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr trajectory_publisher_;
  rclcpp::TimerBase::SharedPtr clear_timer_;
  rclcpp::TimerBase::SharedPtr history_publish_timer_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NavigationCloudDisplayNode>());
  rclcpp::shutdown();
  return 0;
}
