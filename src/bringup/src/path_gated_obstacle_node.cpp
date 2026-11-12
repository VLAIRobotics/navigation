#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/header.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace bringup
{

class PathGatedObstacleNode : public rclcpp::Node
{
  struct VoxelKey
  {
    int x;
    int y;
    int z;

    bool operator==(const VoxelKey & other) const
    {
      return x == other.x && y == other.y && z == other.z;
    }
  };

  struct VoxelKeyHash
  {
    std::size_t operator()(const VoxelKey & key) const
    {
      std::size_t value = std::hash<int>{}(key.x);
      value ^= std::hash<int>{}(key.y) + 0x9e3779b9U + (value << 6U) + (value >> 2U);
      value ^= std::hash<int>{}(key.z) + 0x9e3779b9U + (value << 6U) + (value >> 2U);
      return value;
    }
  };

  struct TrackedObstacle
  {
    tf2::Vector3 position;
    unsigned int missed_frames{0};
    bool seen{false};
  };

  using TrackedObstacles = std::unordered_map<VoxelKey, TrackedObstacle, VoxelKeyHash>;

public:
  PathGatedObstacleNode()
  : Node("path_gated_obstacle"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
  {
    blocking_topic_ = declare_parameter("blocking_topic", std::string("/obstacles/blocking"));
    clearing_topic_ = declare_parameter("clearing_topic", std::string("/cloud_registered_body"));
    path_topic_ = declare_parameter("path_topic", std::string("/plan"));
    global_frame_ = declare_parameter("global_frame", std::string("map"));
    path_blocking_topic_ = declare_parameter(
      "path_blocking_topic", std::string("/obstacles/global_path_blocking"));
    global_clearing_topic_ = declare_parameter(
      "global_clearing_topic", std::string("/obstacles/global_clearing"));
    path_corridor_radius_ = declare_parameter("path_corridor_radius", 0.50);
    publish_rate_hz_ = declare_parameter("publish_rate_hz", 1.0);
    input_timeout_sec_ = declare_parameter("input_timeout_sec", 1.5);
    path_timeout_sec_ = declare_parameter("path_timeout_sec", 3.0);

    if (path_corridor_radius_ <= 0.0 || publish_rate_hz_ <= 0.0 || input_timeout_sec_ <= 0.0 ||
      path_timeout_sec_ <= 0.0)
    {
      throw std::invalid_argument(
              "path corridor radius, publish rate, input timeout, and path timeout must be positive");
    }

    const auto sensor_qos = rclcpp::SensorDataQoS();
    const auto output_qos = rclcpp::QoS(1).reliable();
    blocking_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      path_blocking_topic_, output_qos);
    clearing_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      global_clearing_topic_, output_qos);
    blocking_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      blocking_topic_, sensor_qos,
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr message) {
        latest_blocking_ = std::move(message);
      });
    clearing_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      clearing_topic_, sensor_qos,
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr message) {
        latest_clearing_ = std::move(message);
      });
    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      path_topic_, rclcpp::QoS(1).reliable(),
      [this](nav_msgs::msg::Path::ConstSharedPtr message) {
        latest_path_ = std::move(message);
      });

    const auto period = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::duration<double>(1.0 / publish_rate_hz_));
    timer_ = create_wall_timer(
      std::max(period, std::chrono::milliseconds(1)),
      std::bind(&PathGatedObstacleNode::publishUpdates, this));
    RCLCPP_INFO(
      get_logger(),
      "Publishing global obstacle observations at %.2f Hz within %.2f m of %s",
      publish_rate_hz_, path_corridor_radius_, path_topic_.c_str());
  }

private:
  static constexpr unsigned int kTrackMissedFramesLimit = 3;

  bool isFresh(const std_msgs::msg::Header & header, const rclcpp::Time & now,
    double timeout_sec) const
  {
    if (header.stamp.sec == 0 && header.stamp.nanosec == 0) return false;
    const auto age = (now - rclcpp::Time(header.stamp)).seconds();
    return age >= -0.1 && age <= timeout_sec;
  }

  static double distanceSquaredToPath(
    double x, double y, const std::vector<geometry_msgs::msg::PoseStamped> & poses)
  {
    if (poses.size() == 1) {
      const double dx = x - poses.front().pose.position.x;
      const double dy = y - poses.front().pose.position.y;
      return dx * dx + dy * dy;
    }

    double best = std::numeric_limits<double>::infinity();
    for (std::size_t i = 1; i < poses.size(); ++i) {
      const auto & from = poses[i - 1].pose.position;
      const auto & to = poses[i].pose.position;
      const double vx = to.x - from.x;
      const double vy = to.y - from.y;
      const double length_squared = vx * vx + vy * vy;
      const double projection = length_squared > 1e-12 ?
        std::clamp(((x - from.x) * vx + (y - from.y) * vy) / length_squared, 0.0, 1.0) : 0.0;
      const double dx = x - (from.x + projection * vx);
      const double dy = y - (from.y + projection * vy);
      best = std::min(best, dx * dx + dy * dy);
    }
    return best;
  }

  static VoxelKey voxelKey(const tf2::Vector3 & point)
  {
    constexpr double cell_size = 0.15;
    return {
      static_cast<int>(std::floor(point.x() / cell_size)),
      static_cast<int>(std::floor(point.y() / cell_size)),
      static_cast<int>(std::floor(point.z() / cell_size))};
  }

  TrackedObstacles::iterator findTrackedObstacle(
    const tf2::Vector3 & point, const VoxelKey & key)
  {
    constexpr double association_radius_squared = 0.22 * 0.22;
    auto nearest = tracked_obstacles_.end();
    double nearest_distance_squared = association_radius_squared;
    for (int dx = -1; dx <= 1; ++dx) {
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dz = -1; dz <= 1; ++dz) {
          const auto candidate = tracked_obstacles_.find({key.x + dx, key.y + dy, key.z + dz});
          if (candidate == tracked_obstacles_.end()) continue;
          const double distance_squared = (candidate->second.position - point).length2();
          if (distance_squared <= nearest_distance_squared) {
            nearest = candidate;
            nearest_distance_squared = distance_squared;
          }
        }
      }
    }
    return nearest;
  }

  sensor_msgs::msg::PointCloud2 trackedBlockingCloud(
    const sensor_msgs::msg::PointCloud2 & cloud,
    const nav_msgs::msg::Path & path,
    bool path_is_fresh)
  {
    sensor_msgs::msg::PointCloud2 output;
    output.header = cloud.header;
    const auto transform_message = tf_buffer_.lookupTransform(
      global_frame_, cloud.header.frame_id, rclcpp::Time(cloud.header.stamp),
      rclcpp::Duration::from_seconds(0.05));
    tf2::Transform map_from_cloud;
    tf2::fromMsg(transform_message.transform, map_from_cloud);
    const auto cloud_from_map = map_from_cloud.inverse();

    // Keep route-triggered obstacles marked after replanning until scans miss them repeatedly.
    for (auto & entry : tracked_obstacles_) entry.second.seen = false;

    sensor_msgs::PointCloud2ConstIterator<float> x(cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> y(cloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float> z(cloud, "z");
    const double radius_squared = path_corridor_radius_ * path_corridor_radius_;
    const auto process_point = [this, &path, &map_from_cloud, path_is_fresh, radius_squared](
        float point_x, float point_y, float point_z, bool clear) {
        if (clear || !std::isfinite(point_x) || !std::isfinite(point_y) ||
          !std::isfinite(point_z))
        {
          return;
        }
        const auto map_point = map_from_cloud * tf2::Vector3(point_x, point_y, point_z);
        const auto key = voxelKey(map_point);
        auto tracked = findTrackedObstacle(map_point, key);
        if (tracked != tracked_obstacles_.end()) {
          auto obstacle = tracked->second;
          obstacle.position = map_point;
          obstacle.missed_frames = 0;
          obstacle.seen = true;
          if (!(tracked->first == key) && tracked_obstacles_.find(key) == tracked_obstacles_.end()) {
            tracked_obstacles_.erase(tracked);
            tracked_obstacles_.emplace(key, obstacle);
          } else {
            tracked->second = obstacle;
          }
          return;
        }

        if (path_is_fresh &&
          distanceSquaredToPath(map_point.x(), map_point.y(), path.poses) <= radius_squared)
        {
          tracked_obstacles_.emplace(
            key, TrackedObstacle{map_point, 0, true});
        }
      };
    const auto operation_field = std::find_if(
      cloud.fields.begin(), cloud.fields.end(),
      [](const sensor_msgs::msg::PointField & field) {return field.name == "operation";});
    const bool has_operation = operation_field != cloud.fields.end();
    if (has_operation &&
      (operation_field->datatype != sensor_msgs::msg::PointField::UINT8 ||
      operation_field->count != 1))
    {
      throw std::runtime_error("blocking cloud has an invalid operation field");
    }
    if (has_operation) {
      sensor_msgs::PointCloud2ConstIterator<std::uint8_t> operation(cloud, "operation");
      for (; x != x.end(); ++x, ++y, ++z, ++operation) {
        process_point(*x, *y, *z, *operation != 0U);
      }
    } else {
      for (; x != x.end(); ++x, ++y, ++z) {
        process_point(*x, *y, *z, false);
      }
    }

    for (auto it = tracked_obstacles_.begin(); it != tracked_obstacles_.end();) {
      if (!it->second.seen) ++it->second.missed_frames;
      if (it->second.missed_frames >= kTrackMissedFramesLimit) {
        it = tracked_obstacles_.erase(it);
      } else {
        ++it;
      }
    }

    std::vector<geometry_msgs::msg::Point> matched;
    matched.reserve(tracked_obstacles_.size());
    for (const auto & entry : tracked_obstacles_) {
      const auto cloud_point = cloud_from_map * entry.second.position;
      geometry_msgs::msg::Point result;
      result.x = cloud_point.x();
      result.y = cloud_point.y();
      result.z = cloud_point.z();
      matched.push_back(result);
    }

    sensor_msgs::PointCloud2Modifier modifier(output);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(matched.size());
    sensor_msgs::PointCloud2Iterator<float> out_x(output, "x");
    sensor_msgs::PointCloud2Iterator<float> out_y(output, "y");
    sensor_msgs::PointCloud2Iterator<float> out_z(output, "z");
    for (const auto & point : matched) {
      *out_x = static_cast<float>(point.x);
      *out_y = static_cast<float>(point.y);
      *out_z = static_cast<float>(point.z);
      ++out_x;
      ++out_y;
      ++out_z;
    }
    return output;
  }

  void publishUpdates()
  {
    const auto current_time = now();
    std::size_t matched_count = 0;
    const bool path_is_fresh = latest_path_ && latest_path_->header.frame_id == global_frame_ &&
      !latest_path_->poses.empty() &&
      isFresh(latest_path_->header, current_time, path_timeout_sec_);
    if (latest_path_ && latest_path_->header.frame_id != global_frame_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Ignoring path in unexpected frame '%s' (expected '%s')",
        latest_path_->header.frame_id.c_str(), global_frame_.c_str());
    }
    if (latest_blocking_ && isFresh(latest_blocking_->header, current_time, input_timeout_sec_))
    {
      const auto stamp_ns = rclcpp::Time(latest_blocking_->header.stamp).nanoseconds();
      if (!have_blocking_stamp_ || stamp_ns > last_blocking_stamp_ns_) {
        try {
          const nav_msgs::msg::Path empty_path;
          const auto & path = latest_path_ ? *latest_path_ : empty_path;
          auto blocking = trackedBlockingCloud(*latest_blocking_, path, path_is_fresh);
          matched_count = tracked_obstacles_.size();
          if (matched_count > 0) blocking_pub_->publish(blocking);
        } catch (const tf2::TransformException & error) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 5000, "Cannot check path obstacles against current path: %s",
            error.what());
        } catch (const std::runtime_error & error) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 5000, "Cannot read blocking cloud: %s", error.what());
        }
        last_blocking_stamp_ns_ = stamp_ns;
        have_blocking_stamp_ = true;
      }
    }

    if (latest_clearing_ && isFresh(latest_clearing_->header, current_time, input_timeout_sec_)) {
      const auto stamp_ns = rclcpp::Time(latest_clearing_->header.stamp).nanoseconds();
      if (!have_clearing_stamp_ || stamp_ns > last_clearing_stamp_ns_) {
        clearing_pub_->publish(*latest_clearing_);
        last_clearing_stamp_ns_ = stamp_ns;
        have_clearing_stamp_ = true;
      }
    }

    if (matched_count > 0) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Global costmap received %zu path-affecting obstacle points", matched_count);
    }
  }

  std::string blocking_topic_;
  std::string clearing_topic_;
  std::string path_topic_;
  std::string global_frame_;
  std::string path_blocking_topic_;
  std::string global_clearing_topic_;
  double path_corridor_radius_{0.50};
  double publish_rate_hz_{1.0};
  double input_timeout_sec_{1.5};
  double path_timeout_sec_{3.0};
  TrackedObstacles tracked_obstacles_;
  std::int64_t last_blocking_stamp_ns_{0};
  std::int64_t last_clearing_stamp_ns_{0};
  bool have_blocking_stamp_{false};
  bool have_clearing_stamp_{false};
  sensor_msgs::msg::PointCloud2::ConstSharedPtr latest_blocking_;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr latest_clearing_;
  nav_msgs::msg::Path::ConstSharedPtr latest_path_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr blocking_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr clearing_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr blocking_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr clearing_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace bringup

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<bringup::PathGatedObstacleNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("path_gated_obstacle"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
