#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/u_int64.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "bringup/temporal_voxel_map.hpp"

class StaticMapFilterNode : public rclcpp::Node
{
public:
  StaticMapFilterNode()
  : Node("static_map_filter")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/cloud_registered");
    temporal_parameters_.voxel_size = declare_parameter<double>("voxel_size", 0.18);
    temporal_parameters_.min_observations = declare_parameter<int>("min_observations", 4);
    temporal_parameters_.min_observation_duration =
      declare_parameter<double>("min_observation_duration", 0.4);
    temporal_parameters_.candidate_timeout = declare_parameter<double>("candidate_timeout", 1.5);
    temporal_parameters_.confirmed_miss_limit =
      declare_parameter<int>("confirmed_miss_limit", 10);
    temporal_parameters_.archive_min_observation_duration =
      declare_parameter<double>("archive_min_observation_duration", 1.0);
    temporal_parameters_.archive_min_origin_separation =
      declare_parameter<double>("archive_min_origin_separation", 0.5);
    const double ray_angular_resolution_deg =
      declare_parameter<double>("ray_angular_resolution_deg", 2.0);
    temporal_parameters_.ray_angular_resolution =
      ray_angular_resolution_deg * 3.14159265358979323846 / 180.0;
    temporal_parameters_.min_range = declare_parameter<double>("min_range", 0.5);
    temporal_parameters_.max_range = declare_parameter<double>("max_range", 40.0);
    map_file_path_ = declare_parameter<std::string>("map_file_path", "clean_map.pcd");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/Odometry");
    window_radius_ = declare_parameter<double>("window_radius", 5.0);
    if (window_radius_ <= 0.0) {
      throw std::invalid_argument("window_radius must be positive");
    }
    publish_full_static_map_ = declare_parameter<bool>("publish_full_static_map", false);
    static_map_publish_period_ = declare_parameter<double>("static_map_publish_period", 1.0);
    if (static_map_publish_period_ <= 0.0) {
      throw std::invalid_argument("static_map_publish_period must be positive");
    }
    temporal_map_ = std::make_unique<bringup::TemporalVoxelMap>(temporal_parameters_);

    cloud_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, rclcpp::SensorDataQoS(),
      std::bind(&StaticMapFilterNode::cloud_callback, this, std::placeholders::_1));
    odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::SensorDataQoS(),
      std::bind(&StaticMapFilterNode::odom_callback, this, std::placeholders::_1));
    static_cloud_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/dynamic_filter/static_cloud", rclcpp::QoS(1).transient_local());
    static_map_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/static_map", rclcpp::QoS(1).transient_local());
    window_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/cloud_registered_window", rclcpp::SensorDataQoS().keep_last(1));
    rejected_cloud_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/dynamic_filter/rejected_cloud", rclcpp::SensorDataQoS());
    processed_frames_publisher_ = create_publisher<std_msgs::msg::UInt64>(
      "/dynamic_filter/processed_frames", rclcpp::QoS(1).transient_local());
    save_service_ = create_service<std_srvs::srv::Trigger>(
      "/static_map_save",
      std::bind(
        &StaticMapFilterNode::save_callback, this, std::placeholders::_1,
        std::placeholders::_2));
    static_map_timer_ = create_wall_timer(
      std::chrono::milliseconds(std::max<int64_t>(
        100, static_cast<int64_t>(static_map_publish_period_ * 1000.0))),
      std::bind(&StaticMapFilterNode::maintenance_callback, this));

    RCLCPP_INFO(
      get_logger(),
      "Filtering %s: voxel=%.3f m, observations=%d, duration=%.3f s, timeout=%.3f s, misses=%d, archive=(%.3f s, %.2f m), range=[%.2f, %.2f] m",
      input_topic_.c_str(), temporal_parameters_.voxel_size,
      temporal_parameters_.min_observations, temporal_parameters_.min_observation_duration,
      temporal_parameters_.candidate_timeout, temporal_parameters_.confirmed_miss_limit,
      temporal_parameters_.archive_min_observation_duration,
      temporal_parameters_.archive_min_origin_separation,
      temporal_parameters_.min_range, temporal_parameters_.max_range);
  }

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr message)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    temporal_map_->set_origin(
      message->pose.pose.position.x,
      message->pose.pose.position.y,
      message->pose.pose.position.z);
    window_center_x_ = message->pose.pose.position.x;
    window_center_y_ = message->pose.pose.position.y;
    has_odom_ = true;
  }

  void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr message)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    double stamp = rclcpp::Time(message->header.stamp).seconds();
    if (stamp <= 0.0) {
      stamp = get_clock()->now().seconds();
    }

    std::vector<pcl::PointXYZ> points;
    points.reserve(message->width * message->height);
    try {
      sensor_msgs::PointCloud2ConstIterator<float> iter_x(*message, "x");
      sensor_msgs::PointCloud2ConstIterator<float> iter_y(*message, "y");
      sensor_msgs::PointCloud2ConstIterator<float> iter_z(*message, "z");
      for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
        points.emplace_back(*iter_x, *iter_y, *iter_z);
      }
    } catch (const std::runtime_error & error) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Point cloud must contain float x/y/z fields: %s", error.what());
      return;
    }

    const bringup::TemporalVoxelFrameResult result = temporal_map_->process_frame(points, stamp);
    ++processed_frames_;
    static_map_dirty_ = static_map_dirty_ || result.map_changed;
    last_header_ = message->header;
    has_header_ = true;
    last_cloud_stamp_ = stamp;

    if (static_cloud_publisher_->get_subscription_count()) {
      static_cloud_publisher_->publish(
        make_cloud_message(result.current_static_points, message->header));
    }
    if (rejected_cloud_publisher_->get_subscription_count()) {
      rejected_cloud_publisher_->publish(make_cloud_message(result.rejected_points, message->header));
    }
  }

  void maintenance_callback()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    temporal_map_->cleanup_candidates();
    std_msgs::msg::UInt64 status;
    status.data = processed_frames_;
    processed_frames_publisher_->publish(status);
    if (window_publisher_->get_subscription_count() && has_header_ && has_odom_) {
      const float radius_squared = static_cast<float>(window_radius_ * window_radius_);
      std::vector<pcl::PointXYZ> window_points;
      const auto & static_points = temporal_map_->static_points();
      window_points.reserve(static_points.size());
      for (const auto & point : static_points) {
        const float dx = point.x - static_cast<float>(window_center_x_);
        const float dy = point.y - static_cast<float>(window_center_y_);
        if (dx * dx + dy * dy <= radius_squared) {
          window_points.push_back(point);
        }
      }
      window_publisher_->publish(make_cloud_message(window_points, last_header_));
    }
    if (!publish_full_static_map_ || !static_map_dirty_ ||
      !static_map_publisher_->get_subscription_count() || !has_header_)
    {
      return;
    }
    static_map_publisher_->publish(
      make_cloud_message(temporal_map_->static_points(), last_header_));
    static_map_dirty_ = false;
  }

  sensor_msgs::msg::PointCloud2 make_cloud_message(
    const std::vector<pcl::PointXYZ> & points, const std_msgs::msg::Header & header) const
  {
    sensor_msgs::msg::PointCloud2 message;
    message.header = header;
    sensor_msgs::PointCloud2Modifier modifier(message);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(points.size());
    sensor_msgs::PointCloud2Iterator<float> iter_x(message, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(message, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(message, "z");
    for (const auto & point : points) {
      *iter_x = point.x;
      *iter_y = point.y;
      *iter_z = point.z;
      ++iter_x;
      ++iter_y;
      ++iter_z;
    }
    return message;
  }

  void save_callback(
    const std_srvs::srv::Trigger::Request::SharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const std::vector<pcl::PointXYZ> points = temporal_map_->complete_static_points();
    std::string error_message;
    response->success = bringup::save_points_to_pcd(map_file_path_, points, &error_message);
    response->message = response->success ?
      "Saved " + std::to_string(points.size()) + " points to " + map_file_path_ : error_message;
  }

  std::string input_topic_;
  std::string odom_topic_;
  std::string map_file_path_;
  bringup::TemporalVoxelParameters temporal_parameters_;
  std::unique_ptr<bringup::TemporalVoxelMap> temporal_map_;
  double static_map_publish_period_;
  double window_radius_{5.0};
  double window_center_x_{0.0};
  double window_center_y_{0.0};
  double last_cloud_stamp_{0.0};
  bool publish_full_static_map_{false};
  bool static_map_dirty_{false};
  bool has_header_{false};
  bool has_odom_{false};
  std::uint64_t processed_frames_{0};
  std_msgs::msg::Header last_header_;
  std::mutex mutex_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr static_cloud_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr static_map_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr window_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr rejected_cloud_publisher_;
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr processed_frames_publisher_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_service_;
  rclcpp::TimerBase::SharedPtr static_map_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StaticMapFilterNode>());
  rclcpp::shutdown();
  return 0;
}
