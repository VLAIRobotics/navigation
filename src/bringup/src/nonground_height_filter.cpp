#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nanogrid/bridge/ros2.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "bringup/no_elevation_obstacle.hpp"
#include "bringup/obstacle_evidence_sequence.hpp"
#include "bringup/temporal_obstacle_grid.hpp"

namespace
{

class NongroundHeightFilter : public rclcpp::Node
{
  using PointCloud = sensor_msgs::msg::PointCloud2;
  using CloudSyncPolicy = message_filters::sync_policies::ApproximateTime<
    PointCloud, PointCloud>;
  using CloudSynchronizer = message_filters::Synchronizer<CloudSyncPolicy>;

public:
  NongroundHeightFilter()
  : Node("nonground_height_filter"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
  {
    ground_veto_publisher_id_ = makePublisherInstanceId();
    input_topic_ = declare_parameter("input_topic", std::string("/patchworkpp/nonground"));
    ground_topic_ = declare_parameter("ground_topic", std::string("/fastdem/mapping/gridmap"));
    blocking_topic_ = declare_parameter("blocking_topic", std::string("/obstacles/blocking"));
    global_blocking_topic_ = declare_parameter(
      "global_blocking_topic", std::string("/obstacles/global_blocking"));
    global_costmap_updates_topic_ = declare_parameter(
      "global_costmap_updates_topic", std::string());
    global_clearing_topic_ = declare_parameter(
      "global_clearing_topic", std::string("/obstacles/global_clearing"));
    ground_veto_topic_ = declare_parameter(
      "ground_veto_topic", std::string("/obstacles/ground_veto"));
    ground_veto_fence_service_name_ = declare_parameter(
      "ground_veto_fence_service", std::string("/obstacles/ground_veto_fence"));
    overhead_topic_ = declare_parameter("overhead_topic", std::string("/obstacles/overhead"));
    robot_height_ = declare_parameter("robot_height", 0.40);
    clearance_margin_ = declare_parameter("clearance_margin", 0.10);
    min_obstacle_height_ = declare_parameter("min_obstacle_height", 0.05);
    sensor_height_ = declare_parameter("sensor_height", 0.25);
    no_elevation_z_max_ = declare_parameter(
      "no_elevation_z_max", robot_height_ + clearance_margin_);
    ground_reference_max_slope_ = declare_parameter("ground_reference_max_slope", 10.0);
    map_timeout_ = declare_parameter("map_timeout", 1.2);
    temporal_obstacle_filter_enabled_ = declare_parameter(
      "temporal_obstacle_filter_enabled", false);
    tracking_frame_ = declare_parameter("tracking_frame", std::string("odom"));
    raytrace_topic_ = declare_parameter(
      "raytrace_topic", std::string("/cloud_registered_body"));
    bringup::TemporalObstacleGridParameters obstacle_grid_parameters;
    obstacle_grid_parameters.resolution = declare_parameter("obstacle_grid_resolution", 0.10);
    obstacle_grid_parameters.confirmation_window_frames = declare_parameter(
      "obstacle_confirmation_window_frames", 5);
    obstacle_grid_parameters.confirmation_hits = declare_parameter(
      "obstacle_confirmation_hits", 2);
    obstacle_grid_parameters.clear_ray_frames = declare_parameter(
      "obstacle_clear_ray_frames", 2);
    obstacle_grid_parameters.max_unobserved_frames = declare_parameter(
      "obstacle_max_unobserved_frames", 10);
    obstacle_grid_parameters.local_window_size = declare_parameter(
      "obstacle_local_window_size", 6.0);
    obstacle_grid_parameters.raytrace_height_tolerance = declare_parameter(
      "obstacle_raytrace_height_tolerance", 0.25);
    obstacle_raytrace_min_range_ = declare_parameter("obstacle_raytrace_min_range", 0.5);
    obstacle_raytrace_max_range_ = declare_parameter("obstacle_raytrace_max_range", 5.0);
    obstacle_grid_parameters.raytrace_min_range = obstacle_raytrace_min_range_;
    obstacle_grid_parameters.raytrace_max_range = obstacle_raytrace_max_range_;
    temporal_obstacle_grid_ = std::make_unique<bringup::TemporalObstacleGrid>(
      obstacle_grid_parameters);
    auto global_obstacle_grid_parameters = obstacle_grid_parameters;
    global_obstacle_grid_parameters.max_unobserved_frames = declare_parameter(
      "global_obstacle_max_unobserved_frames", 10);
    global_obstacle_grid_parameters.local_window_size = declare_parameter(
      "global_obstacle_local_window_size", 20.0);
    global_obstacle_max_range_ = declare_parameter("global_obstacle_max_range", 10.0);
    global_obstacle_raytrace_max_range_ = declare_parameter(
      "global_obstacle_raytrace_max_range", 10.0);
    if (!std::isfinite(global_obstacle_max_range_) || global_obstacle_max_range_ <= 0.0) {
      throw std::invalid_argument("global_obstacle_max_range must be finite and positive");
    }
    global_obstacle_grid_parameters.raytrace_max_range = global_obstacle_raytrace_max_range_;
    global_temporal_obstacle_grid_ = std::make_unique<bringup::TemporalObstacleGrid>(
      global_obstacle_grid_parameters);

    const auto obstacle_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    blocking_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      blocking_topic_, obstacle_qos);
    global_blocking_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      global_blocking_topic_, obstacle_qos);
    if (!global_costmap_updates_topic_.empty()) {
      global_costmap_updates_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        global_costmap_updates_topic_, obstacle_qos);
      global_costmap_temporal_obstacle_grid_ =
        std::make_unique<bringup::TemporalObstacleGrid>(global_obstacle_grid_parameters);
    }
    global_clearing_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      global_clearing_topic_, obstacle_qos);
    ground_veto_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      ground_veto_topic_, obstacle_qos);
    ground_veto_fence_service_ = create_service<std_srvs::srv::Trigger>(
      ground_veto_fence_service_name_,
      std::bind(&NongroundHeightFilter::groundVetoFenceCallback, this,
        std::placeholders::_1, std::placeholders::_2));
    overhead_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      overhead_topic_, obstacle_qos);
    ground_subscription_ = create_subscription<grid_map_msgs::msg::GridMap>(
      ground_topic_, rclcpp::QoS(1),
      std::bind(&NongroundHeightFilter::groundCallback, this, std::placeholders::_1));
    if (temporal_obstacle_filter_enabled_) {
      const auto sensor_qos = rclcpp::SensorDataQoS().get_rmw_qos_profile();
      synchronized_input_subscriber_.subscribe(this, input_topic_, sensor_qos);
      ray_cloud_subscriber_.subscribe(this, raytrace_topic_, sensor_qos);
      cloud_synchronizer_ = std::make_shared<CloudSynchronizer>(
        CloudSyncPolicy(10), synchronized_input_subscriber_, ray_cloud_subscriber_);
      cloud_synchronizer_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(0.005));
      cloud_synchronizer_->registerCallback(
        std::bind(
          &NongroundHeightFilter::synchronizedCloudCallback, this,
          std::placeholders::_1, std::placeholders::_2));
    } else {
      cloud_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        input_topic_, rclcpp::SensorDataQoS(),
        std::bind(&NongroundHeightFilter::cloudCallback, this, std::placeholders::_1));
    }
  }

private:
  static std::uint64_t makePublisherInstanceId()
  {
    std::random_device entropy;
    const auto clock_bits = static_cast<std::uint64_t>(
      std::chrono::steady_clock::now().time_since_epoch().count());
    auto publisher_id = (static_cast<std::uint64_t>(entropy()) << 32) ^
      static_cast<std::uint64_t>(entropy()) ^ clock_bits;
    if (publisher_id == 0) publisher_id = 1;
    return publisher_id;
  }

  PointCloud makeGroundVetoCloud(
    const std_msgs::msg::Header & header,
    const std::vector<geometry_msgs::msg::Point> & points,
    const bringup::ObstacleEvidenceFrameMetadata & metadata) const
  {
    PointCloud result;
    result.header = header;
    sensor_msgs::PointCloud2Modifier modifier(result);
    modifier.setPointCloud2Fields(
      7,
      "evidence_publisher_hi", 1, sensor_msgs::msg::PointField::UINT32,
      "evidence_publisher_lo", 1, sensor_msgs::msg::PointField::UINT32,
      "evidence_sequence_hi", 1, sensor_msgs::msg::PointField::UINT32,
      "evidence_sequence_lo", 1, sensor_msgs::msg::PointField::UINT32,
      "x", 1, sensor_msgs::msg::PointField::FLOAT32,
      "y", 1, sensor_msgs::msg::PointField::FLOAT32,
      "z", 1, sensor_msgs::msg::PointField::FLOAT32);
    const bool has_points = !points.empty();
    modifier.resize(has_points ? points.size() : 1);

    sensor_msgs::PointCloud2Iterator<std::uint32_t> publisher_hi(
      result, "evidence_publisher_hi");
    sensor_msgs::PointCloud2Iterator<std::uint32_t> publisher_lo(
      result, "evidence_publisher_lo");
    sensor_msgs::PointCloud2Iterator<std::uint32_t> sequence_hi(
      result, "evidence_sequence_hi");
    sensor_msgs::PointCloud2Iterator<std::uint32_t> sequence_lo(
      result, "evidence_sequence_lo");
    sensor_msgs::PointCloud2Iterator<float> x(result, "x");
    sensor_msgs::PointCloud2Iterator<float> y(result, "y");
    sensor_msgs::PointCloud2Iterator<float> z(result, "z");
    const auto write_metadata = [&]() {
        *publisher_hi = static_cast<std::uint32_t>(metadata.publisher_id >> 32);
        *publisher_lo = static_cast<std::uint32_t>(metadata.publisher_id);
        *sequence_hi = static_cast<std::uint32_t>(metadata.sequence >> 32);
        *sequence_lo = static_cast<std::uint32_t>(metadata.sequence);
      };
    if (!has_points) {
      write_metadata();
      *x = std::numeric_limits<float>::quiet_NaN();
      *y = std::numeric_limits<float>::quiet_NaN();
      *z = std::numeric_limits<float>::quiet_NaN();
      return result;
    }

    for (const auto & point : points) {
      write_metadata();
      *x = static_cast<float>(point.x);
      *y = static_cast<float>(point.y);
      *z = static_cast<float>(point.z);
      ++publisher_hi;
      ++publisher_lo;
      ++sequence_hi;
      ++sequence_lo;
      ++x;
      ++y;
      ++z;
    }
    return result;
  }

  bringup::ObstacleEvidenceFrameMetadata publishGroundVeto(
    const std_msgs::msg::Header & header,
    const std::vector<geometry_msgs::msg::Point> & points)
  {
    const auto source_stamp_ns = rclcpp::Time(header.stamp).nanoseconds();
    std::lock_guard<std::mutex> lock(ground_veto_publish_mutex_);
    ++ground_veto_sequence_;
    if (ground_veto_sequence_ == 0) {
      ground_veto_publisher_id_ = makePublisherInstanceId();
      ground_veto_sequence_ = 1;
    }
    const bringup::ObstacleEvidenceFrameMetadata metadata{
      ground_veto_publisher_id_, ground_veto_sequence_};
    ground_veto_publisher_->publish(makeGroundVetoCloud(header, points, metadata));
    if (source_stamp_ns > 0) {
      latest_ground_veto_stamp_ns_ = source_stamp_ns;
      latest_ground_veto_sequence_ = metadata.sequence;
      have_ground_veto_stamp_ = true;
    }
    return metadata;
  }

  static void tagGlobalClearEvent(
    PointCloud & message, const bringup::ObstacleEvidenceFrameMetadata & metadata)
  {
    message.header.frame_id = bringup::tagObstacleClearEventFrameId(
      message.header.frame_id, metadata);
  }

  void groundVetoFenceCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    std::lock_guard<std::mutex> lock(ground_veto_publish_mutex_);
    response->success = have_ground_veto_stamp_;
    response->message = have_ground_veto_stamp_ ? bringup::serializeObstacleEvidenceFence(
      {ground_veto_publisher_id_, latest_ground_veto_sequence_, latest_ground_veto_stamp_ns_}) :
      std::string();
  }

  void groundCallback(const grid_map_msgs::msg::GridMap::SharedPtr message)
  {
    if (message->header.frame_id.empty()) return;
    try {
      auto map = nanogrid::ros2::fromMsg(*message);
      if (!map.exists("elevation")) return;
      std::lock_guard<std::mutex> lock(mutex_);
      ground_map_ = std::move(map);
      ground_frame_ = message->header.frame_id;
      ground_stamp_ = rclcpp::Time(message->header.stamp);
      have_ground_ = true;
    } catch (const std::exception & error) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Invalid FastDEM grid map: %s", error.what());
    }
  }

  sensor_msgs::msg::PointCloud2 makeCloud(
    const std_msgs::msg::Header & header, const std::vector<geometry_msgs::msg::Point> & points) const
  {
    sensor_msgs::msg::PointCloud2 result;
    result.header = header;
    sensor_msgs::PointCloud2Modifier modifier(result);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(points.size());
    sensor_msgs::PointCloud2Iterator<float> x(result, "x");
    sensor_msgs::PointCloud2Iterator<float> y(result, "y");
    sensor_msgs::PointCloud2Iterator<float> z(result, "z");
    for (const auto & point : points) {
      *x = static_cast<float>(point.x);
      *y = static_cast<float>(point.y);
      *z = static_cast<float>(point.z);
      ++x;
      ++y;
      ++z;
    }
    return result;
  }

  bool lookupTrackingTransform(
    const std_msgs::msg::Header & header,
    geometry_msgs::msg::TransformStamped & transform)
  {
    if (header.frame_id == tracking_frame_) {
      transform.header = header;
      transform.child_frame_id = header.frame_id;
      transform.transform.rotation.w = 1.0;
      return true;
    }
    try {
      transform = tf_buffer_.lookupTransform(
        tracking_frame_, header.frame_id, header.stamp,
        rclcpp::Duration::from_seconds(0.12));
      return true;
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Cannot transform obstacle confirmation cloud to %s: %s",
        tracking_frame_.c_str(), error.what());
      return false;
    }
  }

  sensor_msgs::msg::PointCloud2 makeTemporallyConfirmedCloud(
    bringup::TemporalObstacleGrid & obstacle_grid,
    bool temporal_filter_enabled,
    bool current_hits_only,
    bool include_clear_events,
    double obstacle_max_range,
    const std_msgs::msg::Header & input_header,
    const std::vector<geometry_msgs::msg::Point> & blocking,
    const PointCloud::ConstSharedPtr & ray_cloud,
    PointCloud * ray_clear_events = nullptr,
    bool separate_clear_events = false,
    PointCloud * tracking_updates = nullptr)
  {
    if (ray_clear_events) *ray_clear_events = makeTrackingCloud(input_header, {});
    if (tracking_updates) {
      *tracking_updates = makeTrackingUpdateCloud(input_header, {}, {});
    }
    if (!temporal_filter_enabled) {
      return makeCloud(input_header, blocking);
    }

    geometry_msgs::msg::TransformStamped input_transform;
    if (!lookupTrackingTransform(input_header, input_transform)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Aging confirmed obstacles because the tracking transform is unavailable");
      const auto aged = obstacle_grid.ageWithoutObservation();
      if (current_hits_only || obstacle_max_range > 0.0) {
        auto empty = makeTrackingCloud(input_header, {});
        empty.header.frame_id.clear();
        if (tracking_updates) tracking_updates->header.frame_id.clear();
        return empty;
      }
      return makeTrackingCloud(input_header, aged);
    }
    const auto & translation = input_transform.transform.translation;
    bringup::ObstacleGridPoint sensor_origin{
      translation.x, translation.y, translation.z};

    std::vector<bringup::ObstacleGridPoint> hits;
    hits.reserve(blocking.size());
    tf2::Transform input_tf;
    tf2::fromMsg(input_transform.transform, input_tf);
    for (const auto & point : blocking) {
      const auto transformed = input_tf * tf2::Vector3(point.x, point.y, point.z);
      hits.push_back({transformed.x(), transformed.y(), transformed.z()});
    }

    std::vector<bringup::ObstacleGridPoint> ray_endpoints;
    if (ray_cloud) {
      geometry_msgs::msg::TransformStamped ray_transform;
      if (lookupTrackingTransform(ray_cloud->header, ray_transform)) {
        const auto & ray_translation = ray_transform.transform.translation;
        sensor_origin = {
          ray_translation.x, ray_translation.y, ray_translation.z};
        try {
          tf2::Transform ray_tf;
          tf2::fromMsg(ray_transform.transform, ray_tf);
          sensor_msgs::PointCloud2ConstIterator<float> x(*ray_cloud, "x");
          sensor_msgs::PointCloud2ConstIterator<float> y(*ray_cloud, "y");
          sensor_msgs::PointCloud2ConstIterator<float> z(*ray_cloud, "z");
          ray_endpoints.reserve(ray_cloud->width * ray_cloud->height);
          for (; x != x.end(); ++x, ++y, ++z) {
            const double range_squared =
              static_cast<double>(*x) * *x + static_cast<double>(*y) * *y +
              static_cast<double>(*z) * *z;
            if (range_squared < obstacle_raytrace_min_range_ * obstacle_raytrace_min_range_) {
              continue;
            }
            const auto transformed = ray_tf * tf2::Vector3(*x, *y, *z);
            ray_endpoints.push_back({transformed.x(), transformed.y(), transformed.z()});
          }
        } catch (const std::runtime_error & error) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Cannot read raytrace cloud; retaining confirmed obstacles: %s", error.what());
        }
      }
    } else {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "No synchronized raytrace cloud; retaining confirmed obstacles without miss updates");
    }

    auto confirmed = obstacle_grid.processFrame(
      hits, ray_endpoints, sensor_origin, current_hits_only);
    if (obstacle_max_range > 0.0) {
      const double max_range_squared = obstacle_max_range * obstacle_max_range;
      confirmed.erase(
        std::remove_if(
          confirmed.begin(), confirmed.end(),
          [&sensor_origin, max_range_squared](const auto & point) {
            const double dx = point.x - sensor_origin.x;
            const double dy = point.y - sensor_origin.y;
            const double dz = point.z - sensor_origin.z;
            return dx * dx + dy * dy + dz * dz > max_range_squared;
          }),
        confirmed.end());
    }
    if (include_clear_events) {
      std::vector<bringup::ObstacleGridPoint> ray_cleared_points;
      if (ray_clear_events) {
        ray_cleared_points = obstacle_grid.takeRayClearedPoints();
        *ray_clear_events = makeTrackingCloud(input_header, ray_cleared_points);
      }
      auto cleared_points = obstacle_grid.takeClearedPoints();
      if (tracking_updates) {
        *tracking_updates = makeTrackingUpdateCloud(input_header, confirmed, cleared_points);
      }
      if (separate_clear_events && ray_clear_events) {
        return makeTrackingCloud(input_header, confirmed);
      }
      return makeTrackingUpdateCloud(input_header, confirmed, cleared_points);
    }
    return makeTrackingCloud(input_header, confirmed);
  }

  PointCloud makeTrackingCloud(
    const std_msgs::msg::Header & input_header,
    const std::vector<bringup::ObstacleGridPoint> & confirmed) const
  {
    std::vector<geometry_msgs::msg::Point> output_points;
    output_points.reserve(confirmed.size());
    for (const auto & point : confirmed) {
      geometry_msgs::msg::Point output;
      output.x = point.x;
      output.y = point.y;
      output.z = point.z;
      output_points.push_back(output);
    }
    auto output_header = input_header;
    output_header.frame_id = tracking_frame_;
    return makeCloud(output_header, output_points);
  }

  PointCloud makeTrackingUpdateCloud(
    const std_msgs::msg::Header & input_header,
    const std::vector<bringup::ObstacleGridPoint> & confirmed,
    const std::vector<bringup::ObstacleGridPoint> & cleared) const
  {
    auto output_header = input_header;
    output_header.frame_id = tracking_frame_;
    PointCloud result;
    result.header = output_header;
    sensor_msgs::PointCloud2Modifier modifier(result);
    modifier.setPointCloud2Fields(
      4, "x", 1, sensor_msgs::msg::PointField::FLOAT32,
      "y", 1, sensor_msgs::msg::PointField::FLOAT32,
      "z", 1, sensor_msgs::msg::PointField::FLOAT32,
      "operation", 1, sensor_msgs::msg::PointField::UINT8);
    modifier.resize(confirmed.size() + cleared.size());
    sensor_msgs::PointCloud2Iterator<float> x(result, "x");
    sensor_msgs::PointCloud2Iterator<float> y(result, "y");
    sensor_msgs::PointCloud2Iterator<float> z(result, "z");
    sensor_msgs::PointCloud2Iterator<std::uint8_t> operation(result, "operation");
    const auto write_point = [&x, &y, &z, &operation](
        const bringup::ObstacleGridPoint & point, std::uint8_t op) {
        *x = static_cast<float>(point.x);
        *y = static_cast<float>(point.y);
        *z = static_cast<float>(point.z);
        *operation = op;
        ++x;
        ++y;
        ++z;
        ++operation;
      };
    for (const auto & point : confirmed) {
      write_point(point, 0U);
    }
    for (const auto & point : cleared) {
      write_point(point, 1U);
    }
    return result;
  }

  void publishWithoutGround(
    const std_msgs::msg::Header & header,
    const std::vector<geometry_msgs::msg::Point> & input_points,
    const PointCloud::ConstSharedPtr & ray_cloud,
    const char * reason)
  {
    std::vector<geometry_msgs::msg::Point> blocking;
    std::vector<geometry_msgs::msg::Point> overhead;
    blocking.reserve(input_points.size());
    overhead.reserve(input_points.size());
    for (const auto & point : input_points) {
      if (bringup::isNoElevationBlocking(
          point.z, sensor_height_, min_obstacle_height_, no_elevation_z_max_))
      {
        blocking.push_back(point);
      } else {
        overhead.push_back(point);
      }
    }
    PointCloud local_clearing;
    blocking_publisher_->publish(makeTemporallyConfirmedCloud(
        *temporal_obstacle_grid_, temporal_obstacle_filter_enabled_, true, true,
        0.0, header, blocking, ray_cloud, &local_clearing));
    const auto evidence_metadata = publishGroundVeto(header, blocking);
    // Keep global confirmations until fresh valid evidence can clear them.
    global_blocking_publisher_->publish(makeCloud(header, {}));
    if (global_costmap_updates_publisher_) {
      PointCloud global_costmap_updates;
      (void)makeTemporallyConfirmedCloud(
        *global_costmap_temporal_obstacle_grid_, true, false, true,
        global_obstacle_max_range_, header, blocking, ray_cloud,
        nullptr, false, &global_costmap_updates);
      global_costmap_updates_publisher_->publish(global_costmap_updates);
    }
    if (local_clearing.width > 0) {
      tagGlobalClearEvent(local_clearing, evidence_metadata);
      global_clearing_publisher_->publish(local_clearing);
    }
    if (!overhead.empty()) overhead_publisher_->publish(makeCloud(header, overhead));
    const auto & obstacle_diagnostics = temporal_obstacle_grid_->diagnostics();
    const auto global_costmap_confirmed = global_costmap_temporal_obstacle_grid_ ?
      global_costmap_temporal_obstacle_grid_->diagnostics().confirmed_cells : 0U;
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 3000,
      "%s; keeping %zu points in the ground-relative height band for local blocking, "
      "classifying %zu as overhead; "
      "withholding strict global_blocking evidence (upper height limit %.2f m above estimated ground); "
      "local_temporal_confirmed=%zu rolling_global_temporal_confirmed=%zu "
      "new_confirmed_total=%llu ray_cleared_total=%llu stale_cleared_total=%llu "
      "oldest_confirmed_age=%llu frames",
      reason, blocking.size(), overhead.size(), no_elevation_z_max_,
      obstacle_diagnostics.confirmed_cells, global_costmap_confirmed,
      static_cast<unsigned long long>(obstacle_diagnostics.newly_confirmed_cells_total),
      static_cast<unsigned long long>(obstacle_diagnostics.ray_cleared_cells_total),
      static_cast<unsigned long long>(obstacle_diagnostics.stale_cleared_cells_total),
      static_cast<unsigned long long>(obstacle_diagnostics.oldest_confirmed_age_frames));
  }

  void cloudCallback(const PointCloud::ConstSharedPtr message)
  {
    processCloud(message, nullptr);
  }

  void synchronizedCloudCallback(
    const PointCloud::ConstSharedPtr & message,
    const PointCloud::ConstSharedPtr & ray_cloud)
  {
    processCloud(message, ray_cloud);
  }

  void processCloud(
    const PointCloud::ConstSharedPtr & message,
    const PointCloud::ConstSharedPtr & ray_cloud)
  {
    std::vector<geometry_msgs::msg::Point> blocking;
    std::vector<geometry_msgs::msg::Point> global_blocking;
    std::vector<geometry_msgs::msg::Point> ground_veto;
    std::vector<geometry_msgs::msg::Point> overhead;
    std::vector<geometry_msgs::msg::Point> input_points;
    sensor_msgs::PointCloud2ConstIterator<float> x(*message, "x");
    sensor_msgs::PointCloud2ConstIterator<float> y(*message, "y");
    sensor_msgs::PointCloud2ConstIterator<float> z(*message, "z");
    for (; x != x.end(); ++x, ++y, ++z) {
      geometry_msgs::msg::Point point;
      point.x = *x;
      point.y = *y;
      point.z = *z;
      input_points.push_back(point);
    }

    nanogrid::GridMap map;
    std::string ground_frame;
    rclcpp::Time ground_stamp;
    bool have_ground = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (have_ground_) {
        map = ground_map_;
        ground_frame = ground_frame_;
        ground_stamp = ground_stamp_;
        have_ground = true;
      }
    }

    if (!have_ground || ground_frame.empty() ||
      (get_clock()->now() - ground_stamp).seconds() > map_timeout_)
    {
      publishWithoutGround(
        message->header, input_points, ray_cloud,
        "FastDEM ground unavailable/stale; applying sensor-height ground-relative fallback");
      return;
    }
    if (input_points.empty()) {
      PointCloud local_clearing;
      blocking_publisher_->publish(makeTemporallyConfirmedCloud(
        *temporal_obstacle_grid_, temporal_obstacle_filter_enabled_, true, true,
        0.0, message->header, blocking, ray_cloud, &local_clearing));
      PointCloud global_clearing;
      global_blocking_publisher_->publish(makeTemporallyConfirmedCloud(
        *global_temporal_obstacle_grid_, true, true, true,
        global_obstacle_max_range_,
        message->header, global_blocking, ray_cloud, &global_clearing, true));
      if (global_costmap_updates_publisher_) {
        PointCloud global_costmap_updates;
        (void)makeTemporallyConfirmedCloud(
          *global_costmap_temporal_obstacle_grid_, true, false, true,
          global_obstacle_max_range_, message->header, global_blocking, ray_cloud,
          nullptr, false, &global_costmap_updates);
        global_costmap_updates_publisher_->publish(global_costmap_updates);
      }
      const auto evidence_metadata = publishGroundVeto(message->header, ground_veto);
      if (local_clearing.width > 0) {
        tagGlobalClearEvent(local_clearing, evidence_metadata);
        global_clearing_publisher_->publish(local_clearing);
      }
      if (global_clearing.width > 0) {
        tagGlobalClearEvent(global_clearing, evidence_metadata);
        global_clearing_publisher_->publish(global_clearing);
      }
      return;
    }

    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer_.lookupTransform(
        ground_frame, message->header.frame_id, message->header.stamp,
        rclcpp::Duration::from_seconds(0.12));
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Cannot transform nonground cloud; applying sensor-height ground-relative fallback: %s",
        error.what());
      publishWithoutGround(
        message->header, input_points, ray_cloud, "Nonground transform unavailable");
      return;
    }

    std::size_t no_elev_dropped = 0;
    std::size_t implausible_ground = 0;
    // The FastDEM elevation layer occasionally carries wall tops as "ground" (observed up
    // to 1.995 m).  Such a reference makes a ceiling return look like a near-ground point
    // and promotes it to blocking, thickening the walls of narrow passages.  Ground that
    // far above the robot base cannot be reached within the traversable slope, so fall
    // back to the sensor-relative test instead of trusting it.
    const double base_z = transform.transform.translation.z;
    const double rise_per_meter = std::tan(
      ground_reference_max_slope_ * M_PI / 180.0);
    for (const auto & input : input_points) {
      geometry_msgs::msg::PointStamped source;
      source.header = message->header;
      source.point = input;
      geometry_msgs::msg::PointStamped transformed;
      tf2::doTransform(source, transformed, transform);
      const auto index = map.index(nanogrid::Position(transformed.point.x, transformed.point.y));
      bool has_ground_elevation = index.has_value() &&
        std::isfinite(map.at("elevation", *index));
      if (has_ground_elevation) {
        const double max_rise = std::min(
          rise_per_meter * std::hypot(input.x, input.y) + robot_height_ + clearance_margin_,
          robot_height_ + clearance_margin_ + 0.30);
        if (map.at("elevation", *index) - base_z > max_rise) {
          has_ground_elevation = false;
          ++implausible_ground;
        }
      }
      bool is_blocking;
      if (!has_ground_elevation) {
        // The input cloud is in base_link; estimate height above ground from sensor height.
        is_blocking = bringup::isNoElevationBlocking(
          input.z, sensor_height_, min_obstacle_height_, no_elevation_z_max_);
        if (!is_blocking) ++no_elev_dropped;
      } else {
        const double height_above_ground =
          transformed.point.z - map.at("elevation", *index);
        is_blocking = height_above_ground > min_obstacle_height_ &&
          height_above_ground < robot_height_ + clearance_margin_;
      }
      if (is_blocking) {
        blocking.push_back(input);
        ground_veto.push_back(input);
        const double range_x = transformed.point.x - transform.transform.translation.x;
        const double range_y = transformed.point.y - transform.transform.translation.y;
        const double range_z = transformed.point.z - transform.transform.translation.z;
        if (has_ground_elevation &&
          range_x * range_x + range_y * range_y + range_z * range_z <=
          global_obstacle_max_range_ * global_obstacle_max_range_)
        {
          global_blocking.push_back(input);
        }
      } else {
        overhead.push_back(input);
      }
    }
    PointCloud local_clearing;
    blocking_publisher_->publish(makeTemporallyConfirmedCloud(
      *temporal_obstacle_grid_, temporal_obstacle_filter_enabled_, true, true,
      0.0, message->header, blocking, ray_cloud, &local_clearing));
    PointCloud global_clearing;
    auto confirmed_global = makeTemporallyConfirmedCloud(
      *global_temporal_obstacle_grid_, true, true, true,
      global_obstacle_max_range_,
      message->header, global_blocking, ray_cloud, &global_clearing, true);
    const auto global_temporal_confirmed_count =
      static_cast<std::size_t>(confirmed_global.width) * confirmed_global.height;
    global_blocking_publisher_->publish(confirmed_global);
    if (global_costmap_updates_publisher_) {
      PointCloud global_costmap_updates;
      (void)makeTemporallyConfirmedCloud(
        *global_costmap_temporal_obstacle_grid_, true, false, true,
        global_obstacle_max_range_, message->header, global_blocking, ray_cloud,
        nullptr, false, &global_costmap_updates);
      global_costmap_updates_publisher_->publish(global_costmap_updates);
    }
    const auto evidence_metadata = publishGroundVeto(message->header, ground_veto);
    if (local_clearing.width > 0) {
      tagGlobalClearEvent(local_clearing, evidence_metadata);
      global_clearing_publisher_->publish(local_clearing);
    }
    if (global_clearing.width > 0) {
      tagGlobalClearEvent(global_clearing, evidence_metadata);
      global_clearing_publisher_->publish(global_clearing);
    }
    if (!overhead.empty()) overhead_publisher_->publish(makeCloud(message->header, overhead));
    const auto & obstacle_diagnostics = temporal_obstacle_grid_->diagnostics();
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Obstacle filter input=%zu blocking=%zu global_blocking_candidates=%zu global_temporal_current_confirmed=%zu overhead=%zu no_elev_dropped=%zu implausible_ground=%zu ground_age=%.3f s; temporal confirmed=%zu new_confirmed_total=%llu ray_cleared_total=%llu stale_cleared_total=%llu oldest_confirmed_age=%llu frames",
      input_points.size(), blocking.size(), global_blocking.size(), global_temporal_confirmed_count,
      overhead.size(), no_elev_dropped,
      implausible_ground,
      (get_clock()->now() - ground_stamp).seconds(),
      obstacle_diagnostics.confirmed_cells,
      static_cast<unsigned long long>(obstacle_diagnostics.newly_confirmed_cells_total),
      static_cast<unsigned long long>(obstacle_diagnostics.ray_cleared_cells_total),
      static_cast<unsigned long long>(obstacle_diagnostics.stale_cleared_cells_total),
      static_cast<unsigned long long>(obstacle_diagnostics.oldest_confirmed_age_frames));
  }

  std::string input_topic_;
  std::string ground_topic_;
  std::string blocking_topic_;
  std::string global_blocking_topic_;
  std::string global_costmap_updates_topic_;
  std::string global_clearing_topic_;
  std::string ground_veto_topic_;
  std::string ground_veto_fence_service_name_;
  std::string overhead_topic_;
  double robot_height_{0.40};
  double clearance_margin_{0.10};
  double min_obstacle_height_{0.05};
  double sensor_height_{0.25};
  double no_elevation_z_max_{0.50};
  double ground_reference_max_slope_{10.0};
  double map_timeout_{1.2};
  bool temporal_obstacle_filter_enabled_{false};
  std::string tracking_frame_{"odom"};
  std::string raytrace_topic_{"/cloud_registered_body"};
  double obstacle_raytrace_min_range_{0.5};
  double obstacle_raytrace_max_range_{5.0};
  double global_obstacle_max_range_{10.0};
  double global_obstacle_raytrace_max_range_{10.0};
  std::unique_ptr<bringup::TemporalObstacleGrid> temporal_obstacle_grid_;
  std::unique_ptr<bringup::TemporalObstacleGrid> global_temporal_obstacle_grid_;
  std::unique_ptr<bringup::TemporalObstacleGrid> global_costmap_temporal_obstacle_grid_;
  std::mutex mutex_;
  std::mutex ground_veto_publish_mutex_;
  std::uint64_t ground_veto_publisher_id_{0};
  std::uint64_t ground_veto_sequence_{0};
  std::int64_t latest_ground_veto_stamp_ns_{0};
  std::uint64_t latest_ground_veto_sequence_{0};
  bool have_ground_veto_stamp_{false};
  bool have_ground_{false};
  std::string ground_frame_;
  rclcpp::Time ground_stamp_{0, 0, RCL_ROS_TIME};
  nanogrid::GridMap ground_map_;
  rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr ground_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscription_;
  message_filters::Subscriber<PointCloud> synchronized_input_subscriber_;
  message_filters::Subscriber<PointCloud> ray_cloud_subscriber_;
  std::shared_ptr<CloudSynchronizer> cloud_synchronizer_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr blocking_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr global_blocking_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
    global_costmap_updates_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr global_clearing_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ground_veto_publisher_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr ground_veto_fence_service_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr overhead_publisher_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NongroundHeightFilter>());
  rclcpp::shutdown();
  return 0;
}
