#include "bringup/static_terrain_map_publisher.hpp"
#include "bringup/inflation_surface.hpp"
#include "bringup/static_pcd_clearance.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <deque>
#include <functional>
#include <fstream>
#include <limits>
#include <map>
#include <optional>
#include <set>
#include <stdexcept>
#include <utility>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <fastdem/bridge/ros2.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <map_msgs/msg/occupancy_grid_update.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <std_msgs/msg/u_int64_multi_array.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>

namespace
{

struct ColoredDisplayPoint
{
  float x;
  float y;
  float z;
  float height_above_ground;
};

using GridCellKey = bringup::StaticPcdCell;
using GridCellKeyHash = bringup::StaticPcdCellHash;
constexpr std::size_t kMaxStaticClearanceRaysPerScan = 5000;

struct StaticStructurePoint
{
  ColoredDisplayPoint point;
  GridCellKey cell;
  bringup::StaticPcdHeightClearanceEvidence clearance;
};

struct StaticStructurePointRef
{
  std::size_t tile_index;
  std::size_t point_index;
};

struct StaticStructureTile
{
  std::int32_t marker_id;
  std::vector<StaticStructurePoint> points;
};

std::uint64_t groundVoxelKey(float x, float y, double voxel_size)
{
  const auto ix = static_cast<std::uint32_t>(
    static_cast<std::int32_t>(std::floor(x / voxel_size)));
  const auto iy = static_cast<std::uint32_t>(
    static_cast<std::int32_t>(std::floor(y / voxel_size)));
  return (static_cast<std::uint64_t>(ix) << 32U) | iy;
}

bool sameOccupancyGrid(
  const nav_msgs::msg::OccupancyGrid & lhs,
  const nav_msgs::msg::OccupancyGrid & rhs)
{
  const auto & a = lhs.info;
  const auto & b = rhs.info;
  return lhs.header.frame_id == rhs.header.frame_id &&
         a.resolution == b.resolution && a.width == b.width && a.height == b.height &&
         a.origin.position.x == b.origin.position.x &&
         a.origin.position.y == b.origin.position.y &&
         a.origin.position.z == b.origin.position.z &&
         a.origin.orientation.x == b.origin.orientation.x &&
         a.origin.orientation.y == b.origin.orientation.y &&
         a.origin.orientation.z == b.origin.orientation.z &&
         a.origin.orientation.w == b.origin.orientation.w &&
         lhs.data == rhs.data;
}

bool quantizeGridCoordinate(double coordinate, double origin, double resolution, std::int64_t & cell)
{
  if (!std::isfinite(coordinate) || !std::isfinite(origin) ||
    !std::isfinite(resolution) || resolution <= 0.0)
  {
    return false;
  }
  const double grid_coordinate = std::floor((coordinate - origin) / resolution);
  if (!std::isfinite(grid_coordinate) ||
    grid_coordinate < static_cast<double>(std::numeric_limits<std::int64_t>::min()) ||
    grid_coordinate >= static_cast<double>(std::numeric_limits<std::int64_t>::max()))
  {
    return false;
  }
  cell = static_cast<std::int64_t>(grid_coordinate);
  return true;
}

std::int64_t floorDivide(std::int64_t value, std::int64_t divisor)
{
  const auto quotient = value / divisor;
  const auto remainder = value % divisor;
  return remainder < 0 ? quotient - 1 : quotient;
}

sensor_msgs::msg::PointCloud2 makeColoredCloud(
  const std::vector<ColoredDisplayPoint> & points,
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
    const auto color = bringup::staticDisplayColor(point.height_above_ground);
    *r = color[0];
    *g = color[1];
    *b = color[2];
    ++x; ++y; ++z; ++r; ++g; ++b;
  }
  return message;
}

class StaticTerrainMapPublisher : public rclcpp::Node
{
  using MapGeometryEventKey = std::pair<std::uint64_t, std::uint64_t>;

  struct PendingMapUpdate
  {
    map_msgs::msg::OccupancyGridUpdate::ConstSharedPtr message;
    std::optional<bringup::MapGeometryEvent> geometry;
  };

  static constexpr std::size_t kMaximumPendingMapMessages = 32;

public:
  StaticTerrainMapPublisher()
  : Node("static_terrain_map_publisher"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
  {
    const auto terrain_map_file = declare_parameter<std::string>("terrain_map_file", "");
    const auto topic = declare_parameter<std::string>("topic", "/terrain_map/static");
    frame_id_ = declare_parameter<std::string>("frame_id", "map");
    global_inflation_radius_ = declare_parameter<double>("global_inflation_radius", 0.38);
    local_inflation_radius_ = declare_parameter<double>("local_inflation_radius", 0.25);
    global_inflation_alpha_ = declare_parameter<double>("global_inflation_alpha", 0.25);
    if (!std::isfinite(global_inflation_alpha_) ||
      global_inflation_alpha_ < 0.0 || global_inflation_alpha_ > 1.0)
    {
      throw std::invalid_argument("global_inflation_alpha must be in [0, 1]");
    }
    display_pcd_file_ = declare_parameter<std::string>("display_pcd_file", "");
    display_ground_cache_file_ = declare_parameter<std::string>(
      "display_ground_cache_file", "");
    display_structure_cache_file_ = declare_parameter<std::string>(
      "display_structure_cache_file", "");
    display_topic_ = declare_parameter<std::string>("display_topic", "/terrain_map/static_cloud");
    ground_display_voxel_size_ = declare_parameter<double>(
      "ground_display_voxel_size", 0.12);
    if (!std::isfinite(ground_display_voxel_size_) || ground_display_voxel_size_ <= 0.0) {
      throw std::invalid_argument("ground_display_voxel_size must be positive and finite");
    }
    max_display_height_ = declare_parameter<double>("display_max_height_above_ground", 1.2);
    if (!std::isfinite(max_display_height_) || max_display_height_ <= 0.0) {
      throw std::invalid_argument("display_max_height_above_ground must be positive and finite");
    }
    const auto local_cloud_topic = declare_parameter<std::string>(
      "local_cloud_topic", "/cloud_registered_body");
    const auto clearance_cloud_topic = declare_parameter<std::string>(
      "clearance_cloud_topic", "/cloud_registered_body_clearance");
    const auto local_ground_display_topic = declare_parameter<std::string>(
      "local_ground_display_topic", "/navigation_display/local_ground_cloud");
    const auto local_structure_display_topic = declare_parameter<std::string>(
      "local_structure_display_topic", "/navigation_display/local_structure_cloud");
    const bool merge_live_terrain = declare_parameter<bool>("merge_live_terrain", true);
    local_ground_voxel_size_ = declare_parameter<double>("local_ground_voxel_size", 0.12);
    if (!std::isfinite(local_ground_voxel_size_) || local_ground_voxel_size_ <= 0.0) {
      throw std::invalid_argument("local_ground_voxel_size must be positive and finite");
    }
    local_cloud_radius_ = declare_parameter<double>("local_cloud_radius", 5.0);
    local_cloud_fallback_ground_height_ = declare_parameter<double>(
      "local_cloud_fallback_ground_height", 0.25);
    local_cloud_timeout_ = declare_parameter<double>("local_cloud_timeout", 1.0);
    const auto local_max_height = declare_parameter<double>(
      "local_cloud_max_height_above_ground", max_display_height_);
    if (!std::isfinite(local_cloud_radius_) || local_cloud_radius_ <= 0.0) {
      throw std::invalid_argument("local_cloud_radius must be positive and finite");
    }
    if (!std::isfinite(local_max_height) || local_max_height <= 0.0) {
      throw std::invalid_argument(
        "local_cloud_max_height_above_ground must be positive and finite");
    }
    if (!std::isfinite(local_cloud_fallback_ground_height_) ||
      local_cloud_fallback_ground_height_ <= 0.0)
    {
      throw std::invalid_argument(
        "local_cloud_fallback_ground_height must be positive and finite");
    }
    if (!std::isfinite(local_cloud_timeout_) || local_cloud_timeout_ <= 0.0) {
      throw std::invalid_argument("local_cloud_timeout must be positive and finite");
    }
    clearance_ray_observations_ = declare_parameter<int>("clearance_ray_observations", 3);
    clearance_ray_min_range_ = declare_parameter<double>("clearance_ray_min_range", 0.5);
    clearance_ray_max_range_ = declare_parameter<double>("clearance_ray_max_range", 20.0);
    clearance_ray_height_resolution_ = declare_parameter<double>(
      "clearance_ray_height_resolution", 0.10);
    diagnostic_cell_enabled_ = declare_parameter<bool>("diagnostic_cell_enabled", false);
    diagnostic_cell_x_ = declare_parameter<double>("diagnostic_cell_x", 0.0);
    diagnostic_cell_y_ = declare_parameter<double>("diagnostic_cell_y", 0.0);
    if (clearance_ray_observations_ < 1 ||
      !std::isfinite(clearance_ray_min_range_) || clearance_ray_min_range_ < 0.0 ||
      !std::isfinite(clearance_ray_max_range_) ||
      clearance_ray_max_range_ <= clearance_ray_min_range_ ||
      !std::isfinite(clearance_ray_height_resolution_) ||
      clearance_ray_height_resolution_ <= 0.0 ||
      !std::isfinite(diagnostic_cell_x_) || !std::isfinite(diagnostic_cell_y_))
    {
      throw std::invalid_argument("invalid static PCD ray-clearance parameters");
    }
    local_max_height_above_ground_ = static_cast<float>(local_max_height);

    publisher_ = create_publisher<grid_map_msgs::msg::GridMap>(
      topic, rclcpp::QoS(1).reliable().transient_local());

    std::string error;
    if (!bringup::loadStaticTerrainMap(terrain_map_file, frame_id_, map_, &error)) {
      throw std::runtime_error(error);
    }

    // Reliable offer stays compatible with both reliable and best-effort RViz
    // subscriptions; a best-effort offer is silently dropped for the former.
    local_ground_cloud_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      local_ground_display_topic, rclcpp::QoS(1).reliable());
    local_structure_cloud_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      local_structure_display_topic, rclcpp::QoS(1).reliable());
    local_cloud_callback_group_ = create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions local_cloud_subscription_options;
    local_cloud_subscription_options.callback_group = local_cloud_callback_group_;
    local_cloud_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      local_cloud_topic, rclcpp::SensorDataQoS(),
      std::bind(&StaticTerrainMapPublisher::localCloudCallback, this, std::placeholders::_1),
      local_cloud_subscription_options);
    clearance_cloud_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      clearance_cloud_topic, rclcpp::SensorDataQoS(),
      std::bind(
        &StaticTerrainMapPublisher::clearanceCloudCallback, this, std::placeholders::_1));

    std::vector<float> ground_samples;
    const auto & original_elevation = map_.get(fastdem::layer::elevation);
    const auto & cell_class = map_.get("class");
    for (Eigen::Index row = 0; row < original_elevation.rows(); ++row) {
      for (Eigen::Index col = 0; col < original_elevation.cols(); ++col) {
        if (cell_class(row, col) == 1.0F && std::isfinite(original_elevation(row, col))) {
          ground_samples.push_back(original_elevation(row, col));
        }
      }
    }
    if (!ground_samples.empty()) {
      const auto middle = ground_samples.begin() + ground_samples.size() / 2;
      std::nth_element(ground_samples.begin(), middle, ground_samples.end());
      fallback_ground_ = *middle;
    }
    const auto removed_spikes = bringup::prepareStaticDisplayMap(map_);
    static_ground_map_ = map_.snapshot({fastdem::layer::elevation});
    static_pcd_base_origin_x_ = map_.getPosition().x() - map_.getLength().x() / 2.0;
    static_pcd_base_origin_y_ = map_.getPosition().y() - map_.getLength().y() / 2.0;
    static_pcd_map_resolution_ = map_.getResolution();
    if (diagnostic_cell_enabled_) {
      GridCellKey cell{};
      if (quantizeGridCoordinate(
          diagnostic_cell_x_, static_pcd_base_origin_x_, static_pcd_map_resolution_, cell.x) &&
        quantizeGridCoordinate(
          diagnostic_cell_y_, static_pcd_base_origin_y_, static_pcd_map_resolution_, cell.y))
      {
        diagnostic_cell_ = cell;
        RCLCPP_INFO(
          get_logger(), "Static PCD cell diagnostic enabled: world=(%.3f,%.3f) grid=(%lld,%lld)",
          diagnostic_cell_x_, diagnostic_cell_y_,
          static_cast<long long>(cell.x), static_cast<long long>(cell.y));
      }
    }
    static_pcd_clearance_tracker_.emplace(
      frame_id_, static_pcd_base_origin_x_, static_pcd_base_origin_y_,
      static_pcd_map_resolution_);
    RCLCPP_INFO(
      get_logger(), "Display terrain relief ready; suppressed %zu isolated height spikes",
      removed_spikes);
    map_.setTimestamp(static_cast<uint64_t>(get_clock()->now().nanoseconds()));
    const auto message = fastdem::ros2::toGridMap(map_, "display_elevation");
    publisher_->publish(message);
    last_terrain_map_publish_ = std::chrono::steady_clock::now();
    if (!display_pcd_file_.empty() || !display_ground_cache_file_.empty() ||
      !display_structure_cache_file_.empty())
    {
      display_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        display_topic_, rclcpp::QoS(1).reliable().transient_local());
      static_structure_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "/navigation_display/static_pcd_structures",
        rclcpp::QoS(8).reliable().transient_local());
    }
    RCLCPP_INFO(
      get_logger(), "Published static terrain map '%s' on %s: %dx%d cells at %.3f m",
      terrain_map_file.c_str(), topic.c_str(), map_.getSize()(0), map_.getSize()(1),
      map_.getResolution());

    local_cloud_timeout_timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&StaticTerrainMapPublisher::expireLocalCloud, this),
      local_cloud_callback_group_);

    global_inflation_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/navigation_display/global_inflation", rclcpp::QoS(1).reliable().transient_local());
    local_inflation_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/navigation_display/local_inflation", rclcpp::QoS(1).reliable().transient_local());
    effective_obstacles_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/navigation_display/global_lethal_obstacles",
      rclcpp::QoS(1).reliable().transient_local());
    const auto costmap_qos = rclcpp::QoS(1).reliable();
    global_costmap_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/global_costmap/costmap", costmap_qos,
      [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr costmap) {
        const bool changed = !global_grid_ || !sameOccupancyGrid(*global_grid_, *costmap);
        global_grid_ = *costmap;
        if (changed || !global_surface_drawn_) global_surface_dirty_ = true;
        if (changed || !effective_obstacles_drawn_) effective_obstacles_dirty_ = true;
      });
    local_costmap_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/local_costmap/costmap", costmap_qos,
      [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr costmap) {
        const bool changed = !local_grid_ || !sameOccupancyGrid(*local_grid_, *costmap);
        local_grid_ = *costmap;
        if (changed || !local_surface_drawn_) local_surface_dirty_ = true;
        if (changed) logLocalCostmapCounts(*local_grid_);
      });
    inflation_retry_timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&StaticTerrainMapPublisher::retryInflationSurfaces, this));
    const auto update_qos = rclcpp::QoS(10).reliable();
    global_updates_ = create_subscription<map_msgs::msg::OccupancyGridUpdate>(
      "/global_costmap/costmap_updates", update_qos,
      [this](map_msgs::msg::OccupancyGridUpdate::ConstSharedPtr update) {
        applyUpdate(*update, false);
      });
    local_updates_ = create_subscription<map_msgs::msg::OccupancyGridUpdate>(
      "/local_costmap/costmap_updates", update_qos,
      [this](map_msgs::msg::OccupancyGridUpdate::ConstSharedPtr update) {
        applyUpdate(*update, true);
      });
    map_subscription_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", rclcpp::QoS(1).reliable().transient_local(),
      std::bind(&StaticTerrainMapPublisher::mapCallback, this, std::placeholders::_1));
    map_geometry_subscription_ = create_subscription<std_msgs::msg::UInt64MultiArray>(
      "/map_geometry_events", rclcpp::QoS(64).reliable().transient_local(),
      std::bind(
        &StaticTerrainMapPublisher::mapGeometryEventCallback, this, std::placeholders::_1));
    map_updates_subscription_ = create_subscription<map_msgs::msg::OccupancyGridUpdate>(
      "/map_updates", rclcpp::QoS(10).reliable(),
      std::bind(&StaticTerrainMapPublisher::mapUpdateCallback, this, std::placeholders::_1));
    occupancy_source_subscription_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/navigation/occupancy_source", rclcpp::QoS(1).reliable().transient_local(),
      [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr source) {
        latest_occupancy_source_ = *source;
        applyStartupPriorClearanceEvidence();
      });
    if (merge_live_terrain) {
      online_terrain_subscription_ = create_subscription<grid_map_msgs::msg::GridMap>(
        "/fastdem/postprocess/gridmap", rclcpp::SensorDataQoS(),
        std::bind(&StaticTerrainMapPublisher::onlineTerrainCallback, this, std::placeholders::_1));
    }
    if (display_publisher_) {
      display_load_timer_ = create_wall_timer(
        std::chrono::milliseconds(10), [this]() {
          display_load_timer_->cancel();
          RCLCPP_INFO(
            get_logger(), "Loading static PCD display cache while local cloud callbacks remain active");
          loadStaticPcdDisplay();
        });
    }
  }

private:
  void loadStaticPcdDisplay()
  {
    const bool have_classified_cache = !display_ground_cache_file_.empty() ||
      !display_structure_cache_file_.empty();
    if (have_classified_cache &&
      (display_ground_cache_file_.empty() || display_structure_cache_file_.empty()))
    {
      throw std::runtime_error(
              "display_ground_cache_file and display_structure_cache_file must be supplied together");
    }

    std::vector<ColoredDisplayPoint> ground_points;
    std::vector<ColoredDisplayPoint> structure_points;
    std::unordered_set<std::uint64_t> ground_voxels;
    std::size_t height_rejected = 0;
    std::size_t input_count = 0;
    bool structure_index_loaded = false;
    if (have_classified_cache) {
      pcl::PointCloud<pcl::PointXYZI> cached_ground;
      if (pcl::io::loadPCDFile<pcl::PointXYZI>(display_ground_cache_file_, cached_ground) != 0) {
        throw std::runtime_error(
                "Cannot load classified ground cache: " + display_ground_cache_file_);
      }
      height_rejected += loadStaticStructureIndex(
        display_structure_cache_file_, static_cast<float>(max_display_height_));
      structure_index_loaded = true;
      input_count = cached_ground.size() + staticStructurePointCount() + height_rejected;
      ground_points.reserve(cached_ground.size() / 2);
      ground_voxels.reserve(cached_ground.size() / 2 + 1);
      for (const auto & point : cached_ground.points) {
        if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
          !std::isfinite(point.z) || !std::isfinite(point.intensity) ||
          point.intensity > max_display_height_)
        {
          ++height_rejected;
          continue;
        }
        if (ground_voxels.insert(groundVoxelKey(
            point.x, point.y, ground_display_voxel_size_)).second)
        {
          ground_points.push_back({point.x, point.y, point.z, point.intensity});
        }
      }
    } else if (!display_pcd_file_.empty()) {
      pcl::PointCloud<pcl::PointXYZ> cloud;
      if (pcl::io::loadPCDFile<pcl::PointXYZ>(display_pcd_file_, cloud) != 0) {
        throw std::runtime_error("Cannot load display PCD: " + display_pcd_file_);
      }
      input_count = cloud.size();
      ground_points.reserve(cloud.size() / 5);
      structure_points.reserve(cloud.size());
      ground_voxels.reserve(cloud.size() / 5);
      for (const auto & point : cloud.points) {
        const auto height = bringup::staticDisplayHeightAboveGround(
          map_, point.x, point.y, point.z, fallback_ground_);
        if (!height || *height > max_display_height_) {
          ++height_rejected;
          continue;
        }
        const ColoredDisplayPoint display_point{point.x, point.y, point.z, *height};
        if (std::abs(*height) <= 0.10F) {
          if (ground_voxels.insert(groundVoxelKey(
              point.x, point.y, ground_display_voxel_size_)).second)
          {
            ground_points.push_back(display_point);
          }
        } else {
          structure_points.push_back(display_point);
        }
      }
    } else {
      return;
    }

    const auto stamp = get_clock()->now();
    const auto structure_count = structure_index_loaded
      ? staticStructurePointCount() : structure_points.size();
    display_publisher_->publish(makeColoredCloud(ground_points, frame_id_, stamp));
    pending_structure_points_ = std::move(structure_points);
    if (!structure_index_loaded) buildStaticStructureIndex();
    RCLCPP_INFO(
      get_logger(),
      "Published derived PCD display '%s': input=%zu ground=%zu/%.2fm structure=%zu rejected=%zu max_height=%.2fm cache=%s",
      have_classified_cache ? display_ground_cache_file_.c_str() : display_pcd_file_.c_str(),
      input_count, ground_points.size(), ground_display_voxel_size_,
      structure_count, height_rejected, max_display_height_,
      have_classified_cache ? "preclassified+preindexed" : "generated_at_startup");
  }

  void localCloudCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr message)
  {
    const bool has_local_display_subscriber = (local_ground_cloud_publisher_ &&
      local_ground_cloud_publisher_->get_subscription_count() > 0) ||
      (local_structure_cloud_publisher_ &&
      local_structure_cloud_publisher_->get_subscription_count() > 0);
    if (!has_local_display_subscriber) return;
    if (message->header.frame_id.empty()) return;

    const auto stamp = rclcpp::Time(message->header.stamp);
    const auto ros_now = get_clock()->now();
    const auto source_age = (ros_now - stamp).seconds();
    if (stamp.nanoseconds() == 0 || !std::isfinite(source_age) ||
      source_age > local_cloud_timeout_ || source_age < 0.0)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Dropping stale or future local cloud (age %.3f s)", source_age);
      return;
    }

    tf2::Transform map_from_cloud;
    map_from_cloud.setIdentity();
    if (message->header.frame_id != frame_id_) {
      try {
        const auto stamped = tf_buffer_.lookupTransform(
          frame_id_, message->header.frame_id, stamp,
          rclcpp::Duration::from_seconds(0.05));
        tf2::fromMsg(stamped.transform, map_from_cloud);
      } catch (const tf2::TransformException & error) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 3000, "Cannot transform local cloud to '%s': %s",
          frame_id_.c_str(), error.what());
        return;
      }
    }

    const auto robot_position = map_from_cloud.getOrigin();
    const double fallback_ground = robot_position.z() - local_cloud_fallback_ground_height_;
    const double radius_squared = local_cloud_radius_ * local_cloud_radius_;
    const auto point_count = static_cast<std::size_t>(message->width) * message->height;
    std::vector<ColoredDisplayPoint> ground_points;
    std::vector<ColoredDisplayPoint> structure_points;
    ground_points.reserve(has_local_display_subscriber ? point_count / 8 : 0);
    structure_points.reserve(has_local_display_subscriber ? point_count : 0);
    std::unordered_set<std::uint64_t> ground_voxels;
    ground_voxels.reserve(has_local_display_subscriber ? point_count / 8 : 0);
    std::size_t fallback_count = 0;
    try {
      sensor_msgs::PointCloud2ConstIterator<float> in_x(*message, "x");
      sensor_msgs::PointCloud2ConstIterator<float> in_y(*message, "y");
      sensor_msgs::PointCloud2ConstIterator<float> in_z(*message, "z");
      for (std::size_t i = 0; i < point_count;
        ++i, ++in_x, ++in_y, ++in_z)
      {
        if (!std::isfinite(*in_x) || !std::isfinite(*in_y) || !std::isfinite(*in_z)) continue;
        const auto point = map_from_cloud * tf2::Vector3(*in_x, *in_y, *in_z);
        const double dx = point.x() - robot_position.x();
        const double dy = point.y() - robot_position.y();
        if (dx * dx + dy * dy > radius_squared) continue;
        auto height = bringup::staticDisplayHeightAboveGround(
          static_ground_map_, static_cast<float>(point.x()), static_cast<float>(point.y()),
          static_cast<float>(point.z()), fallback_ground);
        if (!height) {
          height = static_cast<float>(point.z() - fallback_ground);
          ++fallback_count;
        }
        if (*height > local_max_height_above_ground_) continue;

        const ColoredDisplayPoint display_point{
          static_cast<float>(point.x()), static_cast<float>(point.y()),
          static_cast<float>(point.z()), *height};
        if (std::abs(*height) <= 0.10F) {
          if (ground_voxels.insert(groundVoxelKey(
              display_point.x, display_point.y, local_ground_voxel_size_)).second)
          {
            ground_points.push_back(display_point);
          }
        } else {
          structure_points.push_back(display_point);
        }
      }
    } catch (const std::runtime_error & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000, "Dropping local cloud without x/y/z fields: %s",
        error.what());
      return;
    }
    const auto display_stamp = get_clock()->now();
    local_ground_cloud_publisher_->publish(makeColoredCloud(
      ground_points, frame_id_, display_stamp));
    local_structure_cloud_publisher_->publish(makeColoredCloud(
      structure_points, frame_id_, display_stamp));
    const auto visible_count = ground_points.size() + structure_points.size();
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Local display cloud: input=%zu ground=%zu (%.2f m) structure=%zu fallback=%zu",
      point_count, ground_points.size(), local_ground_voxel_size_,
      structure_points.size(), fallback_count);
    if (fallback_count > 0) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Used calibrated sensor-height ground fallback for %zu/%zu local points",
        fallback_count, point_count);
    }
    last_local_cloud_received_ = std::chrono::steady_clock::now();
    have_local_cloud_ = true;
    local_cloud_visible_ = visible_count > 0;
  }

  void clearanceCloudCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr message)
  {
    if (clearance_candidate_cells_.empty() || structure_points_by_cell_.empty() ||
      message->header.frame_id.empty())
    {
      return;
    }

    const auto stamp = rclcpp::Time(message->header.stamp);
    const auto source_age = (get_clock()->now() - stamp).seconds();
    if (stamp.nanoseconds() == 0 || !std::isfinite(source_age) ||
      source_age > local_cloud_timeout_ || source_age < 0.0)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Dropping stale or future static-clearance cloud (age %.3f s)", source_age);
      return;
    }

    tf2::Transform map_from_cloud;
    map_from_cloud.setIdentity();
    if (message->header.frame_id != frame_id_) {
      try {
        const auto stamped = tf_buffer_.lookupTransform(
          frame_id_, message->header.frame_id, stamp,
          rclcpp::Duration::from_seconds(0.05));
        tf2::fromMsg(stamped.transform, map_from_cloud);
      } catch (const tf2::TransformException & error) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 3000,
          "Cannot transform static-clearance cloud to '%s': %s",
          frame_id_.c_str(), error.what());
        return;
      }
    }

    const auto point_count = static_cast<std::size_t>(message->width) * message->height;
    const auto ray_stride = std::max<std::size_t>(
      1, (point_count + kMaxStaticClearanceRaysPerScan - 1) /
      kMaxStaticClearanceRaysPerScan);
    std::vector<bringup::StaticPcdRay3D> rays;
    rays.reserve(std::min(point_count, kMaxStaticClearanceRaysPerScan));
    try {
      sensor_msgs::PointCloud2ConstIterator<float> point_x(*message, "x");
      sensor_msgs::PointCloud2ConstIterator<float> point_y(*message, "y");
      sensor_msgs::PointCloud2ConstIterator<float> point_z(*message, "z");
      sensor_msgs::PointCloud2ConstIterator<float> point_time(
        *message, "time_from_scan_end");
      sensor_msgs::PointCloud2ConstIterator<float> origin_x(*message, "ray_origin_x");
      sensor_msgs::PointCloud2ConstIterator<float> origin_y(*message, "ray_origin_y");
      sensor_msgs::PointCloud2ConstIterator<float> origin_z(*message, "ray_origin_z");
      for (std::size_t i = 0; i < point_count;
        ++i, ++point_x, ++point_y, ++point_z, ++point_time, ++origin_x, ++origin_y,
        ++origin_z)
      {
        if (i % ray_stride != 0 ||
          !std::isfinite(*point_x) || !std::isfinite(*point_y) ||
          !std::isfinite(*point_z) || !std::isfinite(*point_time) ||
          !std::isfinite(*origin_x) ||
          !std::isfinite(*origin_y) || !std::isfinite(*origin_z))
        {
          continue;
        }
        const auto endpoint = map_from_cloud * tf2::Vector3(*point_x, *point_y, *point_z);
        const auto origin = map_from_cloud * tf2::Vector3(*origin_x, *origin_y, *origin_z);
        rays.push_back({
          origin.x(), origin.y(), origin.z(),
          endpoint.x(), endpoint.y(), endpoint.z()});
      }
    } catch (const std::runtime_error & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "Dropping static-clearance cloud without paired point/origin fields: %s",
        error.what());
      return;
    }

    if (!rays.empty()) applyStaticPcdRayClearance(rays, stamp.nanoseconds());
  }

  void applyStaticPcdRayClearance(
    const std::vector<bringup::StaticPcdRay3D> & rays, std::uint64_t stamp_ns)
  {
    constexpr std::uint64_t kMaximumEvidenceGapNanoseconds = 500000000ULL;
    std::size_t newly_hidden_points = 0;
    bool diagnostic_ray_state_changed = false;
    for (const auto & ray : rays) {
      const double dx = ray.end_x - ray.start_x;
      const double dy = ray.end_y - ray.start_y;
      const double dz = ray.end_z - ray.start_z;
      const double range = std::sqrt(dx * dx + dy * dy + dz * dz);
      if (!std::isfinite(range) || range <= clearance_ray_min_range_) continue;
      const double start_fraction = clearance_ray_min_range_ / range;
      const double end_fraction = std::min(clearance_ray_max_range_ / range, 1.0);
      if (end_fraction <= start_fraction) continue;
      const bringup::StaticPcdRay3D clipped_ray{
        ray.start_x + start_fraction * dx,
        ray.start_y + start_fraction * dy,
        ray.start_z + start_fraction * dz,
        ray.start_x + end_fraction * dx,
        ray.start_y + end_fraction * dy,
        ray.start_z + end_fraction * dz};

      GridCellKey start_cell{};
      GridCellKey endpoint_cell{};
      if (!quantizeGridCoordinate(
          clipped_ray.start_x, static_pcd_base_origin_x_, static_pcd_map_resolution_,
          start_cell.x) ||
        !quantizeGridCoordinate(
          clipped_ray.start_y, static_pcd_base_origin_y_, static_pcd_map_resolution_,
          start_cell.y) ||
        !quantizeGridCoordinate(
          clipped_ray.end_x, static_pcd_base_origin_x_, static_pcd_map_resolution_,
          endpoint_cell.x) ||
        !quantizeGridCoordinate(
          clipped_ray.end_y, static_pcd_base_origin_y_, static_pcd_map_resolution_,
          endpoint_cell.y))
      {
        continue;
      }

      bringup::forEachStaticPcdRayCellBeforeEndpoint(
        start_cell, endpoint_cell,
        [this, &clipped_ray, stamp_ns, &newly_hidden_points, &diagnostic_ray_state_changed](
          const GridCellKey & cell) {
          if (clearance_candidate_cells_.find(cell) == clearance_candidate_cells_.end()) return;
          const auto refs = structure_points_by_cell_.find(cell);
          if (refs == structure_points_by_cell_.end()) return;
          for (const auto & ref : refs->second) {
            auto & point = static_structure_tiles_[ref.tile_index].points[ref.point_index];
            if (point.clearance.cleared() || !bringup::staticPcdRayClearsPoint(
                clipped_ray, point.point.x, point.point.y, point.point.z,
                static_pcd_map_resolution_, clearance_ray_height_resolution_))
            {
              continue;
            }
            const auto confirmations_before = point.clearance.observations();
            if (point.clearance.observe(
                stamp_ns, kMaximumEvidenceGapNanoseconds,
                static_cast<std::uint32_t>(clearance_ray_observations_)))
            {
              ++newly_hidden_points;
              dirty_static_structure_tiles_.insert(ref.tile_index);
            }
            if (diagnostic_cell_ && cell == *diagnostic_cell_ &&
              point.clearance.observations() != confirmations_before)
            {
              diagnostic_ray_state_changed = true;
            }
          }
        });
    }

    if (newly_hidden_points > 0) {
      hidden_structure_point_count_ += newly_hidden_points;
      static_structure_refresh_state_.markDirty();
      RCLCPP_INFO(
        get_logger(),
        "Static PCD height clearance: newly_hidden_points=%zu total_hidden_points=%zu ray_samples=%zu",
        newly_hidden_points, hidden_structure_point_count_, rays.size());
    }
    if (diagnostic_ray_state_changed) logDiagnosticRayState("height_ray_evidence", stamp_ns);
  }

  void expireLocalCloud()
  {
    if (!have_local_cloud_ || !local_cloud_visible_) return;
    const auto elapsed = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - last_local_cloud_received_).count();
    if (elapsed <= local_cloud_timeout_) return;

    const std::vector<ColoredDisplayPoint> no_points;
    const auto stamp = get_clock()->now();
    local_ground_cloud_publisher_->publish(makeColoredCloud(no_points, frame_id_, stamp));
    local_structure_cloud_publisher_->publish(makeColoredCloud(no_points, frame_id_, stamp));
    local_cloud_visible_ = false;
    RCLCPP_INFO(get_logger(), "Cleared timed-out local point cloud display");
  }

  void mapCallback(nav_msgs::msg::OccupancyGrid::ConstSharedPtr occupancy_map)
  {
    if (!occupancy_map) return;
    const auto stamp_ns = bringup::mapStampNanoseconds(occupancy_map->header.stamp);
    if (stamp_ns == 0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Ignoring /map without a valid timestamp for geometry pairing");
      return;
    }
    auto [entry, inserted] = pending_full_maps_.emplace(stamp_ns, occupancy_map);
    if (!inserted && !sameOccupancyGrid(*entry->second, *occupancy_map)) {
      pending_full_maps_.erase(entry);
      ambiguous_full_stamps_.insert(stamp_ns);
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Rejecting conflicting /map snapshots with the same timestamp");
    }
    drainMapGeometryQueues();
    boundPendingMapQueues();
  }

  void mapGeometryEventCallback(std_msgs::msg::UInt64MultiArray::ConstSharedPtr message)
  {
    if (!message) return;
    bringup::MapGeometryEvent event;
    if (!bringup::parseMapGeometryEvent(*message, event)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Ignoring malformed /map_geometry_events metadata");
      invalidateMapDeltaChain("malformed_geometry_event");
      return;
    }
    if (event.kind == bringup::MapGeometryEventKind::FULL_MAP) {
      const auto [entry, inserted] = pending_full_geometry_events_.emplace(event.stamp_ns, event);
      if (!inserted && bringup::serializeMapGeometryEvent(entry->second).data != message->data) {
        pending_full_geometry_events_.erase(entry);
        ambiguous_full_stamps_.insert(event.stamp_ns);
      }
    } else if (bringup::staticPcdMapUpdateCoveredByState(
        event.stamp_ns, latest_full_map_stamp_ns_, latest_navigation_map_stamp_ns_))
    {
      discardPendingMapUpdateAtStamp(event.stamp_ns);
    } else if (!attachMapUpdateGeometry(event)) {
      const auto [entry, inserted] = pending_map_update_geometry_.emplace(event.stamp_ns, event);
      if (!inserted && bringup::serializeMapGeometryEvent(entry->second).data != message->data) {
        pending_map_update_geometry_.erase(entry);
        ambiguous_map_update_stamps_.insert(event.stamp_ns);
        invalidateMapDeltaChain("conflicting_update_geometry_events");
      }
    }
    drainMapGeometryQueues();
    boundPendingMapQueues();
  }

  bool attachMapUpdateGeometry(const bringup::MapGeometryEvent & event)
  {
    auto matching = pending_map_updates_.end();
    for (auto entry = pending_map_updates_.begin(); entry != pending_map_updates_.end(); ++entry) {
      if (bringup::mapStampNanoseconds(entry->message->header.stamp) != event.stamp_ns) continue;
      if (entry->geometry) {
        ambiguous_map_update_stamps_.insert(event.stamp_ns);
        invalidateMapDeltaChain("duplicate_update_geometry_pair");
        return true;
      }
      if (matching != pending_map_updates_.end()) {
        ambiguous_map_update_stamps_.insert(event.stamp_ns);
        invalidateMapDeltaChain("ambiguous_update_geometry_pair");
        return true;
      }
      matching = entry;
    }
    if (matching == pending_map_updates_.end()) return false;
    matching->geometry = event;
    return true;
  }

  void mapUpdateCallback(map_msgs::msg::OccupancyGridUpdate::ConstSharedPtr update)
  {
    if (!update) return;
    const auto stamp_ns = bringup::mapStampNanoseconds(update->header.stamp);
    if (stamp_ns == 0) {
      invalidateMapDeltaChain("update_without_timestamp");
      return;
    }
    if (bringup::staticPcdMapUpdateCoveredByState(
        stamp_ns, latest_full_map_stamp_ns_, latest_navigation_map_stamp_ns_))
    {
      discardPendingMapUpdateAtStamp(stamp_ns);
      drainMapGeometryQueues();
      return;
    }
    PendingMapUpdate pending;
    pending.message = update;
    const auto event = pending_map_update_geometry_.find(stamp_ns);
    if (event != pending_map_update_geometry_.end()) {
      pending.geometry = event->second;
      pending_map_update_geometry_.erase(event);
    }
    if (ambiguous_map_update_stamps_.find(stamp_ns) != ambiguous_map_update_stamps_.end()) {
      invalidateMapDeltaChain("ambiguous_update_timestamp");
    }
    const auto duplicate = std::find_if(
      pending_map_updates_.begin(), pending_map_updates_.end(), [stamp_ns](const auto & entry) {
        return bringup::mapStampNanoseconds(entry.message->header.stamp) == stamp_ns;
      });
    if (duplicate != pending_map_updates_.end()) {
      ambiguous_map_update_stamps_.insert(stamp_ns);
      invalidateMapDeltaChain("duplicate_update_timestamp");
    }
    pending_map_updates_.push_back(std::move(pending));
    drainMapGeometryQueues();
    boundPendingMapQueues();
  }

  void drainMapGeometryQueues()
  {
    for (auto entry = pending_full_maps_.begin(); entry != pending_full_maps_.end();) {
      const auto stamp_ns = entry->first;
      const auto event = pending_full_geometry_events_.find(stamp_ns);
      if (event == pending_full_geometry_events_.end()) {
        ++entry;
        continue;
      }
      if (ambiguous_full_stamps_.erase(stamp_ns) > 0) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Ignoring ambiguous /map and geometry event pair at stamp=%llu",
          static_cast<unsigned long long>(stamp_ns));
      } else {
        processFullMapPair(*entry->second, event->second);
      }
      pending_full_geometry_events_.erase(event);
      entry = pending_full_maps_.erase(entry);
    }
    processPendingMapUpdates();
  }

  void processFullMapPair(
    const nav_msgs::msg::OccupancyGrid & snapshot,
    const bringup::MapGeometryEvent & event)
  {
    const auto disposition = bringup::assessStaticPcdFullMap(
      snapshot, event, map_publisher_epoch_, map_geometry_version_,
      latest_full_map_stamp_ns_, latest_navigation_map_stamp_ns_,
      latest_navigation_map_ ? &*latest_navigation_map_ : nullptr);
    if (disposition == bringup::StaticPcdFullMapDisposition::REJECT_MISMATCH ||
      disposition == bringup::StaticPcdFullMapDisposition::REJECT_STALE)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Ignoring /map geometry/version mismatch or stale full snapshot epoch=%llu version=%llu stamp=%llu",
        static_cast<unsigned long long>(event.publisher_epoch),
        static_cast<unsigned long long>(event.geometry_version),
        static_cast<unsigned long long>(event.stamp_ns));
      return;
    }
    if (disposition == bringup::StaticPcdFullMapDisposition::ADVANCE_WATERMARK_ONLY) {
      latest_full_map_stamp_ns_ = event.stamp_ns;
      latest_full_navigation_map_ = snapshot;
      discardStaleStartupSource(snapshot);
      applyStartupPriorClearanceEvidence();
      return;
    }

    bool changed = false;
    std::string error;
    if (!bringup::extendDisplayMapToOccupancyGrid(map_, snapshot, &changed, &error)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Cannot align 3D terrain display to /map: %s",
        error.c_str());
      return;
    }

    discardStaleStartupSource(snapshot);

    if (disposition == bringup::StaticPcdFullMapDisposition::ACCEPT_NEW_EPOCH) {
      std::vector<bringup::StaticPcdCell> revoked_cells;
      revoked_cells.reserve(clearance_candidate_cells_.size());
      for (const auto & cell : clearance_candidate_cells_) revoked_cells.push_back(cell);
      revokeStaticPcdClearanceCandidates(revoked_cells);
      static_pcd_clearance_tracker_->reset();
      startup_prior_clearance_applied_ = false;
      map_delta_chain_valid_ = false;
    }

    map_publisher_epoch_ = event.publisher_epoch;
    map_geometry_version_ = event.geometry_version;
    latest_full_map_stamp_ns_ = event.stamp_ns;
    latest_full_navigation_map_ = snapshot;
    latest_navigation_map_stamp_ns_ = event.stamp_ns;
    latest_navigation_map_ = snapshot;
    map_delta_chain_valid_ = true;
    discardCoveredPendingMapUpdates(event.publisher_epoch, event.stamp_ns);
    cacheNavigationMapSnapshot(snapshot);
    applyStartupPriorClearanceEvidence();
    if (!changed) return;
    terrain_map_dirty_ = true;
    global_surface_dirty_ = true;
    local_surface_dirty_ = true;
    effective_obstacles_dirty_ = true;
    RCLCPP_INFO(
      get_logger(), "Expanded 3D terrain display with /map to %dx%d cells at %.3f m",
      map_.getSize()(0), map_.getSize()(1), map_.getResolution());
  }

  void discardCoveredPendingMapUpdates(
    const std::uint64_t current_epoch, const std::uint64_t full_stamp_ns)
  {
    for (auto entry = pending_map_updates_.begin(); entry != pending_map_updates_.end();) {
      const bool covered_by_snapshot = bringup::mapStampNanoseconds(
        entry->message->header.stamp) <= full_stamp_ns;
      const bool from_old_epoch = entry->geometry &&
        entry->geometry->publisher_epoch < current_epoch;
      if (covered_by_snapshot || from_old_epoch) {
        entry = pending_map_updates_.erase(entry);
      } else {
        ++entry;
      }
    }
    for (auto entry = pending_map_update_geometry_.begin();
      entry != pending_map_update_geometry_.end();)
    {
      if (entry->first <= full_stamp_ns ||
        entry->second.publisher_epoch < current_epoch)
      {
        entry = pending_map_update_geometry_.erase(entry);
      } else {
        ++entry;
      }
    }
    for (auto entry = ambiguous_map_update_stamps_.begin();
      entry != ambiguous_map_update_stamps_.end();)
    {
      if (*entry <= full_stamp_ns) {
        entry = ambiguous_map_update_stamps_.erase(entry);
      } else {
        ++entry;
      }
    }
  }

  void discardPendingMapUpdateAtStamp(const std::uint64_t stamp_ns)
  {
    for (auto entry = pending_map_updates_.begin(); entry != pending_map_updates_.end();) {
      if (bringup::mapStampNanoseconds(entry->message->header.stamp) == stamp_ns) {
        entry = pending_map_updates_.erase(entry);
      } else {
        ++entry;
      }
    }
    pending_map_update_geometry_.erase(stamp_ns);
    ambiguous_map_update_stamps_.erase(stamp_ns);
  }

  void discardStaleStartupSource(const nav_msgs::msg::OccupancyGrid & full_map)
  {
    if (latest_occupancy_source_ && bringup::shouldDiscardStaticPcdOccupancySource(
        full_map, *latest_occupancy_source_))
    {
      latest_occupancy_source_.reset();
    }
  }

  void processPendingMapUpdates()
  {
    while (!pending_map_updates_.empty()) {
      auto & pending = pending_map_updates_.front();
      const auto stamp_ns = bringup::mapStampNanoseconds(pending.message->header.stamp);
      if (bringup::staticPcdMapUpdateCoveredByState(
          stamp_ns, latest_full_map_stamp_ns_, latest_navigation_map_stamp_ns_))
      {
        discardPendingMapUpdateAtStamp(stamp_ns);
        continue;
      }
      if (ambiguous_map_update_stamps_.find(stamp_ns) != ambiguous_map_update_stamps_.end()) {
        invalidateMapDeltaChain("ambiguous_update_pair");
        return;
      }
      if (!pending.geometry) return;
      if (!latest_navigation_map_) return;
      const auto disposition = bringup::assessStaticPcdMapUpdate(
        *latest_navigation_map_, *pending.message, *pending.geometry,
        map_publisher_epoch_, map_geometry_version_, latest_navigation_map_stamp_ns_);
      if (disposition == bringup::StaticPcdMapUpdateDisposition::WAIT_FOR_FULL_MAP) return;
      if (disposition == bringup::StaticPcdMapUpdateDisposition::REJECT_STALE) {
        pending_map_updates_.pop_front();
        continue;
      }
      if (disposition != bringup::StaticPcdMapUpdateDisposition::READY) {
        invalidateMapDeltaChain(
          disposition == bringup::StaticPcdMapUpdateDisposition::REJECT_MISMATCH ?
          "update_geometry_mismatch" : "invalid_update");
        return;
      }
      if (!map_delta_chain_valid_) return;
      auto updated_map = *latest_navigation_map_;
      if (!bringup::applyStaticPcdMapUpdate(
          updated_map, *pending.message, *pending.geometry,
          map_publisher_epoch_, map_geometry_version_))
      {
        invalidateMapDeltaChain("update_rejected_after_validation");
        return;
      }
      latest_navigation_map_stamp_ns_ = stamp_ns;
      const bool changed = updated_map.data != latest_navigation_map_->data;
      latest_navigation_map_ = std::move(updated_map);
      pending_map_updates_.pop_front();
      if (!changed) continue;
      cacheNavigationMapSnapshot(*latest_navigation_map_);
      applyStartupPriorClearanceEvidence();
    }
  }

  void invalidateMapDeltaChain(const char * reason)
  {
    const bool had_pending = !pending_map_updates_.empty() ||
      !pending_map_update_geometry_.empty() || !ambiguous_map_update_stamps_.empty();
    if (!map_delta_chain_valid_ && !had_pending) return;
    map_delta_chain_valid_ = false;
    pending_map_updates_.clear();
    pending_map_update_geometry_.clear();
    ambiguous_map_update_stamps_.clear();
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Pausing /map_updates as clearance evidence until a paired full map arrives: %s", reason);
  }

  void boundPendingMapQueues()
  {
    if (pending_map_updates_.size() > kMaximumPendingMapMessages ||
      pending_map_update_geometry_.size() > kMaximumPendingMapMessages)
    {
      invalidateMapDeltaChain("pending_update_queue_overflow");
    }
    while (pending_full_maps_.size() > kMaximumPendingMapMessages) {
      const auto dropped_stamp = pending_full_maps_.begin()->first;
      pending_full_maps_.erase(pending_full_maps_.begin());
      pending_full_geometry_events_.erase(dropped_stamp);
      ambiguous_full_stamps_.erase(dropped_stamp);
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Dropping oldest unpaired full map metadata after queue overflow");
    }
    while (pending_full_geometry_events_.size() > kMaximumPendingMapMessages) {
      pending_full_geometry_events_.erase(pending_full_geometry_events_.begin());
    }
    while (ambiguous_full_stamps_.size() > kMaximumPendingMapMessages) {
      ambiguous_full_stamps_.erase(ambiguous_full_stamps_.begin());
    }
    while (ambiguous_map_update_stamps_.size() > kMaximumPendingMapMessages) {
      ambiguous_map_update_stamps_.erase(ambiguous_map_update_stamps_.begin());
    }
  }

  void cacheNavigationMapSnapshot(const nav_msgs::msg::OccupancyGrid & snapshot)
  {
    if (!static_pcd_clearance_tracker_) return;
    std::vector<bringup::StaticPcdCell> cleared_cells;
    std::string error;
    const auto result = static_pcd_clearance_tracker_->observe(snapshot, cleared_cells, &error);
    if (result == bringup::StaticPcdSnapshotResult::INVALID) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Cannot use full /map snapshot for static PCD clearing: %s", error.c_str());
      return;
    }
    if (result == bringup::StaticPcdSnapshotResult::STALE) return;

    std::vector<bringup::StaticPcdCell> current_candidates;
    current_candidates.reserve(clearance_candidate_cells_.size());
    for (const auto & cell : clearance_candidate_cells_) current_candidates.push_back(cell);
    std::vector<bringup::StaticPcdCell> revoked_cells;
    if (!static_pcd_clearance_tracker_->collectCandidatesNoLongerFree(
        snapshot, current_candidates, revoked_cells, &error))
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Cannot reconcile static PCD clearance candidates; revoking them conservatively: %s",
        error.c_str());
      revoked_cells = std::move(current_candidates);
    }
    revokeStaticPcdClearanceCandidates(revoked_cells);

    if (result == bringup::StaticPcdSnapshotResult::INITIALIZED) return;

    std::size_t newly_eligible_cells = 0;
    for (const auto & cell : cleared_cells) {
      if (recordStaticPcdClear({cell.x, cell.y})) ++newly_eligible_cells;
    }
    if (diagnostic_cell_ && std::find(
        cleared_cells.begin(), cleared_cells.end(), *diagnostic_cell_) != cleared_cells.end())
    {
      const auto snapshot_stamp_ns = static_cast<std::uint64_t>(snapshot.header.stamp.sec) *
        1000000000ULL + snapshot.header.stamp.nanosec;
      diagnostic_map_free_stamp_ns_ = snapshot_stamp_ns;
      const auto stats = diagnosticRayStats();
      RCLCPP_INFO(
        get_logger(), "Static PCD cell diagnostic: event=map_free world=(%.3f,%.3f) "
        "grid=(%lld,%lld) ray_confirmations=%u hidden_points=%zu map_free_stamp_ns=%llu",
        diagnostic_cell_x_, diagnostic_cell_y_,
        static_cast<long long>(diagnostic_cell_->x),
        static_cast<long long>(diagnostic_cell_->y), stats.max_confirmations,
        stats.hidden_points, static_cast<unsigned long long>(diagnostic_map_free_stamp_ns_));
    }
    if (newly_eligible_cells > 0) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Static PCD map evidence: eligible XY cells=%zu; 3D ray evidence is still required",
        newly_eligible_cells);
    }
  }

  void revokeStaticPcdClearanceCandidates(
    const std::vector<bringup::StaticPcdCell> & revoked_cells)
  {
    std::size_t reset_points = 0;
    std::size_t restored_points = 0;
    for (const auto & cell : revoked_cells) {
      if (clearance_candidate_cells_.erase(cell) == 0) continue;
      const auto refs = structure_points_by_cell_.find(cell);
      if (refs == structure_points_by_cell_.end()) continue;
      for (const auto & ref : refs->second) {
        auto & point = static_structure_tiles_[ref.tile_index].points[ref.point_index];
        if (point.clearance.cleared()) ++restored_points;
        const bool cell_has_clearance_state = point.clearance.observations() > 0 ||
          point.clearance.cleared();
        point.clearance.reset();
        if (cell_has_clearance_state) dirty_static_structure_tiles_.insert(ref.tile_index);
        ++reset_points;
      }
    }
    if (restored_points > 0) {
      hidden_structure_point_count_ = restored_points > hidden_structure_point_count_
        ? 0 : hidden_structure_point_count_ - restored_points;
      static_structure_refresh_state_.markDirty();
    }
    if (!revoked_cells.empty()) {
      RCLCPP_INFO(
        get_logger(),
        "Static PCD clearance candidates revoked after map change: cells=%zu evidence_reset=%zu restored_points=%zu hidden_points=%zu",
        revoked_cells.size(), reset_points, restored_points, hidden_structure_point_count_);
    }
    if (diagnostic_cell_ && std::find(
        revoked_cells.begin(), revoked_cells.end(), *diagnostic_cell_) != revoked_cells.end())
    {
      diagnostic_map_free_stamp_ns_ = 0;
      logDiagnosticRayState("candidate_revoked", 0);
    }
  }

  void applyStartupPriorClearanceEvidence()
  {
    if (startup_prior_clearance_applied_ || !latest_full_navigation_map_ ||
      !latest_navigation_map_ ||
      !latest_occupancy_source_ || !static_pcd_clearance_tracker_)
    {
      return;
    }

    std::vector<bringup::StaticPcdCell> cleared_cells;
    std::string error;
    if (!static_pcd_clearance_tracker_->collectStartupPriorClearanceCandidates(
        *latest_full_navigation_map_, *latest_occupancy_source_, *latest_navigation_map_,
        cleared_cells, &error))
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Cannot use startup /map and occupancy-source evidence for static PCD clearing: %s",
        error.c_str());
      return;
    }

    std::size_t newly_eligible_cells = 0;
    for (const auto & cell : cleared_cells) {
      if (recordStaticPcdClear({cell.x, cell.y})) ++newly_eligible_cells;
    }
    startup_prior_clearance_applied_ = true;
    RCLCPP_INFO(
      get_logger(),
      "Static PCD startup evidence: prior-occupied cells still free=%zu, eligible XY cells=%zu; 3D ray evidence is still required",
      cleared_cells.size(), newly_eligible_cells);
  }

  bool recordStaticPcdClear(const GridCellKey & cell)
  {
    const auto refs = structure_points_by_cell_.find(cell);
    const bool inserted = refs != structure_points_by_cell_.end() &&
      clearance_candidate_cells_.insert(cell).second;
    return inserted;
  }

  struct DiagnosticRayStats
  {
    std::uint32_t max_confirmations{0};
    std::size_t structure_points{0};
    std::size_t hidden_points{0};
  };

  DiagnosticRayStats diagnosticRayStats() const
  {
    DiagnosticRayStats stats;
    if (!diagnostic_cell_) return stats;
    const auto refs = structure_points_by_cell_.find(*diagnostic_cell_);
    if (refs == structure_points_by_cell_.end()) return stats;
    stats.structure_points = refs->second.size();
    for (const auto & ref : refs->second) {
      const auto & evidence = static_structure_tiles_[ref.tile_index].points[ref.point_index].clearance;
      stats.max_confirmations = std::max<std::uint32_t>(
        stats.max_confirmations, evidence.observations());
      if (evidence.cleared()) ++stats.hidden_points;
    }
    return stats;
  }

  void logDiagnosticRayState(const char * event, std::uint64_t stamp_ns)
  {
    if (!diagnostic_cell_) return;
    const auto stats = diagnosticRayStats();
    const bool visible_state_changed = !diagnostic_state_initialized_ ||
      stats.hidden_points != diagnostic_last_hidden_points_;
    if (diagnostic_state_initialized_ &&
      stats.max_confirmations == diagnostic_last_max_confirmations_ &&
      stats.hidden_points == diagnostic_last_hidden_points_ &&
      stats.structure_points == diagnostic_last_structure_points_ &&
      diagnostic_map_free_stamp_ns_ == diagnostic_last_map_free_stamp_ns_)
    {
      return;
    }
    diagnostic_state_initialized_ = true;
    diagnostic_last_max_confirmations_ = stats.max_confirmations;
    diagnostic_last_hidden_points_ = stats.hidden_points;
    diagnostic_last_structure_points_ = stats.structure_points;
    diagnostic_last_map_free_stamp_ns_ = diagnostic_map_free_stamp_ns_;
    if (visible_state_changed) diagnostic_display_pending_ = true;
    RCLCPP_INFO(
      get_logger(), "Static PCD cell diagnostic: event=%s world=(%.3f,%.3f) "
      "grid=(%lld,%lld) ray_confirmations=%u structure_points=%zu hidden_points=%zu "
      "map_free_stamp_ns=%llu ray_stamp_ns=%llu",
      event, diagnostic_cell_x_, diagnostic_cell_y_,
      static_cast<long long>(diagnostic_cell_->x),
      static_cast<long long>(diagnostic_cell_->y), stats.max_confirmations,
      stats.structure_points, stats.hidden_points,
      static_cast<unsigned long long>(diagnostic_map_free_stamp_ns_),
      static_cast<unsigned long long>(stamp_ns));
  }

  void buildStaticStructureIndex()
  {
    if (pending_structure_points_.empty() || static_pcd_map_resolution_ <= 0.0) return;
    constexpr double kTileSizeMeters = 4.0;
    const auto tile_cells = std::max<std::int64_t>(
      1, static_cast<std::int64_t>(std::ceil(kTileSizeMeters / static_pcd_map_resolution_)));
    std::unordered_map<GridCellKey, std::int32_t, GridCellKeyHash> tile_ids;
    tile_ids.reserve(pending_structure_points_.size() / 512 + 1);
    structure_points_by_cell_.reserve(pending_structure_points_.size() / 4 + 1);
    for (const auto & point : pending_structure_points_) {
      GridCellKey cell{};
      if (!quantizeGridCoordinate(
          point.x, static_pcd_base_origin_x_, static_pcd_map_resolution_, cell.x) ||
        !quantizeGridCoordinate(
          point.y, static_pcd_base_origin_y_, static_pcd_map_resolution_, cell.y))
      {
        continue;
      }
      const GridCellKey tile_key{
        floorDivide(cell.x, tile_cells), floorDivide(cell.y, tile_cells)};
      auto tile_id = tile_ids.find(tile_key);
      if (tile_id == tile_ids.end()) {
        if (static_structure_tiles_.size() >=
          static_cast<std::size_t>(std::numeric_limits<std::int32_t>::max()))
        {
          RCLCPP_ERROR(get_logger(), "Static PCD structure display exceeds marker ID capacity");
          break;
        }
        const auto id = static_cast<std::int32_t>(static_structure_tiles_.size());
        static_structure_tiles_.push_back({id, {}});
        tile_id = tile_ids.emplace(tile_key, id).first;
      }
      auto & tile = static_structure_tiles_[static_cast<std::size_t>(tile_id->second)];
      const auto point_index = tile.points.size();
      tile.points.push_back({point, cell, {}});
      structure_points_by_cell_[cell].push_back({
        static_cast<std::size_t>(tile_id->second), point_index});
    }
    pending_structure_points_.clear();
    pending_structure_points_.shrink_to_fit();
    static_structure_refresh_state_.markDirty();
    RCLCPP_INFO(
      get_logger(), "Indexed %zu static PCD structure tiles for evidence-based clearing",
      static_structure_tiles_.size());
  }

  std::size_t staticStructurePointCount() const
  {
    std::size_t count = 0;
    for (const auto & tile : static_structure_tiles_) count += tile.points.size();
    return count;
  }

  std::size_t loadStaticStructureIndex(const std::string & path, float max_height_above_ground)
  {
    std::ifstream input(path, std::ios::binary);
    if (!input) throw std::runtime_error("Cannot open static structure index: " + path);
    constexpr char expected_magic[8]{'L', '1', 'S', 'P', 'C', 'D', 'I', '1'};
    char magic[sizeof(expected_magic)]{};
    input.read(magic, sizeof(magic));
    if (!input || std::memcmp(expected_magic, magic, sizeof(magic)) != 0) {
      throw std::runtime_error("Invalid static structure index header: " + path);
    }
    double resolution = 0.0;
    double origin_x = 0.0;
    double origin_y = 0.0;
    std::int64_t tile_cells = 0;
    std::uint64_t point_count = 0;
    std::uint64_t tile_count = 0;
    input.read(reinterpret_cast<char *>(&resolution), sizeof(resolution));
    input.read(reinterpret_cast<char *>(&origin_x), sizeof(origin_x));
    input.read(reinterpret_cast<char *>(&origin_y), sizeof(origin_y));
    input.read(reinterpret_cast<char *>(&tile_cells), sizeof(tile_cells));
    input.read(reinterpret_cast<char *>(&point_count), sizeof(point_count));
    input.read(reinterpret_cast<char *>(&tile_count), sizeof(tile_count));
    constexpr std::uint64_t kMaxCachedStructurePoints = 20'000'000;
    const auto expected_tile_cells = std::max<std::int64_t>(
      1, static_cast<std::int64_t>(std::ceil(4.0 / static_pcd_map_resolution_)));
    if (!input || !std::isfinite(resolution) || !std::isfinite(origin_x) ||
      !std::isfinite(origin_y) || resolution <= 0.0 || tile_cells != expected_tile_cells ||
      std::abs(resolution - static_pcd_map_resolution_) > 1e-9 ||
      std::abs(origin_x - static_pcd_base_origin_x_) > 1e-6 ||
      std::abs(origin_y - static_pcd_base_origin_y_) > 1e-6 ||
      point_count > kMaxCachedStructurePoints || tile_count > point_count ||
      (point_count == 0) != (tile_count == 0))
    {
      throw std::runtime_error("Static structure index geometry or counts do not match the terrain map");
    }

    structure_points_by_cell_.clear();
    static_structure_tiles_.clear();
    structure_points_by_cell_.reserve(static_cast<std::size_t>(point_count / 4 + 1));
    std::int32_t last_record_tile_id = -1;
    std::int32_t current_loaded_tile_id = -1;
    std::uint64_t source_tile_count = 0;
    std::size_t height_rejected = 0;
    for (std::uint64_t index = 0; index < point_count; ++index) {
      std::int32_t tile_id = -1;
      GridCellKey cell{};
      ColoredDisplayPoint point{};
      std::array<float, 4> cached_point{};
      input.read(reinterpret_cast<char *>(&tile_id), sizeof(tile_id));
      input.read(reinterpret_cast<char *>(&cell.x), sizeof(cell.x));
      input.read(reinterpret_cast<char *>(&cell.y), sizeof(cell.y));
      input.read(reinterpret_cast<char *>(cached_point.data()), sizeof(cached_point));
      point = {cached_point[0], cached_point[1], cached_point[2], cached_point[3]};
      if (!input || tile_id < 0 || !std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.z) || !std::isfinite(point.height_above_ground))
      {
        throw std::runtime_error("Truncated or invalid static structure index record");
      }
      if (tile_id != last_record_tile_id) {
        if (tile_id != last_record_tile_id + 1) {
          throw std::runtime_error("Static structure index tile IDs are not contiguous");
        }
        last_record_tile_id = tile_id;
        current_loaded_tile_id = -1;
        ++source_tile_count;
      }
      GridCellKey expected_cell{};
      if (!quantizeGridCoordinate(
          point.x, static_pcd_base_origin_x_, static_pcd_map_resolution_, expected_cell.x) ||
        !quantizeGridCoordinate(
          point.y, static_pcd_base_origin_y_, static_pcd_map_resolution_, expected_cell.y) ||
        !(expected_cell == cell))
      {
        throw std::runtime_error("Static structure index contains a point/cell mismatch");
      }
      if (point.height_above_ground > max_height_above_ground) {
        ++height_rejected;
        continue;
      }
      if (current_loaded_tile_id < 0) {
        current_loaded_tile_id = static_cast<std::int32_t>(static_structure_tiles_.size());
        static_structure_tiles_.push_back({current_loaded_tile_id, {}});
      }
      auto & tile = static_structure_tiles_.back();
      const auto point_index = tile.points.size();
      tile.points.push_back({point, cell, {}});
      structure_points_by_cell_[cell].push_back({
        static_cast<std::size_t>(current_loaded_tile_id), point_index});
    }
    if (!input || source_tile_count != tile_count) {
      throw std::runtime_error("Static structure index has inconsistent tile records");
    }
    if (input.peek() != std::char_traits<char>::eof()) {
      throw std::runtime_error("Static structure index contains trailing data");
    }
    static_structure_refresh_state_.markDirty();
    RCLCPP_INFO(
      get_logger(), "Loaded preindexed static PCD structures: %zu points, %zu tiles",
      static_cast<std::size_t>(point_count), static_structure_tiles_.size());
    return height_rejected;
  }

  visualization_msgs::msg::Marker makeStaticStructureMarker(
    const StaticStructureTile & tile, const rclcpp::Time & stamp) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id_;
    marker.header.stamp = stamp;
    marker.ns = "static_pcd_structures";
    marker.id = tile.marker_id;
    marker.type = visualization_msgs::msg::Marker::POINTS;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.025;
    marker.scale.y = 0.025;
    marker.scale.z = 0.025;
    marker.points.reserve(tile.points.size());
    marker.colors.reserve(tile.points.size());
    for (const auto & entry : tile.points) {
      if (entry.clearance.cleared()) continue;
      geometry_msgs::msg::Point point;
      point.x = entry.point.x;
      point.y = entry.point.y;
      point.z = entry.point.z;
      marker.points.push_back(point);
      const auto rgb = bringup::staticDisplayColor(entry.point.height_above_ground);
      std_msgs::msg::ColorRGBA color;
      color.r = static_cast<float>(rgb[0]) / 255.0F;
      color.g = static_cast<float>(rgb[1]) / 255.0F;
      color.b = static_cast<float>(rgb[2]) / 255.0F;
      color.a = 1.0F;
      marker.colors.push_back(color);
    }
    if (marker.points.empty()) marker.action = visualization_msgs::msg::Marker::DELETE;
    return marker;
  }

  std::vector<std::string> staticStructureSubscriberIds()
  {
    const auto subscription_count = static_structure_publisher_->get_subscription_count();
    if (subscription_count == 0) return {};

    std::vector<std::string> identifiers;
    try {
      const auto endpoints = get_subscriptions_info_by_topic(
        static_structure_publisher_->get_topic_name());
      identifiers.reserve(endpoints.size());
      for (const auto & endpoint : endpoints) {
        const auto & gid = endpoint.endpoint_gid();
        std::string identifier = endpoint.node_namespace();
        identifier.push_back('/');
        identifier.append(endpoint.node_name());
        identifier.push_back('\0');
        identifier.append(reinterpret_cast<const char *>(gid.data()), gid.size());
        identifiers.push_back(std::move(identifier));
      }
    } catch (const std::exception & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Cannot inspect static PCD display subscribers: %s", error.what());
    }

    if (!identifiers.empty()) return identifiers;

    // A matched publisher without graph details is treated as a new endpoint
    // at each poll. The display rate limit still bounds the full replay.
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Static PCD display has %zu matched subscriber(s), but no endpoint identities yet",
      subscription_count);
    identifiers.push_back(
      "static-pcd-subscriber-discovery-pending-" +
      std::to_string(++subscriber_identity_fallback_epoch_));
    return identifiers;
  }

  void refreshStaticStructureDisplay()
  {
    if (!static_structure_publisher_ || static_structure_tiles_.empty()) return;
    const auto now = std::chrono::steady_clock::now();
    const auto plan = static_structure_refresh_state_.observeSubscribers(
      staticStructureSubscriberIds(), now);
    if (!plan.should_publish) return;

    const auto stamp = get_clock()->now();
    visualization_msgs::msg::MarkerArray markers;
    std::vector<std::size_t> published_tiles;
    if (plan.full_refresh) {
      markers.markers.reserve(static_structure_tiles_.size());
      published_tiles.reserve(static_structure_tiles_.size());
      for (std::size_t index = 0; index < static_structure_tiles_.size(); ++index) {
        markers.markers.push_back(makeStaticStructureMarker(static_structure_tiles_[index], stamp));
        published_tiles.push_back(index);
      }
    } else {
      markers.markers.reserve(dirty_static_structure_tiles_.size());
      published_tiles.assign(
        dirty_static_structure_tiles_.begin(), dirty_static_structure_tiles_.end());
      std::sort(published_tiles.begin(), published_tiles.end());
      for (const auto index : published_tiles) {
        if (index >= static_structure_tiles_.size()) continue;
        markers.markers.push_back(makeStaticStructureMarker(static_structure_tiles_[index], stamp));
      }
    }
    if (markers.markers.empty()) return;
    static_structure_publisher_->publish(markers);
    const auto published_at = std::chrono::steady_clock::now();
    static_structure_refresh_state_.markPublished(published_at, plan.full_refresh);
    for (const auto index : published_tiles) dirty_static_structure_tiles_.erase(index);
    RCLCPP_INFO(
      get_logger(),
      "Published static PCD display %s: markers=%zu hidden_points=%zu matched_subscribers=%zu",
      plan.full_refresh ? "full snapshot" : "clearing-region update", markers.markers.size(),
      hidden_structure_point_count_,
      static_structure_publisher_->get_subscription_count());
    if (diagnostic_display_pending_ && diagnosticCellInTiles(published_tiles, plan.full_refresh)) {
      const auto stats = diagnosticRayStats();
      RCLCPP_INFO(
        get_logger(), "Static PCD cell diagnostic: event=display_updated grid=(%lld,%lld) "
        "ray_confirmations=%u hidden_points=%zu map_free_stamp_ns=%llu display_stamp_ns=%lld",
        static_cast<long long>(diagnostic_cell_->x),
        static_cast<long long>(diagnostic_cell_->y), stats.max_confirmations,
        stats.hidden_points, static_cast<unsigned long long>(diagnostic_map_free_stamp_ns_),
        static_cast<long long>(stamp.nanoseconds()));
      diagnostic_display_pending_ = false;
    }
  }

  bool diagnosticCellInTiles(
    const std::vector<std::size_t> & tiles, bool full_refresh) const
  {
    if (!diagnostic_cell_) return false;
    const auto refs = structure_points_by_cell_.find(*diagnostic_cell_);
    if (refs == structure_points_by_cell_.end()) return false;
    if (full_refresh) return true;
    for (const auto & ref : refs->second) {
      if (std::find(tiles.begin(), tiles.end(), ref.tile_index) != tiles.end()) return true;
    }
    return false;
  }

  void refreshEffectiveObstacles()
  {
    if (!global_grid_ || !effective_obstacles_) return;
    if (effective_obstacles_->get_subscription_count() == 0) {
      effective_obstacles_drawn_ = false;
      effective_obstacles_dirty_ = true;
      return;
    }
    if (!effective_obstacles_dirty_ && effective_obstacles_drawn_) return;
    const auto now = std::chrono::steady_clock::now();
    if (last_effective_obstacles_publish_ != std::chrono::steady_clock::time_point{} &&
      now - last_effective_obstacles_publish_ <
      bringup::StaticPcdRefreshState::kIncrementalPublishPeriod)
    {
      return;
    }

    tf2::Transform map_from_frame;
    map_from_frame.setIdentity();
    if (global_grid_->header.frame_id != frame_id_) {
      try {
        const auto transform = tf_buffer_.lookupTransform(
          frame_id_, global_grid_->header.frame_id, global_grid_->header.stamp,
          rclcpp::Duration::from_seconds(0.05));
        tf2::fromMsg(transform.transform, map_from_frame);
      } catch (const tf2::TransformException & error) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Cannot draw global navigation lethal cells: %s", error.what());
        return;
      }
    }
    tf2::Transform frame_from_grid;
    tf2::fromMsg(global_grid_->info.origin, frame_from_grid);
    const auto map_from_grid = map_from_frame * frame_from_grid;
    const auto previous_publish = last_effective_obstacles_publish_;
    const auto render_started = std::chrono::steady_clock::now();
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id_;
    marker.header.stamp = get_clock()->now();
    marker.ns = "global_navigation_lethal_obstacles";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = global_grid_->info.resolution;
    marker.scale.y = global_grid_->info.resolution;
    marker.scale.z = 0.06;
    marker.color.r = 1.0F;
    marker.color.g = 0.08F;
    marker.color.b = 0.02F;
    marker.color.a = 0.95F;
    const auto width = global_grid_->info.width;
    const auto height = global_grid_->info.height;
    if (width > 0 && height > 0 &&
      global_grid_->data.size() == static_cast<std::size_t>(width) * height)
    {
      for (std::uint32_t y = 0; y < height; ++y) {
        for (std::uint32_t x = 0; x < width; ++x) {
          if (global_grid_->data[static_cast<std::size_t>(y) * width + x] < 100) continue;
          const auto map_point = map_from_grid * tf2::Vector3(
            (static_cast<double>(x) + 0.5) * global_grid_->info.resolution,
            (static_cast<double>(y) + 0.5) * global_grid_->info.resolution, 0.0);
          geometry_msgs::msg::Point point;
          point.x = map_point.x();
          point.y = map_point.y();
          const auto ground = bringup::staticDisplayHeightAboveGround(
            map_, static_cast<float>(point.x), static_cast<float>(point.y), 0.0F,
            fallback_ground_);
          point.z = ground ? -*ground + 0.06 : fallback_ground_ + 0.06;
          marker.points.push_back(point);
        }
      }
    }
    if (marker.points.empty()) marker.action = visualization_msgs::msg::Marker::DELETE;
    const auto cells_drawn = marker.points.size();
    visualization_msgs::msg::MarkerArray markers;
    markers.markers.push_back(std::move(marker));
    const auto render_finished = std::chrono::steady_clock::now();
    const auto publish_started = std::chrono::steady_clock::now();
    effective_obstacles_->publish(markers);
    const auto published_at = std::chrono::steady_clock::now();
    last_effective_obstacles_publish_ = published_at;
    effective_obstacles_dirty_ = false;
    effective_obstacles_drawn_ = true;
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Global lethal display refresh: cells=%zu render=%.3f ms publish=%.3f ms interval=%.1f ms",
      cells_drawn,
      std::chrono::duration<double, std::milli>(
        render_finished - render_started).count(),
      std::chrono::duration<double, std::milli>(
        published_at - publish_started).count(),
      previous_publish == std::chrono::steady_clock::time_point{} ? 0.0 :
      std::chrono::duration<double, std::milli>(published_at - previous_publish).count());
  }

  void onlineTerrainCallback(grid_map_msgs::msg::GridMap::ConstSharedPtr message)
  {
    if (message->header.frame_id.empty()) return;
    if ((!publisher_ || publisher_->get_subscription_count() == 0) &&
      (!global_inflation_ || global_inflation_->get_subscription_count() == 0) &&
      (!local_inflation_ || local_inflation_->get_subscription_count() == 0))
    {
      return;
    }
    latest_online_terrain_message_ = std::move(message);
    online_terrain_dirty_ = true;
  }

  void mergeLatestOnlineTerrain()
  {
    if (!online_terrain_dirty_ || !latest_online_terrain_message_) return;
    const auto now = std::chrono::steady_clock::now();
    if (last_online_terrain_merge_ != std::chrono::steady_clock::time_point{} &&
      now - last_online_terrain_merge_ < std::chrono::milliseconds(1000))
    {
      return;
    }
    const auto message = latest_online_terrain_message_;
    online_terrain_dirty_ = false;
    last_online_terrain_merge_ = now;
    tf2::Transform display_from_online;
    display_from_online.setIdentity();
    if (message->header.frame_id != frame_id_) {
      try {
        const auto stamped = tf_buffer_.lookupTransform(
          frame_id_, message->header.frame_id, rclcpp::Time(message->header.stamp),
          rclcpp::Duration::from_seconds(0.15));
        tf2::fromMsg(stamped.transform, display_from_online);
      } catch (const tf2::TransformException & error) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 3000,
          "Cannot merge live FastDEM terrain into the global 3D display: %s", error.what());
        return;
      }
    }
    try {
      const auto online_map = nanogrid::ros2::fromMsg(*message);
      const auto merged = bringup::mergeOnlineDisplayTerrain(
        map_, online_map, display_from_online, 3, 3);
      if (merged == 0) return;
      terrain_map_dirty_ = true;
      global_surface_dirty_ = true;
      local_surface_dirty_ = true;
      effective_obstacles_dirty_ = true;
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Merged %zu fresh FastDEM ground cells into the shared 3D map", merged);
    } catch (const std::exception & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Cannot decode live FastDEM terrain: %s", error.what());
    }
  }

  void applyUpdate(const map_msgs::msg::OccupancyGridUpdate & update, bool local)
  {
    auto & grid = local ? local_grid_ : global_grid_;
    if (!grid) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "%s costmap update arrived before full map",
        local ? "Local" : "Global");
      return;
    }
    bool changed = false;
    if (!bringup::applyCostmapUpdate(*grid, update, &changed)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "%s costmap update dimensions do not fit",
        local ? "Local" : "Global");
      return;
    }
    grid->header.stamp = update.header.stamp;
    auto & drawn = local ? local_surface_drawn_ : global_surface_drawn_;
    auto & dirty = local ? local_surface_dirty_ : global_surface_dirty_;
    if (changed || !drawn) dirty = true;
    if (!local && changed) effective_obstacles_dirty_ = true;
    if (local && changed) logLocalCostmapCounts(*grid);
  }

  void retryInflationSurfaces()
  {
    mergeLatestOnlineTerrain();
    refreshTerrainMap();
    refreshStaticStructureDisplay();
    refreshEffectiveObstacles();
    refreshInflationSurface(false);
    refreshInflationSurface(true);
  }

  void refreshTerrainMap()
  {
    if (!terrain_map_dirty_ || !publisher_ || publisher_->get_subscription_count() == 0) return;
    const auto now = std::chrono::steady_clock::now();
    if (last_terrain_map_publish_ != std::chrono::steady_clock::time_point{} &&
      now - last_terrain_map_publish_ < std::chrono::milliseconds(1000))
    {
      return;
    }
    map_.setTimestamp(static_cast<uint64_t>(get_clock()->now().nanoseconds()));
    const auto conversion_started = std::chrono::steady_clock::now();
    const auto message = fastdem::ros2::toGridMap(map_, "display_elevation");
    const auto conversion_finished = std::chrono::steady_clock::now();
    const auto publish_started = std::chrono::steady_clock::now();
    publisher_->publish(message);
    const auto publish_finished = std::chrono::steady_clock::now();
    last_terrain_map_publish_ = publish_finished;
    terrain_map_dirty_ = false;
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Global terrain display refresh: cells=%zu conversion=%.3f ms publish=%.3f ms",
      static_cast<std::size_t>(map_.getSize().prod()),
      std::chrono::duration<double, std::milli>(conversion_finished - conversion_started).count(),
      std::chrono::duration<double, std::milli>(publish_finished - publish_started).count());
  }

  void refreshInflationSurface(bool local)
  {
    const auto & grid = local ? local_grid_ : global_grid_;
    auto & publisher = local ? local_inflation_ : global_inflation_;
    auto & dirty = local ? local_surface_dirty_ : global_surface_dirty_;
    auto & drawn = local ? local_surface_drawn_ : global_surface_drawn_;
    auto & last_published = local ? last_local_inflation_publish_ :
      last_global_inflation_publish_;
    if (!grid || !publisher) return;
    if (publisher->get_subscription_count() == 0) {
      drawn = false;
      dirty = true;
      return;
    }
    if (!dirty && drawn) return;
    const auto now = std::chrono::steady_clock::now();
    const auto minimum_period = local ? std::chrono::milliseconds(200) :
      bringup::StaticPcdRefreshState::kIncrementalPublishPeriod;
    if (last_published != std::chrono::steady_clock::time_point{} &&
      now - last_published < minimum_period)
    {
      return;
    }
    drawn = publishInflationSurface(*grid, local);
    dirty = !drawn;
  }

  bool publishInflationSurface(const nav_msgs::msg::OccupancyGrid & costmap, bool local)
  {
    const auto & publisher = local ? local_inflation_ : global_inflation_;
    if (!publisher || publisher->get_subscription_count() == 0) return false;
    tf2::Transform map_from_frame;
    map_from_frame.setIdentity();
    if (costmap.header.frame_id != frame_id_) {
      try {
        const auto transform = tf_buffer_.lookupTransform(
          frame_id_, costmap.header.frame_id, costmap.header.stamp,
          rclcpp::Duration::from_seconds(0.05));
        tf2::fromMsg(transform.transform, map_from_frame);
      } catch (const tf2::TransformException & error) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000, "Cannot draw costmap inflation: %s", error.what());
        return false;
      }
    }
    tf2::Transform frame_from_grid;
    tf2::fromMsg(costmap.info.origin, frame_from_grid);
    const auto map_from_grid = map_from_frame * frame_from_grid;
    const auto mesh_started = std::chrono::steady_clock::now();
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id_;
    marker.header.stamp = get_clock()->now();
    marker.ns = local ? "local_inflation" : "global_inflation";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = marker.scale.y = marker.scale.z = 1.0;
    marker.color.a = 1.0F;
    const float surface_alpha = local ? 1.0F : static_cast<float>(global_inflation_alpha_);
    const auto cells = bringup::inflationCells(
      costmap, local ? local_inflation_radius_ : global_inflation_radius_);
    marker.points.reserve(cells.size() * 6);
    marker.colors.reserve(cells.size() * 6);
    const double resolution = costmap.info.resolution;
    for (const auto & cell : cells) {
      std::array<geometry_msgs::msg::Point, 4> corners;
      const auto cost_color = bringup::inflationColor(cell.cost);
      std_msgs::msg::ColorRGBA color;
      color.r = cost_color.r;
      color.g = cost_color.g;
      color.b = cost_color.b;
      // RViz Humble enables alpha blending for TRIANGLE_LIST only when one of
      // its per-vertex colors has alpha below 1. The marker alpha alone leaves
      // the material in opaque mode when vertex colors are present.
      color.a = surface_alpha;
      for (int i = 0; i < 4; ++i) {
        const double x = (cell.x + (i == 1 || i == 2 ? 1.0 : 0.0)) * resolution;
        const double y = (cell.y + (i >= 2 ? 1.0 : 0.0)) * resolution;
        const auto point = map_from_grid * tf2::Vector3(x, y, 0.0);
        const auto ground = bringup::staticDisplayHeightAboveGround(
          map_, point.x(), point.y(), 0.0F, fallback_ground_);
        corners[i].x = point.x();
        corners[i].y = point.y();
        corners[i].z = (ground ? -*ground : fallback_ground_) + (local ? 0.06 : 0.04);
      }
      for (const int i : {0, 1, 2, 0, 2, 3}) {
        marker.points.push_back(corners[i]);
        marker.colors.push_back(color);
      }
    }
    if (marker.points.empty()) marker.action = visualization_msgs::msg::Marker::DELETE;
    visualization_msgs::msg::MarkerArray markers;
    markers.markers.push_back(std::move(marker));
    const auto mesh_finished = std::chrono::steady_clock::now();
    const auto publish_started = std::chrono::steady_clock::now();
    publisher->publish(markers);
    const auto finished_at = std::chrono::steady_clock::now();
    auto & last_published = local ? last_local_inflation_publish_ :
      last_global_inflation_publish_;
    const double interval_ms = last_published == std::chrono::steady_clock::time_point{} ? 0.0 :
      std::chrono::duration<double, std::milli>(finished_at - last_published).count();
    last_published = finished_at;
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "%s inflation refresh: cells=%zu mesh=%.3f ms publish=%.3f ms interval=%.1f ms",
      local ? "Local" : "Global", cells.size(),
      std::chrono::duration<double, std::milli>(mesh_finished - mesh_started).count(),
      std::chrono::duration<double, std::milli>(finished_at - publish_started).count(),
      interval_ms);
    return true;
  }

  void logLocalCostmapCounts(const nav_msgs::msg::OccupancyGrid & costmap)
  {
    if (costmap.info.width == 0 || costmap.info.height == 0 ||
      costmap.data.size() != static_cast<std::size_t>(costmap.info.width) * costmap.info.height)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Cannot count local costmap cells: invalid grid geometry");
      return;
    }

    std::size_t unknown_cells = 0;
    std::size_t free_cells = 0;
    std::size_t soft_cost_cells = 0;
    std::size_t hard_cost_cells = 0;
    for (const auto cost : costmap.data) {
      if (cost < 0) {
        ++unknown_cells;
      } else if (cost == 0) {
        ++free_cells;
      } else if (cost < 100) {
        ++soft_cost_cells;
      } else {
        ++hard_cost_cells;
      }
    }
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Local costmap cells: unknown=%zu free=%zu soft=%zu hard=%zu",
      unknown_cells, free_cells, soft_cost_cells, hard_cost_cells);
  }

  std::string frame_id_;
  double global_inflation_radius_{0.38};
  double local_inflation_radius_{0.25};
  double global_inflation_alpha_{0.25};
  double local_cloud_radius_{5.0};
  double local_cloud_fallback_ground_height_{0.25};
  double local_cloud_timeout_{1.0};
  double clearance_ray_min_range_{0.5};
  double clearance_ray_max_range_{20.0};
  double clearance_ray_height_resolution_{0.10};
  int clearance_ray_observations_{3};
  bool diagnostic_cell_enabled_{false};
  double diagnostic_cell_x_{0.0};
  double diagnostic_cell_y_{0.0};
  double local_ground_voxel_size_{0.12};
  float local_max_height_above_ground_{1.2F};
  fastdem::ElevationMap map_;
  fastdem::ElevationMap static_ground_map_;
  float fallback_ground_{0.0F};
  std::optional<bringup::StaticPcdClearanceTracker> static_pcd_clearance_tracker_;
  std::optional<GridCellKey> diagnostic_cell_;
  std::optional<nav_msgs::msg::OccupancyGrid> global_grid_;
  std::optional<nav_msgs::msg::OccupancyGrid> local_grid_;
  std::optional<nav_msgs::msg::OccupancyGrid> latest_navigation_map_;
  std::optional<nav_msgs::msg::OccupancyGrid> latest_full_navigation_map_;
  std::optional<nav_msgs::msg::OccupancyGrid> latest_occupancy_source_;
  std::map<std::uint64_t, nav_msgs::msg::OccupancyGrid::ConstSharedPtr> pending_full_maps_;
  std::map<std::uint64_t, bringup::MapGeometryEvent> pending_full_geometry_events_;
  std::map<std::uint64_t, bringup::MapGeometryEvent> pending_map_update_geometry_;
  std::deque<PendingMapUpdate> pending_map_updates_;
  std::set<std::uint64_t> ambiguous_full_stamps_;
  std::set<std::uint64_t> ambiguous_map_update_stamps_;
  std::uint64_t map_publisher_epoch_{0};
  std::uint64_t map_geometry_version_{0};
  std::uint64_t latest_full_map_stamp_ns_{0};
  std::uint64_t latest_navigation_map_stamp_ns_{0};
  bool map_delta_chain_valid_{false};
  bool startup_prior_clearance_applied_{false};
  double static_pcd_base_origin_x_{0.0};
  double static_pcd_base_origin_y_{0.0};
  double static_pcd_map_resolution_{0.0};
  std::vector<ColoredDisplayPoint> pending_structure_points_;
  std::vector<StaticStructureTile> static_structure_tiles_;
  std::unordered_set<std::size_t> dirty_static_structure_tiles_;
  std::unordered_map<GridCellKey, std::vector<StaticStructurePointRef>, GridCellKeyHash>
    structure_points_by_cell_;
  std::unordered_set<GridCellKey, GridCellKeyHash> clearance_candidate_cells_;
  std::size_t hidden_structure_point_count_{0};
  std::uint64_t diagnostic_map_free_stamp_ns_{0};
  std::uint64_t diagnostic_last_map_free_stamp_ns_{0};
  std::uint32_t diagnostic_last_max_confirmations_{0};
  std::size_t diagnostic_last_structure_points_{0};
  std::size_t diagnostic_last_hidden_points_{0};
  bool diagnostic_state_initialized_{false};
  bool diagnostic_display_pending_{false};
  grid_map_msgs::msg::GridMap::ConstSharedPtr latest_online_terrain_message_;
  bool online_terrain_dirty_{false};
  bool terrain_map_dirty_{false};
  bool global_surface_drawn_{false};
  bool local_surface_drawn_{false};
  bringup::StaticPcdRefreshState static_structure_refresh_state_;
  std::uint64_t subscriber_identity_fallback_epoch_{0};
  bool effective_obstacles_drawn_{false};
  bool effective_obstacles_dirty_{true};
  bool global_surface_dirty_{true};
  bool local_surface_dirty_{true};
  std::chrono::steady_clock::time_point last_terrain_map_publish_{};
  std::chrono::steady_clock::time_point last_online_terrain_merge_{};
  std::chrono::steady_clock::time_point last_effective_obstacles_publish_{};
  std::chrono::steady_clock::time_point last_global_inflation_publish_{};
  std::chrono::steady_clock::time_point last_local_inflation_publish_{};
  std::string display_pcd_file_;
  std::string display_ground_cache_file_;
  std::string display_structure_cache_file_;
  std::string display_topic_;
  double ground_display_voxel_size_{0.12};
  double max_display_height_{1.2};
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr display_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr local_ground_cloud_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr local_structure_cloud_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr static_structure_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr global_inflation_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr local_inflation_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr effective_obstacles_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr global_costmap_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr local_costmap_;
  rclcpp::Subscription<map_msgs::msg::OccupancyGridUpdate>::SharedPtr global_updates_;
  rclcpp::Subscription<map_msgs::msg::OccupancyGridUpdate>::SharedPtr local_updates_;
  rclcpp::Subscription<map_msgs::msg::OccupancyGridUpdate>::SharedPtr map_updates_subscription_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscription_;
  rclcpp::Subscription<std_msgs::msg::UInt64MultiArray>::SharedPtr map_geometry_subscription_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_source_subscription_;
  rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr online_terrain_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr local_cloud_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr clearance_cloud_subscription_;
  rclcpp::CallbackGroup::SharedPtr local_cloud_callback_group_;
  rclcpp::TimerBase::SharedPtr local_cloud_timeout_timer_;
  rclcpp::TimerBase::SharedPtr inflation_retry_timer_;
  rclcpp::TimerBase::SharedPtr display_load_timer_;
  std::chrono::steady_clock::time_point last_local_cloud_received_{};
  bool have_local_cloud_{false};
  bool local_cloud_visible_{false};
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<StaticTerrainMapPublisher>();
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
    executor.add_node(node);
    executor.spin();
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("static_terrain_map_publisher"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
