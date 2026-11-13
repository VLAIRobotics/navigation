#include <algorithm>
#include <chrono>
#include <cctype>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <functional>
#include <fastdem/elevation_map.hpp>
#include <fastdem/io/npz.hpp>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <map_msgs/msg/occupancy_grid_update.hpp>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rmw/qos_profiles.h>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <nanogrid/bridge/ros2.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/u_int64_multi_array.hpp>
#include <std_srvs/srv/trigger.hpp>
#include "bringup/consecutive_observation.hpp"
#include "bringup/footprint_cells.hpp"
#include "bringup/fresh_retry_queue.hpp"
#include "bringup/ground_provenance.hpp"
#include "bringup/map_geometry_event.hpp"
#include "bringup/map_fusion_update.hpp"
#include "bringup/map_snapshot_revision.hpp"
#include "bringup/obstacle_evidence_sequence.hpp"
#include "bringup/occupancy_source.hpp"
#include "bringup/occupancy_grid_growth.hpp"
#include "bringup/occupancy_raytrace.hpp"

#include <tf2/LinearMath/Transform.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class GlobalMapFusionNode : public rclcpp::Node
{
  using Cloud = sensor_msgs::msg::PointCloud2;
  using ObstacleRayFrame = std::pair<Cloud::ConstSharedPtr, Cloud::ConstSharedPtr>;
  using GroundVetoSyncPolicy = message_filters::sync_policies::ExactTime<Cloud, Cloud>;
  using ObstacleRaySyncPolicy = message_filters::sync_policies::ExactTime<Cloud, Cloud>;
  using MapPoint = std::pair<double, double>;
  using Cell = std::pair<std::int64_t, std::int64_t>;
  using MapGeometryEventKey = std::pair<std::uint64_t, std::uint64_t>;

  struct MapUpdatePublication
  {
    map_msgs::msg::OccupancyGridUpdate update;
    std_msgs::msg::UInt64MultiArray geometry_event;
  };

  struct ObstacleHitFrame
  {
    Cloud::ConstSharedPtr cloud;
    bringup::ObstacleEvidenceFrameMetadata metadata;
  };

  struct PendingGlobalClearEvent
  {
    Cloud::ConstSharedPtr cloud;
    bringup::ObstacleEvidenceFrameMetadata metadata;
    std::uint64_t event_id{0};
    bool diagnostic_target_checked{false};
    bool diagnostic_targets_cell{false};
  };

  struct LiveTerrainSnapshot
  {
    nanogrid::GridMap map;
    tf2::Transform terrain_from_map;
    std::int64_t stamp_ns{0};
    std::chrono::steady_clock::time_point received_at;
  };

  static std::uint64_t makeMapGeometryPublisherEpoch()
  {
    const auto epoch_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
    return epoch_ns > 0 ? static_cast<std::uint64_t>(epoch_ns) : 1U;
  }

  static std::vector<std::uint8_t> readSourcePgm(
    const std::string & path, const std::uint32_t expected_width,
    const std::uint32_t expected_height)
  {
    std::ifstream input(path, std::ios::binary);
    std::string magic;
    std::uint32_t width = 0;
    std::uint32_t height = 0;
    unsigned int maximum = 0;
    if (!input || !(input >> magic >> width >> height >> maximum) || magic != "P5" ||
      width != expected_width || height != expected_height || maximum != 255U)
    {
      throw std::runtime_error("saved source raster has an invalid PGM header or geometry: " + path);
    }
    char separator = '\0';
    if (!input.get(separator) || !std::isspace(static_cast<unsigned char>(separator))) {
      throw std::runtime_error("saved source raster has no PGM binary separator: " + path);
    }
    if (separator == '\r' && input.peek() == '\n') input.get();
    const auto cell_count = static_cast<std::size_t>(width) * height;
    std::vector<std::uint8_t> pixels(cell_count);
    input.read(reinterpret_cast<char *>(pixels.data()), static_cast<std::streamsize>(cell_count));
    if (input.gcount() != static_cast<std::streamsize>(cell_count)) {
      throw std::runtime_error("saved source raster is truncated: " + path);
    }
    return pixels;
  }

  static std::uint8_t decodeGroundSourcePixel(const std::uint8_t pixel)
  {
    if (pixel == 0U) return 0U;
    if (pixel == 255U) return 1U;
    throw std::runtime_error("ground source raster contains an unsupported source value");
  }

  static std::uint8_t decodeOccupancySourcePixel(const std::uint8_t pixel)
  {
    const auto mask = bringup::decodeOccupancySourcePixel(pixel);
    if (mask) return *mask;
    throw std::runtime_error("occupancy source raster contains an unsupported bit mask");
  }

  static void validateSourceRasterMetadata(
    const std::string & pgm_path, const nav_msgs::msg::OccupancyGrid & map)
  {
    auto yaml_path = std::filesystem::path(pgm_path);
    yaml_path.replace_extension(".yaml");
    std::ifstream input(yaml_path);
    if (!input) {
      throw std::runtime_error("saved source raster metadata is missing: " + yaml_path.string());
    }
    bool have_frame = false;
    bool have_resolution = false;
    bool have_origin = false;
    std::string frame;
    double resolution = 0.0;
    double origin_x = 0.0;
    double origin_y = 0.0;
    double origin_yaw = 0.0;
    std::string line;
    while (std::getline(input, line)) {
      if (line.rfind("frame:", 0) == 0) {
        frame = line.substr(6);
        frame.erase(0, frame.find_first_not_of(" \t"));
        have_frame = !frame.empty();
      } else if (line.rfind("resolution:", 0) == 0) {
        try {
          resolution = std::stod(line.substr(11));
          have_resolution = std::isfinite(resolution);
        } catch (const std::exception &) {
          have_resolution = false;
        }
      } else if (line.rfind("origin:", 0) == 0) {
        std::istringstream values(line.substr(7));
        char open = '\0';
        char first_comma = '\0';
        char second_comma = '\0';
        char close = '\0';
        have_origin = static_cast<bool>(
          values >> open >> origin_x >> first_comma >> origin_y >> second_comma >> origin_yaw >> close) &&
          open == '[' && first_comma == ',' && second_comma == ',' && close == ']' &&
          std::isfinite(origin_x) && std::isfinite(origin_y) && std::isfinite(origin_yaw);
      }
    }
    constexpr double kTolerance = 1.0e-5;
    if (!have_frame || !have_resolution || !have_origin || frame != map.header.frame_id ||
      std::abs(resolution - map.info.resolution) > kTolerance ||
      std::abs(origin_x - map.info.origin.position.x) > kTolerance ||
      std::abs(origin_y - map.info.origin.position.y) > kTolerance ||
      std::abs(origin_yaw - tf2::getYaw(map.info.origin.orientation)) > kTolerance)
    {
      throw std::runtime_error("saved source raster metadata does not match the map version");
    }
  }

  static void validateTerrainGeometry(
    const nav_msgs::msg::OccupancyGrid & map, const nanogrid::GridMap & terrain)
  {
    constexpr double kTolerance = 1.0e-5;
    const auto & orientation = map.info.origin.orientation;
    const double length_x = static_cast<double>(map.info.width) * map.info.resolution;
    const double length_y = static_cast<double>(map.info.height) * map.info.resolution;
    const double center_x = map.info.origin.position.x + length_x * 0.5;
    const double center_y = map.info.origin.position.y + length_y * 0.5;
    if (map.info.width > static_cast<std::uint32_t>(std::numeric_limits<int>::max()) ||
      map.info.height > static_cast<std::uint32_t>(std::numeric_limits<int>::max()) ||
      !std::isfinite(map.info.origin.position.x) || !std::isfinite(map.info.origin.position.y) ||
      !std::isfinite(map.info.origin.position.z) || !std::isfinite(orientation.x) ||
      !std::isfinite(orientation.y) || !std::isfinite(orientation.z) ||
      !std::isfinite(orientation.w) || std::abs(map.info.origin.position.z) > kTolerance ||
      std::abs(orientation.x) > kTolerance || std::abs(orientation.y) > kTolerance ||
      std::abs(orientation.z) > kTolerance || std::abs(std::abs(orientation.w) - 1.0) > kTolerance ||
      terrain.getFrameId() != map.header.frame_id ||
      terrain.getSize()(0) != static_cast<int>(map.info.width) ||
      terrain.getSize()(1) != static_cast<int>(map.info.height) ||
      std::abs(terrain.getResolution() - map.info.resolution) > kTolerance ||
      std::abs(terrain.getLength()(0) - length_x) > kTolerance ||
      std::abs(terrain.getLength()(1) - length_y) > kTolerance ||
      std::abs(terrain.getPosition().x() - center_x) > kTolerance ||
      std::abs(terrain.getPosition().y() - center_y) > kTolerance)
    {
      throw std::runtime_error(
        "saved terrain NPZ geometry does not match the loaded OccupancyGrid version");
    }
  }

  static constexpr std::int64_t kMaxObstacleEvidenceGapNs = 150000000;
  static constexpr std::int64_t kGroundEvidenceWindowNs = 2000000000;
  static constexpr double kMapGrowthQuantumMeters = 1.0;
  static constexpr std::chrono::milliseconds kGlobalClearRetryPeriod{100};
  static constexpr std::chrono::milliseconds kObstacleEvidenceFenceTimeout{500};
  static constexpr std::chrono::seconds kGlobalClearEvidenceMaxAge{2};
  static constexpr std::size_t kGlobalClearEventQueueCapacity = 10;
  static constexpr std::size_t kProcessedObstacleFrameCapacity = 64;
  static constexpr std::size_t kMaxConfirmedSequenceSpan = 256;

public:
  GlobalMapFusionNode()
  : Node("global_map_fusion"),
    map_geometry_publisher_epoch_(makeMapGeometryPublisherEpoch()),
    tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
  {
    prior_map_topic_ = declare_parameter<std::string>("prior_map_topic", "/map_prior");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    initialize_without_prior_map_ = declare_parameter<bool>(
      "initialize_without_prior_map", false);
    initial_map_resolution_ = declare_parameter<double>("initial_map_resolution", 0.10);
    map_topic_ = declare_parameter<std::string>("map_topic", "/map");
    updates_topic_ = declare_parameter<std::string>("updates_topic", "/map_updates");
    ground_topic_ = declare_parameter<std::string>("ground_topic", "/patchworkpp/ground");
    global_clearing_topic_ = declare_parameter<std::string>(
      "global_clearing_topic", "/obstacles/global_clearing");
    ground_veto_topic_ = declare_parameter<std::string>(
      "ground_veto_topic", "/obstacles/ground_veto");
    ground_veto_fence_service_name_ = declare_parameter<std::string>(
      "ground_veto_fence_service", "/obstacles/ground_veto_fence");
    clearing_ray_topic_ = declare_parameter<std::string>(
      "clearing_ray_topic", "/cloud_registered_body");
    health_topic_ = declare_parameter<std::string>("health_topic", "/fastlio/health");
    live_terrain_topic_ = declare_parameter<std::string>("live_terrain_topic", "");
    require_live_terrain_ = declare_parameter<bool>("require_live_terrain", false);
    live_terrain_timeout_ = declare_parameter<double>("live_terrain_timeout", 1.5);
    live_terrain_max_lag_ = declare_parameter<double>("live_terrain_max_lag", 0.5);
    live_terrain_future_tolerance_ = declare_parameter<double>(
      "live_terrain_future_tolerance", 0.1);
    live_terrain_min_points_ = declare_parameter<int>("live_terrain_min_points", 3);
    live_terrain_max_observation_age_ = declare_parameter<int>(
      "live_terrain_max_observation_age", 3);
    ground_max_age_ = declare_parameter<double>("ground_max_age", 0.5);
    ground_future_tolerance_ = declare_parameter<double>("ground_future_tolerance", 0.1);
    free_observations_ = declare_parameter<int>("free_observations", 3);
    obstacle_confirmation_frames_ = declare_parameter<int>("obstacle_confirmation_frames", 2);
    fallback_obstacle_confirmation_frames_ = declare_parameter<int>(
      "fallback_obstacle_confirmation_frames", 5);
    clear_observations_ = declare_parameter<int>("clear_observations", 5);
    online_obstacle_clear_observations_ = declare_parameter<int>(
      "online_obstacle_clear_observations", 5);
    raytrace_min_range_ = declare_parameter<double>("raytrace_min_range", 0.5);
    raytrace_max_range_ = declare_parameter<double>("raytrace_max_range", 20.0);
    max_update_range_ = declare_parameter<double>("max_update_range", 20.0);
    max_map_extension_m_ = declare_parameter<double>("max_map_extension_m", 20.0);
    max_map_cells_ = declare_parameter<std::int64_t>("max_map_cells", 4000000);
    health_timeout_ = declare_parameter<double>("health_timeout", 0.50);
    tf_lookup_timeout_ = declare_parameter<double>("tf_lookup_timeout", 0.15);
    full_map_publish_period_ = declare_parameter<double>("full_map_publish_period", 1.0);
    terrain_map_file_ = declare_parameter<std::string>("terrain_map_file", "");
    ground_provenance_file_ = declare_parameter<std::string>("ground_provenance_file", "");
    occupancy_source_file_ = declare_parameter<std::string>("occupancy_source_file", "");
    footprint_clearing_enabled_ = declare_parameter<bool>("footprint_clearing_enabled", true);
    diagnostic_cell_enabled_ = declare_parameter<bool>("diagnostic_cell_enabled", false);
    diagnostic_cell_x_ = declare_parameter<double>("diagnostic_cell_x", 0.0);
    diagnostic_cell_y_ = declare_parameter<double>("diagnostic_cell_y", 0.0);
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    const auto footprint_vertices = declare_parameter<std::vector<double>>(
      "robot_footprint", {0.275, 0.225, 0.275, -0.225, -0.35, -0.225, -0.35, 0.225});
    if (map_frame_.empty() || !std::isfinite(initial_map_resolution_) ||
      initial_map_resolution_ <= 0.0 ||
      (require_live_terrain_ && live_terrain_topic_.empty()) ||
      !std::isfinite(live_terrain_timeout_) || live_terrain_timeout_ <= 0.0 ||
      !std::isfinite(live_terrain_max_lag_) || live_terrain_max_lag_ < 0.0 ||
      !std::isfinite(live_terrain_future_tolerance_) ||
      live_terrain_future_tolerance_ < 0.0 || live_terrain_min_points_ < 1 ||
      live_terrain_max_observation_age_ < 0 ||
      !std::isfinite(ground_max_age_) || ground_max_age_ <= 0.0 ||
      !std::isfinite(ground_future_tolerance_) || ground_future_tolerance_ < 0.0 ||
      free_observations_ < 1 || free_observations_ > 255 ||
      obstacle_confirmation_frames_ < 1 || obstacle_confirmation_frames_ > 255 ||
      fallback_obstacle_confirmation_frames_ < obstacle_confirmation_frames_ ||
      fallback_obstacle_confirmation_frames_ > 255 ||
      clear_observations_ < 1 || clear_observations_ > 255 ||
      online_obstacle_clear_observations_ < 1 || online_obstacle_clear_observations_ > 255 ||
      !std::isfinite(raytrace_min_range_) || raytrace_min_range_ < 0.0 ||
      !std::isfinite(raytrace_max_range_) || raytrace_max_range_ <= raytrace_min_range_ ||
      !std::isfinite(max_update_range_) || max_update_range_ <= 0.0 ||
      !std::isfinite(max_map_extension_m_) || max_map_extension_m_ < 0.0 ||
      max_map_cells_ < 1 || !std::isfinite(health_timeout_) || health_timeout_ <= 0.0 ||
      !std::isfinite(tf_lookup_timeout_) || tf_lookup_timeout_ <= 0.0 ||
      !std::isfinite(full_map_publish_period_) || full_map_publish_period_ <= 0.0 ||
      !std::isfinite(diagnostic_cell_x_) || !std::isfinite(diagnostic_cell_y_) ||
      base_frame_.empty() || footprint_vertices.size() < 6 ||
      footprint_vertices.size() % 2 != 0 ||
      !std::all_of(footprint_vertices.begin(), footprint_vertices.end(),
        [](double value) { return std::isfinite(value); }))
    {
      throw std::invalid_argument("global map fusion thresholds and timeouts are invalid");
    }
    robot_footprint_.reserve(footprint_vertices.size() / 2);
    double footprint_twice_area = 0.0;
    for (std::size_t i = 0; i < footprint_vertices.size(); i += 2) {
      robot_footprint_.push_back({footprint_vertices[i], footprint_vertices[i + 1]});
    }
    for (std::size_t i = 0; i < robot_footprint_.size(); ++i) {
      const auto & current = robot_footprint_[i];
      const auto & next = robot_footprint_[(i + 1) % robot_footprint_.size()];
      footprint_twice_area += current.x * next.y - next.x * current.y;
    }
    if (!std::isfinite(footprint_twice_area) || std::abs(footprint_twice_area) <= 1.0e-9) {
      throw std::invalid_argument("robot_footprint must enclose a non-zero area");
    }

    prior_map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      prior_map_topic_, rclcpp::QoS(1).reliable().transient_local(),
      std::bind(&GlobalMapFusionNode::priorMapCallback, this, std::placeholders::_1));
    map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
      map_topic_, rclcpp::QoS(1).reliable().transient_local());
    ground_provenance_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
      "/navigation/ground_provenance", rclcpp::QoS(1).reliable().transient_local());
    occupancy_source_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
      "/navigation/occupancy_source", rclcpp::QoS(1).reliable().transient_local());
    global_terrain_sub_ = create_subscription<grid_map_msgs::msg::GridMap>(
      "/navigation/global_terrain_map", rclcpp::QoS(1).reliable().transient_local(),
      std::bind(&GlobalMapFusionNode::globalTerrainCallback, this, std::placeholders::_1));
    updates_pub_ = create_publisher<map_msgs::msg::OccupancyGridUpdate>(
      updates_topic_, rclcpp::QoS(10).reliable());
    map_geometry_pub_ = create_publisher<std_msgs::msg::UInt64MultiArray>(
      "/map_geometry_events", rclcpp::QoS(64).reliable().transient_local());
    health_sub_ = create_subscription<std_msgs::msg::Bool>(
      health_topic_, rclcpp::QoS(1),
      std::bind(&GlobalMapFusionNode::healthCallback, this, std::placeholders::_1));
    if (!live_terrain_topic_.empty()) {
      live_terrain_sub_ = create_subscription<grid_map_msgs::msg::GridMap>(
        live_terrain_topic_, rclcpp::QoS(1).best_effort(),
        std::bind(&GlobalMapFusionNode::liveTerrainCallback, this, std::placeholders::_1));
    }
    obstacle_evidence_sub_ = create_subscription<Cloud>(
      ground_veto_topic_, rclcpp::QoS(10).reliable(),
      std::bind(&GlobalMapFusionNode::obstacleEvidenceCallback, this, std::placeholders::_1));
    ground_veto_fence_client_ = create_client<std_srvs::srv::Trigger>(
      ground_veto_fence_service_name_);
    obstacle_sub_.subscribe(this, ground_veto_topic_, rmw_qos_profile_sensor_data);
    clearing_ray_sub_.subscribe(this, clearing_ray_topic_, rmw_qos_profile_sensor_data);
    obstacle_ray_synchronizer_ =
      std::make_shared<message_filters::Synchronizer<ObstacleRaySyncPolicy>>(
      ObstacleRaySyncPolicy(10), obstacle_sub_, clearing_ray_sub_);
    obstacle_ray_synchronizer_->registerCallback(std::bind(
      &GlobalMapFusionNode::clearingRayCallback, this,
      std::placeholders::_1, std::placeholders::_2));
    global_clearing_sub_ = create_subscription<Cloud>(
      global_clearing_topic_, rclcpp::QoS(10).reliable(),
      std::bind(&GlobalMapFusionNode::globalClearingCallback, this, std::placeholders::_1));
    global_clear_retry_timer_ = create_wall_timer(
      kGlobalClearRetryPeriod,
      std::bind(&GlobalMapFusionNode::retryPendingGlobalClearEvents, this));
    snapshot_timer_ = create_wall_timer(
      std::chrono::milliseconds(std::max<int64_t>(
        1, static_cast<int64_t>(full_map_publish_period_ * 1000.0))),
      std::bind(&GlobalMapFusionNode::publishSnapshot, this));

    ground_sub_.subscribe(this, ground_topic_, rmw_qos_profile_sensor_data);
    ground_veto_sub_.subscribe(this, ground_veto_topic_, rmw_qos_profile_sensor_data);
    ground_synchronizer_ =
      std::make_shared<message_filters::Synchronizer<GroundVetoSyncPolicy>>(
      GroundVetoSyncPolicy(10), ground_sub_, ground_veto_sub_);
    ground_synchronizer_->registerCallback(std::bind(
      &GlobalMapFusionNode::groundCallback, this,
      std::placeholders::_1, std::placeholders::_2));

    if (initialize_without_prior_map_) {
      initializeWithoutPriorMap();
    }
  }

private:
  void priorMapCallback(nav_msgs::msg::OccupancyGrid::ConstSharedPtr message)
  {
    const auto cell_count = static_cast<std::uint64_t>(message->info.width) *
      static_cast<std::uint64_t>(message->info.height);
    const auto & orientation = message->info.origin.orientation;
    const double quaternion_norm = orientation.x * orientation.x +
      orientation.y * orientation.y + orientation.z * orientation.z +
      orientation.w * orientation.w;
    if (message->header.frame_id != map_frame_ || message->info.width == 0 ||
      message->info.height == 0 || !std::isfinite(message->info.resolution) ||
      message->info.resolution <= 0.0F ||
      !std::isfinite(message->info.origin.position.x) ||
      !std::isfinite(message->info.origin.position.y) ||
      !std::isfinite(message->info.origin.position.z) ||
      !std::isfinite(orientation.x) || !std::isfinite(orientation.y) ||
      !std::isfinite(orientation.z) || !std::isfinite(orientation.w) ||
      !std::isfinite(quaternion_norm) || quaternion_norm < 1.0e-12 ||
      message->info.width > static_cast<std::uint32_t>(std::numeric_limits<int>::max()) ||
      message->info.height > static_cast<std::uint32_t>(std::numeric_limits<int>::max()) ||
      cell_count > static_cast<std::uint64_t>(max_map_cells_) ||
      cell_count > std::numeric_limits<std::size_t>::max() ||
      message->data.size() != static_cast<std::size_t>(cell_count))
    {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Prior map must be a valid OccupancyGrid in frame '%s'; fusion remains inactive",
        map_frame_.c_str());
      return;
    }

    nav_msgs::msg::OccupancyGrid latched_map;
    nav_msgs::msg::OccupancyGrid latched_provenance;
    nav_msgs::msg::OccupancyGrid latched_occupancy_source;
    std::uint64_t latched_geometry_version = 0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (map_ready_) return;
      try {
        if (ground_provenance_file_.empty() != occupancy_source_file_.empty()) {
          throw std::runtime_error(
            "saved ground and occupancy source rasters must be loaded as one snapshot");
        }
        map_ = *message;
        map_geometry_version_ = 1;
        latched_geometry_version = map_geometry_version_;
        original_width_ = map_.info.width;
        original_height_ = map_.info.height;
        tf2::Transform map_from_grid;
        tf2::fromMsg(map_.info.origin, map_from_grid);
        grid_from_map_ = map_from_grid.inverse();
        obstacle_evidence_.assign(map_.data.size(), bringup::ConsecutiveObservation{});
        ray_clear_evidence_.assign(map_.data.size(), bringup::ConsecutiveObservation{});
        latest_obstacle_hit_stamp_ns_.assign(map_.data.size(), 0);
        ground_evidence_.assign(map_.data.size(), bringup::TimeWindowObservation{});
        prior_occupied_.resize(map_.data.size());
        online_occupied_.assign(map_.data.size(), 0);
        ground_provenance_.assign(map_.data.size(), 0);
        online_occupied_count_ = 0;
        if (ground_provenance_file_.empty()) {
          std::transform(
            map_.data.begin(), map_.data.end(), prior_occupied_.begin(),
            [](std::int8_t occupancy) { return static_cast<std::uint8_t>(occupancy >= 65); });
        } else {
          validateSourceRasterMetadata(ground_provenance_file_, map_);
          validateSourceRasterMetadata(occupancy_source_file_, map_);
          const auto ground_pixels = readSourcePgm(
            ground_provenance_file_, map_.info.width, map_.info.height);
          const auto occupancy_pixels = readSourcePgm(
            occupancy_source_file_, map_.info.width, map_.info.height);
          for (std::uint32_t image_row = 0; image_row < map_.info.height; ++image_row) {
            const auto grid_y = map_.info.height - 1U - image_row;
            for (std::uint32_t x = 0; x < map_.info.width; ++x) {
              const auto image_index =
                static_cast<std::size_t>(image_row) * map_.info.width + x;
              const auto grid_index =
                static_cast<std::size_t>(grid_y) * map_.info.width + x;
              ground_provenance_[grid_index] = static_cast<std::int8_t>(
                decodeGroundSourcePixel(ground_pixels[image_index]));
              const auto source_mask =
                decodeOccupancySourcePixel(occupancy_pixels[image_index]);
              const auto occupancy_sources = bringup::restoreOccupancySourceFlags(
                source_mask, map_.data[grid_index]);
              if (!occupancy_sources) {
                throw std::runtime_error(
                  "saved occupancy source mask is outside its encoding");
              }
              prior_occupied_[grid_index] =
                static_cast<std::uint8_t>(occupancy_sources->prior);
              online_occupied_[grid_index] =
                static_cast<std::uint8_t>(occupancy_sources->online);
              if (online_occupied_[grid_index]) ++online_occupied_count_;
            }
          }
        }
        if (!terrain_map_file_.empty()) {
          std::string terrain_error;
          if (!fastdem::io::loadNpz(terrain_map_file_, static_terrain_map_, &terrain_error)) {
            throw std::runtime_error(
              "invalid static terrain NPZ '" + terrain_map_file_ + "': " + terrain_error);
          }
          if (!static_terrain_map_.isInitialized() ||
            static_terrain_map_.getFrameId() != map_.header.frame_id)
          {
            throw std::runtime_error(
              "static terrain map is uninitialized or has a mismatched frame");
          }
          validateTerrainGeometry(map_, static_terrain_map_);
          if (!static_terrain_map_.exists("class")) {
            throw std::runtime_error("saved terrain map has no class layer");
          }
          const bool has_source = static_terrain_map_.exists("ground_source");
          for (const auto cell : static_terrain_map_.cells()) {
            const auto position = static_terrain_map_.position(cell);
            if (!position) {
              throw std::runtime_error("saved terrain map contains a cell without a position");
            }
            const auto map_index = mapCell(position->x(), position->y());
            if (!map_index) {
              throw std::runtime_error("saved terrain source cell lies outside the fused map");
            }
            const auto raw_class = static_terrain_map_.get("class")(cell.index);
            if (!std::isfinite(raw_class) || raw_class < -0.5F || raw_class > 2.5F) {
              throw std::runtime_error("saved terrain map contains an invalid class value");
            }
            const auto class_code = static_cast<std::int8_t>(std::lround(raw_class));
            if (std::abs(raw_class - class_code) > 1.0e-4F) {
              throw std::runtime_error("saved terrain map contains a fractional class value");
            }
            std::int8_t source = class_code == 1 ? 1 : 0;
            if (has_source) {
              const auto raw_source = static_terrain_map_.get("ground_source")(cell.index);
              if (!std::isfinite(raw_source) || raw_source < -0.5F || raw_source > 1.5F) {
                throw std::runtime_error("saved terrain map contains an invalid ground source");
              }
              source = static_cast<std::int8_t>(std::lround(raw_source));
              if (std::abs(raw_source - source) > 1.0e-4F) {
                throw std::runtime_error("saved terrain map contains a fractional ground source");
              }
            }
            if (ground_provenance_file_.empty()) {
              ground_provenance_[*map_index] = source;
            }
            if (!bringup::terrainGroundSourceMatches(
                class_code, source, ground_provenance_[*map_index]))
            {
              throw std::runtime_error(
                "saved terrain NPZ, class and ground-source raster are inconsistent");
            }
            if (source != 0 && map_.data[*map_index] < 0) {
              // Ground proven by the terrain revision is traversable in the
              // global occupancy map; the terrain costmap keeps its extra cost.
              map_.data[*map_index] = 0;
            }
          }
        }
        map_dirty_ = false;
        map_ready_ = true;
        map_.header.stamp = get_clock()->now();
        latched_map = map_;
        latched_provenance = makeGroundProvenanceSnapshotLocked();
        latched_occupancy_source = makeOccupancySourceSnapshotLocked();
      } catch (const std::exception & error) {
        map_ready_ = false;
        map_ = nav_msgs::msg::OccupancyGrid{};
        RCLCPP_ERROR(get_logger(), "Rejecting prior map: %s", error.what());
        return;
      }
    }
    publishFullMap(latched_map, latched_geometry_version);
    ground_provenance_pub_->publish(latched_provenance);
    occupancy_source_pub_->publish(latched_occupancy_source);
    RCLCPP_INFO(
      get_logger(), "Loaded prior map %ux%u at %.3f m; ground fusion uses synchronized "
      "per-scan blocking vetoes; map growth is capped at %.1f m and %ld cells",
      latched_map.info.width, latched_map.info.height, latched_map.info.resolution,
      max_map_extension_m_, static_cast<long>(max_map_cells_));
  }

  void initializeWithoutPriorMap()
  {
    nav_msgs::msg::OccupancyGrid initial_map;
    nav_msgs::msg::OccupancyGrid provenance;
    nav_msgs::msg::OccupancyGrid occupancy_source;
    std::uint64_t geometry_version = 0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (map_ready_) return;
      map_.header.frame_id = map_frame_;
      map_.header.stamp = get_clock()->now();
      map_.info.resolution = static_cast<float>(initial_map_resolution_);
      map_.info.width = 1;
      map_.info.height = 1;
      map_.info.origin.position.x = -initial_map_resolution_ * 0.5;
      map_.info.origin.position.y = -initial_map_resolution_ * 0.5;
      map_.info.origin.orientation.w = 1.0;
      map_.data.assign(1, static_cast<std::int8_t>(-1));
      original_width_ = 1;
      original_height_ = 1;
      map_origin_offset_x_ = 0;
      map_origin_offset_y_ = 0;
      tf2::Transform map_from_grid;
      tf2::fromMsg(map_.info.origin, map_from_grid);
      grid_from_map_ = map_from_grid.inverse();
      obstacle_evidence_.assign(1, bringup::ConsecutiveObservation{});
      ray_clear_evidence_.assign(1, bringup::ConsecutiveObservation{});
      latest_obstacle_hit_stamp_ns_.assign(1, 0);
      ground_evidence_.assign(1, bringup::TimeWindowObservation{});
      prior_occupied_.assign(1, 0);
      online_occupied_.assign(1, 0);
      ground_provenance_.assign(1, 0);
      online_occupied_count_ = 0;
      map_geometry_version_ = 1;
      geometry_version = map_geometry_version_;
      map_ready_ = true;
      map_dirty_ = false;
      initial_map = map_;
      provenance = makeGroundProvenanceSnapshotLocked();
      occupancy_source = makeOccupancySourceSnapshotLocked();
    }
    publishFullMap(initial_map, geometry_version);
    ground_provenance_pub_->publish(provenance);
    occupancy_source_pub_->publish(occupancy_source);
    RCLCPP_INFO(
      get_logger(), "Initialized unknown online map in %s at %.3f m; growth capped at %.1f m "
      "and %ld cells",
      map_frame_.c_str(), initial_map_resolution_, max_map_extension_m_,
      static_cast<long>(max_map_cells_));
  }

  void liveTerrainCallback(grid_map_msgs::msg::GridMap::ConstSharedPtr message)
  {
    if (!message || message->header.frame_id.empty()) return;
    const auto source_stamp = rclcpp::Time(message->header.stamp, get_clock()->get_clock_type());
    if (source_stamp.nanoseconds() <= 0) return;
    nanogrid::GridMap terrain;
    try {
      terrain = nanogrid::ros2::fromMsg(*message);
    } catch (const std::exception & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Ignoring invalid live FastDEM snapshot: %s",
        error.what());
      return;
    }
    if (!terrain.exists("elevation") || !terrain.exists("n_points") ||
      !terrain.exists("observation_age"))
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Ignoring live FastDEM snapshot without elevation, n_points, and observation_age");
      return;
    }

    tf2::Transform terrain_from_map;
    terrain_from_map.setIdentity();
    if (message->header.frame_id != map_frame_) {
      try {
        const auto transform = tf_buffer_.lookupTransform(
          message->header.frame_id, map_frame_, source_stamp,
          rclcpp::Duration::from_seconds(tf_lookup_timeout_));
        tf2::fromMsg(transform.transform, terrain_from_map);
      } catch (const tf2::TransformException & error) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Ignoring live FastDEM snapshot without %s -> %s transform: %s",
          map_frame_.c_str(), message->header.frame_id.c_str(), error.what());
        return;
      }
    }

    auto snapshot = std::make_shared<LiveTerrainSnapshot>();
    snapshot->map = std::move(terrain);
    snapshot->terrain_from_map = terrain_from_map;
    snapshot->stamp_ns = source_stamp.nanoseconds();
    snapshot->received_at = std::chrono::steady_clock::now();
    std::lock_guard<std::mutex> lock(live_terrain_mutex_);
    if (live_terrain_snapshot_ &&
      snapshot->stamp_ns <= live_terrain_snapshot_->stamp_ns)
    {
      return;
    }
    live_terrain_snapshot_ = std::move(snapshot);
  }

  bool liveTerrainConfidentAt(
    const double map_x, const double map_y, const std::int64_t evidence_stamp_ns)
  {
    if (!require_live_terrain_) return true;
    std::lock_guard<std::mutex> lock(live_terrain_mutex_);
    if (!live_terrain_snapshot_) return false;
    const auto now = get_clock()->now();
    const auto terrain_stamp = rclcpp::Time(
      live_terrain_snapshot_->stamp_ns, get_clock()->get_clock_type());
    const double source_age = (now - terrain_stamp).seconds();
    const double receive_age = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - live_terrain_snapshot_->received_at).count();
    const double terrain_lag = static_cast<double>(
      evidence_stamp_ns - live_terrain_snapshot_->stamp_ns) * 1.0e-9;
    if (!std::isfinite(source_age) || source_age > live_terrain_timeout_ ||
      source_age < -live_terrain_future_tolerance_ || !std::isfinite(receive_age) ||
      receive_age > live_terrain_timeout_ || terrain_lag > live_terrain_max_lag_ ||
      terrain_lag < -live_terrain_future_tolerance_)
    {
      return false;
    }
    const auto terrain_position = live_terrain_snapshot_->terrain_from_map *
      tf2::Vector3(map_x, map_y, 0.0);
    const auto index = live_terrain_snapshot_->map.index(
      nanogrid::Position(terrain_position.x(), terrain_position.y()));
    if (!index) return false;
    const auto elevation = live_terrain_snapshot_->map.at("elevation", *index);
    const auto points = live_terrain_snapshot_->map.at("n_points", *index);
    const auto observation_age = live_terrain_snapshot_->map.at("observation_age", *index);
    return std::isfinite(elevation) && std::isfinite(points) &&
           points >= static_cast<float>(live_terrain_min_points_) &&
           std::isfinite(observation_age) &&
           observation_age <= static_cast<float>(live_terrain_max_observation_age_);
  }

  void healthCallback(std_msgs::msg::Bool::ConstSharedPtr message)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    healthy_ = message->data;
    last_health_ = std::chrono::steady_clock::now();
    have_health_ = true;
  }

  void globalTerrainCallback(grid_map_msgs::msg::GridMap::ConstSharedPtr message)
  {
    if (!message || message->header.stamp.sec < 0 ||
      message->header.stamp.nanosec >= 1000000000U ||
      (message->header.stamp.sec == 0 && message->header.stamp.nanosec == 0U))
    {
      return;
    }
    nanogrid::GridMap terrain;
    try {
      terrain = nanogrid::ros2::fromMsg(*message);
    } catch (const std::exception & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Ignoring invalid global terrain snapshot: %s",
        error.what());
      return;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    if (!map_ready_) return;
    try {
      validateTerrainGeometry(map_, terrain);
    } catch (const std::exception & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Ignoring mismatched global terrain snapshot: %s",
        error.what());
      return;
    }
    const auto terrain_stamp_ns = static_cast<std::int64_t>(terrain.getTimestamp());
    if (terrain_stamp_ns <= last_global_terrain_stamp_ns_) return;

    const bool has_source = terrain.exists("ground_source");
    const bool has_class = terrain.exists("class");
    if (!has_source && !has_class) return;
    std::vector<std::pair<std::size_t, std::int8_t>> source_updates;
    source_updates.reserve(static_cast<std::size_t>(terrain.getSize().prod()));
    for (const auto cell : terrain.cells()) {
      const auto position = terrain.position(cell);
      if (!position) continue;
      const auto map_index = mapCell(position->x(), position->y());
      if (!map_index) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Ignoring global terrain snapshot with cells outside the fused map");
        return;
      }
      std::int8_t source = 0;
      std::int8_t class_code = 0;
      if (has_class) {
        const auto value = terrain.get("class")(cell.index);
        if (!std::isfinite(value) || value < -0.5F || value > 2.5F) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Ignoring global terrain snapshot with invalid class values");
          return;
        }
        class_code = static_cast<std::int8_t>(std::lround(value));
        if (std::abs(value - class_code) > 1.0e-4F) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Ignoring global terrain snapshot with fractional class values");
          return;
        }
      }
      if (has_source) {
        const auto value = terrain.get("ground_source")(cell.index);
        if (!std::isfinite(value) || value < -0.5F || value > 1.5F) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Ignoring global terrain snapshot with invalid ground_source values");
          return;
        }
        source = static_cast<std::int8_t>(std::lround(value));
        if (std::abs(value - source) > 1.0e-4F) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Ignoring global terrain snapshot with fractional ground_source values");
          return;
        }
      } else if (has_class) {
        source = class_code == 1 ? 1 : 0;
      }
      if (has_class && !bringup::terrainGroundSourceMatches(class_code, source, source)) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Ignoring global terrain snapshot with mismatched class and ground source");
        return;
      }
      source_updates.emplace_back(*map_index, source);
    }
    last_global_terrain_stamp_ns_ = terrain_stamp_ns;
    bool changed = false;
    for (const auto & [map_index, source] : source_updates) {
      if (ground_provenance_[map_index] != source) {
        ground_provenance_[map_index] = source;
        changed = true;
      }
      if (source != 0 && map_.data[map_index] < 0) {
        map_.data[map_index] = 0;
        changed = true;
      }
    }
    if (changed) {
      const bool map_was_already_dirty = map_dirty_;
      map_dirty_ = true;
      const auto current_map_stamp_ns =
        static_cast<std::int64_t>(map_.header.stamp.sec) * 1000000000LL +
        map_.header.stamp.nanosec;
      map_revision_from_terrain_ = !map_was_already_dirty &&
        terrain_stamp_ns > current_map_stamp_ns;
      if (map_revision_from_terrain_) {
        map_.header.stamp = message->header.stamp;
      }
    }
  }

  void publishSnapshot()
  {
    nav_msgs::msg::OccupancyGrid snapshot;
    nav_msgs::msg::OccupancyGrid provenance_snapshot;
    nav_msgs::msg::OccupancyGrid occupancy_source_snapshot;
    std::uint64_t snapshot_geometry_version = 0;
    bool publish_map = false;
    bool publish_stats = false;
    std::size_t occupied_cleared = 0;
    std::size_t online_occupied_cleared = 0;
    std::size_t online_occupied_total = 0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!map_ready_) return;
      if (map_dirty_) {
        map_dirty_ = false;
        if (!map_revision_from_terrain_) {
          map_.header.stamp = bringup::nextMapSnapshotRevision(
            map_.header.stamp,
            bringup::mapSnapshotTimeFromNanoseconds(get_clock()->now().nanoseconds()));
        }
        map_revision_from_terrain_ = false;
        snapshot = map_;
        snapshot_geometry_version = map_geometry_version_;
        publish_map = true;
        provenance_snapshot = makeGroundProvenanceSnapshotLocked();
        occupancy_source_snapshot = makeOccupancySourceSnapshotLocked();
      }
      const auto now = std::chrono::steady_clock::now();
      if (now - last_fusion_stats_log_ >= std::chrono::seconds(5)) {
        occupied_cleared = pending_occupied_cleared_;
        online_occupied_cleared = pending_online_occupied_cleared_;
        online_occupied_total = online_occupied_count_;
        pending_occupied_cleared_ = 0;
        pending_online_occupied_cleared_ = 0;
        last_fusion_stats_log_ = now;
        publish_stats = true;
      }
    }
    if (publish_map) {
      publishFullMap(snapshot, snapshot_geometry_version);
      ground_provenance_pub_->publish(provenance_snapshot);
      occupancy_source_pub_->publish(occupancy_source_snapshot);
    }
    if (publish_stats) {
      RCLCPP_INFO(
        get_logger(),
        "Global map fusion: occupied_cleared=%zu online_occupied_cleared=%zu "
        "online_occupied_total=%zu",
        occupied_cleared, online_occupied_cleared, online_occupied_total);
    }
  }

  nav_msgs::msg::OccupancyGrid makeGroundProvenanceSnapshotLocked() const
  {
    auto message = map_;
    message.data = ground_provenance_;
    message.header.stamp = map_.header.stamp;
    return message;
  }

  nav_msgs::msg::OccupancyGrid makeOccupancySourceSnapshotLocked() const
  {
    auto message = map_;
    message.data.resize(map_.data.size(), 0);
    for (std::size_t index = 0; index < map_.data.size(); ++index) {
      const auto prior = prior_occupied_[index] != 0 ? 1 : 0;
      const auto online = online_occupied_[index] != 0 ? 2 : 0;
      message.data[index] = static_cast<std::int8_t>(prior | online);
    }
    message.header.stamp = map_.header.stamp;
    return message;
  }

  bool localizationHealthy() const
  {
    return have_health_ && healthy_ &&
           std::chrono::duration<double>(std::chrono::steady_clock::now() - last_health_).count() <=
           health_timeout_;
  }

  bool lookupMapTransform(const Cloud & cloud, tf2::Transform & transform)
  {
    if (cloud.header.frame_id.empty()) return false;
    std::string source_frame_id = cloud.header.frame_id;
    bringup::ObstacleEvidenceFrameMetadata clear_metadata;
    (void)bringup::parseObstacleClearEventFrameId(
      cloud.header.frame_id, source_frame_id, clear_metadata);
    try {
      const auto stamped = tf_buffer_.lookupTransform(
        map_frame_, source_frame_id, rclcpp::Time(cloud.header.stamp),
        rclcpp::Duration::from_seconds(tf_lookup_timeout_));
      tf2::fromMsg(stamped.transform, transform);
      return true;
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Skipping global map evidence without map TF: %s",
        error.what());
      return false;
    }
  }

  bool lookupMapBaseTransform(const Cloud & cloud, tf2::Transform & transform)
  {
    try {
      const auto stamped = tf_buffer_.lookupTransform(
        map_frame_, base_frame_, rclcpp::Time(cloud.header.stamp),
        rclcpp::Duration::from_seconds(tf_lookup_timeout_));
      tf2::fromMsg(stamped.transform, transform);
      return true;
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Skipping footprint clearing without map-to-%s TF: %s",
        base_frame_.c_str(), error.what());
      return false;
    }
  }

  std::vector<MapPoint> mapPoints(
    const Cloud & cloud, const tf2::Transform & map_from_cloud,
    bool enforce_max_update_range = true) const
  {
    std::vector<MapPoint> points;
    const auto sensor_origin = map_from_cloud.getOrigin();
    const double max_range_squared = max_update_range_ * max_update_range_;
    sensor_msgs::PointCloud2ConstIterator<float> x(cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> y(cloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float> z(cloud, "z");
    for (; x != x.end(); ++x, ++y, ++z) {
      if (!std::isfinite(*x) || !std::isfinite(*y) || !std::isfinite(*z)) continue;
      const auto point = map_from_cloud * tf2::Vector3(*x, *y, *z);
      const double dx = point.x() - sensor_origin.x();
      const double dy = point.y() - sensor_origin.y();
      if (enforce_max_update_range && dx * dx + dy * dy > max_range_squared) continue;
      points.emplace_back(point.x(), point.y());
    }
    return points;
  }

  std::optional<Cell> mapCellCoordinates(double x, double y) const
  {
    const auto grid_point = grid_from_map_ * tf2::Vector3(x, y, 0.0);
    const double cell_x = grid_point.x() / map_.info.resolution;
    const double cell_y = grid_point.y() / map_.info.resolution;
    if (!std::isfinite(cell_x) || !std::isfinite(cell_y) ||
      cell_x < static_cast<double>(std::numeric_limits<std::int64_t>::min()) ||
      cell_x > static_cast<double>(std::numeric_limits<std::int64_t>::max()) ||
      cell_y < static_cast<double>(std::numeric_limits<std::int64_t>::min()) ||
      cell_y > static_cast<double>(std::numeric_limits<std::int64_t>::max()))
    {
      return std::nullopt;
    }
    return Cell(
      static_cast<std::int64_t>(std::floor(cell_x)),
      static_cast<std::int64_t>(std::floor(cell_y)));
  }

  std::optional<std::size_t> mapCell(double x, double y) const
  {
    const auto coordinates = mapCellCoordinates(x, y);
    if (!coordinates || coordinates->first < 0 || coordinates->second < 0 ||
      coordinates->first >= map_.info.width || coordinates->second >= map_.info.height)
    {
      return std::nullopt;
    }
    return static_cast<std::size_t>(coordinates->second) * map_.info.width +
           static_cast<std::size_t>(coordinates->first);
  }

  std::optional<std::size_t> diagnosticCellIndexLocked() const
  {
    return diagnostic_cell_enabled_ ? mapCell(diagnostic_cell_x_, diagnostic_cell_y_) :
           std::nullopt;
  }

  void logDiagnosticCellLocked(
    const std::size_t index, const char * event, const std::string & veto_reason,
    const std::int64_t source_stamp_ns)
  {
    if (!diagnostic_cell_enabled_ || index >= map_.data.size()) return;
    const auto ground_count = ground_evidence_[index].count();
    const auto occupancy = map_.data[index];
    const auto prior = prior_occupied_[index];
    const auto online = online_occupied_[index];
    const bool changed = !diagnostic_cell_state_initialized_ ||
      diagnostic_last_ground_count_ != ground_count ||
      diagnostic_last_occupancy_ != occupancy || diagnostic_last_prior_ != prior ||
      diagnostic_last_online_ != online || diagnostic_last_veto_reason_ != veto_reason ||
      diagnostic_last_map_free_stamp_ns_ != diagnostic_map_free_stamp_ns_;
    if (!changed) return;

    const auto width = map_.info.width;
    const auto grid_x = index % width;
    const auto grid_y = index / width;
    diagnostic_cell_state_initialized_ = true;
    diagnostic_last_ground_count_ = ground_count;
    diagnostic_last_occupancy_ = occupancy;
    diagnostic_last_prior_ = prior;
    diagnostic_last_online_ = online;
    diagnostic_last_veto_reason_ = veto_reason;
    diagnostic_last_map_free_stamp_ns_ = diagnostic_map_free_stamp_ns_;
    RCLCPP_INFO(
      get_logger(), "Global map cell diagnostic: event=%s world=(%.3f,%.3f) "
      "grid=(%zu,%zu) occupancy=%d prior=%u online=%u ground_confirmations=%u/%d "
      "obstacle_veto=%s map_free_stamp_ns=%lld source_stamp_ns=%lld",
      event, diagnostic_cell_x_, diagnostic_cell_y_, grid_x, grid_y,
      static_cast<int>(occupancy), static_cast<unsigned int>(prior),
      static_cast<unsigned int>(online), static_cast<unsigned int>(ground_count),
      occupancy >= 65 ? clear_observations_ : free_observations_,
      veto_reason.c_str(), static_cast<long long>(diagnostic_map_free_stamp_ns_),
      static_cast<long long>(source_stamp_ns));
  }

  void logDiscardedGlobalClearEvent(
    const PendingGlobalClearEvent & event, const char * reason)
  {
    if (!event.diagnostic_targets_cell) return;
    std::lock_guard<std::mutex> lock(mutex_);
    const auto index = diagnosticCellIndexLocked();
    if (index) {
      logDiagnosticCellLocked(
        *index, "clear_discarded", reason,
        rclcpp::Time(event.cloud->header.stamp).nanoseconds());
    }
  }

  bool growthAllowed(const Cell & cell) const
  {
    const auto extension = static_cast<std::int64_t>(
      std::floor(max_map_extension_m_ / map_.info.resolution));
    return cell.first >= map_origin_offset_x_ - extension &&
           cell.second >= map_origin_offset_y_ - extension &&
           cell.first < map_origin_offset_x_ + static_cast<std::int64_t>(original_width_) + extension &&
           cell.second < map_origin_offset_y_ + static_cast<std::int64_t>(original_height_) + extension;
  }

  bool growMapForGround(const std::vector<MapPoint> & points, bool & changed)
  {
    std::optional<Cell> min_cell;
    std::optional<Cell> max_cell;
    std::size_t outside_limit = 0;
    for (const auto & point : points) {
      const auto cell = mapCellCoordinates(point.first, point.second);
      if (!cell) continue;
      if (!growthAllowed(*cell)) {
        ++outside_limit;
        continue;
      }
      if (!min_cell) {
        min_cell = max_cell = cell;
      } else {
        min_cell->first = std::min(min_cell->first, cell->first);
        min_cell->second = std::min(min_cell->second, cell->second);
        max_cell->first = std::max(max_cell->first, cell->first);
        max_cell->second = std::max(max_cell->second, cell->second);
      }
    }
    if (outside_limit > 0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Ignoring %zu ground points beyond max_map_extension_m=%.1f",
        outside_limit, max_map_extension_m_);
    }
    if (!min_cell) return true;

    const auto quantum_cells = std::max<std::int64_t>(
      1, static_cast<std::int64_t>(std::ceil(
        kMapGrowthQuantumMeters / static_cast<double>(map_.info.resolution))));
    const auto floor_to_quantum = [quantum_cells](const std::int64_t cell) {
        auto quotient = cell / quantum_cells;
        if (cell % quantum_cells < 0) --quotient;
        return quotient * quantum_cells;
      };
    const auto ceil_to_quantum = [&floor_to_quantum, quantum_cells](const std::int64_t cell) {
        const auto floor_cell = floor_to_quantum(cell);
        return floor_cell == cell ? cell : floor_cell + quantum_cells;
      };
    const auto extension_cells = static_cast<std::int64_t>(
      std::floor(max_map_extension_m_ / static_cast<double>(map_.info.resolution)));
    const auto min_allowed_x = map_origin_offset_x_ - extension_cells;
    const auto min_allowed_y = map_origin_offset_y_ - extension_cells;
    const auto max_allowed_x = map_origin_offset_x_ + original_width_ + extension_cells - 1;
    const auto max_allowed_y = map_origin_offset_y_ + original_height_ + extension_cells - 1;
    auto growth_min_x = min_cell->first;
    auto growth_min_y = min_cell->second;
    auto growth_max_x = max_cell->first;
    auto growth_max_y = max_cell->second;
    if (growth_min_x < 0) {
      growth_min_x = std::max(floor_to_quantum(growth_min_x), min_allowed_x);
    }
    if (growth_min_y < 0) {
      growth_min_y = std::max(floor_to_quantum(growth_min_y), min_allowed_y);
    }
    if (growth_max_x >= map_.info.width) {
      growth_max_x = std::min(ceil_to_quantum(growth_max_x + 1) - 1, max_allowed_x);
    }
    if (growth_max_y >= map_.info.height) {
      growth_max_y = std::min(ceil_to_quantum(growth_max_y + 1) - 1, max_allowed_y);
    }

    const auto old_width = map_.info.width;
    const auto old_height = map_.info.height;
    bringup::GridGrowthResult growth;
    std::string error;
    if (!bringup::growOccupancyGridToInclude(
        map_, growth_min_x, growth_min_y, growth_max_x, growth_max_y,
        original_width_, original_height_, map_origin_offset_x_, map_origin_offset_y_,
        max_map_extension_m_,
        static_cast<std::size_t>(max_map_cells_), &growth, &error))
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Cannot extend fused map: %s", error.c_str());
      // Keep processing observations already inside the current map. Only the
      // out-of-bounds cells are dropped when the growth cap is reached.
      return true;
    }
    changed = growth.changed;
    if (!changed) return true;

    std::vector<bringup::ConsecutiveObservation> resized_obstacle(map_.data.size());
    std::vector<bringup::ConsecutiveObservation> resized_ray_clear(map_.data.size());
    std::vector<std::int64_t> resized_latest_obstacle_hit_stamp(map_.data.size(), 0);
    std::vector<bringup::TimeWindowObservation> resized_ground(map_.data.size());
    std::vector<std::uint8_t> resized_online_occupied(map_.data.size(), 0);
    std::vector<std::int8_t> resized_ground_provenance(map_.data.size(), 0);
    for (std::uint32_t y = 0; y < old_height; ++y) {
      for (std::uint32_t x = 0; x < old_width; ++x) {
        const auto old_index = static_cast<std::size_t>(y) * old_width + x;
        const auto new_index = static_cast<std::size_t>(y + growth.old_offset_y) * map_.info.width +
          x + growth.old_offset_x;
        resized_obstacle[new_index] = obstacle_evidence_[old_index];
        resized_ray_clear[new_index] = ray_clear_evidence_[old_index];
        resized_latest_obstacle_hit_stamp[new_index] =
          latest_obstacle_hit_stamp_ns_[old_index];
        resized_ground[new_index] = ground_evidence_[old_index];
        resized_online_occupied[new_index] = online_occupied_[old_index];
        resized_ground_provenance[new_index] = ground_provenance_[old_index];
      }
    }
    obstacle_evidence_ = std::move(resized_obstacle);
    ray_clear_evidence_ = std::move(resized_ray_clear);
    latest_obstacle_hit_stamp_ns_ = std::move(resized_latest_obstacle_hit_stamp);
    ground_evidence_ = std::move(resized_ground);
    online_occupied_ = std::move(resized_online_occupied);
    ground_provenance_ = std::move(resized_ground_provenance);
    prior_occupied_ = bringup::remapPriorOccupiedCells(
      prior_occupied_, old_width, old_height, map_.info.width, map_.info.height,
      growth.old_offset_x, growth.old_offset_y);
    map_origin_offset_x_ += growth.old_offset_x;
    map_origin_offset_y_ += growth.old_offset_y;
    tf2::Transform map_from_grid;
    tf2::fromMsg(map_.info.origin, map_from_grid);
    grid_from_map_ = map_from_grid.inverse();
    RCLCPP_INFO(
      get_logger(), "Expanded fused map to %ux%u; origin moved to (%.2f, %.2f)",
      map_.info.width, map_.info.height,
      map_.info.origin.position.x, map_.info.origin.position.y);
    return true;
  }

  std::vector<std::size_t> uniqueMapCells(const std::vector<MapPoint> & points) const
  {
    std::vector<std::size_t> cells;
    cells.reserve(points.size());
    for (const auto & point : points) {
      const auto index = mapCell(point.first, point.second);
      if (index) cells.push_back(*index);
    }
    std::sort(cells.begin(), cells.end());
    cells.erase(std::unique(cells.begin(), cells.end()), cells.end());
    return cells;
  }

  void groundCallback(
    const Cloud::ConstSharedPtr & ground,
    const Cloud::ConstSharedPtr & ground_veto)
  {
    if (ground->header.stamp.sec != ground_veto->header.stamp.sec ||
      ground->header.stamp.nanosec != ground_veto->header.stamp.nanosec)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "Skipping ground evidence: ground/veto timestamps are not identical");
      return;
    }
    const auto evidence_stamp_ns = rclcpp::Time(ground->header.stamp).nanoseconds();
    const auto source_age_ns = get_clock()->now().nanoseconds() - evidence_stamp_ns;
    const auto max_ground_age_ns = static_cast<std::int64_t>(
      ground_max_age_ * 1.0e9);
    const auto future_tolerance_ns = static_cast<std::int64_t>(
      ground_future_tolerance_ * 1.0e9);
    if (evidence_stamp_ns <= 0 || source_age_ns > max_ground_age_ns ||
      source_age_ns < -future_tolerance_ns)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "Skipping stale/future ground evidence (age=%.3f s)",
        static_cast<double>(source_age_ns) * 1.0e-9);
      return;
    }
    tf2::Transform map_from_ground;
    tf2::Transform map_from_veto;
    if (!lookupMapTransform(*ground, map_from_ground) ||
      !lookupMapTransform(*ground_veto, map_from_veto))
    {
      return;
    }
    tf2::Transform map_from_base;
    const bool have_base_transform = !footprint_clearing_enabled_ ||
      lookupMapBaseTransform(*ground, map_from_base);

    std::vector<MapPoint> ground_points;
    std::vector<MapPoint> veto_points;
    try {
      ground_points = mapPoints(*ground, map_from_ground);
      veto_points = mapPoints(*ground_veto, map_from_veto);
    } catch (const std::runtime_error & error) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000, "Map fusion requires float x/y/z fields: %s",
        error.what());
      return;
    }

    std::vector<MapPoint> confident_ground_points;
    confident_ground_points.reserve(ground_points.size());
    for (const auto & point : ground_points) {
      if (liveTerrainConfidentAt(point.first, point.second, evidence_stamp_ns)) {
        confident_ground_points.push_back(point);
      }
    }
    if (require_live_terrain_ && confident_ground_points.size() < ground_points.size()) {
      RCLCPP_DEBUG_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "FastDEM confidence withheld %zu of %zu ground points from online map updates",
        ground_points.size() - confident_ground_points.size(), ground_points.size());
    }
    std::vector<std::size_t> changed_cells;
    MapUpdatePublication update;
    nav_msgs::msg::OccupancyGrid expanded_map;
    nav_msgs::msg::OccupancyGrid expanded_provenance;
    nav_msgs::msg::OccupancyGrid expanded_occupancy_source;
    std::uint64_t expanded_geometry_version = 0;
    bool map_expanded = false;
    std::size_t occupied_cleared = 0;
    std::size_t online_occupied_cleared = 0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!map_ready_ || !localizationHealthy() || evidence_stamp_ns <= last_ground_stamp_ns_) {
        return;
      }
      last_ground_stamp_ns_ = evidence_stamp_ns;
      if (!growMapForGround(confident_ground_points, map_expanded)) return;
      if (map_expanded) {
        ++map_geometry_version_;
      }
      const auto ground_cells = uniqueMapCells(confident_ground_points);
      const auto veto_cells = uniqueMapCells(veto_points);
      const auto diagnostic_index = diagnosticCellIndexLocked();
      for (const auto index : veto_cells) {
        ground_evidence_[index].reset();
        if (diagnostic_index && index == *diagnostic_index) {
          logDiagnosticCellLocked(
            index, "ground_veto", "same_frame_obstacle_veto", evidence_stamp_ns);
        }
      }
      for (const auto index : ground_cells) {
        if (std::binary_search(veto_cells.begin(), veto_cells.end(), index)) {
          continue;
        }
        const auto occupancy = map_.data[index];
        const bool unknown = occupancy < 0;
        const bool occupied = occupancy >= 65;
        obstacle_evidence_[index].reset();
        if (!unknown && !occupied) {
          ground_evidence_[index].reset();
          if (diagnostic_index && index == *diagnostic_index) {
            logDiagnosticCellLocked(index, "ground_already_free", "none", evidence_stamp_ns);
          }
          continue;
        }
        const auto ground_count = ground_evidence_[index].observe(
          evidence_stamp_ns, kGroundEvidenceWindowNs);
        const auto updated_occupancy = bringup::groundEvidenceUpdate(
          occupancy, false, ground_count, free_observations_, clear_observations_);
        if (updated_occupancy) {
          if (occupancy >= 65) {
            ++occupied_cleared;
          }
          if (online_occupied_[index]) {
            online_occupied_[index] = 0;
            --online_occupied_count_;
            ++online_occupied_cleared;
            ray_clear_evidence_[index].reset();
          }
          if (updated_occupancy) map_.data[index] = *updated_occupancy;
          if (diagnostic_index && index == *diagnostic_index && *updated_occupancy == 0) {
            diagnostic_map_free_stamp_ns_ = evidence_stamp_ns;
            logDiagnosticCellLocked(index, "map_free", "none", evidence_stamp_ns);
          }
          changed_cells.push_back(index);
        } else if (diagnostic_index && index == *diagnostic_index) {
          logDiagnosticCellLocked(index, "ground_confirmed", "none", evidence_stamp_ns);
        }
      }
      if (footprint_clearing_enabled_ && have_base_transform) {
        std::vector<bringup::FootprintPoint2D> footprint_in_grid;
        footprint_in_grid.reserve(robot_footprint_.size());
        for (const auto & vertex : robot_footprint_) {
          const auto point_in_map = map_from_base * tf2::Vector3(vertex.x, vertex.y, 0.0);
          const auto point_in_grid = grid_from_map_ * point_in_map;
          footprint_in_grid.push_back({point_in_grid.x(), point_in_grid.y()});
        }
        // Match PathCollisionFree's footprint expansion for path sampling.
        const double footprint_clearing_margin_meters =
          bringup::path_collision::kMaxSampleDisplacementCells *
          static_cast<double>(map_.info.resolution);
        const auto footprint_cells = bringup::rasterizeFootprintCells(
          map_.info.width, map_.info.height, map_.info.resolution, footprint_in_grid,
          footprint_clearing_margin_meters);
        for (const auto index : footprint_cells) {
          if (prior_occupied_[index] || online_occupied_[index] == 0 ||
            std::binary_search(veto_cells.begin(), veto_cells.end(), index) ||
            map_.data[index] == 0)
          {
            continue;
          }
          if (map_.data[index] >= 65) {
            ++occupied_cleared;
          }
          online_occupied_[index] = 0;
          --online_occupied_count_;
          ++online_occupied_cleared;
          ray_clear_evidence_[index].reset();
        }
        const auto diagnostic_occupancy_before = diagnostic_index ?
          map_.data[*diagnostic_index] : static_cast<std::int8_t>(0);
        bringup::clearFootprintCells(
          map_.data, prior_occupied_, obstacle_evidence_, ground_evidence_,
          footprint_cells, changed_cells, veto_cells);
        if (diagnostic_index && diagnostic_occupancy_before != 0 &&
          map_.data[*diagnostic_index] == 0)
        {
          diagnostic_map_free_stamp_ns_ = evidence_stamp_ns;
          logDiagnosticCellLocked(
            *diagnostic_index, "footprint_map_free", "none", evidence_stamp_ns);
        }
      }
      if (map_expanded) {
        map_.header.stamp = bringup::nextMapSnapshotRevision(
          map_.header.stamp,
          bringup::mapSnapshotTimeFromNanoseconds(get_clock()->now().nanoseconds()));
        expanded_map = map_;
        expanded_geometry_version = map_geometry_version_;
        expanded_provenance = makeGroundProvenanceSnapshotLocked();
        expanded_occupancy_source = makeOccupancySourceSnapshotLocked();
        map_dirty_ = false;
        map_revision_from_terrain_ = false;
      } else if (!changed_cells.empty()) {
        update = makeUpdate(changed_cells);
        map_dirty_ = true;
        map_revision_from_terrain_ = false;
      }
      pending_occupied_cleared_ += occupied_cleared;
      pending_online_occupied_cleared_ += online_occupied_cleared;
    }
    if (map_expanded) {
      publishFullMap(expanded_map, expanded_geometry_version);
      ground_provenance_pub_->publish(expanded_provenance);
      occupancy_source_pub_->publish(expanded_occupancy_source);
    } else if (!changed_cells.empty()) {
      publishMapUpdate(update);
    }
  }

  void globalClearingCallback(const Cloud::ConstSharedPtr message)
  {
    bringup::ObstacleEvidenceFrameMetadata metadata;
    std::string source_frame_id;
    if (!bringup::parseObstacleClearEventFrameId(
        message->header.frame_id, source_frame_id, metadata) ||
      message->width == 0 || message->height == 0)
    {
      return;
    }
    const auto evidence_stamp_ns = rclcpp::Time(message->header.stamp).nanoseconds();
    if (evidence_stamp_ns <= 0) return;
    const auto source_now_ns = get_clock()->now().nanoseconds();

    bringup::FreshRetryQueue<PendingGlobalClearEvent>::EnqueueResult enqueue_result;
    {
      std::lock_guard<std::mutex> lock(pending_clear_mutex_);
      const auto event_id = ++clear_event_id_counter_;
      enqueue_result = pending_global_clear_events_.enqueue(
        PendingGlobalClearEvent{message, metadata, event_id}, evidence_stamp_ns, source_now_ns,
        std::chrono::steady_clock::now());
      (void)pending_global_clear_events_.takeDiscardedSourceStampNs();
      for (const auto & discarded : pending_global_clear_events_.takeDiscardedPayloads()) {
        clear_event_fence_refresh_tracker_.forget(discarded.event_id);
        logDiscardedGlobalClearEvent(discarded, "global_clear_retry_queue_capacity_drop");
      }
      if (enqueue_result !=
        bringup::FreshRetryQueue<PendingGlobalClearEvent>::EnqueueResult::REJECTED_EXPIRED) {
        latest_pending_clear_event_id_ = event_id;
      }
    }
    if (enqueue_result ==
      bringup::FreshRetryQueue<PendingGlobalClearEvent>::EnqueueResult::REJECTED_EXPIRED)
    {
      return;
    }
    if (enqueue_result ==
      bringup::FreshRetryQueue<PendingGlobalClearEvent>::EnqueueResult::DROPPED_OLDEST)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Global clear event retry queue is full; dropping its oldest pending event");
    }
  }

  void obstacleEvidenceCallback(const Cloud::ConstSharedPtr message)
  {
    const auto source_stamp_ns = rclcpp::Time(message->header.stamp).nanoseconds();
    if (source_stamp_ns <= 0) return;
    bringup::ObstacleEvidenceFrameMetadata metadata;
    if (!readObstacleEvidenceMetadata(*message, metadata)) {
      std::lock_guard<std::mutex> lock(pending_clear_mutex_);
      markObstacleHitEvidenceIncompleteThrough(source_stamp_ns);
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Obstacle hit evidence has invalid sequence metadata; holding global clears");
      return;
    }
    registerObstacleEvidenceFrame(metadata);
    const auto received_at = std::chrono::steady_clock::now();
    if (!recordObstacleHitEvidence(*message, metadata, source_stamp_ns, received_at)) {
      queueObstacleHitEvidence(message, metadata, received_at);
    }
  }

  void queueObstacleHitEvidence(
    const Cloud::ConstSharedPtr & message,
    const bringup::ObstacleEvidenceFrameMetadata & metadata,
    const std::chrono::steady_clock::time_point received_at)
  {
    const auto source_stamp_ns = rclcpp::Time(message->header.stamp).nanoseconds();
    const auto source_now_ns = get_clock()->now().nanoseconds();
    std::lock_guard<std::mutex> lock(pending_clear_mutex_);
    const auto result = pending_obstacle_hit_frames_.enqueue(
      ObstacleHitFrame{message, metadata}, source_stamp_ns, source_now_ns, received_at);
    const auto discarded_source_stamp_ns =
      pending_obstacle_hit_frames_.takeDiscardedSourceStampNs();
    if (discarded_source_stamp_ns > 0) {
      markObstacleHitEvidenceIncompleteThrough(discarded_source_stamp_ns);
    }
    if (result ==
      bringup::FreshRetryQueue<ObstacleHitFrame>::EnqueueResult::REJECTED_EXPIRED)
    {
      markObstacleHitEvidenceIncompleteThrough(source_stamp_ns);
    }
    if (discarded_source_stamp_ns > 0 || result ==
      bringup::FreshRetryQueue<ObstacleHitFrame>::EnqueueResult::REJECTED_EXPIRED)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Obstacle hit evidence was discarded; holding clear events at or before its source time");
    }
  }

  bool readObstacleEvidenceMetadata(
    const Cloud & message, bringup::ObstacleEvidenceFrameMetadata & metadata) const
  {
    if (message.width == 0 || message.height == 0) return false;
    try {
      sensor_msgs::PointCloud2ConstIterator<std::uint32_t> publisher_hi(
        message, "evidence_publisher_hi");
      sensor_msgs::PointCloud2ConstIterator<std::uint32_t> publisher_lo(
        message, "evidence_publisher_lo");
      sensor_msgs::PointCloud2ConstIterator<std::uint32_t> sequence_hi(
        message, "evidence_sequence_hi");
      sensor_msgs::PointCloud2ConstIterator<std::uint32_t> sequence_lo(
        message, "evidence_sequence_lo");
      metadata.publisher_id = (static_cast<std::uint64_t>(*publisher_hi) << 32) |
        static_cast<std::uint64_t>(*publisher_lo);
      metadata.sequence = (static_cast<std::uint64_t>(*sequence_hi) << 32) |
        static_cast<std::uint64_t>(*sequence_lo);
      return metadata.publisher_id != 0 && metadata.sequence != 0;
    } catch (const std::runtime_error &) {
      return false;
    }
  }

  void registerObstacleEvidenceFrame(
    const bringup::ObstacleEvidenceFrameMetadata & metadata)
  {
    std::lock_guard<std::mutex> pending_lock(pending_clear_mutex_);
    std::lock_guard<std::mutex> map_lock(mutex_);
    const auto result = obstacle_evidence_sequence_tracker_.observe(metadata);
    if (result == bringup::ObstacleEvidenceSequenceTracker::ObserveResult::PUBLISHER_CHANGED) {
      processed_obstacle_frame_stamps_.clear();
      obstacle_evidence_watermark_ns_ = 0;
      obstacle_hit_evidence_incomplete_through_stamp_ns_ = 0;
      confirmed_obstacle_evidence_fence_ = {};
      confirmed_fence_clear_event_id_ = 0;
      clear_event_fence_refresh_tracker_.reset();
      std::fill(latest_obstacle_hit_stamp_ns_.begin(), latest_obstacle_hit_stamp_ns_.end(), 0);
      std::fill(obstacle_evidence_.begin(), obstacle_evidence_.end(),
        bringup::ConsecutiveObservation{});
      std::fill(ray_clear_evidence_.begin(), ray_clear_evidence_.end(),
        bringup::ConsecutiveObservation{});
    }
  }

  void markObstacleHitEvidenceIncompleteThrough(const std::int64_t source_stamp_ns)
  {
    // Clears at or before a discarded blocker frame cannot establish a complete time range.
    obstacle_hit_evidence_incomplete_through_stamp_ns_ = std::max(
      obstacle_hit_evidence_incomplete_through_stamp_ns_, source_stamp_ns);
  }

  void requestObstacleEvidenceFenceLocked()
  {
    const auto steady_now = std::chrono::steady_clock::now();
    if (obstacle_evidence_fence_request_pending_ &&
      steady_now >= obstacle_evidence_fence_request_started_at_ &&
      steady_now - obstacle_evidence_fence_request_started_at_ >=
      kObstacleEvidenceFenceTimeout)
    {
      obstacle_evidence_fence_request_pending_ = false;
      ++obstacle_evidence_fence_request_token_;
      if (ground_veto_fence_client_ && obstacle_evidence_fence_request_id_ > 0) {
        (void)ground_veto_fence_client_->remove_pending_request(
          obstacle_evidence_fence_request_id_);
      }
      obstacle_evidence_fence_request_id_ = 0;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Ground veto publication fence timed out; retrying without clearing the map");
    }
    if (pending_global_clear_events_.empty() || obstacle_evidence_fence_request_pending_ ||
      (latest_pending_clear_event_id_ <= confirmed_fence_clear_event_id_ &&
      !clear_event_fence_refresh_tracker_.hasEventsNeedingFreshFence()))
    {
      return;
    }
    if (!ground_veto_fence_client_ || !ground_veto_fence_client_->service_is_ready()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Ground veto publication fence is unavailable; retaining global clear events");
      return;
    }

    obstacle_evidence_fence_request_pending_ = true;
    obstacle_evidence_fence_request_started_at_ = steady_now;
    // Freeze the batch boundary. Events enqueued while this request is in flight
    // are assigned later IDs and will be covered by the next fence request.
    const auto request_clear_event_id = latest_pending_clear_event_id_;
    const auto refresh_event_ids =
      clear_event_fence_refresh_tracker_.eventsNeedingFreshFenceThrough(request_clear_event_id);
    const auto request_token = ++obstacle_evidence_fence_request_token_;
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    const auto sent_request = ground_veto_fence_client_->async_send_request(
      request,
      [this, request_clear_event_id, refresh_event_ids, request_token](
        rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future)
      {
        bringup::ObstacleEvidenceFence fence;
        try {
          const auto response = future.get();
          if (response->success && !response->message.empty()) {
            (void)bringup::parseObstacleEvidenceFence(response->message, fence);
          }
        } catch (const std::exception & error) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "Ground veto publication fence request failed: %s", error.what());
        }

        std::lock_guard<std::mutex> lock(pending_clear_mutex_);
        if (request_token != obstacle_evidence_fence_request_token_) return;
        obstacle_evidence_fence_request_pending_ = false;
        obstacle_evidence_fence_request_id_ = 0;
        bool accepted = false;
        {
          std::lock_guard<std::mutex> map_lock(mutex_);
          accepted = obstacle_evidence_sequence_tracker_.establishFence(fence);
          if (accepted) confirmed_obstacle_evidence_fence_ = fence;
        }
        if (accepted) {
          confirmed_fence_clear_event_id_ = std::max(
            confirmed_fence_clear_event_id_, request_clear_event_id);
          clear_event_fence_refresh_tracker_.acceptFreshFence(refresh_event_ids);
        }
      });
    obstacle_evidence_fence_request_id_ = sent_request.request_id;
  }

  void refreshFenceForPendingClearLocked()
  {
    if (!fence_refresh_after_failed_clear_ &&
      !clear_event_fence_refresh_tracker_.hasEventsNeedingFreshFence()) return;
    // A response already in flight may still cover its captured batch. Do not
    // cancel it because evidence later became ready; the refresh tracker gates
    // those events until a request started after their wait is confirmed.
    if (obstacle_evidence_fence_request_pending_) return;
    ++obstacle_evidence_fence_request_token_;
    confirmed_fence_clear_event_id_ = 0;
    {
      std::lock_guard<std::mutex> map_lock(mutex_);
      confirmed_obstacle_evidence_fence_ = {};
    }
    fence_refresh_after_failed_clear_ = false;
  }

  bool clearEvidenceExpired(
    const std::int64_t source_stamp_ns,
    const std::chrono::steady_clock::time_point received_at)
  {
    const auto source_now_ns = get_clock()->now().nanoseconds();
    const auto max_age_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      kGlobalClearEvidenceMaxAge).count();
    const auto steady_now = std::chrono::steady_clock::now();
    const bool source_expired = source_now_ns > source_stamp_ns &&
      source_now_ns - source_stamp_ns >= max_age_ns;
    const bool receive_expired = steady_now >= received_at &&
      steady_now - received_at >= kGlobalClearEvidenceMaxAge;
    return source_stamp_ns <= 0 || source_expired || receive_expired;
  }

  bool recordObstacleHitEvidence(
    const Cloud & message, const bringup::ObstacleEvidenceFrameMetadata & metadata,
    const std::int64_t source_stamp_ns,
    const std::chrono::steady_clock::time_point received_at)
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (obstacle_evidence_sequence_tracker_.publisherId() != metadata.publisher_id) {
        return true;  // A queued frame from the previous publisher instance is stale.
      }
      if (obstacle_evidence_sequence_tracker_.isProcessed(metadata)) return true;
    }
    if (clearEvidenceExpired(source_stamp_ns, received_at) || message.header.frame_id.empty() ||
      rclcpp::Time(message.header.stamp).nanoseconds() != source_stamp_ns)
    {
      return false;
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!map_ready_ || !localizationHealthy()) return false;
    }
    tf2::Transform map_from_obstacles;
    if (!lookupMapTransform(message, map_from_obstacles)) return false;

    std::vector<MapPoint> obstacle_points;
    try {
      obstacle_points = mapPoints(message, map_from_obstacles, false);
    } catch (const std::runtime_error & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Cannot read obstacle hit evidence; retaining pending clears: %s", error.what());
      return false;
    }

    std::vector<MapPoint> terrain_confident_obstacles;
    if (require_live_terrain_) {
      terrain_confident_obstacles.reserve(obstacle_points.size());
      for (const auto & point : obstacle_points) {
        if (liveTerrainConfidentAt(point.first, point.second, source_stamp_ns)) {
          terrain_confident_obstacles.push_back(point);
        }
      }
    }

    MapUpdatePublication update;
    nav_msgs::msg::OccupancyGrid expanded_map;
    nav_msgs::msg::OccupancyGrid expanded_provenance;
    nav_msgs::msg::OccupancyGrid expanded_occupancy_source;
    std::uint64_t expanded_geometry_version = 0;
    bool map_expanded = false;
    std::vector<std::size_t> changed_cells;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (clearEvidenceExpired(source_stamp_ns, received_at) ||
        !map_ready_ || !localizationHealthy())
      {
        return false;
      }
      if (obstacle_evidence_sequence_tracker_.publisherId() != metadata.publisher_id) {
        return true;
      }
      if (obstacle_evidence_sequence_tracker_.isProcessed(metadata)) return true;
      const bool source_is_new = source_stamp_ns > last_obstacle_map_stamp_ns_;
      if (source_is_new && !growMapForGround(obstacle_points, map_expanded)) return false;
      if (map_expanded) ++map_geometry_version_;

      const auto obstacle_hit_cells = source_is_new ? uniqueMapCells(obstacle_points) :
        std::vector<std::size_t>{};
      const auto terrain_confident_cells = source_is_new ?
        uniqueMapCells(terrain_confident_obstacles) : std::vector<std::size_t>{};
      const auto source_now_ns = get_clock()->now().nanoseconds();
      const auto max_age_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        kGlobalClearEvidenceMaxAge).count();
      while (!processed_obstacle_frame_stamps_.empty() &&
        source_now_ns > *processed_obstacle_frame_stamps_.begin() &&
        source_now_ns - *processed_obstacle_frame_stamps_.begin() >= max_age_ns)
      {
        processed_obstacle_frame_stamps_.erase(processed_obstacle_frame_stamps_.begin());
      }
      processed_obstacle_frame_stamps_.insert(source_stamp_ns);
      if (!obstacle_evidence_sequence_tracker_.markProcessed(metadata)) return false;
      obstacle_evidence_watermark_ns_ =
        std::max(obstacle_evidence_watermark_ns_, source_stamp_ns);
      while (processed_obstacle_frame_stamps_.size() > kProcessedObstacleFrameCapacity) {
        processed_obstacle_frame_stamps_.erase(processed_obstacle_frame_stamps_.begin());
      }

      if (source_is_new) {
        last_obstacle_map_stamp_ns_ = source_stamp_ns;
        for (const auto index : obstacle_hit_cells) {
          latest_obstacle_hit_stamp_ns_[index] = std::max(
            latest_obstacle_hit_stamp_ns_[index], source_stamp_ns);
          ray_clear_evidence_[index].reset();
          const auto hit_count = obstacle_evidence_[index].observe(
            metadata.sequence, source_stamp_ns, kMaxObstacleEvidenceGapNs);
          const bool terrain_confident = std::binary_search(
            terrain_confident_cells.begin(), terrain_confident_cells.end(), index);
          const auto required_hits = terrain_confident || !require_live_terrain_ ?
            obstacle_confirmation_frames_ : fallback_obstacle_confirmation_frames_;
          if (hit_count < required_hits) continue;
          ground_evidence_[index].reset();
          if (prior_occupied_[index] != 0) continue;
          const bool changed = map_.data[index] != 100 || online_occupied_[index] == 0;
          map_.data[index] = 100;
          if (online_occupied_[index] == 0) {
            online_occupied_[index] = 1;
            ++online_occupied_count_;
          }
          if (changed) changed_cells.push_back(index);
        }
      }

      if (map_expanded) {
        map_.header.stamp = bringup::nextMapSnapshotRevision(
          map_.header.stamp,
          bringup::mapSnapshotTimeFromNanoseconds(get_clock()->now().nanoseconds()));
        expanded_map = map_;
        expanded_geometry_version = map_geometry_version_;
        expanded_provenance = makeGroundProvenanceSnapshotLocked();
        expanded_occupancy_source = makeOccupancySourceSnapshotLocked();
        map_dirty_ = false;
        map_revision_from_terrain_ = false;
      } else if (!changed_cells.empty()) {
        update = makeUpdate(changed_cells);
        map_dirty_ = true;
        map_revision_from_terrain_ = false;
      }
    }
    if (map_expanded) {
      publishFullMap(expanded_map, expanded_geometry_version);
      ground_provenance_pub_->publish(expanded_provenance);
      occupancy_source_pub_->publish(expanded_occupancy_source);
    } else if (!changed_cells.empty()) {
      publishMapUpdate(update);
    }
    return true;
  }

  void retryPendingGlobalClearEvents()
  {
    std::lock_guard<std::mutex> lock(pending_clear_mutex_);
    refreshFenceForPendingClearLocked();
    requestObstacleEvidenceFenceLocked();
    const auto steady_now = std::chrono::steady_clock::now();
    const auto source_now_ns = get_clock()->now().nanoseconds();
    (void)pending_obstacle_hit_frames_.retryAvailable(
      steady_now, source_now_ns,
      [this](const ObstacleHitFrame & frame, const std::int64_t source_stamp_ns,
        const std::chrono::steady_clock::time_point received_at) {
        {
          std::lock_guard<std::mutex> map_lock(mutex_);
          if (obstacle_evidence_sequence_tracker_.publisherId() != frame.metadata.publisher_id) {
            return true;
          }
        }
        if (clearEvidenceExpired(source_stamp_ns, received_at)) {
          markObstacleHitEvidenceIncompleteThrough(source_stamp_ns);
          return true;
        }
        return recordObstacleHitEvidence(
          *frame.cloud, frame.metadata, source_stamp_ns, received_at);
      });
    const auto discarded_hit_stamp_ns =
      pending_obstacle_hit_frames_.takeDiscardedSourceStampNs();
    if (discarded_hit_stamp_ns > 0) {
      markObstacleHitEvidenceIncompleteThrough(discarded_hit_stamp_ns);
    }
    (void)pending_obstacle_ray_frames_.retryAvailable(
      steady_now, source_now_ns,
      [this](const ObstacleRayFrame & frame, const std::int64_t source_stamp_ns,
        const std::chrono::steady_clock::time_point received_at) {
        return processClearingRayFrame(frame, source_stamp_ns, received_at);
      });
    (void)pending_obstacle_ray_frames_.takeDiscardedSourceStampNs();
    (void)pending_global_clear_events_.retryAvailableNewestFirst(
      steady_now, source_now_ns,
      [this](PendingGlobalClearEvent & event, const std::int64_t source_stamp_ns,
        const std::chrono::steady_clock::time_point received_at) {
        const bool applied = applyGlobalClearEvent(event, source_stamp_ns, received_at);
        if (applied) clear_event_fence_refresh_tracker_.forget(event.event_id);
        return applied;
      });
    (void)pending_global_clear_events_.takeDiscardedSourceStampNs();
    for (const auto & discarded : pending_global_clear_events_.takeDiscardedPayloads()) {
      clear_event_fence_refresh_tracker_.forget(discarded.event_id);
      logDiscardedGlobalClearEvent(discarded, "global_clear_event_expired_before_evidence_ready");
    }
  }

  bool applyGlobalClearEvent(
    PendingGlobalClearEvent & event, const std::int64_t source_stamp_ns,
    const std::chrono::steady_clock::time_point received_at)
  {
    const auto & message = *event.cloud;
    if (message.header.frame_id.empty() || message.width == 0 || message.height == 0 ||
      source_stamp_ns <= 0 ||
      rclcpp::Time(message.header.stamp).nanoseconds() != source_stamp_ns ||
      clearEvidenceExpired(source_stamp_ns, received_at))
    {
      return true;
    }
    if (diagnostic_cell_enabled_ && !event.diagnostic_target_checked) {
      tf2::Transform diagnostic_map_from_clear;
      if (lookupMapTransform(message, diagnostic_map_from_clear)) {
        try {
          const auto diagnostic_points = mapPoints(message, diagnostic_map_from_clear, false);
          std::lock_guard<std::mutex> lock(mutex_);
          if (map_ready_) {
            const auto diagnostic_index = diagnosticCellIndexLocked();
            const auto event_cells = uniqueMapCells(diagnostic_points);
            event.diagnostic_targets_cell = diagnostic_index &&
              std::binary_search(event_cells.begin(), event_cells.end(), *diagnostic_index);
            event.diagnostic_target_checked = true;
          }
        } catch (const std::runtime_error & error) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "Cannot associate pending clear event with diagnostic cell: %s", error.what());
        }
      }
    }
    const auto requireFenceRefresh = [this, &event]() {
        if (bringup::clearEventCoveredByFence(
            event.event_id, confirmed_fence_clear_event_id_))
        {
          fence_refresh_after_failed_clear_ = true;
        }
      };
    const auto blockerEvidenceReady = [this, source_stamp_ns, &event](std::string * reason) {
      const auto deny = [reason](const char * value) {
          if (reason) *reason = value;
          return false;
        };
      if (!bringup::clearEventCoveredByFence(
          event.event_id, confirmed_fence_clear_event_id_))
      {
        if (obstacle_evidence_fence_request_pending_) {
          return deny("obstacle_evidence_fence_request_pending");
        }
        if (!ground_veto_fence_client_ || !ground_veto_fence_client_->service_is_ready()) {
          return deny("obstacle_evidence_fence_service_unavailable");
        }
        return deny("obstacle_evidence_fence_request_required");
      }
      if (fence_refresh_after_failed_clear_) {
        return deny("fresh_fence_required_after_failed_clear");
      }
      const auto & fence = confirmed_obstacle_evidence_fence_;
      if (fence.publisher_id != event.metadata.publisher_id) {
        return deny("fence_publisher_mismatch");
      }
      if (fence.sequence < event.metadata.sequence) {
        return deny("fence_sequence_before_clear_event");
      }
      if (fence.source_stamp_ns < source_stamp_ns) {
        return deny("fence_stamp_before_clear_event");
      }
      const bool sequence_range_processed =
        obstacle_evidence_sequence_tracker_.sequenceRangeProcessed(
        event.metadata.publisher_id, event.metadata.sequence, fence.sequence);
      // If this range was incomplete on an earlier retry, its recovery only
      // arms a fresh-fence request; the old cutoff cannot release the clear.
      if (!clear_event_fence_refresh_tracker_.sequenceRangeReady(
          event.event_id, sequence_range_processed))
      {
        return deny(sequence_range_processed ?
          "fresh_fence_waiting_after_sequence_recovery" :
          "obstacle_evidence_sequence_range_incomplete");
      }
      const bool same_frame_processed =
        obstacle_evidence_sequence_tracker_.isProcessed(event.metadata) &&
        processed_obstacle_frame_stamps_.count(source_stamp_ns) != 0;
      const bool pending_blocker =
        pending_obstacle_hit_frames_.hasSourceStampAtOrAfter(source_stamp_ns);
      if (pending_blocker) return deny("newer_or_same_time_obstacle_frame_pending");
      if (obstacle_hit_evidence_incomplete_through_stamp_ns_ >= source_stamp_ns) {
        return deny("obstacle_evidence_queue_dropped_frame");
      }
      if (!same_frame_processed) return deny("same_frame_obstacle_evidence_pending");
      if (fence.source_stamp_ns < source_stamp_ns) {
        return deny("fence_stamp_before_clear_event");
      }
      if (obstacle_evidence_watermark_ns_ < fence.source_stamp_ns) {
        return deny("obstacle_evidence_watermark_before_fence");
      }
      return bringup::globalClearEvidenceReady(
        source_stamp_ns,
        same_frame_processed,
        fence.source_stamp_ns, obstacle_evidence_watermark_ns_,
        pending_blocker,
        obstacle_hit_evidence_incomplete_through_stamp_ns_);
    };
    const auto logDiagnosticWait = [this, &event, source_stamp_ns](const std::string & reason) {
        if (!event.diagnostic_targets_cell) return;
        const auto diagnostic_index = diagnosticCellIndexLocked();
        if (diagnostic_index) {
          logDiagnosticCellLocked(
            *diagnostic_index, "clear_wait", reason, source_stamp_ns);
        }
      };
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!map_ready_ || !localizationHealthy()) {
        requireFenceRefresh();
        logDiagnosticWait(!map_ready_ ? "map_not_ready" : "localization_unhealthy");
        return false;
      }
      std::string reason;
      if (!blockerEvidenceReady(&reason)) {
        logDiagnosticWait(reason);
        return false;
      }
    }

    tf2::Transform map_from_clear;
    if (!lookupMapTransform(message, map_from_clear)) {
      requireFenceRefresh();
      return false;
    }
    std::vector<MapPoint> clear_points;
    try {
      // These points are already range-gated by the obstacle trackers. Keep
      // their map coordinates unchanged even if odom's origin is far from the robot.
      clear_points = mapPoints(message, map_from_clear, false);
    } catch (const std::runtime_error & error) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000, "Map fusion requires float x/y/z clear events: %s",
        error.what());
      return true;
    }

    std::vector<std::size_t> changed_cells;
    MapUpdatePublication update;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (clearEvidenceExpired(source_stamp_ns, received_at)) return true;
      if (!map_ready_ || !localizationHealthy()) {
        requireFenceRefresh();
        logDiagnosticWait(!map_ready_ ? "map_not_ready" : "localization_unhealthy");
        return false;
      }
      std::string reason;
      if (!blockerEvidenceReady(&reason)) {
        logDiagnosticWait(reason);
        return false;
      }
      const auto diagnostic_index = diagnosticCellIndexLocked();
      for (const auto index : uniqueMapCells(clear_points)) {
        if (bringup::obstacleHitVetoesClearEvent(
            source_stamp_ns, latest_obstacle_hit_stamp_ns_[index]))
        {
          if (diagnostic_index && index == *diagnostic_index) {
            logDiagnosticCellLocked(
              index, "clear_denied", "same_or_newer_obstacle_hit", source_stamp_ns);
          }
          continue;
        }
        if (prior_occupied_[index] != 0 || online_occupied_[index] == 0) {
          if (diagnostic_index && index == *diagnostic_index) {
            logDiagnosticCellLocked(
              index, "clear_denied",
              prior_occupied_[index] != 0 ? "prior_occupied_protected" :
              "not_online_occupied", source_stamp_ns);
          }
          continue;
        }
        const auto updated_occupancy = bringup::onlineObstacleClearEventUpdate(
          map_.data[index], prior_occupied_[index] != 0, online_occupied_[index] != 0);
        if (updated_occupancy) {
          map_.data[index] = *updated_occupancy;
          ++pending_occupied_cleared_;
          if (diagnostic_index && index == *diagnostic_index && *updated_occupancy == 0) {
            diagnostic_map_free_stamp_ns_ = source_stamp_ns;
          }
        }
        online_occupied_[index] = 0;
        --online_occupied_count_;
        obstacle_evidence_[index].reset();
        ray_clear_evidence_[index].reset();
        ground_evidence_[index].reset();
        changed_cells.push_back(index);
        ++pending_online_occupied_cleared_;
        if (diagnostic_index && index == *diagnostic_index) {
          logDiagnosticCellLocked(
            index, "map_free", "none", source_stamp_ns);
        }
      }
      if (!changed_cells.empty()) {
        update = makeUpdate(changed_cells);
        map_dirty_ = true;
        map_revision_from_terrain_ = false;
      }
    }
    if (!changed_cells.empty()) {
      publishMapUpdate(update);
    }
    return true;
  }

  void clearingRayCallback(
    const Cloud::ConstSharedPtr & obstacle_message,
    const Cloud::ConstSharedPtr & ray_message)
  {
    if (obstacle_message->header.frame_id.empty() || ray_message->header.frame_id.empty() ||
      obstacle_message->header.stamp.sec != ray_message->header.stamp.sec ||
      obstacle_message->header.stamp.nanosec != ray_message->header.stamp.nanosec)
    {
      return;
    }
    const auto evidence_stamp_ns = rclcpp::Time(obstacle_message->header.stamp).nanoseconds();
    if (evidence_stamp_ns <= 0) return;
    const auto received_at = std::chrono::steady_clock::now();
    const ObstacleRayFrame frame{obstacle_message, ray_message};
    if (!processClearingRayFrame(frame, evidence_stamp_ns, received_at)) {
      const auto source_now_ns = get_clock()->now().nanoseconds();
      std::lock_guard<std::mutex> lock(pending_clear_mutex_);
      const auto result = pending_obstacle_ray_frames_.enqueue(
        frame, evidence_stamp_ns, source_now_ns, received_at);
      (void)pending_obstacle_ray_frames_.takeDiscardedSourceStampNs();
      if (result ==
        bringup::FreshRetryQueue<ObstacleRayFrame>::EnqueueResult::DROPPED_OLDEST)
      {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Global ray evidence retry queue is full; dropping its oldest frame conservatively");
      }
    }
  }

  // False means the same-frame blocker has not been recorded yet; the caller keeps the pair.
  bool processClearingRayFrame(
    const ObstacleRayFrame & frame, const std::int64_t evidence_stamp_ns,
    const std::chrono::steady_clock::time_point received_at)
  {
    const auto & obstacle_message = frame.first;
    const auto & ray_message = frame.second;
    if (clearEvidenceExpired(evidence_stamp_ns, received_at)) return true;
    bringup::ObstacleEvidenceFrameMetadata metadata;
    if (!readObstacleEvidenceMetadata(*obstacle_message, metadata)) return true;
    std::uint64_t evidence_frame = 0;
    bool can_clear_rays = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!map_ready_) return true;
      if (obstacle_evidence_sequence_tracker_.publisherId() != 0 &&
        obstacle_evidence_sequence_tracker_.publisherId() != metadata.publisher_id)
      {
        return true;
      }
      if (!bringup::rayClearEvidenceReady(
          evidence_stamp_ns, obstacle_evidence_sequence_tracker_.isProcessed(metadata)))
      {
        return false;
      }
      can_clear_rays = localizationHealthy();
      if (!can_clear_rays) {
        std::fill(ray_clear_evidence_.begin(), ray_clear_evidence_.end(),
          bringup::ConsecutiveObservation{});
      } else if (ray_clear_frame_ == std::numeric_limits<std::uint64_t>::max()) {
        std::fill(ray_clear_evidence_.begin(), ray_clear_evidence_.end(),
          bringup::ConsecutiveObservation{});
        ray_clear_frame_ = 0;
      }
      if (can_clear_rays) evidence_frame = ++ray_clear_frame_;
    }

    if (!can_clear_rays) return true;

    tf2::Transform map_from_obstacles;
    tf2::Transform map_from_cloud;
    if (!lookupMapTransform(*obstacle_message, map_from_obstacles) ||
      !lookupMapTransform(*ray_message, map_from_cloud))
    {
      return true;
    }
    std::vector<MapPoint> obstacle_points;
    try {
      obstacle_points = mapPoints(*obstacle_message, map_from_obstacles, false);
    } catch (const std::runtime_error & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Cannot map same-frame blocker cells; skipping global ray evidence: %s", error.what());
      return true;
    }
    std::vector<std::size_t> obstacle_hit_cells;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!map_ready_) return true;
      if (obstacle_evidence_sequence_tracker_.publisherId() != metadata.publisher_id) {
        return true;
      }
      if (!bringup::rayClearEvidenceReady(
          evidence_stamp_ns, obstacle_evidence_sequence_tracker_.isProcessed(metadata)))
      {
        return false;
      }
      obstacle_hit_cells = uniqueMapCells(obstacle_points);
    }

    struct WorldRay
    {
      MapPoint start;
      MapPoint endpoint;
    };
    std::vector<WorldRay> rays;
    rays.reserve(static_cast<std::size_t>(ray_message->width) * ray_message->height);

    try {
      sensor_msgs::PointCloud2ConstIterator<float> x(*ray_message, "x");
      sensor_msgs::PointCloud2ConstIterator<float> y(*ray_message, "y");
      sensor_msgs::PointCloud2ConstIterator<float> z(*ray_message, "z");
      for (; x != x.end(); ++x, ++y, ++z) {
        if (!std::isfinite(*x) || !std::isfinite(*y) || !std::isfinite(*z)) continue;
        const double range = std::sqrt(
          static_cast<double>(*x) * *x + static_cast<double>(*y) * *y +
          static_cast<double>(*z) * *z);
        if (range <= raytrace_min_range_) continue;

        const double start_scale = raytrace_min_range_ / range;
        const double endpoint_scale = std::min(raytrace_max_range_ / range, 1.0);
        const auto start = map_from_cloud * tf2::Vector3(
          *x * start_scale, *y * start_scale, *z * start_scale);
        const auto endpoint = map_from_cloud * tf2::Vector3(
          *x * endpoint_scale, *y * endpoint_scale, *z * endpoint_scale);
        rays.push_back({{start.x(), start.y()}, {endpoint.x(), endpoint.y()}});
      }
    } catch (const std::runtime_error & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Cannot read global clearing cloud; skipping ray evidence: %s", error.what());
      return true;
    }

    MapUpdatePublication update;
    std::vector<std::size_t> changed_cells;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!map_ready_) return true;
      if (!localizationHealthy()) {
        std::fill(ray_clear_evidence_.begin(), ray_clear_evidence_.end(),
          bringup::ConsecutiveObservation{});
        return true;
      }
      const auto diagnostic_index = diagnosticCellIndexLocked();
      std::vector<std::size_t> free_cells;
      free_cells.reserve(online_occupied_count_);
      for (const auto & ray : rays) {
        const auto start = mapCellCoordinates(ray.start.first, ray.start.second);
        const auto endpoint = mapCellCoordinates(ray.endpoint.first, ray.endpoint.second);
        if (!start || !endpoint) continue;
        auto crossed_cells = bringup::rasterizeFreeRay(
          {start->first, start->second}, {endpoint->first, endpoint->second},
          map_.info.width, map_.info.height, &online_occupied_);
        free_cells.insert(free_cells.end(), crossed_cells.begin(), crossed_cells.end());
      }
      free_cells = bringup::excludeCurrentObstacleHitsFromRayClears(
        std::move(free_cells), obstacle_hit_cells);

      for (const auto index : free_cells) {
        if (bringup::obstacleHitVetoesRayClear(
            evidence_stamp_ns, latest_obstacle_hit_stamp_ns_[index]))
        {
          if (diagnostic_index && index == *diagnostic_index) {
            logDiagnosticCellLocked(
              index, "ray_clear_denied", "same_or_newer_obstacle_hit", evidence_stamp_ns);
          }
          continue;
        }
        const auto clear_count = ray_clear_evidence_[index].observe(
          evidence_frame, evidence_stamp_ns, kMaxObstacleEvidenceGapNs);
        const auto updated_occupancy = bringup::onlineObstacleRayClearUpdate(
          map_.data[index], prior_occupied_[index] != 0,
          online_occupied_[index] != 0, clear_count,
          online_obstacle_clear_observations_);
        if (!updated_occupancy) {
          if (diagnostic_index && index == *diagnostic_index &&
            (prior_occupied_[index] != 0 || online_occupied_[index] == 0))
          {
            logDiagnosticCellLocked(
              index, "ray_clear_denied",
              prior_occupied_[index] != 0 ? "prior_occupied_protected" :
              "not_online_occupied", evidence_stamp_ns);
          }
          continue;
        }

        map_.data[index] = *updated_occupancy;
        if (diagnostic_index && index == *diagnostic_index && *updated_occupancy == 0) {
          diagnostic_map_free_stamp_ns_ = evidence_stamp_ns;
        }
        online_occupied_[index] = 0;
        --online_occupied_count_;
        obstacle_evidence_[index].reset();
        ray_clear_evidence_[index].reset();
        ground_evidence_[index].reset();
        changed_cells.push_back(index);
        ++pending_occupied_cleared_;
        ++pending_online_occupied_cleared_;
        if (diagnostic_index && index == *diagnostic_index) {
          logDiagnosticCellLocked(index, "map_free", "none", evidence_stamp_ns);
        }
      }
      if (!changed_cells.empty()) {
        update = makeUpdate(changed_cells);
        map_dirty_ = true;
        map_revision_from_terrain_ = false;
      }
    }
    if (!changed_cells.empty()) {
      publishMapUpdate(update);
    }
    return true;
  }

  void publishFullMap(
    const nav_msgs::msg::OccupancyGrid & map, const std::uint64_t geometry_version)
  {
    const auto event = bringup::makeMapGeometryEvent(
      map, map.header.stamp, map_geometry_publisher_epoch_, geometry_version,
      bringup::MapGeometryEventKind::FULL_MAP);
    map_geometry_pub_->publish(bringup::serializeMapGeometryEvent(event));
    map_pub_->publish(map);
  }

  void publishMapUpdate(const MapUpdatePublication & publication)
  {
    map_geometry_pub_->publish(publication.geometry_event);
    updates_pub_->publish(publication.update);
  }

  MapUpdatePublication makeUpdate(const std::vector<std::size_t> & changed)
  {
    const auto width = map_.info.width;
    std::uint32_t min_x = width;
    std::uint32_t min_y = map_.info.height;
    std::uint32_t max_x = 0;
    std::uint32_t max_y = 0;
    for (const auto index : changed) {
      const auto x = static_cast<std::uint32_t>(index % width);
      const auto y = static_cast<std::uint32_t>(index / width);
      min_x = std::min(min_x, x);
      min_y = std::min(min_y, y);
      max_x = std::max(max_x, x);
      max_y = std::max(max_y, y);
    }

    MapUpdatePublication publication;
    auto & update = publication.update;
    update.header.frame_id = map_.header.frame_id;
    map_.header.stamp = bringup::nextMapSnapshotRevision(
      map_.header.stamp,
      bringup::mapSnapshotTimeFromNanoseconds(get_clock()->now().nanoseconds()));
    update.header.stamp = map_.header.stamp;
    update.x = min_x;
    update.y = min_y;
    update.width = max_x - min_x + 1;
    update.height = max_y - min_y + 1;
    update.data.resize(static_cast<std::size_t>(update.width) * update.height);
    for (std::uint32_t y = 0; y < update.height; ++y) {
      for (std::uint32_t x = 0; x < update.width; ++x) {
        const auto source = static_cast<std::size_t>(min_y + y) * width + min_x + x;
        update.data[static_cast<std::size_t>(y) * update.width + x] = map_.data[source];
      }
    }
    const auto event = bringup::makeMapGeometryEvent(
      map_, update.header.stamp, map_geometry_publisher_epoch_, map_geometry_version_,
      bringup::MapGeometryEventKind::INCREMENTAL_UPDATE);
    publication.geometry_event = bringup::serializeMapGeometryEvent(event);
    return publication;
  }

  std::string prior_map_topic_;
  std::string map_frame_{"map"};
  bool initialize_without_prior_map_{false};
  double initial_map_resolution_{0.10};
  std::string map_topic_;
  std::string updates_topic_;
  std::string ground_topic_;
  std::string global_clearing_topic_;
  std::string ground_veto_topic_;
  std::string ground_veto_fence_service_name_;
  std::string clearing_ray_topic_;
  std::string health_topic_;
  std::string live_terrain_topic_;
  bool require_live_terrain_{false};
  double live_terrain_timeout_{1.5};
  double live_terrain_max_lag_{0.5};
  double live_terrain_future_tolerance_{0.1};
  int live_terrain_min_points_{3};
  int live_terrain_max_observation_age_{3};
  double ground_max_age_{0.5};
  double ground_future_tolerance_{0.1};
  std::string terrain_map_file_;
  int free_observations_{3};
  int obstacle_confirmation_frames_{2};
  int fallback_obstacle_confirmation_frames_{5};
  int clear_observations_{5};
  int online_obstacle_clear_observations_{5};
  double max_update_range_{20.0};
  double raytrace_min_range_{0.5};
  double raytrace_max_range_{20.0};
  double max_map_extension_m_{20.0};
  std::int64_t max_map_cells_{4000000};
  double health_timeout_{0.50};
  double tf_lookup_timeout_{0.15};
  double full_map_publish_period_{1.0};
  bool diagnostic_cell_enabled_{false};
  double diagnostic_cell_x_{0.0};
  double diagnostic_cell_y_{0.0};
  bool diagnostic_cell_state_initialized_{false};
  std::uint8_t diagnostic_last_ground_count_{255};
  std::int8_t diagnostic_last_occupancy_{127};
  std::uint8_t diagnostic_last_prior_{255};
  std::uint8_t diagnostic_last_online_{255};
  std::int64_t diagnostic_map_free_stamp_ns_{0};
  std::int64_t diagnostic_last_map_free_stamp_ns_{-1};
  std::string diagnostic_last_veto_reason_;
  std::string ground_provenance_file_;
  std::string occupancy_source_file_;
  bool footprint_clearing_enabled_{true};
  std::string base_frame_{"base_link"};
  std::vector<bringup::FootprintPoint2D> robot_footprint_;
  std::uint32_t original_width_{0};
  std::uint32_t original_height_{0};
  std::int64_t map_origin_offset_x_{0};
  std::int64_t map_origin_offset_y_{0};
  bool map_ready_{false};
  bool map_dirty_{false};
  bool map_revision_from_terrain_{false};
  bool have_health_{false};
  bool healthy_{false};
  std::chrono::steady_clock::time_point last_health_{};
  nav_msgs::msg::OccupancyGrid map_;
  fastdem::ElevationMap static_terrain_map_;
  std::vector<std::int8_t> ground_provenance_;
  std::int64_t last_global_terrain_stamp_ns_{0};
  tf2::Transform grid_from_map_;
  std::vector<bringup::ConsecutiveObservation> obstacle_evidence_;
  std::vector<bringup::ConsecutiveObservation> ray_clear_evidence_;
  std::vector<std::int64_t> latest_obstacle_hit_stamp_ns_;
  std::set<std::int64_t> processed_obstacle_frame_stamps_;
  std::int64_t obstacle_evidence_watermark_ns_{0};
  std::vector<bringup::TimeWindowObservation> ground_evidence_;
  std::int64_t last_ground_stamp_ns_{0};
  std::int64_t last_obstacle_map_stamp_ns_{0};
  std::vector<std::uint8_t> prior_occupied_;
  std::vector<std::uint8_t> online_occupied_;
  std::size_t online_occupied_count_{0};
  std::size_t pending_occupied_cleared_{0};
  std::size_t pending_online_occupied_cleared_{0};
  std::chrono::steady_clock::time_point last_fusion_stats_log_{};
  std::uint64_t ray_clear_frame_{0};
  std::uint64_t map_geometry_publisher_epoch_{0};
  std::uint64_t map_geometry_version_{0};
  std::mutex mutex_;
  mutable std::mutex live_terrain_mutex_;
  std::shared_ptr<const LiveTerrainSnapshot> live_terrain_snapshot_;
  std::mutex pending_clear_mutex_;
  std::int64_t obstacle_hit_evidence_incomplete_through_stamp_ns_{0};
  bringup::ObstacleEvidenceSequenceTracker obstacle_evidence_sequence_tracker_;
  bringup::ObstacleEvidenceFence confirmed_obstacle_evidence_fence_;
  std::uint64_t clear_event_id_counter_{0};
  std::uint64_t latest_pending_clear_event_id_{0};
  std::uint64_t confirmed_fence_clear_event_id_{0};
  std::uint64_t obstacle_evidence_fence_request_token_{0};
  bool obstacle_evidence_fence_request_pending_{false};
  bool fence_refresh_after_failed_clear_{false};
  bringup::ClearEventFenceRefreshTracker clear_event_fence_refresh_tracker_;
  std::int64_t obstacle_evidence_fence_request_id_{0};
  std::chrono::steady_clock::time_point obstacle_evidence_fence_request_started_at_{};
  bringup::FreshRetryQueue<ObstacleHitFrame> pending_obstacle_hit_frames_{
    kGlobalClearEventQueueCapacity, kGlobalClearEvidenceMaxAge};
  bringup::FreshRetryQueue<PendingGlobalClearEvent> pending_global_clear_events_{
    kGlobalClearEventQueueCapacity, kGlobalClearEvidenceMaxAge, true};
  bringup::FreshRetryQueue<ObstacleRayFrame> pending_obstacle_ray_frames_{
    kGlobalClearEventQueueCapacity, kGlobalClearEvidenceMaxAge};
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr prior_map_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr health_sub_;
  rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr global_terrain_sub_;
  rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr live_terrain_sub_;
  rclcpp::Subscription<Cloud>::SharedPtr obstacle_evidence_sub_;
  rclcpp::Subscription<Cloud>::SharedPtr global_clearing_sub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr ground_veto_fence_client_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr ground_provenance_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_source_pub_;
  rclcpp::Publisher<map_msgs::msg::OccupancyGridUpdate>::SharedPtr updates_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt64MultiArray>::SharedPtr map_geometry_pub_;
  rclcpp::TimerBase::SharedPtr snapshot_timer_;
  rclcpp::TimerBase::SharedPtr global_clear_retry_timer_;
  message_filters::Subscriber<Cloud> obstacle_sub_;
  message_filters::Subscriber<Cloud> clearing_ray_sub_;
  std::shared_ptr<message_filters::Synchronizer<ObstacleRaySyncPolicy>>
    obstacle_ray_synchronizer_;
  message_filters::Subscriber<Cloud> ground_sub_;
  message_filters::Subscriber<Cloud> ground_veto_sub_;
  std::shared_ptr<message_filters::Synchronizer<GroundVetoSyncPolicy>> ground_synchronizer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GlobalMapFusionNode>());
  rclcpp::shutdown();
  return 0;
}
