#include "nav2_terrain_layer/static_terrain_layer.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <optional>
#include <stdexcept>
#include <unordered_set>
#include <utility>
#include <vector>

#include <fastdem/io/npz.hpp>
#include <nanogrid/bridge/ros2.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2/exceptions.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace nav2_terrain_layer
{
namespace
{
bool hasLayer(const fastdem::ElevationMap & map, const char * name)
{
  return map.exists(name);
}

bool validGridMapGeometry(const grid_map_msgs::msg::GridMap & message)
{
  const auto & info = message.info;
  const auto & orientation = info.pose.orientation;
  const double quaternion_norm_squared = orientation.x * orientation.x +
    orientation.y * orientation.y + orientation.z * orientation.z + orientation.w * orientation.w;
  constexpr double kGeometryTolerance = 1e-6;
  if (message.layers.size() != message.data.size() ||
    !std::isfinite(info.resolution) || info.resolution <= 0.0 ||
    !std::isfinite(info.length_x) || !std::isfinite(info.length_y) ||
    info.length_x <= 0.0 || info.length_y <= 0.0 ||
    !std::isfinite(info.pose.position.x) || !std::isfinite(info.pose.position.y) ||
    !std::isfinite(info.pose.position.z) || std::abs(info.pose.position.z) > kGeometryTolerance ||
    !std::isfinite(quaternion_norm_squared) ||
    std::abs(quaternion_norm_squared - 1.0) > kGeometryTolerance ||
    std::abs(orientation.x) > kGeometryTolerance ||
    std::abs(orientation.y) > kGeometryTolerance ||
    std::abs(orientation.z) > kGeometryTolerance ||
    std::abs(std::abs(orientation.w) - 1.0) > kGeometryTolerance)
  {
    return false;
  }
  const double rows_value = info.length_x / info.resolution;
  const double cols_value = info.length_y / info.resolution;
  if (!std::isfinite(rows_value) || !std::isfinite(cols_value) ||
    rows_value < 1.0 || cols_value < 1.0 ||
    rows_value > std::numeric_limits<int>::max() ||
    cols_value > std::numeric_limits<int>::max())
  {
    return false;
  }
  const auto rows = std::llround(rows_value);
  const auto cols = std::llround(cols_value);
  if (rows < 1 || cols < 1 || rows > std::numeric_limits<int>::max() ||
    cols > std::numeric_limits<int>::max() ||
    message.outer_start_index >= static_cast<std::uint32_t>(rows) ||
    message.inner_start_index >= static_cast<std::uint32_t>(cols))
  {
    return false;
  }
  const auto cell_count = static_cast<std::size_t>(rows) * static_cast<std::size_t>(cols);
  std::unordered_set<std::string> layer_names;
  for (std::size_t i = 0; i < message.layers.size(); ++i) {
    const auto & layout = message.data[i].layout;
    if (!layer_names.insert(message.layers[i]).second || layout.dim.size() != 2 ||
      layout.data_offset != 0 || layout.dim[0].label != "column_index" ||
      layout.dim[1].label != "row_index" ||
      layout.dim[0].size != static_cast<std::uint32_t>(cols) ||
      layout.dim[1].size != static_cast<std::uint32_t>(rows) ||
      layout.dim[0].stride != cell_count ||
      layout.dim[1].stride != static_cast<std::uint32_t>(rows) ||
      message.data[i].data.size() != cell_count)
    {
      return false;
    }
  }
  return true;
}
}  // namespace

void StaticTerrainLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("StaticTerrainLayer lifecycle node expired during initialization");
  }

  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("terrain_map_file", rclcpp::ParameterValue(std::string("")));
  declareParameter("global_frame", rclcpp::ParameterValue(std::string("map")));
  declareParameter("robot_height", rclcpp::ParameterValue(0.40));
  declareParameter("clearance_margin", rclcpp::ParameterValue(0.10));
  declareParameter("max_climbable_step", rclcpp::ParameterValue(0.03));
  declareParameter("max_traversable_slope", rclcpp::ParameterValue(10.0));
  declareParameter("slope_soft_threshold", rclcpp::ParameterValue(5.0));
  declareParameter("slope_lethal_threshold", rclcpp::ParameterValue(10.0));
  declareParameter("residual_step_soft_threshold", rclcpp::ParameterValue(0.015));
  declareParameter("residual_step_lethal_threshold", rclcpp::ParameterValue(0.05));
  declareParameter("roughness_soft_threshold", rclcpp::ParameterValue(0.03));
  declareParameter("roughness_lethal_threshold", rclcpp::ParameterValue(0.10));
  declareParameter("soft_cost_scale", rclcpp::ParameterValue(1.0));
  declareParameter("min_observations", rclcpp::ParameterValue(1));
  declareParameter("hard_feature_constraints", rclcpp::ParameterValue(true));
  declareParameter("residual_step_hard_constraint",
    rclcpp::ParameterValue(residual_step_hard_constraint_));
  declareParameter("use_live_terrain", rclcpp::ParameterValue(use_live_terrain_));
  declareParameter("live_terrain_topic", rclcpp::ParameterValue(live_terrain_topic_));
  declareParameter("global_terrain_topic", rclcpp::ParameterValue(global_terrain_topic_));
  declareParameter("live_residual_timeout", rclcpp::ParameterValue(live_residual_timeout_));
  declareParameter("live_max_observation_age_frames",
    rclcpp::ParameterValue(live_max_observation_age_frames_));
  declareParameter("live_confirmation_frames", rclcpp::ParameterValue(live_confirmation_frames_));
  declareParameter(
    "live_confirmation_window_frames", rclcpp::ParameterValue(live_confirmation_window_frames_));
  declareParameter("live_min_neighbor_support", rclcpp::ParameterValue(live_min_neighbor_support_));

  node->get_parameter(getFullName("enabled"), enabled_);
  node->get_parameter(getFullName("terrain_map_file"), terrain_map_file_);
  node->get_parameter(getFullName("global_frame"), global_frame_);
  node->get_parameter(getFullName("robot_height"), cost_config_.robot_height);
  node->get_parameter(getFullName("clearance_margin"), cost_config_.clearance_margin);
  node->get_parameter(getFullName("max_climbable_step"), cost_config_.max_climbable_step);
  node->get_parameter(getFullName("max_traversable_slope"), cost_config_.max_traversable_slope);
  node->get_parameter(getFullName("slope_soft_threshold"), cost_config_.slope_soft);
  node->get_parameter(getFullName("slope_lethal_threshold"), cost_config_.slope_lethal);
  node->get_parameter(
    getFullName("residual_step_soft_threshold"), cost_config_.residual_step_soft);
  node->get_parameter(
    getFullName("residual_step_lethal_threshold"), cost_config_.residual_step_lethal);
  node->get_parameter(getFullName("roughness_soft_threshold"), cost_config_.roughness_soft);
  node->get_parameter(getFullName("roughness_lethal_threshold"), cost_config_.roughness_lethal);
  node->get_parameter(getFullName("soft_cost_scale"), cost_config_.soft_cost_scale);
  node->get_parameter(getFullName("min_observations"), min_observations_);
  node->get_parameter(getFullName("hard_feature_constraints"), hard_feature_constraints_);
  node->get_parameter(
    getFullName("residual_step_hard_constraint"), residual_step_hard_constraint_);
  node->get_parameter(getFullName("use_live_terrain"), use_live_terrain_);
  node->get_parameter(getFullName("live_terrain_topic"), live_terrain_topic_);
  node->get_parameter(getFullName("global_terrain_topic"), global_terrain_topic_);
  node->get_parameter(getFullName("live_residual_timeout"), live_residual_timeout_);
  node->get_parameter(
    getFullName("live_max_observation_age_frames"), live_max_observation_age_frames_);
  node->get_parameter(getFullName("live_confirmation_frames"), live_confirmation_frames_);
  node->get_parameter(
    getFullName("live_confirmation_window_frames"), live_confirmation_window_frames_);
  node->get_parameter(getFullName("live_min_neighbor_support"), live_min_neighbor_support_);

  // Costmap2D's default constructor leaves default_value_ indeterminate, and resetMaps()
  // refills the whole internal map with it.  updateWithMax() skips NO_INFORMATION, so that
  // is the only value which keeps a cell outside the loaded map from raising the master.
  default_value_ = nav2_costmap_2d::NO_INFORMATION;

  if (!enabled_) {
    matchSize();
    current_ = true;
    return;
  }
  if (terrain_map_file_.empty()) {
    throw std::runtime_error(
      "StaticTerrainLayer requires static_terrain_layer.terrain_map_file when enabled");
  }
  if (global_frame_.empty()) {
    throw std::runtime_error("StaticTerrainLayer global_frame must not be empty");
  }
  if (min_observations_ < 1 || !std::isfinite(cost_config_.robot_height) ||
    cost_config_.robot_height < 0.0 || !std::isfinite(cost_config_.clearance_margin) ||
    cost_config_.clearance_margin < 0.0 || !std::isfinite(cost_config_.max_climbable_step) ||
    cost_config_.max_climbable_step < 0.0 || !std::isfinite(cost_config_.max_traversable_slope) ||
    cost_config_.max_traversable_slope < 0.0 ||
    !std::isfinite(cost_config_.soft_cost_scale) || cost_config_.soft_cost_scale < 0.0 ||
    cost_config_.soft_cost_scale > 1.0 ||
    (use_live_terrain_ && live_terrain_topic_.empty()) ||
    global_terrain_topic_.empty() ||
    !std::isfinite(live_residual_timeout_) || live_residual_timeout_ <= 0.0 ||
    live_residual_timeout_ >=
    static_cast<double>(std::numeric_limits<std::int64_t>::max()) / 1e9 ||
    live_max_observation_age_frames_ < 0 || live_confirmation_frames_ < 1 ||
    live_confirmation_window_frames_ < live_confirmation_frames_ ||
    live_confirmation_window_frames_ > 64 || live_min_neighbor_support_ < 1 ||
    live_min_neighbor_support_ > 8)
  {
    throw std::runtime_error("StaticTerrainLayer capability parameters are invalid");
  }
  if (use_live_terrain_) {
    live_residual_timeout_ns_ = static_cast<std::int64_t>(
      std::llround(live_residual_timeout_ * 1e9));
  }
  if (!fastdem::io::loadNpz(terrain_map_file_, terrain_map_)) {
    throw std::runtime_error("StaticTerrainLayer failed to load terrain map: " + terrain_map_file_);
  }
  if (!validMap()) {
    throw std::runtime_error("StaticTerrainLayer terrain map failed geometry/layer validation: " +
      terrain_map_file_);
  }

  map_loaded_ = true;
  matchSize();
  current_ = geometry_valid_;
  global_terrain_publisher_ = node->create_publisher<grid_map_msgs::msg::GridMap>(
    global_terrain_topic_, rclcpp::QoS(1).reliable().transient_local());
  terrain_changed_publisher_ = node->create_publisher<std_msgs::msg::Empty>(
    "/navigation/terrain_map_changed", rclcpp::QoS(1).reliable());
  terrain_map_.setTimestamp(static_cast<std::uint64_t>(clock_->now().nanoseconds()));
  terrain_snapshot_cache_ = fastdem::ros2::toGridMap(terrain_map_, "elevation");
  terrain_snapshot_cache_valid_ = true;
  global_terrain_publisher_->publish(terrain_snapshot_cache_);
  fused_map_subscription_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map", rclcpp::QoS(1).reliable().transient_local(),
    std::bind(&StaticTerrainLayer::fusedMapCallback, this, std::placeholders::_1));
  if (use_live_terrain_) {
    live_terrain_subscription_ = node->create_subscription<grid_map_msgs::msg::GridMap>(
      live_terrain_topic_, rclcpp::SensorDataQoS(),
      std::bind(&StaticTerrainLayer::onlineTerrainCallback, this, std::placeholders::_1));
  }
  RCLCPP_INFO(
    logger_, "Loaded static terrain map '%s': %dx%d cells at %.3f m in frame '%s'; "
    "live updates %s",
    terrain_map_file_.c_str(), terrain_map_.getSize()(0), terrain_map_.getSize()(1),
    terrain_map_.getResolution(), terrain_map_.getFrameId().c_str(),
    use_live_terrain_ ? "enabled" : "disabled");
}

void StaticTerrainLayer::fusedMapCallback(
  const nav_msgs::msg::OccupancyGrid::ConstSharedPtr message)
{
  constexpr double kGeometryTolerance = 1.0e-5;
  if (!message) {
    std::lock_guard<std::mutex> lock(terrain_map_mutex_);
    geometry_valid_ = false;
    current_ = false;
    return;
  }
  const auto & origin_orientation = message->info.origin.orientation;
  if (message->header.frame_id != global_frame_ || message->info.width == 0 ||
    message->info.height == 0 || !std::isfinite(message->info.resolution) ||
    message->info.resolution <= 0.0F ||
    message->info.width > static_cast<std::uint32_t>(std::numeric_limits<int>::max()) ||
    message->info.height > static_cast<std::uint32_t>(std::numeric_limits<int>::max()) ||
    !std::isfinite(message->info.origin.position.x) ||
    !std::isfinite(message->info.origin.position.y) ||
    message->data.size() != static_cast<std::size_t>(message->info.width) * message->info.height ||
    std::abs(origin_orientation.x) > kGeometryTolerance ||
    std::abs(origin_orientation.y) > kGeometryTolerance ||
    std::abs(origin_orientation.z) > kGeometryTolerance ||
    std::abs(std::abs(origin_orientation.w) - 1.0) > kGeometryTolerance ||
    std::abs(message->info.origin.position.z) > kGeometryTolerance ||
    message->header.stamp.sec < 0 || message->header.stamp.nanosec >= 1000000000U)
  {
    {
      std::lock_guard<std::mutex> lock(terrain_map_mutex_);
      geometry_valid_ = false;
      current_ = false;
    }
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 2000, "Ignoring fused map with invalid terrain geometry or version");
    return;
  }

  const double resolution = message->info.resolution;
  const double length_x = static_cast<double>(message->info.width) * resolution;
  const double length_y = static_cast<double>(message->info.height) * resolution;
  const nanogrid::Position center(
    message->info.origin.position.x + length_x * 0.5,
    message->info.origin.position.y + length_y * 0.5);
  const auto timestamp = static_cast<std::uint64_t>(message->header.stamp.sec) * 1000000000ULL +
    message->header.stamp.nanosec;
  double snapshot_conversion_ms = 0.0;
  double snapshot_publish_ms = 0.0;
  bool geometry_changed = false;
  bool publish_snapshot = false;
  std::size_t layer_data_bytes = 0;
  std::size_t terrain_cell_count = 0;
  try {
    std::lock_guard<std::mutex> lock(terrain_map_mutex_);
    if (!map_loaded_ || std::abs(terrain_map_.getResolution() - resolution) > kGeometryTolerance) {
      throw std::runtime_error("fused map and static terrain resolutions do not match");
    }
    const auto & old_length = terrain_map_.getLength();
    const auto & old_position = terrain_map_.getPosition();
    const bool same_geometry = terrain_map_.getSize()(0) == static_cast<int>(message->info.width) &&
      terrain_map_.getSize()(1) == static_cast<int>(message->info.height) &&
      std::abs(old_length(0) - length_x) <= kGeometryTolerance &&
      std::abs(old_length(1) - length_y) <= kGeometryTolerance &&
      std::abs(old_position.x() - center.x()) <= kGeometryTolerance &&
      std::abs(old_position.y() - center.y()) <= kGeometryTolerance;
    if (!same_geometry) {
      geometry_changed = true;
      {
        // Live evidence keys are linearized using the current row count. A
        // geometry change invalidates every key, even where world cells stay put.
        std::lock_guard<std::mutex> evidence_lock(live_evidence_mutex_);
        live_residual_evidence_.clear();
        last_live_source_stamp_ns_ = 0;
        live_source_sequence_ = 0;
      }
      fastdem::ElevationMap resized(
        static_cast<float>(length_x), static_cast<float>(length_y),
        static_cast<float>(resolution), global_frame_);
      resized.setPosition(center);
      resized.setTimestamp(timestamp);
      for (const auto & layer : terrain_map_.getLayers()) {
        if (!resized.exists(layer)) resized.add(layer);
        float fill = std::numeric_limits<float>::quiet_NaN();
        if (layer == "class" || layer == "ground_source") fill = 0.0F;
        else if (layer == "clearance") fill = std::numeric_limits<float>::infinity();
        else if (layer == "n_points" || layer == "observation_age") fill = 0.0F;
        resized.get(layer).setConstant(fill);
      }
      if (resized.getSize()(0) != static_cast<int>(message->info.width) ||
        resized.getSize()(1) != static_cast<int>(message->info.height))
      {
        throw std::runtime_error("expanded terrain dimensions do not match the fused map");
      }
      for (const auto cell : terrain_map_.cells()) {
        const auto position = terrain_map_.position(cell);
        if (!position) continue;
        const auto new_index = resized.index(*position);
        if (!new_index) continue;
        const auto new_position = resized.position(*new_index);
        if (!new_position ||
          std::abs(new_position->x() - position->x()) > kGeometryTolerance ||
          std::abs(new_position->y() - position->y()) > kGeometryTolerance)
        {
          throw std::runtime_error("expanded terrain grid would move a measured cell");
        }
        for (const auto & layer : terrain_map_.getLayers()) {
          resized.at(layer, *new_index) = terrain_map_.get(layer)(cell.index);
        }
      }
      terrain_map_ = std::move(resized);
      dirty_terrain_cells_.clear();
      full_rebuild_needed_ = true;
    }
    terrain_map_.setTimestamp(timestamp);
    geometry_valid_ = costmapGeometryMatches();
    current_ = geometry_valid_;
    if (geometry_changed || !terrain_snapshot_cache_valid_) {
      const auto conversion_started = std::chrono::steady_clock::now();
      terrain_snapshot_cache_ = fastdem::ros2::toGridMap(terrain_map_, "elevation");
      terrain_snapshot_cache_valid_ = true;
      terrain_cell_count = static_cast<std::size_t>(terrain_map_.getSize().prod());
      snapshot_conversion_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - conversion_started).count();
    } else {
      terrain_snapshot_cache_.header.stamp = message->header.stamp;
      terrain_cell_count = static_cast<std::size_t>(terrain_map_.getSize().prod());
    }
    if (global_terrain_publisher_) {
      // Keep this publication in the same terrain-map critical section as the
      // cache update above. updateCosts() cannot replace its revision between
      // the fused-map update and publication.
      for (const auto & data : terrain_snapshot_cache_.data) {
        layer_data_bytes += data.data.size() * sizeof(float);
      }
      const auto publish_started = std::chrono::steady_clock::now();
      global_terrain_publisher_->publish(terrain_snapshot_cache_);
      snapshot_publish_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - publish_started).count();
      publish_snapshot = true;
    }
  } catch (const std::exception & error) {
    {
      std::lock_guard<std::mutex> lock(terrain_map_mutex_);
      geometry_valid_ = false;
      current_ = false;
    }
    RCLCPP_ERROR_THROTTLE(
      logger_, *clock_, 2000, "Cannot align global terrain with fused map: %s", error.what());
    return;
  }
  if (publish_snapshot) {
    if (++terrain_snapshot_publish_count_ % 10 == 0) {
      RCLCPP_INFO(
        logger_,
        "StaticTerrainLayer snapshot: conversion=%.3f ms publish=%.3f ms "
        "layer_data=%zu bytes cells=%zu geometry_changed=%s publish_count=%llu",
        snapshot_conversion_ms, snapshot_publish_ms,
        layer_data_bytes, terrain_cell_count,
        geometry_changed ? "true" : "false",
        static_cast<unsigned long long>(terrain_snapshot_publish_count_));
    }
  }
}

void StaticTerrainLayer::onlineTerrainCallback(
  const grid_map_msgs::msg::GridMap::SharedPtr message)
{
  if (!message || message->header.frame_id.empty() || message->header.stamp.sec < 0 ||
    message->header.stamp.nanosec >= 1000000000U ||
    (message->header.stamp.sec == 0 && message->header.stamp.nanosec == 0))
  {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 2000, "Ignoring live terrain correction with an invalid source header");
    return;
  }

  const auto has_layer = [&message](const char * name) {
      return std::find(message->layers.begin(), message->layers.end(), name) !=
             message->layers.end();
    };
  if (!has_layer("elevation") || !has_layer("slope") ||
    !has_layer("step_residual") || !has_layer("roughness") ||
    !has_layer("n_points") || !has_layer("observation_age"))
  {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 2000,
      "Ignoring live terrain correction without elevation, residual, and confidence layers");
    return;
  }
  if (!validGridMapGeometry(*message)) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 2000, "Ignoring live terrain correction with malformed grid geometry");
    return;
  }

  nanogrid::GridMap live_map;
  try {
    live_map = nanogrid::ros2::fromMsg(*message);
  } catch (const std::exception & error) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 2000, "Ignoring malformed live terrain correction: %s", error.what());
    return;
  }
  const auto & live_map_size = live_map.getSize();
  if (live_map_size(0) <= 0 || live_map_size(1) <= 0 ||
    !std::isfinite(live_map.getResolution()) || live_map.getResolution() <= 0.0)
  {
    return;
  }

  const auto stamp_ns = static_cast<std::uint64_t>(message->header.stamp.sec) * 1000000000ULL +
    message->header.stamp.nanosec;
  const auto now_ns = clock_->now().nanoseconds();
  const auto source_age_ns = now_ns - static_cast<std::int64_t>(stamp_ns);
  constexpr std::int64_t kFutureStampToleranceNs = 100000000LL;
  if (source_age_ns > live_residual_timeout_ns_ || source_age_ns < -kFutureStampToleranceNs) {
    return;
  }
  {
    std::lock_guard<std::mutex> lock(live_evidence_mutex_);
    if (stamp_ns <= last_live_source_stamp_ns_) {
      if (last_live_source_stamp_ns_ - stamp_ns <=
        static_cast<std::uint64_t>(live_residual_timeout_ns_))
      {
        return;
      }
    }
  }

  tf2::Transform global_from_live;
  global_from_live.setIdentity();
  if (message->header.frame_id != global_frame_) {
    try {
      const auto transform = tf_->lookupTransform(
        global_frame_, message->header.frame_id,
        rclcpp::Time(message->header.stamp, clock_->get_clock_type()),
        rclcpp::Duration::from_seconds(0.05));
      tf2::fromMsg(transform.transform, global_from_live);
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 2000,
        "Waiting for live terrain transform %s <- %s: %s",
        global_frame_.c_str(), message->header.frame_id.c_str(), error.what());
      return;
    }
  }

  struct TransformedFeatures
  {
    float slope;
    float residual;
    float roughness;
  };
  const bool same_global_frame = message->header.frame_id == global_frame_;
  const bool have_surface_normals = live_map.exists("_normal_x") &&
    live_map.exists("_normal_y") && live_map.exists("_normal_z");
  const auto transformed_features = [&](const auto & index_or_cell)
    -> std::optional<TransformedFeatures>
    {
      const auto slope = live_map.at("slope", index_or_cell);
      const auto residual = live_map.at("step_residual", index_or_cell);
      const auto roughness = live_map.at("roughness", index_or_cell);
      if (!std::isfinite(slope) || !std::isfinite(residual) || residual < 0.0F ||
        !std::isfinite(roughness) || roughness < 0.0F)
      {
        return std::nullopt;
      }
      if (same_global_frame) return TransformedFeatures{slope, residual, roughness};
      if (!have_surface_normals) return std::nullopt;
      const tf2::Vector3 local_normal(
        live_map.at("_normal_x", index_or_cell),
        live_map.at("_normal_y", index_or_cell),
        live_map.at("_normal_z", index_or_cell));
      if (!std::isfinite(local_normal.x()) || !std::isfinite(local_normal.y()) ||
        !std::isfinite(local_normal.z()) || local_normal.length2() < 1.0e-8)
      {
        return std::nullopt;
      }
      const auto global_normal = global_from_live.getBasis() * local_normal;
      const auto global_normal_length = global_normal.length();
      const double local_normal_z = std::abs(local_normal.z()) / local_normal.length();
      if (!std::isfinite(global_normal_length) || global_normal_length < 1.0e-4) {
        return std::nullopt;
      }
      const double global_normal_z = std::abs(global_normal.z()) / global_normal_length;
      if (!std::isfinite(local_normal_z) || !std::isfinite(global_normal_z) ||
        global_normal_z < 1.0e-4)
      {
        return std::nullopt;
      }
      const auto global_slope = static_cast<float>(
        std::acos(std::clamp(global_normal_z, 0.0, 1.0)) * 180.0 / M_PI);
      const auto global_residual = static_cast<float>(
        residual * local_normal_z / global_normal_z);
      if (!std::isfinite(global_slope) || !std::isfinite(global_residual)) {
        return std::nullopt;
      }
      // PCA roughness is the smallest covariance eigenvalue's square root and
      // remains invariant under the rigid 3D transform.
      return TransformedFeatures{global_slope, global_residual, roughness};
    };
  const auto live_cell_is_confident = [&](const auto & index_or_cell) {
      const auto elevation = live_map.at("elevation", index_or_cell);
      const auto features = transformed_features(index_or_cell);
      const auto points = live_map.at("n_points", index_or_cell);
      const auto age = live_map.at("observation_age", index_or_cell);
      return std::isfinite(elevation) && features && std::isfinite(points) &&
             points >= static_cast<float>(min_observations_) && std::isfinite(age) &&
             age >= 0.0F && age <= static_cast<float>(live_max_observation_age_frames_);
    };

  struct CellUpdate
  {
    std::int64_t key;
    double residual;
    bool supported;
    bool freshly_observed;
    float elevation;
    float slope;
    float roughness;
    float points;
    float observation_age;
    float variance;
  };
  std::unordered_map<std::int64_t, CellUpdate> updates;
  updates.reserve(static_cast<std::size_t>(live_map.getSize().prod()));
  std::lock_guard<std::mutex> terrain_lock(terrain_map_mutex_);
  if (!map_loaded_) return;
  const int map_rows = terrain_map_.getSize()(0);
  for (const auto cell : live_map.cells()) {
    const auto position = live_map.position(cell);
    if (!position) {
      continue;
    }
    const auto local_elevation = live_map.at("elevation", cell);
    const auto global_position = global_from_live * tf2::Vector3(
      position->x(), position->y(), std::isfinite(local_elevation) ? local_elevation : 0.0F);
    const auto static_index = terrain_map_.index(
      nanogrid::Position(global_position.x(), global_position.y()));
    if (!static_index) {
      continue;
    }

    const bool confident = live_cell_is_confident(cell);
    const auto features = transformed_features(cell);
    int supported_neighbors = 0;
    if (confident) {
      constexpr std::array<std::array<int, 2>, 8> kNeighborOffsets = {{{-1, -1}, {-1, 0},
        {-1, 1}, {0, -1}, {0, 1}, {1, -1}, {1, 0}, {1, 1}}};
      for (const auto & offset : kNeighborOffsets) {
        const auto neighbor_index = live_map.index(nanogrid::Position(
          position->x() + offset[0] * live_map.getResolution(),
          position->y() + offset[1] * live_map.getResolution()));
        if (neighbor_index && live_cell_is_confident(*neighbor_index)) {
          ++supported_neighbors;
        }
      }
    }
    const auto key = static_cast<std::int64_t>(
      (*static_index)(0) + (*static_index)(1) * map_rows);
    const CellUpdate update{
      key, features ? static_cast<double>(features->residual) : NAN,
      confident && supported_neighbors >= live_min_neighbor_support_,
      confident && live_map.at("observation_age", cell) == 0.0F &&
      supported_neighbors >= live_min_neighbor_support_,
      static_cast<float>(global_position.z()),
      features ? features->slope : NAN,
      features ? features->roughness : NAN,
      live_map.at("n_points", cell),
      live_map.at("observation_age", cell),
      same_global_frame && live_map.exists("variance") ?
      live_map.at("variance", cell) : NAN};
    const auto [entry, inserted] = updates.emplace(key, update);
    if (!inserted) {
      if (update.supported && (!entry->second.supported ||
        update.residual > entry->second.residual))
      {
        entry->second.residual = update.residual;
      }
      entry->second.supported = entry->second.supported && update.supported;
      entry->second.freshly_observed =
        entry->second.freshly_observed && update.freshly_observed;
    }
  }

  std::lock_guard<std::mutex> lock(live_evidence_mutex_);
  if (stamp_ns <= last_live_source_stamp_ns_) {
    if (last_live_source_stamp_ns_ - stamp_ns <=
      static_cast<std::uint64_t>(live_residual_timeout_ns_))
    {
      return;
    }
    for (const auto & [key, evidence] : live_residual_evidence_) {
      (void)evidence;
      dirty_terrain_cells_.insert(key);
    }
    live_residual_evidence_.clear();
    last_live_source_stamp_ns_ = 0;
    live_source_sequence_ = 0;
  }
  if (last_live_source_stamp_ns_ != 0 &&
    stamp_ns - last_live_source_stamp_ns_ > static_cast<std::uint64_t>(live_residual_timeout_ns_))
  {
    for (const auto & [key, evidence] : live_residual_evidence_) {
      (void)evidence;
      dirty_terrain_cells_.insert(key);
    }
    live_residual_evidence_.clear();
    live_source_sequence_ = 0;
  }
  last_live_source_stamp_ns_ = stamp_ns;
  ++live_source_sequence_;
  if (live_source_sequence_ == 0) {
    for (const auto & [key, evidence] : live_residual_evidence_) {
      (void)evidence;
      dirty_terrain_cells_.insert(key);
    }
    live_residual_evidence_.clear();
    live_source_sequence_ = 1;
  }
  const std::uint64_t history_mask = live_confirmation_window_frames_ == 64 ?
    std::numeric_limits<std::uint64_t>::max() :
    (std::uint64_t{1} << live_confirmation_window_frames_) - 1;
  for (const auto & [key, update] : updates) {
    const auto existing = live_residual_evidence_.find(key);
    if (existing == live_residual_evidence_.end() && !update.freshly_observed) {
      continue;
    }
    auto & evidence = live_residual_evidence_[key];
    const auto confirmed_count = [](std::uint64_t bits) {
        int count = 0;
        while (bits != 0) {
          bits &= bits - 1;
          ++count;
        }
        return count;
      };
    const bool was_confirmed =
      confirmed_count(evidence.low_residual_history) >= live_confirmation_frames_ ||
      confirmed_count(evidence.hard_step_history) >= live_confirmation_frames_;
    const auto elapsed_frames = evidence.last_source_sequence == 0 ?
      live_source_sequence_ : live_source_sequence_ - evidence.last_source_sequence;
    if (elapsed_frames >= static_cast<std::uint64_t>(live_confirmation_window_frames_)) {
      evidence.low_residual_history = 0;
      evidence.hard_step_history = 0;
    } else {
      evidence.low_residual_history = (evidence.low_residual_history << elapsed_frames) & history_mask;
      evidence.hard_step_history = (evidence.hard_step_history << elapsed_frames) & history_mask;
    }
    evidence.last_source_sequence = live_source_sequence_;
    if (update.freshly_observed) {
      evidence.last_fresh_observation_stamp_ns = static_cast<std::int64_t>(stamp_ns);
      evidence.cost_expired = false;
      evidence.residual_step = update.residual;
      evidence.elevation = update.elevation;
      evidence.slope = update.slope;
      evidence.roughness = update.roughness;
      evidence.points = update.points;
      evidence.observation_age = update.observation_age;
      evidence.variance = update.variance;
      if (update.residual >= cost_config_.max_climbable_step - 1.0e-9) {
        evidence.hard_step_history |= 1;
        evidence.low_residual_history = 0;
      } else {
        evidence.low_residual_history |= 1;
        evidence.hard_step_history = 0;
      }
    }
    if (update.freshly_observed || was_confirmed) {
      dirty_terrain_cells_.insert(key);
    }
  }
  for (auto it = live_residual_evidence_.begin(); it != live_residual_evidence_.end();) {
    if (it->second.last_fresh_observation_stamp_ns != 0 &&
      static_cast<std::int64_t>(stamp_ns) - it->second.last_fresh_observation_stamp_ns >
      3 * live_residual_timeout_ns_)
    {
      dirty_terrain_cells_.insert(it->first);
      it = live_residual_evidence_.erase(it);
    } else {
      ++it;
    }
  }
}

bool StaticTerrainLayer::costmapGeometryMatches() const
{
  if (!map_loaded_ || getSizeInCellsX() == 0 || getSizeInCellsY() == 0) {
    return false;
  }
  const auto & position = terrain_map_.getPosition();
  const auto & length = terrain_map_.getLength();
  constexpr double kToleranceMeters = 1e-5;
  constexpr double kMaxAlignmentToleranceCells = 1e-4;
  const double resolution = getResolution();
  if (getSizeInCellsX() == 0 || getSizeInCellsY() == 0 ||
    !std::isfinite(resolution) || resolution <= 0.0 ||
    std::abs(resolution - terrain_map_.getResolution()) > kToleranceMeters)
  {
    return false;
  }
  const double terrain_origin_x = position.x() - length.x() / 2.0;
  const double terrain_origin_y = position.y() - length.y() / 2.0;
  const double offset_x = (getOriginX() - terrain_origin_x) / resolution;
  const double offset_y = (getOriginY() - terrain_origin_y) / resolution;
  const double alignment_tolerance_cells = std::min(
    kToleranceMeters / resolution, kMaxAlignmentToleranceCells);
  const double costmap_max_x = getOriginX() + getSizeInCellsX() * resolution;
  const double costmap_max_y = getOriginY() + getSizeInCellsY() * resolution;
  if (getOriginX() > terrain_origin_x + kToleranceMeters ||
    getOriginY() > terrain_origin_y + kToleranceMeters ||
    costmap_max_x < position.x() + length.x() / 2.0 - kToleranceMeters ||
    costmap_max_y < position.y() + length.y() / 2.0 - kToleranceMeters)
  {
    return false;
  }
  return std::isfinite(offset_x) && std::isfinite(offset_y) &&
         std::abs(offset_x - std::round(offset_x)) <= alignment_tolerance_cells &&
         std::abs(offset_y - std::round(offset_y)) <= alignment_tolerance_cells;
}

bool StaticTerrainLayer::validMap() const
{
  if (!terrain_map_.isInitialized() || terrain_map_.getFrameId() != global_frame_ ||
    !std::isfinite(terrain_map_.getResolution()) || terrain_map_.getResolution() <= 0.0 ||
    !std::isfinite(terrain_map_.getLength()(0)) || !std::isfinite(terrain_map_.getLength()(1)) ||
    terrain_map_.getLength()(0) <= 0.0 || terrain_map_.getLength()(1) <= 0.0)
  {
    return false;
  }
  return hasLayer(terrain_map_, "elevation") && hasLayer(terrain_map_, "class") &&
    hasLayer(terrain_map_, "clearance") && hasLayer(terrain_map_, "slope") &&
    hasLayer(terrain_map_, "step_residual") && hasLayer(terrain_map_, "roughness") &&
    hasLayer(terrain_map_, "n_points");
}

void StaticTerrainLayer::addMapBounds(
  double * min_x, double * min_y, double * max_x, double * max_y) const
{
  const auto & position = terrain_map_.getPosition();
  const auto & length = terrain_map_.getLength();
  *min_x = std::min(*min_x, position.x() - length.x() / 2.0);
  *min_y = std::min(*min_y, position.y() - length.y() / 2.0);
  *max_x = std::max(*max_x, position.x() + length.x() / 2.0);
  *max_y = std::max(*max_y, position.y() + length.y() / 2.0);
}

void StaticTerrainLayer::updateBounds(
  double, double, double, double * min_x, double * min_y, double * max_x, double * max_y)
{
  const auto started = std::chrono::steady_clock::now();
  std::lock_guard<std::mutex> terrain_lock(terrain_map_mutex_);
  if (enabled_ && map_loaded_ && (!geometry_valid_ || !costmapGeometryMatches())) {
    CostmapLayer::matchSize();
    geometry_valid_ = costmapGeometryMatches();
    dirty_terrain_cells_.clear();
    full_rebuild_needed_ = true;
    if (geometry_valid_) {
      RCLCPP_INFO(
        logger_, "StaticTerrainLayer geometry matched loaded terrain map after costmap resize");
    }
  }
  current_ = !enabled_ || (map_loaded_ && geometry_valid_);
  if (enabled_ && map_loaded_ && geometry_valid_) {
    const auto now_ns = clock_->now().nanoseconds();
    {
      std::lock_guard<std::mutex> evidence_lock(live_evidence_mutex_);
      for (auto & [key, evidence] : live_residual_evidence_) {
        const auto age_ns = now_ns - evidence.last_fresh_observation_stamp_ns;
        if (!evidence.cost_expired && evidence.last_fresh_observation_stamp_ns != 0 &&
          age_ns > live_residual_timeout_ns_)
        {
          evidence.cost_expired = true;
          dirty_terrain_cells_.insert(key);
        }
      }
    }

    if (full_rebuild_needed_) {
      addMapBounds(min_x, min_y, max_x, max_y);
    } else {
      const int rows = terrain_map_.getSize()(0);
      const int cols = terrain_map_.getSize()(1);
      const double half_cell = terrain_map_.getResolution() * 0.5;
      for (const auto key : dirty_terrain_cells_) {
        if (rows <= 0 || key < 0 || key >= static_cast<std::int64_t>(rows) * cols) {
          full_rebuild_needed_ = true;
          break;
        }
        const nanogrid::Index index(
          static_cast<int>(key % rows), static_cast<int>(key / rows));
        const auto position = terrain_map_.position(index);
        if (!position) {
          full_rebuild_needed_ = true;
          break;
        }
        *min_x = std::min(*min_x, position->x() - half_cell);
        *min_y = std::min(*min_y, position->y() - half_cell);
        *max_x = std::max(*max_x, position->x() + half_cell);
        *max_y = std::max(*max_y, position->y() + half_cell);
      }
      if (full_rebuild_needed_) {
        addMapBounds(min_x, min_y, max_x, max_y);
      }
    }
  } else if (enabled_ && map_loaded_) {
    *min_x = std::min(*min_x, getOriginX());
    *min_y = std::min(*min_y, getOriginY());
    *max_x = std::max(*max_x, getOriginX() + getSizeInMetersX());
    *max_y = std::max(*max_y, getOriginY() + getSizeInMetersY());
  }
  last_update_bounds_ms_ = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - started).count();
}

void StaticTerrainLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j, int max_i, int max_j)
{
  const auto started = std::chrono::steady_clock::now();
  std::unique_lock<std::mutex> terrain_lock(terrain_map_mutex_);
  if (!enabled_ || !map_loaded_) {
    current_ = !enabled_;
    return;
  }
  if (!geometry_valid_ || !costmapGeometryMatches()) {
    CostmapLayer::matchSize();
    geometry_valid_ = costmapGeometryMatches();
    dirty_terrain_cells_.clear();
    full_rebuild_needed_ = true;
  }
  if (!geometry_valid_) {
    current_ = false;
    RCLCPP_ERROR_THROTTLE(
      logger_, *clock_, 2000,
      "StaticTerrainLayer grid mismatch: terrain=%dx%d @ %.3f origin=(%.3f, %.3f), "
      "costmap=%ux%u @ %.3f origin=(%.3f, %.3f); refusing costs",
      terrain_map_.getSize()(0), terrain_map_.getSize()(1), terrain_map_.getResolution(),
      terrain_map_.getPosition().x() - terrain_map_.getLength().x() / 2.0,
      terrain_map_.getPosition().y() - terrain_map_.getLength().y() / 2.0,
      getSizeInCellsX(), getSizeInCellsY(), getResolution(), getOriginX(), getOriginY());
    return;
  }

  const auto & slope = terrain_map_.get("slope");
  const auto & residual = terrain_map_.get("step_residual");
  const auto & roughness = terrain_map_.get("roughness");
  const auto & terrain_class = terrain_map_.get("class");
  auto & mutable_terrain_class = terrain_map_.get("class");
  auto * ground_source = terrain_map_.exists("ground_source") ?
    &terrain_map_.get("ground_source") : nullptr;
  const auto & clearance = terrain_map_.get("clearance");
  const auto & n_points = terrain_map_.get("n_points");
  const auto now_ns = clock_->now().nanoseconds();
  std::unique_lock<std::mutex> lock(live_evidence_mutex_);
  std::size_t live_correction_cells = 0;
  std::size_t confirmed_live_step_cells = 0;
  std::size_t promoted_ground_cells = 0;
  std::size_t recomputed_cells = 0;
  const auto count_bits = [](std::uint64_t bits) {
      int count = 0;
      while (bits != 0) {
        bits &= bits - 1;
        ++count;
      }
      return count;
    };
  const auto update_cell = [&](const nanogrid::Index & index) {
      const auto position = terrain_map_.position(index);
      if (!position) return false;
      unsigned int mx;
      unsigned int my;
      if (!master_grid.worldToMap(position->x(), position->y(), mx, my)) return false;
      if (static_cast<int>(mx) < min_i || static_cast<int>(mx) >= max_i ||
        static_cast<int>(my) < min_j || static_cast<int>(my) >= max_j)
      {
        return false;
      }

      const auto row = index(0);
      const auto column = index(1);
      const auto cell_key = static_cast<std::int64_t>(
        row + column * terrain_map_.getSize()(0));
      auto cell_class = terrain_class(row, column);
      const bool unknown_ground = std::isfinite(cell_class) && cell_class == 0.0F &&
        ground_source && (*ground_source)(row, column) == 0.0F;
      if (unknown_ground) {
        const auto live_evidence = live_residual_evidence_.find(cell_key);
        if (live_evidence != live_residual_evidence_.end()) {
          const auto & evidence = live_evidence->second;
          const auto age_ns = now_ns - evidence.last_fresh_observation_stamp_ns;
          const bool confirmed_low = count_bits(evidence.low_residual_history) >=
            live_confirmation_frames_;
          const bool fresh_supported_measurement =
            evidence.last_fresh_observation_stamp_ns != 0 && age_ns >= 0 &&
            age_ns <= live_residual_timeout_ns_ && !evidence.cost_expired &&
            std::isfinite(evidence.elevation) &&
            std::isfinite(evidence.slope) && std::isfinite(evidence.residual_step) &&
            std::isfinite(evidence.roughness) && std::isfinite(evidence.points) &&
            evidence.points >= static_cast<float>(min_observations_);
          const bool measured_ground_geometry = fresh_supported_measurement &&
            evidence.slope < cost_config_.max_traversable_slope &&
            evidence.residual_step < cost_config_.max_climbable_step &&
            evidence.roughness < cost_config_.roughness_lethal;
          if (fresh_supported_measurement && confirmed_low && measured_ground_geometry) {
            terrain_map_.get(fastdem::layer::elevation)(row, column) = evidence.elevation;
            terrain_map_.get(fastdem::layer::elevation_min)(row, column) = evidence.elevation;
            terrain_map_.get(fastdem::layer::elevation_max)(row, column) = evidence.elevation;
            terrain_map_.get(fastdem::layer::n_points)(row, column) = evidence.points;
            terrain_map_.get(fastdem::layer::observation_age)(row, column) =
              evidence.observation_age;
            if (terrain_map_.exists("variance") && std::isfinite(evidence.variance)) {
              terrain_map_.get("variance")(row, column) = evidence.variance;
            }
            if (terrain_map_.exists("slope") && std::isfinite(evidence.slope)) {
              terrain_map_.get("slope")(row, column) = evidence.slope;
            }
            if (terrain_map_.exists("step_residual")) {
              terrain_map_.get("step_residual")(row, column) =
                static_cast<float>(evidence.residual_step);
            }
            if (terrain_map_.exists("roughness") && std::isfinite(evidence.roughness)) {
              terrain_map_.get("roughness")(row, column) = evidence.roughness;
            }
            mutable_terrain_class(row, column) = 1.0F;
            if (ground_source) (*ground_source)(row, column) = 1.0F;
            cell_class = 1.0F;
            ++promoted_ground_cells;
          }
        }
      }
      // Occupancy and obstacle clearing come from the live map layer. Keep this
      // static layer limited to traversability features on known ground so its
      // original unknown/obstacle labels cannot restore stale occupancy.
      if (!std::isfinite(cell_class) || cell_class < 0.5F || cell_class >= 1.5F ||
        !std::isfinite(n_points(row, column)) ||
        n_points(row, column) < static_cast<float>(min_observations_))
      {
        setCost(mx, my, nav2_costmap_2d::NO_INFORMATION);
        ++recomputed_cells;
        return true;
      }
      double residual_step = residual(row, column);
      bool confirmed_live_step = false;
      const auto live_evidence = live_residual_evidence_.find(cell_key);
      if (live_evidence != live_residual_evidence_.end()) {
        const auto & evidence = live_evidence->second;
        const auto age_ns = now_ns - evidence.last_fresh_observation_stamp_ns;
        if (evidence.last_fresh_observation_stamp_ns != 0 && age_ns >= 0 &&
          age_ns <= live_residual_timeout_ns_ && !evidence.cost_expired)
        {
          if (count_bits(evidence.low_residual_history) >= live_confirmation_frames_) {
            residual_step = evidence.residual_step;
            ++live_correction_cells;
          } else if (count_bits(evidence.hard_step_history) >= live_confirmation_frames_) {
            residual_step = evidence.residual_step;
            confirmed_live_step = true;
            ++live_correction_cells;
            ++confirmed_live_step_cells;
          }
        }
      }
      unsigned char cost = terrainCost(
        1.0F, clearance(row, column), slope(row, column), residual_step,
        roughness(row, column), cost_config_, hard_feature_constraints_,
        false, hard_feature_constraints_);
      if (confirmed_live_step && residual_step_hard_constraint_) {
        cost = nav2_costmap_2d::LETHAL_OBSTACLE;
      }
      cost = scaleSoftTerrainCost(cost, cost_config_.soft_cost_scale);
      setCost(mx, my, cost);
      ++recomputed_cells;
      return true;
    };

  const bool full_rebuild = full_rebuild_needed_;
  const auto dirty_count = dirty_terrain_cells_.size();
  std::unordered_set<std::int64_t> pending_dirty_cells;
  if (full_rebuild) {
    resetMaps();
    for (const auto cell : terrain_map_.cells()) {
      if (!update_cell(nanogrid::Index(cell.bufRow, cell.bufCol))) {
        current_ = false;
        return;
      }
    }
    full_rebuild_needed_ = false;
    dirty_terrain_cells_.clear();
  } else {
    const int rows = terrain_map_.getSize()(0);
    const int cols = terrain_map_.getSize()(1);
    for (const auto key : dirty_terrain_cells_) {
      if (rows <= 0 || key < 0 || key >= static_cast<std::int64_t>(rows) * cols) {
        full_rebuild_needed_ = true;
        current_ = false;
        return;
      }
      const nanogrid::Index index(
        static_cast<int>(key % rows), static_cast<int>(key / rows));
      const auto position = terrain_map_.position(index);
      unsigned int mx;
      unsigned int my;
      if (!position || !master_grid.worldToMap(position->x(), position->y(), mx, my)) {
        full_rebuild_needed_ = true;
        current_ = false;
        return;
      }
      if (static_cast<int>(mx) < min_i || static_cast<int>(mx) >= max_i ||
        static_cast<int>(my) < min_j || static_cast<int>(my) >= max_j)
      {
        pending_dirty_cells.insert(key);
        continue;
      }
      if (!update_cell(index)) {
        full_rebuild_needed_ = true;
        current_ = false;
        return;
      }
    }
    dirty_terrain_cells_ = std::move(pending_dirty_cells);
  }
  updateWithMax(master_grid, min_i, min_j, max_i, max_j);
  grid_map_msgs::msg::GridMap promoted_snapshot;
  const bool publish_promoted_snapshot = promoted_ground_cells > 0 &&
    static_cast<bool>(global_terrain_publisher_);
  if (publish_promoted_snapshot) {
    terrain_map_.setTimestamp(static_cast<std::uint64_t>(now_ns));
    terrain_snapshot_cache_ = fastdem::ros2::toGridMap(terrain_map_, "elevation");
    terrain_snapshot_cache_valid_ = true;
    promoted_snapshot = terrain_snapshot_cache_;
  }
  const auto map_cell_count = static_cast<std::size_t>(terrain_map_.getSize().prod());
  const auto remaining_dirty_count = dirty_terrain_cells_.size();
  const auto bounds_duration_ms = last_update_bounds_ms_;
  lock.unlock();
  terrain_lock.unlock();
  if (publish_promoted_snapshot) {
    global_terrain_publisher_->publish(promoted_snapshot);
  }
  if (promoted_ground_cells > 0 && terrain_changed_publisher_) {
    terrain_changed_publisher_->publish(std_msgs::msg::Empty{});
  }
  if (live_correction_cells > 0) {
    RCLCPP_INFO_THROTTLE(
      logger_, *clock_, 2000,
      "StaticTerrainLayer applied fresh supported live residuals to %zu cells; "
      "%zu cells confirmed as >= %.3f m steps, promoted unknown ground cells=%zu",
      live_correction_cells, confirmed_live_step_cells, cost_config_.max_climbable_step,
      promoted_ground_cells);
  }
  const auto update_finished = std::chrono::steady_clock::now();
  if (++cost_update_count_ % 10 == 0) {
    const auto window_width = std::max(0, max_i - min_i);
    const auto window_height = std::max(0, max_j - min_j);
    RCLCPP_INFO(
      logger_,
      "StaticTerrainLayer update: bounds=%.3f ms costs=%.3f ms recomputed=%zu/%zu "
      "dirty=%zu remaining=%zu window=%zu map_cells=%zu full_rebuild=%s promoted=%zu",
      bounds_duration_ms,
      std::chrono::duration<double, std::milli>(update_finished - started).count(),
      recomputed_cells, map_cell_count, dirty_count,
      remaining_dirty_count,
      static_cast<std::size_t>(window_width) * static_cast<std::size_t>(window_height),
      map_cell_count,
      full_rebuild ? "true" : "false", promoted_ground_cells);
  }
}

void StaticTerrainLayer::reset()
{
  std::lock_guard<std::mutex> terrain_lock(terrain_map_mutex_);
  {
    std::lock_guard<std::mutex> evidence_lock(live_evidence_mutex_);
    live_residual_evidence_.clear();
    last_live_source_stamp_ns_ = 0;
    live_source_sequence_ = 0;
  }
  dirty_terrain_cells_.clear();
  resetMaps();
  full_rebuild_needed_ = map_loaded_;
  current_ = !enabled_ || (map_loaded_ && geometry_valid_);
}

void StaticTerrainLayer::matchSize()
{
  std::lock_guard<std::mutex> terrain_lock(terrain_map_mutex_);
  CostmapLayer::matchSize();
  if (!enabled_ || !map_loaded_ || getSizeInCellsX() == 0 || getSizeInCellsY() == 0) {
    geometry_valid_ = !enabled_ || !map_loaded_;
    current_ = geometry_valid_;
    return;
  }
  geometry_valid_ = costmapGeometryMatches();
  dirty_terrain_cells_.clear();
  full_rebuild_needed_ = true;
  if (geometry_valid_) {
    RCLCPP_INFO(
      logger_, "StaticTerrainLayer geometry matched loaded terrain map after costmap resize");
  }
  current_ = geometry_valid_;
}
}  // namespace nav2_terrain_layer

PLUGINLIB_EXPORT_CLASS(nav2_terrain_layer::StaticTerrainLayer, nav2_costmap_2d::Layer)
