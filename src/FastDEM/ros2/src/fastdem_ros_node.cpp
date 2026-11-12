// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024 Ikhyeon Cho <tre0430@korea.ac.kr>

#include <spdlog/spdlog.h>

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <fastdem/bridge/ros2.hpp>
#include <fastdem/fastdem.hpp>
#include <fastdem/postprocess/feature_extraction.hpp>
#include <fastdem/postprocess/inpainting.hpp>
#include <fastdem/postprocess/uncertainty_fusion.hpp>
#include <atomic>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>
#include <nanopcl/bridge/ros2.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <shared_mutex>
#include <std_srvs/srv/trigger.hpp>
#include <utility>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "fastdem_ros/parameters.hpp"
#include "fastdem_ros/postprocess_publication_guard.hpp"
#include "fastdem_ros/tf_bridge.hpp"

namespace fastdem::ros2 {

using fastdem::ElevationMap;
using fastdem::FastDEM;
using fastdem::PointCloud;

class MappingNode : public rclcpp::Node {
 public:
  MappingNode() : rclcpp::Node("fastdem_node") {}

  bool initialize() {
    if (!loadParameters()) return false;

    setupCore();
    setupPublishersAndSubscribers();
    printNodeSummary();

    spdlog::info("FastDEM node initialized successfully!");
    return true;
  }

 private:
  bool loadParameters() {
    this->declare_parameter<std::string>("config_file", "");
    std::string config_path = this->get_parameter("config_file").as_string();

    // Launch arg override: input_scan
    this->declare_parameter<std::string>("input_scan", "");

    try {
      cfg_ = NodeConfig::load(config_path);
    } catch (const std::exception& e) {
      spdlog::error("Failed to load config: {}", e.what());
      return false;
    }

    auto input_scan = this->get_parameter("input_scan").as_string();
    if (!input_scan.empty()) {
      cfg_.topics.input_scans = {input_scan};
    }

    spdlog::set_level(spdlog::level::from_str(cfg_.logger_level));
    return true;
  }

  void setupCore() {
    // Elevation map geometry
    map_.setGeometry(cfg_.map.width, cfg_.map.height, cfg_.map.resolution);
    map_.setFrameId(cfg_.tf.map_frame);

    // Mapper
    mapper_ = std::make_unique<FastDEM>(map_, cfg_.pipeline);

    // ROS TF provides both Calibration and Odometry
    tf_ = std::make_shared<TFBridge>(this->get_clock(),        //
                                     cfg_.tf.base_frame,       //
                                     cfg_.tf.map_frame,        //
                                     cfg_.tf.max_wait_time,    //
                                     cfg_.tf.max_stale_time);  //
    mapper_->setTransformProvider(tf_);

    // Register callbacks to publish intermediate clouds
    mapper_->onScanPreprocessed(
        [this](const PointCloud& cloud) {
          if (pub_scan_->get_subscription_count() > 0) pending_processed_ = cloud;
        });
    mapper_->onScanRasterized(
        [this](const PointCloud& cloud) {
          if (pub_rasterized_->get_subscription_count() > 0) pending_rasterized_ = cloud;
        });
  }

  void setupPublishersAndSubscribers() {
    // clang-format off
    using Cloud       = sensor_msgs::msg::PointCloud2;
    using GridMapMsg  = grid_map_msgs::msg::GridMap;
    using Marker      = visualization_msgs::msg::Marker;
    using MarkerArray = visualization_msgs::msg::MarkerArray;
    using Trigger     = std_srvs::srv::Trigger;

    scan_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    postprocess_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    publish_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    service_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions scan_options;
    scan_options.callback_group = scan_group_;
    for (const auto& topic : cfg_.topics.input_scans)
      scan_subs_.push_back(this->create_subscription<Cloud>(
          topic, rclcpp::SensorDataQoS().keep_last(1),
          std::bind(&MappingNode::scanCallback, this, std::placeholders::_1),
          scan_options));

    pub_scan_         = this->create_publisher<Cloud>("~/scan/preprocessed", 1);
    pub_rasterized_   = this->create_publisher<Cloud>("~/scan/rasterized", 1);
    pub_map_          = this->create_publisher<Cloud>("~/mapping/cloud", 1);
    pub_gridmap_      = this->create_publisher<GridMapMsg>("~/mapping/gridmap", 1);
    pub_boundary_     = this->create_publisher<Marker>("~/mapping/boundary", 1);
    pub_global_map_   = this->create_publisher<Cloud>("~/mapping/cloud_global", 1);

    pub_post_map_     = this->create_publisher<Cloud>("~/postprocess/cloud", 1);
    pub_post_gridmap_ = this->create_publisher<GridMapMsg>("~/postprocess/gridmap", 1);
    pub_normals_      = this->create_publisher<MarkerArray>("~/postprocess/normals", 1);

    auto to_ms = [](double rate) {
      return std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::duration<double>(1.0 / rate));
    };

    timer_scan_output_ = this->create_wall_timer(
        to_ms(cfg_.topics.publish_rate),
        std::bind(&MappingNode::publishPendingScans, this), publish_group_);
    timer_clock_rollback_ = this->create_wall_timer(
        std::chrono::milliseconds(20),
        std::bind(&MappingNode::clockRollbackMaintenanceCallback, this), publish_group_);
    timer_viz_ = this->create_wall_timer(
        to_ms(cfg_.topics.publish_rate),
        std::bind(&MappingNode::publishLocalView, this), publish_group_);
    if (cfg_.pipeline.mapping.mode == MappingMode::GLOBAL)
      timer_global_ = this->create_wall_timer(
          to_ms(cfg_.topics.global_publish_rate),
          std::bind(&MappingNode::publishGlobalView, this), publish_group_);

    auto bind_srv = [this](auto callback) {
      return std::bind(callback, this, std::placeholders::_1, std::placeholders::_2);
    };
    srv_reset_       = this->create_service<Trigger>("~/reset_map",             bind_srv(&MappingNode::resetMapCallback), rmw_qos_profile_services_default, service_group_);
    srv_postprocess_ = this->create_service<Trigger>("~/run_postprocess",       bind_srv(&MappingNode::runPostProcessCallback), rmw_qos_profile_services_default, service_group_);
    srv_inpainting_  = this->create_service<Trigger>("~/run_inpainting",        bind_srv(&MappingNode::runInpaintingCallback), rmw_qos_profile_services_default, service_group_);
    srv_uf_          = this->create_service<Trigger>("~/run_uncertainty_fusion", bind_srv(&MappingNode::runUncertaintyFusionCallback), rmw_qos_profile_services_default, service_group_);
    srv_fe_          = this->create_service<Trigger>("~/run_feature_extraction", bind_srv(&MappingNode::runFeatureExtractionCallback), rmw_qos_profile_services_default, service_group_);
    // clang-format on
  }

  // ==================== Services ====================

  using TriggerRequest = std::shared_ptr<std_srvs::srv::Trigger::Request>;
  using TriggerResponse = std::shared_ptr<std_srvs::srv::Trigger::Response>;

  void resetMapCallback(const TriggerRequest, TriggerResponse res) {
    std::lock_guard publish_lock(publish_mutex_);
    std::unique_lock lock(map_mutex_);
    mapper_->reset();
    ++map_generation_;
    ++reset_generation_;
    auto empty_map = map_.snapshot(
        {layer::elevation, layer::upper_bound, layer::lower_bound,
         layer::n_points, layer::observation_age});
    lock.unlock();
    publishEmptyMapOutputs(std::move(empty_map));
    res->success = true;
    res->message = "Map reset";
    spdlog::info("Map reset by service call");
  }

  void runPostProcessCallback(const TriggerRequest, TriggerResponse res) {
    res->success = runPostProcess(true, true, true);
    res->message = res->success ? "Post-processing published"
                                : "Post-processing skipped";
  }

  void runInpaintingCallback(const TriggerRequest, TriggerResponse res) {
    res->success = runPostProcess(false, true, false);
    res->message = res->success ? "Inpainting published" : "Inpainting skipped";
  }

  void runUncertaintyFusionCallback(const TriggerRequest, TriggerResponse res) {
    res->success = runPostProcess(true, false, false);
    res->message = res->success ? "Uncertainty fusion published"
                                : "Uncertainty fusion skipped";
  }

  void runFeatureExtractionCallback(const TriggerRequest, TriggerResponse res) {
    res->success = runPostProcess(false, false, true);
    res->message = res->success ? "Feature extraction published"
                                : "Feature extraction skipped";
  }

  // ==================== Process ====================

  void scanCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    const auto callback_started = std::chrono::steady_clock::now();
    if (!handleClockRollback()) return;
    auto cloud = std::make_shared<PointCloud>(nanopcl::from(*msg));
    if (!isSourceTimestampPlausible(cloud->timestamp())) {
      spdlog::warn("Input cloud timestamp is zero or too far in the future. "
                   "Skipping...");
      return;
    }

    static bool first_scan = true;
    if (first_scan) {
      spdlog::info("First scan received. Mapping started...");
      const bool has_postproc = cfg_.postprocess.uncertainty_fusion.enabled ||
                                cfg_.postprocess.inpainting.enabled ||
                                cfg_.postprocess.feature_extraction.enabled;
      if (has_postproc && cfg_.topics.post_process_rate > 0.0) {
        auto period =
            std::chrono::duration<double>(1.0 / cfg_.topics.post_process_rate);
        timer_post_process_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::milliseconds>(period),
            std::bind(&MappingNode::postProcessCallback, this),
            postprocess_group_);
      }
      first_scan = false;
    }

    std::optional<PointCloud> processed;
    std::optional<PointCloud> rasterized;
    uint64_t reset_generation;
    const auto lock_requested = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point lock_acquired;
    std::chrono::steady_clock::time_point mapping_finished;
    {
      std::unique_lock lock(map_mutex_);
      lock_acquired = std::chrono::steady_clock::now();
      pending_processed_.reset();
      pending_rasterized_.reset();
      if (mapper_->integrate(cloud)) ++map_generation_;
      processed = std::move(pending_processed_);
      rasterized = std::move(pending_rasterized_);
      reset_generation = reset_generation_.load();
      mapping_finished = std::chrono::steady_clock::now();
    }
    if (processed || rasterized) {
      std::lock_guard queue_lock(scan_output_mutex_);
      queued_processed_ = std::move(processed);
      queued_rasterized_ = std::move(rasterized);
      queued_reset_generation_ = reset_generation;
    }
    const auto callback_finished = std::chrono::steady_clock::now();
    if (++scan_timing_count_ % 20 == 0) {
      const auto milliseconds = [](const auto duration) {
          return std::chrono::duration<double, std::milli>(duration).count();
        };
      spdlog::info(
        "FastDEM scan: callback={} ms map_lock_wait={} ms integrate_and_stage={} ms "
        "deferred_publish={} ms",
        milliseconds(callback_finished - callback_started),
        milliseconds(lock_acquired - lock_requested),
        milliseconds(mapping_finished - lock_acquired),
        milliseconds(callback_finished - mapping_finished));
    }
  }

  // ==================== Post-processing ====================

  void postProcessCallback() {
    const auto& pp = cfg_.postprocess;
    runPostProcess(pp.uncertainty_fusion.enabled, pp.inpainting.enabled,
                   pp.feature_extraction.enabled);
  }

  bool runPostProcess(bool do_uf, bool do_inpainting, bool do_fe) {
    if (clock_rollback_pending_.load()) return false;
    const auto started = std::chrono::steady_clock::now();
    const bool want_cloud = pub_post_map_->get_subscription_count() > 0;
    const bool want_gridmap = pub_post_gridmap_->get_subscription_count() > 0 &&
                              cfg_.pipeline.mapping.mode != MappingMode::GLOBAL;
    const bool want_normals =
        pub_normals_->get_subscription_count() > 0 && do_fe;
    if (!want_cloud && !want_gridmap && !want_normals) return false;

    ElevationMap map_copy;
    uint64_t generation = 0;
    uint64_t reset_generation = 0;
    const auto snapshot_lock_requested = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point snapshot_lock_acquired;
    {
      std::shared_lock lock(map_mutex_);
      snapshot_lock_acquired = std::chrono::steady_clock::now();
      map_copy = map_.snapshot(
          {layer::elevation, layer::upper_bound, layer::lower_bound,
           layer::n_points, layer::observation_age});
      generation = map_generation_.load();
      reset_generation = reset_generation_.load();
    }
    const auto snapshot_finished = std::chrono::steady_clock::now();
    const auto source_timestamp = map_copy.getTimestamp();

    if (map_copy.isEmpty() || !isMapFresh(map_copy)) {
      std::lock_guard publish_lock(publish_mutex_);
      std::shared_lock lock(map_mutex_);
      if (clock_rollback_pending_.load() || reset_generation != reset_generation_.load() ||
        !postprocess_publication_order_.canClearStaleSnapshot(
            source_timestamp, generation, map_generation_.load())) return false;
      std::lock_guard state_lock(clock_rollback_state_mutex_);
      if (!postprocessSnapshotCanPublish(
          reset_generation, reset_generation_.load(), clock_rollback_pending_.load()))
        return false;
      if (postprocess_output_valid_.exchange(false)) {
        postprocess_publication_order_.recordPublished(source_timestamp);
        publishEmptyPostProcess(std::move(map_copy), source_timestamp);
      }
      return false;
    }

    // Post-processing on snapshot (lock-free)
    const auto processing_started = std::chrono::steady_clock::now();
    const auto& pp = cfg_.postprocess;
    if (do_uf) applyUncertaintyFusion(map_copy, pp.uncertainty_fusion);
    if (do_inpainting)
      applyInpainting(map_copy, pp.inpainting.max_iterations,
                       pp.inpainting.min_valid_neighbors, /*inplace=*/false,
                       pp.inpainting.max_hole_cells);
    if (do_fe)
      applyFeatureExtraction(map_copy, pp.feature_extraction.analysis_radius,
                             pp.feature_extraction.min_valid_neighbors,
                             pp.feature_extraction.step_lower_percentile,
                             pp.feature_extraction.step_upper_percentile,
                             pp.feature_extraction.min_observations,
                             pp.feature_extraction.max_observation_age_frames);

    // Compute derived layer for visualization
    nanogrid::Matrix range_mat =
        map_copy.get(layer::upper_bound) - map_copy.get(layer::lower_bound);
    map_copy.add("uncertainty_range", range_mat);
    const auto processing_finished = std::chrono::steady_clock::now();

    // Publish
    const char* output_layer =
        do_inpainting ? layer::elevation_inpainted : layer::elevation;
    std::optional<sensor_msgs::msg::PointCloud2> cloud_message;
    std::optional<grid_map_msgs::msg::GridMap> gridmap_message;
    std::optional<visualization_msgs::msg::MarkerArray> normals_message;
    double cloud_build_ms = 0.0;
    double gridmap_build_ms = 0.0;
    double normals_build_ms = 0.0;
    if (want_cloud) {
      const auto build_started = std::chrono::steady_clock::now();
      cloud_message = ros2::toPointCloud2(map_copy, output_layer);
      cloud_build_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - build_started).count();
    }
    if (want_gridmap) {
      const auto build_started = std::chrono::steady_clock::now();
      std::vector<std::string> navigation_layers{
        layer::elevation, layer::slope, layer::roughness, layer::step_residual,
        layer::n_points, layer::observation_age};
      const std::string output_layer_name(output_layer);
      if (std::find(navigation_layers.begin(), navigation_layers.end(), output_layer_name) ==
        navigation_layers.end())
      {
        navigation_layers.push_back(output_layer_name);
      }
      gridmap_message = ros2::toGridMap(
        map_copy, output_layer, /*include_surface_normals=*/true, &navigation_layers);
      gridmap_build_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - build_started).count();
    }
    if (want_normals) {
      const auto& nm = cfg_.visualization.feature_extraction.normals;
      const auto build_started = std::chrono::steady_clock::now();
      normals_message = ros2::toNormalMarkers(map_copy, nm.arrow_length, nm.stride);
      normals_build_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - build_started).count();
    }

    const auto publish_lock_requested = std::chrono::steady_clock::now();
    std::unique_lock publish_lock(publish_mutex_);
    const auto publish_lock_acquired = std::chrono::steady_clock::now();
    const auto final_map_lock_requested = std::chrono::steady_clock::now();
    std::shared_lock lock(map_mutex_);
    const auto final_map_lock_acquired = std::chrono::steady_clock::now();
    if (!postprocessSnapshotCanPublish(
        reset_generation, reset_generation_.load(), clock_rollback_pending_.load()))
      return false;
    if (!postprocess_publication_order_.accepts(source_timestamp)) return false;
    if (!isMapFresh(map_copy)) {
      if (!postprocess_publication_order_.canClearStaleSnapshot(
          source_timestamp, generation, map_generation_.load())) return false;
      std::lock_guard state_lock(clock_rollback_state_mutex_);
      if (!postprocessSnapshotCanPublish(
          reset_generation, reset_generation_.load(), clock_rollback_pending_.load()))
        return false;
      if (postprocess_output_valid_.exchange(false)) {
        postprocess_publication_order_.recordPublished(source_timestamp);
        publishEmptyPostProcess(std::move(map_copy), source_timestamp);
      }
      return false;
    }
    lock.unlock();
    const auto state_lock_requested = std::chrono::steady_clock::now();
    std::unique_lock state_lock(clock_rollback_state_mutex_);
    const auto state_lock_acquired = std::chrono::steady_clock::now();
    if (!postprocessSnapshotCanPublish(
        reset_generation, reset_generation_.load(), clock_rollback_pending_.load()))
      return false;
    if (!postprocess_publication_order_.accepts(source_timestamp)) return false;
    const auto publish_started = std::chrono::steady_clock::now();
    double gridmap_publish_ms = 0.0;
    std::size_t gridmap_layer_data_bytes = 0;
    std::size_t gridmap_layer_count = 0;
    const auto gridmap_subscriber_count = pub_post_gridmap_->get_subscription_count();
    if (cloud_message) pub_post_map_->publish(*cloud_message);
    if (gridmap_message) {
      gridmap_layer_count = gridmap_message->data.size();
      for (const auto& layer_data : gridmap_message->data) {
        gridmap_layer_data_bytes += layer_data.data.size() * sizeof(float);
      }
      const auto gridmap_publish_started = std::chrono::steady_clock::now();
      pub_post_gridmap_->publish(*gridmap_message);
      gridmap_publish_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - gridmap_publish_started).count();
    }
    if (normals_message) pub_normals_->publish(*normals_message);
    postprocess_publication_order_.recordPublished(source_timestamp);
    postprocess_output_valid_ = true;
    const auto publish_finished = std::chrono::steady_clock::now();
    const auto milliseconds = [](const auto duration) {
        return std::chrono::duration<double, std::milli>(duration).count();
      };
    if (++postprocess_count_ % 10 == 0) {
      spdlog::info(
        "FastDEM post-process: total={} ms snapshot_lock_wait={} ms snapshot_copy={} ms "
        "filters={} ms build(cloud/gridmap/normals)={}/{}/{} ms "
        "publish_mutex_wait={} ms final_map_lock_wait={} ms state_lock_wait={} ms publish={} ms "
        "gridmap_publish={} ms gridmap_layers={} gridmap_layer_data_bytes={} "
        "gridmap_subscribers={} (gridmap={}, cloud={}, normals={})",
        milliseconds(publish_finished - started),
        milliseconds(snapshot_lock_acquired - snapshot_lock_requested),
        milliseconds(snapshot_finished - snapshot_lock_acquired),
        milliseconds(processing_finished - processing_started),
        cloud_build_ms, gridmap_build_ms, normals_build_ms,
        milliseconds(publish_lock_acquired - publish_lock_requested),
        milliseconds(final_map_lock_acquired - final_map_lock_requested),
        milliseconds(state_lock_acquired - state_lock_requested),
        milliseconds(publish_finished - publish_started), gridmap_publish_ms,
        gridmap_layer_count, gridmap_layer_data_bytes, gridmap_subscriber_count,
        want_gridmap, want_cloud, want_normals);
    }
    return true;
  }

  bool isMapFresh(const ElevationMap& map) {
    const uint64_t timestamp = map.getTimestamp();
    if (timestamp == 0) return false;
    if (cfg_.topics.max_map_age == 0.0) return true;
    const int64_t now = this->get_clock()->now().nanoseconds();
    if (now <= 0) return false;
    if (timestamp > static_cast<uint64_t>(now)) {
      const double clock_lead =
          static_cast<double>(timestamp - static_cast<uint64_t>(now)) * 1e-9;
      return clock_lead <= cfg_.topics.max_future_stamp_offset;
    }
    const double age = static_cast<double>(static_cast<uint64_t>(now) - timestamp) *
                       1e-9;
    return age <= cfg_.topics.max_map_age;
  }

  bool isSourceTimestampPlausible(uint64_t timestamp) {
    if (timestamp == 0) return false;
    const int64_t now = this->get_clock()->now().nanoseconds();
    if (now <= 0) return false;
    if (timestamp <= static_cast<uint64_t>(now)) return true;
    const double clock_lead =
        static_cast<double>(timestamp - static_cast<uint64_t>(now)) * 1e-9;
    return clock_lead <= cfg_.topics.max_future_stamp_offset;
  }

  bool handleClockRollback() {
    const int64_t now = this->get_clock()->now().nanoseconds();
    if (now <= 0) return !clock_rollback_pending_.load();
    if (clock_rollback_pending_.load()) {
      last_clock_time_.store(now);
      return completeClockRollback();
    }
    const int64_t previous = last_clock_time_.exchange(now);
    const int64_t tolerance = static_cast<int64_t>(
        cfg_.pipeline.mapping.max_timestamp_regression * 1e9);
    if (previous <= 0 || now + tolerance >= previous) return true;

    {
      std::lock_guard state_lock(clock_rollback_state_mutex_);
      ++clock_rollback_generation_;
      clock_rollback_pending_ = true;
      ++map_generation_;
      ++reset_generation_;
    }
    return completeClockRollback();
  }

  bool completeClockRollback() {
    if (!clock_rollback_pending_.load()) return true;
    std::unique_lock publish_lock(publish_mutex_, std::try_to_lock);
    if (!publish_lock.owns_lock()) return false;
    if (!clock_rollback_pending_.load()) return true;

    const auto rollback_generation = clock_rollback_generation_.load();
    std::unique_lock lock(map_mutex_);
    mapper_->reset();
    ++map_generation_;
    auto empty_map = map_.snapshot(
        {layer::elevation, layer::upper_bound, layer::lower_bound,
         layer::n_points, layer::observation_age});
    lock.unlock();
    publishEmptyMapOutputs(std::move(empty_map));
    spdlog::warn("ROS clock moved backwards; FastDEM map reset.");
    {
      std::lock_guard state_lock(clock_rollback_state_mutex_);
      if (clock_rollback_generation_.load() == rollback_generation) {
        clock_rollback_pending_ = false;
      }
    }
    return !clock_rollback_pending_.load();
  }

  void clockRollbackMaintenanceCallback() {
    if (clock_rollback_pending_.load()) completeClockRollback();
  }

  void publishEmptyPostProcess(
      ElevationMap map, std::optional<uint64_t> source_timestamp = std::nullopt) {
    map.clearAll();
    map.setTimestamp(source_timestamp.value_or(
        static_cast<uint64_t>(this->get_clock()->now().nanoseconds())));
    for (const auto* required_layer : {
        layer::slope, layer::step_residual, layer::roughness,
        layer::n_points, layer::observation_age}) {
      if (!map.exists(required_layer)) map.add(required_layer, NAN);
    }
    if (!map.exists(layer::elevation_inpainted))
      map.add(layer::elevation_inpainted, NAN);
    if (!map.exists(layer::inpainted_mask))
      map.add(layer::inpainted_mask, NAN);
    if (pub_post_map_->get_subscription_count() > 0)
      pub_post_map_->publish(
          ros2::toPointCloud2(map, layer::elevation_inpainted));
    if (pub_post_gridmap_->get_subscription_count() > 0 &&
        cfg_.pipeline.mapping.mode != MappingMode::GLOBAL)
      pub_post_gridmap_->publish(
          ros2::toGridMap(map, layer::elevation_inpainted));
    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = map.getFrameId();
    marker.header.stamp = this->get_clock()->now();
    marker.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(marker);
    pub_normals_->publish(markers);
  }

  void publishEmptyMapOutputs(ElevationMap map) {
    map.clearAll();
    map.setTimestamp(static_cast<uint64_t>(this->get_clock()->now().nanoseconds()));
    if (pub_map_->get_subscription_count() > 0)
      pub_map_->publish(ros2::toPointCloud2(map));
    if (pub_global_map_->get_subscription_count() > 0)
      pub_global_map_->publish(ros2::toPointCloud2(map));
    if (pub_gridmap_->get_subscription_count() > 0 &&
        cfg_.pipeline.mapping.mode != MappingMode::GLOBAL)
      pub_gridmap_->publish(ros2::toGridMap(map));
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = map.getFrameId();
    marker.header.stamp = this->get_clock()->now();
    marker.action = visualization_msgs::msg::Marker::DELETEALL;
    pub_boundary_->publish(marker);
    postprocess_publication_order_.reset();
    publishEmptyPostProcess(std::move(map));
    map_output_valid_ = false;
    postprocess_output_valid_ = false;
  }

  // ==================== Publishers ====================

  void publishLocalView() {
    if (clock_rollback_pending_.load()) return;
    const bool is_global = cfg_.pipeline.mapping.mode == MappingMode::GLOBAL;
    const bool want_cloud = pub_map_->get_subscription_count() > 0;
    const bool want_gridmap = pub_gridmap_->get_subscription_count() > 0 && !is_global;
    const bool want_boundary = pub_boundary_->get_subscription_count() > 0;
    if (!want_cloud && !want_gridmap && !want_boundary) return;

    std::lock_guard publish_lock(publish_mutex_);
    if (clock_rollback_pending_.load()) return;
    ElevationMap snapshot;
    uint64_t generation;
    {
      std::shared_lock lock(map_mutex_);
      snapshot = map_;
      generation = map_generation_.load();
    }
    if (snapshot.isEmpty()) {
      if (map_output_valid_.exchange(false))
        publishEmptyMapOutputs(std::move(snapshot));
      return;
    }
    std::optional<sensor_msgs::msg::PointCloud2> cloud_message;
    std::optional<grid_map_msgs::msg::GridMap> gridmap_message;
    std::optional<visualization_msgs::msg::Marker> boundary_message;
    if (want_cloud) {
      if (is_global) {
        auto pose = tf_->getPoseAt(snapshot.getTimestamp());
        if (pose) cloud_message = ros2::toPointCloud2(
            snapshot, pose->translation().head<2>(), {15.0, 15.0});
      } else {
        cloud_message = ros2::toPointCloud2(snapshot);
      }
    }
    if (want_gridmap) gridmap_message = ros2::toGridMap(snapshot);
    if (want_boundary) boundary_message = ros2::toMapBoundary(snapshot);
    std::lock_guard state_lock(clock_rollback_state_mutex_);
    if (clock_rollback_pending_.load() || generation != map_generation_.load()) return;
    if (cloud_message) pub_map_->publish(*cloud_message);
    if (gridmap_message) pub_gridmap_->publish(*gridmap_message);
    if (boundary_message) pub_boundary_->publish(*boundary_message);
    map_output_valid_ = true;
  }

  void publishGlobalView() {
    if (clock_rollback_pending_.load()) return;
    if (pub_global_map_->get_subscription_count() == 0) return;
    std::lock_guard publish_lock(publish_mutex_);
    if (clock_rollback_pending_.load()) return;
    ElevationMap snapshot;
    uint64_t generation;
    {
      std::shared_lock lock(map_mutex_);
      snapshot = map_;
      generation = map_generation_.load();
    }
    if (snapshot.isEmpty()) {
      if (map_output_valid_.exchange(false))
        publishEmptyMapOutputs(std::move(snapshot));
      return;
    }
    auto cloud_message = ros2::toPointCloud2(snapshot);
    std::lock_guard state_lock(clock_rollback_state_mutex_);
    if (clock_rollback_pending_.load() || generation != map_generation_.load()) return;
    pub_global_map_->publish(cloud_message);
    map_output_valid_ = true;
  }

  void publishPendingScans() {
    std::optional<PointCloud> processed;
    std::optional<PointCloud> rasterized;
    uint64_t reset_generation;
    {
      std::lock_guard queue_lock(scan_output_mutex_);
      processed.swap(queued_processed_);
      rasterized.swap(queued_rasterized_);
      reset_generation = queued_reset_generation_;
    }
    if (!processed && !rasterized) return;
    std::optional<sensor_msgs::msg::PointCloud2> processed_msg;
    std::optional<sensor_msgs::msg::PointCloud2> rasterized_msg;
    if (processed && pub_scan_->get_subscription_count() > 0)
      processed_msg = nanopcl::to(*processed);
    if (rasterized && pub_rasterized_->get_subscription_count() > 0)
      rasterized_msg = nanopcl::to(*rasterized);
    std::lock_guard publish_lock(publish_mutex_);
    std::lock_guard state_lock(clock_rollback_state_mutex_);
    if (clock_rollback_pending_.load() || reset_generation != reset_generation_.load()) return;
    if (processed_msg) pub_scan_->publish(*processed_msg);
    if (rasterized_msg) pub_rasterized_->publish(*rasterized_msg);
  }

  void printNodeSummary() const {
    auto mode_str = [](MappingMode m) {
      return m == MappingMode::LOCAL ? "local" : "global";
    };
    auto est_str = [](EstimationType e) {
      return e == EstimationType::Kalman ? "kalman_filter" : "p2_quantile";
    };
    auto sensor_str = [](SensorType s) {
      switch (s) {
        case SensorType::Constant:
          return "constant";
        case SensorType::LiDAR:
          return "lidar";
        case SensorType::RGBD:
          return "rgbd";
        default:
          return "unknown";
      }
    };

    std::string postproc;
    const auto& pp = cfg_.postprocess;
    if (pp.uncertainty_fusion.enabled) postproc += "uncertainty_fusion, ";
    if (pp.inpainting.enabled) postproc += "inpainting, ";
    if (pp.feature_extraction.enabled) postproc += "feature_extraction, ";
    if (!postproc.empty())
      postproc.erase(postproc.size() - 2);
    else
      postproc = "none";

    const auto& m = cfg_.pipeline.mapping;

    spdlog::info("");
    spdlog::info("===== FastDEM Mapping Node =====");
    spdlog::info("  Map        : {} ({:.1f} x {:.1f} m, res={:.2f} m)",
                 mode_str(m.mode), cfg_.map.width, cfg_.map.height,
                 cfg_.map.resolution);
    spdlog::info("  Estimator  : {}", est_str(m.estimation_type));
    spdlog::info("  Sensor     : {}",
                 sensor_str(cfg_.pipeline.sensor_model.type));
    spdlog::info("  Raycasting : {}",
                 cfg_.pipeline.raycasting.enabled ? "on" : "off");
    spdlog::info("  Post-proc  : {}", postproc);
    spdlog::info("  TF         : {} -> {}", cfg_.tf.base_frame,
                 cfg_.tf.map_frame);
    if (cfg_.topics.input_scans.size() == 1) {
      spdlog::info("  Input      : {}", cfg_.topics.input_scans[0]);
    } else {
      for (size_t i = 0; i < cfg_.topics.input_scans.size(); ++i)
        spdlog::info("  Input [{}]  : {}", i, cfg_.topics.input_scans[i]);
    }
    if (postproc != "none")
      spdlog::info("  Pub rate   : {} Hz (post: {} Hz)",
                   cfg_.topics.publish_rate, cfg_.topics.post_process_rate);
    else
      spdlog::info("  Pub rate   : {} Hz", cfg_.topics.publish_rate);
    spdlog::info("================================");
    spdlog::info("");
  }

  NodeConfig cfg_;

  // Core objects
  ElevationMap map_;
  std::unique_ptr<FastDEM> mapper_;
  std::shared_ptr<TFBridge> tf_;
  mutable std::shared_mutex map_mutex_;
  std::mutex publish_mutex_;
  std::mutex scan_output_mutex_;
  std::optional<PointCloud> queued_processed_;
  std::optional<PointCloud> queued_rasterized_;
  uint64_t queued_reset_generation_{0};
  std::optional<PointCloud> pending_processed_;
  std::optional<PointCloud> pending_rasterized_;
  std::uint64_t scan_timing_count_{0};

  // clang-format off
  // ROS2 handles
  std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> scan_subs_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr    pub_scan_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr    pub_rasterized_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr    pub_map_;
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr      pub_gridmap_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr  pub_boundary_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr    pub_global_map_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr    pub_post_map_;
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr      pub_post_gridmap_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_normals_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_reset_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_postprocess_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_inpainting_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_uf_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_fe_;
  rclcpp::TimerBase::SharedPtr timer_scan_output_;
  rclcpp::TimerBase::SharedPtr timer_clock_rollback_;
  rclcpp::TimerBase::SharedPtr timer_viz_;
  rclcpp::TimerBase::SharedPtr timer_global_;
  rclcpp::TimerBase::SharedPtr timer_post_process_;
  rclcpp::CallbackGroup::SharedPtr scan_group_;
  rclcpp::CallbackGroup::SharedPtr postprocess_group_;
  rclcpp::CallbackGroup::SharedPtr publish_group_;
  rclcpp::CallbackGroup::SharedPtr service_group_;
  std::atomic_bool postprocess_output_valid_{false};
  // Accessed only while publish_mutex_ is held.
  PostprocessPublicationOrder postprocess_publication_order_;
  std::atomic_bool map_output_valid_{false};
  std::atomic<uint64_t> map_generation_{0};
  std::atomic<uint64_t> reset_generation_{0};
  std::atomic<int64_t> last_clock_time_{0};
  std::atomic_bool clock_rollback_pending_{false};
  std::atomic<std::uint64_t> clock_rollback_generation_{0};
  std::mutex clock_rollback_state_mutex_;
  std::atomic<std::uint64_t> postprocess_count_{0};
  // clang-format on
};

}  // namespace fastdem::ros2

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<fastdem::ros2::MappingNode>();
  if (!node->initialize()) {
    return 1;
  }

  // Multi-threaded executor for parallel processing:
  // Thread 1: Point cloud processing (mapping)
  // Thread 2: Async post-processing (inpainting, feature extraction)
  // Thread 3: Map publishing (visualization)
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(),
                                                    3);
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
