#include <algorithm>
#include <array>
#include <atomic>
#include <cinttypes>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <exception>
#include <functional>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2/exceptions.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <small_gicp/ann/kdtree_omp.hpp>
#include <small_gicp/ann/traits.hpp>
#include <small_gicp/factors/gicp_factor.hpp>
#include <small_gicp_relocalization/relocalization_hysteresis.hpp>
#include <small_gicp_relocalization/registration_termination.hpp>
#include <small_gicp_relocalization/relocalization_generation.hpp>
#include <small_gicp_relocalization/snapshot_storage.hpp>
#include <small_gicp_relocalization/pose_utils.hpp>
#include <small_gicp_relocalization/manual_seed_deadline.hpp>
#include <small_gicp_relocalization/fastlio_health.hpp>
#include <small_gicp_relocalization/msg/map_odom_target.hpp>
#include <small_gicp_relocalization/msg/relocalization_status.hpp>
#include <small_gicp/pcl/pcl_point.hpp>
#include <small_gicp/pcl/pcl_point_traits.hpp>
#include <small_gicp/registration/registration.hpp>
#include <small_gicp/registration/reduction_omp.hpp>
#include <small_gicp/util/downsampling_omp.hpp>
#include <small_gicp/util/normal_estimation_omp.hpp>

namespace {

using CovarianceCloud = pcl::PointCloud<pcl::PointCovariance>;
using CovarianceTree = small_gicp::KdTree<CovarianceCloud>;

struct MatchCoverage {
  std::size_t matched_points{0};
  std::size_t sectors{0};
  double ratio{0.0};
  double rmse_m{std::numeric_limits<double>::infinity()};
  double p95_m{std::numeric_limits<double>::infinity()};
};

Eigen::Isometry3d toEigen(const geometry_msgs::msg::TransformStamped &transform) {
  return tf2::transformToEigen(transform);
}

Eigen::Isometry3d poseToIsometry(const geometry_msgs::msg::Pose &pose) {
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.translation() = Eigen::Vector3d(
    pose.position.x, pose.position.y, pose.position.z);
  result.linear() = Eigen::Quaterniond(
    pose.orientation.w, pose.orientation.x, pose.orientation.y,
    pose.orientation.z).normalized().toRotationMatrix();
  return result;
}

geometry_msgs::msg::TransformStamped toTransform(
  const Eigen::Isometry3d &transform, const std::string &parent, const std::string &child,
  const rclcpp::Time &stamp) {
  geometry_msgs::msg::TransformStamped message;
  message.header.stamp = stamp;
  message.header.frame_id = parent;
  message.child_frame_id = child;
  message.transform.translation.x = transform.translation().x();
  message.transform.translation.y = transform.translation().y();
  message.transform.translation.z = transform.translation().z();
  const Eigen::Quaterniond rotation(transform.rotation());
  message.transform.rotation.x = rotation.x();
  message.transform.rotation.y = rotation.y();
  message.transform.rotation.z = rotation.z();
  message.transform.rotation.w = rotation.w();
  return message;
}

}  // namespace

class SmallGicpRelocalizationNode final : public rclcpp::Node {
public:
  SmallGicpRelocalizationNode()
  : Node("small_gicp_relocalization"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_) {
    producer_session_id_ = static_cast<std::uint64_t>(
      std::chrono::steady_clock::now().time_since_epoch().count());
    if (producer_session_id_ == 0) {
      producer_session_id_ = 1;
    }
    pcd_file_ = declare_parameter<std::string>("pcd_file", "");
    cloud_topic_ = declare_parameter<std::string>(
      "cloud_topic", "/cloud_registered_body_downsampled");
    initial_pose_topic_ = declare_parameter<std::string>("initial_pose_topic", "/initialpose");
    bbs_global_pose_topic_ = declare_parameter<std::string>("bbs_global_pose_topic", "");
    bbs_seed_retry_sec_ = declare_parameter<double>("bbs_seed_retry_sec", 15.0);
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    body_frame_ = declare_parameter<std::string>("body_frame", "body");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    registration_voxel_size_ = declare_parameter<double>("registration_voxel_size", 0.20);
    tracking_input_voxel_size_ = declare_parameter<double>("tracking_input_voxel_size", 0.25);
    display_voxel_size_ = declare_parameter<double>("display_voxel_size", 0.10);
    num_neighbors_ = declare_parameter<int>("num_neighbors", 20);
    num_threads_ = declare_parameter<int>("num_threads", 2);
    max_correspondence_distance_ = declare_parameter<double>("max_correspondence_distance", 1.0);
    max_iterations_ = declare_parameter<int>("max_iterations", 60);
    tracking_frequency_hz_ = declare_parameter<double>("tracking_frequency_hz", 1.0);
    tracking_max_iterations_ = declare_parameter<int>("tracking_max_iterations", 25);
    tracking_max_processing_time_sec_ = declare_parameter<double>(
      "tracking_max_processing_time_sec", 0.8);
    tracking_small_correction_translation_m_ = declare_parameter<double>(
      "tracking_small_correction_translation_m", 0.05);
    tracking_small_correction_rotation_rad_ = declare_parameter<double>(
      "tracking_small_correction_rotation_rad", 0.03);
    tracking_consistency_translation_tolerance_m_ = declare_parameter<double>(
      "tracking_consistency_translation_tolerance_m", 0.10);
    tracking_consistency_rotation_tolerance_rad_ = declare_parameter<double>(
      "tracking_consistency_rotation_tolerance_rad", 0.05);
    tracking_consistency_frames_required_ = declare_parameter<int>(
      "tracking_consistency_frames_required", 2);
    tracking_max_translation_correction_ = declare_parameter<double>(
      "tracking_max_translation_correction", 0.50);
    tracking_max_rotation_correction_ = declare_parameter<double>(
      "tracking_max_rotation_correction", 0.15);
    tracking_max_cloud_age_sec_ = declare_parameter<double>("tracking_max_cloud_age_sec", 1.0);
    min_inliers_ = declare_parameter<int>("min_inliers", 50);
    max_translation_jump_ = declare_parameter<double>("max_translation_jump", 2.0);
    max_rotation_jump_ = declare_parameter<double>("max_rotation_jump", 0.50);
    min_inlier_ratio_ = declare_parameter<double>("min_inlier_ratio", 0.35);
    max_geometry_rmse_m_ = declare_parameter<double>("max_geometry_rmse_m", 0.18);
    max_geometry_p95_m_ = declare_parameter<double>("max_geometry_p95_m", 0.38);
    stable_frames_required_ = declare_parameter<int>("stable_frames_required", 4);
    min_stable_duration_sec_ = declare_parameter<double>("stable_duration", 0.30);
    max_stable_translation_spread_ = declare_parameter<double>("max_stable_translation_spread", 0.05);
    max_stable_yaw_spread_ = declare_parameter<double>("max_stable_yaw_spread", 0.03);
    min_match_sectors_ = declare_parameter<int>("min_match_sectors", 5);
    require_multistart_consistency_ = declare_parameter<bool>("require_multistart_consistency", true);
    multistart_seed_translation_ = declare_parameter<double>("multistart_seed_translation", 0.25);
    multistart_seed_yaw_ = declare_parameter<double>("multistart_seed_yaw", 0.15);
    max_multistart_translation_spread_ = declare_parameter<double>(
      "max_multistart_translation_spread", 0.15);
    max_multistart_yaw_spread_ = declare_parameter<double>("max_multistart_yaw_spread", 0.05);
    validity_timeout_sec_ = declare_parameter<double>("validity_timeout", 10.0);
    max_consecutive_failures_ = declare_parameter<int>("max_consecutive_failures", 3);
    tf_lookup_timeout_sec_ = declare_parameter<double>("tf_lookup_timeout_sec", 0.05);
    manual_seed_input_timeout_sec_ = declare_parameter<double>(
      "manual_seed_input_timeout_sec", 3.0);
    manual_seed_validation_timeout_sec_ = declare_parameter<double>(
      "manual_seed_validation_timeout_sec", 10.0);
    manual_seed_bbs_height_max_age_sec_ = declare_parameter<double>(
      "manual_seed_bbs_height_max_age_sec", 1.0);
    timing_log_period_ms_ = declare_parameter<int>("timing_log_period_ms", 1000);
    offline_snapshot_dir_ = declare_parameter<std::string>("offline_snapshot_dir", "");
    navigation_map_file_ = declare_parameter<std::string>("navigation_map_file", "");
    startup_context_ = declare_parameter<std::string>("startup_context", "");
    offline_snapshot_max_count_ = declare_parameter<int>("offline_snapshot_max_count", 8);
    handoff_topic_ = declare_parameter<std::string>(
      "handoff_topic", "/localization/map_odom_handoff");
    exit_after_localized_ = declare_parameter<bool>("exit_after_localized", false);
    fastlio_health_topic_ = declare_parameter<std::string>("fastlio_health_topic", "/fastlio/health");
    fastlio_health_timeout_sec_ = declare_parameter<double>("fastlio_health_timeout_sec", 0.5);
    if (!std::isfinite(min_inlier_ratio_) || !std::isfinite(max_geometry_rmse_m_) ||
      !std::isfinite(max_geometry_p95_m_) ||
      !std::isfinite(min_stable_duration_sec_) ||
      !std::isfinite(max_stable_translation_spread_) || !std::isfinite(max_stable_yaw_spread_) ||
      !std::isfinite(multistart_seed_translation_) || !std::isfinite(multistart_seed_yaw_) ||
      !std::isfinite(max_multistart_translation_spread_) || !std::isfinite(max_multistart_yaw_spread_) ||
      !std::isfinite(validity_timeout_sec_) || !std::isfinite(fastlio_health_timeout_sec_) ||
      !std::isfinite(bbs_seed_retry_sec_) || !std::isfinite(tracking_input_voxel_size_) ||
      !std::isfinite(tracking_frequency_hz_) ||
      !std::isfinite(tracking_max_translation_correction_) ||
      !std::isfinite(tracking_max_rotation_correction_) ||
      !std::isfinite(tracking_max_cloud_age_sec_) ||
      !std::isfinite(tracking_max_processing_time_sec_) ||
      !std::isfinite(manual_seed_input_timeout_sec_) ||
      !std::isfinite(manual_seed_validation_timeout_sec_) ||
      !std::isfinite(manual_seed_bbs_height_max_age_sec_) ||
      !std::isfinite(tracking_small_correction_translation_m_) ||
      !std::isfinite(tracking_small_correction_rotation_rad_) ||
      !std::isfinite(tracking_consistency_translation_tolerance_m_) ||
      !std::isfinite(tracking_consistency_rotation_tolerance_rad_))
    {
      throw std::invalid_argument("relocalization quality parameters must be finite");
    }
    registration_voxel_size_ = std::max(registration_voxel_size_, 1e-3);
    tracking_input_voxel_size_ = std::max(tracking_input_voxel_size_, 1e-3);
    display_voxel_size_ = std::max(display_voxel_size_, 1e-3);
    num_neighbors_ = std::max(num_neighbors_, 5);
    num_threads_ = std::max(num_threads_, 1);
    max_correspondence_distance_ = std::max(max_correspondence_distance_, 1e-3);
    max_iterations_ = std::max(max_iterations_, 1);
    tracking_frequency_hz_ = std::max(tracking_frequency_hz_, 0.01);
    tracking_max_iterations_ = std::max(tracking_max_iterations_, 1);
    tracking_max_translation_correction_ = std::max(tracking_max_translation_correction_, 0.0);
    tracking_max_rotation_correction_ = std::max(tracking_max_rotation_correction_, 0.0);
    tracking_max_cloud_age_sec_ = std::max(tracking_max_cloud_age_sec_, 0.01);
    tracking_max_processing_time_sec_ = std::max(tracking_max_processing_time_sec_, 0.01);
    tracking_small_correction_translation_m_ = std::max(
      tracking_small_correction_translation_m_, 0.0);
    tracking_small_correction_rotation_rad_ = std::max(
      tracking_small_correction_rotation_rad_, 0.0);
    tracking_consistency_translation_tolerance_m_ = std::max(
      tracking_consistency_translation_tolerance_m_, 0.0);
    tracking_consistency_rotation_tolerance_rad_ = std::max(
      tracking_consistency_rotation_tolerance_rad_, 0.0);
    tracking_consistency_frames_required_ = std::max(tracking_consistency_frames_required_, 2);
    min_inliers_ = std::max(min_inliers_, 1);
    max_translation_jump_ = std::max(max_translation_jump_, 0.0);
    max_rotation_jump_ = std::max(max_rotation_jump_, 0.0);
    min_inlier_ratio_ = std::clamp(min_inlier_ratio_, 0.0, 1.0);
    max_geometry_rmse_m_ = std::max(max_geometry_rmse_m_, 0.0);
    max_geometry_p95_m_ = std::max(max_geometry_p95_m_, 0.0);
    stable_frames_required_ = std::max(stable_frames_required_, 1);
    min_stable_duration_sec_ = std::max(min_stable_duration_sec_, 0.0);
    max_stable_translation_spread_ = std::max(max_stable_translation_spread_, 0.0);
    max_stable_yaw_spread_ = std::max(max_stable_yaw_spread_, 0.0);
    min_match_sectors_ = std::clamp(min_match_sectors_, 1, 8);
    multistart_seed_translation_ = std::max(multistart_seed_translation_, 0.0);
    multistart_seed_yaw_ = std::max(multistart_seed_yaw_, 0.0);
    max_multistart_translation_spread_ = std::max(max_multistart_translation_spread_, 0.0);
    max_multistart_yaw_spread_ = std::max(max_multistart_yaw_spread_, 0.0);
    validity_timeout_sec_ = std::max(validity_timeout_sec_, 0.01);
    max_consecutive_failures_ = std::max(max_consecutive_failures_, 1);
    tf_lookup_timeout_sec_ = std::max(tf_lookup_timeout_sec_, 0.0);
    manual_seed_input_timeout_sec_ = std::max(manual_seed_input_timeout_sec_, 0.1);
    manual_seed_validation_timeout_sec_ = std::max(manual_seed_validation_timeout_sec_, 0.1);
    manual_seed_bbs_height_max_age_sec_ = std::max(manual_seed_bbs_height_max_age_sec_, 0.0);
    offline_snapshot_max_count_ = std::clamp(offline_snapshot_max_count_, 1, 32);
    bbs_seed_retry_sec_ = std::max(bbs_seed_retry_sec_, 0.1);
    timing_log_period_ms_ = std::max(timing_log_period_ms_, 1);
    fastlio_health_timeout_sec_ = std::max(fastlio_health_timeout_sec_, 0.01);

    prior_map_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/small_gicp/prior_map", rclcpp::QoS(1).transient_local());
    prior_map_display_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/small_gicp/prior_map_display", rclcpp::QoS(1).transient_local());
    aligned_cloud_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/small_gicp/aligned_cloud", rclcpp::QoS(10).reliable());
    pose_publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/small_gicp/pose", rclcpp::QoS(10));
    candidate_pose_publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/small_gicp/candidate_pose", rclcpp::QoS(10));
    status_publisher_ = create_publisher<small_gicp_relocalization::msg::RelocalizationStatus>(
      "/small_gicp/relocalization_status", rclcpp::QoS(10));
    handoff_publisher_ = create_publisher<small_gicp_relocalization::msg::MapOdomTarget>(
      handoff_topic_, rclcpp::QoS(1).reliable().transient_local());
    preview_map_odom_publisher_ =
      create_publisher<geometry_msgs::msg::TransformStamped>(
      "/small_gicp/preview_map_odom", rclcpp::QoS(1).reliable().transient_local());

    initial_pose_callback_group_ = create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions initial_pose_subscription_options;
    initial_pose_subscription_options.callback_group = initial_pose_callback_group_;
    initial_pose_subscription_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      initial_pose_topic_, rclcpp::QoS(10),
      std::bind(&SmallGicpRelocalizationNode::initialPoseCallback, this, std::placeholders::_1),
      initial_pose_subscription_options);
    cancel_manual_seed_service_ = create_service<std_srvs::srv::Trigger>(
      "/small_gicp/cancel_manual_seed",
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
      {
        response->success = cancelManualSeedAndResumeAuto("service request");
        response->message = response->success ?
          "Manual seed cancelled; automatic BBS relocalization is active" :
          "No active manual seed";
      });
    if (!bbs_global_pose_topic_.empty()) {
      bbs_global_pose_subscription_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        bbs_global_pose_topic_, rclcpp::QoS(10),
        std::bind(&SmallGicpRelocalizationNode::bbsGlobalPoseCallback, this, std::placeholders::_1),
        initial_pose_subscription_options);
    }
    cloud_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic_, rclcpp::SensorDataQoS().keep_last(1),
      std::bind(&SmallGicpRelocalizationNode::cloudCallback, this, std::placeholders::_1));
    odometry_topic_ = declare_parameter<std::string>("odometry_topic", "/Odometry");
    odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      odometry_topic_, rclcpp::QoS(64).best_effort(),
      [this](const nav_msgs::msg::Odometry::SharedPtr message) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        odometry_history_.push_back(*message);
        while (odometry_history_.size() > 64) odometry_history_.pop_front();
      });
    dynamic_tf_subscription_ = create_subscription<tf2_msgs::msg::TFMessage>(
      "/tf", rclcpp::QoS(128).best_effort(),
      [this](const tf2_msgs::msg::TFMessage::SharedPtr message) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        dynamic_tf_history_.push_back(*message);
        while (dynamic_tf_history_.size() > 128) dynamic_tf_history_.pop_front();
      });
    static_tf_subscription_ = create_subscription<tf2_msgs::msg::TFMessage>(
      "/tf_static", rclcpp::QoS(1).reliable().transient_local(),
      [this](const tf2_msgs::msg::TFMessage::SharedPtr message) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        static_tf_snapshot_ = *message;
        have_static_tf_snapshot_ = true;
      });
    fastlio_health_callback_group_ = create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions health_subscription_options;
    health_subscription_options.callback_group = fastlio_health_callback_group_;
    fastlio_health_subscription_ = create_subscription<std_msgs::msg::Bool>(
      fastlio_health_topic_, rclcpp::QoS(1),
      [this](const std_msgs::msg::Bool::SharedPtr message) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        fastlio_healthy_ = message->data;
        has_fastlio_health_ = true;
        last_fastlio_health_wall_time_ = std::chrono::steady_clock::now();
      }, health_subscription_options);

    heartbeat_callback_group_ = create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
    heartbeat_timer_ = create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&SmallGicpRelocalizationNode::heartbeatCallback, this),
      heartbeat_callback_group_);

    map_load_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    map_load_timer_ = create_wall_timer(
      std::chrono::milliseconds(10),
      [this]() {
        map_load_timer_->cancel();
        loadMap();
      },
      map_load_callback_group_);
    if (!offline_snapshot_dir_.empty()) {
      snapshot_queue_.push_back([this]() {
        try {
          const auto root = std::filesystem::path(offline_snapshot_dir_);
          const auto removed = small_gicp_relocalization::pruneSnapshotDirectories(
            root, static_cast<std::size_t>(offline_snapshot_max_count_));
          if (!pcd_file_.empty()) {
            const auto map_version = small_gicp_relocalization::snapshotMapVersionKey(
              pcd_file_, navigation_map_file_);
            small_gicp_relocalization::pruneSnapshotMapVersions(root, map_version);
          }
          if (!removed.empty()) {
            RCLCPP_INFO(
              get_logger(), "Pruned %zu old offline GICP snapshots across sessions", removed.size());
          }
        } catch (const std::exception & exception) {
          RCLCPP_WARN(
            get_logger(), "Failed to prune old offline GICP snapshots: %s", exception.what());
        }
      });
    }
    snapshot_worker_thread_ = std::thread(
      std::bind(&SmallGicpRelocalizationNode::snapshotWorkerLoop, this));
    snapshot_queue_condition_.notify_one();
  }

  ~SmallGicpRelocalizationNode() override
  {
    {
      std::lock_guard<std::mutex> lock(snapshot_queue_mutex_);
      snapshot_worker_stopping_ = true;
    }
    snapshot_queue_condition_.notify_all();
    if (snapshot_worker_thread_.joinable()) {
      snapshot_worker_thread_.join();
    }
  }

private:
  void loadMap() {
    const auto map_load_start = std::chrono::steady_clock::now();
    if (pcd_file_.empty()) {
      RCLCPP_FATAL(get_logger(), "Required parameter pcd_file is empty");
      throw std::runtime_error("Required parameter pcd_file is empty");
    }
    std::error_code path_error;
    if (!std::filesystem::is_regular_file(pcd_file_, path_error)) {
      RCLCPP_FATAL(
        get_logger(), "pcd_file must be a readable regular file, got: %s", pcd_file_.c_str());
      throw std::runtime_error("pcd_file must be a regular PCD file: " + pcd_file_);
    }
    pcl::PointCloud<pcl::PointXYZ> raw_map;
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file_, raw_map) != 0 || raw_map.empty()) {
      RCLCPP_FATAL(get_logger(), "Failed to load non-empty PCD map: %s", pcd_file_.c_str());
      throw std::runtime_error("Failed to load PCD map: " + pcd_file_);
    }

    pcl::VoxelGrid<pcl::PointXYZ> display_filter;
    display_filter.setInputCloud(raw_map.makeShared());
    const float display_leaf_size = static_cast<float>(display_voxel_size_);
    display_filter.setLeafSize(display_leaf_size, display_leaf_size, display_leaf_size);
    pcl::PointCloud<pcl::PointXYZ> display_cloud;
    display_filter.filter(display_cloud);
    const double display_downsample_ms = elapsedMilliseconds(map_load_start);
    sensor_msgs::msg::PointCloud2 prior_map;
    pcl::toROSMsg(display_cloud, prior_map);
    prior_map.header.frame_id = map_frame_;
    prior_map.header.stamp = now();
    prior_map_publisher_->publish(prior_map);
    prior_map_display_publisher_->publish(prior_map);
    RCLCPP_INFO(
      get_logger(),
      "Published display map: %zu raw points -> %zu points (%.3f m); preparing GICP map",
      raw_map.size(), display_cloud.size(), display_voxel_size_);

    map_cloud_ = small_gicp::voxelgrid_sampling_omp<pcl::PointCloud<pcl::PointXYZ>, CovarianceCloud>(
      raw_map, registration_voxel_size_, num_threads_);
    const double map_downsample_ms = elapsedMilliseconds(map_load_start) - display_downsample_ms;
    if (map_cloud_->size() < static_cast<size_t>(std::max(num_neighbors_, 5))) {
      RCLCPP_FATAL(
        get_logger(), "Map has too few points after downsampling: %zu", map_cloud_->size());
      throw std::runtime_error("PCD map has too few points after downsampling");
    }

    small_gicp::estimate_covariances_omp(*map_cloud_, num_neighbors_, num_threads_);
    const double map_covariance_ms = elapsedMilliseconds(map_load_start) -
      display_downsample_ms - map_downsample_ms;
    map_tree_ = std::make_shared<CovarianceTree>(
      map_cloud_, small_gicp::KdTreeBuilderOMP(num_threads_));
    const double map_kdtree_ms = elapsedMilliseconds(map_load_start) - display_downsample_ms -
      map_downsample_ms - map_covariance_ms;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      map_ready_.store(true, std::memory_order_release);
      if (relocalization_state_ ==
        small_gicp_relocalization::msg::RelocalizationStatus::MAP_LOADING)
      {
        relocalization_state_ = has_initial_pose_
          ? small_gicp_relocalization::msg::RelocalizationStatus::VERIFYING
          : small_gicp_relocalization::msg::RelocalizationStatus::WAITING_INITIAL_POSE;
      }
      publishStatusLocked(now());
    }

    RCLCPP_INFO(
      get_logger(),
      "Loaded map %s: %zu raw points, %zu registration points (%.3f m), "
      "%zu display points (%.3f m)",
      pcd_file_.c_str(), raw_map.size(), map_cloud_->size(), registration_voxel_size_,
      display_cloud.size(), display_voxel_size_);
    RCLCPP_INFO(
      get_logger(),
      "Map timing: display_downsample=%.1fms registration_downsample=%.1fms "
      "covariance=%.1fms kdtree=%.1fms total=%.1fms threads=%d",
      display_downsample_ms, map_downsample_ms, map_covariance_ms, map_kdtree_ms,
      elapsedMilliseconds(map_load_start), num_threads_);
    tryApplyPendingInitialPose("map loaded");
  }

  void resetInitializationLocked(
    const geometry_msgs::msg::Pose & pose, bool pose_is_map_body,
    const builtin_interfaces::msg::Time & pose_stamp, bool manual_seed, const char * source)
  {
    pending_initial_pose_ = pose;
    seed_initial_pose_ = pose;
    seed_initial_pose_covariance_.fill(0.0);
    seed_initial_pose_is_map_body_ = pose_is_map_body;
    pending_pose_is_map_body_ = pose_is_map_body;
    pending_pose_stamp_ = pose_stamp;
    has_pending_initial_pose_ = true;
    manual_seed_active_ = manual_seed;
    ++initialization_generation_;
    const auto generation = initialization_generation_;
    has_initial_pose_ = false;
    applied_initialization_generation_ = 0;
    relocalization_state_ = map_ready_.load(std::memory_order_acquire)
      ? small_gicp_relocalization::msg::RelocalizationStatus::VERIFYING
      : small_gicp_relocalization::msg::RelocalizationStatus::MAP_LOADING;
    consecutive_successes_ = 0;
    stable_map_odom_.clear();
    has_stable_sequence_start_ = false;
    has_accepted_pose_ = false;
    initial_map_odom_.setIdentity();
    has_last_map_odom_ = has_trusted_map_odom_;
    if (has_trusted_map_odom_) {
      preview_map_odom_ = last_trusted_map_odom_;
      last_map_odom_ = last_trusted_map_odom_;
    } else {
      preview_map_odom_.setIdentity();
      last_map_odom_.setIdentity();
    }
    last_processed_stamp_ns_ = 0;
    has_success_wall_time_ = false;
    has_verification_start_wall_time_ = false;
    has_multistart_validation_ = false;
    last_multistart_consistent_ = false;
    has_tracking_attempt_wall_time_ = false;
    has_pending_tracking_candidate_ = false;
    pending_tracking_candidate_count_ = 0;
    consecutive_failures_ = 0;
    last_bbs_seed_time_ = std::chrono::steady_clock::now();
    has_bbs_seed_time_ = true;
    last_success_stamp_ = builtin_interfaces::msg::Time{};
    candidate_source_ = manual_seed ? "manual" : "bbs3d";
    status_detail_ = manual_seed ? "manual seed waiting for timestamped TF and input" :
      "BBS seed verification started";
    manual_seed_received_wall_time_ = std::chrono::steady_clock::now();
    has_latest_candidate_pose_ = false;
    last_match_status_ = small_gicp_relocalization::msg::RelocalizationStatus{};
    publishStatusLocked(
      pose_stamp.sec != 0 || pose_stamp.nanosec != 0 ? rclcpp::Time(pose_stamp) : now());
    if (has_trusted_map_odom_) {
      preview_map_odom_publisher_->publish(toTransform(
        last_trusted_map_odom_, map_frame_, odom_frame_, now()));
    }
    RCLCPP_INFO(
      get_logger(),
      "Received %s generation=%" PRIu64 " pose=(%.3f, %.3f, %.3f) yaw=%.3f; "
      "waiting for %s->%s TF%s",
      source, generation, pose.position.x, pose.position.y, pose.position.z, yaw(pose),
      odom_frame_.c_str(), pose_is_map_body ? body_frame_.c_str() : base_frame_.c_str(),
      pose_is_map_body ? " at the cloud timestamp" : "");
  }

  void initialPoseCallback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr message)
  {
    if (!message->header.frame_id.empty() && message->header.frame_id != map_frame_) {
      RCLCPP_WARN(
        get_logger(), "Ignoring initial pose in frame '%s' (expected '%s')",
        message->header.frame_id.c_str(), map_frame_.c_str());
      return;
    }

    auto pose = message->pose.pose;
    const auto & q = pose.orientation;
    const double q_norm = std::sqrt(
      q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
    if (!std::isfinite(pose.position.x) || !std::isfinite(pose.position.y) ||
      !std::isfinite(pose.position.z) || !std::isfinite(q_norm) || q_norm < 1e-8)
    {
      RCLCPP_WARN(get_logger(), "Ignoring non-finite or invalid /initialpose");
      return;
    }
    pose.orientation.x /= q_norm;
    pose.orientation.y /= q_norm;
    pose.orientation.z /= q_norm;
    pose.orientation.w /= q_norm;
    if (message->header.stamp.sec == 0 && message->header.stamp.nanosec == 0) {
      RCLCPP_WARN(
        get_logger(), "Manual /initialpose has a zero stamp; applying it with the latest odom->base TF");
    }
    if (message->header.frame_id.empty()) {
      RCLCPP_WARN(
        get_logger(), "Manual /initialpose has an empty frame_id; interpreting its pose in '%s'",
        map_frame_.c_str());
    }
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      resetInitializationLocked(
        pose, false, message->header.stamp, true, "manual initial pose for base_link");
      seed_initial_pose_covariance_ = message->pose.covariance;
    }
    tryApplyPendingInitialPose("initial pose callback");
  }

  bool cancelManualSeedAndResumeAuto(const char * reason, bool failed = false)
  {
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (!manual_seed_active_) {
        return false;
      }
    }
    saveOfflineSnapshotIfRequested(failed ? "manual_seed_failed" : "manual_seed_cancelled", now());
    std::uint64_t generation = 0;
    geometry_msgs::msg::PoseStamped::SharedPtr recent_bbs_candidate;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (!manual_seed_active_) {
        return false;
      }
      manual_seed_active_ = false;
      has_pending_initial_pose_ = false;
      pending_pose_is_map_body_ = false;
      has_initial_pose_ = false;
      applied_initialization_generation_ = 0;
      ++initialization_generation_;
      generation = initialization_generation_;
      has_latest_candidate_pose_ = false;
      last_match_status_ = small_gicp_relocalization::msg::RelocalizationStatus{};
      relocalization_state_ = map_ready_.load(std::memory_order_acquire)
        ? small_gicp_relocalization::msg::RelocalizationStatus::LOST
        : small_gicp_relocalization::msg::RelocalizationStatus::MAP_LOADING;
      consecutive_successes_ = 0;
      consecutive_failures_ = 0;
      stable_map_odom_.clear();
      has_stable_sequence_start_ = false;
      has_verification_start_wall_time_ = false;
      has_success_wall_time_ = false;
      clearPendingTrackingCandidateLocked();
      candidate_source_ = "automatic_recovery";
      status_detail_ = std::string("manual seed cancelled: ") + reason;
      automatic_recovery_start_wall_time_ = std::chrono::steady_clock::now();
      has_automatic_recovery_start_wall_time_ = true;
      last_bbs_seed_time_ = std::chrono::steady_clock::now();
      has_bbs_seed_time_ = true;
      if (has_trusted_map_odom_) {
        preview_map_odom_ = last_trusted_map_odom_;
        last_map_odom_ = last_trusted_map_odom_;
        has_last_map_odom_ = true;
      }
      if (has_latest_bbs_pose_) {
        const rclcpp::Time bbs_stamp(
          latest_bbs_pose_stamp_, get_clock()->get_clock_type());
        const double bbs_age = std::abs((now() - bbs_stamp).seconds());
        if (bbs_age <= manual_seed_bbs_height_max_age_sec_) {
          recent_bbs_candidate = std::make_shared<geometry_msgs::msg::PoseStamped>();
          recent_bbs_candidate->header.frame_id = map_frame_;
          recent_bbs_candidate->header.stamp = latest_bbs_pose_stamp_;
          recent_bbs_candidate->pose = latest_bbs_pose_;
        }
      }
      publishStatusLocked(now());
    }
    bool has_trusted = false;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      has_trusted = has_trusted_map_odom_;
    }
    if (failed) {
      RCLCPP_ERROR(
        get_logger(), "Manual seed failed (%s); generation=%" PRIu64
        " returned to automatic BBS search; last trusted map->odom retained=%s",
        reason, generation, has_trusted ? "yes" : "no");
    } else {
      RCLCPP_WARN(
        get_logger(), "Manual seed cancelled (%s); generation=%" PRIu64
        " returned to automatic BBS search; last trusted map->odom retained=%s",
        reason, generation, has_trusted ? "yes" : "no");
    }
    if (recent_bbs_candidate) {
      bbsGlobalPoseCallback(recent_bbs_candidate);
    } else {
      RCLCPP_INFO(
        get_logger(), "Automatic relocalization is active; waiting for a fresh 3D-BBS candidate");
    }
    return true;
  }

  void bbsGlobalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr message)
  {
    if (message->header.frame_id != map_frame_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Ignoring 3D-BBS pose in frame '%s' (expected '%s')",
        message->header.frame_id.c_str(), map_frame_.c_str());
      return;
    }
    if (message->header.stamp.sec == 0 && message->header.stamp.nanosec == 0) {
      RCLCPP_WARN(get_logger(), "Ignoring 3D-BBS pose with a zero cloud timestamp");
      return;
    }

    auto pose = message->pose;
    const auto & q = pose.orientation;
    const double q_norm = std::sqrt(
      q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
    if (!std::isfinite(pose.position.x) || !std::isfinite(pose.position.y) ||
      !std::isfinite(pose.position.z) || !std::isfinite(q_norm) || q_norm < 1e-8)
    {
      RCLCPP_WARN(get_logger(), "Ignoring invalid 3D-BBS pose");
      return;
    }
    pose.orientation.x /= q_norm;
    pose.orientation.y /= q_norm;
    pose.orientation.z /= q_norm;
    pose.orientation.w /= q_norm;

    bool manual_seed_active = false;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      latest_bbs_pose_ = pose;
      latest_bbs_pose_stamp_ = message->header.stamp;
      has_latest_bbs_pose_ = true;
      manual_seed_active = manual_seed_active_;
      if (manual_seed_active) {
        status_detail_ = "manual seed waiting for GICP; BBS supplies height/tilt only";
      }
    }
    if (manual_seed_active) {
      tryApplyPendingInitialPose("BBS height/tilt prior");
      return;
    }

    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (has_pending_initial_pose_ && !pending_pose_is_map_body_) {
        return;
      }
      const bool waiting_for_seed = !has_initial_pose_ ||
        relocalization_state_ ==
        small_gicp_relocalization::msg::RelocalizationStatus::WAITING_INITIAL_POSE ||
        relocalization_state_ == small_gicp_relocalization::msg::RelocalizationStatus::LOST;
      const bool retrying_seed = relocalization_state_ ==
        small_gicp_relocalization::msg::RelocalizationStatus::VERIFYING &&
        !has_accepted_pose_ && has_bbs_seed_time_ &&
        std::chrono::duration<double>(
          std::chrono::steady_clock::now() - last_bbs_seed_time_).count() >= bbs_seed_retry_sec_;
      if (!waiting_for_seed && !retrying_seed) {
        return;
      }
      resetInitializationLocked(
        pose, true, message->header.stamp, false, "3D-BBS global pose");
    }
    tryApplyPendingInitialPose("3D-BBS global pose callback");
  }

  static double yaw(const geometry_msgs::msg::Pose & pose)
  {
    return std::atan2(
      2.0 * (pose.orientation.w * pose.orientation.z +
      pose.orientation.x * pose.orientation.y),
      1.0 - 2.0 * (pose.orientation.y * pose.orientation.y +
      pose.orientation.z * pose.orientation.z));
  }

  void tryApplyPendingInitialPose(const char * source)
  {
    geometry_msgs::msg::Pose seed_pose;
    std::uint64_t generation = 0;
    bool pose_is_map_body = false;
    builtin_interfaces::msg::Time pose_stamp;
    bool manual_seed = false;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (!has_pending_initial_pose_) {
        return;
      }
      seed_pose = pending_initial_pose_;
      generation = initialization_generation_;
      pose_is_map_body = pending_pose_is_map_body_;
      pose_stamp = pending_pose_stamp_;
      manual_seed = manual_seed_active_;
    }

    const std::string odometry_child_frame = pose_is_map_body ? body_frame_ : base_frame_;
    Eigen::Isometry3d odom_reference = Eigen::Isometry3d::Identity();
    bool have_odom_reference = pose_is_map_body &&
      lookupCachedOdomBody(pose_stamp, odom_reference);
    try {
      if (!have_odom_reference) {
        const bool has_source_stamp = pose_stamp.sec != 0 || pose_stamp.nanosec != 0;
        const auto lookup_time = has_source_stamp ?
          rclcpp::Time(pose_stamp, get_clock()->get_clock_type()) :
          rclcpp::Time(0, 0, get_clock()->get_clock_type());
        const auto odom_reference_transform = tf_buffer_.lookupTransform(
          odom_frame_, odometry_child_frame, lookup_time,
          rclcpp::Duration::from_seconds(tf_lookup_timeout_sec_));
        odom_reference = toEigen(odom_reference_transform);
      }
    } catch (const tf2::TransformException & exception) {
      updateInitialPoseWaitStatus(
        generation, "TF unavailable at initial pose timestamp");
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Waiting to apply initial pose generation=%" PRIu64 " from %s: %s->%s TF "
        "unavailable: %s",
        generation, source, odom_frame_.c_str(), odometry_child_frame.c_str(), exception.what());
      return;
    }

    if (manual_seed) {
      const bool pose_has_explicit_height = std::abs(seed_pose.position.z) > 1e-3;
      Eigen::Isometry3d trusted_map_odom = Eigen::Isometry3d::Identity();
      bool has_trusted = false;
      geometry_msgs::msg::PoseStamped bbs_pose;
      bool has_bbs_pose = false;
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        has_trusted = has_trusted_map_odom_;
        trusted_map_odom = last_trusted_map_odom_;
        if (has_latest_bbs_pose_) {
          bbs_pose.pose = latest_bbs_pose_;
          bbs_pose.header.stamp = latest_bbs_pose_stamp_;
          has_bbs_pose = true;
        }
      }
      if (!pose_has_explicit_height) {
        if (has_trusted) {
          const Eigen::Isometry3d trusted_map_base = trusted_map_odom * odom_reference;
          seed_pose = small_gicp_relocalization::clickedPlanarPoseWithReferenceHeightAndTilt(
            seed_pose, trusted_map_base);
          RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "Manual /initialpose kept trusted map-frame base height and tilt at its TF timestamp");
        } else if (has_bbs_pose) {
          const auto now_time = now();
          const rclcpp::Time bbs_stamp(bbs_pose.header.stamp, get_clock()->get_clock_type());
          const rclcpp::Time manual_stamp =
            (pose_stamp.sec != 0 || pose_stamp.nanosec != 0) ?
            rclcpp::Time(pose_stamp, get_clock()->get_clock_type()) : now_time;
          const double age = std::abs((manual_stamp - bbs_stamp).seconds());
          if (age <= manual_seed_bbs_height_max_age_sec_) {
            try {
              Eigen::Isometry3d bbs_odom_body = Eigen::Isometry3d::Identity();
              if (!lookupCachedOdomBody(bbs_pose.header.stamp, bbs_odom_body)) {
                const auto bbs_odom_body_transform = tf_buffer_.lookupTransform(
                  odom_frame_, body_frame_, bbs_pose.header.stamp,
                  rclcpp::Duration::from_seconds(tf_lookup_timeout_sec_));
                bbs_odom_body = toEigen(bbs_odom_body_transform);
              }
              const Eigen::Isometry3d map_body = poseToIsometry(bbs_pose.pose);
              const Eigen::Isometry3d bbs_map_odom = map_body * bbs_odom_body.inverse();
              const Eigen::Isometry3d map_base_at_manual_stamp = bbs_map_odom * odom_reference;
              seed_pose = small_gicp_relocalization::clickedPlanarPoseWithReferenceHeightAndTilt(
                seed_pose, map_base_at_manual_stamp);
              RCLCPP_INFO(
                get_logger(), "Manual /initialpose uses BBS height/tilt projected to the click "
                "timestamp (candidate age=%.3fs); clicked base XY/yaw are retained", age);
            } catch (const tf2::TransformException & exception) {
              updateInitialPoseWaitStatus(
                generation, "TF unavailable for BBS height projection");
              RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 1000,
                "Waiting for timestamped odom->body TF to resolve manual seed height/tilt: %s",
                exception.what());
              return;
            }
          } else {
            updateInitialPoseWaitStatus(
              generation, "waiting for a timestamp-matched BBS height prior");
            RCLCPP_WARN_THROTTLE(
              get_logger(), *get_clock(), 1000,
              "Waiting for a timestamp-matched BBS height prior for manual /initialpose "
              "(candidate age=%.3fs limit=%.3fs)", age, manual_seed_bbs_height_max_age_sec_);
            return;
          }
        } else {
          updateInitialPoseWaitStatus(
            generation, "waiting for trusted or BBS map height; zero height is not assumed");
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "Manual /initialpose has no trusted height and no recent BBS height prior; "
            "waiting instead of assuming map Z=0");
          return;
        }
      }
    }

    Eigen::Isometry3d applied_map_odom;
    bool applied = false;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (!has_pending_initial_pose_ || !small_gicp_relocalization::resultBelongsToInitialization(
          generation, initialization_generation_)) {
        return;
      }
      if (pose_is_map_body) {
        applied_map_odom = small_gicp_relocalization::mapOdomFromMapBody(
          poseToIsometry(seed_pose), odom_reference);
      } else {
        applied_map_odom = small_gicp_relocalization::mapOdomFromClickedBasePose(
          seed_pose, odom_reference);
      }
      initial_map_odom_ = applied_map_odom;
      if (!has_trusted_map_odom_) {
        preview_map_odom_ = applied_map_odom;
      }
      last_map_odom_ = has_trusted_map_odom_ ? last_trusted_map_odom_ : applied_map_odom;
      has_initial_pose_ = true;
      applied_initialization_generation_ = generation;
      has_accepted_pose_ = false;
      has_last_map_odom_ = has_trusted_map_odom_;
      has_pending_initial_pose_ = false;
      pending_pose_is_map_body_ = false;
      seed_initial_pose_ = seed_pose;
      seed_initial_pose_is_map_body_ = pose_is_map_body;
      status_detail_ = manual_seed ? "manual seed under GICP validation" :
        "BBS seed under GICP validation";
      publishStatusLocked(now());
      if (has_trusted_map_odom_) {
        preview_map_odom_publisher_->publish(toTransform(
          last_trusted_map_odom_, map_frame_, odom_frame_, now()));
      } else {
        preview_map_odom_publisher_->publish(toTransform(
          applied_map_odom, map_frame_, odom_frame_, now()));
      }
      applied = true;
    }

    if (applied) {
      RCLCPP_INFO(
        get_logger(),
        "Applied %s initial pose generation=%" PRIu64 " state=VERIFYING "
        "seed=(%.3f, %.3f, %.3f) yaw=%.3f odom_reference=%s "
        "map_to_odom_translation=(%.3f, %.3f, %.3f) yaw=%.3f",
        manual_seed ? "manual base_link" : "BBS body",
        generation, seed_pose.position.x, seed_pose.position.y, seed_pose.position.z,
        yaw(seed_pose), odometry_child_frame.c_str(), applied_map_odom.translation().x(), applied_map_odom.translation().y(),
        applied_map_odom.translation().z(), yaw(applied_map_odom));
    }
  }

  void updateInitialPoseWaitStatus(std::uint64_t generation, const std::string & detail)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (generation != initialization_generation_ || !has_pending_initial_pose_ ||
      status_detail_ == detail)
    {
      return;
    }
    status_detail_ = detail;
    publishStatusLocked(now());
  }

  bool lookupCachedOdomBody(
    const builtin_interfaces::msg::Time & stamp, Eigen::Isometry3d & odom_body)
  {
    const int64_t stamp_ns = static_cast<int64_t>(stamp.sec) * 1000000000LL + stamp.nanosec;
    std::lock_guard<std::mutex> lock(state_mutex_);
    for (auto it = odom_body_history_.rbegin(); it != odom_body_history_.rend(); ++it) {
      if (it->first == stamp_ns) {
        odom_body = it->second;
        return true;
      }
    }
    return false;
  }

  void snapshotWorkerLoop()
  {
    while (true) {
      std::function<void()> snapshot_writer;
      {
        std::unique_lock<std::mutex> lock(snapshot_queue_mutex_);
        snapshot_queue_condition_.wait(lock, [this]() {
          return snapshot_worker_stopping_ || !snapshot_queue_.empty();
        });
        if (snapshot_queue_.empty()) {
          return;
        }
        snapshot_writer = std::move(snapshot_queue_.front());
        snapshot_queue_.pop_front();
      }
      try {
        snapshot_writer();
      } catch (const std::exception & exception) {
        RCLCPP_WARN(get_logger(), "Offline GICP snapshot worker failed: %s", exception.what());
      } catch (...) {
        RCLCPP_WARN(get_logger(), "Offline GICP snapshot worker failed with an unknown error");
      }
    }
  }

  void saveOfflineSnapshotIfRequested(const std::string & event, const rclcpp::Time & stamp)
  {
    if (offline_snapshot_dir_.empty()) {
      return;
    }
    std::unique_lock<std::mutex> queue_lock(snapshot_queue_mutex_);
    if (snapshot_worker_stopping_ ||
      !small_gicp_relocalization::snapshotQueueHasCapacity(snapshot_queue_.size()))
    {
      queue_lock.unlock();
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Offline GICP snapshot queue is full; dropping event '%s'", event.c_str());
      return;
    }
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr raw_source;
    Eigen::Isometry3d odom_body = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d seed_map_odom = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d trusted_map_odom = Eigen::Isometry3d::Identity();
    geometry_msgs::msg::Pose seed_pose;
    std::array<double, 36> seed_pose_covariance{};
    geometry_msgs::msg::Pose bbs_pose;
    builtin_interfaces::msg::Time bbs_stamp;
    tf2_msgs::msg::TFMessage static_tf;
    std::deque<tf2_msgs::msg::TFMessage> dynamic_tf;
    std::deque<nav_msgs::msg::Odometry> odometry;
    bool has_static_tf = false;
    bool has_trusted = false;
    bool has_cloud_odom = false;
    bool has_bbs_pose = false;
    bool has_candidate_pose = false;
    geometry_msgs::msg::PoseStamped candidate_pose;
    small_gicp_relocalization::msg::RelocalizationStatus latest_status;
    bool fastlio_healthy = false;
    bool has_fastlio_health = false;
    std::string detail;
    std::string candidate_source;
    std::uint64_t generation = 0;
    std::uint64_t cloud_generation = 0;
    std::uint64_t snapshot_sequence = 0;
    uint8_t state = small_gicp_relocalization::msg::RelocalizationStatus::MAP_LOADING;
    uint32_t consecutive_failures = 0;
    uint32_t pending_candidate_confirmations = 0;
    double trusted_pose_age_sec = -1.0;
    double point_cloud_age_sec = -1.0;
    double tf_wait_sec = 0.0;
    double matching_duration_ms = 0.0;
    bool manual_seed_active = false;
    int64_t cloud_stamp_ns = 0;
    const auto event_stamp_ns = stamp.nanoseconds();
    builtin_interfaces::msg::Time seed_pose_stamp;
    bool seed_pose_is_map_body = false;
    bool seed_transform_valid = false;
    std::string snapshot_event_key;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (!last_snapshot_cloud_) {
        return;
      }
      generation = initialization_generation_;
      cloud_generation = last_snapshot_generation_;
      cloud_stamp_ns = last_snapshot_stamp_ns_;
      const auto key = std::to_string(generation) + ":" +
        std::to_string(cloud_stamp_ns) + ":" + event;
      if (!snapshot_events_saved_.insert(key)) {
        return;
      }
      if (small_gicp_relocalization::isRateLimitedSnapshotEvent(event) &&
        !snapshot_event_rate_limiter_.admit(
          std::chrono::steady_clock::now(),
          small_gicp_relocalization::kSnapshotEventMinimumInterval))
      {
        snapshot_events_saved_.erase(key);
        return;
      }
      snapshot_event_key = key;
      snapshot_sequence = ++snapshot_sequence_;
      raw_source = last_snapshot_cloud_;
      odom_body = last_snapshot_odom_body_;
      has_cloud_odom = last_snapshot_has_odom_;
      seed_pose = seed_initial_pose_;
      seed_pose_covariance = seed_initial_pose_covariance_;
      seed_pose_stamp = pending_pose_stamp_;
      seed_pose_is_map_body = seed_initial_pose_is_map_body_;
      const bool current_seed_applied = has_initial_pose_ &&
        applied_initialization_generation_ == initialization_generation_;
      seed_map_odom = has_trusted_map_odom_ && has_accepted_pose_ ? last_map_odom_ :
        (current_seed_applied ? initial_map_odom_ : last_trusted_map_odom_);
      seed_transform_valid = has_cloud_odom &&
        (current_seed_applied || (has_trusted_map_odom_ && has_accepted_pose_));
      trusted_map_odom = last_trusted_map_odom_;
      has_trusted = has_trusted_map_odom_;
      if (has_latest_bbs_pose_) {
        bbs_pose = latest_bbs_pose_;
        bbs_stamp = latest_bbs_pose_stamp_;
        has_bbs_pose = true;
      }
      has_candidate_pose = has_latest_candidate_pose_;
      candidate_pose = latest_candidate_pose_;
      has_candidate_pose = has_candidate_pose &&
        latest_candidate_generation_ == initialization_generation_;
      latest_status = last_match_status_;
      static_tf = static_tf_snapshot_;
      has_static_tf = have_static_tf_snapshot_;
      dynamic_tf = dynamic_tf_history_;
      odometry = odometry_history_;
      has_fastlio_health = has_fastlio_health_;
      fastlio_healthy = fastlio_healthy_;
      detail = status_detail_;
      candidate_source = candidate_source_;
      state = relocalization_state_;
      consecutive_failures = consecutive_failures_;
      pending_candidate_confirmations = pending_tracking_candidate_count_;
      trusted_pose_age_sec = has_last_trusted_success_wall_time_
        ? std::chrono::duration<double>(
          std::chrono::steady_clock::now() - last_trusted_success_wall_time_).count()
        : -1.0;
      point_cloud_age_sec = std::max(
        0.0, (stamp.nanoseconds() - last_snapshot_stamp_ns_) / 1.0e9);
      tf_wait_sec = latest_tf_wait_sec_;
      matching_duration_ms = latest_match_duration_ms_;
      manual_seed_active = manual_seed_active_;
    }

    std::function<void()> snapshot_writer = [=,
      raw_source = std::move(raw_source),
      static_tf = std::move(static_tf),
      dynamic_tf = std::move(dynamic_tf),
      odometry = std::move(odometry)]() {
    std::filesystem::path event_dir;
    bool map_pcd_copied = false;
    bool navigation_map_copied = false;
    try {
      const auto root = std::filesystem::path(offline_snapshot_dir_);
      std::filesystem::create_directories(root);
      const auto map_version = small_gicp_relocalization::snapshotMapVersionKey(
        pcd_file_, navigation_map_file_);
      const auto map_cache_dir = root / "maps" / ("version_" + map_version);
      std::ostringstream sequence_name;
      sequence_name << "sample_" << producer_session_id_ << "_" << std::setw(6) <<
        std::setfill('0') << snapshot_sequence;
      event_dir = root / (sequence_name.str() +
        "_gen_" + std::to_string(generation) + "_" + event);
      if (!std::filesystem::create_directory(event_dir)) {
        throw std::runtime_error("snapshot directory already exists: " + event_dir.string());
      }
      std::filesystem::create_directories(map_cache_dir);
      const auto cached_localization_map = map_cache_dir / "map.pcd";
      small_gicp_relocalization::cacheSnapshotAsset(
        pcd_file_, cached_localization_map, std::to_string(producer_session_id_));
      small_gicp_relocalization::linkOrCopySnapshotAsset(
        cached_localization_map, event_dir / "map.pcd");
      map_pcd_copied = true;
      if (!navigation_map_file_.empty()) {
        const auto cached_navigation_map = map_cache_dir / "navigation_terrain_map.npz";
        small_gicp_relocalization::cacheSnapshotAsset(
          navigation_map_file_, cached_navigation_map, std::to_string(producer_session_id_));
        small_gicp_relocalization::linkOrCopySnapshotAsset(
          cached_navigation_map, event_dir / "navigation_terrain_map.npz");
        navigation_map_copied = true;
      }
      const auto cloud_path = event_dir /
        "cloud_registered_body_downsampled.pcd";
      const auto odom_body_path = event_dir /
        "odom_to_body.txt";
      const auto initial_pose_path = event_dir /
        "initial_pose.yaml";
      const auto seed_path = event_dir /
        "seed_map_body.txt";
      if (pcl::io::savePCDFileBinary(cloud_path.string(), *raw_source) != 0) {
        throw std::runtime_error("failed to save snapshot PCD");
      }
      {
        if (has_cloud_odom) {
          std::ofstream odom_body_file(odom_body_path);
          odom_body_file << odom_body.matrix() << '\n';
        }
        std::ofstream seed_odom_file(event_dir / "seed_map_odom.txt");
        if (seed_transform_valid) seed_odom_file << seed_map_odom.matrix() << '\n';
        if (has_trusted) {
          std::ofstream trusted_file(event_dir / "last_trusted_map_odom.txt");
          trusted_file << trusted_map_odom.matrix() << '\n';
        }
      }
      std::ofstream initial_pose_file(initial_pose_path);
      initial_pose_file << "position: [" << seed_pose.position.x << ", "
                        << seed_pose.position.y << ", " << seed_pose.position.z << "]\n"
                        << "orientation: [" << seed_pose.orientation.x << ", "
                        << seed_pose.orientation.y << ", " << seed_pose.orientation.z << ", "
                        << seed_pose.orientation.w << "]\n"
                        << "stamp_ns: " << seed_pose_stamp.sec * 1000000000LL +
                        seed_pose_stamp.nanosec << "\n"
                        << "frame_id: " << map_frame_ << "\n"
                        << "pose_child_frame: " <<
                        (seed_pose_is_map_body ? body_frame_ : base_frame_) << "\n";
      initial_pose_file << "covariance: [";
      for (std::size_t index = 0; index < seed_pose_covariance.size(); ++index) {
        if (index > 0) initial_pose_file << ", ";
        initial_pose_file << seed_pose_covariance[index];
      }
      initial_pose_file << "]\n";
      if (seed_transform_valid) {
        const Eigen::Isometry3d seed_map_body = seed_map_odom * odom_body;
        std::ofstream seed_file(seed_path);
        seed_file << seed_map_body.matrix() << '\n';
      }
      const auto write_tf = [](std::ostream & output, const tf2_msgs::msg::TFMessage & message) {
          for (const auto & transform : message.transforms) {
            output << transform.header.stamp.sec << '.' << transform.header.stamp.nanosec << ' '
                   << transform.header.frame_id << ' ' << transform.child_frame_id << ' '
                   << transform.transform.translation.x << ' '
                   << transform.transform.translation.y << ' '
                   << transform.transform.translation.z << ' '
                   << transform.transform.rotation.x << ' '
                   << transform.transform.rotation.y << ' '
                   << transform.transform.rotation.z << ' '
                   << transform.transform.rotation.w << '\n';
          }
        };
      {
        std::ofstream tf_static_file(event_dir / "tf_static.txt");
        if (has_static_tf) write_tf(tf_static_file, static_tf);
        std::ofstream tf_file(event_dir / "tf_history.txt");
        for (const auto & message : dynamic_tf) write_tf(tf_file, message);
      }
      {
        std::ofstream odometry_file(event_dir / "Odometry_history.txt");
        for (const auto & message : odometry) {
          odometry_file << message.header.stamp.sec << '.' << message.header.stamp.nanosec << ' '
                        << message.header.frame_id << ' ' << message.child_frame_id << ' '
                        << message.pose.pose.position.x << ' ' << message.pose.pose.position.y << ' '
                        << message.pose.pose.position.z << ' '
                        << message.pose.pose.orientation.x << ' '
                        << message.pose.pose.orientation.y << ' '
                        << message.pose.pose.orientation.z << ' '
                        << message.pose.pose.orientation.w << ' '
                        << message.twist.twist.linear.x << ' '
                        << message.twist.twist.linear.y << ' '
                        << message.twist.twist.linear.z << ' '
                        << message.twist.twist.angular.x << ' '
                        << message.twist.twist.angular.y << ' '
                        << message.twist.twist.angular.z << '\n';
        }
      }
      {
        std::ofstream bbs_file(event_dir / "bbs_candidate.txt");
        if (has_bbs_pose) {
          bbs_file << bbs_stamp.sec << '.' << bbs_stamp.nanosec << ' '
                   << bbs_pose.position.x << ' ' << bbs_pose.position.y << ' '
                   << bbs_pose.position.z << ' ' << bbs_pose.orientation.x << ' '
                   << bbs_pose.orientation.y << ' ' << bbs_pose.orientation.z << ' '
                   << bbs_pose.orientation.w << '\n';
        }
        std::ofstream health_file(event_dir / "fastlio_health.txt");
        health_file << "received=" << has_fastlio_health << " healthy=" << fastlio_healthy << '\n';
      }
      {
        std::ofstream candidate_file(event_dir / "gicp_candidate.txt");
        if (has_candidate_pose) {
          candidate_file << candidate_pose.header.stamp.sec << '.'
                         << candidate_pose.header.stamp.nanosec << ' '
                         << candidate_pose.header.frame_id << ' '
                         << candidate_pose.pose.position.x << ' '
                         << candidate_pose.pose.position.y << ' '
                         << candidate_pose.pose.position.z << ' '
                         << candidate_pose.pose.orientation.x << ' '
                         << candidate_pose.pose.orientation.y << ' '
                         << candidate_pose.pose.orientation.z << ' '
                         << candidate_pose.pose.orientation.w << '\n';
        }
      }
      {
        std::ofstream manifest(event_dir / "manifest.yaml");
        manifest << "event: " << event << '\n'
                 << "generation: " << generation << '\n'
                 << "cloud_generation: " << cloud_generation << '\n'
                 << "producer_session_id: " << producer_session_id_ << '\n'
                 << "event_stamp_ns: " << event_stamp_ns << '\n'
                 << "cloud_stamp_ns: " << cloud_stamp_ns << '\n'
                 << "cloud_points: " << raw_source->size() << '\n'
                 << "cloud_odom_available: " << has_cloud_odom << '\n'
                 << "seed_transform_valid: " << seed_transform_valid << '\n'
                 << "cloud_frame: " << body_frame_ << '\n'
                 << "state: " << static_cast<unsigned int>(state) << '\n'
                 << "map_frame: " << map_frame_ << '\n'
                 << "odom_frame: " << odom_frame_ << '\n'
                 << "base_frame: " << base_frame_ << '\n'
                 << "candidate_source: " << candidate_source << '\n'
                 << "state_detail: \"" << detail << "\"\n"
                 << "manual_seed_active: " << manual_seed_active << '\n'
                 << "consecutive_failures: " << consecutive_failures << '\n'
                 << "pending_candidate_confirmations: " <<
                 pending_candidate_confirmations << '\n'
                 << "last_trusted_pose_age_sec: " << trusted_pose_age_sec << '\n'
                 << "point_cloud_age_sec: " << point_cloud_age_sec << '\n'
                 << "tf_wait_sec: " << tf_wait_sec << '\n'
                 << "matching_duration_ms: " << matching_duration_ms << '\n'
                 << "registration_voxel_size: " << registration_voxel_size_ << '\n'
                 << "tracking_input_voxel_size: " << tracking_input_voxel_size_ << '\n'
                 << "tracking_frequency_hz: " << tracking_frequency_hz_ << '\n'
                 << "max_translation_jump: " << max_translation_jump_ << '\n'
                 << "max_rotation_jump: " << max_rotation_jump_ << '\n'
                 << "num_threads: " << num_threads_ << '\n'
                 << "pcd_file: \"" << pcd_file_ << "\"\n"
                 << "navigation_map_file: \"" << navigation_map_file_ << "\"\n"
                 << "map_version: " << map_version << '\n'
                 << "startup_context: \"" << startup_context_ << "\"\n"
                 << "static_tf_captured: " << has_static_tf << '\n'
                 << "localization_map_copied: " << map_pcd_copied << '\n'
                 << "navigation_map_copied: " << navigation_map_copied << '\n'
                 << "dynamic_tf_messages: " << dynamic_tf.size() << '\n'
                 << "odometry_messages: " << odometry.size() << '\n'
                 << "fastlio_health_received: " << has_fastlio_health << '\n'
                 << "fastlio_healthy: " << fastlio_healthy << '\n'
                 << "gicp_candidate_available: " << has_candidate_pose << '\n'
                 << "gicp_candidate_stamp_ns: " <<
                 static_cast<std::int64_t>(candidate_pose.header.stamp.sec) * 1000000000LL +
                 candidate_pose.header.stamp.nanosec << '\n'
                 << "gicp_num_inliers: " << latest_status.num_inliers << '\n'
                 << "gicp_inlier_ratio: " << latest_status.inlier_ratio << '\n'
                 << "gicp_match_sectors: " << latest_status.match_sectors << '\n'
                 << "gicp_match_coverage_ratio: " << latest_status.match_coverage_ratio << '\n'
                 << "gicp_multistart_consistent: " << latest_status.multistart_consistent << '\n'
                 << "gicp_geometry_rmse_m: " << latest_status.geometry_rmse_m << '\n'
                 << "gicp_geometry_p95_m: " << latest_status.geometry_p95_m << '\n'
                 << "gicp_correction_translation_m: " <<
                 latest_status.correction_translation << '\n'
                 << "gicp_correction_rotation_rad: " <<
                 latest_status.correction_rotation << '\n'
                 << "max_correspondence_distance_m: " << max_correspondence_distance_ << '\n'
                 << "max_iterations: " << max_iterations_ << '\n'
                 << "min_inliers: " << min_inliers_ << '\n'
                 << "min_inlier_ratio: " << min_inlier_ratio_ << '\n'
                 << "max_geometry_rmse_m: " << max_geometry_rmse_m_ << '\n'
                 << "max_geometry_p95_m: " << max_geometry_p95_m_ << '\n'
                 << "min_match_sectors: " << min_match_sectors_ << '\n'
                 << "require_multistart_consistency: " << require_multistart_consistency_ << '\n'
                 << "max_multistart_translation_spread_m: " <<
                 max_multistart_translation_spread_ << '\n'
                 << "max_multistart_yaw_spread_rad: " << max_multistart_yaw_spread_ << '\n'
                 << "stable_frames_required: " << stable_frames_required_ << '\n'
                 << "stable_duration_sec: " << min_stable_duration_sec_ << '\n'
                 << "tracking_frequency_hz: " << tracking_frequency_hz_ << '\n'
                 << "tracking_max_cloud_age_sec: " << tracking_max_cloud_age_sec_ << '\n'
                 << "manual_seed_input_timeout_sec: " << manual_seed_input_timeout_sec_ << '\n'
                 << "manual_seed_validation_timeout_sec: " <<
                 manual_seed_validation_timeout_sec_ << '\n';
      }
      small_gicp_relocalization::pruneSnapshotDirectories(
        root, static_cast<std::size_t>(offline_snapshot_max_count_));
      small_gicp_relocalization::pruneSnapshotMapVersions(root, map_version);
      RCLCPP_INFO(
        get_logger(), "Saved offline GICP %s snapshot to %s (cloud=%zu points, generation=%" PRIu64 ")",
        event.c_str(), event_dir.c_str(), raw_source->size(), generation);
    } catch (const std::exception & exception) {
      if (!event_dir.empty()) {
        std::error_code cleanup_error;
        std::filesystem::remove_all(event_dir, cleanup_error);
      }
      if (!snapshot_event_key.empty()) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        snapshot_events_saved_.erase(snapshot_event_key);
      }
      RCLCPP_WARN(get_logger(), "Failed to save offline GICP snapshot: %s", exception.what());
    }
    };
    snapshot_queue_.push_back(std::move(snapshot_writer));
    queue_lock.unlock();
    snapshot_queue_condition_.notify_one();
  }

  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr message) {
    const auto processing_start = std::chrono::steady_clock::now();
    const rclcpp::Time stamp(message->header.stamp);
    bool tracking_mode = false;
    uint8_t result_state = small_gicp_relocalization::msg::RelocalizationStatus::MAP_LOADING;
    Eigen::Isometry3d odom_body_eigen = Eigen::Isometry3d::Identity();
    bool have_odom_body_transform = false;
    const auto first_tf_lookup_start = std::chrono::steady_clock::now();
    if (message->header.frame_id == body_frame_ && stamp.nanoseconds() > 0) {
      try {
        const auto stamped_odom_body = tf_buffer_.lookupTransform(
          odom_frame_, body_frame_, message->header.stamp,
          rclcpp::Duration::from_seconds(tf_lookup_timeout_sec_));
        odom_body_eigen = toEigen(stamped_odom_body);
        have_odom_body_transform = true;
        std::lock_guard<std::mutex> lock(state_mutex_);
        odom_body_history_.emplace_back(stamp.nanoseconds(), odom_body_eigen);
        latest_tf_wait_sec_ = std::chrono::duration<double>(
          std::chrono::steady_clock::now() - first_tf_lookup_start).count();
        constexpr std::size_t kMaxOdomBodyHistory = 2048;
        while (odom_body_history_.size() > kMaxOdomBodyHistory) {
          odom_body_history_.pop_front();
        }
      } catch (const tf2::TransformException &) {
        // The registered cloud can arrive before its matching TF; the normal path below
        // retries with the configured timeout after initialization has been applied.
        std::lock_guard<std::mutex> lock(state_mutex_);
        latest_tf_wait_sec_ = std::chrono::duration<double>(
          std::chrono::steady_clock::now() - first_tf_lookup_start).count();
      }
    }
    if (!map_ready_.load(std::memory_order_acquire)) {
      std::lock_guard<std::mutex> lock(state_mutex_);
      const bool candidate_cleared = clearPendingTrackingCandidateLocked();
      if (relocalization_state_ !=
        small_gicp_relocalization::msg::RelocalizationStatus::MAP_LOADING || candidate_cleared) {
        relocalization_state_ = small_gicp_relocalization::msg::RelocalizationStatus::MAP_LOADING;
        publishStatusLocked(now());
      }
      return;
    }
    tryApplyPendingInitialPose("point cloud callback");
    if (!message->header.frame_id.empty() && message->header.frame_id != body_frame_) {
      interruptPendingTrackingCandidateForSkippedFrame("input frame mismatch");
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Ignoring cloud in frame '%s' (expected '%s')",
        message->header.frame_id.c_str(), body_frame_.c_str());
      return;
    }
    std::uint64_t result_generation = 0;

    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (stamp.nanoseconds() <= 0 || stamp.nanoseconds() <= last_processed_stamp_ns_) {
        if (relocalization_state_ ==
          small_gicp_relocalization::msg::RelocalizationStatus::LOCALIZED &&
          has_accepted_pose_ && clearPendingTrackingCandidateLocked())
        {
          publishStatusLocked(now());
        }
        return;
      }
      result_generation = initialization_generation_;
      if (!generationIsAppliedLocked(result_generation)) {
        return;
      }
      tracking_mode = relocalization_state_ ==
        small_gicp_relocalization::msg::RelocalizationStatus::LOCALIZED && has_accepted_pose_;
      result_state = relocalization_state_;
      if (tracking_mode) {
        const auto attempt_time = std::chrono::steady_clock::now();
        const double interval_sec = 1.0 / tracking_frequency_hz_;
        if (has_tracking_attempt_wall_time_ &&
          std::chrono::duration<double>(attempt_time - last_tracking_attempt_wall_time_).count() <
          interval_sec)
        {
          last_processed_stamp_ns_ = stamp.nanoseconds();
          return;
        }
        last_tracking_attempt_wall_time_ = attempt_time;
        has_tracking_attempt_wall_time_ = true;
      }
      if (!fastlioHealthyLocked()) {
        const bool cleared_candidate =
          tracking_mode && clearPendingTrackingCandidateLocked();
        status_detail_ = "input expired: FAST-LIO health is false or stale";
        if (tracking_mode || cleared_candidate) publishStatusLocked(now());
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Skipping GICP cloud stamp=%" PRId64 ": FAST-LIO health is false or stale",
          stamp.nanoseconds());
        return;
      }
      last_processed_stamp_ns_ = stamp.nanoseconds();
    }

    if (tracking_mode) {
      const double cloud_age_sec = (now() - stamp).seconds();
      if (!std::isfinite(cloud_age_sec) || cloud_age_sec < -0.05 ||
        cloud_age_sec > tracking_max_cloud_age_sec_)
      {
        interruptPendingTrackingCandidateForSkippedFrame("stale input cloud");
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), static_cast<uint64_t>(timing_log_period_ms_),
          "Dropping stale tracking cloud age=%.3fs max=%.3fs; no GICP backlog is processed",
          cloud_age_sec, tracking_max_cloud_age_sec_);
        return;
      }
    }

    pcl::PointCloud<pcl::PointXYZ> raw_source;
    try {
      pcl::fromROSMsg(*message, raw_source);
    } catch (const std::exception &exception) {
      RCLCPP_WARN(get_logger(), "Failed to convert input cloud: %s", exception.what());
      markRegistrationFailure(
        stamp, 0, 0.0, 0.0, 0.0, 0.0, result_generation, 0.0, false,
        "input point cloud conversion failed");
      return;
    }
    if (raw_source.empty()) {
      RCLCPP_WARN(
        get_logger(), "Rejecting GICP generation=%" PRIu64 ": source cloud is empty",
        result_generation);
      markRegistrationFailure(
        stamp, 0, 0.0, 0.0, 0.0, 0.0, result_generation, 0.0, false,
        "input point cloud is empty");
      return;
    }
    {
      auto scan_snapshot = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(raw_source);
      std::lock_guard<std::mutex> lock(state_mutex_);
      last_snapshot_cloud_ = std::move(scan_snapshot);
      last_snapshot_stamp_ns_ = stamp.nanoseconds();
      last_snapshot_generation_ = result_generation;
      last_snapshot_has_odom_ = false;
    }

    if (!have_odom_body_transform) {
      const auto tf_lookup_start = std::chrono::steady_clock::now();
      try {
        const auto odom_body = tf_buffer_.lookupTransform(
          odom_frame_, body_frame_, message->header.stamp,
          rclcpp::Duration::from_seconds(tf_lookup_timeout_sec_));
        odom_body_eigen = toEigen(odom_body);
      } catch (const tf2::TransformException &exception) {
        {
          std::lock_guard<std::mutex> lock(state_mutex_);
          latest_tf_wait_sec_ = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - tf_lookup_start).count();
        }
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000, "Cannot lookup %s->%s: %s",
          odom_frame_.c_str(), body_frame_.c_str(), exception.what());
        RCLCPP_WARN(
          get_logger(), "Rejecting GICP generation=%" PRIu64 ": odom->body TF unavailable",
          result_generation);
        {
          std::lock_guard<std::mutex> lock(state_mutex_);
          status_detail_ = "input TF unavailable at cloud timestamp";
        }
        markRegistrationFailure(
          stamp, 0, 0.0, 0.0, 0.0, 0.0, result_generation, 0.0, false,
          "input TF unavailable at cloud timestamp");
        return;
      }
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        latest_tf_wait_sec_ = std::chrono::duration<double>(
          std::chrono::steady_clock::now() - tf_lookup_start).count();
      }
    }
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (last_snapshot_stamp_ns_ == stamp.nanoseconds() &&
        last_snapshot_generation_ == result_generation)
      {
        last_snapshot_odom_body_ = odom_body_eigen;
        last_snapshot_has_odom_ = true;
      }
    }

    Eigen::Isometry3d predicted_map_body = odom_body_eigen;
    Eigen::Isometry3d predicted_map_odom = Eigen::Isometry3d::Identity();
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (!generationIsAppliedLocked(result_generation)) {
        return;
      }
      const Eigen::Isometry3d &map_odom_seed =
        relocalization_state_ ==
        small_gicp_relocalization::msg::RelocalizationStatus::LOCALIZED && has_accepted_pose_
        ? last_map_odom_ : (has_trusted_map_odom_ ? initial_map_odom_ : preview_map_odom_);
      predicted_map_odom = map_odom_seed;
      predicted_map_body = small_gicp_relocalization::predictMapBody(
        map_odom_seed, odom_body_eigen);
    }

    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (!has_verification_start_wall_time_ && generationIsAppliedLocked(result_generation)) {
        has_verification_start_wall_time_ = true;
        verification_start_wall_time_ = std::chrono::steady_clock::now();
        relocalization_state_ = small_gicp_relocalization::msg::RelocalizationStatus::VERIFYING;
        publishStatusLocked(stamp);
        RCLCPP_INFO(
          get_logger(), "First registrable scan received; verification timeout starts now");
      }
    }

    const auto source_downsample_start = std::chrono::steady_clock::now();

    const double input_voxel_size = tracking_mode
      ? tracking_input_voxel_size_ : registration_voxel_size_;
    auto source_cloud =
        small_gicp::voxelgrid_sampling_omp<pcl::PointCloud<pcl::PointXYZ>, CovarianceCloud>(
      raw_source, input_voxel_size, num_threads_);
    if (source_cloud->size() < static_cast<size_t>(std::max(num_neighbors_, 5))) {
      RCLCPP_WARN(
        get_logger(),
        "Rejecting GICP generation=%" PRIu64 ": source points after voxelization=%zu, "
        "minimum=%d",
        result_generation, source_cloud->size(), std::max(num_neighbors_, 5));
      markRegistrationFailure(
        stamp, 0, 0.0, 0.0, 0.0, 0.0, result_generation, 0.0, false,
        "too few source points after voxelization");
      return;
    }
    const double source_downsample_ms = elapsedMilliseconds(source_downsample_start);
    const auto source_covariance_start = std::chrono::steady_clock::now();
    small_gicp::estimate_covariances_omp(*source_cloud, num_neighbors_, num_threads_);
    const double source_covariance_ms = elapsedMilliseconds(source_covariance_start);

    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (!generationIsAppliedLocked(result_generation)) {
        RCLCPP_WARN(
          get_logger(),
          "Discarding GICP generation=%" PRIu64 " before registration; current_generation=%" PRIu64
          " applied_generation=%" PRIu64 " processing_elapsed=%.1fms",
          result_generation, initialization_generation_, applied_initialization_generation_,
          elapsedMilliseconds(processing_start));
        return;
      }
    }

    const auto registration_start = std::chrono::steady_clock::now();
    const int effective_max_iterations = tracking_mode ? tracking_max_iterations_ : max_iterations_;
    const int alternate_max_iterations = std::max(1, effective_max_iterations / 2);
    const auto run_registration = [this, &source_cloud](
      const Eigen::Isometry3d &seed, int iteration_limit) {
        small_gicp::Registration<small_gicp::GICPFactor, small_gicp::ParallelReductionOMP> registration;
        registration.reduction.num_threads = num_threads_;
        registration.rejector.max_dist_sq =
          max_correspondence_distance_ * max_correspondence_distance_;
        registration.optimizer.max_iterations = iteration_limit;
        registration.criteria.rotation_eps = 0.1 * 3.14159265358979323846 / 180.0;
        registration.criteria.translation_eps = 1e-3;
      return registration.align(*map_cloud_, *source_cloud, *map_tree_, seed);
      };
    const auto main_registration_start = std::chrono::steady_clock::now();
    const auto result = run_registration(predicted_map_body, effective_max_iterations);
    const double main_registration_ms = elapsedMilliseconds(main_registration_start);

    const Eigen::Isometry3d map_body = result.T_target_source;
    const std::size_t main_iterations_completed = result.iterations + 1U;
    const bool main_iteration_cap_accepted = !result.converged &&
      main_iterations_completed >= static_cast<std::size_t>(effective_max_iterations);
    const bool main_termination_gate =
      small_gicp_relocalization::registrationTerminationAccepted(
      result.converged, main_iterations_completed,
      static_cast<std::size_t>(effective_max_iterations));
    const bool finite_map_body = isFiniteIsometry(map_body);
    const double correction_translation = finite_map_body
      ? small_gicp_relocalization::translationCorrection(map_body, predicted_map_body)
      : std::numeric_limits<double>::infinity();
    const double correction_rotation = finite_map_body
      ? small_gicp_relocalization::rotationCorrection(map_body, predicted_map_body)
      : std::numeric_limits<double>::infinity();
    const double inlier_ratio = source_cloud->empty()
      ? 0.0 : static_cast<double>(result.num_inliers) / static_cast<double>(source_cloud->size());
    const auto coverage = evaluateMatchCoverage(map_body, *source_cloud);
    const auto seed_coverage = evaluateMatchCoverage(predicted_map_body, *source_cloud);
    const bool geometry_gate = std::isfinite(coverage.rmse_m) &&
      std::isfinite(coverage.p95_m) && coverage.rmse_m <= max_geometry_rmse_m_ &&
      coverage.p95_m <= max_geometry_p95_m_;
    const bool tracking_translation_gate = !tracking_mode ||
      correction_translation <= tracking_max_translation_correction_;
    const bool tracking_rotation_gate = !tracking_mode ||
      correction_rotation <= tracking_max_rotation_correction_;
    const bool main_quality_without_multistart = finite_map_body && main_termination_gate &&
      result.num_inliers >= static_cast<std::size_t>(min_inliers_) &&
      std::isfinite(inlier_ratio) && inlier_ratio >= min_inlier_ratio_ && inlier_ratio <= 1.0 &&
      std::isfinite(correction_translation) && correction_translation <= max_translation_jump_ &&
      std::isfinite(correction_rotation) && correction_rotation <= max_rotation_jump_ &&
      tracking_translation_gate && tracking_rotation_gate &&
      coverage.matched_points >= static_cast<std::size_t>(min_inliers_) &&
      coverage.ratio >= min_inlier_ratio_ &&
      coverage.sectors >= static_cast<std::size_t>(min_match_sectors_) && geometry_gate;
    bool run_alternate = true;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      run_alternate = require_multistart_consistency_ &&
        (!has_multistart_validation_ || !main_quality_without_multistart ||
        (tracking_mode &&
        (correction_translation > tracking_small_correction_translation_m_ ||
        correction_rotation > tracking_small_correction_rotation_rad_)));
    }
    Eigen::Isometry3d alternate_seed = predicted_map_body;
    alternate_seed.translation().x() += multistart_seed_translation_;
    alternate_seed.linear() = Eigen::AngleAxisd(
      multistart_seed_yaw_, Eigen::Vector3d::UnitZ()).toRotationMatrix() * alternate_seed.linear();

    const auto alternate_registration_start = std::chrono::steady_clock::now();
    small_gicp::RegistrationResult alternate_result;
    MatchCoverage alternate_coverage;
    bool alternate_ran = false;
    if (run_alternate) {
      alternate_ran = true;
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (!generationIsAppliedLocked(result_generation)) {
          RCLCPP_WARN(
            get_logger(),
            "Discarding GICP generation=%" PRIu64 " before alternate registration; "
            "current_generation=%" PRIu64 " applied_generation=%" PRIu64
            " processing_elapsed=%.1fms",
            result_generation, initialization_generation_, applied_initialization_generation_,
            elapsedMilliseconds(processing_start));
          return;
        }
      }
      alternate_result = run_registration(alternate_seed, alternate_max_iterations);
      alternate_coverage = evaluateMatchCoverage(alternate_result.T_target_source, *source_cloud);
    }
    const double alternate_elapsed_ms = alternate_ran
      ? elapsedMilliseconds(alternate_registration_start) : 0.0;
    const std::size_t alternate_iterations_completed = alternate_ran
      ? alternate_result.iterations + 1U : 0U;
    const bool alternate_iteration_cap_accepted = alternate_ran && !alternate_result.converged &&
      alternate_iterations_completed >= static_cast<std::size_t>(alternate_max_iterations);
    const bool alternate_termination_gate = alternate_ran &&
      small_gicp_relocalization::registrationTerminationAccepted(
      alternate_result.converged, alternate_iterations_completed,
      static_cast<std::size_t>(alternate_max_iterations));
    const bool alternate_finite = isFiniteIsometry(alternate_result.T_target_source) &&
      alternate_ran;
    const double alternate_inlier_ratio = source_cloud->empty()
      ? 0.0 : static_cast<double>(alternate_result.num_inliers) / static_cast<double>(source_cloud->size());
    const double alternate_correction_translation = alternate_finite
      ? small_gicp_relocalization::translationCorrection(
      alternate_result.T_target_source, predicted_map_body)
      : std::numeric_limits<double>::infinity();
    const double alternate_correction_rotation = alternate_finite
      ? small_gicp_relocalization::rotationCorrection(
      alternate_result.T_target_source, predicted_map_body)
      : std::numeric_limits<double>::infinity();
    const bool alternate_quality_gate = alternate_finite &&
      small_gicp_relocalization::registrationPassesGate(
        alternate_termination_gate, alternate_result.num_inliers, static_cast<size_t>(min_inliers_),
        alternate_correction_translation, alternate_correction_rotation,
        max_translation_jump_, max_rotation_jump_) &&
      (!tracking_mode ||
      (alternate_correction_translation <= tracking_max_translation_correction_ &&
      alternate_correction_rotation <= tracking_max_rotation_correction_)) &&
      std::isfinite(alternate_inlier_ratio) && alternate_inlier_ratio >= 0.0 &&
      alternate_inlier_ratio <= 1.0 && alternate_inlier_ratio >= min_inlier_ratio_ &&
      std::isfinite(alternate_coverage.rmse_m) &&
      std::isfinite(alternate_coverage.p95_m) &&
      alternate_coverage.matched_points >= static_cast<std::size_t>(min_inliers_) &&
      alternate_coverage.ratio >= min_inlier_ratio_ &&
      alternate_coverage.sectors >= static_cast<std::size_t>(min_match_sectors_) &&
      alternate_coverage.rmse_m <= max_geometry_rmse_m_ &&
      alternate_coverage.p95_m <= max_geometry_p95_m_;
    const double multistart_translation_spread = finite_map_body && alternate_finite
      ? small_gicp_relocalization::translationCorrection(
      alternate_result.T_target_source, map_body)
      : std::numeric_limits<double>::infinity();
    const double multistart_yaw_spread = finite_map_body && alternate_finite
      ? small_gicp_relocalization::rotationCorrection(
      alternate_result.T_target_source, map_body)
      : std::numeric_limits<double>::infinity();
    const bool multistart_translation_spread_gate = std::isfinite(multistart_translation_spread) &&
      multistart_translation_spread <= max_multistart_translation_spread_;
    const bool multistart_yaw_spread_gate = std::isfinite(multistart_yaw_spread) &&
      multistart_yaw_spread <= max_multistart_yaw_spread_;
    const bool multistart_alternate_quality_gate = alternate_ran ? alternate_quality_gate :
      has_multistart_validation_;
    const bool multistart_translation_gate = multistart_translation_spread_gate;
    const bool multistart_yaw_gate = multistart_yaw_spread_gate;
    const bool multistart_consistent = alternate_ran
      ? alternate_quality_gate && multistart_translation_spread_gate && multistart_yaw_spread_gate
      : has_multistart_validation_ && last_multistart_consistent_;
    const double registration_elapsed_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - registration_start).count();
    const double processing_elapsed_ms = elapsedMilliseconds(processing_start);
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      latest_match_duration_ms_ = processing_elapsed_ms;
    }
    const bool finite_gate = finite_map_body && std::isfinite(inlier_ratio) &&
      std::isfinite(coverage.rmse_m) && std::isfinite(coverage.p95_m) &&
      std::isfinite(correction_translation) && std::isfinite(correction_rotation);
    const bool convergence_gate = main_termination_gate;
    const bool inlier_count_gate = result.num_inliers >= static_cast<std::size_t>(min_inliers_);
    const bool inlier_ratio_gate = std::isfinite(inlier_ratio) &&
      inlier_ratio >= min_inlier_ratio_ && inlier_ratio <= 1.0;
    const bool geometry_rmse_gate = std::isfinite(coverage.rmse_m) &&
      coverage.rmse_m <= max_geometry_rmse_m_;
    const bool geometry_p95_gate = std::isfinite(coverage.p95_m) &&
      coverage.p95_m <= max_geometry_p95_m_;
    const bool translation_gate = std::isfinite(correction_translation) &&
      correction_translation <= max_translation_jump_;
    const bool rotation_gate = std::isfinite(correction_rotation) &&
      correction_rotation <= max_rotation_jump_;
    const bool coverage_points_gate = coverage.matched_points >= static_cast<std::size_t>(min_inliers_);
    const bool coverage_ratio_gate = std::isfinite(coverage.ratio) &&
      coverage.ratio >= min_inlier_ratio_;
    const bool coverage_sectors_gate = coverage.sectors >= static_cast<std::size_t>(min_match_sectors_);
    const bool multistart_gate = !require_multistart_consistency_ || multistart_consistent;
    const Eigen::Isometry3d map_odom = small_gicp_relocalization::mapOdomFromMapBody(
      map_body, odom_body_eigen);
    const bool passes_quality = finite_gate && convergence_gate && inlier_count_gate &&
      inlier_ratio_gate && geometry_rmse_gate && geometry_p95_gate && translation_gate && rotation_gate &&
      tracking_translation_gate && tracking_rotation_gate &&
      coverage_points_gate && coverage_ratio_gate && coverage_sectors_gate && multistart_gate;
    double result_cloud_age_sec = 0.0;
    double result_processing_elapsed_ms = 0.0;
    std::string result_rejection_reason;
    if (!registrationResultCurrentAndFresh(
        stamp, processing_start, tracking_mode, result_state, result_generation,
        result_cloud_age_sec, result_processing_elapsed_ms, result_rejection_reason))
    {
      if (tracking_mode &&
        (result_rejection_reason == "cloud_stale" ||
        result_rejection_reason == "processing_timeout"))
      {
        markRegistrationFailure(stamp, result.num_inliers, inlier_ratio, coverage.rmse_m,
          correction_translation, correction_rotation, result_generation, coverage.p95_m, false,
          "tracking input expired: " + result_rejection_reason);
      }
      RCLCPP_WARN(
        get_logger(), "Dropping completed GICP result generation=%" PRIu64
        " reason=%s mode=%s cloud_age=%.3fs processing_elapsed=%.1fms",
        result_generation, result_rejection_reason.c_str(),
        tracking_mode ? "tracking" : "initialization", result_cloud_age_sec,
        result_processing_elapsed_ms);
      return;
    }
    // A bounded main or alternate result may advance the preview only when its geometric
    // fit improves over the current seed. Formal acceptance below still requires the
    // original RMSE/P95, multi-start, and temporal gates.
    const bool preview_candidate_gate = finite_gate && inlier_count_gate &&
      inlier_ratio_gate && translation_gate && rotation_gate && coverage_points_gate &&
      coverage_ratio_gate && coverage_sectors_gate;
    const auto improvesSeedGeometry = [&seed_coverage](const MatchCoverage & candidate) {
        return std::isfinite(seed_coverage.rmse_m) &&
          std::isfinite(seed_coverage.p95_m) && std::isfinite(candidate.rmse_m) &&
          std::isfinite(candidate.p95_m) &&
          ((candidate.rmse_m < seed_coverage.rmse_m &&
          candidate.p95_m <= seed_coverage.p95_m) ||
          (candidate.p95_m < seed_coverage.p95_m &&
          candidate.rmse_m <= seed_coverage.rmse_m));
      };
    const bool preview_geometry_improved = improvesSeedGeometry(coverage);
    const bool alternate_preview_geometry_improved = alternate_ran &&
      improvesSeedGeometry(alternate_coverage);
    const bool main_preview_eligible = preview_candidate_gate && preview_geometry_improved;
    const bool alternate_preview_eligible = alternate_quality_gate &&
      alternate_preview_geometry_improved;
    const bool preview_candidate_available = main_preview_eligible || alternate_preview_eligible;
    const bool use_alternate_preview = alternate_preview_eligible &&
      (!main_preview_eligible || alternate_coverage.rmse_m + alternate_coverage.p95_m <
      coverage.rmse_m + coverage.p95_m);
    const auto preview_map_odom = use_alternate_preview
      ? small_gicp_relocalization::mapOdomFromMapBody(
      alternate_result.T_target_source, odom_body_eigen)
      : map_odom;
    const double preview_correction_translation = use_alternate_preview
      ? alternate_correction_translation : correction_translation;
    const double preview_correction_rotation = use_alternate_preview
      ? alternate_correction_rotation : correction_rotation;
    const double preview_rmse = use_alternate_preview
      ? alternate_coverage.rmse_m : coverage.rmse_m;
    const double preview_p95 = use_alternate_preview
      ? alternate_coverage.p95_m : coverage.p95_m;
    bool preview_updated = false;
    if (preview_candidate_available) {
      std::lock_guard<std::mutex> lock(state_mutex_);
      const bool can_update_preview = relocalization_state_ ==
        small_gicp_relocalization::msg::RelocalizationStatus::VERIFYING ||
        (manual_seed_active_ && relocalization_state_ ==
        small_gicp_relocalization::msg::RelocalizationStatus::LOST);
      if (generationIsAppliedLocked(result_generation) &&
        can_update_preview)
      {
        if (!has_trusted_map_odom_) {
          preview_map_odom_ = preview_map_odom;
        }
        status_detail_ = "GICP candidate preview; awaiting validation/continuity";
        preview_updated = true;
      }
    }
    if (preview_candidate_available) {
      const Eigen::Isometry3d candidate_map_body = preview_map_odom * odom_body_eigen;
      geometry_msgs::msg::PoseStamped candidate_pose;
      candidate_pose.header.stamp = stamp;
      candidate_pose.header.frame_id = map_frame_;
      candidate_pose.pose.position.x = candidate_map_body.translation().x();
      candidate_pose.pose.position.y = candidate_map_body.translation().y();
      candidate_pose.pose.position.z = candidate_map_body.translation().z();
      const Eigen::Quaterniond candidate_rotation(candidate_map_body.rotation());
      candidate_pose.pose.orientation.x = candidate_rotation.x();
      candidate_pose.pose.orientation.y = candidate_rotation.y();
      candidate_pose.pose.orientation.z = candidate_rotation.z();
      candidate_pose.pose.orientation.w = candidate_rotation.w();
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (generationIsAppliedLocked(result_generation)) {
          latest_candidate_pose_ = candidate_pose;
          has_latest_candidate_pose_ = true;
          latest_candidate_generation_ = result_generation;
        }
      }
      candidate_pose_publisher_->publish(candidate_pose);
    }
    if (preview_updated) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), static_cast<uint64_t>(timing_log_period_ms_),
        "GICP preview update generation=%" PRIu64 " seed=%s correction=(%.3fm, %.4frad) "
        "geometry=(%.3fm/%.3fm) quality=%s",
        result_generation, use_alternate_preview ? "alternate" : "main",
        preview_correction_translation, preview_correction_rotation,
        preview_rmse, preview_p95, passes_quality ? "pass" :
        "improved");
    }

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), static_cast<uint64_t>(timing_log_period_ms_),
      "GICP timing mode=%s stamp=%" PRId64 "ms source_voxel=%.3fm source_downsample=%.1fms covariance=%.1fms "
      "main_registration=%.1fms alternate_registration=%.1fms alternate_ran=%s "
      "total=%.1fms threads=%d max_iterations=%d alternate_max_iterations=%d",
      tracking_mode ? "tracking" : "initialization",
      static_cast<int64_t>(stamp.nanoseconds() / 1000000LL), input_voxel_size,
      source_downsample_ms, source_covariance_ms,
      main_registration_ms, alternate_elapsed_ms, alternate_ran ? "true" : "false",
      processing_elapsed_ms, num_threads_, effective_max_iterations, alternate_max_iterations);

    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (!generationIsAppliedLocked(result_generation)) {
        RCLCPP_WARN(
          get_logger(),
          "Discarding stale GICP result generation=%" PRIu64 " current_generation=%" PRIu64
          " applied_generation=%" PRIu64 " processing_elapsed=%.1fms",
          result_generation, initialization_generation_, applied_initialization_generation_,
          processing_elapsed_ms);
        return;
      }
    }

    if (!passes_quality) {
      std::size_t stable_frames_progress = 0;
      double stable_duration_progress = 0.0;
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (!generationIsAppliedLocked(result_generation)) {
          return;
        }
        stable_frames_progress = stable_map_odom_.size();
        if (has_stable_sequence_start_) {
          stable_duration_progress = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - stable_sequence_start_).count();
        }
      }
      std::string failed_gates;
      const auto add_failure = [&failed_gates](const char * name, bool passed) {
          if (!passed) {
            if (!failed_gates.empty()) failed_gates += ",";
            failed_gates += name;
          }
        };
      add_failure("finite", finite_gate);
      add_failure("convergence", convergence_gate);
      add_failure("inlier_count", inlier_count_gate);
      add_failure("inlier_ratio", inlier_ratio_gate);
      add_failure("geometry_rmse", geometry_rmse_gate);
      add_failure("geometry_p95", geometry_p95_gate);
      add_failure("translation_correction", translation_gate);
      add_failure("rotation_correction", rotation_gate);
      add_failure("tracking_translation_correction", tracking_translation_gate);
      add_failure("tracking_rotation_correction", tracking_rotation_gate);
      add_failure("coverage_points", coverage_points_gate);
      add_failure("coverage_ratio", coverage_ratio_gate);
      add_failure("coverage_sectors", coverage_sectors_gate);
      if (alternate_ran) {
        add_failure("multistart_alternate_quality", multistart_alternate_quality_gate);
        add_failure("multistart_translation_spread", multistart_translation_gate);
        add_failure("multistart_yaw_spread", multistart_yaw_gate);
      }
      add_failure("multistart_consistency", multistart_gate);
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (generationIsAppliedLocked(result_generation)) {
          status_detail_ = "registration rejected: " + failed_gates;
        }
      }
      saveOfflineSnapshotIfRequested("match_rejected", stamp);
      RCLCPP_WARN(
        get_logger(),
        "Rejecting GICP generation=%" PRIu64 " processing_elapsed=%.1fms "
        "registration_elapsed=%.1fms iterations=%zu/%d alternate_iterations=%zu/%d "
        "stable_frames=%zu/%d stable_duration=%.3f/%.3fs failed=[%s] "
        "converged=%s iteration_cap_accepted=%s inliers=%zu/%d ratio=%.4f "
        "geometry_rmse=%.6fm geometry_p95=%.6fm "
        "coverage_points=%zu ratio=%.4f sectors=%zu/%d "
        "correction=(%.4fm, %.4frad) gates=(finite=%s convergence=%s inlier_count=%s "
        "inlier_ratio=%s geometry_rmse=%s geometry_p95=%s translation=%s rotation=%s "
        "tracking_translation=%s tracking_rotation=%s "
        "coverage_points=%s coverage_ratio=%s coverage_sectors=%s multistart=%s) "
        "thresholds=(ratio>=%.4f rmse<=%.6fm p95<=%.6fm correction<=%.4fm/%.4frad "
        "tracking_correction<=%.4fm/%.4frad "
        "coverage_ratio>=%.4f sectors>=%d) "
        "alternate=(ran=%s converged=%s iteration_cap_accepted=%s "
        "inliers=%zu ratio=%.4f geometry_rmse=%.6fm "
        "geometry_p95=%.6fm "
        "correction=(%.4fm, %.4frad))",
        result_generation, processing_elapsed_ms, registration_elapsed_ms,
        result.iterations + 1U, effective_max_iterations,
        alternate_ran ? alternate_result.iterations + 1U : 0U,
        alternate_max_iterations, stable_frames_progress, stable_frames_required_, stable_duration_progress,
        min_stable_duration_sec_, failed_gates.c_str(),
        result.converged ? "true" : "false",
        main_iteration_cap_accepted ? "true" : "false",
        result.num_inliers, min_inliers_, inlier_ratio,
        coverage.rmse_m, coverage.p95_m, coverage.matched_points, coverage.ratio,
        coverage.sectors, min_match_sectors_,
        correction_translation, correction_rotation,
        finite_gate ? "pass" : "fail", convergence_gate ? "pass" : "fail",
        inlier_count_gate ? "pass" : "fail", inlier_ratio_gate ? "pass" : "fail",
        geometry_rmse_gate ? "pass" : "fail", geometry_p95_gate ? "pass" : "fail",
        translation_gate ? "pass" : "fail", rotation_gate ? "pass" : "fail",
        tracking_translation_gate ? "pass" : "fail", tracking_rotation_gate ? "pass" : "fail",
        coverage_points_gate ? "pass" : "fail", coverage_ratio_gate ? "pass" : "fail",
        coverage_sectors_gate ? "pass" : "fail", multistart_gate ? "pass" : "fail",
        min_inlier_ratio_, max_geometry_rmse_m_, max_geometry_p95_m_,
        max_translation_jump_, max_rotation_jump_, tracking_max_translation_correction_,
        tracking_max_rotation_correction_, min_inlier_ratio_, min_match_sectors_,
        alternate_ran ? "true" : "false", alternate_result.converged ? "true" : "false",
        alternate_iteration_cap_accepted ? "true" : "false",
        alternate_result.num_inliers, alternate_inlier_ratio, alternate_coverage.rmse_m,
        alternate_coverage.p95_m, alternate_correction_translation,
        alternate_correction_rotation);
      markRegistrationFailure(stamp, result.num_inliers, inlier_ratio, coverage.rmse_m,
        correction_translation, correction_rotation, result_generation, coverage.p95_m,
        tracking_mode && (!tracking_translation_gate || !tracking_rotation_gate),
        "GICP quality gates failed: " + failed_gates);
      return;
    }

    bool became_localized = false;
    bool is_localized = false;
    uint64_t accepted_localization_epoch = 0;
    std::uint32_t accepted_frames = 0;

    bool fastlio_healthy = false;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (!generationIsAppliedLocked(result_generation)) {
        RCLCPP_WARN(
          get_logger(),
          "Discarding stale GICP result generation=%" PRIu64 " current_generation=%" PRIu64
          " applied_generation=%" PRIu64 " before acceptance processing_elapsed=%.1fms",
          result_generation, initialization_generation_, applied_initialization_generation_,
          processing_elapsed_ms);
        return;
      }
      fastlio_healthy = fastlioHealthyLocked();
    }
    if (!fastlio_healthy) {
      RCLCPP_WARN(
        get_logger(),
        "Discarding GICP result generation=%" PRIu64 " elapsed=%.1fms because FAST-LIO "
        "health is false or stale",
        result_generation, registration_elapsed_ms);
      markRegistrationFailure(stamp, result.num_inliers, inlier_ratio, coverage.rmse_m,
        correction_translation, correction_rotation, result_generation, coverage.p95_m, false,
        "FAST-LIO health is false or stale");
      return;
    }

    if (tracking_mode) {
      const bool small_correction =
        correction_translation <= tracking_small_correction_translation_m_ &&
        correction_rotation <= tracking_small_correction_rotation_rad_;
      bool candidate_state_current = false;
      bool correction_confirmed = small_correction;
      uint32_t confirmation_count = 0;
      double candidate_spread_translation = 0.0;
      double candidate_spread_rotation = 0.0;
      // Compare map->odom candidates in the shared map/odom frames. Comparing
      // map->body deltas here makes a fixed correction appear to vary as the
      // robot moves or rotates between scans.
      const Eigen::Vector3d candidate_translation_delta =
        map_odom.translation() - predicted_map_odom.translation();
      Eigen::Quaterniond candidate_rotation_delta(
        map_odom.rotation() * predicted_map_odom.rotation().transpose());
      candidate_rotation_delta.normalize();
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        candidate_state_current = generationIsAppliedLocked(result_generation) &&
          relocalization_state_ == result_state && has_accepted_pose_;
        if (candidate_state_current) {
          if (small_correction) {
            correction_confirmed = true;
          } else {
            candidate_spread_translation = has_pending_tracking_candidate_
              ? (candidate_translation_delta - pending_tracking_translation_delta_).norm()
              : std::numeric_limits<double>::infinity();
            if (has_pending_tracking_candidate_) {
              Eigen::Quaterniond rotation_spread = candidate_rotation_delta *
                pending_tracking_rotation_delta_.inverse();
              rotation_spread.normalize();
              candidate_spread_rotation = 2.0 * std::acos(std::clamp(
                  std::abs(rotation_spread.w()), 0.0, 1.0));
            } else {
              candidate_spread_rotation = std::numeric_limits<double>::infinity();
            }
            const bool consistent = has_pending_tracking_candidate_ &&
              candidate_spread_translation <= tracking_consistency_translation_tolerance_m_ &&
              candidate_spread_rotation <= tracking_consistency_rotation_tolerance_rad_;
            if (consistent) {
              ++pending_tracking_candidate_count_;
            } else {
              pending_tracking_translation_delta_ = candidate_translation_delta;
              pending_tracking_rotation_delta_ = candidate_rotation_delta;
              pending_tracking_candidate_count_ = 1;
              has_pending_tracking_candidate_ = true;
            }
            confirmation_count = pending_tracking_candidate_count_;
            correction_confirmed = confirmation_count >=
              static_cast<uint32_t>(tracking_consistency_frames_required_);
            if (!correction_confirmed) {
              status_detail_ = "tracking candidate awaiting consecutive confirmation";
              publishStatusLocked(stamp, result.num_inliers, inlier_ratio, coverage.rmse_m,
                correction_translation, correction_rotation, coverage.sectors, coverage.ratio,
                multistart_consistent, coverage.p95_m);
            }
          }
        }
      }
      if (!candidate_state_current) {
        RCLCPP_WARN(
          get_logger(), "Dropping tracking result generation=%" PRIu64
          " because localization changed before continuity confirmation",
          result_generation);
        return;
      }
      if (!correction_confirmed) {
        saveOfflineSnapshotIfRequested("tracking_candidate_pending", stamp);
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), static_cast<uint64_t>(timing_log_period_ms_),
          "Holding tracking correction pending consecutive confirmation count=%u/%d "
          "correction=(%.3fm, %.4frad) previous_map_to_odom_candidate_spread=(%.3fm, %.4frad)",
          confirmation_count, tracking_consistency_frames_required_, correction_translation,
          correction_rotation, candidate_spread_translation, candidate_spread_rotation);
        return;
      }
    }

    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = stamp;
    pose.header.frame_id = map_frame_;
    pose.pose.position.x = map_body.translation().x();
    pose.pose.position.y = map_body.translation().y();
    pose.pose.position.z = map_body.translation().z();
    const Eigen::Quaterniond rotation(map_body.rotation());
    pose.pose.orientation.x = rotation.x();
    pose.pose.orientation.y = rotation.y();
    pose.pose.orientation.z = rotation.z();
    pose.pose.orientation.w = rotation.w();

    pcl::PointCloud<pcl::PointXYZ> aligned_points;
    aligned_points.resize(source_cloud->size());
    for (size_t i = 0; i < source_cloud->size(); ++i) {
      const Eigen::Vector3d transformed = map_body * Eigen::Vector3d(
        source_cloud->at(i).x, source_cloud->at(i).y, source_cloud->at(i).z);
      aligned_points[i].x = transformed.x();
      aligned_points[i].y = transformed.y();
      aligned_points[i].z = transformed.z();
    }
    sensor_msgs::msg::PointCloud2 aligned_cloud;
    pcl::toROSMsg(aligned_points, aligned_cloud);
    aligned_cloud.header = message->header;
    aligned_cloud.header.frame_id = map_frame_;
    bool published_result = false;
    bool final_result_stale = false;
    bool final_fastlio_unhealthy = false;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (!generationIsAppliedLocked(result_generation)) {
        RCLCPP_WARN(
          get_logger(),
          "Discarding stale GICP generation=%" PRIu64
          " before publishing pose/aligned cloud current_generation=%" PRIu64
          " applied_generation=%" PRIu64 " processing_elapsed=%.1fms",
          result_generation, initialization_generation_, applied_initialization_generation_,
          processing_elapsed_ms);
        return;
      }
      if (relocalization_state_ != result_state ||
        (tracking_mode && !has_accepted_pose_))
      {
        RCLCPP_WARN(
          get_logger(), "Dropping GICP result generation=%" PRIu64
          " because localization state changed before publication",
          result_generation);
        return;
      }
      if (tracking_mode) {
        const double final_cloud_age_sec = (now() - stamp).seconds();
        const double final_processing_elapsed_ms = elapsedMilliseconds(processing_start);
        if (!std::isfinite(final_cloud_age_sec) || final_cloud_age_sec < -0.05 ||
          final_cloud_age_sec > tracking_max_cloud_age_sec_ ||
          !std::isfinite(final_processing_elapsed_ms) ||
          final_processing_elapsed_ms > tracking_max_processing_time_sec_ * 1000.0)
        {
          RCLCPP_WARN(
            get_logger(), "Dropping tracking result generation=%" PRIu64
            " before commit cloud_age=%.3fs processing_elapsed=%.1fms limits=(%.3fs/%.1fms)",
            result_generation, final_cloud_age_sec, final_processing_elapsed_ms,
            tracking_max_cloud_age_sec_, tracking_max_processing_time_sec_ * 1000.0);
          final_result_stale = true;
        }
      }
      if (!final_result_stale && !fastlioHealthyLocked()) {
        RCLCPP_WARN(
          get_logger(), "Dropping GICP result generation=%" PRIu64
          " before commit because FAST-LIO health is false or stale",
          result_generation);
        final_fastlio_unhealthy = true;
      }

      if (!final_result_stale && !final_fastlio_unhealthy) {
        last_map_odom_ = map_odom;
        has_accepted_pose_ = true;
        has_last_map_odom_ = true;
        stable_map_odom_.push_back(map_odom);
        if (!has_stable_sequence_start_) {
          stable_sequence_start_ = std::chrono::steady_clock::now();
          has_stable_sequence_start_ = true;
        }
        if (stable_map_odom_.size() > static_cast<std::size_t>(stable_frames_required_)) {
          stable_map_odom_.pop_front();
        }
        ++consecutive_successes_;
        accepted_frames = consecutive_successes_;
        const double stable_duration = std::chrono::duration<double>(
          std::chrono::steady_clock::now() - stable_sequence_start_).count();
        const bool was_localized = relocalization_state_ ==
          small_gicp_relocalization::msg::RelocalizationStatus::LOCALIZED;
        if (was_localized ||
          (stable_map_odom_.size() >= static_cast<std::size_t>(stable_frames_required_) &&
          stablePoseSpreadWithinGate() && stable_duration >= min_stable_duration_sec_))
        {
          became_localized = relocalization_state_ !=
            small_gicp_relocalization::msg::RelocalizationStatus::LOCALIZED;
          if (became_localized) {
            ++localization_epoch_;
          }
          relocalization_state_ = small_gicp_relocalization::msg::RelocalizationStatus::LOCALIZED;
          manual_seed_active_ = false;
          candidate_source_ = candidate_source_ == "manual" ? "manual_verified" :
            "bbs3d_verified";
          status_detail_ = "localized; verified transform accepted";
          has_automatic_recovery_start_wall_time_ = false;
        } else {
          relocalization_state_ = small_gicp_relocalization::msg::RelocalizationStatus::VERIFYING;
          status_detail_ = "candidate accepted; waiting for consecutive verification";
        }
        if (relocalization_state_ ==
          small_gicp_relocalization::msg::RelocalizationStatus::LOCALIZED)
        {
          preview_map_odom_ = map_odom;
          last_trusted_map_odom_ = map_odom;
          has_trusted_map_odom_ = true;
          last_trusted_success_wall_time_ = std::chrono::steady_clock::now();
          has_last_trusted_success_wall_time_ = true;
          is_localized = true;
          accepted_localization_epoch = localization_epoch_;
        } else if (preview_candidate_available) {
          preview_map_odom_ = preview_map_odom;
        }
        if (alternate_ran) {
          has_multistart_validation_ = multistart_consistent;
          last_multistart_consistent_ = multistart_consistent;
        }
        has_pending_tracking_candidate_ = false;
        pending_tracking_candidate_count_ = 0;
        consecutive_failures_ = 0;
        last_success_wall_time_ = std::chrono::steady_clock::now();
        has_success_wall_time_ = true;
        last_success_stamp_.sec = static_cast<int32_t>(stamp.nanoseconds() / 1000000000LL);
        last_success_stamp_.nanosec = static_cast<uint32_t>(stamp.nanoseconds() % 1000000000LL);
        publishStatusLocked(stamp, result.num_inliers, inlier_ratio, coverage.rmse_m,
          correction_translation, correction_rotation, coverage.sectors, coverage.ratio,
          multistart_consistent, coverage.p95_m);
        pose_publisher_->publish(pose);
        aligned_cloud_publisher_->publish(aligned_cloud);
        published_result = true;
      }
    }
    if (final_result_stale) {
      markRegistrationFailure(stamp, result.num_inliers, inlier_ratio, coverage.rmse_m,
        correction_translation, correction_rotation, result_generation, coverage.p95_m, false,
        "matching input expired before commit");
      return;
    }
    if (final_fastlio_unhealthy) {
      markRegistrationFailure(stamp, result.num_inliers, inlier_ratio, coverage.rmse_m,
        correction_translation, correction_rotation, result_generation, coverage.p95_m, false,
        "FAST-LIO health expired before commit");
      return;
    }

    if (is_localized && published_result) {
      small_gicp_relocalization::msg::MapOdomTarget target;
      target.transform = toTransform(map_odom, map_frame_, odom_frame_, stamp);
      target.producer_session_id = producer_session_id_;
      target.initialization_generation = result_generation;
      target.localization_epoch = accepted_localization_epoch;
      handoff_publisher_->publish(target);
      if (became_localized) {
        saveOfflineSnapshotIfRequested("normal_localized", stamp);
        RCLCPP_INFO(
          get_logger(), "Initial accepted localization published on %s", handoff_topic_.c_str());
      } else {
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), static_cast<uint64_t>(timing_log_period_ms_),
          "Published accepted tracking correction=(%.3fm, %.4frad) on %s",
          correction_translation, correction_rotation, handoff_topic_.c_str());
      }
      if (became_localized && exit_after_localized_) {
        handoff_exit_timer_ = create_wall_timer(
          std::chrono::milliseconds(250), [this]() {
            if (handoff_exit_timer_) {
              handoff_exit_timer_->cancel();
            }
            RCLCPP_INFO(get_logger(), "GICP stage exiting after localization handoff");
            rclcpp::shutdown();
          });
      }
      if (became_localized) {
        RCLCPP_INFO(get_logger(),
          "Relocalization SUCCESS: generation=%" PRIu64 " map->odom is stable after %u "
          "accepted frames (processing_elapsed=%.1fms registration_elapsed=%.1fms)",
          result_generation, accepted_frames, processing_elapsed_ms, registration_elapsed_ms);
      }
    }
  }

  static double elapsedMilliseconds(const std::chrono::steady_clock::time_point & start)
  {
    return std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - start).count();
  }

  bool registrationResultCurrentAndFresh(
    const rclcpp::Time & stamp,
    const std::chrono::steady_clock::time_point & processing_start,
    const bool tracking_mode, const uint8_t expected_state, const std::uint64_t generation,
    double & cloud_age_sec, double & processing_elapsed_ms, std::string & rejection_reason)
  {
    cloud_age_sec = (now() - stamp).seconds();
    processing_elapsed_ms = elapsedMilliseconds(processing_start);
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (!generationIsAppliedLocked(generation)) {
        rejection_reason = "generation_changed";
        return false;
      }
      if (relocalization_state_ != expected_state ||
        (tracking_mode && !has_accepted_pose_))
      {
        rejection_reason = "localization_state_changed";
        return false;
      }
    }
    if (!tracking_mode) {
      return true;
    }
    if (!std::isfinite(cloud_age_sec) || cloud_age_sec < -0.05 ||
      cloud_age_sec > tracking_max_cloud_age_sec_)
    {
      rejection_reason = "cloud_stale";
      return false;
    }
    if (!std::isfinite(processing_elapsed_ms) ||
      processing_elapsed_ms > tracking_max_processing_time_sec_ * 1000.0)
    {
      rejection_reason = "processing_timeout";
      return false;
    }
    return true;
  }

  bool generationIsAppliedLocked(std::uint64_t generation) const
  {
    return has_initial_pose_ &&
      small_gicp_relocalization::resultBelongsToAppliedInitialization(
      generation, initialization_generation_, applied_initialization_generation_);
  }

  static bool isFiniteIsometry(const Eigen::Isometry3d & transform)
  {
    return transform.matrix().array().isFinite().all();
  }

  bool clearPendingTrackingCandidateLocked()
  {
    if (!has_pending_tracking_candidate_ && pending_tracking_candidate_count_ == 0) {
      return false;
    }
    has_pending_tracking_candidate_ = false;
    pending_tracking_candidate_count_ = 0;
    return true;
  }

  void interruptPendingTrackingCandidateForSkippedFrame(const char * reason)
  {
    bool cleared = false;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (relocalization_state_ ==
        small_gicp_relocalization::msg::RelocalizationStatus::LOCALIZED &&
        has_accepted_pose_)
      {
        cleared = clearPendingTrackingCandidateLocked();
        status_detail_ = std::string("input expired/skipped: ") + reason;
        publishStatusLocked(now());
      }
    }
    if (cleared) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), static_cast<uint64_t>(timing_log_period_ms_),
        "Cleared pending tracking correction after skipped matching opportunity: %s", reason);
    }
  }

  void markRegistrationFailure(
    const rclcpp::Time & stamp, std::size_t num_inliers = 0, double inlier_ratio = 0.0,
    double geometry_rmse_m = 0.0, double correction_translation = 0.0,
    double correction_rotation = 0.0, std::uint64_t generation = 0,
    double geometry_p95_m = 0.0, bool immediate_loss = false,
    const std::string & failure_detail = "")
  {
    bool changed = false;
    bool entered_lost = false;
    bool manual_candidate_failed = false;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (generation != 0 && !generationIsAppliedLocked(generation)) {
        return;
      }
      if (!has_initial_pose_) return;
      clearPendingTrackingCandidateLocked();
      consecutive_successes_ = 0;
      stable_map_odom_.clear();
      has_stable_sequence_start_ = false;
      if (!map_ready_.load(std::memory_order_acquire)) {
        relocalization_state_ = small_gicp_relocalization::msg::RelocalizationStatus::MAP_LOADING;
      } else if (relocalization_state_ ==
        small_gicp_relocalization::msg::RelocalizationStatus::LOCALIZED)
      {
        ++consecutive_failures_;
        const double elapsed_since_success = has_success_wall_time_
          ? std::chrono::duration<double>(
          std::chrono::steady_clock::now() - last_success_wall_time_).count()
          : std::numeric_limits<double>::infinity();
        if (immediate_loss || !small_gicp_relocalization::localizedFailureKeepsValidity(
            consecutive_failures_, static_cast<std::uint32_t>(max_consecutive_failures_),
            elapsed_since_success, validity_timeout_sec_))
        {
          relocalization_state_ = small_gicp_relocalization::msg::RelocalizationStatus::LOST;
        }
      } else if (relocalization_state_ !=
        small_gicp_relocalization::msg::RelocalizationStatus::LOST)
      {
        const bool geometric_manual_rejection = manual_seed_active_ &&
          (failure_detail.rfind("GICP quality gates failed", 0) == 0 ||
          failure_detail.rfind("too few source points", 0) == 0);
        if (geometric_manual_rejection) {
          ++consecutive_failures_;
          if (consecutive_failures_ >= static_cast<std::uint32_t>(max_consecutive_failures_)) {
            relocalization_state_ = small_gicp_relocalization::msg::RelocalizationStatus::LOST;
            manual_candidate_failed = true;
          } else {
            relocalization_state_ = small_gicp_relocalization::msg::RelocalizationStatus::VERIFYING;
          }
        } else {
          relocalization_state_ = has_initial_pose_
            ? small_gicp_relocalization::msg::RelocalizationStatus::VERIFYING
            : small_gicp_relocalization::msg::RelocalizationStatus::WAITING_INITIAL_POSE;
        }
      }
      if (relocalization_state_ == small_gicp_relocalization::msg::RelocalizationStatus::LOST) {
        has_success_wall_time_ = false;
        status_detail_ = failure_detail.empty()
          ? "localization lost after registration failure"
          : "localization lost after: " + failure_detail;
        candidate_source_ = manual_seed_active_ ? "manual_failed" : "automatic_recovery";
        if (!manual_seed_active_ && !has_automatic_recovery_start_wall_time_) {
          automatic_recovery_start_wall_time_ = std::chrono::steady_clock::now();
          has_automatic_recovery_start_wall_time_ = true;
        }
        entered_lost = true;
      } else {
        status_detail_ = failure_detail.empty()
          ? "registration failed; last trusted transform retained"
          : "last trusted transform retained; " + failure_detail;
      }
      publishStatusLocked(stamp, num_inliers, inlier_ratio, geometry_rmse_m, correction_translation,
        correction_rotation, 0, 0.0, false, geometry_p95_m);
      changed = true;
    }
    if (changed) saveOfflineSnapshotIfRequested("registration_failure", stamp);
    if (entered_lost) saveOfflineSnapshotIfRequested("lost", stamp);
    if (manual_candidate_failed) {
      cancelManualSeedAndResumeAuto("repeated GICP quality rejection", true);
    }
  }

  static double yaw(const Eigen::Isometry3d & transform)
  {
    return std::atan2(transform.rotation()(1, 0), transform.rotation()(0, 0));
  }

  MatchCoverage evaluateMatchCoverage(
    const Eigen::Isometry3d &map_body, const CovarianceCloud &source_cloud) const
  {
    MatchCoverage coverage;
    if (!isFiniteIsometry(map_body) || !map_tree_ || !map_cloud_ || map_cloud_->empty()) {
      return coverage;
    }

    std::vector<Eigen::Vector2d> matched_points;
    matched_points.reserve(source_cloud.size());
    std::vector<double> distances_m;
    distances_m.reserve(source_cloud.size());
    double squared_distance_sum = 0.0;
    const double max_distance_sq = max_correspondence_distance_ * max_correspondence_distance_;
    for (const auto &point : source_cloud) {
      const Eigen::Vector3d transformed = map_body * Eigen::Vector3d(point.x, point.y, point.z);
      if (!transformed.allFinite()) {
        continue;
      }
      const Eigen::Vector4d query(transformed.x(), transformed.y(), transformed.z(), 1.0);
      std::size_t index = 0;
      double squared_distance = 0.0;
      if (small_gicp::traits::nearest_neighbor_search(
          *map_tree_, query, &index, &squared_distance) != 0 &&
        std::isfinite(squared_distance) && squared_distance <= max_distance_sq)
      {
        matched_points.emplace_back(transformed.x(), transformed.y());
        const double distance_m = std::sqrt(std::max(0.0, squared_distance));
        distances_m.push_back(distance_m);
        squared_distance_sum += squared_distance;
      }
    }

    coverage.matched_points = matched_points.size();
    coverage.ratio = source_cloud.empty() ? 0.0 :
      static_cast<double>(coverage.matched_points) / static_cast<double>(source_cloud.size());
    if (!distances_m.empty()) {
      coverage.rmse_m = std::sqrt(squared_distance_sum /
        static_cast<double>(distances_m.size()));
      const auto p95_index = static_cast<std::size_t>(0.95 *
        static_cast<double>(distances_m.size() - 1));
      std::nth_element(distances_m.begin(), distances_m.begin() + p95_index, distances_m.end());
      coverage.p95_m = distances_m[p95_index];
    }
    if (matched_points.empty()) {
      return coverage;
    }

    Eigen::Vector2d centroid = Eigen::Vector2d::Zero();
    for (const auto &point : matched_points) {
      centroid += point;
    }
    centroid /= static_cast<double>(matched_points.size());
    std::array<bool, 8> occupied_sectors{};
    for (const auto &point : matched_points) {
      const double angle = std::atan2(point.y() - centroid.y(), point.x() - centroid.x());
      const auto sector = static_cast<std::size_t>(std::min(7.0, std::floor(
        (angle + 3.14159265358979323846) /
        (2.0 * 3.14159265358979323846) * 8.0)));
      occupied_sectors[sector] = true;
    }
    coverage.sectors = static_cast<std::size_t>(std::count(
      occupied_sectors.begin(), occupied_sectors.end(), true));
    return coverage;
  }

  bool stablePoseSpreadWithinGate() const
  {
    for (const auto & first : stable_map_odom_) {
      for (const auto & second : stable_map_odom_) {
        if ((first.translation() - second.translation()).norm() > max_stable_translation_spread_) {
          return false;
        }
        const double delta = std::atan2(std::sin(yaw(first) - yaw(second)),
          std::cos(yaw(first) - yaw(second)));
        if (std::abs(delta) > max_stable_yaw_spread_) return false;
      }
    }
    return true;
  }

  void publishStatusLocked(
    const rclcpp::Time & stamp, std::size_t num_inliers = 0, double inlier_ratio = 0.0,
    double geometry_rmse_m = 0.0, double correction_translation = 0.0,
    double correction_rotation = 0.0, std::size_t match_sectors = 0,
    double match_coverage_ratio = 0.0, bool multistart_consistent = false,
    double geometry_p95_m = 0.0)
  {
    small_gicp_relocalization::msg::RelocalizationStatus status;
    status.header.stamp = stamp;
    status.header.frame_id = map_frame_;
    status.state = relocalization_state_;
    status.valid = relocalization_state_ ==
      small_gicp_relocalization::msg::RelocalizationStatus::LOCALIZED;
    status.degraded = status.valid &&
      (consecutive_failures_ > 0 || has_pending_tracking_candidate_);
    status.fastlio_healthy = fastlioHealthyLocked();
    status.producer_session_id = producer_session_id_;
    status.initialization_generation = initialization_generation_;
    status.localization_epoch = localization_epoch_;
    status.consecutive_successes = consecutive_successes_;
    status.num_inliers = static_cast<uint32_t>(std::min<std::size_t>(num_inliers, UINT32_MAX));
    status.inlier_ratio = static_cast<float>(inlier_ratio);
    status.match_sectors = static_cast<uint32_t>(std::min<std::size_t>(match_sectors, UINT32_MAX));
    status.match_coverage_ratio = static_cast<float>(match_coverage_ratio);
    status.multistart_consistent = multistart_consistent;
    status.geometry_rmse_m = geometry_rmse_m;
    status.geometry_p95_m = geometry_p95_m;
    status.correction_translation = correction_translation;
    status.correction_rotation = correction_rotation;
    status.last_success_time = last_success_stamp_;
    status.candidate_source = candidate_source_;
    status.state_detail = status_detail_;
    status.last_trusted_pose_age_sec = has_last_trusted_success_wall_time_
      ? std::chrono::duration<double>(
        std::chrono::steady_clock::now() - last_trusted_success_wall_time_).count()
      : -1.0;
    if (manual_seed_active_) {
      const auto current = std::chrono::steady_clock::now();
      const double elapsed_since_received = std::chrono::duration<double>(
        current - manual_seed_received_wall_time_).count();
      const double elapsed_since_usable_cloud = has_verification_start_wall_time_
        ? std::chrono::duration<double>(current - verification_start_wall_time_).count() : 0.0;
      const auto deadline_state = small_gicp_relocalization::manualSeedDeadlineState(
        true, has_verification_start_wall_time_, elapsed_since_received,
        elapsed_since_usable_cloud, manual_seed_input_timeout_sec_,
        manual_seed_validation_timeout_sec_);
      status.manual_seed_time_remaining_sec = deadline_state ==
        small_gicp_relocalization::ManualSeedDeadlineState::WAITING_FOR_INPUT
        ? std::max(0.0, manual_seed_input_timeout_sec_ - elapsed_since_received)
        : (deadline_state == small_gicp_relocalization::ManualSeedDeadlineState::VALIDATING
        ? std::max(0.0, manual_seed_validation_timeout_sec_ - elapsed_since_usable_cloud) : 0.0);
    } else {
      status.manual_seed_time_remaining_sec = 0.0;
    }
    status.map_point_count = static_cast<uint32_t>(std::min<std::size_t>(
        map_cloud_ ? map_cloud_->size() : 0, UINT32_MAX));
    status.source_point_count = static_cast<uint32_t>(std::min<std::size_t>(
        last_snapshot_cloud_ ? last_snapshot_cloud_->size() : 0, UINT32_MAX));
    status.point_cloud_age_sec = -1.0;
    if (last_snapshot_stamp_ns_ > 0) {
      try {
        const rclcpp::Time cloud_stamp(last_snapshot_stamp_ns_, get_clock()->get_clock_type());
        status.point_cloud_age_sec = std::max(0.0, (stamp - cloud_stamp).seconds());
      } catch (const std::exception &) {
        status.point_cloud_age_sec = -1.0;
      }
    }
    status.tf_wait_sec = latest_tf_wait_sec_;
    status.matching_duration_ms = latest_match_duration_ms_;
    status.consecutive_failures = consecutive_failures_;
    status.pending_candidate_confirmations = pending_tracking_candidate_count_;
    status.automatic_recovery_active = !manual_seed_active_ &&
      relocalization_state_ == small_gicp_relocalization::msg::RelocalizationStatus::LOST;
    status.automatic_recovery_elapsed_sec = status.automatic_recovery_active &&
      has_automatic_recovery_start_wall_time_
      ? std::chrono::duration<double>(
        std::chrono::steady_clock::now() - automatic_recovery_start_wall_time_).count()
      : 0.0;
    if (num_inliers > 0 || geometry_rmse_m > 0.0 || geometry_p95_m > 0.0) {
      last_match_status_ = status;
    }
    status_publisher_->publish(status);
  }

  void heartbeatCallback()
  {
    tryApplyPendingInitialPose("heartbeat timer");
    const auto heartbeat_stamp = now();
    Eigen::Isometry3d map_odom = Eigen::Isometry3d::Identity();
    bool timed_out = false;
    bool manual_seed_timed_out = false;
    std::string timeout_reason;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (!map_ready_.load(std::memory_order_acquire)) {
        relocalization_state_ = small_gicp_relocalization::msg::RelocalizationStatus::MAP_LOADING;
      } else if (manual_seed_active_) {
        const auto current = std::chrono::steady_clock::now();
        const double elapsed_since_received = std::chrono::duration<double>(
          current - manual_seed_received_wall_time_).count();
        const double elapsed_since_usable_cloud = has_verification_start_wall_time_
          ? std::chrono::duration<double>(current - verification_start_wall_time_).count() : 0.0;
        const auto deadline_state = small_gicp_relocalization::manualSeedDeadlineState(
          true, has_verification_start_wall_time_, elapsed_since_received,
          elapsed_since_usable_cloud, manual_seed_input_timeout_sec_,
          manual_seed_validation_timeout_sec_);
        if (deadline_state == small_gicp_relocalization::ManualSeedDeadlineState::EXPIRED) {
          manual_seed_timed_out = true;
          timeout_reason = has_verification_start_wall_time_ ? "GICP validation deadline expired" :
            "TF/input wait deadline expired";
        }
      } else if (has_initial_pose_ && !has_pending_initial_pose_ &&
        has_verification_start_wall_time_) {
        const auto reference = has_success_wall_time_
          ? last_success_wall_time_ : verification_start_wall_time_;
        if (std::chrono::duration<double>(std::chrono::steady_clock::now() - reference).count() <=
          validity_timeout_sec_)
        {
          // The preview or last accepted transform remains active while verifying.
        } else if (relocalization_state_ !=
          small_gicp_relocalization::msg::RelocalizationStatus::LOST)
        {
          relocalization_state_ = small_gicp_relocalization::msg::RelocalizationStatus::LOST;
          consecutive_successes_ = 0;
          stable_map_odom_.clear();
          has_stable_sequence_start_ = false;
          consecutive_failures_ = 0;
          has_success_wall_time_ = false;
          clearPendingTrackingCandidateLocked();
          status_detail_ = "localization validity expired; automatic relocalization available";
          candidate_source_ = "automatic_recovery";
          automatic_recovery_start_wall_time_ = std::chrono::steady_clock::now();
          has_automatic_recovery_start_wall_time_ = true;
          timed_out = true;
        }
      }
      map_odom = preview_map_odom_;
      publishStatusLocked(heartbeat_stamp);
      if (has_trusted_map_odom_ || has_initial_pose_) {
        preview_map_odom_publisher_->publish(
          toTransform(map_odom, map_frame_, odom_frame_, heartbeat_stamp));
      }
    }
    if (manual_seed_timed_out) {
      RCLCPP_ERROR(
        get_logger(), "Manual seed failed: %s after input_wait=%.1fs validation=%.1fs; "
        "releasing BBS and returning to automatic relocalization",
        timeout_reason.c_str(), manual_seed_input_timeout_sec_,
        manual_seed_validation_timeout_sec_);
      cancelManualSeedAndResumeAuto(timeout_reason.c_str(), true);
    }
    if (timed_out) {
      RCLCPP_WARN(
        get_logger(), "Relocalization validity timed out after %.1f seconds; entering LOST",
        validity_timeout_sec_);
      saveOfflineSnapshotIfRequested("lost", heartbeat_stamp);
    }
  }

  bool fastlioHealthyLocked() const
  {
    return small_gicp_relocalization::fastlioHealthIsUsable(
      has_fastlio_health_, fastlio_healthy_,
      std::chrono::duration<double>(
      std::chrono::steady_clock::now() - last_fastlio_health_wall_time_).count(),
      fastlio_health_timeout_sec_);
  }

  std::string pcd_file_;
  std::string cloud_topic_;
  std::string initial_pose_topic_;
  std::string bbs_global_pose_topic_;
  double bbs_seed_retry_sec_;
  std::string map_frame_;
  std::string odom_frame_;
  std::string body_frame_;
  std::string base_frame_;
  std::string odometry_topic_;
  double registration_voxel_size_;
  double tracking_input_voxel_size_;
  double display_voxel_size_;
  int num_neighbors_;
  int num_threads_;
  double max_correspondence_distance_;
  int max_iterations_;
  double tracking_frequency_hz_;
  int tracking_max_iterations_;
  double tracking_max_processing_time_sec_;
  double tracking_small_correction_translation_m_;
  double tracking_small_correction_rotation_rad_;
  double tracking_consistency_translation_tolerance_m_;
  double tracking_consistency_rotation_tolerance_rad_;
  int tracking_consistency_frames_required_;
  double tracking_max_translation_correction_;
  double tracking_max_rotation_correction_;
  double tracking_max_cloud_age_sec_;
  int min_inliers_;
  double max_translation_jump_;
  double max_rotation_jump_;
  double min_inlier_ratio_;
  double max_geometry_rmse_m_;
  double max_geometry_p95_m_;
  int stable_frames_required_;
  double min_stable_duration_sec_;
  double max_stable_translation_spread_;
  double max_stable_yaw_spread_;
  int min_match_sectors_;
  bool require_multistart_consistency_;
  double multistart_seed_translation_;
  double multistart_seed_yaw_;
  double max_multistart_translation_spread_;
  double max_multistart_yaw_spread_;
  double validity_timeout_sec_;
  int max_consecutive_failures_;
  double tf_lookup_timeout_sec_;
  double manual_seed_input_timeout_sec_;
  double manual_seed_validation_timeout_sec_;
  double manual_seed_bbs_height_max_age_sec_;
  int timing_log_period_ms_;
  std::string offline_snapshot_dir_;
  int offline_snapshot_max_count_{8};
  std::string navigation_map_file_;
  std::string startup_context_;
  std::string handoff_topic_;
  bool exit_after_localized_{false};
  std::string fastlio_health_topic_;
  double fastlio_health_timeout_sec_;

  std::atomic_bool map_ready_{false};
  bool has_initial_pose_{false};
  bool manual_seed_active_{false};
  bool has_accepted_pose_{false};
  bool has_last_map_odom_{false};
  bool has_pending_initial_pose_{false};
  bool has_pending_tracking_candidate_{false};
  bool pending_pose_is_map_body_{false};
  bool has_bbs_seed_time_{false};
  bool has_trusted_map_odom_{false};
  bool has_last_trusted_success_wall_time_{false};
  bool has_latest_bbs_pose_{false};
  bool have_static_tf_snapshot_{false};
  builtin_interfaces::msg::Time pending_pose_stamp_{};
  std::chrono::steady_clock::time_point last_bbs_seed_time_{};
  uint8_t relocalization_state_{small_gicp_relocalization::msg::RelocalizationStatus::MAP_LOADING};
  uint64_t initialization_generation_{0};
  uint64_t applied_initialization_generation_{0};
  uint64_t producer_session_id_{0};
  uint64_t localization_epoch_{0};
  uint32_t pending_tracking_candidate_count_{0};
  uint32_t consecutive_successes_{0};
  uint32_t consecutive_failures_{0};
  int64_t last_processed_stamp_ns_{0};
  bool has_success_wall_time_{false};
  bool has_verification_start_wall_time_{false};
  bool has_multistart_validation_{false};
  bool last_multistart_consistent_{false};
  bool has_tracking_attempt_wall_time_{false};
  bool has_fastlio_health_{false};
  bool fastlio_healthy_{false};
  std::chrono::steady_clock::time_point manual_seed_received_wall_time_{};
  std::chrono::steady_clock::time_point last_success_wall_time_{};
  std::chrono::steady_clock::time_point last_trusted_success_wall_time_{};
  std::chrono::steady_clock::time_point last_tracking_attempt_wall_time_{};
  std::chrono::steady_clock::time_point verification_start_wall_time_{};
  std::chrono::steady_clock::time_point stable_sequence_start_{};
  bool has_stable_sequence_start_{false};
  builtin_interfaces::msg::Time last_success_stamp_{};
  std::deque<Eigen::Isometry3d> stable_map_odom_;
  std::deque<std::pair<int64_t, Eigen::Isometry3d>> odom_body_history_;
  std::deque<nav_msgs::msg::Odometry> odometry_history_;
  std::deque<tf2_msgs::msg::TFMessage> dynamic_tf_history_;
  small_gicp_relocalization::RecentSnapshotEventCache snapshot_events_saved_{
    small_gicp_relocalization::kSnapshotEventCacheCapacity};
  std::uint64_t snapshot_sequence_{0};
  std::uint64_t last_snapshot_generation_{0};
  int64_t last_snapshot_stamp_ns_{0};
  bool last_snapshot_has_odom_{false};
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr last_snapshot_cloud_;
  Eigen::Isometry3d last_snapshot_odom_body_{Eigen::Isometry3d::Identity()};
  geometry_msgs::msg::Pose pending_initial_pose_;
  geometry_msgs::msg::Pose seed_initial_pose_;
  geometry_msgs::msg::PoseStamped latest_candidate_pose_;
  bool has_latest_candidate_pose_{false};
  std::uint64_t latest_candidate_generation_{0};
  small_gicp_relocalization::msg::RelocalizationStatus last_match_status_;
  std::array<double, 36> seed_initial_pose_covariance_{};
  bool seed_initial_pose_is_map_body_{false};
  geometry_msgs::msg::Pose latest_bbs_pose_;
  builtin_interfaces::msg::Time latest_bbs_pose_stamp_{};
  std::string candidate_source_{"automatic"};
  std::string status_detail_{"waiting for initial pose"};
  Eigen::Isometry3d initial_map_odom_{Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d preview_map_odom_{Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d last_map_odom_{Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d last_trusted_map_odom_{Eigen::Isometry3d::Identity()};
  Eigen::Vector3d pending_tracking_translation_delta_{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond pending_tracking_rotation_delta_{Eigen::Quaterniond::Identity()};
  std::chrono::steady_clock::time_point last_fastlio_health_wall_time_{};
  std::chrono::steady_clock::time_point automatic_recovery_start_wall_time_{};
  bool has_automatic_recovery_start_wall_time_{false};
  double latest_tf_wait_sec_{0.0};
  double latest_match_duration_ms_{0.0};
  std::mutex state_mutex_;
  std::mutex snapshot_queue_mutex_;
  std::condition_variable snapshot_queue_condition_;
  std::deque<std::function<void()>> snapshot_queue_;
  std::thread snapshot_worker_thread_;
  bool snapshot_worker_stopping_{false};
  small_gicp_relocalization::SnapshotRateLimiter snapshot_event_rate_limiter_;

  CovarianceCloud::Ptr map_cloud_;
  std::shared_ptr<CovarianceTree> map_tree_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr bbs_global_pose_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr fastlio_health_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr dynamic_tf_subscription_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr static_tf_subscription_;
  rclcpp::CallbackGroup::SharedPtr initial_pose_callback_group_;
  rclcpp::CallbackGroup::SharedPtr fastlio_health_callback_group_;
  rclcpp::CallbackGroup::SharedPtr heartbeat_callback_group_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr prior_map_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr prior_map_display_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr aligned_cloud_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr candidate_pose_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr
    preview_map_odom_publisher_;
  rclcpp::Publisher<small_gicp_relocalization::msg::RelocalizationStatus>::SharedPtr status_publisher_;
  rclcpp::Publisher<small_gicp_relocalization::msg::MapOdomTarget>::SharedPtr
    handoff_publisher_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr cancel_manual_seed_service_;
  tf2_msgs::msg::TFMessage static_tf_snapshot_;
  rclcpp::CallbackGroup::SharedPtr map_load_callback_group_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
  rclcpp::TimerBase::SharedPtr map_load_timer_;
  rclcpp::TimerBase::SharedPtr handoff_exit_timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<SmallGicpRelocalizationNode>();
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
    executor.add_node(node);
    executor.spin();
  } catch (const std::exception &exception) {
    RCLCPP_ERROR(rclcpp::get_logger("small_gicp_relocalization"), "%s", exception.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
