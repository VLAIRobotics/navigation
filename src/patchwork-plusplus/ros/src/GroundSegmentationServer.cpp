#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

#include <Eigen/Core>

// Patchwork++-ROS
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/exceptions.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#include "GroundSegmentationServer.hpp"
#include "GroundResidualFilter.hpp"

#include <cstdint>
#include <unordered_map>
#include "Utils.hpp"

namespace patchworkpp_ros {

using utils::EigenToPointCloud2;
using utils::GetTimestamps;
using utils::PointCloud2ToEigen;

// ---------------------------------------------------------------------------
// Parameter loaders
// ---------------------------------------------------------------------------

patchwork::Params GroundSegmentationServer::loadPlusplusParamsFromROS() {
  patchwork::Params params;

  params.sensor_height = declare_parameter<double>("sensor_height", params.sensor_height);
  sensor_height_ = params.sensor_height;
  params.num_iter      = declare_parameter<int>("num_iter", params.num_iter);
  params.num_lpr       = declare_parameter<int>("num_lpr", params.num_lpr);
  params.num_min_pts   = declare_parameter<int>("num_min_pts", params.num_min_pts);
  params.th_seeds      = declare_parameter<double>("th_seeds", params.th_seeds);

  params.th_dist    = declare_parameter<double>("th_dist", params.th_dist);
  params.th_seeds_v = declare_parameter<double>("th_seeds_v", params.th_seeds_v);
  params.th_dist_v  = declare_parameter<double>("th_dist_v", params.th_dist_v);

  params.max_range       = declare_parameter<double>("max_range", params.max_range);
  params.min_range       = declare_parameter<double>("min_range", params.min_range);
  ground_min_range_ = params.min_range;
  params.uprightness_thr = declare_parameter<double>("uprightness_thr", params.uprightness_thr);

  params.verbose = get_parameter<bool>("verbose", params.verbose);

  // ToDo. Support intensity
  params.enable_RNR = false;

  return params;
}

patchwork::PatchworkParams GroundSegmentationServer::loadClassicParamsFromROS() {
  patchwork::PatchworkParams params;

  params.sensor_height = declare_parameter<double>("sensor_height", params.sensor_height);
  sensor_height_ = params.sensor_height;
  params.max_range     = declare_parameter<double>("max_range", params.max_range);
  params.min_range     = declare_parameter<double>("min_range", params.min_range);
  ground_min_range_ = params.min_range;

  params.num_iter    = declare_parameter<int>("num_iter", params.num_iter);
  params.num_lpr     = declare_parameter<int>("num_lpr", params.num_lpr);
  params.num_min_pts = declare_parameter<int>("num_min_pts", params.num_min_pts);
  params.th_seeds    = declare_parameter<double>("th_seeds", params.th_seeds);
  params.th_dist     = declare_parameter<double>("th_dist", params.th_dist);

  params.uprightness_thr = declare_parameter<double>("uprightness_thr", params.uprightness_thr);

  params.adaptive_seed_selection_margin = declare_parameter<double>(
      "adaptive_seed_selection_margin", params.adaptive_seed_selection_margin);

  params.using_global_thr = declare_parameter<bool>("using_global_thr", params.using_global_thr);
  params.global_elevation_thr =
      declare_parameter<double>("global_elevation_thr", params.global_elevation_thr);

  params.ATAT_ON        = declare_parameter<bool>("ATAT_ON", params.ATAT_ON);
  params.max_h_for_ATAT = declare_parameter<double>("max_h_for_ATAT", params.max_h_for_ATAT);
  params.num_sectors_for_ATAT =
      declare_parameter<int>("num_sectors_for_ATAT", params.num_sectors_for_ATAT);
  params.noise_bound = declare_parameter<double>("noise_bound", params.noise_bound);

  params.verbose = declare_parameter<bool>("verbose", params.verbose);

  return params;
}

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------

GroundSegmentationServer::GroundSegmentationServer(const rclcpp::NodeOptions &options)
    : rclcpp::Node("patchworkpp_node", options),
      tf_buffer_(get_clock()),
      tf_listener_(tf_buffer_) {
  base_frame_ = declare_parameter<std::string>("base_frame", base_frame_);
  ground_visualization_range_ = declare_parameter<double>("ground_visualization_range", 5.0);
  ground_visualization_publish_stride_ =
      declare_parameter<int>("ground_visualization_publish_stride", 2);
  ground_visualization_voxel_size_ =
      declare_parameter<double>("ground_visualization_voxel_size", 0.08);
  ground_residual_xy_tolerance_ =
      declare_parameter<double>("ground_residual_xy_tolerance", 0.10);
  ground_residual_z_tolerance_ =
      declare_parameter<double>("ground_residual_z_tolerance", 0.03);
  if (!std::isfinite(ground_residual_xy_tolerance_) ||
    (ground_residual_xy_tolerance_ > 0.0 && ground_residual_xy_tolerance_ < 0.01) ||
    ground_residual_xy_tolerance_ > 1.0)
  {
    ground_residual_xy_tolerance_ = 0.10;
    RCLCPP_WARN(get_logger(), "Invalid ground_residual_xy_tolerance; using 0.10 m");
  }
  if (!std::isfinite(ground_residual_z_tolerance_) || ground_residual_z_tolerance_ < 0.0 ||
    ground_residual_z_tolerance_ > 1.0)
  {
    ground_residual_z_tolerance_ = 0.03;
    RCLCPP_WARN(get_logger(), "Invalid ground_residual_z_tolerance; using 0.03 m");
  }

  const std::string algorithm = declare_parameter<std::string>("algorithm", "patchworkpp");

  if (algorithm == "patchwork") {
    patchwork::PatchworkParams classic_params = loadClassicParamsFromROS();
    impl_ = std::make_unique<patchwork::PatchWork>(classic_params);
    RCLCPP_INFO(get_logger(), "Algorithm: patchwork (classic)");
  } else {
    patchwork::Params plusplus_params = loadPlusplusParamsFromROS();
    impl_                             = std::make_unique<patchwork::PatchWorkpp>(plusplus_params);
    RCLCPP_INFO(get_logger(), "Algorithm: patchworkpp (default)");
  }

  // Initialize subscribers
  pointcloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "pointcloud_topic",
      rclcpp::SensorDataQoS(),
      std::bind(&GroundSegmentationServer::EstimateGround, this, std::placeholders::_1));

  auto qos = rclcpp::SensorDataQoS().keep_last(1);

  cloud_publisher_  = create_publisher<sensor_msgs::msg::PointCloud2>("/patchworkpp/cloud", qos);
  ground_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("/patchworkpp/ground", qos);
  ground_window_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/patchworkpp/ground_window", qos);
  nonground_publisher_ =
      create_publisher<sensor_msgs::msg::PointCloud2>("/patchworkpp/nonground", qos);

  RCLCPP_INFO(this->get_logger(), "Patchwork++ ROS 2 node initialized");
}

// ---------------------------------------------------------------------------
// Callbacks
// ---------------------------------------------------------------------------

void GroundSegmentationServer::EstimateGround(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg) {
  if (msg->header.frame_id.empty()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                         "Dropping cloud with empty frame_id");
    return;
  }

  sensor_msgs::msg::PointCloud2 leveled_msg;
  try {
    const auto transform = tf_buffer_.lookupTransform(
        base_frame_, msg->header.frame_id, msg->header.stamp,
        tf2::durationFromSec(0.1));
    tf2::doTransform(*msg, leveled_msg, transform);
  } catch (const tf2::TransformException &ex) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                         "Dropping cloud: TF %s -> %s unavailable: %s",
                         msg->header.frame_id.c_str(), base_frame_.c_str(), ex.what());
    return;
  }
  auto leveled_cloud = std::make_shared<sensor_msgs::msg::PointCloud2>(std::move(leveled_msg));
  const auto &cloud = patchworkpp_ros::utils::PointCloud2ToEigenMat(leveled_cloud);
  std::size_t input_near_range_count = 0;
  if (std::isfinite(ground_min_range_) && ground_min_range_ < 0.50) {
    const double minimum_range_squared = ground_min_range_ * ground_min_range_;
    for (Eigen::Index index = 0; index < cloud.rows(); ++index) {
      const double x = cloud(index, 0);
      const double y = cloud(index, 1);
      const double range_squared = x * x + y * y;
      if (std::isfinite(range_squared) && range_squared >= minimum_range_squared &&
        range_squared < 0.25)
      {
        ++input_near_range_count;
      }
    }
  }

  // Estimate ground
  std::visit([&](auto &impl) { impl->estimateGround(cloud); }, impl_);
  if (cloud_publisher_->get_subscription_count() > 0) {
    cloud_publisher_->publish(
        patchworkpp_ros::utils::EigenMatToPointCloud2(cloud, leveled_cloud->header));
  }

  // Get ground and nonground
  Eigen::MatrixX3f ground    = std::visit([](auto &impl) { return impl->getGround(); }, impl_);
  Eigen::MatrixX3f nonground = std::visit([](auto &impl) { return impl->getNonground(); }, impl_);
  double time_taken          = std::visit([](auto &impl) { return impl->getTimeTaken(); }, impl_);
  RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Patchwork++ input=%ld ground=%ld nonground=%ld runtime=%.3f ms",
      static_cast<long>(cloud.rows()), static_cast<long>(ground.rows()),
      static_cast<long>(nonground.rows()), time_taken / 1000.0);
  PublishClouds(
    ground, nonground, leveled_cloud->header, msg->header.frame_id, input_near_range_count);
}

void GroundSegmentationServer::PublishClouds(const Eigen::MatrixX3f &est_ground,
                                             const Eigen::MatrixX3f &est_nonground,
                                             const std_msgs::msg::Header header_msg,
                                             const std::string &input_frame,
                                             const std::size_t input_near_range_count) {
  std_msgs::msg::Header header = header_msg;
  header.frame_id              = base_frame_;
  const auto residual_split = utils::splitGroundResiduals(
      est_ground, est_nonground,
      static_cast<float>(ground_residual_xy_tolerance_),
      static_cast<float>(ground_residual_z_tolerance_));
  const auto & terrain_ground = residual_split.ground;
  const auto & filtered_nonground = residual_split.nonground;
  const auto count_near_range = [this](const Eigen::MatrixX3f & points) {
      std::size_t count = 0;
      if (!std::isfinite(ground_min_range_) || ground_min_range_ >= 0.50) {
        return count;
      }
      const double minimum_range_squared = ground_min_range_ * ground_min_range_;
      for (Eigen::Index index = 0; index < points.rows(); ++index) {
        const double x = points(index, 0);
        const double y = points(index, 1);
        const double range_squared = x * x + y * y;
        if (std::isfinite(range_squared) && range_squared >= minimum_range_squared &&
          range_squared < 0.25)
        {
          ++count;
        }
      }
      return count;
    };
  const auto ground_near_range_count = count_near_range(terrain_ground);
  const auto nonground_near_range_count = count_near_range(filtered_nonground);
  const auto near_field_z_range = [this](const Eigen::MatrixX3f & points) {
      double min_z = std::numeric_limits<double>::infinity();
      double max_z = -std::numeric_limits<double>::infinity();
      const double minimum_range_squared = ground_min_range_ * ground_min_range_;
      for (Eigen::Index index = 0; index < points.rows(); ++index) {
        const double x = points(index, 0);
        const double y = points(index, 1);
        const double range_squared = x * x + y * y;
        if (std::isfinite(range_squared) && range_squared >= minimum_range_squared &&
          range_squared < 0.25 && std::isfinite(points(index, 2)))
        {
          min_z = std::min(min_z, static_cast<double>(points(index, 2)));
          max_z = std::max(max_z, static_cast<double>(points(index, 2)));
        }
      }
      if (!std::isfinite(min_z) || !std::isfinite(max_z)) {
        const auto nan = std::numeric_limits<double>::quiet_NaN();
        return std::pair<double, double>{nan, nan};
      }
      return std::pair<double, double>{min_z, max_z};
    };
  const auto ground_z_range = near_field_z_range(terrain_ground);
  const auto nonground_z_range = near_field_z_range(filtered_nonground);
  const auto source_stamp_ns =
    static_cast<long long>(header_msg.stamp.sec) * 1000000000LL +
    static_cast<long long>(header_msg.stamp.nanosec);
  RCLCPP_INFO_THROTTLE(
    get_logger(), *get_clock(), 5000,
    "Patchwork++ near-field source_stamp_ns=%lld input_frame='%s' base_frame='%s' "
    "sensor_height=%.3f [horizontal %.2f,0.50)m: input=%zu ground=%zu "
    "ground_z=[%.3f,%.3f] nonground=%zu nonground_z=[%.3f,%.3f]",
    source_stamp_ns, input_frame.c_str(), base_frame_.c_str(), sensor_height_,
    ground_min_range_, input_near_range_count, ground_near_range_count,
    ground_z_range.first, ground_z_range.second, nonground_near_range_count,
    nonground_z_range.first, nonground_z_range.second);
  if (residual_split.promoted_ground_points > 0) {
    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Patchwork++ source_stamp_ns=%lld retained %zu small supported surface returns as ground",
        source_stamp_ns, residual_split.promoted_ground_points);
  }
  if (ground_publisher_->get_subscription_count() > 0) {
    ground_publisher_->publish(
        std::move(patchworkpp_ros::utils::EigenMatToPointCloud2(terrain_ground, header)));
  }
  ++frame_count_;
  if (ground_window_publisher_->get_subscription_count() > 0 &&
      ground_visualization_publish_stride_ > 0 &&
      frame_count_ % static_cast<std::uint64_t>(ground_visualization_publish_stride_) == 0) {
    Eigen::MatrixX3f ground_window(terrain_ground.rows(), 3);
    Eigen::Index window_points = 0;
    const float range_squared = static_cast<float>(ground_visualization_range_ *
                                                    ground_visualization_range_);
    for (Eigen::Index i = 0; i < terrain_ground.rows(); ++i) {
      if (std::isfinite(terrain_ground(i, 0)) && std::isfinite(terrain_ground(i, 1)) &&
          terrain_ground(i, 0) * terrain_ground(i, 0) + terrain_ground(i, 1) * terrain_ground(i, 1) <=
              range_squared) {
        ground_window.row(window_points++) = terrain_ground.row(i);
      }
    }
    ground_window.conservativeResize(window_points, 3);
    if (ground_visualization_voxel_size_ > 0.0) {
      const float voxel = static_cast<float>(ground_visualization_voxel_size_);
      struct Voxel {
        Eigen::Vector3f sum{Eigen::Vector3f::Zero()};
        std::size_t count{0};
      };
      std::unordered_map<std::int64_t, Voxel> voxels;
      voxels.reserve(static_cast<std::size_t>(ground_window.rows()));
      for (Eigen::Index i = 0; i < ground_window.rows(); ++i) {
        const auto ix = static_cast<std::int64_t>(std::floor(ground_window(i, 0) / voxel));
        const auto iy = static_cast<std::int64_t>(std::floor(ground_window(i, 1) / voxel));
        const auto key = (static_cast<std::uint64_t>(static_cast<std::uint32_t>(ix)) << 32) |
                         static_cast<std::uint32_t>(iy);
        auto &cell = voxels[key];
        cell.sum += ground_window.row(i).transpose();
        ++cell.count;
      }
      Eigen::MatrixX3f filtered(static_cast<Eigen::Index>(voxels.size()), 3);
      Eigen::Index i = 0;
      for (const auto &entry : voxels) {
        filtered.row(i++) = entry.second.sum / static_cast<float>(entry.second.count);
      }
      ground_window = std::move(filtered);
    }
    ground_window_publisher_->publish(
        std::move(patchworkpp_ros::utils::EigenMatToPointCloud2(ground_window, header)));
  }
  if (nonground_publisher_->get_subscription_count() > 0) {
    nonground_publisher_->publish(
        std::move(patchworkpp_ros::utils::EigenMatToPointCloud2(filtered_nonground, header)));
  }
}
}  // namespace patchworkpp_ros

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(patchworkpp_ros::GroundSegmentationServer)
