#include "bringup/chassis_driver.hpp"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cinttypes>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include <net/if.h>
#include <rclcpp/rclcpp.hpp>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav2_msgs/msg/costmap.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <small_gicp_relocalization/msg/relocalization_status.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/LinearMath/Transform.hpp>
#include <tf2/LinearMath/Vector3.hpp>
#include <tf2/utils.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

namespace
{
std::int16_t decode_int16_le(const std::uint8_t low, const std::uint8_t high)
{
  const auto raw = static_cast<std::int32_t>(low) |
    (static_cast<std::int32_t>(high) << 8);
  const auto signed_value = raw >= 0x8000 ? raw - 0x10000 : raw;
  return static_cast<std::int16_t>(signed_value);
}

class ChassisDriverNode : public rclcpp::Node
{
public:
  ChassisDriverNode()
  : Node("chassis_driver"), tf_buffer_(get_clock())
  {
    declare_parameter("chassis_model", "two_wheel");
    chassis_model_ = get_parameter("chassis_model").as_string();
    if (chassis_model_ != "two_wheel" && chassis_model_ != "omni") {
      throw std::invalid_argument("chassis_model must be two_wheel or omni");
    }
    if (chassis_model_ == "omni") {
      throw std::runtime_error(
        "Refusing to open CAN for chassis_model=omni: the lateral-velocity CAN field, "
        "direction, and scale have not been verified; use enable_chassis:=false until verified");
    }
    declare_parameter("cmd_vel_topic", "/cmd_vel");
    declare_parameter("can_interface", "can3");
    declare_parameter("can_id", 0x050);
    declare_parameter("max_raw_speed", 800);
    declare_parameter("max_command_forward_speed", 0.30);
    declare_parameter("max_command_reverse_speed", 0.10);
    declare_parameter("max_command_angular_speed", 0.30);
    declare_parameter("max_linear_speed", 0.5);
    declare_parameter("max_angular_speed", 1.8);
    declare_parameter("send_rate", 200.0);
    declare_parameter("command_timeout", 0.5);
    declare_parameter("fastlio_health_loss_grace", 0.3);
    declare_parameter("require_perception", true);
    declare_parameter("perception_timeout", 0.5);
    declare_parameter("require_fastlio_health", true);
    declare_parameter("fastlio_health_topic", "/fastlio/health");
    declare_parameter("require_terrain", true);
    declare_parameter("require_relocalization", false);
    declare_parameter("relocalization_topic", "/small_gicp/relocalization_status");
    declare_parameter("relocalization_timeout", 1.0);
    declare_parameter("require_navigation_ready", false);
    declare_parameter("navigation_ready_topic", "/navigation_ready");
    declare_parameter("terrain_topic", "/fastdem/postprocess/gridmap");
    declare_parameter("terrain_frame", "odom");
    declare_parameter("terrain_max_age", 0.5);
    declare_parameter("terrain_max_future_stamp_offset", 0.1);
    declare_parameter("terrain_min_observations", 3);
    declare_parameter("terrain_max_observation_age_frames", 3);
    declare_parameter("terrain_min_confident_cells", 20);
    declare_parameter("terrain_max_lethal_fraction", 0.90);
    declare_parameter("terrain_slope_lethal_threshold", 30.0);
    declare_parameter("terrain_residual_step_lethal_threshold", 0.15);
    declare_parameter("terrain_roughness_lethal_threshold", 0.15);
    declare_parameter("require_motion_costmap", false);
    declare_parameter("motion_costmap_topic", "/local_costmap/costmap_raw");
    declare_parameter("require_reverse_costmap", false);
    declare_parameter("reverse_costmap_topic", "/local_costmap/costmap_raw");
    declare_parameter("reverse_blocked_topic", "/chassis/reverse_blocked");
    declare_parameter("safety_status_topic", "/chassis/safety_status");
    declare_parameter("reverse_costmap_timeout", 0.75);
    declare_parameter("reverse_odometry_timeout", 0.25);
    declare_parameter("reverse_base_frame", "base_link");
    declare_parameter("reverse_projection_time", 0.5);
    declare_parameter("reverse_linear_deceleration", 0.5);
    declare_parameter("reverse_angular_deceleration", 0.6);
    declare_parameter("reverse_safety_margin", 0.10);
    declare_parameter("reverse_motion_threshold", 0.05);
    declare_parameter("reverse_confirmation_samples", 3);
    declare_parameter("reverse_latch_reset_cycles", 20);
    declare_parameter("reverse_unknown_is_blocked", false);
    declare_parameter("reverse_block_cost", 253);
    declare_parameter("reverse_footprint", std::vector<double>{
        0.275, 0.225, 0.275, -0.225, -0.35, -0.225, -0.35, 0.225});
    declare_parameter("motion_footprint", std::vector<double>{
        0.275, 0.225, 0.275, -0.225, -0.35, -0.225, -0.35, 0.225});

    bringup::ChassisConfig config;
    config.can_id = get_parameter("can_id").as_int();
    config.max_raw_speed = get_parameter("max_raw_speed").as_int();
    config.max_command_forward_speed =
      get_parameter("max_command_forward_speed").as_double();
    config.max_command_reverse_speed =
      get_parameter("max_command_reverse_speed").as_double();
    config.max_command_angular_speed =
      get_parameter("max_command_angular_speed").as_double();
    config.max_linear_speed = get_parameter("max_linear_speed").as_double();
    config.max_angular_speed = get_parameter("max_angular_speed").as_double();
    config.send_rate = get_parameter("send_rate").as_double();
    config.command_timeout = get_parameter("command_timeout").as_double();
    config.fastlio_health_loss_grace =
      get_parameter("fastlio_health_loss_grace").as_double();
    bringup::validate_config(config);
    reverse_max_linear_speed_ = config.max_linear_speed;
    reverse_max_angular_speed_ = config.max_angular_speed;
    fastlio_health_loss_grace_sec_ = config.fastlio_health_loss_grace;
    require_perception_ = get_parameter("require_perception").as_bool();
    perception_timeout_ = get_parameter("perception_timeout").as_double();
    require_fastlio_health_ = get_parameter("require_fastlio_health").as_bool();
    require_terrain_ = get_parameter("require_terrain").as_bool();
    require_relocalization_ = get_parameter("require_relocalization").as_bool();
    relocalization_timeout_ = get_parameter("relocalization_timeout").as_double();
    require_navigation_ready_ = get_parameter("require_navigation_ready").as_bool();
    require_reverse_costmap_ = get_parameter("require_motion_costmap").as_bool() ||
      get_parameter("require_reverse_costmap").as_bool();
    reverse_costmap_timeout_ = get_parameter("reverse_costmap_timeout").as_double();
    reverse_odometry_timeout_ = get_parameter("reverse_odometry_timeout").as_double();
    reverse_base_frame_ = get_parameter("reverse_base_frame").as_string();
    reverse_projection_time_ = get_parameter("reverse_projection_time").as_double();
    reverse_linear_deceleration_ = get_parameter("reverse_linear_deceleration").as_double();
    reverse_angular_deceleration_ = get_parameter("reverse_angular_deceleration").as_double();
    reverse_safety_margin_ = get_parameter("reverse_safety_margin").as_double();
    reverse_motion_threshold_ = get_parameter("reverse_motion_threshold").as_double();
    reverse_confirmation_samples_ = get_parameter("reverse_confirmation_samples").as_int();
    reverse_latch_reset_cycles_ = get_parameter("reverse_latch_reset_cycles").as_int();
    reverse_unknown_is_blocked_ = get_parameter("reverse_unknown_is_blocked").as_bool();
    reverse_block_cost_ = get_parameter("reverse_block_cost").as_int();
    auto reverse_footprint = get_parameter("motion_footprint").as_double_array();
    if (get_parameter("reverse_footprint").as_double_array() !=
      std::vector<double>{0.275, 0.225, 0.275, -0.225, -0.35, -0.225, -0.35, 0.225})
    {
      // Preserve explicit settings from older launch files while the parameter
      // name moves from the reverse-only gate to the shared motion gate.
      reverse_footprint = get_parameter("reverse_footprint").as_double_array();
    }
    if (reverse_footprint.size() < 6 || reverse_footprint.size() % 2 != 0) {
      throw std::invalid_argument("reverse_footprint must contain at least three x/y vertices");
    }
    for (std::size_t i = 0; i < reverse_footprint.size(); i += 2) {
      if (!std::isfinite(reverse_footprint[i]) || !std::isfinite(reverse_footprint[i + 1])) {
        throw std::invalid_argument("reverse_footprint vertices must be finite");
      }
      reverse_footprint_.emplace_back(reverse_footprint[i], reverse_footprint[i + 1]);
    }
    const auto terrain_topic = get_parameter("terrain_topic").as_string();
    terrain_config_.frame_id = get_parameter("terrain_frame").as_string();
    terrain_config_.max_age = get_parameter("terrain_max_age").as_double();
    terrain_config_.max_future_stamp_offset =
      get_parameter("terrain_max_future_stamp_offset").as_double();
    terrain_config_.min_observations = get_parameter("terrain_min_observations").as_int();
    terrain_config_.max_observation_age_frames =
      get_parameter("terrain_max_observation_age_frames").as_int();
    terrain_config_.min_confident_cells =
      get_parameter("terrain_min_confident_cells").as_int();
    terrain_config_.max_lethal_fraction =
      get_parameter("terrain_max_lethal_fraction").as_double();
    terrain_config_.slope_lethal =
      get_parameter("terrain_slope_lethal_threshold").as_double();
    terrain_config_.residual_step_lethal =
      get_parameter("terrain_residual_step_lethal_threshold").as_double();
    terrain_config_.roughness_lethal =
      get_parameter("terrain_roughness_lethal_threshold").as_double();
    if (!std::isfinite(perception_timeout_) || perception_timeout_ <= 0.0) {
      throw std::invalid_argument("perception_timeout must be finite and greater than zero");
    }
    if (!std::isfinite(relocalization_timeout_) || relocalization_timeout_ <= 0.0) {
      throw std::invalid_argument("relocalization_timeout must be finite and greater than zero");
    }
    if (!std::isfinite(terrain_config_.max_age) || terrain_config_.max_age <= 0.0) {
      throw std::invalid_argument("terrain_max_age must be finite and greater than zero");
    }
    if (!std::isfinite(reverse_costmap_timeout_) || reverse_costmap_timeout_ <= 0.0 ||
      !std::isfinite(reverse_odometry_timeout_) || reverse_odometry_timeout_ <= 0.0 ||
      reverse_base_frame_.empty() || !std::isfinite(reverse_projection_time_) ||
      reverse_projection_time_ <= 0.0 || !std::isfinite(reverse_linear_deceleration_) ||
      reverse_linear_deceleration_ <= 0.0 || !std::isfinite(reverse_angular_deceleration_) ||
      reverse_angular_deceleration_ <= 0.0 || !std::isfinite(reverse_safety_margin_) ||
      reverse_safety_margin_ < 0.0 || !std::isfinite(reverse_motion_threshold_) ||
      reverse_motion_threshold_ <= 0.0 || reverse_confirmation_samples_ < 2 ||
      reverse_latch_reset_cycles_ < 1 || reverse_block_cost_ < 1 || reverse_block_cost_ > 254)
    {
      throw std::invalid_argument("reverse costmap safety parameters are invalid");
    }
    if (require_reverse_costmap_) {
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(tf_buffer_, this);
    }

    interface_ = get_parameter("can_interface").as_string();
    open_can(interface_);
    state_ = std::make_unique<bringup::CommandState>(config);

    const auto reverse_blocked_topic = get_parameter("reverse_blocked_topic").as_string();
    const auto safety_status_topic = get_parameter("safety_status_topic").as_string();
    if (reverse_blocked_topic.empty() || safety_status_topic.empty()) {
      throw std::invalid_argument("chassis safety status topics must not be empty");
    }
    reverse_blocked_publisher_ = create_publisher<std_msgs::msg::Bool>(
      reverse_blocked_topic, rclcpp::QoS(1).reliable().transient_local());
    safety_status_publisher_ = create_publisher<std_msgs::msg::String>(
      safety_status_topic, rclcpp::QoS(1).reliable().transient_local());
    std_msgs::msg::String initial_safety_status;
    initial_safety_status.data = "waiting";
    safety_status_publisher_->publish(initial_safety_status);
    std_msgs::msg::Bool initial_reverse_status;
    initial_reverse_status.data = false;
    reverse_blocked_publisher_->publish(initial_reverse_status);
    have_reverse_blocked_publish_ = true;
    last_reverse_blocked_publish_ = std::chrono::steady_clock::now();

    const auto topic = get_parameter("cmd_vel_topic").as_string();
    subscription_ = create_subscription<geometry_msgs::msg::Twist>(
      topic, 10, [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!state_->update(*msg, std::chrono::steady_clock::now())) {
          RCLCPP_ERROR(
            get_logger(),
            "Rejected invalid or unsupported /cmd_vel component (including linear.y); "
            "stopping chassis and latching the safety fault");
        }
      });
    odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      "/Odometry", rclcpp::SensorDataQoS(), [this](nav_msgs::msg::Odometry::SharedPtr message) {
        std::lock_guard<std::mutex> lock(mutex_);
        last_odometry_ = std::chrono::steady_clock::now();
        reverse_odometry_received_at_ = last_odometry_;
        have_odometry_ = true;
        reverse_odometry_ = std::move(message);
      });
    const auto fastlio_health_topic = get_parameter("fastlio_health_topic").as_string();
    fastlio_health_subscription_ = create_subscription<std_msgs::msg::Bool>(
      fastlio_health_topic, rclcpp::QoS(1),
      [this](std_msgs::msg::Bool::SharedPtr message) {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto received_at = std::chrono::steady_clock::now();
        have_fastlio_health_ = true;
        fastlio_healthy_ = message->data;
        if (fastlio_healthy_) {
          have_ever_fastlio_healthy_ = true;
          fastlio_health_loss_pending_ = false;
          state_->set_health_loss_hold(false);
        } else if (have_ever_fastlio_healthy_ && !fastlio_health_loss_pending_) {
          fastlio_health_loss_pending_ = true;
          fastlio_health_loss_since_ = received_at;
          if (require_fastlio_health_) {
            state_->set_health_loss_hold(true);
          }
        }
        last_fastlio_health_ = received_at;
      });
    obstacle_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/patchworkpp/nonground", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::PointCloud2::SharedPtr) {
        std::lock_guard<std::mutex> lock(mutex_);
        last_obstacle_ = std::chrono::steady_clock::now();
        have_obstacle_ = true;
      });
    if (require_reverse_costmap_) {
      auto reverse_costmap_topic = get_parameter("motion_costmap_topic").as_string();
      const auto legacy_reverse_costmap_topic =
        get_parameter("reverse_costmap_topic").as_string();
      if (reverse_costmap_topic == "/local_costmap/costmap_raw" &&
        legacy_reverse_costmap_topic != "/local_costmap/costmap_raw")
      {
        reverse_costmap_topic = legacy_reverse_costmap_topic;
      }
      reverse_costmap_subscription_ = create_subscription<nav2_msgs::msg::Costmap>(
        reverse_costmap_topic, rclcpp::QoS(1).reliable().transient_local(),
        [this](nav2_msgs::msg::Costmap::SharedPtr message) {
          std::lock_guard<std::mutex> lock(mutex_);
          reverse_costmap_ = std::move(message);
          reverse_costmap_received_at_ = std::chrono::steady_clock::now();
        });
    }
    const auto relocalization_topic = get_parameter("relocalization_topic").as_string();
    relocalization_subscription_ = create_subscription<small_gicp_relocalization::msg::RelocalizationStatus>(
      relocalization_topic, rclcpp::QoS(10),
      [this](small_gicp_relocalization::msg::RelocalizationStatus::SharedPtr message) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (message->valid && message->state ==
          small_gicp_relocalization::msg::RelocalizationStatus::LOCALIZED)
        {
          last_relocalization_ = std::chrono::steady_clock::now();
          have_relocalization_ = true;
        } else {
          have_relocalization_ = false;
        }
      });
    const auto navigation_ready_topic = get_parameter("navigation_ready_topic").as_string();
    navigation_ready_subscription_ = create_subscription<std_msgs::msg::Bool>(
      navigation_ready_topic, rclcpp::QoS(1).transient_local(),
      [this](std_msgs::msg::Bool::SharedPtr message) {
        std::lock_guard<std::mutex> lock(mutex_);
        have_navigation_ready_ = true;
        navigation_ready_ = message->data;
      });
    terrain_subscription_ = create_subscription<grid_map_msgs::msg::GridMap>(
      terrain_topic, rclcpp::QoS(1),
      [this](grid_map_msgs::msg::GridMap::SharedPtr msg) {
        const auto now_ros_ns = get_clock()->now().nanoseconds();
        const auto reject_terrain = [this]() {
            std::lock_guard<std::mutex> lock(mutex_);
            have_terrain_ = false;
          };
        if (!bringup::terrain_message_is_current(*msg, now_ros_ns, terrain_config_)) {
          reject_terrain();
          RCLCPP_ERROR_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Invalid or non-current FastDEM terrain input; stopping chassis");
          return;
        }

        const auto stamp_ns = static_cast<int64_t>(msg->header.stamp.sec) * 1000000000LL +
          static_cast<int64_t>(msg->header.stamp.nanosec);
        std::lock_guard<std::mutex> lock(mutex_);
        observeRosClockLocked(get_clock()->now().nanoseconds());
        if (have_terrain_stamp_) {
          if (stamp_ns < last_terrain_stamp_ns_) {
            have_terrain_ = false;
            return;
          }
          if (stamp_ns == last_terrain_stamp_ns_) {
            return;
          }
        }
        last_terrain_stamp_ns_ = stamp_ns;
        have_terrain_stamp_ = true;
        last_terrain_ = std::chrono::steady_clock::now();
        have_terrain_ = true;
      });
    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / config.send_rate));
    timer_ = create_wall_timer(period, [this]() { send_current(); });
    RCLCPP_INFO(
      get_logger(), "Chassis model=%s footprint_vertices=%zu CAN=%s; swept-footprint "
      "safety checks all commanded/measured planar motion",
      chassis_model_.c_str(), reverse_footprint_.size(), interface_.c_str());
  }

  ~ChassisDriverNode() override
  {
    const auto stop = bringup::build_frame(
      get_parameter("can_id").as_int(), 0, 0);
    for (int i = 0; i < 5; ++i) {
      try {
        write_frame(stop);
      } catch (const std::exception & error) {
        RCLCPP_ERROR(get_logger(), "CAN stop send failed on %s: %s", interface_.c_str(), error.what());
        break;
      }
    }
    if (socket_fd_ >= 0) {
      close(socket_fd_);
    }
  }

private:
  void publishReverseBlockedStatusLocked(
    const bool blocked, const std::chrono::steady_clock::time_point now)
  {
    const bool heartbeat_due = blocked && (!have_reverse_blocked_publish_ ||
      now - last_reverse_blocked_publish_ >= std::chrono::milliseconds(100));
    if (!reverse_blocked_publisher_ ||
      (blocked == reverse_blocked_status_ && !heartbeat_due))
    {
      return;
    }

    std_msgs::msg::Bool message;
    message.data = blocked;
    reverse_blocked_publisher_->publish(message);
    reverse_blocked_status_ = blocked;
    last_reverse_blocked_publish_ = now;
    have_reverse_blocked_publish_ = true;
  }

  void publishSafetyStatusLocked(
    const std::string & status, const std::chrono::steady_clock::time_point now)
  {
    const bool heartbeat_due = !have_safety_status_publish_ ||
      now - last_safety_status_publish_ >= std::chrono::seconds(1);
    if (!safety_status_publisher_ ||
      (status == last_safety_status_ && !heartbeat_due))
    {
      return;
    }
    std_msgs::msg::String message;
    message.data = status;
    safety_status_publisher_->publish(message);
    last_safety_status_ = status;
    last_safety_status_publish_ = now;
    have_safety_status_publish_ = true;
  }

  bool measuredVelocityInBaseLocked(
    const int64_t now_ros_ns, const std::chrono::steady_clock::time_point now,
    tf2::Vector3 & linear_base, tf2::Vector3 & angular_base, std::string & reason) const
  {
    if (!reverse_odometry_) {
      reason = "no odometry sample for reverse motion";
      return false;
    }
    const auto & odometry = *reverse_odometry_;
    const auto stamp = odometry.header.stamp;
    if (odometry.header.frame_id.empty() || odometry.child_frame_id.empty() ||
      stamp.sec < 0 || stamp.nanosec >= 1000000000U)
    {
      reason = "reverse odometry frame or timestamp is invalid";
      return false;
    }
    const int64_t stamp_ns = static_cast<int64_t>(stamp.sec) * 1000000000LL + stamp.nanosec;
    const auto max_age_ns = static_cast<int64_t>(std::llround(reverse_odometry_timeout_ * 1e9));
    const int64_t ros_age_ns = now_ros_ns - stamp_ns;
    if (now_ros_ns <= 0 || stamp_ns <= 0 || ros_age_ns < 0 || ros_age_ns > max_age_ns ||
      now - reverse_odometry_received_at_ >
      std::chrono::duration<double>(reverse_odometry_timeout_))
    {
      reason = "reverse odometry sample is stale";
      return false;
    }

    const auto & measured = odometry.twist.twist;
    if (!std::isfinite(measured.linear.x) || !std::isfinite(measured.linear.y) ||
      !std::isfinite(measured.linear.z) || !std::isfinite(measured.angular.x) ||
      !std::isfinite(measured.angular.y) || !std::isfinite(measured.angular.z))
    {
      reason = "reverse odometry velocity is invalid";
      return false;
    }

    tf2::Transform base_from_child;
    base_from_child.setIdentity();
    if (odometry.child_frame_id != reverse_base_frame_) {
      try {
        const auto transform = tf_buffer_.lookupTransform(
          reverse_base_frame_, odometry.child_frame_id, tf2::TimePointZero);
        const auto & translation = transform.transform.translation;
        const auto & rotation = transform.transform.rotation;
        const double norm_squared = rotation.x * rotation.x + rotation.y * rotation.y +
          rotation.z * rotation.z + rotation.w * rotation.w;
        if (!std::isfinite(translation.x) || !std::isfinite(translation.y) ||
          !std::isfinite(translation.z) || !std::isfinite(norm_squared) ||
          std::abs(norm_squared - 1.0) > 1e-3)
        {
          reason = "reverse odometry transform is invalid";
          return false;
        }
        tf2::fromMsg(transform.transform, base_from_child);
      } catch (const tf2::TransformException &) {
        reason = "transform from odometry child frame to chassis frame is unavailable";
        return false;
      }
    }

    const auto rotation = base_from_child.getBasis();
    const tf2::Vector3 angular_child(
      measured.angular.x, measured.angular.y, measured.angular.z);
    angular_base = rotation * angular_child;
    const tf2::Vector3 linear_child(measured.linear.x, measured.linear.y, measured.linear.z);
    const tf2::Vector3 linear_base_at_child = rotation * linear_child;
    linear_base =
      linear_base_at_child - angular_base.cross(base_from_child.getOrigin());
    if (!std::isfinite(linear_base.x()) || !std::isfinite(linear_base.y()) ||
      !std::isfinite(linear_base.z()) || !std::isfinite(angular_base.x()) ||
      !std::isfinite(angular_base.y()) || !std::isfinite(angular_base.z()))
    {
      reason = "reverse odometry velocity is invalid";
      return false;
    }
    return true;
  }

  bool motionVelocityEnvelopeLocked(
    const geometry_msgs::msg::Twist & command, const int64_t now_ros_ns,
    const std::chrono::steady_clock::time_point now,
    std::vector<std::pair<double, double>> & linear_velocities,
    std::vector<double> & angular_velocities, bool & reverse_motion,
    std::string & reason) const
  {
    tf2::Vector3 linear_base;
    tf2::Vector3 angular_base;
    if (!measuredVelocityInBaseLocked(now_ros_ns, now, linear_base, angular_base, reason)) {
      return false;
    }
    const double measured_linear_speed = std::hypot(linear_base.x(), linear_base.y());
    const double measured_angular_speed = std::abs(angular_base.z());
    if (!std::isfinite(measured_linear_speed) || !std::isfinite(measured_angular_speed) ||
      measured_linear_speed > reverse_max_linear_speed_ ||
      measured_angular_speed > reverse_max_angular_speed_)
    {
      reason = "measured odometry velocity exceeds the configured chassis limits";
      return false;
    }

    reverse_motion = linear_base.x() < -reverse_motion_threshold_ ||
      command.linear.x < -reverse_motion_threshold_;
    linear_velocities.clear();
    const double command_linear_speed = std::hypot(command.linear.x, command.linear.y);
    if (command_linear_speed > reverse_motion_threshold_) {
      linear_velocities.emplace_back(command.linear.x, command.linear.y);
    }
    if (measured_linear_speed > reverse_motion_threshold_) {
      linear_velocities.emplace_back(linear_base.x(), linear_base.y());
    }
    if (linear_velocities.empty()) {
      linear_velocities.emplace_back(0.0, 0.0);
    }
    angular_velocities.clear();
    if (std::abs(command.angular.z) > reverse_motion_threshold_) {
      angular_velocities.push_back(command.angular.z);
    }
    if (measured_angular_speed > reverse_motion_threshold_) {
      angular_velocities.push_back(angular_base.z());
    }
    if (angular_velocities.empty()) {
      angular_velocities.push_back(0.0);
    }
    return true;
  }

  bool motionCommandIsSafeLocked(
    const geometry_msgs::msg::Twist & command, const int64_t now_ros_ns,
    const std::chrono::steady_clock::time_point now, bool & reverse_motion,
    std::string & reason) const
  {
    if (!reverse_costmap_) {
      reason = "no local costmap snapshot";
      return false;
    }
    const auto & costmap = *reverse_costmap_;
    const auto & metadata = costmap.metadata;
    const auto max_age_ns = static_cast<int64_t>(std::llround(reverse_costmap_timeout_ * 1e9));
    const auto stamp = rclcpp::Time(costmap.header.stamp, get_clock()->get_clock_type());
    const auto ros_age_ns = now_ros_ns - stamp.nanoseconds();
    if (now_ros_ns <= 0 || ros_age_ns < 0 || ros_age_ns > max_age_ns ||
      now - reverse_costmap_received_at_ > std::chrono::duration<double>(reverse_costmap_timeout_))
    {
      reason = "local costmap snapshot is stale";
      return false;
    }
    if (costmap.header.frame_id.empty() || metadata.layer != "master" ||
      metadata.size_x == 0 || metadata.size_y == 0 ||
      !std::isfinite(metadata.resolution) || metadata.resolution <= 0.0F ||
      costmap.data.size() != static_cast<std::size_t>(metadata.size_x) * metadata.size_y)
    {
      reason = "local costmap snapshot is invalid";
      return false;
    }
    const auto & origin = metadata.origin;
    const double quaternion_norm_squared = origin.orientation.x * origin.orientation.x +
      origin.orientation.y * origin.orientation.y + origin.orientation.z * origin.orientation.z +
      origin.orientation.w * origin.orientation.w;
    if (!std::isfinite(origin.position.x) || !std::isfinite(origin.position.y) ||
      !std::isfinite(origin.position.z) ||
      !std::isfinite(quaternion_norm_squared) ||
      std::abs(quaternion_norm_squared - 1.0) > 1e-3)
    {
      reason = "local costmap origin is invalid";
      return false;
    }

    geometry_msgs::msg::TransformStamped robot_transform;
    try {
      robot_transform = tf_buffer_.lookupTransform(
        costmap.header.frame_id, reverse_base_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException &) {
      reason = "robot transform into local costmap is unavailable";
      return false;
    }
    const auto transform_stamp = rclcpp::Time(
      robot_transform.header.stamp, get_clock()->get_clock_type());
    const auto transform_age_ns = now_ros_ns - transform_stamp.nanoseconds();
    if (transform_age_ns < 0 || transform_age_ns > max_age_ns) {
      reason = "robot transform is stale";
      return false;
    }
    const auto & transform = robot_transform.transform;
    const double transform_quaternion_norm_squared =
      transform.rotation.x * transform.rotation.x +
      transform.rotation.y * transform.rotation.y +
      transform.rotation.z * transform.rotation.z +
      transform.rotation.w * transform.rotation.w;
    if (!std::isfinite(transform.translation.x) || !std::isfinite(transform.translation.y) ||
      !std::isfinite(transform.translation.z) ||
      !std::isfinite(transform_quaternion_norm_squared) ||
      std::abs(transform_quaternion_norm_squared - 1.0) > 1e-3)
    {
      reason = "robot transform is invalid";
      return false;
    }

    tf2::Transform global_from_grid;
    tf2::Transform global_from_robot;
    tf2::fromMsg(origin, global_from_grid);
    tf2::fromMsg(robot_transform.transform, global_from_robot);
    const auto grid_from_global = global_from_grid.inverse();
    const double resolution = metadata.resolution;
    std::vector<std::pair<double, double>> projection_linear_velocities;
    std::vector<double> projection_angular_velocities;
    if (!motionVelocityEnvelopeLocked(
        command, now_ros_ns, now, projection_linear_velocities,
        projection_angular_velocities, reverse_motion, reason))
    {
      return false;
    }

    const auto point_in_or_near_footprint = [this](const double x, const double y) {
        bool inside = false;
        double min_distance_squared = std::numeric_limits<double>::infinity();
        for (std::size_t i = 0, j = reverse_footprint_.size() - 1;
          i < reverse_footprint_.size(); j = i++)
        {
          const auto [ax, ay] = reverse_footprint_[j];
          const auto [bx, by] = reverse_footprint_[i];
          const bool crosses = ((ay > y) != (by > y)) &&
            (x < (bx - ax) * (y - ay) / (by - ay) + ax);
          if (crosses) {
            inside = !inside;
          }
          const double edge_x = bx - ax;
          const double edge_y = by - ay;
          const double edge_length_squared = edge_x * edge_x + edge_y * edge_y;
          const double projection = edge_length_squared > 0.0
            ? std::clamp(((x - ax) * edge_x + (y - ay) * edge_y) / edge_length_squared,
              0.0, 1.0)
            : 0.0;
          const double dx = x - (ax + projection * edge_x);
          const double dy = y - (ay + projection * edge_y);
          min_distance_squared = std::min(min_distance_squared, dx * dx + dy * dy);
        }
        return inside || min_distance_squared <= reverse_safety_margin_ * reverse_safety_margin_;
      };

    const auto footprint_is_clear = [&](const tf2::Transform & pose) {
        double min_x = std::numeric_limits<double>::infinity();
        double min_y = std::numeric_limits<double>::infinity();
        double max_x = -std::numeric_limits<double>::infinity();
        double max_y = -std::numeric_limits<double>::infinity();
        for (const auto & vertex : reverse_footprint_) {
          const auto point = grid_from_global * pose * tf2::Vector3(vertex.first, vertex.second, 0.0);
          min_x = std::min(min_x, point.x());
          min_y = std::min(min_y, point.y());
          max_x = std::max(max_x, point.x());
          max_y = std::max(max_y, point.y());
        }
        const auto min_cell_x = static_cast<int>(std::floor(
            (min_x - reverse_safety_margin_) / resolution));
        const auto min_cell_y = static_cast<int>(std::floor(
            (min_y - reverse_safety_margin_) / resolution));
        const auto max_cell_x = static_cast<int>(std::floor(
            (max_x + reverse_safety_margin_) / resolution));
        const auto max_cell_y = static_cast<int>(std::floor(
            (max_y + reverse_safety_margin_) / resolution));
        if (min_cell_x < 0 || min_cell_y < 0 ||
          max_cell_x >= static_cast<int>(metadata.size_x) ||
          max_cell_y >= static_cast<int>(metadata.size_y))
        {
          return false;
        }

        const auto robot_from_global = pose.inverse();
        for (int cell_y = min_cell_y; cell_y <= max_cell_y; ++cell_y) {
          for (int cell_x = min_cell_x; cell_x <= max_cell_x; ++cell_x) {
            const auto index = static_cast<std::size_t>(cell_y) * metadata.size_x +
              static_cast<std::size_t>(cell_x);
            const auto cost = costmap.data[index];
            if (!bringup::reverse_cost_is_blocked(
                cost, static_cast<int>(reverse_block_cost_), reverse_unknown_is_blocked_))
            {
              continue;
            }
            const auto point = robot_from_global * global_from_grid * tf2::Vector3(
              (static_cast<double>(cell_x) + 0.5) * resolution,
              (static_cast<double>(cell_y) + 0.5) * resolution, 0.0);
            if (point_in_or_near_footprint(point.x(), point.y())) {
              return false;
            }
          }
        }
        return true;
      };

    const auto trajectory_is_clear = [&](
      const double start_linear_x, const double start_linear_y,
      const double start_angular_velocity)
      {
        const double linear_stop_time =
          std::hypot(start_linear_x, start_linear_y) / reverse_linear_deceleration_;
        const double angular_stop_time =
          std::abs(start_angular_velocity) / reverse_angular_deceleration_;
        const double total_time = reverse_projection_time_ +
          std::max(linear_stop_time, angular_stop_time);
        const double time_step = 0.05;
        tf2::Transform pose = global_from_robot;
        double elapsed = 0.0;
        while (true) {
          if (!footprint_is_clear(pose)) {
            return false;
          }
          if (elapsed >= total_time) {
            return true;
          }
          const double step = std::min(time_step, total_time - elapsed);
          const double midpoint = elapsed + step * 0.5;
          const auto velocity_scale = [this, midpoint](const double stop_time) {
              if (midpoint <= reverse_projection_time_ || stop_time <= 0.0) {
                return 1.0;
              }
              return std::clamp(
                1.0 - (midpoint - reverse_projection_time_) / stop_time, 0.0, 1.0);
            };
          const double linear_velocity_x = start_linear_x * velocity_scale(linear_stop_time);
          const double linear_velocity_y = start_linear_y * velocity_scale(linear_stop_time);
          const double angular_velocity =
            start_angular_velocity * velocity_scale(angular_stop_time);
          const double yaw = tf2::getYaw(pose.getRotation());
          double next_x = pose.getOrigin().x();
          double next_y = pose.getOrigin().y();
          double next_yaw = yaw;
          if (std::abs(angular_velocity) < 1e-6) {
            next_x += (linear_velocity_x * std::cos(yaw) -
              linear_velocity_y * std::sin(yaw)) * step;
            next_y += (linear_velocity_x * std::sin(yaw) +
              linear_velocity_y * std::cos(yaw)) * step;
          } else {
            next_yaw += angular_velocity * step;
            next_x += linear_velocity_x / angular_velocity *
              (std::sin(next_yaw) - std::sin(yaw)) +
              linear_velocity_y / angular_velocity * (std::cos(next_yaw) - std::cos(yaw));
            next_y += -linear_velocity_x / angular_velocity *
              (std::cos(next_yaw) - std::cos(yaw)) +
              linear_velocity_y / angular_velocity * (std::sin(next_yaw) - std::sin(yaw));
          }
          tf2::Quaternion rotation;
          rotation.setRPY(0.0, 0.0, next_yaw);
          pose.setOrigin(tf2::Vector3(next_x, next_y, pose.getOrigin().z()));
          pose.setRotation(rotation);
          elapsed += step;
        }
      };
    for (const auto & linear_velocity : projection_linear_velocities) {
      for (const auto angular_velocity : projection_angular_velocities) {
        if (!trajectory_is_clear(
            linear_velocity.first, linear_velocity.second, angular_velocity))
        {
          reason = "swept footprint intersects occupied or unknown costmap cells";
          return false;
        }
      }
    }
    return true;
  }

  void observeRosClockLocked(const int64_t now_ns)
  {
    if (now_ns <= 0) {
      return;
    }
    if (last_ros_clock_ns_ > 0 && now_ns < last_ros_clock_ns_) {
      have_terrain_ = false;
      have_terrain_stamp_ = false;
      last_terrain_stamp_ns_ = 0;
      RCLCPP_WARN(get_logger(), "ROS clock moved backwards; resetting terrain timestamp gate");
    }
    last_ros_clock_ns_ = now_ns;
  }

  void open_can(const std::string & interface)
  {
    socket_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socket_fd_ < 0) {
      throw std::runtime_error("CAN open failed: " + std::string(std::strerror(errno)));
    }
    ifreq request{};
    std::strncpy(request.ifr_name, interface.c_str(), IFNAMSIZ - 1);
    if (ioctl(socket_fd_, SIOCGIFINDEX, &request) < 0) {
      const auto error = std::string(std::strerror(errno));
      close(socket_fd_);
      socket_fd_ = -1;
      throw std::runtime_error("CAN interface lookup failed for " + interface + ": " + error);
    }
    sockaddr_can address{};
    address.can_family = AF_CAN;
    address.can_ifindex = request.ifr_ifindex;
    if (bind(socket_fd_, reinterpret_cast<sockaddr *>(&address), sizeof(address)) < 0) {
      const auto error = std::string(std::strerror(errno));
      close(socket_fd_);
      socket_fd_ = -1;
      throw std::runtime_error("CAN bind failed for " + interface + ": " + error);
    }
  }

  void write_frame(const can_frame & frame)
  {
    const auto result = write(socket_fd_, &frame, sizeof(frame));
    if (result != static_cast<ssize_t>(sizeof(frame))) {
      throw std::runtime_error("CAN write failed: " + std::string(std::strerror(errno)));
    }
  }

  void record_successful_frame(const can_frame & frame)
  {
    const auto now = std::chrono::steady_clock::now();
    if (can_log_window_start_ == std::chrono::steady_clock::time_point{}) {
      can_log_window_start_ = now;
    }

    const auto vy = decode_int16_le(frame.data[2], frame.data[3]);
    const auto vw = decode_int16_le(frame.data[4], frame.data[5]);
    ++can_log_window_frames_;
    if (vy != 0 || vw != 0) {
      ++can_log_window_nonzero_frames_;
      if (!can_log_window_has_nonzero_sample_) {
        can_log_window_first_nonzero_frame_ = frame;
        can_log_window_first_nonzero_at_ms_ = std::chrono::duration<double, std::milli>(
          now - can_log_window_start_).count();
        can_log_window_has_nonzero_sample_ = true;
      }
    }
    can_log_window_peak_abs_vy_ = std::max(
      can_log_window_peak_abs_vy_,
      static_cast<std::uint32_t>(std::abs(static_cast<std::int32_t>(vy))));
    can_log_window_peak_abs_vw_ = std::max(
      can_log_window_peak_abs_vw_,
      static_cast<std::uint32_t>(std::abs(static_cast<std::int32_t>(vw))));

    const double window_seconds = std::chrono::duration<double>(
      now - can_log_window_start_).count();
    if (window_seconds < 2.0) {
      return;
    }

    const auto zero_frames = can_log_window_frames_ - can_log_window_nonzero_frames_;
    const auto sample_vy = can_log_window_has_nonzero_sample_ ?
      decode_int16_le(
        can_log_window_first_nonzero_frame_.data[2],
        can_log_window_first_nonzero_frame_.data[3]) : 0;
    const auto sample_vw = can_log_window_has_nonzero_sample_ ?
      decode_int16_le(
        can_log_window_first_nonzero_frame_.data[4],
        can_log_window_first_nonzero_frame_.data[5]) : 0;
    RCLCPP_INFO(
      get_logger(),
      "Chassis CAN write window on %s: %.2fs frames=%zu nonzero=%zu zero=%zu "
      "vy=%d vw=%d peak_abs_vy=%u peak_abs_vw=%u "
      "vy_le=[%02X %02X] vw_le=[%02X %02X] "
      "first_nonzero=%s at=%.1fms vy=%d vw=%d "
      "vy_le=[%02X %02X] vw_le=[%02X %02X]",
      interface_.c_str(), window_seconds, can_log_window_frames_,
      can_log_window_nonzero_frames_, zero_frames, static_cast<int>(vy),
      static_cast<int>(vw), static_cast<unsigned int>(can_log_window_peak_abs_vy_),
      static_cast<unsigned int>(can_log_window_peak_abs_vw_),
      static_cast<unsigned int>(frame.data[2]), static_cast<unsigned int>(frame.data[3]),
      static_cast<unsigned int>(frame.data[4]), static_cast<unsigned int>(frame.data[5]),
      can_log_window_has_nonzero_sample_ ? "yes" : "none",
      can_log_window_has_nonzero_sample_ ? can_log_window_first_nonzero_at_ms_ : -1.0,
      static_cast<int>(sample_vy), static_cast<int>(sample_vw),
      static_cast<unsigned int>(can_log_window_first_nonzero_frame_.data[2]),
      static_cast<unsigned int>(can_log_window_first_nonzero_frame_.data[3]),
      static_cast<unsigned int>(can_log_window_first_nonzero_frame_.data[4]),
      static_cast<unsigned int>(can_log_window_first_nonzero_frame_.data[5]));

    can_log_window_start_ = now;
    can_log_window_frames_ = 0;
    can_log_window_nonzero_frames_ = 0;
    can_log_window_peak_abs_vy_ = 0;
    can_log_window_peak_abs_vw_ = 0;
    can_log_window_first_nonzero_frame_ = can_frame{};
    can_log_window_first_nonzero_at_ms_ = -1.0;
    can_log_window_has_nonzero_sample_ = false;
  }

  void send_current()
  {
    try {
      can_frame frame;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto now_ros_ns = get_clock()->now().nanoseconds();
        observeRosClockLocked(now_ros_ns);
        if (now_ros_ns <= 0) {
          have_terrain_ = false;
        }
        const auto now = std::chrono::steady_clock::now();
        const auto timeout = std::chrono::duration<double>(perception_timeout_);
        const auto terrain_timeout = std::chrono::duration<double>(terrain_config_.max_age);
        const bool perception_ready =
          !require_perception_ ||
          (have_odometry_ && have_obstacle_ &&
           now - last_odometry_ <= timeout && now - last_obstacle_ <= timeout);
        const bool fastlio_health_ready = have_fastlio_health_ && fastlio_healthy_ &&
          now - last_fastlio_health_ <= timeout;
        if (fastlio_health_ready) {
          fastlio_health_loss_pending_ = false;
          state_->set_health_loss_hold(false);
        } else if (require_fastlio_health_ && have_ever_fastlio_healthy_ &&
          !fastlio_health_loss_pending_)
        {
          fastlio_health_loss_pending_ = true;
          fastlio_health_loss_since_ = last_fastlio_health_ +
            std::chrono::duration_cast<std::chrono::steady_clock::duration>(timeout);
          state_->set_health_loss_hold(true);
        }
        const double fastlio_unhealthy_duration = fastlio_health_loss_pending_
          ? std::chrono::duration<double>(now - fastlio_health_loss_since_).count()
          : 0.0;
        const bool fastlio_ready = !require_fastlio_health_ ||
          bringup::fastlio_health_within_grace(
          fastlio_health_ready, have_ever_fastlio_healthy_, fastlio_unhealthy_duration,
          fastlio_health_loss_grace_sec_);
        const bool terrain_ready =
          !require_terrain_ || (have_terrain_ && now_ros_ns > 0 &&
          now - last_terrain_ <= terrain_timeout);
        const auto relocalization_timeout =
          std::chrono::duration<double>(relocalization_timeout_);
        const bool relocalization_ready =
          !require_relocalization_ ||
          (have_relocalization_ && now - last_relocalization_ <= relocalization_timeout);
        const bool navigation_ready = !require_navigation_ready_ ||
          (have_navigation_ready_ && navigation_ready_);
        std::string safety_reason;
        const auto append_safety_reason = [&safety_reason](const char * reason) {
            if (!safety_reason.empty()) {
              safety_reason += ", ";
            }
            safety_reason += reason;
          };
        if (!perception_ready) {
          append_safety_reason("perception_unavailable");
        }
        if (!fastlio_ready) {
          append_safety_reason("fastlio_unhealthy_or_stale");
        } else if (fastlio_health_loss_pending_) {
          append_safety_reason("fastlio_recovery_hold");
        }
        if (!terrain_ready) {
          append_safety_reason("terrain_invalid_or_expired");
        }
        if (!relocalization_ready) {
          append_safety_reason("relocalization_unavailable");
        }
        if (!navigation_ready) {
          append_safety_reason("navigation_not_ready");
        }
        if (state_->safety_fault_latched()) {
          append_safety_reason("command_safety_fault_latched");
        }
        if (!perception_ready || !fastlio_ready || !terrain_ready || !relocalization_ready || !navigation_ready) {
          state_->clear();
          if (!perception_ready && !perception_error_reported_) {
            const double odom_age =
              have_odometry_
              ? std::chrono::duration<double>(now - last_odometry_).count()
              : -1.0;
            const double obs_age =
              have_obstacle_
              ? std::chrono::duration<double>(now - last_obstacle_).count()
              : -1.0;
            RCLCPP_WARN(
              get_logger(),
              "Perception timed out; stopping chassis"
              " [odom %.2fs obs %.2fs limit %.2fs]",
              odom_age, obs_age, perception_timeout_);
            perception_error_reported_ = true;
          }
          if (!fastlio_ready) {
            RCLCPP_WARN_THROTTLE(
              get_logger(), *get_clock(), 2000,
              "FAST-LIO health is false or stale; stopping chassis");
          }
          if (!terrain_ready && !terrain_error_reported_) {
            const double terrain_age =
              have_terrain_
              ? std::chrono::duration<double>(now - last_terrain_).count()
              : -1.0;
            RCLCPP_WARN(
              get_logger(),
              "Terrain invalid or expired; stopping chassis"
              " [terrain %.2fs stamp_ns %" PRId64 " limit %.2fs]",
              terrain_age, last_terrain_stamp_ns_,
              terrain_config_.max_age);
            terrain_error_reported_ = true;
          }
          if (!relocalization_ready && !relocalization_error_reported_) {
            const double relocalization_age =
              have_relocalization_
              ? std::chrono::duration<double>(now - last_relocalization_).count()
              : -1.0;
            RCLCPP_WARN(
              get_logger(),
              "Relocalization timed out; stopping chassis"
              " [pose %.2fs limit %.2fs]",
              relocalization_age, relocalization_timeout_);
            relocalization_error_reported_ = true;
          }
          if (!navigation_ready) {
            RCLCPP_WARN_THROTTLE(
              get_logger(), *get_clock(), 2000,
              "Navigation lifecycle is not ready; stopping chassis");
          }
        } else {
          if (perception_error_reported_) {
            RCLCPP_INFO(get_logger(), "Perception data recovered");
            perception_error_reported_ = false;
          }
          if (terrain_error_reported_) {
            RCLCPP_INFO(get_logger(), "Terrain data recovered");
            terrain_error_reported_ = false;
          }
          if (relocalization_error_reported_) {
            RCLCPP_INFO(get_logger(), "Relocalization recovered");
            relocalization_error_reported_ = false;
          }
        }
        if (require_reverse_costmap_) {
          const auto command = state_->command(now);
          bool measured_motion = false;
          bool measured_reverse = false;
          if (reverse_odometry_ && now - reverse_odometry_received_at_ <=
            std::chrono::duration<double>(reverse_odometry_timeout_))
          {
            const auto & measured = reverse_odometry_->twist.twist;
            if (std::isfinite(measured.linear.x) && std::isfinite(measured.linear.y) &&
              std::isfinite(measured.angular.z))
            {
              geometry_msgs::msg::Twist measured_twist;
              measured_twist.linear.x = measured.linear.x;
              measured_twist.linear.y = measured.linear.y;
              measured_twist.angular.z = measured.angular.z;
              measured_motion = bringup::motion_safety_check_required(
                measured_twist, 0.0, 0.0, 0.0, reverse_motion_threshold_);
              measured_reverse = measured.linear.x < -reverse_motion_threshold_;
            }
          }
          const bool command_motion = bringup::motion_safety_check_required(
            command, 0.0, 0.0, 0.0, reverse_motion_threshold_);
          if (command_motion || measured_motion) {
            std::string reason;
            bool reverse_motion = measured_reverse ||
              command.linear.x < -reverse_motion_threshold_;
            if (!motionCommandIsSafeLocked(
                command, now_ros_ns, now, reverse_motion, reason))
            {
              state_->clear();
              motion_safety_trip_latch_.trip(now);
              motion_safety_reverse_blocked_ = reverse_motion;
              RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Motion command stopped by swept-footprint safety gate: %s", reason.c_str());
            }
          }
        }
        const bool reverse_safety_blocked =
          motion_safety_trip_latch_.update(now, false);
        if (!reverse_safety_blocked) {
          motion_safety_reverse_blocked_ = false;
        }
        publishReverseBlockedStatusLocked(
          reverse_safety_blocked && motion_safety_reverse_blocked_, now);
        if (reverse_safety_blocked) {
          append_safety_reason("swept_footprint_safety_gate");
        }
        publishSafetyStatusLocked(
          safety_reason.empty() ? "ready" : "stop: " + safety_reason, now);
        frame = state_->frame(now);
      }
      write_frame(frame);
      record_successful_frame(frame);
      if (can_error_reported_) {
        RCLCPP_INFO(get_logger(), "CAN send recovered");
        can_error_reported_ = false;
      }
    } catch (const std::exception & error) {
      if (!can_error_reported_) {
        RCLCPP_ERROR(get_logger(), "CAN send failed on %s: %s", interface_.c_str(), error.what());
        can_error_reported_ = true;
      }
    }
  }

  int socket_fd_{-1};
  std::string interface_;
  std::string chassis_model_;
  std::unique_ptr<bringup::CommandState> state_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr reverse_blocked_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr safety_status_publisher_;
  std::mutex mutex_;
  bool require_perception_{true};
  bool require_fastlio_health_{true};
  bool require_terrain_{true};
  bool require_relocalization_{false};
  bool require_navigation_ready_{false};
  bool require_reverse_costmap_{true};
  bool reverse_unknown_is_blocked_{false};
  bringup::ReverseSafetyTripLatch motion_safety_trip_latch_;
  bool motion_safety_reverse_blocked_{false};
  bool reverse_blocked_status_{false};
  bool have_reverse_blocked_publish_{false};
  bool have_safety_status_publish_{false};
  std::string last_safety_status_;
  std::chrono::steady_clock::time_point last_safety_status_publish_{};
  std::chrono::steady_clock::time_point last_reverse_blocked_publish_{};
  double perception_timeout_{0.5};
  double relocalization_timeout_{1.0};
  double reverse_costmap_timeout_{0.75};
  double reverse_odometry_timeout_{0.25};
  double reverse_projection_time_{0.5};
  double reverse_linear_deceleration_{0.5};
  double reverse_angular_deceleration_{0.6};
  double reverse_safety_margin_{0.10};
  double reverse_motion_threshold_{0.05};
  double reverse_max_linear_speed_{0.5};
  double reverse_max_angular_speed_{1.8};
  int64_t reverse_block_cost_{253};
  int reverse_confirmation_samples_{3};
  int reverse_latch_reset_cycles_{20};
  std::string reverse_base_frame_{"base_link"};
  std::vector<std::pair<double, double>> reverse_footprint_;
  bringup::TerrainGateConfig terrain_config_;
  bool have_odometry_{false};
  bool have_fastlio_health_{false};
  bool fastlio_healthy_{false};
  bool have_ever_fastlio_healthy_{false};
  bool fastlio_health_loss_pending_{false};
  double fastlio_health_loss_grace_sec_{0.3};
  bool have_obstacle_{false};
  bool have_terrain_{false};
  bool have_relocalization_{false};
  bool have_navigation_ready_{false};
  bool navigation_ready_{false};
  bool have_terrain_stamp_{false};
  int64_t last_terrain_stamp_ns_{0};
  int64_t last_ros_clock_ns_{0};
  bool perception_error_reported_{false};
  bool terrain_error_reported_{false};
  bool relocalization_error_reported_{false};
  std::chrono::steady_clock::time_point last_odometry_{};
  std::chrono::steady_clock::time_point last_fastlio_health_{};
  std::chrono::steady_clock::time_point fastlio_health_loss_since_{};
  std::chrono::steady_clock::time_point last_obstacle_{};
  std::chrono::steady_clock::time_point reverse_costmap_received_at_{};
  std::chrono::steady_clock::time_point reverse_odometry_received_at_{};
  std::chrono::steady_clock::time_point last_terrain_{};
  std::chrono::steady_clock::time_point last_relocalization_{};
  std::chrono::steady_clock::time_point can_log_window_start_{};
  std::size_t can_log_window_frames_{0};
  std::size_t can_log_window_nonzero_frames_{0};
  std::uint32_t can_log_window_peak_abs_vy_{0};
  std::uint32_t can_log_window_peak_abs_vw_{0};
  can_frame can_log_window_first_nonzero_frame_{};
  double can_log_window_first_nonzero_at_ms_{-1.0};
  bool can_log_window_has_nonzero_sample_{false};
  bool can_error_reported_{false};
  nav2_msgs::msg::Costmap::SharedPtr reverse_costmap_;
  nav_msgs::msg::Odometry::SharedPtr reverse_odometry_;
  tf2_ros::Buffer tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr fastlio_health_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr obstacle_subscription_;
  rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr reverse_costmap_subscription_;
  rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr terrain_subscription_;
  rclcpp::Subscription<small_gicp_relocalization::msg::RelocalizationStatus>::SharedPtr relocalization_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr navigation_ready_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<ChassisDriverNode>();
    rclcpp::spin(node);
  } catch (const std::exception & error) {
    std::cerr << "chassis_driver startup failed: " << error.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
