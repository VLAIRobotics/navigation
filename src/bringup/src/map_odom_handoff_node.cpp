#include <algorithm>
#include <chrono>
#include <cerrno>
#include <cmath>
#include <cinttypes>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <fcntl.h>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <sys/file.h>
#include <unistd.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <small_gicp_relocalization/msg/map_odom_target.hpp>
#include <small_gicp_relocalization/msg/navigation_pause_request.hpp>
#include <small_gicp_relocalization/msg/navigation_pause_status.hpp>
#include <small_gicp_relocalization/msg/relocalization_status.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>

class MapOdomHandoffNode final : public rclcpp::Node
{
public:
  MapOdomHandoffNode()
  : Node("map_odom_handoff")
  {
    handoff_session_id_ = static_cast<std::uint64_t>(
      std::chrono::steady_clock::now().time_since_epoch().count());
    if (handoff_session_id_ == 0) {
      handoff_session_id_ = 1;
    }
    const char * domain_id = std::getenv("ROS_DOMAIN_ID");
    const std::string default_lock_file =
      std::string("/tmp/nav2_map_odom_handoff_") +
      (domain_id != nullptr ? domain_id : "0") + ".lock";
    const auto lock_file = declare_parameter<std::string>("lock_file", default_lock_file);
    lock_fd_ = ::open(lock_file.c_str(), O_CREAT | O_RDWR | O_CLOEXEC, 0666);
    if (lock_fd_ < 0) {
      throw std::runtime_error(
              "Could not open map -> odom publisher lock " + lock_file + ": " +
              std::strerror(errno));
    }
    if (::flock(lock_fd_, LOCK_EX | LOCK_NB) != 0) {
      const auto lock_error = errno;
      ::close(lock_fd_);
      lock_fd_ = -1;
      throw std::runtime_error(
              "Another map_odom_handoff publisher is already active (lock " + lock_file +
              ": " + std::strerror(lock_error) + ")");
    }

    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    body_frame_ = declare_parameter<std::string>("body_frame", "body");
    const auto handoff_topic = declare_parameter<std::string>(
      "handoff_topic", "/localization/map_odom_handoff");
    const auto ready_topic = declare_parameter<std::string>(
      "ready_topic", "/localization/handoff_ready");
    const auto odometry_topic = declare_parameter<std::string>("odometry_topic", "/Odometry");
    const auto localization_status_topic = declare_parameter<std::string>(
      "localization_status_topic", "/small_gicp/relocalization_status");
    const auto navigation_pause_request_topic = declare_parameter<std::string>(
      "navigation_pause_request_topic", "/localization/navigation_pause_request");
    const auto navigation_pause_status_topic = declare_parameter<std::string>(
      "navigation_pause_status_topic", "/localization/navigation_pause_status");
    smoothing_time_constant_sec_ = declare_parameter<double>(
      "smoothing_time_constant_sec", 4.0);
    max_translation_speed_mps_ = declare_parameter<double>(
      "max_translation_speed_mps", 0.02);
    max_rotation_speed_deg_s_ = declare_parameter<double>(
      "max_rotation_speed_deg_s", 1.0);
    max_odometry_age_sec_ = declare_parameter<double>("max_odometry_age_sec", 0.5);
    require_navigation_pause_confirmation_ = declare_parameter<bool>(
      "require_navigation_pause_confirmation", false);
    large_relocalization_translation_threshold_m_ = declare_parameter<double>(
      "large_relocalization_translation_threshold_m", 0.20);
    large_relocalization_rotation_threshold_rad_ = declare_parameter<double>(
      "large_relocalization_rotation_threshold_rad", 0.08);
    localization_status_timeout_sec_ = declare_parameter<double>(
      "localization_status_timeout_sec", 1.0);
    navigation_pause_confirmation_timeout_sec_ = declare_parameter<double>(
      "navigation_pause_confirmation_timeout_sec", 1.0);
    const auto update_frequency_hz = declare_parameter<double>("update_frequency_hz", 100.0);
    if (!std::isfinite(smoothing_time_constant_sec_) || smoothing_time_constant_sec_ <= 0.0 ||
      !std::isfinite(max_translation_speed_mps_) || max_translation_speed_mps_ <= 0.0 ||
      !std::isfinite(max_rotation_speed_deg_s_) || max_rotation_speed_deg_s_ <= 0.0 ||
      !std::isfinite(max_odometry_age_sec_) || max_odometry_age_sec_ <= 0.0 ||
      !std::isfinite(large_relocalization_translation_threshold_m_) ||
      large_relocalization_translation_threshold_m_ < 0.0 ||
      !std::isfinite(large_relocalization_rotation_threshold_rad_) ||
      large_relocalization_rotation_threshold_rad_ < 0.0 ||
      !std::isfinite(localization_status_timeout_sec_) ||
      localization_status_timeout_sec_ <= 0.0 ||
      !std::isfinite(navigation_pause_confirmation_timeout_sec_) ||
      navigation_pause_confirmation_timeout_sec_ <= 0.0 ||
      !std::isfinite(update_frequency_hz) || update_frequency_hz <= 0.0)
    {
      throw std::invalid_argument(
              "handoff smoothing and update-rate parameters must be finite and positive");
    }

    ready_publisher_ = create_publisher<std_msgs::msg::Bool>(
      ready_topic, rclcpp::QoS(1).reliable().transient_local());
    std_msgs::msg::Bool not_ready;
    not_ready.data = false;
    ready_publisher_->publish(not_ready);
    RCLCPP_INFO(get_logger(), "Waiting for a fresh map -> odom localization handoff");

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      odometry_topic, rclcpp::QoS(10),
      [this](const nav_msgs::msg::Odometry::SharedPtr message) {
        odometryCallback(message);
      });
    localization_status_subscription_ = create_subscription<
      small_gicp_relocalization::msg::RelocalizationStatus>(
      localization_status_topic, rclcpp::QoS(10),
      [this](const small_gicp_relocalization::msg::RelocalizationStatus::SharedPtr message) {
        localizationStatusCallback(message);
      });
    navigation_pause_request_publisher_ = create_publisher<
      small_gicp_relocalization::msg::NavigationPauseRequest>(
      navigation_pause_request_topic, rclcpp::QoS(1).reliable().transient_local());
    navigation_pause_status_subscription_ = create_subscription<
      small_gicp_relocalization::msg::NavigationPauseStatus>(
      navigation_pause_status_topic, rclcpp::QoS(10).reliable().transient_local(),
      [this](const small_gicp_relocalization::msg::NavigationPauseStatus::SharedPtr message) {
        navigationPauseStatusCallback(message);
      });
    handoff_subscription_ = create_subscription<small_gicp_relocalization::msg::MapOdomTarget>(
      handoff_topic, rclcpp::QoS(1).reliable().transient_local(),
      [this](const small_gicp_relocalization::msg::MapOdomTarget::SharedPtr message) {
        handoffCallback(message);
      });
    const auto update_period = std::max(
      std::chrono::milliseconds(1), std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(1.0 / update_frequency_hz)));
    heartbeat_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(update_period),
      [this]() {publishCurrentTransform();});
  }

  ~MapOdomHandoffNode() override
  {
    if (lock_fd_ >= 0) {
      ::flock(lock_fd_, LOCK_UN);
      ::close(lock_fd_);
    }
  }

private:
  static bool finite(const tf2::Transform & transform)
  {
    const auto & origin = transform.getOrigin();
    const auto & rotation = transform.getRotation();
    return std::isfinite(origin.x()) && std::isfinite(origin.y()) &&
           std::isfinite(origin.z()) && std::isfinite(rotation.x()) &&
           std::isfinite(rotation.y()) && std::isfinite(rotation.z()) &&
           std::isfinite(rotation.w()) && rotation.length2() > 1e-12;
  }

  static void normalize(tf2::Transform & transform)
  {
    auto rotation = transform.getRotation();
    rotation.normalize();
    transform.setRotation(rotation);
  }

  void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr message)
  {
    if (message->header.frame_id != odom_frame_ || message->child_frame_id != body_frame_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Ignoring odometry %s -> %s; expected %s -> %s",
        message->header.frame_id.c_str(), message->child_frame_id.c_str(),
        odom_frame_.c_str(), body_frame_.c_str());
      return;
    }
    if (message->header.stamp.sec == 0 && message->header.stamp.nanosec == 0) {
      return;
    }

    tf2::Transform odom_body;
    tf2::fromMsg(message->pose.pose, odom_body);
    if (!finite(odom_body)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Ignoring non-finite odometry");
      return;
    }
    normalize(odom_body);
    const rclcpp::Time stamp(message->header.stamp, get_clock()->get_clock_type());
    if (has_odometry_ && stamp <= last_odometry_stamp_) {
      return;
    }
    odom_body_ = odom_body;
    last_odometry_stamp_ = stamp;
    last_odometry_wall_time_ = std::chrono::steady_clock::now();
    has_odometry_ = true;
    tryAcceptPendingTarget();
    publishReadinessIfChanged();
  }

  void handoffCallback(
    const small_gicp_relocalization::msg::MapOdomTarget::SharedPtr message)
  {
    const auto & transform = message->transform;
    if (transform.header.frame_id != map_frame_ || transform.child_frame_id != odom_frame_) {
      RCLCPP_ERROR(
        get_logger(), "Rejecting localization handoff %s -> %s; expected %s -> %s",
        transform.header.frame_id.c_str(), transform.child_frame_id.c_str(),
        map_frame_.c_str(), odom_frame_.c_str());
      return;
    }
    if ((transform.header.stamp.sec == 0 && transform.header.stamp.nanosec == 0) ||
      message->producer_session_id == 0 || message->initialization_generation == 0 ||
      message->localization_epoch == 0)
    {
      RCLCPP_ERROR(get_logger(), "Rejecting localization target with a zero stamp or identity");
      return;
    }

    tf2::Transform candidate;
    tf2::fromMsg(transform.transform, candidate);
    if (!finite(candidate)) {
      RCLCPP_ERROR(get_logger(), "Rejecting non-finite localization handoff");
      return;
    }
    normalize(candidate);

    const rclcpp::Time stamp(transform.header.stamp, get_clock()->get_clock_type());
    const bool matches_applied_localization = has_transform_ &&
      message->producer_session_id == transform_producer_session_id_ &&
      message->initialization_generation == transform_initialization_generation_ &&
      message->localization_epoch == transform_localization_epoch_;
    if (matches_applied_localization && has_target_ && stamp <= last_target_stamp_) {
      return;
    }
    const bool matches_pending_localization = has_pending_target_ &&
      message->producer_session_id == pending_target_session_id_ &&
      message->initialization_generation == pending_target_initialization_generation_ &&
      message->localization_epoch == pending_target_localization_epoch_;
    if (matches_pending_localization && stamp <= pending_target_stamp_) {
      return;
    }

    if (has_pending_target_ &&
      message->producer_session_id < pending_target_session_id_)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
        "Rejecting localization target from an older GICP process session");
      return;
    }
    pending_target_transform_ = candidate;
    pending_target_stamp_ = stamp;
    pending_target_session_id_ = message->producer_session_id;
    pending_target_initialization_generation_ = message->initialization_generation;
    pending_target_localization_epoch_ = message->localization_epoch;
    has_pending_target_ = true;
    tryAcceptPendingTarget();
    publishReadinessIfChanged();
  }

  bool pauseRequestMatchesPendingTarget() const
  {
    return pause_request_active_ &&
      pause_request_producer_session_id_ == pending_target_session_id_ &&
      pause_request_initialization_generation_ == pending_target_initialization_generation_ &&
      pause_request_localization_epoch_ == pending_target_localization_epoch_;
  }

  void publishPauseRequest(const bool pause_requested)
  {
    small_gicp_relocalization::msg::NavigationPauseRequest request;
    request.handoff_session_id = handoff_session_id_;
    request.request_id = pause_request_id_;
    request.producer_session_id = pause_request_producer_session_id_;
    request.initialization_generation = pause_request_initialization_generation_;
    request.localization_epoch = pause_request_localization_epoch_;
    request.pause_requested = pause_requested;
    navigation_pause_request_publisher_->publish(request);
  }

  void ensurePauseRequestForPendingTarget()
  {
    if (pauseRequestMatchesPendingTarget()) {
      return;
    }
    ++next_pause_request_id_;
    if (next_pause_request_id_ == 0) {
      ++next_pause_request_id_;
    }
    pause_request_id_ = next_pause_request_id_;
    pause_request_producer_session_id_ = pending_target_session_id_;
    pause_request_initialization_generation_ = pending_target_initialization_generation_;
    pause_request_localization_epoch_ = pending_target_localization_epoch_;
    pause_request_active_ = true;
    pause_request_sent_wall_time_ = std::chrono::steady_clock::now();
    navigation_pause_confirmed_ = false;
    navigation_pause_ack_request_id_ = 0;
    pause_request_active_gate_session_id_ = 0;
    candidate_after_localization_ = false;
    publishPauseRequest(true);
    publishReadinessIfChanged();
    RCLCPP_INFO(
      get_logger(), "Requested correlated Nav2 pause session=%" PRIu64
      " request=%" PRIu64 " localization=(%" PRIu64 ",%" PRIu64 ",%" PRIu64 ")",
      handoff_session_id_, pause_request_id_, pause_request_producer_session_id_,
      pause_request_initialization_generation_, pause_request_localization_epoch_);
  }

  bool navigationPauseConfirmationFreshForPendingTarget() const
  {
    if (!pauseRequestMatchesPendingTarget() || !navigation_pause_confirmed_ ||
      navigation_pause_ack_request_id_ != pause_request_id_ ||
      pause_request_active_gate_session_id_ == 0 ||
      pause_request_active_gate_session_id_ != latest_gate_session_id_ ||
      pause_status_received_wall_time_ < pause_request_sent_wall_time_)
    {
      return false;
    }
    return std::chrono::duration<double>(
      std::chrono::steady_clock::now() - pause_status_received_wall_time_).count() <=
           navigation_pause_confirmation_timeout_sec_;
  }

  void releasePauseRequest()
  {
    if (!pause_request_active_) {
      return;
    }
    publishPauseRequest(false);
    pause_request_active_ = false;
    navigation_pause_confirmed_ = false;
    navigation_pause_ack_request_id_ = 0;
    pause_request_active_gate_session_id_ = 0;
  }

  void navigationPauseStatusCallback(
    const small_gicp_relocalization::msg::NavigationPauseStatus::SharedPtr message)
  {
    if (message->gate_session_id == 0) {
      return;
    }
    if (latest_gate_session_id_ != 0 && message->gate_session_id < latest_gate_session_id_) {
      return;
    }
    if (message->gate_session_id > latest_gate_session_id_) {
      latest_gate_session_id_ = message->gate_session_id;
      navigation_pause_confirmed_ = false;
      navigation_pause_ack_request_id_ = 0;
      pause_request_active_gate_session_id_ = 0;
    }

    const bool matches_active_request = pause_request_active_ &&
      message->handoff_session_id == handoff_session_id_ &&
      message->request_id == pause_request_id_ &&
      message->producer_session_id == pause_request_producer_session_id_ &&
      message->initialization_generation == pause_request_initialization_generation_ &&
      message->localization_epoch == pause_request_localization_epoch_;
    if (!matches_active_request) {
      navigation_pause_confirmed_ = false;
      navigation_pause_ack_request_id_ = 0;
      pause_request_active_gate_session_id_ = 0;
    } else {
      navigation_pause_confirmed_ = message->confirmed;
      navigation_pause_ack_request_id_ = message->request_id;
      pause_request_active_gate_session_id_ = message->gate_session_id;
      pause_status_received_wall_time_ = std::chrono::steady_clock::now();
    }
    if (navigationPauseConfirmationFreshForPendingTarget()) {
      tryAcceptPendingTarget();
    }
    publishReadinessIfChanged();
  }

  void tryAcceptPendingTarget()
  {
    if (!has_pending_target_ || !has_localization_status_) {
      return;
    }
    if (status_producer_session_id_ > pending_target_session_id_ ||
      (status_producer_session_id_ == pending_target_session_id_ &&
      status_initialization_generation_ > pending_target_initialization_generation_) ||
      (status_producer_session_id_ == pending_target_session_id_ &&
      status_initialization_generation_ == pending_target_initialization_generation_ &&
      status_localization_epoch_ > pending_target_localization_epoch_))
    {
      has_pending_target_ = false;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
        "Discarding localization target older than current localization status");
      return;
    }
    if (status_producer_session_id_ != pending_target_session_id_ ||
      status_initialization_generation_ != pending_target_initialization_generation_ ||
      status_localization_epoch_ != pending_target_localization_epoch_ ||
      !localization_valid_ || !localizationStatusFresh())
    {
      return;
    }

    const bool same_localization = has_transform_ &&
      transform_producer_session_id_ == pending_target_session_id_ &&
      transform_initialization_generation_ == pending_target_initialization_generation_ &&
      transform_localization_epoch_ == pending_target_localization_epoch_;
    bool large_relocalization = false;
    double robot_correction_m = 0.0;
    double robot_correction_rad = 0.0;
    if (has_transform_ && !same_localization) {
      if (!odometryIsFresh()) {
        return;
      }
      const auto current_map_body = current_transform_ * odom_body_;
      const auto candidate_map_body = pending_target_transform_ * odom_body_;
      robot_correction_m =
        (candidate_map_body.getOrigin() - current_map_body.getOrigin()).length();
      auto delta = candidate_map_body.getRotation() * current_map_body.getRotation().inverse();
      delta.normalize();
      robot_correction_rad = 2.0 * std::acos(std::clamp(std::abs(delta.w()), 0.0, 1.0));
      large_relocalization =
        robot_correction_m > large_relocalization_translation_threshold_m_ ||
        robot_correction_rad > large_relocalization_rotation_threshold_rad_;
    }

    const bool requires_pause_confirmation = require_navigation_pause_confirmation_ &&
      (!has_transform_ || large_relocalization || pause_request_active_);
    if (requires_pause_confirmation) {
      ensurePauseRequestForPendingTarget();
      if (!navigationPauseConfirmationFreshForPendingTarget()) {
        return;
      }
    }
    const bool snap_to_baseline = !has_transform_ || large_relocalization ||
      (require_navigation_pause_confirmation_ && pause_request_active_);

    target_transform_ = pending_target_transform_;
    if (snap_to_baseline) {
      current_transform_ = pending_target_transform_;
      has_transform_ = true;
    }
    transform_producer_session_id_ = pending_target_session_id_;
    transform_initialization_generation_ = pending_target_initialization_generation_;
    transform_localization_epoch_ = pending_target_localization_epoch_;
    last_target_stamp_ = pending_target_stamp_;
    has_target_ = true;
    has_pending_target_ = false;
    last_update_wall_time_ = std::chrono::steady_clock::now();
    if (snap_to_baseline) {
      broadcastCurrentTransform();
    }
    candidate_after_localization_ = true;
    publishReadinessIfChanged();
    releasePauseRequest();
    RCLCPP_INFO(
      get_logger(), "Accepted map -> odom target session=%" PRIu64 " generation=%" PRIu64
      " epoch=%" PRIu64 " correction=%.3fm/%.3fdeg snap=%s pause_confirmed=%s",
      transform_producer_session_id_, transform_initialization_generation_,
      transform_localization_epoch_, robot_correction_m,
      robot_correction_rad * 180.0 / 3.14159265358979323846,
      snap_to_baseline ? "true" : "false",
      requires_pause_confirmation ? "true" : "false");
  }

  static tf2::Quaternion limitRotation(
    const tf2::Quaternion & current, const tf2::Quaternion & target,
    const double alpha, const double max_step_rad)
  {
    auto delta = target * current.inverse();
    delta.normalize();
    if (delta.w() < 0.0) {
      delta = tf2::Quaternion(-delta.x(), -delta.y(), -delta.z(), -delta.w());
    }
    const double vector_norm = std::sqrt(
      delta.x() * delta.x() + delta.y() * delta.y() + delta.z() * delta.z());
    if (vector_norm < 1e-12) {
      return current;
    }
    const double angle = 2.0 * std::atan2(vector_norm, std::clamp(delta.w(), 0.0, 1.0));
    const double step = std::min(angle * alpha, max_step_rad);
    tf2::Vector3 axis(delta.x() / vector_norm, delta.y() / vector_norm, delta.z() / vector_norm);
    auto result = tf2::Quaternion(axis, step) * current;
    result.normalize();
    return result;
  }

  void advanceTransform(const double dt_sec)
  {
    if (!has_target_ || !has_odometry_) {
      return;
    }
    const double odometry_age = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - last_odometry_wall_time_).count();
    if (odometry_age > max_odometry_age_sec_) {
      return;
    }

    const auto current_map_body = current_transform_ * odom_body_;
    const auto target_map_body = target_transform_ * odom_body_;
    const auto position_delta = target_map_body.getOrigin() - current_map_body.getOrigin();
    const double distance = position_delta.length();
    const double alpha = 1.0 - std::exp(-dt_sec / smoothing_time_constant_sec_);
    tf2::Vector3 next_position = current_map_body.getOrigin();
    if (distance > 1e-9) {
      const double step = std::min(distance * alpha, max_translation_speed_mps_ * dt_sec);
      next_position += position_delta * (step / distance);
    }

    const double max_rotation_step_rad = max_rotation_speed_deg_s_ *
      3.14159265358979323846 / 180.0 * dt_sec;
    const auto next_rotation = limitRotation(
      current_map_body.getRotation(), target_map_body.getRotation(), alpha,
      max_rotation_step_rad);
    const tf2::Transform next_map_body(next_rotation, next_position);
    current_transform_ = next_map_body * odom_body_.inverse();
  }

  void localizationStatusCallback(
    const small_gicp_relocalization::msg::RelocalizationStatus::SharedPtr message)
  {
    if (message->producer_session_id == 0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000, "Ignoring localization status without a producer session");
      return;
    }
    if (has_localization_status_ &&
      (message->producer_session_id < status_producer_session_id_ ||
      (message->producer_session_id == status_producer_session_id_ &&
      (message->initialization_generation < status_initialization_generation_ ||
      (message->initialization_generation == status_initialization_generation_ &&
      message->localization_epoch < status_localization_epoch_)))))
    {
      return;
    }

    const bool new_producer_session = !has_localization_status_ ||
      message->producer_session_id != status_producer_session_id_;
    status_producer_session_id_ = message->producer_session_id;
    status_initialization_generation_ = message->initialization_generation;
    status_localization_epoch_ = message->localization_epoch;
    localization_valid_ = message->valid &&
      message->state == small_gicp_relocalization::msg::RelocalizationStatus::LOCALIZED;
    has_localization_status_ = true;
    status_stale_reported_ = false;
    last_localization_status_wall_time_ = std::chrono::steady_clock::now();
    if (new_producer_session && has_transform_) {
      target_transform_ = current_transform_;
      candidate_after_localization_ = false;
    }
    if (localization_valid_ && message->degraded) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "Localization tracking is degraded; holding the last trusted map -> odom target");
    }
    if (!localization_valid_ && has_transform_) {
      target_transform_ = current_transform_;
      candidate_after_localization_ = false;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "Localization degraded; holding the last published map -> odom TF");
    }
    if (localization_valid_ && has_transform_ &&
      (status_producer_session_id_ != transform_producer_session_id_ ||
      status_initialization_generation_ != transform_initialization_generation_ ||
      status_localization_epoch_ != transform_localization_epoch_))
    {
      candidate_after_localization_ = false;
    }
    tryAcceptPendingTarget();
    publishReadinessIfChanged();
  }

  void checkLocalizationFreshness()
  {
    const double status_age = has_localization_status_ ? std::chrono::duration<double>(
      std::chrono::steady_clock::now() - last_localization_status_wall_time_).count() :
      std::numeric_limits<double>::infinity();
    if (localization_valid_ && status_age > localization_status_timeout_sec_ &&
      !status_stale_reported_)
    {
      status_stale_reported_ = true;
      RCLCPP_WARN(
        get_logger(), "Localization status heartbeat stale after %.2f s; freezing map -> odom updates",
        status_age);
    }
    publishReadinessIfChanged();
  }

  bool localizationStatusFresh() const
  {
    return has_localization_status_ && std::chrono::duration<double>(
      std::chrono::steady_clock::now() - last_localization_status_wall_time_).count() <=
      localization_status_timeout_sec_;
  }

  void publishReadinessIfChanged()
  {
    const bool ready = has_transform_ && candidate_after_localization_ &&
      localization_valid_ && localizationStatusFresh() && odometryIsFresh();
    if (ready == ready_) {
      return;
    }
    ready_ = ready;
    std_msgs::msg::Bool message;
    message.data = ready_;
    ready_publisher_->publish(message);
    RCLCPP_INFO(
      get_logger(), "Localization handoff readiness changed: %s",
      ready_ ? "ready" : "not ready");
  }

  bool odometryIsFresh() const
  {
    return has_odometry_ && std::chrono::duration<double>(
      std::chrono::steady_clock::now() - last_odometry_wall_time_).count() <=
      max_odometry_age_sec_;
  }

  void publishCurrentTransform()
  {
    checkLocalizationFreshness();
    tryAcceptPendingTarget();
    if (!has_transform_) {
      return;
    }
    const auto update_time = std::chrono::steady_clock::now();
    const double dt_sec = std::chrono::duration<double>(
      update_time - last_update_wall_time_).count();
    if (ready_ && dt_sec > 0.0) {
      advanceTransform(dt_sec);
    }
    last_update_wall_time_ = update_time;

    broadcastCurrentTransform();
  }

  void broadcastCurrentTransform()
  {
    if (!has_transform_) {
      return;
    }
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = now();
    transform.header.frame_id = map_frame_;
    transform.child_frame_id = odom_frame_;
    transform.transform = tf2::toMsg(current_transform_);
    tf_broadcaster_->sendTransform(transform);
  }

  int lock_fd_{-1};
  bool ready_{false};
  bool has_transform_{false};
  bool has_target_{false};
  bool has_pending_target_{false};
  bool has_odometry_{false};
  bool localization_valid_{false};
  bool has_localization_status_{false};
  bool candidate_after_localization_{false};
  bool navigation_pause_confirmed_{false};
  bool pause_request_active_{false};
  bool require_navigation_pause_confirmation_{false};
  bool status_stale_reported_{false};
  std::string map_frame_;
  std::string odom_frame_;
  std::string body_frame_;
  double smoothing_time_constant_sec_{4.0};
  double max_translation_speed_mps_{0.02};
  double max_rotation_speed_deg_s_{1.0};
  double max_odometry_age_sec_{0.5};
  double large_relocalization_translation_threshold_m_{0.20};
  double large_relocalization_rotation_threshold_rad_{0.08};
  double localization_status_timeout_sec_{1.0};
  double navigation_pause_confirmation_timeout_sec_{1.0};
  std::uint64_t handoff_session_id_{0};
  std::uint64_t next_pause_request_id_{0};
  std::uint64_t pause_request_id_{0};
  std::uint64_t pause_request_producer_session_id_{0};
  std::uint64_t pause_request_initialization_generation_{0};
  std::uint64_t pause_request_localization_epoch_{0};
  std::uint64_t latest_gate_session_id_{0};
  std::uint64_t navigation_pause_ack_request_id_{0};
  std::uint64_t pause_request_active_gate_session_id_{0};
  std::uint64_t status_producer_session_id_{0};
  std::uint64_t status_initialization_generation_{0};
  std::uint64_t status_localization_epoch_{0};
  std::uint64_t transform_producer_session_id_{0};
  std::uint64_t transform_initialization_generation_{0};
  std::uint64_t transform_localization_epoch_{0};
  std::uint64_t pending_target_session_id_{0};
  std::uint64_t pending_target_initialization_generation_{0};
  std::uint64_t pending_target_localization_epoch_{0};
  rclcpp::Time last_odometry_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_target_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time pending_target_stamp_{0, 0, RCL_ROS_TIME};
  tf2::Transform current_transform_{tf2::Transform::getIdentity()};
  tf2::Transform target_transform_{tf2::Transform::getIdentity()};
  tf2::Transform pending_target_transform_{tf2::Transform::getIdentity()};
  tf2::Transform odom_body_{tf2::Transform::getIdentity()};
  std::chrono::steady_clock::time_point last_odometry_wall_time_{};
  std::chrono::steady_clock::time_point last_update_wall_time_{};
  std::chrono::steady_clock::time_point last_localization_status_wall_time_{};
  std::chrono::steady_clock::time_point pause_request_sent_wall_time_{};
  std::chrono::steady_clock::time_point pause_status_received_wall_time_{};
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready_publisher_;
  rclcpp::Publisher<
    small_gicp_relocalization::msg::NavigationPauseRequest>::SharedPtr
    navigation_pause_request_publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
  rclcpp::Subscription<
    small_gicp_relocalization::msg::RelocalizationStatus>::SharedPtr
    localization_status_subscription_;
  rclcpp::Subscription<
    small_gicp_relocalization::msg::NavigationPauseStatus>::SharedPtr
    navigation_pause_status_subscription_;
  rclcpp::Subscription<
    small_gicp_relocalization::msg::MapOdomTarget>::SharedPtr handoff_subscription_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<MapOdomHandoffNode>());
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("map_odom_handoff"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
