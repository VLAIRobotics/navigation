#include <algorithm>
#include <chrono>
#include <cmath>
#include <cinttypes>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <action_msgs/srv/cancel_goal.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/srv/get_state.hpp>
#include <nav2_msgs/srv/manage_lifecycle_nodes.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <small_gicp_relocalization/msg/navigation_pause_request.hpp>
#include <small_gicp_relocalization/msg/navigation_pause_status.hpp>
#include <small_gicp_relocalization/msg/relocalization_status.hpp>
#include <std_msgs/msg/bool.hpp>

#include "bringup/navigation_gate_state.hpp"
#include "bringup/status_heartbeat.hpp"

class NavigationGateNode final : public rclcpp::Node
{
public:
  NavigationGateNode()
  : Node("navigation_gate")
  {
    gate_session_id_ = static_cast<std::uint64_t>(
      std::chrono::steady_clock::now().time_since_epoch().count());
    if (gate_session_id_ == 0) {
      gate_session_id_ = 1;
    }
    const auto status_topic = declare_parameter<std::string>(
      "status_topic", "/small_gicp/relocalization_status");
    const auto localization_ready_topic = declare_parameter<std::string>(
      "localization_ready_topic", "/localization/handoff_ready");
    const bool use_relocalization_status = declare_parameter<bool>(
      "use_relocalization_status", true);
    lifecycle_service_ = declare_parameter<std::string>(
      "lifecycle_service", "/lifecycle_manager_navigation/manage_nodes");
    navigation_ready_topic_ = declare_parameter<std::string>(
      "navigation_ready_topic", "/navigation_ready");
    startup_epoch_ns_ = declare_parameter<std::int64_t>("startup_epoch_ns", 0);
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/Odometry");
    odom_freshness_timeout_sec_ = declare_parameter<double>(
      "odom_freshness_timeout", 0.4);
    require_perception_ = declare_parameter<bool>("require_perception", true);
    perception_topic_ = declare_parameter<std::string>(
      "perception_topic", "/obstacles/ground_veto");
    perception_timeout_sec_ = declare_parameter<double>("perception_timeout", 1.5);
    navigation_paused_topic_ = declare_parameter<std::string>(
      "navigation_paused_topic", "/navigation_paused");
    navigation_pause_request_topic_ = declare_parameter<std::string>(
      "navigation_pause_request_topic", "/localization/navigation_pause_request");
    navigation_pause_status_topic_ = declare_parameter<std::string>(
      "navigation_pause_status_topic", "/localization/navigation_pause_status");
    status_timeout_sec_ = declare_parameter<double>("localization_status_timeout", 1.0);
    require_fastlio_health_ = declare_parameter<bool>("require_fastlio_health", false);
    fastlio_health_topic_ = declare_parameter<std::string>("fastlio_health_topic", "/fastlio/health");
    fastlio_health_is_localization_signal_ = !use_relocalization_status &&
      require_fastlio_health_ && localization_ready_topic == fastlio_health_topic_;
    fastlio_health_timeout_sec_ = declare_parameter<double>("fastlio_health_timeout", 0.5);
    fastlio_health_loss_grace_sec_ = declare_parameter<double>(
      "fastlio_health_loss_grace", 1.25);
    require_terrain_status_ = declare_parameter<bool>("require_terrain_status", false);
    terrain_status_topic_ = declare_parameter<std::string>(
      "terrain_status_topic", "/fastdem/terrain_valid");
    terrain_timeout_sec_ = declare_parameter<double>("terrain_timeout", 1.2);
    terrain_valid_topic_ = declare_parameter<std::string>(
      "terrain_valid_topic", "/navigation/terrain_valid");
    lifecycle_retry_period_sec_ = declare_parameter<double>("lifecycle_retry_period", 1.0);
    lifecycle_retry_backoff_max_sec_ = declare_parameter<double>(
      "lifecycle_retry_backoff_max", 30.0);
    lifecycle_request_timeout_sec_ = declare_parameter<double>("lifecycle_request_timeout", 10.0);
    lifecycle_state_poll_period_sec_ = declare_parameter<double>(
      "lifecycle_state_poll_period", 0.5);
    managed_nodes_ = declare_parameter<std::vector<std::string>>(
      "managed_nodes", {
        "controller_server", "smoother_server", "planner_server", "behavior_server",
        "bt_navigator", "waypoint_follower", "velocity_smoother"});
    if (!std::isfinite(status_timeout_sec_) || status_timeout_sec_ <= 0.0 ||
      !std::isfinite(lifecycle_retry_period_sec_) || lifecycle_retry_period_sec_ <= 0.0 ||
      !std::isfinite(lifecycle_retry_backoff_max_sec_) ||
      lifecycle_retry_backoff_max_sec_ < lifecycle_retry_period_sec_ ||
      !std::isfinite(lifecycle_request_timeout_sec_) || lifecycle_request_timeout_sec_ <= 0.0 ||
      !std::isfinite(lifecycle_state_poll_period_sec_) ||
      lifecycle_state_poll_period_sec_ <= 0.0 ||
      !std::isfinite(fastlio_health_timeout_sec_) || fastlio_health_timeout_sec_ <= 0.0 ||
      !std::isfinite(fastlio_health_loss_grace_sec_) || fastlio_health_loss_grace_sec_ < 0.0 ||
      !std::isfinite(terrain_timeout_sec_) || terrain_timeout_sec_ <= 0.0 ||
      !std::isfinite(odom_freshness_timeout_sec_) || odom_freshness_timeout_sec_ <= 0.0 ||
      !std::isfinite(perception_timeout_sec_) || perception_timeout_sec_ <= 0.0 ||
      perception_topic_.empty() ||
      managed_nodes_.empty())
    {
      throw std::invalid_argument(
        "lifecycle and health timeout parameters must be finite and greater than zero; "
        "health loss grace must be finite and non-negative; managed_nodes must not be empty");
    }

    ready_publisher_ = create_publisher<std_msgs::msg::Bool>(
      navigation_ready_topic_, rclcpp::QoS(1).transient_local());
    publishReady(false);
    navigation_paused_publisher_ = create_publisher<std_msgs::msg::Bool>(
      navigation_paused_topic_, rclcpp::QoS(1).reliable().transient_local());
    navigation_pause_status_publisher_ = create_publisher<
      small_gicp_relocalization::msg::NavigationPauseStatus>(
      navigation_pause_status_topic_, rclcpp::QoS(1).reliable().transient_local());
    publishNavigationPauseStatus();
    navigation_pause_request_subscription_ = create_subscription<
      small_gicp_relocalization::msg::NavigationPauseRequest>(
      navigation_pause_request_topic_, rclcpp::QoS(1).reliable().transient_local(),
      [this](const small_gicp_relocalization::msg::NavigationPauseRequest::SharedPtr message) {
        navigationPauseRequestCallback(message);
      });
    terrain_valid_publisher_ = create_publisher<std_msgs::msg::Bool>(
      terrain_valid_topic_, rclcpp::QoS(1).transient_local());
    publishTerrainStatus(false);
    terrain_status_subscription_ = create_subscription<std_msgs::msg::Bool>(
      terrain_status_topic_, rclcpp::QoS(1).reliable().transient_local(),
      [this](const std_msgs::msg::Bool::SharedPtr message) {
        have_terrain_status_ = true;
        terrain_status_message_valid_ = message->data;
        last_terrain_status_ = std::chrono::steady_clock::now();
        updateTerrainStatus();
        reconcileLocalizationReadiness(
          terrainReady() ? "FastDEM terrain is fresh" : "FastDEM terrain is stale or invalid",
          terrainReady() ? "terrain_ready" : "terrain_data_stale");
      });
    if (use_relocalization_status) {
      handoff_subscription_ = create_subscription<std_msgs::msg::Bool>(
        localization_ready_topic, rclcpp::QoS(1).reliable().transient_local(),
        [this, localization_ready_topic](const std_msgs::msg::Bool::SharedPtr message) {
          handoff_ready_ = message->data;
          localization_baseline_ready_ = handoff_ready_ && localization_status_ready_;
          reconcileLocalizationReadiness(
            localization_baseline_ready_ ? "localization handoff is settled" :
            "localization handoff is degraded or awaiting a trusted target",
            localization_baseline_ready_ ? "localization_ready" : "localization_handoff_unavailable");
        });
      status_subscription_ = create_subscription<small_gicp_relocalization::msg::RelocalizationStatus>(
        status_topic, rclcpp::QoS(10),
        [this, status_topic](const small_gicp_relocalization::msg::RelocalizationStatus::SharedPtr message) {
          have_status_heartbeat_ = true;
          last_status_heartbeat_ = std::chrono::steady_clock::now();
          const bool localized = message->valid &&
            message->state == small_gicp_relocalization::msg::RelocalizationStatus::LOCALIZED;
          localization_status_ready_ = localized;
          localization_baseline_ready_ = localized && handoff_ready_;
          if (localized) {
            if (message->degraded) {
              RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Localization tracking is degraded (generation=%" PRIu64 "); retaining the "
                "last trusted map -> odom until recovery or the configured validity timeout",
                message->initialization_generation);
            } else {
              RCLCPP_INFO_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Relocalization success received (generation=%" PRIu64 "); checking FAST-LIO health "
                "before Nav2 lifecycle startup",
                message->initialization_generation);
            }
          } else {
            RCLCPP_INFO_THROTTLE(
              get_logger(), *get_clock(), 3000,
              "Waiting for relocalization success on %s (state=%u valid=%s generation=%" PRIu64 ")",
              status_topic.c_str(), message->state, message->valid ? "true" : "false",
              message->initialization_generation);
          }
          reconcileLocalizationReadiness(
            localized ? "localization status is valid" : "localization status is invalid",
            !localized ? "localization_lost" :
            (handoff_ready_ ? "localization_ready" : "localization_handoff_unavailable"));
        });
    } else {
      handoff_subscription_ = create_subscription<std_msgs::msg::Bool>(
        localization_ready_topic, rclcpp::QoS(1),
        [this, localization_ready_topic](const std_msgs::msg::Bool::SharedPtr message) {
          localization_baseline_ready_ = bringup::updateLocalizationBaselineFromHandoff(
            localization_baseline_ready_, message->data,
            fastlio_health_is_localization_signal_);
          if (message->data) {
            RCLCPP_INFO_THROTTLE(
              get_logger(), *get_clock(), 2000,
              "Localization handoff received on %s; checking FAST-LIO health before Nav2 activation",
              localization_ready_topic.c_str());
          }
          reconcileLocalizationReadiness(
            message->data ? "localization handoff ready" : "localization handoff is not ready",
            message->data ? "localization_ready" : "localization_lost");
        });
    }

    if (require_fastlio_health_) {
      fastlio_health_subscription_ = create_subscription<std_msgs::msg::Bool>(
        fastlio_health_topic_, rclcpp::QoS(1),
        [this](const std_msgs::msg::Bool::SharedPtr message) {
          const bool loss_expired = fastlioHealthLossExpired();
          const auto now = std::chrono::steady_clock::now();
          have_fastlio_health_ = true;
          if (fastlio_health_is_localization_signal_) {
            localization_baseline_ready_ = bringup::updateLocalizationBaselineFromHandoff(
              localization_baseline_ready_, message->data, true);
          }
          if (message->data) {
            if (loss_expired && desired_active_) {
              handleFastlioHealthLossDeadline();
            }
            const bool recovered_during_grace =
              fastlio_health_loss_pending_ && desired_active_ && !loss_expired;
            fastlio_health_loss_pending_ = false;
            fastlio_healthy_ = true;
            last_fastlio_health_ = now;
            if (recovered_during_grace) {
              RCLCPP_INFO(
                get_logger(), "FAST-LIO health recovered within grace; retaining the navigation goal");
            }
            reconcileLocalizationReadiness("FAST-LIO health recovered", "fastlio_health_ready");
          } else {
            fastlio_healthy_ = false;
            last_fastlio_health_ = now;
            beginFastlioHealthLoss("FAST-LIO health is false");
            handleFastlioHealthLossDeadline();
            if (desired_active_) {
              reconcileLifecycle();
            } else {
              reconcileLocalizationReadiness("FAST-LIO health is false", "fastlio_health_loss");
            }
          }
        });
    }

    odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::QoS(10),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr message) {
        last_odom_stamp_ = message->header.stamp;
        last_odom_receive_ = std::chrono::steady_clock::now();
        have_odom_ = true;
        ++odom_messages_since_report_;
      });
    perception_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      perception_topic_, rclcpp::QoS(10).reliable(),
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr message) {
        last_perception_stamp_ = message->header.stamp;
        last_perception_receive_ = std::chrono::steady_clock::now();
        have_perception_message_ = true;
        ++perception_messages_since_report_;
      });

    lifecycle_client_ = create_client<nav2_msgs::srv::ManageLifecycleNodes>(lifecycle_service_);
    for (const auto & node_name : managed_nodes_) {
      if (node_name.empty()) {
        throw std::invalid_argument("managed_nodes must not contain an empty node name");
      }
      auto state_service = node_name;
      if (state_service.front() != '/') {
        state_service = "/" + state_service;
      }
      state_service += "/get_state";
      lifecycle_state_clients_.push_back(create_client<LifecycleState>(state_service));
    }
    navigate_through_poses_cancel_client_ = create_client<action_msgs::srv::CancelGoal>(
      "/navigate_through_poses/_action/cancel_goal");
    timer_ = create_wall_timer(std::chrono::milliseconds(100), [this, use_relocalization_status]() {
      if (use_relocalization_status) {
        const double status_age = std::chrono::duration<double>(
          std::chrono::steady_clock::now() - last_status_heartbeat_).count();
        if (!bringup::localizationStatusHeartbeatIsFresh(
            have_status_heartbeat_, status_age, status_timeout_sec_))
        {
          if (have_status_heartbeat_) {
            have_status_heartbeat_ = false;
            localization_status_ready_ = false;
            localization_baseline_ready_ = false;
            loseLocalization(
              "relocalization status heartbeat timed out", "localization_lost");
          }
        }
      }
      if (require_fastlio_health_ && fastlio_healthy_ && !fastlioHealthFresh()) {
        fastlio_healthy_ = false;
        beginFastlioHealthLoss("FAST-LIO health heartbeat timed out");
      }
      updateTerrainStatus();
      if (localization_baseline_ready_ && require_terrain_status_ &&
        (desired_active_ || navigation_ever_started_))
      {
        reconcileLocalizationReadiness(
          "FastDEM terrain data is stale or invalid", "terrain_data_stale");
      }
      handleFastlioHealthLossDeadline();
      observeLifecycleManager();
      checkLifecycleRequestTimeout();
      checkLifecycleStateQueryTimeout();
      startLifecycleStateQuery();
      reconcileLifecycle();
      logOdometryDiagnostics();
      publishNavigationReadyHeartbeatIfDue();
    });
  }

private:
  using Lifecycle = nav2_msgs::srv::ManageLifecycleNodes;
  using LifecycleState = lifecycle_msgs::srv::GetState;
  using LifecycleStateClient = rclcpp::Client<LifecycleState>;

  struct PendingStateRequest
  {
    LifecycleStateClient::SharedPtr client;
    std::int64_t request_id;
  };

  bool fastlioHealthFresh() const
  {
    if (!have_fastlio_health_) return false;
    const double age = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - last_fastlio_health_).count();
    return bringup::navigationGateShouldBeActive(
      true, true, have_fastlio_health_, fastlio_healthy_, age, fastlio_health_timeout_sec_);
  }

  double terrainStatusAgeSeconds() const
  {
    return have_terrain_status_ ? std::chrono::duration<double>(
      std::chrono::steady_clock::now() - last_terrain_status_).count() :
      std::numeric_limits<double>::infinity();
  }

  bool terrainStatusFresh() const
  {
    return bringup::navigationGateTerrainStatusIsFresh(
      have_terrain_status_, terrainStatusAgeSeconds(), terrain_timeout_sec_);
  }

  bool terrainReady() const
  {
    return bringup::navigationGateTerrainIsReady(
      require_terrain_status_, terrain_status_message_valid_, terrainStatusAgeSeconds(),
      terrain_timeout_sec_);
  }

  bool perceptionFresh() const
  {
    if (!require_perception_) {
      return true;
    }
    if (!have_perception_message_) {
      return false;
    }
    const auto steady_now = std::chrono::steady_clock::now();
    const double receive_age = std::chrono::duration<double>(
      steady_now - last_perception_receive_).count();
    const double source_age = (now() - rclcpp::Time(
      last_perception_stamp_, get_clock()->get_clock_type())).seconds();
    return receive_age >= 0.0 && receive_age <= perception_timeout_sec_ &&
      source_age >= 0.0 && source_age <= perception_timeout_sec_;
  }

  void publishTerrainStatus(const bool valid)
  {
    if (terrain_valid_publisher_ == nullptr ||
      (have_published_terrain_status_ && terrain_status_published_ == valid))
    {
      return;
    }
    std_msgs::msg::Bool message;
    message.data = valid;
    terrain_valid_publisher_->publish(message);
    terrain_status_published_ = valid;
    have_published_terrain_status_ = true;
  }

  void updateTerrainStatus()
  {
    const bool valid = bringup::navigationGateTerrainIsReady(
      true, terrain_status_message_valid_, terrainStatusAgeSeconds(), terrain_timeout_sec_);
    publishTerrainStatus(valid);
  }

  void beginFastlioHealthLoss(const std::string & reason)
  {
    if (!fastlio_health_loss_pending_) {
      fastlio_health_loss_pending_ = true;
      fastlio_health_loss_since_ = std::chrono::steady_clock::now();
      RCLCPP_WARN(
        get_logger(), "FAST-LIO health lost; holding navigation goal for %.2f s: %s",
        fastlio_health_loss_grace_sec_, reason.c_str());
    }
    if (fastlio_health_loss_grace_sec_ == 0.0 && desired_active_) {
      loseLocalization(reason, "fastlio_health_loss");
    }
  }

  bool fastlioHealthLossExpired() const
  {
    if (!fastlio_health_loss_pending_) {
      return false;
    }
    const double unhealthy_duration = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - fastlio_health_loss_since_).count();
    return unhealthy_duration >= fastlio_health_loss_grace_sec_;
  }

  void handleFastlioHealthLossDeadline()
  {
    if (require_fastlio_health_ && desired_active_ && fastlioHealthLossExpired()) {
      loseLocalization(
        "FAST-LIO health remained false beyond the grace period", "fastlio_health_loss");
    }
  }

  void setDesiredActive(const bool active)
  {
    if (desired_active_ == active) {
      return;
    }
    desired_active_ = active;
    if (active) {
      last_navigation_loss_reason_.clear();
      last_navigation_pause_cause_.clear();
      pause_reason_reported_ = false;
    }
    lifecycle_retry_failures_ = 0;
    next_lifecycle_retry_ = std::chrono::steady_clock::now();
  }

  void reconcileLocalizationReadiness(
    const std::string & reason, const std::string & cause)
  {
    if (desired_active_ && localization_baseline_ready_ && require_fastlio_health_ &&
      !fastlioHealthFresh() && !fastlio_health_loss_pending_)
    {
      beginFastlioHealthLoss("FAST-LIO health is false or stale");
    }
    handleFastlioHealthLossDeadline();

    const double age = have_fastlio_health_ ? std::chrono::duration<double>(
      std::chrono::steady_clock::now() - last_fastlio_health_).count() : 0.0;
    const bool core_inputs_ready = bringup::navigationGateShouldBeActive(
      localization_baseline_ready_, require_fastlio_health_, have_fastlio_health_, fastlio_healthy_,
      age, fastlio_health_timeout_sec_);
    const bool terrain_ready = terrainReady();
    if (terrain_ready) {
      terrain_recovery_probe_active_ = false;
      terrain_recovery_probe_failures_ = 0;
      next_terrain_recovery_probe_ = std::chrono::steady_clock::now();
    }
    const auto now = std::chrono::steady_clock::now();
    const bool lifecycle_not_active = !navigation_started_ || navigation_paused_;
    // Cleanup may remove the TerrainLayer publisher. A bounded lifecycle probe
    // recreates it while navigation readiness remains false until fresh status arrives.
    if (bringup::navigationGateShouldStartTerrainRecoveryProbe(
        require_terrain_status_, terrain_ready, navigation_ever_started_,
        lifecycle_not_active, !terrainStatusFresh(), terrain_recovery_probe_active_,
        now >= next_terrain_recovery_probe_))
    {
      terrain_recovery_probe_active_ = true;
      terrain_recovery_probe_started_ = now;
    }
    if (terrain_recovery_probe_active_ && !terrain_ready) {
      const double probe_age = std::chrono::duration<double>(
        now - terrain_recovery_probe_started_).count();
      if (probe_age > terrain_timeout_sec_) {
        terrain_recovery_probe_active_ = false;
        const double retry_delay = scheduleTerrainRecoveryProbeRetry(now);
        RCLCPP_WARN(
          get_logger(),
          "Terrain recovery probe expired without valid data; next probe in %.1f s "
          "(failure %zu)",
          retry_delay, terrain_recovery_probe_failures_);
      }
    }
    const bool terrain_allows_lifecycle = bringup::navigationGateTerrainAllowsLifecycle(
      require_terrain_status_, terrain_ready, !navigation_ever_started_,
      terrain_recovery_probe_active_);
    const bool should_activate = core_inputs_ready && terrain_allows_lifecycle;
    if (navigation_pause_request_active_) {
      last_navigation_pause_cause_ = "localization_rebaseline";
      last_navigation_loss_reason_ = "awaiting accepted localization baseline";
      setDesiredActive(false);
      if (navigation_ready_) {
        navigation_ready_ = false;
        publishReady(false);
      }
      reconcileLifecycle();
      return;
    }
    if (should_activate) {
      setDesiredActive(true);
      reconcileLifecycle();
    } else if (bringup::navigationGateKeepsLifecycleDuringFastlioGrace(
        desired_active_, localization_baseline_ready_, fastlio_health_loss_pending_,
        terrainReady()))
    {
      // Keep Nav2 active and preserve its goal while FAST-LIO alone is in grace.
      reconcileLifecycle();
    } else if (desired_active_) {
      loseLocalization(reason, cause);
    } else {
      reconcileLifecycle();
    }
  }

  void publishReady(const bool ready)
  {
    std_msgs::msg::Bool message;
    message.data = ready;
    ready_publisher_->publish(message);
    last_navigation_ready_publish_ = std::chrono::steady_clock::now();
  }

  void publishNavigationReadyHeartbeatIfDue()
  {
    const auto now = std::chrono::steady_clock::now();
    if (now - last_navigation_ready_publish_ >=
      bringup::kNavigationReadyHeartbeatPeriod)
    {
      publishReady(navigation_ready_);
    }
  }

  void publishNavigationPauseStatus()
  {
    if (!navigation_pause_status_publisher_) {
      return;
    }
    small_gicp_relocalization::msg::NavigationPauseStatus status;
    status.gate_session_id = gate_session_id_;
    if (navigation_pause_request_active_) {
      status.handoff_session_id = active_pause_handoff_session_id_;
      status.request_id = active_pause_request_id_;
      status.producer_session_id = active_pause_producer_session_id_;
      status.initialization_generation = active_pause_initialization_generation_;
      status.localization_epoch = active_pause_localization_epoch_;
      status.confirmed = lifecycle_manager_available_ && actual_state_known_ &&
        !desired_active_ && all_nodes_inactive_ && pending_command_ < 0;
    }
    navigation_pause_status_publisher_->publish(status);
    last_navigation_pause_status_publish_ = std::chrono::steady_clock::now();
  }

  void navigationPauseRequestCallback(
    const small_gicp_relocalization::msg::NavigationPauseRequest::SharedPtr message)
  {
    if (message->handoff_session_id == 0 || message->request_id == 0 ||
      message->producer_session_id == 0 || message->initialization_generation == 0 ||
      message->localization_epoch == 0)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000, "Ignoring navigation pause request with zero identity");
      return;
    }

    const bool same_request = navigation_pause_request_active_ &&
      message->handoff_session_id == active_pause_handoff_session_id_ &&
      message->request_id == active_pause_request_id_ &&
      message->producer_session_id == active_pause_producer_session_id_ &&
      message->initialization_generation == active_pause_initialization_generation_ &&
      message->localization_epoch == active_pause_localization_epoch_;
    if (message->pause_requested) {
      const bool older_request = message->handoff_session_id < last_pause_handoff_session_id_ ||
        (message->handoff_session_id == last_pause_handoff_session_id_ &&
        message->request_id < last_pause_request_id_);
      const bool reused_request_id =
        message->handoff_session_id == last_pause_handoff_session_id_ &&
        message->request_id == last_pause_request_id_ && !same_request;
      if (older_request || reused_request_id)
      {
        return;
      }
      if (!same_request) {
        navigation_pause_request_active_ = true;
        active_pause_handoff_session_id_ = message->handoff_session_id;
        active_pause_request_id_ = message->request_id;
        active_pause_producer_session_id_ = message->producer_session_id;
        active_pause_initialization_generation_ = message->initialization_generation;
        active_pause_localization_epoch_ = message->localization_epoch;
        last_pause_handoff_session_id_ = message->handoff_session_id;
        last_pause_request_id_ = message->request_id;
        actual_state_known_ = false;
        all_nodes_active_ = false;
        all_nodes_inactive_ = false;
        for (const auto & pending : pending_state_requests_) {
          pending.client->remove_pending_request(pending.request_id);
        }
        pending_state_requests_.clear();
        lifecycle_state_query_pending_ = false;
        pending_state_responses_ = 0;
        ++lifecycle_state_query_generation_;
        next_state_query_ = std::chrono::steady_clock::now();
        last_navigation_loss_reason_ = "awaiting accepted localization baseline";
        pause_reason_reported_ = false;
        RCLCPP_WARN(
          get_logger(), "Handoff requested Nav2 pause session=%" PRIu64
          " request=%" PRIu64 " localization=(%" PRIu64 ",%" PRIu64 ",%" PRIu64 ")",
          active_pause_handoff_session_id_, active_pause_request_id_,
          active_pause_producer_session_id_, active_pause_initialization_generation_,
          active_pause_localization_epoch_);
      }
      setDesiredActive(false);
      if (navigation_ready_) {
        navigation_ready_ = false;
        publishReady(false);
      }
      cancelNavigationGoals();
      publishNavigationPauseStatus();
      reconcileLocalizationReadiness(
        "handoff requested lifecycle-confirmed Nav2 pause", "localization_rebaseline");
      return;
    }

    if (!same_request) {
      return;
    }
    navigation_pause_request_active_ = false;
    RCLCPP_INFO(
      get_logger(), "Handoff released Nav2 pause session=%" PRIu64 " request=%" PRIu64
      " after applying the localization baseline",
      active_pause_handoff_session_id_, active_pause_request_id_);
    publishNavigationPauseStatus();
    reconcileLocalizationReadiness(
      "handoff applied localization baseline and released pause", "localization_ready");
  }

  void publishNavigationPauseState()
  {
    const bool confirmed_paused = lifecycle_manager_available_ && actual_state_known_ &&
      !desired_active_ && all_nodes_inactive_ && pending_command_ < 0;
    const bool changed = confirmed_paused != navigation_pause_confirmed_;
    if (changed) {
      navigation_pause_confirmed_ = confirmed_paused;
      std_msgs::msg::Bool message;
      message.data = confirmed_paused;
      navigation_paused_publisher_->publish(message);
      RCLCPP_INFO(
        get_logger(), "Navigation pause confirmation changed: %s",
        confirmed_paused ? "confirmed" : "active or transitioning");
    }
    const bool request_status_due = navigation_pause_request_active_ &&
      std::chrono::duration<double>(
      std::chrono::steady_clock::now() - last_navigation_pause_status_publish_).count() >= 0.25;
    if (changed || request_status_due) {
      publishNavigationPauseStatus();
    }
  }

  void loseLocalization(const std::string & reason, const std::string & cause)
  {
    last_navigation_loss_reason_ = reason;
    last_navigation_pause_cause_ = cause;
    pause_reason_reported_ = false;
    setDesiredActive(false);
    if (navigation_ready_) {
      navigation_ready_ = false;
      publishReady(false);
    }
    cancelNavigationGoals();
    reconcileLifecycle();
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Navigation gate LOST: cause=%s reason=%s", cause.c_str(), reason.c_str());
  }

  void updateNavigationReady()
  {
    publishNavigationPauseState();
    const bool lifecycle_ready = bringup::navigationIsActuallyReady(
      desired_active_, lifecycle_manager_available_, actual_state_known_, all_nodes_active_,
      pending_command_ >= 0);
    // Health loss immediately withdraws the goal-dispatch permit so the goal proxy
    // stops its child action. The chassis safety gate independently applies its
    // bounded health-loss stop; this grace only keeps Nav2 lifecycle nodes available
    // while the same localization process tries to recover.
    const bool fastlio_ready = !require_fastlio_health_ || fastlioHealthFresh();
    const bool ready = lifecycle_ready &&
      fastlio_ready && terrainReady() && perceptionFresh();
    if (ready != navigation_ready_) {
      navigation_ready_ = ready;
      publishReady(ready);
      const auto system_now = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
      const double startup_elapsed_sec = startup_epoch_ns_ > 0 &&
        system_now >= startup_epoch_ns_ ?
        static_cast<double>(system_now - startup_epoch_ns_) / 1.0e9 :
        std::chrono::duration<double>(
          std::chrono::steady_clock::now() - gate_started_at_).count();
      RCLCPP_INFO(
        get_logger(), "Navigation readiness changed: %s (desired_active=%s manager=%s "
        "state_known=%s all_active=%s command_pending=%s terrain_ready=%s "
        "perception_ready=%s startup_elapsed=%.3f s)",
        ready ? "ready" : "not ready", desired_active_ ? "true" : "false",
        lifecycle_manager_available_ ? "true" : "false", actual_state_known_ ? "true" : "false",
        all_nodes_active_ ? "true" : "false", pending_command_ >= 0 ? "true" : "false",
        terrainReady() ? "true" : "false", perceptionFresh() ? "true" : "false",
        startup_elapsed_sec);
    }
  }

  void reconcileLifecycle()
  {
    updateNavigationReady();
    if (!lifecycle_manager_available_ || !actual_state_known_ || lifecycle_state_query_pending_) {
      return;
    }
    const bool must_pause_observed_active_nodes =
      !desired_active_ && !all_nodes_inactive_;
    const auto action = bringup::reconcileLifecycle(
      desired_active_, navigation_started_ || must_pause_observed_active_nodes,
      navigation_paused_ && !must_pause_observed_active_nodes, pending_command_ >= 0);
    if (action == bringup::LifecycleReconcileAction::NONE) {
      return;
    }
    uint8_t command = Lifecycle::Request::STARTUP;
    if (action == bringup::LifecycleReconcileAction::PAUSE) {
      command = Lifecycle::Request::PAUSE;
      if (!pause_reason_reported_) {
        const auto & reason = last_navigation_loss_reason_.empty()
          ? "navigation inputs are not ready" : last_navigation_loss_reason_;
        const auto & cause = last_navigation_pause_cause_.empty()
          ? "navigation_inputs_unavailable" : last_navigation_pause_cause_;
        RCLCPP_WARN(
          get_logger(), "Navigation gate PAUSED: cause=%s reason=%s",
          cause.c_str(), reason.c_str());
        pause_reason_reported_ = true;
      }
    } else if (action == bringup::LifecycleReconcileAction::RESUME) {
      command = Lifecycle::Request::RESUME;
    }
    requestLifecycle(command);
  }

  void requestLifecycle(const uint8_t command)
  {
    if (!lifecycle_manager_available_ || pending_command_ >= 0) {
      return;
    }
    const auto now = std::chrono::steady_clock::now();
    const auto retry_period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(lifecycle_retry_period_sec_));
    if (now < next_lifecycle_retry_) {
      return;
    }
    if (!lifecycle_client_->service_is_ready()) {
      observeLifecycleManager();
      next_lifecycle_retry_ = now + retry_period;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Lifecycle manager service %s is not ready; retrying in %.1f s",
        lifecycle_service_.c_str(), lifecycle_retry_period_sec_);
      return;
    }
    auto request = std::make_shared<Lifecycle::Request>();
    request->command = command;
    pending_command_ = command;
    const auto request_generation = ++lifecycle_request_generation_;
    pending_request_sent_at_ = now;
    if (command == Lifecycle::Request::STARTUP) {
      RCLCPP_INFO(
        get_logger(),
        "Relocalization gate opening Nav2 lifecycle; Nav2 activation started (command=%u)",
        command);
    } else if (command == Lifecycle::Request::RESUME) {
      RCLCPP_INFO(
        get_logger(), "Relocalization gate resuming Nav2 lifecycle (command=%u)", command);
    } else {
      RCLCPP_INFO(
        get_logger(), "Relocalization gate pausing Nav2 lifecycle (command=%u)", command);
    }
    const auto future_and_request_id = lifecycle_client_->async_send_request(
      request, [this, command, request_generation](rclcpp::Client<Lifecycle>::SharedFuture future) {
        if (request_generation != lifecycle_request_generation_ || pending_command_ != command) {
          return;
        }
        pending_command_ = -1;
        pending_request_id_ = 0;
        if (!future.get()->success) {
          const auto retry_delay = scheduleLifecycleRetry();
          resetObservedLifecycleState("lifecycle command failed", false);
          RCLCPP_ERROR(
            get_logger(), "Navigation lifecycle command %u failed; retrying in %.1f s",
            command, retry_delay);
          return;
        }
        lifecycle_retry_failures_ = 0;
        next_lifecycle_retry_ = std::chrono::steady_clock::now();
        resetObservedLifecycleState("lifecycle command completed", false);
        next_state_query_ = std::chrono::steady_clock::now();
        reconcileLifecycle();
      });
    pending_request_id_ = future_and_request_id.request_id;
    next_lifecycle_retry_ = now + retry_period;
  }

  double retryBackoffDelay(const std::size_t failures) const
  {
    const auto exponent = std::min<std::size_t>(failures, 16);
    const double multiplier = static_cast<double>(std::uint64_t{1} << exponent);
    return std::min(
      lifecycle_retry_period_sec_ * multiplier, lifecycle_retry_backoff_max_sec_);
  }

  double scheduleLifecycleRetry()
  {
    const double delay = retryBackoffDelay(lifecycle_retry_failures_);
    if (lifecycle_retry_failures_ < 16) {
      ++lifecycle_retry_failures_;
    }
    next_lifecycle_retry_ = std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(delay));
    return delay;
  }

  double scheduleTerrainRecoveryProbeRetry(
    const std::chrono::steady_clock::time_point now)
  {
    const double delay = retryBackoffDelay(terrain_recovery_probe_failures_);
    if (terrain_recovery_probe_failures_ < 16) {
      ++terrain_recovery_probe_failures_;
    }
    next_terrain_recovery_probe_ = now +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(delay));
    return delay;
  }

  std::chrono::steady_clock::duration statePollPeriod() const
  {
    return std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(lifecycle_state_poll_period_sec_));
  }

  void resetObservedLifecycleState(const std::string & reason, const bool apply_retry_backoff)
  {
    const auto now = std::chrono::steady_clock::now();
    if (pending_command_ >= 0 && pending_request_id_ != 0) {
      lifecycle_client_->remove_pending_request(pending_request_id_);
    }
    pending_command_ = -1;
    pending_request_id_ = 0;
    ++lifecycle_request_generation_;
    for (const auto & pending : pending_state_requests_) {
      pending.client->remove_pending_request(pending.request_id);
    }
    pending_state_requests_.clear();
    lifecycle_state_query_pending_ = false;
    ++lifecycle_state_query_generation_;
    actual_state_known_ = false;
    all_nodes_active_ = false;
    all_nodes_inactive_ = false;
    navigation_started_ = false;
    navigation_paused_ = false;
    next_state_query_ = now + statePollPeriod();
    if (apply_retry_backoff) {
      scheduleLifecycleRetry();
    }
    if (navigation_ready_) {
      navigation_ready_ = false;
      publishReady(false);
    }
    publishNavigationPauseState();
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000, "Clearing observed navigation lifecycle state: %s",
      reason.c_str());
  }

  void observeLifecycleManager()
  {
    const bool available = lifecycle_client_->service_is_ready();
    if (!available) {
      if (lifecycle_manager_available_) {
        resetObservedLifecycleState("lifecycle manager service disappeared", true);
      }
      lifecycle_manager_available_ = false;
      updateNavigationReady();
      return;
    }
    if (!lifecycle_manager_available_) {
      lifecycle_manager_available_ = true;
      resetObservedLifecycleState("lifecycle manager service connected", false);
      next_state_query_ = std::chrono::steady_clock::now();
    }
  }

  void checkLifecycleRequestTimeout()
  {
    if (pending_command_ < 0) {
      return;
    }
    const double elapsed = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - pending_request_sent_at_).count();
    if (!bringup::lifecycleRequestExpired(
        true, elapsed, lifecycle_request_timeout_sec_))
    {
      return;
    }
    if (pending_request_id_ != 0) {
      lifecycle_client_->remove_pending_request(pending_request_id_);
    }
    const bool was_ready = navigation_ready_;
    RCLCPP_ERROR(
      get_logger(), "Lifecycle command %d timed out after %.2f seconds",
      pending_command_, elapsed);
    resetObservedLifecycleState("lifecycle command response timeout", true);
    if (!was_ready) {
      publishReady(false);
    }
  }

  void startLifecycleStateQuery()
  {
    if (!lifecycle_manager_available_ || lifecycle_state_query_pending_) {
      return;
    }
    const auto now = std::chrono::steady_clock::now();
    if (actual_state_known_ && now < next_state_query_) {
      return;
    }
    for (const auto & client : lifecycle_state_clients_) {
      if (!client->service_is_ready()) {
        resetObservedLifecycleState("managed lifecycle state service is not ready", false);
        return;
      }
    }

    lifecycle_state_query_pending_ = true;
    const auto query_generation = ++lifecycle_state_query_generation_;
    pending_state_responses_ = lifecycle_state_clients_.size();
    observed_state_ids_.assign(lifecycle_state_clients_.size(), 0);
    pending_state_requests_.clear();
    for (std::size_t index = 0; index < lifecycle_state_clients_.size(); ++index) {
      auto request = std::make_shared<LifecycleState::Request>();
      const auto client = lifecycle_state_clients_[index];
      const auto future_and_request_id = client->async_send_request(
        request, [this, query_generation, index](LifecycleStateClient::SharedFuture future) {
          if (!lifecycle_state_query_pending_ || query_generation != lifecycle_state_query_generation_) {
            return;
          }
          observed_state_ids_[index] = future.get()->current_state.id;
          if (pending_state_responses_ == 0) {
            return;
          }
          --pending_state_responses_;
          if (pending_state_responses_ == 0) {
            finishLifecycleStateQuery();
          }
        });
      pending_state_requests_.push_back({client, future_and_request_id.request_id});
    }
    state_query_started_at_ = now;
  }

  void finishLifecycleStateQuery()
  {
    lifecycle_state_query_pending_ = false;
    pending_state_requests_.clear();
    next_state_query_ = std::chrono::steady_clock::now() + statePollPeriod();
    const auto summary = bringup::summarizeLifecycleStates(
      observed_state_ids_, lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
    actual_state_known_ = summary.known;
    all_nodes_active_ = summary.known && summary.all_active;
    all_nodes_inactive_ = summary.known && summary.all_inactive;
    navigation_ever_started_ = navigation_ever_started_ ||
      (summary.known && summary.all_active);
    const bool any_node_active = std::find(
      observed_state_ids_.begin(), observed_state_ids_.end(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) != observed_state_ids_.end();
    if (desired_active_) {
      navigation_started_ = summary.known && !summary.any_unconfigured;
      navigation_paused_ = summary.known && !summary.all_active;
    } else {
      navigation_started_ = summary.known && (!summary.any_unconfigured || any_node_active);
      navigation_paused_ = summary.known && summary.all_inactive;
    }
    if (!actual_state_known_) {
      navigation_started_ = false;
      navigation_paused_ = false;
    }
    reconcileLifecycle();
  }

  void checkLifecycleStateQueryTimeout()
  {
    if (!lifecycle_state_query_pending_) {
      return;
    }
    const double elapsed = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - state_query_started_at_).count();
    if (!bringup::lifecycleRequestExpired(
        true, elapsed, lifecycle_request_timeout_sec_))
    {
      return;
    }
    resetObservedLifecycleState("managed lifecycle state query timeout", true);
  }

  void cancelNavigationGoals()
  {
    const auto cancel = [](const rclcpp::Client<action_msgs::srv::CancelGoal>::SharedPtr & client) {
      if (!client->service_is_ready()) return;
      auto request = std::make_shared<action_msgs::srv::CancelGoal::Request>();
      client->async_send_request(request);
    };
    cancel(navigate_through_poses_cancel_client_);
  }

  void logOdometryDiagnostics()
  {
    const auto now = std::chrono::steady_clock::now();
    if (last_odom_diagnostic_.time_since_epoch().count() != 0 &&
      now - last_odom_diagnostic_ < std::chrono::seconds(5))
    {
      return;
    }
    last_odom_diagnostic_ = now;
    const double receive_age = have_odom_ ? std::chrono::duration<double>(
      now - last_odom_receive_).count() : std::numeric_limits<double>::infinity();
    double source_age = std::numeric_limits<double>::infinity();
    if (have_odom_) {
      source_age = (this->now() - rclcpp::Time(
        last_odom_stamp_, this->get_clock()->get_clock_type())).seconds();
    }
    const bool fresh = have_odom_ && receive_age >= 0.0 &&
      receive_age <= odom_freshness_timeout_sec_ && source_age >= 0.0 &&
      source_age <= odom_freshness_timeout_sec_;
    const double perception_receive_age = have_perception_message_ ?
      std::chrono::duration<double>(now - last_perception_receive_).count() :
      std::numeric_limits<double>::infinity();
    const double perception_source_age = have_perception_message_ ?
      (this->now() - rclcpp::Time(
        last_perception_stamp_, this->get_clock()->get_clock_type())).seconds() :
      std::numeric_limits<double>::infinity();
    const bool perception_fresh = perceptionFresh();
    RCLCPP_INFO(
      get_logger(),
      "Navigation input timing: odom_topic=%s messages_5s=%zu source_age=%.3f s "
      "receive_age=%.3f s freshness_limit=%.3f s fresh=%s perception_topic=%s "
      "messages_5s=%zu source_age=%.3f s receive_age=%.3f s freshness_limit=%.3f s "
      "required=%s fresh=%s",
      odom_topic_.c_str(), odom_messages_since_report_, source_age, receive_age,
      odom_freshness_timeout_sec_, fresh ? "true" : "false", perception_topic_.c_str(),
      perception_messages_since_report_, perception_source_age, perception_receive_age,
      perception_timeout_sec_, require_perception_ ? "true" : "false",
      perception_fresh ? "true" : "false");
    odom_messages_since_report_ = 0;
    perception_messages_since_report_ = 0;
  }

  std::string lifecycle_service_;
  std::string navigation_ready_topic_;
  std::string odom_topic_;
  std::int64_t startup_epoch_ns_{0};
  std::chrono::steady_clock::time_point gate_started_at_{std::chrono::steady_clock::now()};
  double odom_freshness_timeout_sec_{0.4};
  bool require_perception_{true};
  std::string perception_topic_;
  double perception_timeout_sec_{1.5};
  std::string navigation_paused_topic_;
  std::string navigation_pause_request_topic_;
  std::string navigation_pause_status_topic_;
  std::uint64_t gate_session_id_{0};
  std::uint64_t active_pause_handoff_session_id_{0};
  std::uint64_t active_pause_request_id_{0};
  std::uint64_t active_pause_producer_session_id_{0};
  std::uint64_t active_pause_initialization_generation_{0};
  std::uint64_t active_pause_localization_epoch_{0};
  std::uint64_t last_pause_handoff_session_id_{0};
  std::uint64_t last_pause_request_id_{0};
  double status_timeout_sec_{1.0};
  bool require_fastlio_health_{false};
  std::string fastlio_health_topic_;
  bool fastlio_health_is_localization_signal_{false};
  double fastlio_health_timeout_sec_{0.5};
  double fastlio_health_loss_grace_sec_{1.25};
  bool fastlio_health_loss_pending_{false};
  std::chrono::steady_clock::time_point fastlio_health_loss_since_{};
  bool localization_baseline_ready_{false};
  bool localization_status_ready_{false};
  bool handoff_ready_{false};
  bool have_fastlio_health_{false};
  bool fastlio_healthy_{false};
  std::chrono::steady_clock::time_point last_fastlio_health_{};
  bool require_terrain_status_{false};
  std::string terrain_status_topic_;
  double terrain_timeout_sec_{1.2};
  std::string terrain_valid_topic_;
  bool have_terrain_status_{false};
  bool terrain_status_message_valid_{false};
  std::chrono::steady_clock::time_point last_terrain_status_{};
  bool have_published_terrain_status_{false};
  bool terrain_status_published_{false};
  double lifecycle_retry_period_sec_{1.0};
  double lifecycle_retry_backoff_max_sec_{30.0};
  std::size_t lifecycle_retry_failures_{0};
  double lifecycle_request_timeout_sec_{10.0};
  double lifecycle_state_poll_period_sec_{0.5};
  std::vector<std::string> managed_nodes_;
  bool desired_active_{false};
  std::string last_navigation_loss_reason_;
  std::string last_navigation_pause_cause_;
  bool pause_reason_reported_{false};
  bool navigation_started_{false};
  bool navigation_paused_{false};
  bool all_nodes_inactive_{false};
  bool navigation_ever_started_{false};
  bool terrain_recovery_probe_active_{false};
  std::chrono::steady_clock::time_point terrain_recovery_probe_started_{};
  std::chrono::steady_clock::time_point next_terrain_recovery_probe_{};
  std::size_t terrain_recovery_probe_failures_{0};
  bool navigation_ready_{false};
  bool navigation_pause_confirmed_{false};
  bool navigation_pause_request_active_{false};
  bool lifecycle_manager_available_{false};
  bool actual_state_known_{false};
  bool all_nodes_active_{false};
  bool lifecycle_state_query_pending_{false};
  int pending_command_{-1};
  std::int64_t pending_request_id_{0};
  std::uint64_t lifecycle_request_generation_{0};
  std::uint64_t lifecycle_state_query_generation_{0};
  std::size_t pending_state_responses_{0};
  std::vector<std::uint8_t> observed_state_ids_;
  std::vector<PendingStateRequest> pending_state_requests_;
  bool have_status_heartbeat_{false};
  std::chrono::steady_clock::time_point last_status_heartbeat_{};
  std::chrono::steady_clock::time_point last_navigation_pause_status_publish_{};
  std::chrono::steady_clock::time_point last_navigation_ready_publish_{};
  std::chrono::steady_clock::time_point last_odom_receive_{};
  std::chrono::steady_clock::time_point last_odom_diagnostic_{};
  std::chrono::steady_clock::time_point last_perception_receive_{};
  builtin_interfaces::msg::Time last_odom_stamp_{};
  builtin_interfaces::msg::Time last_perception_stamp_{};
  std::size_t odom_messages_since_report_{0};
  std::size_t perception_messages_since_report_{0};
  bool have_odom_{false};
  bool have_perception_message_{false};
  std::chrono::steady_clock::time_point next_lifecycle_retry_{};
  std::chrono::steady_clock::time_point pending_request_sent_at_{};
  std::chrono::steady_clock::time_point state_query_started_at_{};
  std::chrono::steady_clock::time_point next_state_query_{};
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr navigation_paused_publisher_;
  rclcpp::Publisher<
    small_gicp_relocalization::msg::NavigationPauseStatus>::SharedPtr
    navigation_pause_status_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr terrain_valid_publisher_;
  rclcpp::Subscription<small_gicp_relocalization::msg::RelocalizationStatus>::SharedPtr status_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr handoff_subscription_;
  rclcpp::Subscription<
    small_gicp_relocalization::msg::NavigationPauseRequest>::SharedPtr
    navigation_pause_request_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr fastlio_health_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr terrain_status_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr perception_subscription_;
  rclcpp::Client<Lifecycle>::SharedPtr lifecycle_client_;
  std::vector<LifecycleStateClient::SharedPtr> lifecycle_state_clients_;
  rclcpp::Client<action_msgs::srv::CancelGoal>::SharedPtr navigate_through_poses_cancel_client_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<NavigationGateNode>());
  } catch (const std::exception & exception) {
    RCLCPP_ERROR(rclcpp::get_logger("navigation_gate"), "%s", exception.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
