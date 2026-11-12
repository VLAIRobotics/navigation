#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <array>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <map_msgs/msg/occupancy_grid_update.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int64_multi_array.hpp>
#include <tf2/time.hpp>
#include <tf2/utils.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

#include "bringup/online_goal_route.hpp"
#include "bringup/online_map_state.hpp"

namespace bringup
{

class NavigationGoalProxy : public rclcpp::Node
{
public:
  using Action = nav2_msgs::action::NavigateToPose;
  using PublicGoalHandle = rclcpp_action::ServerGoalHandle<Action>;
  using InternalGoalHandle = rclcpp_action::ClientGoalHandle<Action>;

  explicit NavigationGoalProxy(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("navigation_goal_proxy", options)
  {
    const auto public_action = declare_parameter<std::string>(
      "public_action", "/navigate_to_pose");
    const auto internal_action = declare_parameter<std::string>(
      "internal_action", "/navigate_to_pose_internal");
    const auto ready_topic = declare_parameter<std::string>(
      "navigation_ready_topic", "/navigation_ready");
    const auto goal_pose_topic = declare_parameter<std::string>("goal_pose_topic", "/goal_pose");
    global_frame_ = declare_parameter<std::string>("global_frame", "map");
    manage_goal_routes_ = declare_parameter<bool>("manage_goal_routes", false);
    map_topic_ = declare_parameter<std::string>("map_topic", "/map");
    map_updates_topic_ = declare_parameter<std::string>("map_updates_topic", "/map_updates");
    map_geometry_events_topic_ = declare_parameter<std::string>(
      "map_geometry_events_topic", "/map_geometry_events");
    costmap_topic_ = declare_parameter<std::string>("costmap_topic", "/global_costmap/costmap");
    costmap_updates_topic_ = declare_parameter<std::string>(
      "costmap_updates_topic", "/global_costmap/costmap_updates");
    observation_topic_ = declare_parameter<std::string>(
      "observation_topic", "/cloud_registered_body");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    segment_length_ = declare_parameter<double>("segment_length", 4.0);
    observation_wait_timeout_ = declare_parameter<double>("observation_wait_timeout", 3.0);
    route_retry_period_ = declare_parameter<double>("route_retry_period", 1.0);
    max_frontier_attempts_ = declare_parameter<int>("max_frontier_attempts", 6);
    max_route_expansions_ = declare_parameter<int>("max_route_expansions", 500000);
    route_goal_tolerance_ = declare_parameter<double>("route_goal_tolerance", 0.20);
    frontier_exclusion_radius_ = declare_parameter<double>("frontier_exclusion_radius", 0.80);
    start_unknown_escape_radius_ = declare_parameter<double>(
      "start_unknown_escape_radius", 0.50);
    route_footprint_ = declare_parameter<std::vector<double>>(
      "robot_footprint", {0.275, 0.225, 0.275, -0.225,
        -0.35, -0.225, -0.35, 0.225});
    route_data_wait_timeout_ = declare_parameter<double>("route_data_wait_timeout", 5.0);
    max_no_route_progress_updates_ = declare_parameter<int>(
      "max_no_route_progress_updates", 6);
    if (public_action.empty() || internal_action.empty() || ready_topic.empty() ||
      global_frame_.empty() || public_action == internal_action || map_topic_.empty() ||
      map_updates_topic_.empty() || map_geometry_events_topic_.empty() ||
      costmap_topic_.empty() || costmap_updates_topic_.empty() ||
      observation_topic_.empty() || base_frame_.empty() || !std::isfinite(segment_length_) ||
      segment_length_ < 1.0 || segment_length_ > 10.0 ||
      !std::isfinite(observation_wait_timeout_) || observation_wait_timeout_ <= 0.0 ||
      !std::isfinite(route_retry_period_) || route_retry_period_ <= 0.0 ||
      !std::isfinite(route_data_wait_timeout_) || route_data_wait_timeout_ <= 0.0 ||
      max_frontier_attempts_ < 1 || max_no_route_progress_updates_ < 1 ||
      max_route_expansions_ < 1000 ||
      !std::isfinite(route_goal_tolerance_) || route_goal_tolerance_ <= 0.0 ||
      !std::isfinite(frontier_exclusion_radius_) || frontier_exclusion_radius_ <= 0.0 ||
      !std::isfinite(start_unknown_escape_radius_) ||
      start_unknown_escape_radius_ < 0.0 || start_unknown_escape_radius_ > 1.0 ||
      route_footprint_.size() < 6 || route_footprint_.size() % 2 != 0)
    {
      throw std::invalid_argument("NavigationGoalProxy action names and ready topic are invalid");
    }
    for (const auto coordinate : route_footprint_) {
      if (!std::isfinite(coordinate)) {
        throw std::invalid_argument("NavigationGoalProxy robot_footprint is not finite");
      }
    }
    route_footprint_points_.reserve(route_footprint_.size() / 2);
    for (std::size_t i = 0; i < route_footprint_.size(); i += 2) {
      route_footprint_points_.push_back({route_footprint_[i], route_footprint_[i + 1]});
      route_footprint_radius_ = std::max(route_footprint_radius_, std::hypot(
          route_footprint_[i], route_footprint_[i + 1]));
    }

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    internal_client_ = rclcpp_action::create_client<Action>(this, internal_action);
    requested_goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/navigation/requested_goal", rclcpp::QoS(1).reliable().transient_local());
    current_waypoint_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/navigation/current_waypoint", rclcpp::QoS(1).reliable().transient_local());
    goal_status_pub_ = create_publisher<std_msgs::msg::String>(
      "/navigation/goal_manager_status", rclcpp::QoS(1).reliable().transient_local());
    goal_identity_pub_ = create_publisher<std_msgs::msg::String>(
      "/navigation/goal_identity", rclcpp::QoS(1).reliable().transient_local());
    goal_pose_identity_pub_ = create_publisher<std_msgs::msg::String>(
      "/navigation/goal_pose_identity", rclcpp::QoS(1).reliable().transient_local());
    ready_sub_ = create_subscription<std_msgs::msg::Bool>(
      ready_topic, rclcpp::QoS(1).reliable().transient_local(),
      [this](std_msgs::msg::Bool::ConstSharedPtr message) {readyCallback(message->data);});
    if (manage_goal_routes_) {
      if (!goal_pose_topic.empty()) {
        topic_goal_client_ = rclcpp_action::create_client<Action>(this, public_action);
        goal_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
          goal_pose_topic, rclcpp::QoS(10).reliable(),
          [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr message) {
            if (!message || !validGoal(*message)) {
              RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 3000, "Ignoring invalid /goal_pose input");
              return;
            }
            Action::Goal goal;
            goal.pose = *message;
            topic_goal_client_->async_send_goal(goal);
          });
      }
      map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        map_topic_, rclcpp::QoS(1).reliable().transient_local(),
        [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr message) {
          if (!bringup::online_goal_route::validGrid(*message) ||
            message->header.frame_id != global_frame_)
          {
            RCLCPP_WARN_THROTTLE(
              get_logger(), *get_clock(), 5000, "Ignoring invalid online navigation map");
            return;
          }
          online_map_state_.receiveFullMap(*message);
          map_revision_ = online_map_state_.revision();
        });
      map_updates_sub_ = create_subscription<map_msgs::msg::OccupancyGridUpdate>(
        map_updates_topic_, rclcpp::QoS(10).reliable(),
        [this](map_msgs::msg::OccupancyGridUpdate::ConstSharedPtr update) {
          if (!update) return;
          online_map_state_.receiveUpdate(*update);
          map_revision_ = online_map_state_.revision();
        });
      map_geometry_events_sub_ = create_subscription<std_msgs::msg::UInt64MultiArray>(
        map_geometry_events_topic_, rclcpp::QoS(64).reliable().transient_local(),
        [this](std_msgs::msg::UInt64MultiArray::ConstSharedPtr event) {
          if (!event) return;
          online_map_state_.receiveGeometryEvent(*event);
          map_revision_ = online_map_state_.revision();
        });
      costmap_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        costmap_topic_, rclcpp::QoS(1).reliable().transient_local(),
        [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr message) {
          if (!bringup::online_goal_route::validGrid(*message) ||
            message->header.frame_id != global_frame_)
          {
            RCLCPP_WARN_THROTTLE(
              get_logger(), *get_clock(), 5000, "Ignoring invalid global costmap snapshot");
            return;
          }
          global_costmap_ = *message;
          costmap_received_at_ = std::chrono::steady_clock::now();
          ++costmap_revision_;
          for (const auto & update : pending_costmap_updates_) {
            applyCostmapUpdateToGrid(update);
          }
          pending_costmap_updates_.clear();
        });
      costmap_updates_sub_ = create_subscription<map_msgs::msg::OccupancyGridUpdate>(
        costmap_updates_topic_, rclcpp::QoS(10).reliable(),
        [this](map_msgs::msg::OccupancyGridUpdate::ConstSharedPtr update) {
          if (!global_costmap_) {
            pending_costmap_updates_.push_back(*update);
            if (pending_costmap_updates_.size() > 32) pending_costmap_updates_.erase(
              pending_costmap_updates_.begin());
            return;
          }
          applyCostmapUpdateToGrid(*update);
        });
      observation_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        observation_topic_, rclcpp::QoS(1).best_effort(),
        [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr message) {
          if (!message || message->header.stamp.sec < 0 ||
            message->header.stamp.nanosec >= 1000000000U ||
            message->header.frame_id.empty() ||
            static_cast<std::uint64_t>(message->width) * message->height == 0U)
          {
            return;
          }
          const auto stamp = rclcpp::Time(message->header.stamp, get_clock()->get_clock_type());
          const auto stamp_ns = stamp.nanoseconds();
          const auto age = (now() - stamp).seconds();
          if (!std::isfinite(age) || age < -0.1 || age > 2.0) return;
          if (stamp_ns <= last_observation_stamp_ns_) return;
          last_observation_stamp_ns_ = stamp_ns;
          ++observation_count_;
        });
    }
    server_ = rclcpp_action::create_server<Action>(
      this, public_action,
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const Action::Goal> goal) {
        return goal && validGoal(goal->pose) ? rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE :
               rclcpp_action::GoalResponse::REJECT;
      },
      [this](const std::shared_ptr<PublicGoalHandle> goal) {
        return cancelCallback(goal);
      },
      [this](const std::shared_ptr<PublicGoalHandle> goal) {acceptedCallback(goal);});
    retry_timer_ = create_wall_timer(
      std::chrono::milliseconds(100), [this]() {
        retryPendingPublicFinalizations();
        dispatchWhenReady();
      });

    RCLCPP_INFO(
      get_logger(), "Navigation goal proxy active: public=%s internal=%s readiness=%s "
      "global_frame=%s start_unknown_escape_radius=%.2f m",
      public_action.c_str(), internal_action.c_str(), ready_topic.c_str(),
      global_frame_.c_str(), start_unknown_escape_radius_);
  }

private:
  struct ForwardState
  {
    std::uint64_t generation{0};
    Action::Goal goal;
    geometry_msgs::msg::PoseStamped dispatch_pose;
    std::shared_ptr<PublicGoalHandle> public_handle;
    std::shared_ptr<InternalGoalHandle> internal_handle;
    bool send_pending{false};
    bool cancel_requested{false};
    bool system_interrupted{false};
    bool waiting_for_pause_signal{false};
    bool waiting_for_replaced_goal{false};
    bool goal_normalized{false};
    bool transform_wait_started{false};
    bool public_terminalized{false};
    bool dispatch_is_final{false};
    bool dispatch_is_frontier{false};
    bool dispatch_prepared{false};
    bool waiting_for_observation{false};
    bool waiting_for_search_update{false};
    bool route_wait_started{false};
    std::size_t segment_number{0};
    std::size_t frontier_attempts{0};
    std::uint64_t frontier_map_revision{0};
    std::uint64_t frontier_observation_count{0};
    std::uint64_t search_wait_map_revision{0};
    std::uint64_t search_wait_costmap_revision{0};
    std::uint64_t search_wait_observation_count{0};
    bool search_wait_pose_valid{false};
    double search_wait_x{0.0};
    double search_wait_y{0.0};
    double search_wait_yaw{0.0};
    std::vector<std::pair<double, double>> frontier_unknown_cells_before;
    std::uint64_t route_wait_map_revision{0};
    std::uint64_t route_wait_observation_count{0};
    int no_route_progress_updates{0};
    std::vector<std::pair<double, double>> excluded_frontiers;
    double last_frontier_x{0.0};
    double last_frontier_y{0.0};
    std::string terminal_reason;
    std::string last_route_diagnostic;
    std::string source_goal_frame;
    rclcpp_action::ResultCode interrupted_result_code{rclcpp_action::ResultCode::UNKNOWN};
    std::shared_ptr<Action::Result> interrupted_result;
    std::chrono::steady_clock::time_point pause_signal_deadline{};
    std::chrono::steady_clock::time_point replaced_goal_deadline{};
    std::chrono::steady_clock::time_point transform_wait_last_status{};
    std::chrono::steady_clock::time_point observation_wait_deadline{};
    std::chrono::steady_clock::time_point route_wait_deadline{};
    std::chrono::steady_clock::time_point last_route_plan_at{};
  };

  enum class PublicTerminal
  {
    SUCCEEDED,
    ABORTED,
    CANCELED
  };

  struct PendingPublicFinalization
  {
    std::shared_ptr<ForwardState> state;
    PublicTerminal terminal{PublicTerminal::ABORTED};
    std::shared_ptr<Action::Result> result;
  };

  static bool validGoal(const geometry_msgs::msg::PoseStamped & pose)
  {
    const auto & position = pose.pose.position;
    const auto & orientation = pose.pose.orientation;
    const double quaternion_norm = orientation.x * orientation.x +
      orientation.y * orientation.y + orientation.z * orientation.z +
      orientation.w * orientation.w;
    return !pose.header.frame_id.empty() &&
      std::isfinite(position.x) && std::isfinite(position.y) &&
      std::isfinite(position.z) && std::isfinite(orientation.x) &&
      std::isfinite(orientation.y) && std::isfinite(orientation.z) &&
      std::isfinite(orientation.w) && quaternion_norm > 1.0e-12;
  }

  enum class RoutePlanState
  {
    READY,
    WAIT,
    ABORT
  };

  struct RoutePlan
  {
    RoutePlanState state{RoutePlanState::WAIT};
    geometry_msgs::msg::PoseStamped pose;
    bool is_final{false};
    bool is_frontier{false};
    std::string status;
    std::string reason;
  };

  geometry_msgs::msg::PoseStamped makePose(
    const double x, const double y, const double yaw) const
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = global_frame_;
    pose.header.stamp = now();
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    tf2::Quaternion orientation;
    orientation.setRPY(0.0, 0.0, yaw);
    pose.pose.orientation = tf2::toMsg(orientation);
    return pose;
  }

  static const char * footprintBlockerName(
    const bringup::online_goal_route::FootprintBlocker blocker)
  {
    using bringup::online_goal_route::FootprintBlocker;
    switch (blocker) {
      case FootprintBlocker::NONE: return "none";
      case FootprintBlocker::INVALID_GRID: return "invalid_grid";
      case FootprintBlocker::INVALID_ARGUMENT: return "invalid_argument";
      case FootprintBlocker::OUT_OF_BOUNDS: return "out_of_bounds";
      case FootprintBlocker::UNKNOWN: return "unknown";
      case FootprintBlocker::OCCUPIED: return "occupied";
    }
    return "unknown_status";
  }

  void logRouteDiagnostic(const std::string & diagnostic)
  {
    if (diagnostic.empty()) return;
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 3000, "online goal route preflight: %s",
      diagnostic.c_str());
  }

  RoutePlan waitForRouteProgress(
    const std::shared_ptr<ForwardState> & state, const std::string & status,
    const std::string & diagnostic)
  {
    if (!diagnostic.empty()) {
      state->last_route_diagnostic = diagnostic;
      logRouteDiagnostic(diagnostic);
    }
    const auto steady_now = std::chrono::steady_clock::now();
    if (!state->route_wait_started) {
      state->route_wait_started = true;
      state->route_wait_deadline = steady_now +
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(route_data_wait_timeout_));
      state->route_wait_map_revision = map_revision_;
      state->route_wait_observation_count = observation_count_;
      return {RoutePlanState::WAIT, geometry_msgs::msg::PoseStamped{}, false, false, status, {}};
    }
    if (map_revision_ > state->route_wait_map_revision &&
      observation_count_ > state->route_wait_observation_count)
    {
      if (state->no_route_progress_updates < max_no_route_progress_updates_) {
        ++state->no_route_progress_updates;
      }
      state->route_wait_deadline = steady_now +
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(route_data_wait_timeout_));
      state->route_wait_map_revision = map_revision_;
      state->route_wait_observation_count = observation_count_;
      const auto wait_status = state->no_route_progress_updates >= max_no_route_progress_updates_ ?
        "地图和观测持续更新仍受阻；保留目标继续等待" : status;
      return {RoutePlanState::WAIT, geometry_msgs::msg::PoseStamped{}, false, false,
        wait_status, {}};
    }
    if (steady_now >= state->route_wait_deadline) {
      state->route_wait_deadline = steady_now +
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(route_data_wait_timeout_));
      state->route_wait_map_revision = map_revision_;
      state->route_wait_observation_count = observation_count_;
      return {RoutePlanState::WAIT, geometry_msgs::msg::PoseStamped{}, false, false,
        "受阻等待：尚无可达路线或新观测", {}};
    }
    return {RoutePlanState::WAIT, geometry_msgs::msg::PoseStamped{}, false, false, status, {}};
  }

  void waitForNextFrontierSearchRound(const std::shared_ptr<ForwardState> & state)
  {
    state->frontier_attempts = 0;
    state->waiting_for_search_update = true;
    state->search_wait_map_revision = map_revision_;
    state->search_wait_costmap_revision = costmap_revision_;
    state->search_wait_observation_count = observation_count_;
    state->search_wait_pose_valid = false;
    state->route_wait_started = false;
    state->last_route_plan_at = std::chrono::steady_clock::now();
  }

  RoutePlan handleFootprintBlocker(
    const std::shared_ptr<ForwardState> & state, const std::string & status,
    const bringup::online_goal_route::FootprintAssessment & assessment,
    const std::string & diagnostic)
  {
    using bringup::online_goal_route::FootprintBlocker;
    if (assessment.blocker == FootprintBlocker::UNKNOWN ||
      assessment.blocker == FootprintBlocker::OCCUPIED ||
      assessment.blocker == FootprintBlocker::OUT_OF_BOUNDS)
    {
      state->waiting_for_search_update = true;
      const auto wait_status = assessment.blocker == FootprintBlocker::OUT_OF_BOUNDS ?
        "当前地图覆盖不足；等待地图扩展后重新检查" : status;
      return waitForRouteProgress(state, wait_status, diagnostic);
    }
    return {RoutePlanState::ABORT, geometry_msgs::msg::PoseStamped{}, false, false, {},
      diagnostic};
  }

  RoutePlan prepareManagedDispatch(const std::shared_ptr<ForwardState> & state)
  {
    const auto steady_now = std::chrono::steady_clock::now();
    // Pause every map-dependent decision while a snapshot/delta is unmatched.
    // In particular, do not consume a frontier's observation timeout while the
    // map state is waiting for its geometry metadata or matching full map.
    const auto & online_map = online_map_state_.map();
    if (online_map_state_.synchronizationPending()) {
      state->route_wait_started = false;
      state->no_route_progress_updates = 0;
      std::ostringstream diagnostic;
      diagnostic << "branch=online_map_geometry_sync_pending map_revision=" << map_revision_
                 << " observation_count=" << observation_count_
                 << " map_present=" << static_cast<bool>(online_map);
      logRouteDiagnostic(diagnostic.str());
      return {RoutePlanState::WAIT, geometry_msgs::msg::PoseStamped{}, false, false,
        "等待在线地图与几何事件配对", {}};
    }
    if (state->waiting_for_observation) {
      if (online_map && map_revision_ > state->frontier_map_revision &&
        observation_count_ > state->frontier_observation_count &&
        bringup::online_goal_route::frontierObservationGained(
          *online_map, state->frontier_unknown_cells_before))
      {
        state->waiting_for_observation = false;
        state->route_wait_started = false;
        state->frontier_attempts = 0;
        state->excluded_frontiers.clear();
        state->last_route_plan_at = {};
      } else if (steady_now < state->observation_wait_deadline) {
        return {RoutePlanState::WAIT, geometry_msgs::msg::PoseStamped{}, false, false,
          "等待观测：检查已选前沿附近是否出现新地图证据", {}};
      } else {
        state->excluded_frontiers.emplace_back(state->last_frontier_x, state->last_frontier_y);
        ++state->frontier_attempts;
        state->waiting_for_observation = false;
        state->route_wait_started = false;
        state->last_route_plan_at = {};
        if (state->frontier_attempts >= static_cast<std::size_t>(max_frontier_attempts_)) {
          waitForNextFrontierSearchRound(state);
          return {RoutePlanState::WAIT, geometry_msgs::msg::PoseStamped{}, false, false,
            "本轮前沿尝试达到上限；等待新地图和新观测后继续搜索", {}};
        }
      }
    }
    // The session map is durable state: no map message is expected while an
    // already observed area remains unchanged.  Freshness is enforced on the
    // rolling costmap and on the live observation stream instead.
    const bool have_fresh_map = online_map &&
      bringup::online_goal_route::validGrid(*online_map);
    const bool have_fresh_costmap = global_costmap_ &&
      bringup::online_goal_route::validGrid(*global_costmap_) &&
      std::chrono::duration<double>(steady_now - costmap_received_at_).count() <= 3.0;
    if (!have_fresh_map || !have_fresh_costmap) {
      const double costmap_age = costmap_received_at_.time_since_epoch().count() == 0 ?
        -1.0 : std::chrono::duration<double>(steady_now - costmap_received_at_).count();
      std::ostringstream diagnostic;
      diagnostic << "branch=" << (!have_fresh_map ? "online_map_unavailable" :
        "global_costmap_unavailable")
                 << " map_valid=" << have_fresh_map
                 << " map_revision=" << map_revision_
                 << " observation_count=" << observation_count_
                 << " costmap_present=" << static_cast<bool>(global_costmap_)
                 << " costmap_valid=" << (global_costmap_ &&
        bringup::online_goal_route::validGrid(*global_costmap_))
                 << " costmap_age_s=" << costmap_age;
      if (global_costmap_ && bringup::online_goal_route::validGrid(*global_costmap_)) {
        diagnostic << " costmap=" << global_costmap_->info.width << 'x'
                   << global_costmap_->info.height << '@'
                   << global_costmap_->info.resolution;
      }
      const auto wait = waitForRouteProgress(
        state, "等待在线地图或滚动代价地图", diagnostic.str());
      return wait;
    }

    geometry_msgs::msg::TransformStamped robot_transform;
    try {
      robot_transform = tf_buffer_->lookupTransform(
        global_frame_, base_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException &) {
      std::ostringstream diagnostic;
      diagnostic << "branch=robot_tf_unavailable map_revision=" << map_revision_
                 << " observation_count=" << observation_count_
                 << " costmap_age_s=" << std::chrono::duration<double>(
        steady_now - costmap_received_at_).count();
      return waitForRouteProgress(
        state, "等待机器人位姿进入地图坐标系", diagnostic.str());
    }
    const double robot_x = robot_transform.transform.translation.x;
    const double robot_y = robot_transform.transform.translation.y;
    const double robot_yaw = tf2::getYaw(robot_transform.transform.rotation);
    if (state->waiting_for_search_update) {
      const bool map_changed = map_revision_ > state->search_wait_map_revision;
      const bool costmap_changed = costmap_revision_ > state->search_wait_costmap_revision;
      const bool observation_changed =
        observation_count_ > state->search_wait_observation_count;
      bool robot_pose_changed = false;
      if (state->search_wait_pose_valid) {
        const double yaw_delta = std::atan2(
          std::sin(robot_yaw - state->search_wait_yaw),
          std::cos(robot_yaw - state->search_wait_yaw));
        robot_pose_changed = std::hypot(
          robot_x - state->search_wait_x, robot_y - state->search_wait_y) >= 0.10 ||
          std::abs(yaw_delta) >= 0.10;
      } else {
        state->search_wait_pose_valid = true;
      }
      const auto retry_period =
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(route_retry_period_));
      const bool retry_due = steady_now - state->last_route_plan_at >= retry_period;
      const bool environment_changed = map_changed || costmap_changed ||
        observation_changed || robot_pose_changed;
      if (!retry_due) {
        return {RoutePlanState::WAIT, geometry_msgs::msg::PoseStamped{}, false, false,
          environment_changed ? "导航环境已变化；等待 1 Hz 路线复核" :
          "前沿候选本轮已用尽；等待地图、代价图、位姿变化或周期复核", {}};
      }
      state->waiting_for_search_update = false;
      state->last_route_plan_at = {};
      if (environment_changed) {
        state->frontier_attempts = 0;
        state->excluded_frontiers.clear();
        state->route_wait_started = false;
        state->no_route_progress_updates = 0;
      }
    }
    if (state->last_route_plan_at.time_since_epoch().count() != 0 &&
      steady_now - state->last_route_plan_at <
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(route_retry_period_)))
    {
      return {RoutePlanState::WAIT, geometry_msgs::msg::PoseStamped{}, false, false,
        "等待地图、代价图或机器人位姿变化后复核路线", {}};
    }
    state->last_route_plan_at = steady_now;
    state->search_wait_map_revision = map_revision_;
    state->search_wait_costmap_revision = costmap_revision_;
    state->search_wait_observation_count = observation_count_;
    state->search_wait_pose_valid = true;
    state->search_wait_x = robot_x;
    state->search_wait_y = robot_y;
    state->search_wait_yaw = robot_yaw;
    const auto & map = *online_map;
    const auto & costmap = *global_costmap_;
    const auto & goal = state->goal.pose.pose;
    const double goal_x = goal.position.x;
    const double goal_y = goal.position.y;
    const double goal_yaw = tf2::getYaw(goal.orientation);
    const auto start_cell = bringup::detail::gridCellAtWorld(map, robot_x, robot_y);
    const auto goal_cell = bringup::detail::gridCellAtWorld(map, goal_x, goal_y);
    const auto cell_value = [](const nav_msgs::msg::OccupancyGrid & grid,
        const std::optional<bringup::detail::GridCell> & cell) {
        return cell && bringup::detail::inside(grid, *cell) ?
          static_cast<int>(grid.data[bringup::online_goal_route::flatIndex(
            grid, cell->first, cell->second)]) : -2;
      };
    const auto start_value = cell_value(map, start_cell);
    const auto goal_value = cell_value(map, goal_cell);
    const bool map_escape_active = bringup::online_goal_route::footprintTouchesUnknown(
      map, robot_x, robot_y, robot_yaw, route_footprint_points_);
    const bool costmap_escape_active = bringup::online_goal_route::footprintTouchesUnknown(
      costmap, robot_x, robot_y, robot_yaw, route_footprint_points_);
    const double map_escape_radius = map_escape_active ?
      start_unknown_escape_radius_ : 0.0;
    const double costmap_escape_radius = costmap_escape_active ?
      start_unknown_escape_radius_ : 0.0;
    const auto start_clearance = start_cell ?
      bringup::online_goal_route::assessRouteClearance(
      map, start_cell->first, start_cell->second, route_footprint_radius_,
      robot_x, robot_y, robot_yaw, route_footprint_points_, map_escape_radius) :
      bringup::online_goal_route::FootprintAssessment{};
    const auto actual_start_footprint =
      bringup::online_goal_route::assessFootprintWithStartEscape(
      map, costmap, robot_x, robot_y, robot_yaw, route_footprint_points_,
      robot_x, robot_y, map_escape_radius, costmap_escape_radius);
    if (goal_cell && bringup::detail::inside(map, *goal_cell)) {
      const auto goal_index = bringup::online_goal_route::flatIndex(
        map, goal_cell->first, goal_cell->second);
      if (map.data[goal_index] > 50) {
        state->waiting_for_search_update = true;
        std::ostringstream diagnostic;
        diagnostic << "branch=goal_occupied map_revision=" << map_revision_
                   << " goal_cell=(" << goal_cell->first << ',' << goal_cell->second
                   << ") value=" << static_cast<int>(map.data[goal_index]);
        logRouteDiagnostic(diagnostic.str());
        return waitForRouteProgress(
          state, "最终目标暂不可达；等待地图更新后继续", diagnostic.str());
      }
    }

    std::optional<bringup::online_goal_route::Route> route;
    bool route_reaches_final = false;
    bool selected_frontier = false;
    bool direct_route_attempted = false;
    std::string direct_route_result = "goal_not_known_free";
    std::string frontier_route_result = "not_attempted";
    bringup::online_goal_route::FootprintAssessment direct_blocker{
      true, bringup::online_goal_route::FootprintBlocker::NONE};
    bringup::online_goal_route::FootprintAssessment frontier_blocker{
      true, bringup::online_goal_route::FootprintBlocker::NONE};
    bringup::online_goal_route::FrontierRoute frontier_route;
    if (start_clearance.clear && actual_start_footprint.clear &&
      goal_cell && bringup::detail::inside(map, *goal_cell) &&
      bringup::online_goal_route::knownFree(map,
      bringup::online_goal_route::flatIndex(map, goal_cell->first, goal_cell->second)))
    {
      direct_route_attempted = true;
      route = bringup::online_goal_route::planKnownPath(
        map, robot_x, robot_y, goal_x, goal_y, route_footprint_radius_,
        route_footprint_points_, static_cast<std::size_t>(max_route_expansions_),
        map_escape_radius, robot_yaw, goal_yaw, &direct_blocker);
      route_reaches_final = route.has_value();
      direct_route_result = route_reaches_final ? "route_found" : "no_known_path";
    }
    if (!route) {
      if (start_clearance.clear && actual_start_footprint.clear) {
        auto candidate = bringup::online_goal_route::selectFrontierRoute(
          map, robot_x, robot_y, goal_x, goal_y, route_footprint_radius_,
          route_footprint_points_, state->excluded_frontiers, frontier_exclusion_radius_,
          static_cast<std::size_t>(max_route_expansions_), map_escape_radius,
          robot_yaw, &frontier_blocker);
        if (candidate) {
          frontier_route = std::move(*candidate);
          route = frontier_route.route;
          selected_frontier = true;
          frontier_route_result = "frontier_found";
        } else {
          frontier_route_result = "no_reachable_frontier";
        }
      } else {
        frontier_route_result = "blocked_at_start";
      }
    }
    if (!route || route->points.empty()) {
      const auto * blocker = &start_clearance;
      const char * blocker_source = "start_route_clearance";
      if (start_clearance.clear && !actual_start_footprint.clear) {
        blocker = &actual_start_footprint;
        blocker_source = "start_actual_footprint";
      } else if (start_clearance.clear && actual_start_footprint.clear &&
        direct_blocker.blocker != bringup::online_goal_route::FootprintBlocker::NONE)
      {
        blocker = &direct_blocker;
        blocker_source = "direct_route_search";
      } else if (start_clearance.clear && actual_start_footprint.clear &&
        frontier_blocker.blocker != bringup::online_goal_route::FootprintBlocker::NONE)
      {
        blocker = &frontier_blocker;
        blocker_source = "frontier_search";
      }
      std::ostringstream diagnostic;
      diagnostic << "branch=" << (!start_clearance.clear ? "start_route_clearance_blocked" :
        (!actual_start_footprint.clear ? "start_actual_footprint_blocked" :
        "no_known_route_or_reachable_frontier"))
                 << " map_revision=" << map_revision_
                 << " observation_count=" << observation_count_
                 << " map=" << map.info.width << 'x' << map.info.height << '@'
                 << map.info.resolution
                 << " costmap=" << costmap.info.width << 'x' << costmap.info.height << '@'
                 << costmap.info.resolution
                 << " costmap_age_s=" << std::chrono::duration<double>(
        steady_now - costmap_received_at_).count()
                 << " robot=(" << robot_x << ',' << robot_y << ',' << robot_yaw << ')'
                 << " start_cell=(" << (start_cell ? start_cell->first : -1) << ','
                 << (start_cell ? start_cell->second : -1) << ") value=" << start_value
                 << " start_clear=" << start_clearance.clear
                 << " physical_footprint_clear=" << actual_start_footprint.clear
                 << " map_escape_active=" << map_escape_active
                 << " map_escape_radius=" << map_escape_radius
                 << " costmap_escape_active=" << costmap_escape_active
                 << " costmap_escape_radius=" << costmap_escape_radius
                 << " blocker_source=" << blocker_source
                 << " blocker=" << footprintBlockerName(blocker->blocker)
                 << " blocker_cell=(" << blocker->cell_x << ',' << blocker->cell_y << ')'
                 << " blocker_value=" << static_cast<int>(blocker->value)
                 << " goal_cell=(" << (goal_cell ? goal_cell->first : -1) << ','
                 << (goal_cell ? goal_cell->second : -1) << ") value=" << goal_value
                 << " direct=" << (direct_route_attempted ? direct_route_result :
        (goal_value == -2 ? "goal_out_of_map" : (goal_value < 0 ? "goal_unknown" :
        (goal_value > 50 ? "goal_occupied" : "start_blocked"))))
                 << " frontier=" << frontier_route_result;
      state->waiting_for_search_update = true;
      return waitForRouteProgress(
        state, "等待观测：当前地图没有可达路线或前沿", diagnostic.str());
    }

    route->points.front() = {robot_x, robot_y};
    if (route->points.size() > 1) {
      const double first_motion_yaw = std::atan2(
        route->points[1].second - route->points[0].second,
        route->points[1].first - route->points[0].first);
      // Check the actual robot's initial turn on both grids before validating
      // or dispatching any downstream waypoint.
      const auto start_rotation =
        bringup::online_goal_route::assessFootprintRotationSweepWithStartEscape(
        map, costmap, robot_x, robot_y, robot_yaw, first_motion_yaw,
        route_footprint_points_, robot_x, robot_y,
        map_escape_radius, costmap_escape_radius);
      if (!start_rotation.clear) {
        const auto map_start_rotation =
          bringup::online_goal_route::assessFootprintRotationSweepWithStartEscape(
          map, map, robot_x, robot_y, robot_yaw, first_motion_yaw,
          route_footprint_points_, robot_x, robot_y,
          map_escape_radius, map_escape_radius);
        const char * blocker_source = map_start_rotation.clear ? "global_costmap" : "map";
        std::ostringstream diagnostic;
        diagnostic << "branch=start_rotation_sweep_blocked source=" << blocker_source
                   << " robot=(" << robot_x << ',' << robot_y
                   << ") first_motion_yaw=" << first_motion_yaw
                   << " blocker=" << footprintBlockerName(start_rotation.blocker)
                   << " cell=(" << start_rotation.cell_x << ',' << start_rotation.cell_y
                   << ") value=" << static_cast<int>(start_rotation.value);
        return handleFootprintBlocker(
          state, "等待起步旋转扫掠通过地图和滚动代价图校验",
          start_rotation, diagnostic.str());
      }
    }
    if (route->points.size() == 1 && route_reaches_final &&
      std::hypot(goal_x - robot_x, goal_y - robot_y) <= route_goal_tolerance_)
    {
      const auto final_target = bringup::online_goal_route::assessFootprintWithStartEscape(
        map, costmap, goal_x, goal_y, goal_yaw, route_footprint_points_,
        robot_x, robot_y, map_escape_radius, costmap_escape_radius);
      if (!final_target.clear) {
        return handleFootprintBlocker(state, "最终目标暂被占用；等待地图恢复",
          final_target, "最终精确位姿未通过机器人 footprint 和滚动代价地图校验；blocker=" +
          std::string(footprintBlockerName(final_target.blocker)) + " cell=(" +
          std::to_string(final_target.cell_x) + "," +
          std::to_string(final_target.cell_y) + ") value=" +
          std::to_string(static_cast<int>(final_target.value)));
      }
      const auto final_rotation =
        bringup::online_goal_route::assessFootprintRotationSweepWithStartEscape(
        map, costmap, goal_x, goal_y, robot_yaw, goal_yaw, route_footprint_points_,
        robot_x, robot_y, map_escape_radius, costmap_escape_radius);
      if (!final_rotation.clear) {
        return handleFootprintBlocker(state, "最终目标旋转区域暂不可达；等待恢复",
          final_rotation, "最终精确位姿旋转扫掠未通过 footprint 和滚动代价地图校验；blocker=" +
          std::string(footprintBlockerName(final_rotation.blocker)) + " cell=(" +
          std::to_string(final_rotation.cell_x) + "," +
          std::to_string(final_rotation.cell_y) + ") value=" +
          std::to_string(static_cast<int>(final_rotation.value)));
      }
      state->route_wait_started = false;
      state->no_route_progress_updates = 0;
      state->last_route_diagnostic.clear();
      return {RoutePlanState::READY, state->goal.pose, true, false,
        "分段导航：最终目标在当前可达范围内", {}};
    }

    std::size_t waypoint_index = 0;
    bool have_waypoint = false;
    bringup::online_goal_route::FootprintAssessment blocked_rotation{
      true, bringup::online_goal_route::FootprintBlocker::NONE};
    double traversed_length = 0.0;
    for (std::size_t i = 1; i < route->points.size(); ++i) {
      const auto & previous = route->points[i - 1];
      const auto & point = route->points[i];
      const double step = std::hypot(point.first - previous.first, point.second - previous.second);
      if (traversed_length + step > segment_length_) break;
      const auto & next = route->points[std::min(i + 1, route->points.size() - 1)];
      double yaw = std::atan2(next.second - point.second, next.first - point.first);
      const bool at_route_end = i + 1 == route->points.size();
      if (at_route_end && route_reaches_final) yaw = goal_yaw;
      if (at_route_end && selected_frontier) yaw = frontier_route.frontier.yaw;
      const double waypoint_map_escape = at_route_end && selected_frontier ? 0.0 :
        map_escape_radius;
      const double waypoint_costmap_escape = at_route_end && selected_frontier ? 0.0 :
        costmap_escape_radius;
      const auto waypoint_footprint =
        bringup::online_goal_route::assessFootprintWithStartEscape(
        map, costmap, point.first, point.second, yaw, route_footprint_points_,
        robot_x, robot_y, waypoint_map_escape, waypoint_costmap_escape);
      if (!waypoint_footprint.clear) {
        break;
      }
      const auto & previous_point = route->points[i - 1];
      const double arrival_yaw = std::atan2(
        point.second - previous_point.second, point.first - previous_point.first);
      const auto waypoint_rotation =
        bringup::online_goal_route::assessFootprintRotationSweepWithStartEscape(
        map, costmap, point.first, point.second, arrival_yaw, yaw,
        route_footprint_points_, robot_x, robot_y,
        waypoint_map_escape, waypoint_costmap_escape);
      if (!waypoint_rotation.clear) {
        blocked_rotation = waypoint_rotation;
        break;
      }
      traversed_length += step;
      waypoint_index = i;
      have_waypoint = true;
    }
    if (!have_waypoint && selected_frontier && route->points.size() == 1) {
      // The robot is already in a safe observation cell; it may still need a
      // short pose adjustment to face the newly selected frontier.
      waypoint_index = 0;
      have_waypoint = true;
    }
    if (!have_waypoint) {
      const auto & blocked_point = route->points.size() > 1 ? route->points[1] :
        route->points.front();
      const double blocked_yaw = route->points.size() > 1 ? std::atan2(
        route->points[1].second - route->points[0].second,
        route->points[1].first - route->points[0].first) : robot_yaw;
      const auto blocker = bringup::online_goal_route::assessFootprintWithStartEscape(
        map, costmap, blocked_point.first, blocked_point.second, blocked_yaw,
        route_footprint_points_, robot_x, robot_y, map_escape_radius,
        costmap_escape_radius);
      const auto & reported_blocker = blocked_rotation.clear ? blocker : blocked_rotation;
      std::ostringstream diagnostic;
      diagnostic << "branch=next_waypoint_footprint_blocked map_revision=" << map_revision_
                 << " robot=(" << robot_x << ',' << robot_y << ')'
                 << " start_cell=(" << (start_cell ? start_cell->first : -1) << ','
                 << (start_cell ? start_cell->second : -1) << ") value=" << start_value
                 << " waypoint=(" << blocked_point.first << ',' << blocked_point.second << ')'
                 << " blocker=" << footprintBlockerName(reported_blocker.blocker)
                 << " blocker_cell=(" << reported_blocker.cell_x << ','
                 << reported_blocker.cell_y << ')'
                 << " blocker_value=" << static_cast<int>(reported_blocker.value);
      return waitForRouteProgress(
        state, "等待滚动窗口覆盖下一段安全航点", diagnostic.str());
    }

    const bool at_route_end = waypoint_index + 1 == route->points.size();
    bool is_final = route_reaches_final && at_route_end;
    bool is_frontier = selected_frontier && at_route_end;
    auto point = route->points[waypoint_index];
    double waypoint_yaw = std::atan2(
      point.second - robot_y, point.first - robot_x);
    if (is_frontier) {
      point = {frontier_route.frontier.x, frontier_route.frontier.y};
      waypoint_yaw = frontier_route.frontier.yaw;
    } else if (waypoint_index + 1 < route->points.size()) {
      const auto & next = route->points[waypoint_index + 1];
      waypoint_yaw = std::atan2(next.second - point.second, next.first - point.first);
    }
    const double dispatch_x = is_final ? goal_x : point.first;
    const double dispatch_y = is_final ? goal_y : point.second;
    const double dispatch_yaw = is_final ? goal_yaw : waypoint_yaw;
    const double dispatch_map_escape = is_frontier ? 0.0 : map_escape_radius;
    const double dispatch_costmap_escape = is_frontier ? 0.0 : costmap_escape_radius;
    const auto dispatch_footprint =
      bringup::online_goal_route::assessFootprintWithStartEscape(
      map, costmap, dispatch_x, dispatch_y, dispatch_yaw, route_footprint_points_,
      robot_x, robot_y, dispatch_map_escape, dispatch_costmap_escape);
    if (!dispatch_footprint.clear) {
      return handleFootprintBlocker(state, "下一航点暂被占用；等待通路恢复",
        dispatch_footprint, "航点未通过真实机器人 footprint 或滚动代价地图校验；blocker=" +
        std::string(footprintBlockerName(dispatch_footprint.blocker)) + " cell=(" +
        std::to_string(dispatch_footprint.cell_x) + "," +
        std::to_string(dispatch_footprint.cell_y) + ") value=" +
        std::to_string(static_cast<int>(dispatch_footprint.value)));
    }
    double arrival_yaw = robot_yaw;
    if (waypoint_index > 0) {
      const auto & previous_point = route->points[waypoint_index - 1];
      const auto & arrival_point = route->points[waypoint_index];
      arrival_yaw = std::atan2(
        arrival_point.second - previous_point.second,
        arrival_point.first - previous_point.first);
    }
    const auto dispatch_rotation =
      bringup::online_goal_route::assessFootprintRotationSweepWithStartEscape(
      map, costmap, dispatch_x, dispatch_y, arrival_yaw, dispatch_yaw,
      route_footprint_points_, robot_x, robot_y,
      dispatch_map_escape, dispatch_costmap_escape);
    if (!dispatch_rotation.clear) {
      return handleFootprintBlocker(state, "航点旋转区域暂不可达；等待通路恢复",
        dispatch_rotation, "航点旋转扫掠未通过真实机器人 footprint 或滚动代价地图校验；blocker=" +
        std::string(footprintBlockerName(dispatch_rotation.blocker)) + " cell=(" +
        std::to_string(dispatch_rotation.cell_x) + "," +
        std::to_string(dispatch_rotation.cell_y) + ") value=" +
        std::to_string(static_cast<int>(dispatch_rotation.value)));
    }
    point = {dispatch_x, dispatch_y};
    waypoint_yaw = dispatch_yaw;
    state->route_wait_started = false;
    state->no_route_progress_updates = 0;
    state->last_route_diagnostic.clear();
    state->dispatch_is_final = is_final;
    state->dispatch_is_frontier = is_frontier;
    if (is_frontier) {
      state->last_frontier_x = point.first;
      state->last_frontier_y = point.second;
      state->frontier_map_revision = map_revision_;
      state->frontier_observation_count = observation_count_;
      state->frontier_unknown_cells_before =
        bringup::online_goal_route::unknownCellsAhead(
        map, point.first, point.second, waypoint_yaw);
    }
    const auto status = is_frontier ? "探索前沿：前往可观察位置" :
      (is_final ? "分段导航：前往最终目标" : "分段导航：前往当前航点");
    return {RoutePlanState::READY, makePose(point.first, point.second, waypoint_yaw),
      is_final, is_frontier, status, {}};
  }

  void publishGoalStatus(const std::string & status)
  {
    if (!manage_goal_routes_ || status == last_goal_status_) return;
    std_msgs::msg::String message;
    message.data = status;
    goal_status_pub_->publish(message);
    last_goal_status_ = status;
  }

  static std::string uuidText(const std::array<std::uint8_t, 16> & uuid)
  {
    std::string text;
    text.reserve(32);
    constexpr char digits[] = "0123456789abcdef";
    for (const auto byte : uuid) {
      text.push_back(digits[(byte >> 4) & 0x0f]);
      text.push_back(digits[byte & 0x0f]);
    }
    return text;
  }

  bool tryNormalizeGoal(
    const std::shared_ptr<ForwardState> & state,
    geometry_msgs::msg::PoseStamped & normalized,
    std::string & failure_reason) const
  {
    try {
      if (state->goal.pose.header.frame_id == global_frame_) {
        normalized = state->goal.pose;
      } else {
        const auto frames = tf_buffer_->getAllFrameNames();
        if (std::find(frames.begin(), frames.end(), state->goal.pose.header.frame_id) ==
          frames.end())
        {
          failure_reason = "unknown_goal_frame";
          return false;
        }
        normalized = tf_buffer_->transform(
          state->goal.pose, global_frame_, tf2::durationFromSec(0.0));
      }
    } catch (const tf2::TransformException & exception) {
      failure_reason = exception.what();
      return false;
    }
    normalized.header.frame_id = global_frame_;
    normalized.header.stamp = now();
    if (!validGoal(normalized)) {
      failure_reason = "transformed_goal_is_invalid";
      return false;
    }
    return true;
  }

  void applyCostmapUpdateToGrid(const map_msgs::msg::OccupancyGridUpdate & update)
  {
    if (!global_costmap_ || update.width == 0 || update.height == 0 ||
      (!update.header.frame_id.empty() && update.header.frame_id != global_frame_) ||
      update.x > global_costmap_->info.width || update.y > global_costmap_->info.height ||
      update.width > global_costmap_->info.width - update.x ||
      update.height > global_costmap_->info.height - update.y ||
      update.data.size() != static_cast<std::size_t>(update.width) * update.height)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Ignoring malformed global costmap update");
      return;
    }
    if ((update.header.stamp.sec != 0 || update.header.stamp.nanosec != 0) &&
      rclcpp::Time(update.header.stamp, get_clock()->get_clock_type()) <=
      rclcpp::Time(global_costmap_->header.stamp, get_clock()->get_clock_type()))
    {
      return;
    }
    for (std::uint32_t y = 0; y < update.height; ++y) {
      for (std::uint32_t x = 0; x < update.width; ++x) {
        const auto source = static_cast<std::size_t>(y) * update.width + x;
        const auto target = static_cast<std::size_t>(update.y + y) *
          global_costmap_->info.width + update.x + x;
        global_costmap_->data[target] = update.data[source];
      }
    }
    global_costmap_->header.stamp = update.header.stamp;
    costmap_received_at_ = std::chrono::steady_clock::now();
    ++costmap_revision_;
  }

  bool hasUnresolvedInternalActionsLocked() const
  {
    return !unresolved_internal_actions_.empty();
  }

  void removeUnresolvedInternalActionLocked(const std::shared_ptr<ForwardState> & state)
  {
    for (auto it = unresolved_internal_actions_.begin();
      it != unresolved_internal_actions_.end(); ++it)
    {
      if (*it == state) {
        unresolved_internal_actions_.erase(it);
        return;
      }
    }
  }

  bool tryFinalizePublicGoal(
    const std::shared_ptr<ForwardState> & state,
    const PublicTerminal terminal,
    const std::shared_ptr<Action::Result> & result)
  {
    if (!state || state->public_terminalized) {
      return true;
    }
    auto & handle = state->public_handle;
    if (handle->is_canceling()) {
      handle->canceled(result);
      state->public_terminalized = true;
      return true;
    }
    if (!handle->is_active()) {
      state->public_terminalized = true;
      return true;
    }
    if (state->cancel_requested) {
      return false;
    }
    if (!handle->is_executing()) {
      return false;
    }
    switch (terminal) {
      case PublicTerminal::SUCCEEDED:
        handle->succeed(result);
        break;
      case PublicTerminal::ABORTED:
        handle->abort(result);
        break;
      case PublicTerminal::CANCELED:
        RCLCPP_ERROR(
          get_logger(), "Refusing canceled terminal transition without a public cancel request; "
          "aborting generation=%llu",
          static_cast<unsigned long long>(state->generation));
        handle->abort(result);
        break;
    }
    state->public_terminalized = true;
    return true;
  }

  void completeOrDeferPublicGoal(
    const std::shared_ptr<ForwardState> & state,
    const PublicTerminal terminal,
    const std::shared_ptr<Action::Result> & result)
  {
    if (tryFinalizePublicGoal(state, terminal, result)) {
      return;
    }
    for (const auto & pending : pending_public_finalizations_) {
      if (pending.state == state) {
        return;
      }
    }
    pending_public_finalizations_.push_back({state, terminal, result});
    RCLCPP_WARN(
      get_logger(), "Deferring public action terminal until its legal state transition is ready; "
      "generation=%llu cancel_requested=%s",
      static_cast<unsigned long long>(state->generation),
      state->cancel_requested ? "true" : "false");
  }

  void retryPendingPublicFinalizations()
  {
    for (auto it = pending_public_finalizations_.begin();
      it != pending_public_finalizations_.end();)
    {
      if (tryFinalizePublicGoal(it->state, it->terminal, it->result)) {
        it = pending_public_finalizations_.erase(it);
      } else {
        ++it;
      }
    }
  }

  rclcpp_action::CancelResponse cancelCallback(
    const std::shared_ptr<PublicGoalHandle> & goal)
  {
    std::shared_ptr<InternalGoalHandle> internal;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!active_ || active_->public_handle != goal) {
        return rclcpp_action::CancelResponse::REJECT;
      }
      active_->cancel_requested = true;
      internal = active_->internal_handle;
    }
    if (internal) {
      internal_client_->async_cancel_goal(internal);
    }
    publishGoalStatus("已取消：探索与分段导航任务结束");
    RCLCPP_INFO(get_logger(), "Navigation intent canceled by its action client");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void acceptedCallback(const std::shared_ptr<PublicGoalHandle> & goal)
  {
    auto state = std::make_shared<ForwardState>();
    state->goal = *goal->get_goal();
    state->source_goal_frame = state->goal.pose.header.frame_id;
    state->public_handle = goal;
    std::shared_ptr<ForwardState> replaced;
    std::shared_ptr<InternalGoalHandle> replaced_internal;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      replaced = active_;
      if (replaced) {
        replaced_internal = replaced->internal_handle;
      }
      state->generation = ++generation_;
      if (hasUnresolvedInternalActionsLocked())
      {
        state->waiting_for_replaced_goal = true;
        state->replaced_goal_deadline = std::chrono::steady_clock::now() +
          std::chrono::seconds(2);
      }
      active_ = state;
    }

    if (replaced) {
      if (replaced_internal) {
        internal_client_->async_cancel_goal(replaced_internal);
      }
      completeOrDeferPublicGoal(
        replaced, PublicTerminal::ABORTED, std::make_shared<Action::Result>());
      RCLCPP_INFO(
        get_logger(), "Navigation intent replaced generation=%llu by generation=%llu "
        "previous_public_action_finalized=%s waiting_for_previous_internal_goal=%s",
        static_cast<unsigned long long>(replaced->generation),
        static_cast<unsigned long long>(state->generation),
        replaced->public_terminalized ? "true" : "deferred",
        state->waiting_for_replaced_goal ? "true" : "false");
    }

    RCLCPP_INFO(
      get_logger(), "Navigation intent accepted generation=%llu raw_target=(%.3f,%.3f) "
      "source_frame=%s global_frame=%s",
      static_cast<unsigned long long>(state->generation),
      state->goal.pose.pose.position.x, state->goal.pose.pose.position.y,
      state->source_goal_frame.c_str(), global_frame_.c_str());
    publishGoalStatus("分段导航：已接收最终目标，等待导航就绪");
    dispatchWhenReady();
  }

  void readyCallback(const bool ready)
  {
    std::shared_ptr<InternalGoalHandle> internal;
    bool changed = false;
    bool report_pause_state = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      changed = ready_ != ready;
      ready_ = ready;
      report_pause_state = changed && static_cast<bool>(active_) &&
        !active_->cancel_requested;
      if (changed && ready_ && active_ && active_->waiting_for_replaced_goal) {
        active_->replaced_goal_deadline =
          std::chrono::steady_clock::now() + std::chrono::seconds(2);
      }
      if (!ready_ && active_) {
        active_->system_interrupted = true;
        if (!active_->goal_normalized) {
          active_->transform_wait_started = false;
          active_->transform_wait_last_status = {};
        }
        internal = active_->internal_handle;
      }
    }
    if (changed) {
      RCLCPP_INFO(
        get_logger(), "Navigation readiness %s; saved goal intent %s",
        ready ? "ready" : "not ready", ready ? "will be replanned" : "is held");
    }
    if (report_pause_state) {
      publishGoalStatus(ready ?
        "定位恢复：正在为当前目标重新规划" :
        "暂停：导航输入暂不可用，底盘停车并保留当前目标");
    }
    if (internal) {
      internal_client_->async_cancel_goal(internal);
    }
    if (ready) {
      dispatchWhenReady();
    }
  }

  void dispatchWhenReady()
  {
    std::shared_ptr<ForwardState> state;
    std::shared_ptr<ForwardState> finish_state;
    PublicTerminal finish_terminal{PublicTerminal::ABORTED};
    std::shared_ptr<Action::Result> finish_result;
    bool normalize_goal = false;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      if (!active_) {
        return;
      }
      if (active_->waiting_for_replaced_goal && !active_->cancel_requested) {
        if (!ready_) {
          return;
        }
        if (hasUnresolvedInternalActionsLocked()) {
          if (std::chrono::steady_clock::now() < active_->replaced_goal_deadline) {
            return;
          }
          RCLCPP_ERROR(
            get_logger(), "Previous Nav2 goal cancellation timed out; aborting replacement "
            "generation=%llu while retaining the internal-action stop barrier",
            static_cast<unsigned long long>(active_->generation));
          active_->terminal_reason = "上一个 Nav2 子任务停止超时";
          finish_state = active_;
          finish_terminal = PublicTerminal::ABORTED;
          finish_result = std::make_shared<Action::Result>();
          active_.reset();
        } else {
          active_->waiting_for_replaced_goal = false;
        }
      }
      if (!finish_state && active_->waiting_for_pause_signal) {
        if (active_->cancel_requested) {
          finish_state = active_;
          finish_terminal = PublicTerminal::CANCELED;
          finish_result = std::make_shared<Action::Result>();
          active_.reset();
        } else if (!ready_) {
          active_->waiting_for_pause_signal = false;
          active_->system_interrupted = true;
        } else if (std::chrono::steady_clock::now() >= active_->pause_signal_deadline) {
          finish_state = active_;
          finish_terminal = PublicTerminal::ABORTED;
          active_->terminal_reason = "Nav2 当前航点失败，任务在真实障碍或不连通路线处受阻";
          finish_result = active_->interrupted_result ? active_->interrupted_result :
            std::make_shared<Action::Result>();
          active_.reset();
        } else {
          return;
        }
      }
      if (!finish_state && active_) {
        if (active_->cancel_requested && !active_->send_pending && !active_->internal_handle) {
          finish_state = active_;
          finish_terminal = PublicTerminal::CANCELED;
          finish_result = std::make_shared<Action::Result>();
          active_.reset();
        } else if (ready_ && !active_->cancel_requested && !active_->send_pending &&
          !active_->internal_handle)
        {
          state = active_;
          normalize_goal = !state->goal_normalized;
          if (!normalize_goal) {
            state->send_pending = true;
          }
        }
      }
    }
    if (finish_state) {
      if (finish_terminal == PublicTerminal::ABORTED &&
        !finish_state->terminal_reason.empty())
      {
        publishGoalStatus("受阻：" + finish_state->terminal_reason);
      }
      completeOrDeferPublicGoal(finish_state, finish_terminal, finish_result);
      return;
    }
    if (normalize_goal && state) {
      geometry_msgs::msg::PoseStamped normalized_goal;
      std::string failure_reason;
      if (!tryNormalizeGoal(state, normalized_goal, failure_reason)) {
        if (failure_reason == "unknown_goal_frame") {
          bool finish_public = false;
          {
            std::lock_guard<std::mutex> lock(mutex_);
            if (active_ == state && !state->cancel_requested) {
              active_.reset();
              finish_public = true;
            }
          }
          if (finish_public) {
            state->terminal_reason = "目标坐标系不存在：" + state->source_goal_frame;
            RCLCPP_ERROR(
              get_logger(), "Rejecting navigation goal with unknown frame: generation=%llu "
              "frame=%s",
              static_cast<unsigned long long>(state->generation),
              state->source_goal_frame.c_str());
            publishGoalStatus("请求错误：" + state->terminal_reason);
            completeOrDeferPublicGoal(
              state, PublicTerminal::ABORTED, std::make_shared<Action::Result>());
          }
          return;
        }
        const auto now = std::chrono::steady_clock::now();
        bool publish_wait_status = false;
        {
          std::lock_guard<std::mutex> lock(mutex_);
          if (active_ == state && !state->cancel_requested) {
            if (!state->transform_wait_started) {
              state->transform_wait_started = true;
              publish_wait_status = true;
            } else if (state->transform_wait_last_status ==
              std::chrono::steady_clock::time_point{} ||
              now - state->transform_wait_last_status >= std::chrono::seconds(1))
            {
              publish_wait_status = true;
            }
            if (publish_wait_status) {
              state->transform_wait_last_status = now;
            }
          }
        }
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Waiting for a stable-frame goal transform: generation=%llu source_frame=%s "
          "target_frame=%s reason=%s",
          static_cast<unsigned long long>(state->generation),
          state->source_goal_frame.c_str(), global_frame_.c_str(), failure_reason.c_str());
        if (publish_wait_status) {
          publishGoalStatus("暂停等待定位：保留当前目标，坐标变换恢复后重新规划");
        }
        return;
      }
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (active_ != state || state->cancel_requested || !ready_) {
          return;
        }
        state->goal.pose = normalized_goal;
        state->goal_normalized = true;
        state->transform_wait_started = false;
      }
      requested_goal_pub_->publish(normalized_goal);
      RCLCPP_INFO(
        get_logger(), "Navigation intent frozen in global frame: generation=%llu "
        "source_frame=%s target=(%.3f,%.3f) frame=%s",
        static_cast<unsigned long long>(state->generation),
        state->source_goal_frame.c_str(), normalized_goal.pose.position.x,
        normalized_goal.pose.position.y, normalized_goal.header.frame_id.c_str());
      dispatchWhenReady();
      return;
    }

    if (state && state->goal_normalized) {
      if (manage_goal_routes_ && !state->dispatch_prepared) {
        auto route_plan = prepareManagedDispatch(state);
        if (route_plan.state == RoutePlanState::WAIT) {
          {
            std::lock_guard<std::mutex> lock(mutex_);
            if (active_ == state) state->send_pending = false;
          }
          publishGoalStatus(route_plan.status);
          return;
        }
        if (route_plan.state == RoutePlanState::ABORT) {
          bool finish_public = false;
          {
            std::lock_guard<std::mutex> lock(mutex_);
            if (active_ == state) {
              state->send_pending = false;
              active_.reset();
              finish_public = true;
            }
          }
          if (finish_public) {
            state->terminal_reason = route_plan.reason;
            publishGoalStatus("受阻：" + route_plan.reason);
            RCLCPP_WARN(
              get_logger(), "Navigation intent stopped: generation=%llu reason=%s",
              static_cast<unsigned long long>(state->generation), route_plan.reason.c_str());
            completeOrDeferPublicGoal(
              state, PublicTerminal::ABORTED, std::make_shared<Action::Result>());
          }
          return;
        }
        state->dispatch_pose = route_plan.pose;
        state->dispatch_is_final = route_plan.is_final;
        state->dispatch_is_frontier = route_plan.is_frontier;
        state->dispatch_prepared = true;
        state->segment_number++;
        publishGoalStatus(route_plan.status + "，航点 " +
          std::to_string(state->dispatch_pose.pose.position.x) + "," +
          std::to_string(state->dispatch_pose.pose.position.y));
      } else if (!manage_goal_routes_) {
        state->dispatch_pose = state->goal.pose;
        state->dispatch_is_final = true;
      }
    }
    if (!state || !internal_client_->action_server_is_ready()) {
      if (state) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (active_ == state) {
          state->send_pending = false;
        }
      }
      return;
    }

    if (state && manage_goal_routes_) {
      current_waypoint_pub_->publish(state->dispatch_pose);
    }

    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (active_ != state || state->cancel_requested || !ready_) {
        state->send_pending = false;
        return;
      }
      unresolved_internal_actions_.push_back(state);
      state->dispatch_prepared = false;
    }

    auto options = rclcpp_action::Client<Action>::SendGoalOptions();
    options.goal_response_callback =
      [this, state](InternalGoalHandle::SharedPtr internal) {
        bool cancel_internal = false;
        bool dispatch_replacement = false;
        bool publish_identity = false;
        bool reject_public = false;
        bool retry_frontier = false;
        bool frontier_failed = false;
        bool wait_for_search_update = false;
        {
          std::lock_guard<std::mutex> lock(mutex_);
          state->send_pending = false;
          if (!internal) {
            removeUnresolvedInternalActionLocked(state);
            if (active_ == state && state->cancel_requested) {
              return;
            } else if (active_ == state && (!ready_ || state->system_interrupted)) {
              state->system_interrupted = false;
            } else if (active_ == state && ready_) {
              if (manage_goal_routes_ && state->dispatch_is_frontier) {
                state->excluded_frontiers.emplace_back(
                  state->last_frontier_x, state->last_frontier_y);
                ++state->frontier_attempts;
                state->dispatch_is_frontier = false;
                state->dispatch_prepared = false;
                state->last_route_plan_at = {};
                frontier_failed = true;
                if (state->frontier_attempts >=
                  static_cast<std::size_t>(max_frontier_attempts_))
                {
                  waitForNextFrontierSearchRound(state);
                  wait_for_search_update = true;
                } else {
                  retry_frontier = true;
                }
              } else {
                reject_public = true;
                state->terminal_reason = "Nav2 拒绝当前航点";
                active_.reset();
              }
            }
            if (active_ && active_->waiting_for_replaced_goal &&
              !hasUnresolvedInternalActionsLocked())
            {
              active_->waiting_for_replaced_goal = false;
              dispatch_replacement = true;
            }
          } else {
            state->internal_handle = internal;
            cancel_internal = active_ != state || state->cancel_requested || !ready_;
            publish_identity = active_ == state && !state->cancel_requested && ready_;
          }
        }
        if (internal && cancel_internal) {
          internal_client_->async_cancel_goal(internal);
        }
        if (reject_public) {
          RCLCPP_ERROR(get_logger(), "Nav2 rejected the current navigation goal");
          if (!state->terminal_reason.empty()) {
            publishGoalStatus("受阻：" + state->terminal_reason);
          }
          completeOrDeferPublicGoal(
            state, PublicTerminal::ABORTED, std::make_shared<Action::Result>());
        }
        if (frontier_failed && retry_frontier) {
          publishGoalStatus("受阻：前沿航点被拒绝，正在尝试其他候选");
        }
        if (wait_for_search_update) {
          publishGoalStatus("本轮前沿候选已用尽；保留原目标，等待新地图和新观测");
        }
        if (publish_identity && internal) {
          const auto & pose = state->dispatch_pose;
          const auto & pose_stamp = pose.header.stamp;
          std_msgs::msg::String identity;
          identity.data = std::to_string(state->generation) + ":" +
            uuidText(state->public_handle->get_goal_id()) + ":" +
            uuidText(internal->get_goal_id());
          goal_identity_pub_->publish(identity);
          std_msgs::msg::String pose_identity;
          pose_identity.data = uuidText(state->public_handle->get_goal_id()) + ":" +
            uuidText(internal->get_goal_id()) + ":" +
            std::to_string(pose_stamp.sec) + ":" +
            std::to_string(pose_stamp.nanosec) + ":" +
            std::to_string(pose.pose.position.x) + ":" +
            std::to_string(pose.pose.position.y);
          goal_pose_identity_pub_->publish(pose_identity);
          RCLCPP_INFO(
            get_logger(), "Navigation goal identity mapped generation=%llu public_id=%s "
            "internal_id=%s",
            static_cast<unsigned long long>(state->generation),
            uuidText(state->public_handle->get_goal_id()).c_str(),
            uuidText(internal->get_goal_id()).c_str());
        }
        if (retry_frontier || dispatch_replacement || wait_for_search_update) {
          dispatchWhenReady();
        }
      };
    options.feedback_callback =
      [this, state](InternalGoalHandle::SharedPtr,
      const std::shared_ptr<const Action::Feedback> feedback) {
        std::shared_ptr<PublicGoalHandle> target;
        {
          std::lock_guard<std::mutex> lock(mutex_);
          if (active_ == state && !state->cancel_requested && ready_) {
            target = state->public_handle;
          }
        }
        if (target && feedback) {
          target->publish_feedback(std::make_shared<Action::Feedback>(*feedback));
        }
      };
    options.result_callback =
      [this, state](const InternalGoalHandle::WrappedResult & result) {
        std::shared_ptr<ForwardState> finish_state;
        PublicTerminal finish_terminal{PublicTerminal::ABORTED};
        bool retry = false;
        bool dispatch_replacement = false;
        bool publish_terminal_status = false;
        std::string progress_status;
        std::shared_ptr<Action::Result> finish_result;
        {
          std::lock_guard<std::mutex> lock(mutex_);
          state->internal_handle.reset();
          state->send_pending = false;
          removeUnresolvedInternalActionLocked(state);
          if (active_ != state) {
            finish_state = state;
            finish_terminal = state->cancel_requested ?
              PublicTerminal::CANCELED : PublicTerminal::ABORTED;
          } else if (state->cancel_requested) {
            finish_state = state;
            finish_terminal = PublicTerminal::CANCELED;
            active_.reset();
            publish_terminal_status = true;
          } else if (!ready_ || state->system_interrupted) {
            state->system_interrupted = false;
            retry = ready_;
          } else if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            if (!manage_goal_routes_ || state->dispatch_is_final) {
              finish_state = state;
              finish_terminal = PublicTerminal::SUCCEEDED;
              active_.reset();
              publish_terminal_status = true;
              progress_status = "完成：已到达原始最终目标";
            } else if (state->dispatch_is_frontier) {
              state->dispatch_is_frontier = false;
              state->waiting_for_observation = true;
              state->observation_wait_deadline = std::chrono::steady_clock::now() +
                std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                std::chrono::duration<double>(observation_wait_timeout_));
              progress_status = "等待观测：已到达前沿，等待新地图和有效帧";
            } else {
              state->dispatch_prepared = false;
              state->last_route_plan_at = {};
              state->route_wait_started = false;
              retry = true;
              progress_status = "分段导航：当前航点已到达，正在复核长距离路线";
            }
          } else if (result.code == rclcpp_action::ResultCode::ABORTED) {
            finish_state = state;
            finish_terminal = PublicTerminal::ABORTED;
            state->terminal_reason = "Nav2 子任务异常终止；按程序故障停止并报告";
            active_.reset();
            publish_terminal_status = true;
          } else if (manage_goal_routes_ && state->dispatch_is_frontier) {
            state->excluded_frontiers.emplace_back(
              state->last_frontier_x, state->last_frontier_y);
            ++state->frontier_attempts;
            state->dispatch_is_frontier = false;
            state->dispatch_prepared = false;
            state->last_route_plan_at = {};
            if (state->frontier_attempts >=
              static_cast<std::size_t>(max_frontier_attempts_))
            {
              waitForNextFrontierSearchRound(state);
              retry = true;
              progress_status = "本轮前沿候选已用尽；保留原目标，等待新地图和新观测";
            } else {
              retry = true;
              progress_status = "受阻：一个前沿导航失败，正在尝试其他可达候选";
            }
          } else {
            state->waiting_for_pause_signal = true;
            state->interrupted_result_code = result.code;
            state->interrupted_result = result.result ?
              std::make_shared<Action::Result>(*result.result) :
              std::make_shared<Action::Result>();
            state->pause_signal_deadline = std::chrono::steady_clock::now() +
              std::chrono::milliseconds(500);
          }
          if (active_ && active_->waiting_for_replaced_goal &&
            !hasUnresolvedInternalActionsLocked())
          {
            active_->waiting_for_replaced_goal = false;
            dispatch_replacement = true;
          }
          if (finish_state) {
            finish_result = result.result ?
              std::make_shared<Action::Result>(*result.result) :
              std::make_shared<Action::Result>();
          }
        }
        if (finish_state) {
          completeOrDeferPublicGoal(finish_state, finish_terminal, finish_result);
        }
        if (publish_terminal_status && progress_status.empty()) {
          progress_status = finish_terminal == PublicTerminal::CANCELED ?
            "已取消：探索与分段导航任务结束" : "受阻：" + state->terminal_reason;
        }
        if (!progress_status.empty()) publishGoalStatus(progress_status);
        if (retry || dispatch_replacement) {
          dispatchWhenReady();
        }
      };
    try {
      auto dispatched_goal = state->goal;
      dispatched_goal.pose = state->dispatch_pose;
      internal_client_->async_send_goal(dispatched_goal, options);
    } catch (const std::exception & exception) {
      bool finish_public = false;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        removeUnresolvedInternalActionLocked(state);
        state->send_pending = false;
        if (active_ == state) {
          active_.reset();
          finish_public = true;
        }
      }
      RCLCPP_ERROR(
        get_logger(), "Failed to submit internal navigation goal generation=%llu: %s",
        static_cast<unsigned long long>(state->generation), exception.what());
      if (finish_public) {
        state->terminal_reason = "无法提交 Nav2 航点";
        publishGoalStatus("受阻：" + state->terminal_reason);
        completeOrDeferPublicGoal(
          state, PublicTerminal::ABORTED, std::make_shared<Action::Result>());
      }
      dispatchWhenReady();
    }
  }

  std::mutex mutex_;
  std::uint64_t generation_{0};
  bool ready_{false};
  std::shared_ptr<ForwardState> active_;
  std::vector<PendingPublicFinalization> pending_public_finalizations_;
  std::vector<std::shared_ptr<ForwardState>> unresolved_internal_actions_;
  std::string global_frame_;
  std::string map_topic_;
  std::string map_updates_topic_;
  std::string map_geometry_events_topic_;
  std::string costmap_topic_;
  std::string costmap_updates_topic_;
  std::string observation_topic_;
  std::string base_frame_;
  std::string last_goal_status_;
  bool manage_goal_routes_{false};
  double segment_length_{4.0};
  double observation_wait_timeout_{3.0};
  double route_retry_period_{1.0};
  double route_data_wait_timeout_{5.0};
  int max_frontier_attempts_{6};
  int max_no_route_progress_updates_{6};
  int max_route_expansions_{500000};
  double route_goal_tolerance_{0.20};
  double frontier_exclusion_radius_{0.80};
  double start_unknown_escape_radius_{0.50};
  double route_footprint_radius_{0.0};
  std::vector<double> route_footprint_;
  std::vector<FrontierFootprintPoint> route_footprint_points_;
  bringup::OnlineMapState online_map_state_;
  std::optional<nav_msgs::msg::OccupancyGrid> global_costmap_;
  std::vector<map_msgs::msg::OccupancyGridUpdate> pending_costmap_updates_;
  std::uint64_t map_revision_{0};
  std::uint64_t costmap_revision_{0};
  std::uint64_t observation_count_{0};
  std::int64_t last_observation_stamp_ns_{0};
  std::chrono::steady_clock::time_point costmap_received_at_{};
  tf2_ros::Buffer::SharedPtr tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp_action::Client<Action>::SharedPtr internal_client_;
  rclcpp_action::Client<Action>::SharedPtr topic_goal_client_;
  rclcpp_action::Server<Action>::SharedPtr server_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr requested_goal_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr current_waypoint_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr goal_status_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr goal_identity_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr goal_pose_identity_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr ready_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<map_msgs::msg::OccupancyGridUpdate>::SharedPtr map_updates_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt64MultiArray>::SharedPtr map_geometry_events_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
  rclcpp::Subscription<map_msgs::msg::OccupancyGridUpdate>::SharedPtr costmap_updates_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr observation_sub_;
  rclcpp::TimerBase::SharedPtr retry_timer_;
};

}  // namespace bringup

#ifndef BRINGUP_NAVIGATION_GOAL_PROXY_NO_MAIN
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<bringup::NavigationGoalProxy>());
  rclcpp::shutdown();
  return 0;
}
#endif
