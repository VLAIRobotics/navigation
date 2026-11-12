#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <action_msgs/msg/goal_status.hpp>
#include <action_msgs/msg/goal_status_array.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/qos.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/string.hpp>

#include "bringup/path_display_state.hpp"
#include "nav2_theta_star_planner/goal_cell_endpoint.hpp"

namespace bringup
{

class Nav2PathDisplayNode : public rclcpp::Node
{
public:
  Nav2PathDisplayNode()
  : Node("nav2_path_display")
  {
    direct_nav2_mode_ = declare_parameter<bool>("direct_nav2_mode", false);
    const auto navigation_status_topic = declare_parameter<std::string>(
      "navigation_goal_status_topic", "/navigate_to_pose/_action/status");
    const auto execution_status_topic = declare_parameter<std::string>(
      "execution_goal_status_topic", navigation_status_topic);
    const auto requested_goal_topic = declare_parameter<std::string>(
      "requested_goal_topic", "/navigation/requested_goal");
    const auto goal_identity_topic = declare_parameter<std::string>(
      "goal_identity_topic", "/navigation/goal_identity");
    const auto navigation_ready_topic = declare_parameter<std::string>(
      "navigation_ready_topic", "/navigation_ready");
    RCLCPP_INFO(
      get_logger(), "Navigation path display mode: %s",
      direct_nav2_mode_ ? "direct_nav2" : "identity_mapped");
    global_display_pub_ = create_publisher<nav_msgs::msg::Path>("/plan_display", 10);
    local_display_pub_ = create_publisher<nav_msgs::msg::Path>("/local_plan_display", 10);

    global_path_sub_ = create_subscription<nav_msgs::msg::Path>(
      "/plan", rclcpp::QoS(10),
      [this](nav_msgs::msg::Path::ConstSharedPtr message) {forwardPath(message, true);});
    local_path_sub_ = create_subscription<nav_msgs::msg::Path>(
      "/local_plan", rclcpp::QoS(10),
      [this](nav_msgs::msg::Path::ConstSharedPtr message) {forwardPath(message, false);});
    requested_goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      requested_goal_topic, rclcpp::QoS(1).reliable().transient_local(),
      [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr message) {
        requested_goal_ = *message;
        requested_goal_received_at_ = now();
        have_requested_goal_ = true;
        RCLCPP_INFO(
          get_logger(), "Navigation display target updated: goal_id=%s target=(%.3f,%.3f) "
          "frame=%s stamp=%d.%09u", navigation_goal_id_.c_str(),
          message->pose.position.x, message->pose.position.y,
          message->header.frame_id.c_str(), message->header.stamp.sec,
          message->header.stamp.nanosec);
      });
    navigation_ready_sub_ = create_subscription<std_msgs::msg::Bool>(
      navigation_ready_topic, rclcpp::QoS(1).reliable().transient_local(),
      [this](std_msgs::msg::Bool::ConstSharedPtr message) {
        navigation_paused_ = !message->data;
        if (navigation_paused_) {
          clearLocalPathForPause();
        }
      });
    goal_identity_sub_ = create_subscription<std_msgs::msg::String>(
      goal_identity_topic, rclcpp::QoS(1).reliable().transient_local(),
      [this](std_msgs::msg::String::ConstSharedPtr message) {
        const auto first_separator = message->data.find(':');
        const auto second_separator = first_separator == std::string::npos ?
          std::string::npos : message->data.find(':', first_separator + 1);
        if (first_separator == std::string::npos || second_separator == std::string::npos ||
          second_separator - first_separator != 33 || message->data.size() - second_separator != 33)
        {
          have_goal_identity_ = false;
          RCLCPP_ERROR(get_logger(), "Navigation goal identity mapping is malformed");
          return;
        }
        goal_identity_generation_ = message->data.substr(0, first_separator);
        mapped_navigation_goal_id_ = message->data.substr(first_separator + 1, 32);
        mapped_execution_goal_id_ = message->data.substr(second_separator + 1);
        have_goal_identity_ = true;
        RCLCPP_INFO(
          get_logger(), "Navigation display goal mapping: generation=%s public_id=%s "
          "execution_id=%s current_public=%s current_execution=%s",
          goal_identity_generation_.c_str(), mapped_navigation_goal_id_.c_str(),
          mapped_execution_goal_id_.c_str(), navigation_goal_id_.c_str(),
          execution_goal_id_.c_str());
      });

    compute_status_sub_ = create_subscription<action_msgs::msg::GoalStatusArray>(
      "/compute_path_to_pose/_action/status", rclcpp_action::DefaultActionStatusQoS(),
      [this](action_msgs::msg::GoalStatusArray::ConstSharedPtr message) {
        processStatuses(NavigationAction::ComputePath, *message);
      });
    follow_status_sub_ = create_subscription<action_msgs::msg::GoalStatusArray>(
      "/follow_path/_action/status", rclcpp_action::DefaultActionStatusQoS(),
      [this](action_msgs::msg::GoalStatusArray::ConstSharedPtr message) {
        processStatuses(NavigationAction::FollowPath, *message);
      });
    navigate_status_sub_ = create_subscription<action_msgs::msg::GoalStatusArray>(
      navigation_status_topic, rclcpp_action::DefaultActionStatusQoS(),
      [this](action_msgs::msg::GoalStatusArray::ConstSharedPtr message) {
        processStatuses(NavigationAction::NavigateToPose, *message);
      });
    execution_status_sub_ = create_subscription<action_msgs::msg::GoalStatusArray>(
      execution_status_topic, rclcpp_action::DefaultActionStatusQoS(),
      [this](action_msgs::msg::GoalStatusArray::ConstSharedPtr message) {
        observeExecutionStatus(*message);
      });
    last_display_stats_ = std::chrono::steady_clock::now();
  }

private:
  using SteadyClock = std::chrono::steady_clock;

  struct DisplayedPathInfo
  {
    builtin_interfaces::msg::Time stamp;
    std::string frame_id;
    std::size_t point_count{0};
    double start_x{0.0};
    double start_y{0.0};
    double end_x{0.0};
    double end_y{0.0};
  };

  void processStatuses(
    NavigationAction action, const action_msgs::msg::GoalStatusArray & message)
  {
    state_.observeClock(now().nanoseconds());
    const auto * latest = latestStatus(message);
    if (latest && action == NavigationAction::NavigateToPose) {
      const auto next_id = uuidText(latest->goal_info.goal_id.uuid);
      if (navigation_goal_id_ != next_id) {
        if (have_navigation_identity_ &&
          stampNewer(navigation_goal_stamp_, latest->goal_info.stamp))
        {
          return;
        }
        if (have_global_displayed_path_ &&
          global_display_navigation_goal_id_ != next_id)
        {
          clearDisplayedPath(
            true, "NavigateToPose", statusText(latest->status), latest,
            "public_goal_changed");
        }
        if (have_local_displayed_path_ &&
          local_display_navigation_goal_id_ != next_id)
        {
          clearDisplayedPath(
            false, "NavigateToPose", statusText(latest->status), latest,
            "public_goal_changed");
        }
        navigation_goal_id_ = next_id;
        navigation_goal_stamp_ = latest->goal_info.stamp;
        have_navigation_identity_ = true;
        have_navigation_goal_ = isActive(latest->status);
        RCLCPP_INFO(
          get_logger(), "Navigation display identity: goal_id=%s status=%s stamp=%d.%09u",
          navigation_goal_id_.c_str(), statusText(latest->status),
          latest->goal_info.stamp.sec, latest->goal_info.stamp.nanosec);
      } else if (isActive(latest->status)) {
        have_navigation_goal_ = true;
      } else {
        have_navigation_goal_ = false;
      }
    }
    if (direct_nav2_mode_ && latest) {
      if (action == NavigationAction::NavigateToPose) {
        direct_path_association_.observeNavigationGoal(*latest);
      } else {
        direct_path_association_.observeActionGoal(action, *latest);
      }
    }
    const auto clear = state_.observeStatuses(action, message);
    if (direct_nav2_mode_ && action != NavigationAction::NavigateToPose) {
      reconcileDirectPathDisplay(action, latest);
      return;
    }
    const bool terminal_is_old = latest &&
      action != NavigationAction::NavigateToPose && have_execution_goal_ &&
      stampNewer(execution_goal_stamp_, latest->goal_info.stamp);
    const auto terminal_id = latest ? uuidText(latest->goal_info.goal_id.uuid) : "none";
    const bool global_identity_matches = latest &&
      (action == NavigationAction::NavigateToPose ?
      global_display_navigation_goal_id_ == terminal_id :
      global_display_execution_id_ == execution_goal_id_);
    const bool local_identity_matches = latest &&
      (action == NavigationAction::NavigateToPose ?
      local_display_navigation_goal_id_ == terminal_id :
      local_display_execution_id_ == execution_goal_id_);
    const bool ignore_global = terminal_is_old || !global_identity_matches;
    const bool ignore_local = terminal_is_old || !local_identity_matches;
    if (clear.global && latest && !ignore_global && have_global_displayed_path_) {
      clearDisplayedPath(
        true, actionName(action), statusText(latest->status), latest, "current_goal_terminal");
    }
    if (clear.local && latest && !ignore_local && have_local_displayed_path_) {
      clearDisplayedPath(
        false, actionName(action), statusText(latest->status), latest, "current_goal_terminal");
    }
    if ((clear.global && ignore_global) || (clear.local && ignore_local) ||
      ((clear.global || clear.local) && latest == nullptr))
    {
      const char * reason = latest == nullptr ? "status_without_goal" :
        (terminal_is_old ? "old_action_terminal" : "path_bound_to_different_goal");
      RCLCPP_INFO(
        get_logger(), "Navigation display clear ignored: reason=%s action=%s "
        "terminal_id=%s current_navigation_id=%s current_execution_id=%s "
        "global_match=%s local_match=%s",
        reason, actionName(action), latest ?
        uuidText(latest->goal_info.goal_id.uuid).c_str() : "none",
        navigation_goal_id_.c_str(), execution_goal_id_.c_str(),
        global_identity_matches ? "true" : "false",
        local_identity_matches ? "true" : "false");
    }
  }

  static bool isActive(const std::uint8_t status)
  {
    return status == action_msgs::msg::GoalStatus::STATUS_ACCEPTED ||
      status == action_msgs::msg::GoalStatus::STATUS_EXECUTING;
  }

  static bool isTerminal(const std::uint8_t status)
  {
    return status == action_msgs::msg::GoalStatus::STATUS_SUCCEEDED ||
      status == action_msgs::msg::GoalStatus::STATUS_CANCELED ||
      status == action_msgs::msg::GoalStatus::STATUS_ABORTED;
  }

  static const action_msgs::msg::GoalStatus * latestStatus(
    const action_msgs::msg::GoalStatusArray & message)
  {
    const action_msgs::msg::GoalStatus * latest = nullptr;
    for (const auto & status : message.status_list) {
      if (!latest || stampNewer(status.goal_info.stamp, latest->goal_info.stamp) ||
        (!stampNewer(latest->goal_info.stamp, status.goal_info.stamp) &&
        status.goal_info.goal_id.uuid > latest->goal_info.goal_id.uuid))
      {
        latest = &status;
      }
    }
    return latest;
  }

  static bool stampNewer(
    const builtin_interfaces::msg::Time & lhs,
    const builtin_interfaces::msg::Time & rhs)
  {
    return lhs.sec > rhs.sec || (lhs.sec == rhs.sec && lhs.nanosec > rhs.nanosec);
  }

  static bool stampOlder(
    const builtin_interfaces::msg::Time & lhs,
    const builtin_interfaces::msg::Time & rhs)
  {
    return lhs.sec < rhs.sec || (lhs.sec == rhs.sec && lhs.nanosec < rhs.nanosec);
  }

  static std::string uuidText(const std::array<std::uint8_t, 16> & uuid)
  {
    std::ostringstream stream;
    stream << std::hex;
    for (const auto byte : uuid) {
      stream.width(2);
      stream.fill('0');
      stream << static_cast<unsigned int>(byte);
    }
    return stream.str();
  }

  static const char * statusText(const std::uint8_t status)
  {
    using GoalStatus = action_msgs::msg::GoalStatus;
    switch (status) {
      case GoalStatus::STATUS_ACCEPTED: return "accepted";
      case GoalStatus::STATUS_EXECUTING: return "executing";
      case GoalStatus::STATUS_SUCCEEDED: return "succeeded";
      case GoalStatus::STATUS_CANCELED: return "canceled";
      case GoalStatus::STATUS_ABORTED: return "aborted";
      default: return "unknown";
    }
  }

  static const char * actionName(const NavigationAction action)
  {
    switch (action) {
      case NavigationAction::ComputePath: return "ComputePathToPose";
      case NavigationAction::FollowPath: return "FollowPath";
      case NavigationAction::NavigateToPose: return "NavigateToPose";
      default: return "unknown";
    }
  }

  void observeExecutionStatus(const action_msgs::msg::GoalStatusArray & message)
  {
    const auto * latest = latestStatus(message);
    if (!latest) {
      return;
    }
    if (have_execution_identity_ &&
      stampNewer(execution_goal_stamp_, latest->goal_info.stamp))
    {
      return;
    }
    const auto next_id = uuidText(latest->goal_info.goal_id.uuid);
    const bool changed = !have_execution_identity_ || execution_goal_id_ != next_id ||
      execution_status_ != latest->status;
    execution_goal_id_ = next_id;
    execution_goal_stamp_ = latest->goal_info.stamp;
    execution_status_ = latest->status;
    have_execution_identity_ = true;
    have_execution_goal_ = isActive(latest->status);
    if (changed) {
      RCLCPP_INFO(
        get_logger(), "Navigation execution identity: execution_id=%s status=%s "
        "stamp=%d.%09u", execution_goal_id_.c_str(), statusText(latest->status),
        latest->goal_info.stamp.sec, latest->goal_info.stamp.nanosec);
    }
    if (!have_execution_goal_ && have_navigation_goal_ && isTerminal(latest->status) &&
      latest->status != action_msgs::msg::GoalStatus::STATUS_SUCCEEDED)
    {
      const bool current_global_path = have_global_displayed_path_ &&
        global_display_execution_id_ == execution_goal_id_;
      const bool current_local_path = have_local_displayed_path_ &&
        local_display_execution_id_ == execution_goal_id_;
      if (current_global_path) {
        clearDisplayedPath(
          true, "NavigateToPoseInternal", statusText(latest->status), latest,
          "execution_interrupted_while_goal_active");
      }
      if (current_local_path) {
        clearDisplayedPath(
          false, "NavigateToPoseInternal", statusText(latest->status), latest,
          "execution_interrupted_while_goal_active");
      }
      if (!current_global_path || !current_local_path) {
        RCLCPP_INFO(
          get_logger(), "Navigation display clear ignored: reason=execution_identity_mismatch "
          "execution_id=%s global_match=%s local_match=%s",
          execution_goal_id_.c_str(), current_global_path ? "true" : "false",
          current_local_path ? "true" : "false");
      }
    }
  }

  void reconcileDirectPathDisplay(
    NavigationAction action, const action_msgs::msg::GoalStatus * latest)
  {
    if (latest == nullptr) {
      return;
    }
    const auto * compute_goal = direct_path_association_.currentComputeGoal();
    const auto * follow_goal = direct_path_association_.currentFollowGoal();
    if (have_global_displayed_path_ &&
      (compute_goal == nullptr ||
      global_display_execution_id_ != uuidText(compute_goal->goal_info.goal_id.uuid) ||
      !direct_path_association_.globalPathMatchesCurrentGoal(global_displayed_path_.stamp)))
    {
      clearDisplayedPath(
        true, actionName(action), statusText(latest->status), latest,
        "compute_path_handoff_changed");
    }
    if (have_local_displayed_path_ &&
      (follow_goal == nullptr ||
      local_display_execution_id_ != uuidText(follow_goal->goal_info.goal_id.uuid) ||
      !direct_path_association_.localPathMatchesCurrentGoal(local_displayed_path_.stamp)))
    {
      clearDisplayedPath(
        false, actionName(action), statusText(latest->status), latest,
        "follow_path_handoff_changed");
    }
  }

  bool pathIsCurrent(
    const nav_msgs::msg::Path & path, const bool global, const char * & reason) const
  {
    bool endpoint_frames_differ = false;
    if (navigation_paused_) {
      reason = "navigation_paused";
      return false;
    }
    if (!have_navigation_goal_) {
      reason = "public_goal_inactive";
      return false;
    }
    if (!direct_nav2_mode_) {
      if (!have_execution_goal_) {
        reason = "internal_execution_inactive";
        return false;
      }
      if (!have_goal_identity_) {
        reason = "identity_mapping_missing";
        return false;
      }
      if (mapped_navigation_goal_id_ != navigation_goal_id_) {
        reason = "public_goal_identity_mismatch";
        return false;
      }
      if (mapped_execution_goal_id_ != execution_goal_id_) {
        reason = "execution_identity_mismatch";
        return false;
      }
    }
    if (path.poses.empty()) {
      reason = "empty_path";
      return false;
    }
    if (path.header.frame_id.empty()) {
      reason = "empty_path_frame";
      return false;
    }
    if (direct_nav2_mode_) {
      const bool current_path = global ?
        direct_path_association_.globalPathMatchesCurrentGoal(path.header.stamp) :
        direct_path_association_.localPathMatchesCurrentGoal(path.header.stamp);
      if (!current_path) {
        reason = global ? "compute_path_handoff_missing" : "follow_path_handoff_missing";
        return false;
      }
    } else if (stampOlder(path.header.stamp, navigation_goal_stamp_)) {
      reason = "path_before_public_goal";
      return false;
    }
    if (!direct_nav2_mode_ && stampOlder(path.header.stamp, execution_goal_stamp_)) {
      reason = "path_before_execution_goal";
      return false;
    }
    if (have_requested_goal_ && global) {
      const auto & end = path.poses.back().pose.position;
      const auto endpoint_result = nav2_theta_star_planner::checkGoalCellEndpoint(
        path.header.frame_id, path.poses.size(), end.x, end.y,
        requested_goal_.header.frame_id, requested_goal_.pose.position.x,
        requested_goal_.pose.position.y);
      if (endpoint_result ==
        nav2_theta_star_planner::GoalCellEndpointResult::ENDPOINT_MISMATCH)
      {
        reason = "goal_endpoint_mismatch";
        return false;
      }
      endpoint_frames_differ = endpoint_result ==
        nav2_theta_star_planner::GoalCellEndpointResult::DIFFERENT_FRAMES;
    }
    if (have_requested_goal_) {
      const rclcpp::Time path_stamp(path.header.stamp, get_clock()->get_clock_type());
      if (path_stamp < requested_goal_received_at_) {
        reason = "path_before_requested_goal";
        return false;
      }
    }
    reason = endpoint_frames_differ ? "current_goal_identity_valid_frames_differ" :
      "current_goal";
    return true;
  }

  void forwardPath(const nav_msgs::msg::Path::ConstSharedPtr & message, const bool global)
  {
    const auto started = SteadyClock::now();
    if (global) {
      ++global_received_;
    } else {
      ++local_received_;
    }
    const char * reason = "unknown";
    const bool forward = pathIsCurrent(*message, global, reason);
    if (forward) {
      if (global) {
        global_display_pub_->publish(*message);
        ++global_forwarded_;
        have_global_displayed_path_ = !message->poses.empty();
        global_display_navigation_goal_id_ = navigation_goal_id_;
        const auto * compute_goal = direct_nav2_mode_ ?
          direct_path_association_.currentComputeGoal() : nullptr;
        global_display_execution_id_ = compute_goal == nullptr ? execution_goal_id_ :
          uuidText(compute_goal->goal_info.goal_id.uuid);
        global_displayed_path_ = describePath(*message);
      } else {
        local_display_pub_->publish(*message);
        ++local_forwarded_;
        have_local_displayed_path_ = !message->poses.empty();
        local_display_navigation_goal_id_ = navigation_goal_id_;
        const auto * follow_goal = direct_nav2_mode_ ?
          direct_path_association_.currentFollowGoal() : nullptr;
        local_display_execution_id_ = follow_goal == nullptr ? execution_goal_id_ :
          uuidText(follow_goal->goal_info.goal_id.uuid);
        local_displayed_path_ = describePath(*message);
      }
    } else if (global) {
      ++global_discarded_;
    } else {
      ++local_discarded_;
    }

    const auto end_text = message->poses.empty() ? std::string("empty") :
      std::to_string(message->poses.back().pose.position.x) + "," +
      std::to_string(message->poses.back().pose.position.y);
    const auto start_text = message->poses.empty() ? std::string("empty") :
      std::to_string(message->poses.front().pose.position.x) + "," +
      std::to_string(message->poses.front().pose.position.y);
    if (global || SteadyClock::now() - last_local_path_log_ >= std::chrono::seconds(1)) {
      last_local_path_log_ = SteadyClock::now();
      RCLCPP_INFO(
        get_logger(), "Navigation path display: stream=%s result=%s reason=%s "
        "goal_id=%s execution_id=%s path_stamp=%d.%09u frame=%s points=%zu "
        "start=(%s) end=(%s)", global ? "global" : "local",
        forward ? "forwarded" : "discarded", reason,
        navigation_goal_id_.c_str(), execution_goal_id_.c_str(), message->header.stamp.sec,
        message->header.stamp.nanosec, message->header.frame_id.c_str(),
        message->poses.size(), start_text.c_str(), end_text.c_str());
    }
    accountDisplayWork(started);
  }

  void accountDisplayWork(const SteadyClock::time_point started)
  {
    const double elapsed_ms = std::chrono::duration<double, std::milli>(
      SteadyClock::now() - started).count();
    display_callback_total_ms_ += elapsed_ms;
    display_callback_max_ms_ = std::max(display_callback_max_ms_, elapsed_ms);
    ++display_callback_count_;
    const auto now_steady = SteadyClock::now();
    if (now_steady - last_display_stats_ >= std::chrono::seconds(5)) {
      RCLCPP_INFO(
        get_logger(), "Navigation display load: window_s=%.1f callbacks=%zu "
        "global=%zu/%zu/%zu local=%zu/%zu/%zu callback_total=%.3f ms "
        "callback_max=%.3f ms",
        std::chrono::duration<double>(now_steady - last_display_stats_).count(),
        display_callback_count_, global_received_, global_forwarded_, global_discarded_,
        local_received_, local_forwarded_, local_discarded_, display_callback_total_ms_,
        display_callback_max_ms_);
      last_display_stats_ = now_steady;
      display_callback_count_ = 0;
      global_received_ = 0;
      global_forwarded_ = 0;
      global_discarded_ = 0;
      local_received_ = 0;
      local_forwarded_ = 0;
      local_discarded_ = 0;
      display_callback_total_ms_ = 0.0;
      display_callback_max_ms_ = 0.0;
    }
  }

  static DisplayedPathInfo describePath(const nav_msgs::msg::Path & path)
  {
    DisplayedPathInfo info;
    info.stamp = path.header.stamp;
    info.frame_id = path.header.frame_id;
    info.point_count = path.poses.size();
    if (!path.poses.empty()) {
      const auto & start = path.poses.front().pose.position;
      const auto & end = path.poses.back().pose.position;
      info.start_x = start.x;
      info.start_y = start.y;
      info.end_x = end.x;
      info.end_y = end.y;
    }
    return info;
  }

  void clearDisplayedPath(
    const bool global, const char * action, const char * status,
    const action_msgs::msg::GoalStatus * goal, const char * reason)
  {
    if (global) {
      if (!have_global_displayed_path_) {
        return;
      }
      const bool public_navigation_event = std::string(action) == "NavigateToPose";
      std::string terminal_execution_id = execution_goal_id_;
      std::string current_execution_id = execution_goal_id_;
      if (direct_nav2_mode_ && std::string(action) == "ComputePathToPose" && goal != nullptr) {
        terminal_execution_id = uuidText(goal->goal_info.goal_id.uuid);
        const auto * current_compute_goal = direct_path_association_.latestComputeGoal();
        current_execution_id = current_compute_goal == nullptr ? terminal_execution_id :
          uuidText(current_compute_goal->goal_info.goal_id.uuid);
      }
      if (!public_navigation_event) {
        const auto disposition = globalPathTerminalDisposition(
          have_navigation_goal_, global_display_navigation_goal_id_, navigation_goal_id_,
          global_display_execution_id_, terminal_execution_id, current_execution_id);
        if (disposition != GlobalPathTerminalDisposition::CLEAR) {
          RCLCPP_INFO(
            get_logger(), "Navigation global route retained or stale terminal ignored: "
            "action=%s status=%s reason=%s disposition=%s public_goal=%s "
            "path_goal=%s terminal_execution=%s current_execution=%s path_execution=%s",
            action, status, reason,
            disposition == GlobalPathTerminalDisposition::RETAIN ? "retain" : "ignore",
            navigation_goal_id_.c_str(), global_display_navigation_goal_id_.c_str(),
            terminal_execution_id.c_str(), current_execution_id.c_str(),
            global_display_execution_id_.c_str());
          return;
        }
      }
      publishEmptyPath(
        global_display_pub_, action, status, goal, reason,
        global_display_navigation_goal_id_, global_display_execution_id_, global_displayed_path_);
      have_global_displayed_path_ = false;
      global_display_navigation_goal_id_.clear();
      global_display_execution_id_.clear();
      global_displayed_path_ = DisplayedPathInfo();
    } else {
      if (!have_local_displayed_path_) {
        return;
      }
      publishEmptyPath(
        local_display_pub_, action, status, goal, reason,
        local_display_navigation_goal_id_, local_display_execution_id_, local_displayed_path_);
      have_local_displayed_path_ = false;
      local_display_navigation_goal_id_.clear();
      local_display_execution_id_.clear();
      local_displayed_path_ = DisplayedPathInfo();
    }
  }

  void clearLocalPathForPause()
  {
    if (!have_local_displayed_path_) {
      return;
    }
    nav_msgs::msg::Path empty;
    empty.header.frame_id = local_displayed_path_.frame_id.empty() ?
      "odom" : local_displayed_path_.frame_id;
    empty.header.stamp = now();
    local_display_pub_->publish(empty);
    RCLCPP_INFO(
      get_logger(), "Navigation local route cleared while paused: goal_id=%s "
      "execution_id=%s source_stamp=%d.%09u",
      local_display_navigation_goal_id_.c_str(), local_display_execution_id_.c_str(),
      local_displayed_path_.stamp.sec, local_displayed_path_.stamp.nanosec);
    have_local_displayed_path_ = false;
    local_display_navigation_goal_id_.clear();
    local_display_execution_id_.clear();
    local_displayed_path_ = DisplayedPathInfo();
  }

  void publishEmptyPath(
    const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr & publisher,
    const char * action, const char * status, const action_msgs::msg::GoalStatus * goal,
    const char * reason, const std::string & path_navigation_goal_id,
    const std::string & path_execution_id, const DisplayedPathInfo & path_info)
  {
    std_msgs::msg::Header header;
    header.frame_id = path_info.frame_id.empty() ? "odom" : path_info.frame_id;
    header.stamp = now();
    nav_msgs::msg::Path empty;
    empty.header = header;
    publisher->publish(empty);
    RCLCPP_INFO(
      get_logger(), "Navigation path display cleared: action=%s status=%s terminal_id=%s "
      "path_goal_id=%s path_execution_id=%s path_stamp=%d.%09u frame=%s points=%zu "
      "start=(%.3f,%.3f) end=(%.3f,%.3f) clear_stamp=%d.%09u reason=%s",
      action, status, uuidText(goal->goal_info.goal_id.uuid).c_str(),
      path_navigation_goal_id.c_str(), path_execution_id.c_str(), path_info.stamp.sec,
      path_info.stamp.nanosec, path_info.frame_id.c_str(), path_info.point_count,
      path_info.start_x, path_info.start_y, path_info.end_x, path_info.end_y,
      header.stamp.sec, header.stamp.nanosec, reason);
  }

  PathDisplayState state_;
  DirectNav2PathAssociation direct_path_association_;
  bool direct_nav2_mode_{false};
  geometry_msgs::msg::PoseStamped requested_goal_;
  builtin_interfaces::msg::Time navigation_goal_stamp_;
  builtin_interfaces::msg::Time execution_goal_stamp_;
  rclcpp::Time requested_goal_received_at_;
  std::string navigation_goal_id_;
  std::string execution_goal_id_;
  std::string goal_identity_generation_;
  std::string mapped_navigation_goal_id_;
  std::string mapped_execution_goal_id_;
  std::string global_display_navigation_goal_id_;
  std::string local_display_navigation_goal_id_;
  std::string global_display_execution_id_;
  std::string local_display_execution_id_;
  DisplayedPathInfo global_displayed_path_;
  DisplayedPathInfo local_displayed_path_;
  std::uint8_t execution_status_{action_msgs::msg::GoalStatus::STATUS_UNKNOWN};
  bool have_navigation_identity_{false};
  bool have_execution_identity_{false};
  bool have_goal_identity_{false};
  bool have_navigation_goal_{false};
  bool have_execution_goal_{false};
  bool have_global_displayed_path_{false};
  bool have_local_displayed_path_{false};
  bool navigation_paused_{true};
  bool have_requested_goal_{false};
  SteadyClock::time_point last_local_path_log_{};
  SteadyClock::time_point last_display_stats_{};
  std::size_t global_received_{0};
  std::size_t global_forwarded_{0};
  std::size_t global_discarded_{0};
  std::size_t local_received_{0};
  std::size_t local_forwarded_{0};
  std::size_t local_discarded_{0};
  std::size_t display_callback_count_{0};
  double display_callback_total_ms_{0.0};
  double display_callback_max_ms_{0.0};
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr global_display_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr local_display_pub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr global_path_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr local_path_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr requested_goal_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr navigation_ready_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr goal_identity_sub_;
  rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr compute_status_sub_;
  rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr follow_status_sub_;
  rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr navigate_status_sub_;
  rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr execution_status_sub_;
};

}  // namespace bringup

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<bringup::Nav2PathDisplayNode>());
  rclcpp::shutdown();
  return 0;
}
