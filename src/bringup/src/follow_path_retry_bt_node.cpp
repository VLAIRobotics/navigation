#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <memory>
#include <sstream>
#include <vector>

#include <action_msgs/msg/goal_status_array.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>

#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/control_node.h>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int64.hpp>

#include "nav2_theta_star_planner/goal_cell_endpoint.hpp"

namespace bringup
{
class FollowPathRetry : public BT::ControlNode
{
public:
  FollowPathRetry(const std::string & name, const BT::NodeConfiguration & config)
  : BT::ControlNode(name, config), node_(config.blackboard->get<rclcpp::Node::SharedPtr>("node"))
  {
    group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
    executor_.add_callback_group(group_, node_->get_node_base_interface());
    rclcpp::SubscriptionOptions options;
    options.callback_group = group_;
    ack_ = node_->create_subscription<std_msgs::msg::UInt64>(
      "/navigation/follow_path_attempt_ack", rclcpp::QoS(10).reliable(),
      [this](std_msgs::msg::UInt64::SharedPtr msg) {ack_id_ = msg->data;}, options);
    progress_ = node_->create_subscription<std_msgs::msg::UInt64>(
      "/navigation/follow_path_progress", rclcpp::QoS(10).reliable(),
      [this](std_msgs::msg::UInt64::SharedPtr msg) {progress_id_ = msg->data;}, options);
    teb_failure_ = node_->create_subscription<std_msgs::msg::String>(
      "/navigation/teb_failure_diagnostic", rclcpp::QoS(1).reliable().transient_local(),
      [this](std_msgs::msg::String::SharedPtr msg) {last_collision_ = msg->data;}, options);
    goals_ = node_->create_subscription<action_msgs::msg::GoalStatusArray>(
      "/navigate_to_pose_internal/_action/status", rclcpp::QoS(10).reliable(),
      [this](action_msgs::msg::GoalStatusArray::SharedPtr msg) {
        const action_msgs::msg::GoalStatus * newest = nullptr;
        for (const auto & status : msg->status_list) {
          if (status.status != action_msgs::msg::GoalStatus::STATUS_ACCEPTED &&
            status.status != action_msgs::msg::GoalStatus::STATUS_EXECUTING) {continue;}
          if (!newest || status.goal_info.stamp.sec > newest->goal_info.stamp.sec ||
            (status.goal_info.stamp.sec == newest->goal_info.stamp.sec &&
            status.goal_info.stamp.nanosec > newest->goal_info.stamp.nanosec)) {newest = &status;}
        }
        if (newest) {
          pending_goal_.assign(newest->goal_info.goal_id.uuid.begin(),
            newest->goal_info.goal_id.uuid.end());
          pending_goal_stamp_ = newest->goal_info.stamp;
        }
      }, options);
    publisher_ = node_->create_publisher<std_msgs::msg::UInt64>(
      "/navigation/follow_path_attempt", rclcpp::QoS(10).reliable());
    local_status_publisher_ = node_->create_publisher<std_msgs::msg::String>(
      "/navigation/local_wait_status", rclcpp::QoS(1).reliable().transient_local());
  }
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<geometry_msgs::msg::PoseStamped>("goal", "Current NavigateToPose target"),
      BT::BidirectionalPort<nav_msgs::msg::Path>(
        "path", "Current ComputePathToPose result")};
  }

  BT::NodeStatus tick() override
  {
    if (children_nodes_.size() != 2) {
      throw BT::LogicError("FollowPathRetry requires FollowPath and goal check children");
    }
    setStatus(BT::NodeStatus::RUNNING);
    executor_.spin_some();
    const auto now = Clock::now();
    geometry_msgs::msg::PoseStamped target;
    nav_msgs::msg::Path path;
    if (!getInput("goal", target) || !getInput("path", path)) {
      RCLCPP_ERROR(node_->get_logger(),
        "Navigation recovery: category=path_identity_unavailable goal_or_path_missing=true");
      return BT::NodeStatus::FAILURE;
    }
    bool path_invalidated = false;
    if (have_goal_pose_ && !sameGoalPose(target, goal_pose_)) {
      haltChildren();
      reset();
      awaiting_goal_switch_ = !goal_.empty();
      awaiting_fresh_path_ = true;
      path_wait_started_ = now;
      last_collision_.clear();
      publishLocalStatus("planning", "goal_pose_changed");
      goal_pose_ = target;
      have_goal_pose_ = true;
      invalidatePath(target);
      path_invalidated = true;
      RCLCPP_INFO(node_->get_logger(),
        "Navigation recovery: category=goal_pose_changed; waiting for matching action and plan");
    }
    if (!have_goal_pose_) {
      goal_pose_ = target;
      have_goal_pose_ = true;
      path_baseline_stamp_ = builtin_interfaces::msg::Time();
      awaiting_fresh_path_ = true;
      path_wait_started_ = now;
      publishLocalStatus("planning", "new_goal");
      if (!path.poses.empty() && !pathMatchesCurrentGoal(path, target)) {
        invalidatePath(target);
        path_invalidated = true;
      }
    }
    if (!pending_goal_.empty() && pending_goal_ != goal_) {
      haltChildren();
      reset();
      goal_ = pending_goal_;
      goal_started_stamp_ = pending_goal_stamp_;
      awaiting_goal_switch_ = false;
      awaiting_fresh_path_ = true;
      path_wait_started_ = now;
      last_collision_.clear();
      publishLocalStatus("planning", "goal_identity_changed");
      if (!pathMatchesCurrentGoal(path, target)) {
        invalidatePath(target);
        path_invalidated = true;
      }
      goal_pose_ = target;
      have_goal_pose_ = true;
      RCLCPP_INFO(node_->get_logger(),
        "Navigation recovery: category=goal_identity_changed; stopping old FollowPath and waiting for a new plan");
    }
    if (path_invalidated) {
      return BT::NodeStatus::RUNNING;
    }
    if (goal_.empty() || awaiting_goal_switch_) {
      if (pathWaitExpired(now)) {
        return failPathWait(
          goal_.empty() ? "goal_identity_timeout" : "goal_switch_timeout", path);
      }
      return BT::NodeStatus::RUNNING;
    }
    if (path.poses.empty()) {
      if (!awaiting_fresh_path_) {
        haltChildren();
        reset();
        awaiting_fresh_path_ = true;
        path_wait_started_ = now;
        path_baseline_stamp_ = path.header.stamp;
      }
      retry_episode_active_ = true;
      publishLocalStatus("blocked_waiting_retry", "global_path_unavailable");
      return BT::NodeStatus::RUNNING;
    }
    if (awaiting_fresh_path_) {
      if (!pathMatchesCurrentGoal(path, target)) {
        if (pathWaitExpired(now)) {
          return failPathWait("matching_plan_timeout", path);
        }
        return BT::NodeStatus::RUNNING;
      }
      awaiting_fresh_path_ = false;
      const auto endpoint_contract = path.header.frame_id == target.header.frame_id ?
        "exact_goal_or_same_cell" : "identity_valid_frames_differ";
      RCLCPP_INFO(node_->get_logger(),
        "Navigation recovery: category=current_goal_path_ready endpoint_contract=%s "
        "path_points=%zu stamp=%d.%09u path_frame=%s target_frame=%s",
        endpoint_contract, path.poses.size(), path.header.stamp.sec,
        path.header.stamp.nanosec, path.header.frame_id.c_str(),
        target.header.frame_id.c_str());
      if (retry_episode_active_) {
        publishLocalStatus("retrying", "fresh_path_ready");
      }
    }
    if (retry_episode_active_ && !waiting_ && attempt_id_ != 0 &&
      progress_id_ == attempt_id_)
    {
      retry_episode_active_ = false;
      RCLCPP_INFO(node_->get_logger(),
        "Navigation recovery: category=local_wait_released reason=measured_route_progress "
        "attempt=%llu",
        static_cast<unsigned long long>(attempt_id_));
      last_collision_.clear();
      publishLocalStatus("executing", "measured_route_progress");
    }
    if (waiting_) {
      if (now - wait_started_ < std::chrono::seconds(2)) {
        publishLocalStatus("blocked_waiting_retry", "FollowPath_failure_goal_still_valid");
        return BT::NodeStatus::RUNNING;
      }
      waiting_ = false;
      publishLocalStatus("retrying", "retry_interval_elapsed");
    }
    if (attempt_id_ == 0) {
      static std::atomic<std::uint64_t> last{0};
      const auto time_id = static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count());
      auto previous = last.load();
      do {
        attempt_id_ = std::max(time_id, previous + 1);
      } while (!last.compare_exchange_weak(previous, attempt_id_));
      ack_started_ = now;
    }
    if (ack_id_ != attempt_id_) {
      if (now - ack_started_ >= std::chrono::seconds(3)) {
        publishLocalStatus("failed", "TEB_ack_timeout");
        RCLCPP_ERROR(node_->get_logger(),
          "Navigation recovery: category=attempt_reset_failed reason=TEB_ack_timeout");
        haltChildren();
        reset();
        return BT::NodeStatus::FAILURE;
      }
      std_msgs::msg::UInt64 msg;
      msg.data = attempt_id_;
      publisher_->publish(msg);
      return BT::NodeStatus::RUNNING;
    }
    publishLocalStatus("executing", "follow_path_started");
    const auto result = children_nodes_[0]->executeTick();
    if (result == BT::NodeStatus::RUNNING) {return result;}
    if (result == BT::NodeStatus::SUCCESS) {
      haltChildren();
      publishLocalStatus("executing", "local_path_completed");
      last_collision_.clear();
      reset();
      return result;
    }
    haltChild(0);
    retry_episode_active_ = true;
    if (children_nodes_[1]->executeTick() != BT::NodeStatus::SUCCESS) {
      haltChildren();
      reset();
      retry_episode_active_ = true;
      awaiting_fresh_path_ = true;
      path_wait_started_ = now;
      invalidatePath(target);
      wait_started_ = now;
      waiting_ = true;
      publishLocalStatus("blocked_waiting_retry", "goal_or_map_check_unavailable");
      RCLCPP_WARN(node_->get_logger(),
        "Navigation recovery: category=blocked_waiting reason=goal_or_map_check_unavailable "
        "stopped=true retrying_with_global_planner=true");
      return BT::NodeStatus::RUNNING;
    }
    haltChild(1);
    // The controller rejected this attempt. Clear the old reference path so
    // GlobalPlanRetry replans against the current costmap during the wait.
    invalidatePath(target);
    awaiting_fresh_path_ = true;
    path_wait_started_ = now;
    RCLCPP_WARN(node_->get_logger(),
      "Navigation recovery: category=local_wait reason=FollowPath_failure_goal_still_valid "
      "stopped=true retry_after=2s global_replan=true teb_diagnostic='%s'",
      last_collision_.c_str());
    attempt_id_ = 0;
    wait_started_ = now;
    waiting_ = true;
    publishLocalStatus("blocked_waiting_retry", "FollowPath_failure_goal_still_valid");
    return BT::NodeStatus::RUNNING;
  }
  void halt() override
  {
    haltChildren();
    publishLocalStatus("stopped", "behavior_tree_halted");
    reset();
    goal_.clear();
    pending_goal_.clear();
    have_goal_pose_ = false;
    awaiting_fresh_path_ = true;
    awaiting_goal_switch_ = false;
    path_wait_started_ = Clock::time_point{};
    BT::ControlNode::halt();
  }
private:
  using Clock = std::chrono::steady_clock;
  static constexpr auto kFreshPathWaitTimeout = std::chrono::seconds(5);

  static std::string goalIdText(const std::vector<std::uint8_t> & goal)
  {
    std::ostringstream output;
    output << std::hex << std::setfill('0');
    for (const auto byte : goal) {
      output << std::setw(2) << static_cast<unsigned int>(byte);
    }
    return output.str();
  }

  bool pathWaitExpired(const Clock::time_point now) const
  {
    return path_wait_started_ != Clock::time_point{} &&
      now - path_wait_started_ >= kFreshPathWaitTimeout;
  }

  BT::NodeStatus failPathWait(const char * reason, const nav_msgs::msg::Path & path)
  {
    publishLocalStatus("failed", reason);
    RCLCPP_ERROR(node_->get_logger(),
      "Navigation recovery: category=fresh_path_wait_timeout reason=%s timeout=5s "
      "goal_identity_bytes=%zu path_points=%zu goal=(%.3f,%.3f,%s) path_frame=%s",
      reason, goal_.size(), path.poses.size(), goal_pose_.pose.position.x,
      goal_pose_.pose.position.y, goal_pose_.header.frame_id.c_str(),
      path.header.frame_id.c_str());
    haltChildren();
    reset();
    return BT::NodeStatus::FAILURE;
  }

  void invalidatePath(const geometry_msgs::msg::PoseStamped & target)
  {
    nav_msgs::msg::Path empty_path;
    empty_path.header.frame_id = target.header.frame_id;
    empty_path.header.stamp = static_cast<builtin_interfaces::msg::Time>(node_->now());
    setOutput("path", empty_path);
    path_baseline_stamp_ = empty_path.header.stamp;
  }

  static bool sameGoalPose(
    const geometry_msgs::msg::PoseStamped & lhs,
    const geometry_msgs::msg::PoseStamped & rhs)
  {
    const auto close = [](double a, double b) {return std::abs(a - b) <= 1.0e-4;};
    return lhs.header.frame_id == rhs.header.frame_id &&
      close(lhs.pose.position.x, rhs.pose.position.x) &&
      close(lhs.pose.position.y, rhs.pose.position.y) &&
      close(lhs.pose.position.z, rhs.pose.position.z) &&
      close(lhs.pose.orientation.x, rhs.pose.orientation.x) &&
      close(lhs.pose.orientation.y, rhs.pose.orientation.y) &&
      close(lhs.pose.orientation.z, rhs.pose.orientation.z) &&
      close(lhs.pose.orientation.w, rhs.pose.orientation.w);
  }

  bool pathMatchesCurrentGoal(
    const nav_msgs::msg::Path & path,
    const geometry_msgs::msg::PoseStamped & target) const
  {
    if (path.poses.empty() || path.header.frame_id.empty()) {
      return false;
    }
    const auto older = [](const builtin_interfaces::msg::Time & lhs,
        const builtin_interfaces::msg::Time & rhs) {
        return lhs.sec < rhs.sec || (lhs.sec == rhs.sec && lhs.nanosec < rhs.nanosec);
      };
    if (older(path.header.stamp, path_baseline_stamp_) ||
      older(path.header.stamp, goal_started_stamp_))
    {
      return false;
    }
    const auto & endpoint = path.poses.back().pose.position;
    const auto endpoint_result = nav2_theta_star_planner::checkGoalCellEndpoint(
      path.header.frame_id, path.poses.size(), endpoint.x, endpoint.y,
      target.header.frame_id, target.pose.position.x, target.pose.position.y);
    return endpoint_result !=
      nav2_theta_star_planner::GoalCellEndpointResult::EMPTY_PATH &&
      endpoint_result != nav2_theta_star_planner::GoalCellEndpointResult::ENDPOINT_MISMATCH;
  }

  void reset()
  {
    waiting_ = false;
    retry_episode_active_ = false;
    attempt_id_ = 0;
    progress_id_ = 0;
  }

  void publishLocalStatus(const std::string & state, const std::string & reason)
  {
    std::ostringstream output;
    output << "state=" << state;
    if (!goal_.empty()) {
      output << ";goal_id=" << goalIdText(goal_);
    }
    if (attempt_id_ != 0) {
      output << ";attempt_id=" << attempt_id_;
    }
    if (!reason.empty()) {
      output << ";reason=" << reason;
    }
    const auto now = Clock::now();
    if (!last_collision_.empty() &&
      (state == "blocked_waiting_retry" || state == "retrying" || state == "failed"))
    {
      output << ";last_collision=" << last_collision_;
    }
    const std::string value = output.str();
    const bool heartbeat_due = state == "blocked_waiting_retry" &&
      (last_status_publish_time_ == Clock::time_point{} ||
      now - last_status_publish_time_ >= std::chrono::seconds(1));
    if (!local_status_publisher_ ||
      (value == last_published_local_status_ && !heartbeat_due))
    {
      return;
    }
    std_msgs::msg::String message;
    message.data = value;
    local_status_publisher_->publish(message);
    last_published_local_status_ = value;
    last_status_publish_time_ = now;
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr group_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::UInt64>::SharedPtr ack_;
  rclcpp::Subscription<std_msgs::msg::UInt64>::SharedPtr progress_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr teb_failure_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr local_status_publisher_;
  rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr goals_;
  std::vector<std::uint8_t> goal_, pending_goal_;
  std::string last_collision_, last_published_local_status_;
  geometry_msgs::msg::PoseStamped goal_pose_;
  builtin_interfaces::msg::Time pending_goal_stamp_;
  builtin_interfaces::msg::Time goal_started_stamp_;
  builtin_interfaces::msg::Time path_baseline_stamp_;
  std::uint64_t attempt_id_{0}, ack_id_{0}, progress_id_{0};
  bool have_goal_pose_{false};
  bool awaiting_fresh_path_{true};
  bool awaiting_goal_switch_{false};
  bool waiting_{false};
  bool retry_episode_active_{false};
  Clock::time_point wait_started_, ack_started_, path_wait_started_;
  Clock::time_point last_status_publish_time_{};
};
}
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<bringup::FollowPathRetry>("FollowPathRetry");
}
