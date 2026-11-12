#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <exception>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <action_msgs/msg/goal_status_array.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/string.hpp>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/control_node.h>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include "nav2_theta_star_planner/goal_cell_endpoint.hpp"

namespace bringup
{
class GlobalPlanRetry : public BT::ControlNode
{
public:
  GlobalPlanRetry(const std::string & name, const BT::NodeConfiguration & config)
  : BT::ControlNode(name, config), node_(config.blackboard->get<rclcpp::Node::SharedPtr>("node"))
  {
    if (!node_->get_parameter("global_frame", global_frame_) || global_frame_.empty()) {
      throw BT::RuntimeError("GlobalPlanRetry requires the bt_navigator global_frame parameter");
    }
    double hz = 1.0;
    if (!getInput("retry_hz", hz) || !std::isfinite(hz) || hz <= 0.0 || hz > 10.0) {
      throw BT::RuntimeError("GlobalPlanRetry retry_hz must be in (0, 10]");
    }
    retry_period_ = std::chrono::duration_cast<Clock::duration>(
      std::chrono::duration<double>(1.0 / hz));
    group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
    executor_.add_callback_group(group_, node_->get_node_base_interface());
    rclcpp::SubscriptionOptions options;
    options.callback_group = group_;
    planner_status_ = node_->create_subscription<std_msgs::msg::String>(
      "/navigation/global_plan_status", rclcpp::QoS(1).reliable().transient_local(),
      [this](std_msgs::msg::String::ConstSharedPtr msg) {parsePlannerStatus(msg->data);}, options);
    goal_status_ = node_->create_subscription<action_msgs::msg::GoalStatusArray>(
      "/navigate_to_pose_internal/_action/status", rclcpp::QoS(10).reliable(),
      [this](action_msgs::msg::GoalStatusArray::ConstSharedPtr msg) {
        const action_msgs::msg::GoalStatus * newest = nullptr;
        for (const auto & status : msg->status_list) {
          if (status.status != action_msgs::msg::GoalStatus::STATUS_ACCEPTED &&
            status.status != action_msgs::msg::GoalStatus::STATUS_EXECUTING) continue;
          if (!newest || status.goal_info.stamp.sec > newest->goal_info.stamp.sec ||
            (status.goal_info.stamp.sec == newest->goal_info.stamp.sec &&
            status.goal_info.stamp.nanosec > newest->goal_info.stamp.nanosec)) newest = &status;
        }
        if (newest) {
          goal_id_.assign(newest->goal_info.goal_id.uuid.begin(),
            newest->goal_info.goal_id.uuid.end());
        }
      }, options);
    status_pub_ = node_->create_publisher<std_msgs::msg::String>(
      "/navigation/local_wait_status", rclcpp::QoS(1).reliable().transient_local());
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<geometry_msgs::msg::PoseStamped>("goal", "Current navigation goal"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>(
        "planning_goal", "Current goal with unique identity for this ComputePath request"),
      BT::BidirectionalPort<nav_msgs::msg::Path>("path", "Validated path for current goal"),
      BT::BidirectionalPort<nav_msgs::msg::Path>(
        "candidate_path", "New plan checked before execution"),
      BT::InputPort<double>("retry_hz", 1.0, "Rate for retrying global planning")};
  }

  BT::NodeStatus tick() override
  {
    if (children_nodes_.size() != 3) {
      throw BT::LogicError("GlobalPlanRetry requires start check, planner, and path check children");
    }
    executor_.spin_some();
    geometry_msgs::msg::PoseStamped goal;
    if (!getInput("goal", goal)) {
      RCLCPP_ERROR(node_->get_logger(),
        "Navigation planning: category=invalid_bt_input goal_missing=true");
      return BT::NodeStatus::FAILURE;
    }
    const auto q = goal.pose.orientation;
    const double q_norm = std::hypot(std::hypot(q.x, q.y), std::hypot(q.z, q.w));
    if (goal.header.frame_id.empty() || !std::isfinite(goal.pose.position.x) ||
      !std::isfinite(goal.pose.position.y) || !std::isfinite(goal.pose.position.z) ||
      !std::isfinite(q_norm) || q_norm < 1.0e-6)
    {
      RCLCPP_ERROR(node_->get_logger(), "Navigation planning: category=invalid_goal_pose");
      publishStatus("failed", "invalid_goal_pose");
      return BT::NodeStatus::FAILURE;
    }
    const bool action_goal_changed = !goal_id_.empty() && goal_id_ != owned_goal_id_;
    if (!have_goal_ || !sameGoal(goal, goal_) || action_goal_changed) {
      haltChildren();
      goal_ = goal;
      owned_goal_id_ = goal_id_;
      have_goal_ = true;
      have_path_ = false;
      stage_ = 0;
      planning_ = false;
      planner_request_active_ = false;
      planner_request_id_.clear();
      if (!clearPath(goal)) return BT::NodeStatus::FAILURE;
      next_attempt_ = Clock::now();
      if (goal_id_.empty()) {
        if (goal_id_wait_started_ == Clock::time_point{}) goal_id_wait_started_ = Clock::now();
        if (Clock::now() - goal_id_wait_started_ > std::chrono::seconds(5)) {
          RCLCPP_ERROR(node_->get_logger(),
            "Navigation planning: category=goal_identity_unavailable timeout=5s");
          publishStatus("failed", "goal_identity_unavailable");
          return BT::NodeStatus::FAILURE;
        }
        publishStatus("planning", "waiting_for_goal_identity");
        return BT::NodeStatus::RUNNING;
      }
      goal_id_wait_started_ = Clock::time_point{};
      return BT::NodeStatus::RUNNING;
    }
    if (goal_id_.empty()) {
      if (goal_id_wait_started_ == Clock::time_point{}) goal_id_wait_started_ = Clock::now();
      if (Clock::now() - goal_id_wait_started_ > std::chrono::seconds(5)) {
        RCLCPP_ERROR(node_->get_logger(),
          "Navigation planning: category=goal_identity_unavailable timeout=5s");
        publishStatus("failed", "goal_identity_unavailable");
        return BT::NodeStatus::FAILURE;
      }
      publishStatus("planning", "waiting_for_goal_identity");
      return BT::NodeStatus::RUNNING;
    }
    if (have_path_) {
      nav_msgs::msg::Path current_path;
      if (!getInput("path", current_path)) {
        RCLCPP_ERROR(node_->get_logger(),
          "Navigation planning: category=bt_input_read_failed port=path");
        publishStatus("failed", "bt_input_read_failed");
        return BT::NodeStatus::FAILURE;
      }
      if (current_path.poses.empty()) {
        have_path_ = false;
        planning_ = false;
        stage_ = 0;
        next_attempt_ = Clock::now();
        publishStatus("planning", "validated_path_invalidated");
      }
    }
    if (planner_result_waiting_) return resolvePlannerFailure(goal);
    if (!planning_) {
      if (Clock::now() < next_attempt_) {
        return have_path_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::RUNNING;
      }
      planning_ = true;
      stage_ = 0;
      nav_msgs::msg::Path empty;
      if (!writePath("candidate_path", empty)) return BT::NodeStatus::FAILURE;
      publishStatus("planning", "global_replan");
    }

    while (stage_ < 3) {
      if (stage_ == 1 && !planner_request_active_ && !beginPlannerRequest(goal)) {
        return BT::NodeStatus::FAILURE;
      }
      const auto result = children_nodes_[stage_]->executeTick();
      if (result == BT::NodeStatus::RUNNING) {
        return have_path_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::RUNNING;
      }
      if (result == BT::NodeStatus::FAILURE) {
        haltChild(stage_);
        if (stage_ == 1) {
          executor_.spin_some();
          if (const auto * outcome = matchingPlannerOutcome()) {
            planner_request_active_ = false;
            return handlePlannerOutcome(goal, *outcome);
          }
          if (!clearPath(goal)) return BT::NodeStatus::FAILURE;
          have_path_ = false;
          planner_result_waiting_ = true;
          planner_result_deadline_ = Clock::now() + std::chrono::milliseconds(500);
          publishStatus("blocked_waiting_retry", "awaiting_planner_result");
          return BT::NodeStatus::RUNNING;
        }
        return waitAndRetry(goal, stage_ == 0 ?
          "localization_or_costmap_unavailable" : "candidate_path_rejected");
      }
      haltChild(stage_);
      if (stage_ == 1) planner_request_active_ = false;
      ++stage_;
    }

    nav_msgs::msg::Path candidate;
    if (!getInput("candidate_path", candidate) || candidate.poses.empty() ||
      candidate.header.frame_id.empty())
    {
      RCLCPP_ERROR(node_->get_logger(),
        "Navigation planning: category=invalid_planner_result candidate_path_empty=true");
      publishStatus("failed", "invalid_planner_result");
      return BT::NodeStatus::FAILURE;
    }
    const auto & endpoint = candidate.poses.back().pose.position;
    const auto endpoint_status = nav2_theta_star_planner::checkGoalCellEndpoint(
      candidate.header.frame_id, candidate.poses.size(), endpoint.x, endpoint.y,
      goal.header.frame_id, goal.pose.position.x, goal.pose.position.y);
    if (endpoint_status !=
      nav2_theta_star_planner::GoalCellEndpointResult::MATCHED_REQUESTED_GOAL &&
      endpoint_status != nav2_theta_star_planner::GoalCellEndpointResult::SAME_CELL_PLAN)
    {
      RCLCPP_ERROR(node_->get_logger(),
        "Navigation planning: category=planner_goal_mismatch; stopping on invalid path");
      publishStatus("failed", "planner_goal_mismatch");
      return BT::NodeStatus::FAILURE;
    }
    if (!writePath("path", candidate)) return BT::NodeStatus::FAILURE;
    planning_ = false;
    have_path_ = true;
    next_attempt_ = Clock::now() + retry_period_;
    return BT::NodeStatus::SUCCESS;
  }

  void halt() override
  {
    haltChildren();
    have_goal_ = false;
    have_path_ = false;
    planning_ = false;
    planner_request_active_ = false;
    planner_request_id_.clear();
    planner_result_waiting_ = false;
    owned_goal_id_.clear();
    publishStatus("stopped", "behavior_tree_halted");
    BT::ControlNode::halt();
  }

private:
  using Clock = std::chrono::steady_clock;
  struct PlannerOutcome
  {
    std::string instance;
    std::string request_id;
    std::uint64_t sequence{0};
    std::string state;
    std::string frame;
    builtin_interfaces::msg::Time target_stamp;
    builtin_interfaces::msg::Time event_stamp;
    std::uint64_t event_steady_ns{0};
    geometry_msgs::msg::Pose target_pose;
  };

  static bool sameGoal(
    const geometry_msgs::msg::PoseStamped & lhs,
    const geometry_msgs::msg::PoseStamped & rhs)
  {
    const auto close = [](double a, double b) {return std::abs(a - b) <= 1.0e-5;};
    return lhs.header.frame_id == rhs.header.frame_id &&
      lhs.header.stamp.sec == rhs.header.stamp.sec &&
      lhs.header.stamp.nanosec == rhs.header.stamp.nanosec &&
      close(lhs.pose.position.x, rhs.pose.position.x) &&
      close(lhs.pose.position.y, rhs.pose.position.y) &&
      close(lhs.pose.position.z, rhs.pose.position.z) &&
      close(lhs.pose.orientation.x, rhs.pose.orientation.x) &&
      close(lhs.pose.orientation.y, rhs.pose.orientation.y) &&
      close(lhs.pose.orientation.z, rhs.pose.orientation.z) &&
      close(lhs.pose.orientation.w, rhs.pose.orientation.w);
  }

  static std::string field(const std::string & text, const std::string & key)
  {
    const auto start = text.find(key + "=");
    if (start == std::string::npos) return {};
    const auto value_start = start + key.size() + 1;
    const auto end = text.find(';', value_start);
    return text.substr(value_start, end == std::string::npos ? end : end - value_start);
  }

  static std::string requestId(const builtin_interfaces::msg::Time & stamp)
  {
    return "plan:" + std::to_string(stamp.sec) + ':' + std::to_string(stamp.nanosec);
  }

  void parsePlannerStatus(const std::string & text)
  {
    try {
      PlannerOutcome value;
      value.instance = field(text, "instance");
      value.sequence = std::stoull(field(text, "seq"));
      value.request_id = field(text, "request_id");
      value.state = field(text, "state");
      value.frame = field(text, "frame");
      value.target_stamp.sec = std::stoi(field(text, "target_sec"));
      value.target_stamp.nanosec = static_cast<std::uint32_t>(
        std::stoul(field(text, "target_nanosec")));
      value.target_pose.position.x = std::stod(field(text, "x"));
      value.target_pose.position.y = std::stod(field(text, "y"));
      value.target_pose.position.z = std::stod(field(text, "z"));
      value.target_pose.orientation.x = std::stod(field(text, "qx"));
      value.target_pose.orientation.y = std::stod(field(text, "qy"));
      value.target_pose.orientation.z = std::stod(field(text, "qz"));
      value.target_pose.orientation.w = std::stod(field(text, "qw"));
      value.event_stamp.sec = std::stoi(field(text, "event_sec"));
      value.event_stamp.nanosec = static_cast<std::uint32_t>(
        std::stoul(field(text, "event_nanosec")));
      value.event_steady_ns = std::stoull(field(text, "event_steady_ns"));
      if (value.instance.empty() || value.sequence == 0 || value.frame.empty() ||
        value.target_stamp.sec < 0 || value.target_stamp.nanosec >= 1000000000U ||
        value.request_id != requestId(value.target_stamp))
      {
        throw std::invalid_argument("invalid planner instance, request identity, or target");
      }
      const auto duplicate = std::find_if(
        planner_outcomes_.begin(), planner_outcomes_.end(), [&value](const PlannerOutcome & old) {
          return old.request_id == value.request_id && old.instance == value.instance &&
                 old.sequence == value.sequence;
        });
      if (duplicate == planner_outcomes_.end()) planner_outcomes_.push_back(std::move(value));
      while (planner_outcomes_.size() > 32) planner_outcomes_.pop_front();
    } catch (const std::exception &) {
      RCLCPP_ERROR(node_->get_logger(),
        "Navigation planning: category=invalid_planner_status message='%s'", text.c_str());
    }
  }

  const PlannerOutcome * matchingPlannerOutcome() const
  {
    if (!planner_request_active_ || planner_request_id_.empty()) return nullptr;
    const auto same_time = [](const builtin_interfaces::msg::Time & lhs,
        const builtin_interfaces::msg::Time & rhs) {
        return lhs.sec == rhs.sec && lhs.nanosec == rhs.nanosec;
      };
    const auto close = [](const double lhs, const double rhs) {
        return std::isfinite(lhs) && std::isfinite(rhs) && std::abs(lhs - rhs) <= 1.0e-7;
      };
    for (auto it = planner_outcomes_.rbegin(); it != planner_outcomes_.rend(); ++it) {
      if (it->event_steady_ns < planner_request_started_steady_ns_ ||
        it->request_id != planner_request_id_ ||
        it->frame != planner_goal_.header.frame_id ||
        !same_time(it->target_stamp, planner_goal_.header.stamp)) continue;
      const auto & lhs = it->target_pose;
      const auto & rhs = planner_goal_.pose;
      if (close(lhs.position.x, rhs.position.x) &&
        close(lhs.position.y, rhs.position.y) && close(lhs.position.z, rhs.position.z) &&
        close(lhs.orientation.x, rhs.orientation.x) &&
        close(lhs.orientation.y, rhs.orientation.y) &&
        close(lhs.orientation.z, rhs.orientation.z) &&
        close(lhs.orientation.w, rhs.orientation.w)) return &(*it);
    }
    return nullptr;
  }

  bool beginPlannerRequest(const geometry_msgs::msg::PoseStamped & goal)
  {
    if (goal.header.frame_id != global_frame_) {
      RCLCPP_ERROR(node_->get_logger(),
        "Navigation planning: category=goal_frame_mismatch request_frame=%s global_frame=%s",
        goal.header.frame_id.c_str(), global_frame_.c_str());
      publishStatus("failed", "goal_frame_mismatch");
      return false;
    }
    executor_.spin_some();
    planner_request_started_steady_ns_ = static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
      Clock::now().time_since_epoch()).count());
    const auto ros_now = node_->get_clock()->now().nanoseconds();
    const auto ros_request_stamp_ns = ros_now > 0 ?
      static_cast<std::uint64_t>(ros_now) : 1U;
    static std::atomic<std::uint64_t> last_request_stamp_ns{0};
    auto previous = last_request_stamp_ns.load(std::memory_order_relaxed);
    std::uint64_t request_stamp_ns;
    do {
      request_stamp_ns = std::max(ros_request_stamp_ns, previous + 1);
    } while (!last_request_stamp_ns.compare_exchange_weak(
        previous, request_stamp_ns, std::memory_order_relaxed));
    planner_goal_ = goal;
    planner_goal_.header.stamp.sec = static_cast<std::int32_t>(request_stamp_ns / 1000000000ULL);
    planner_goal_.header.stamp.nanosec = static_cast<std::uint32_t>(
      request_stamp_ns % 1000000000ULL);
    planner_request_id_ = requestId(planner_goal_.header.stamp);
    const auto write_result = setOutput("planning_goal", planner_goal_);
    if (!write_result) {
      RCLCPP_ERROR(node_->get_logger(),
        "Navigation planning: category=bt_output_write_failed port=planning_goal error=%s",
        write_result.error().c_str());
      publishStatus("failed", "bt_output_write_failed");
      return false;
    }
    planner_request_active_ = true;
    return true;
  }

  BT::NodeStatus handlePlannerOutcome(
    const geometry_msgs::msg::PoseStamped & goal, const PlannerOutcome & outcome)
  {
    if (outcome.state == "no_path") {
      return waitAndRetry(goal, "global_no_path");
    }
    if (outcome.state == "invalid_goal") {
      return waitAndRetry(goal, "waiting_for_goal_map_coverage");
    }
    const std::string reason = "planner_failed_without_no_path";
    RCLCPP_ERROR(node_->get_logger(),
      "Navigation planning: category=unrecoverable_planner_result state=%s reason=%s",
      outcome.state.c_str(), reason.c_str());
    publishStatus("failed", reason);
    return BT::NodeStatus::FAILURE;
  }

  BT::NodeStatus resolvePlannerFailure(const geometry_msgs::msg::PoseStamped & goal)
  {
    executor_.spin_some();
    if (const auto * outcome = matchingPlannerOutcome()) {
      planner_result_waiting_ = false;
      planner_request_active_ = false;
      return handlePlannerOutcome(goal, *outcome);
    }
    if (Clock::now() >= planner_result_deadline_) {
      const auto health = children_nodes_[0]->executeTick();
      haltChild(0);
      if (health != BT::NodeStatus::SUCCESS) {
        return waitAndRetry(goal, "localization_or_costmap_unavailable");
      }
      RCLCPP_ERROR(node_->get_logger(),
        "Navigation planning: category=planner_action_failed_without_outcome; stopping");
      planner_request_active_ = false;
      publishStatus("failed", "planner_action_failed_without_outcome");
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus waitAndRetry(
    const geometry_msgs::msg::PoseStamped & goal, const std::string & reason)
  {
    haltChildren();
    if (!clearPath(goal)) return BT::NodeStatus::FAILURE;
    have_path_ = false;
    planning_ = false;
    planner_result_waiting_ = false;
    planner_request_active_ = false;
    stage_ = 0;
    next_attempt_ = Clock::now() + retry_period_;
    publishStatus("blocked_waiting_retry", reason);
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 3000,
      "Navigation planning: category=blocked_waiting reason=%s retry_hz=%.2f",
      reason.c_str(), 1.0 / std::chrono::duration<double>(retry_period_).count());
    return BT::NodeStatus::RUNNING;
  }

  bool writePath(const std::string & port, const nav_msgs::msg::Path & path)
  {
    const auto result = setOutput(port, path);
    if (!result) {
      RCLCPP_ERROR(node_->get_logger(),
        "Navigation planning: category=bt_output_write_failed port=%s error=%s",
        port.c_str(), result.error().c_str());
      publishStatus("failed", "bt_output_write_failed");
      return false;
    }
    return true;
  }

  bool clearPath(const geometry_msgs::msg::PoseStamped & goal)
  {
    nav_msgs::msg::Path empty;
    empty.header.frame_id = goal.header.frame_id;
    empty.header.stamp = static_cast<builtin_interfaces::msg::Time>(node_->now());
    return writePath("path", empty);
  }

  std::string goalId() const
  {
    std::ostringstream out;
    out << std::hex << std::setfill('0');
    for (const auto byte : goal_id_) out << std::setw(2) << static_cast<unsigned int>(byte);
    return out.str();
  }

  void publishStatus(const std::string & state, const std::string & reason)
  {
    if (goal_id_.empty() || !status_pub_) return;
    std_msgs::msg::String msg;
    msg.data = "state=" + state + ";goal_id=" + goalId() + ";reason=" + reason;
    const auto now = Clock::now();
    if (msg.data == last_status_ &&
      (state != "blocked_waiting_retry" ||
      now - last_status_time_ < std::chrono::seconds(1))) return;
    status_pub_->publish(msg);
    last_status_ = msg.data;
    last_status_time_ = now;
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr group_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr planner_status_;
  rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr goal_status_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  std::vector<std::uint8_t> goal_id_;
  std::vector<std::uint8_t> owned_goal_id_;
  geometry_msgs::msg::PoseStamped goal_;
  geometry_msgs::msg::PoseStamped planner_goal_;
  std::deque<PlannerOutcome> planner_outcomes_;
  std::string global_frame_, planner_request_id_, last_status_;
  std::uint64_t planner_request_started_steady_ns_{0};
  bool have_goal_{false}, have_path_{false}, planning_{false}, planner_result_waiting_{false};
  bool planner_request_active_{false};
  std::size_t stage_{0};
  Clock::duration retry_period_{std::chrono::seconds(1)};
  Clock::time_point next_attempt_{}, planner_result_deadline_{}, goal_id_wait_started_{}, last_status_time_{};
};
}

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<bringup::GlobalPlanRetry>("GlobalPlanRetry");
}
