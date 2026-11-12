#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/condition_node.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/msg/costmap.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>

namespace bringup
{
namespace
{
using SteadyClock = std::chrono::steady_clock;
constexpr auto kCostmapSnapshotFreshnessLimit = std::chrono::milliseconds(1500);
constexpr auto kFreshCostmapWait = std::chrono::milliseconds(2500);
}  // namespace

class GoalCostSafeCondition : public BT::ConditionNode
{
public:
  GoalCostSafeCondition(
    const std::string & name,
    const BT::NodeConfiguration & config)
  : BT::ConditionNode(name, config)
  {
    node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
    tf_buffer_ = config.blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
    callback_group_ = node_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive, false);
    callback_group_executor_.add_callback_group(
      callback_group_, node_->get_node_base_interface());

    const auto topic = getInput<std::string>("costmap_topic");
    if (!topic || topic.value().empty()) {
      throw std::invalid_argument("GoalCostSafe requires a non-empty costmap_topic");
    }
    rclcpp::SubscriptionOptions options;
    options.callback_group = callback_group_;
    costmap_sub_ = node_->create_subscription<nav2_msgs::msg::Costmap>(
      topic.value() + "_raw", rclcpp::QoS(1).reliable().transient_local(),
      std::bind(&GoalCostSafeCondition::costmapCallback, this, std::placeholders::_1), options);
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<geometry_msgs::msg::PoseStamped>("goal", "Navigation target"),
      BT::InputPort<std::string>(
        "costmap_topic", "/global_costmap/costmap",
        "Costmap publisher topic; the raw locked snapshot is read from <topic>_raw"),
      BT::InputPort<bool>(
        "require_new_snapshot", false,
        "Wait for a raw costmap snapshot newer than this check before evaluating the goal"),
      BT::InputPort<int>(
        "max_cost_exclusive", 254,
        "Reject goal cells whose raw cost is greater than or equal to this value"),
      BT::InputPort<bool>(
        "allow_unknown", false,
        "Allow recovery checks for goals in unknown space")
    };
  }

  BT::NodeStatus tick() override
  {
    callback_group_executor_.spin_some();

    geometry_msgs::msg::PoseStamped goal;
    if (!getInput("goal", goal)) {
      RCLCPP_ERROR(node_->get_logger(), "GoalCostSafe could not read the navigation goal");
      return BT::NodeStatus::FAILURE;
    }
    bool require_new_snapshot = false;
    if (!getInput("require_new_snapshot", require_new_snapshot)) {
      RCLCPP_ERROR(node_->get_logger(), "GoalCostSafe could not read require_new_snapshot");
      return BT::NodeStatus::FAILURE;
    }
    int max_cost_exclusive = 254;
    if (!getInput("max_cost_exclusive", max_cost_exclusive) ||
      max_cost_exclusive <= 0 || max_cost_exclusive > 255)
    {
      RCLCPP_ERROR(
        node_->get_logger(), "GoalCostSafe requires max_cost_exclusive in [1, 255]");
      return BT::NodeStatus::FAILURE;
    }

    bool allow_unknown = false;
    if (!getInput("allow_unknown", allow_unknown)) {
      RCLCPP_ERROR(node_->get_logger(), "GoalCostSafe could not read allow_unknown");
      return BT::NodeStatus::FAILURE;
    }

    const auto check_started = node_->now();
    nav2_msgs::msg::Costmap::ConstSharedPtr costmap_snapshot;
    SteadyClock::time_point snapshot_received_at;
    if (require_new_snapshot) {
      const auto snapshot_deadline = SteadyClock::now() + kFreshCostmapWait;
      while (SteadyClock::now() < snapshot_deadline) {
        callback_group_executor_.spin_some();
        {
          std::lock_guard<std::mutex> lock(costmap_mutex_);
          if (costmap_snapshot_ && rclcpp::Time(
              costmap_snapshot_->header.stamp, node_->get_clock()->get_clock_type()) >
            check_started)
          {
            costmap_snapshot = costmap_snapshot_;
            snapshot_received_at = costmap_received_at_;
          }
        }
        if (costmap_snapshot && isSnapshotFresh(*costmap_snapshot, snapshot_received_at)) {
          break;
        }
        costmap_snapshot.reset();
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
      }
      if (!costmap_snapshot) {
        RCLCPP_WARN_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 2000,
          "Cannot check goal occupancy: no new fresh raw global costmap snapshot arrived within "
          "2.5 seconds");
        return BT::NodeStatus::FAILURE;
      }
    } else {
      callback_group_executor_.spin_some();
      {
        std::lock_guard<std::mutex> lock(costmap_mutex_);
        costmap_snapshot = costmap_snapshot_;
        snapshot_received_at = costmap_received_at_;
      }
      if (!costmap_snapshot || !isSnapshotFresh(*costmap_snapshot, snapshot_received_at)) {
        RCLCPP_WARN_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 2000,
          "Cannot check goal occupancy: current raw global costmap snapshot is missing or "
          "stale (limit 1.5 seconds)");
        return BT::NodeStatus::FAILURE;
      }
    }

    const auto & costmap = *costmap_snapshot;
    const auto & metadata = costmap.metadata;
    if (costmap.header.frame_id.empty() || metadata.layer != "master" ||
      metadata.size_x == 0 || metadata.size_y == 0 ||
      !std::isfinite(metadata.resolution) || metadata.resolution <= 0.0F ||
      costmap.data.size() != static_cast<std::size_t>(metadata.size_x) * metadata.size_y)
    {
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 2000,
        "Cannot check goal occupancy: current global costmap is invalid");
      return BT::NodeStatus::FAILURE;
    }
    if (goal.header.frame_id.empty()) {
      RCLCPP_ERROR(node_->get_logger(), "Cannot check goal occupancy: goal frame is empty");
      return BT::NodeStatus::FAILURE;
    }

    geometry_msgs::msg::PoseStamped goal_in_costmap;
    try {
      if (goal.header.frame_id == costmap.header.frame_id) {
        goal_in_costmap = goal;
      } else {
        const auto transform = tf_buffer_->lookupTransform(
          costmap.header.frame_id, goal.header.frame_id,
          rclcpp::Time(goal.header.stamp), rclcpp::Duration::from_seconds(0.05));
        tf2::doTransform(goal, goal_in_costmap, transform);
      }
    } catch (const tf2::TransformException & error) {
      RCLCPP_ERROR(
        node_->get_logger(), "Cannot check goal occupancy: TF to global costmap failed: %s",
        error.what());
      return BT::NodeStatus::FAILURE;
    }

    tf2::Transform grid_from_costmap;
    tf2::fromMsg(metadata.origin, grid_from_costmap);
    const auto point = grid_from_costmap.inverse() * tf2::Vector3(
      goal_in_costmap.pose.position.x, goal_in_costmap.pose.position.y, 0.0);
    const double cell_x = point.x() / metadata.resolution;
    const double cell_y = point.y() / metadata.resolution;
    if (!std::isfinite(cell_x) || !std::isfinite(cell_y) || cell_x < 0.0 || cell_y < 0.0 ||
      cell_x >= metadata.size_x || cell_y >= metadata.size_y)
    {
      RCLCPP_ERROR(
        node_->get_logger(),
        "Goal is outside the current inflated global costmap: target=(%.3f,%.3f) "
        "goal_stamp=%d.%09u; stopping recovery for this goal",
        goal_in_costmap.pose.position.x, goal_in_costmap.pose.position.y,
        goal.header.stamp.sec, goal.header.stamp.nanosec);
      return BT::NodeStatus::FAILURE;
    }

    const auto x = static_cast<std::uint32_t>(std::floor(cell_x));
    const auto y = static_cast<std::uint32_t>(std::floor(cell_y));
    const auto index = static_cast<std::size_t>(y) * metadata.size_x + x;
    const auto cost = costmap.data[index];
    if (cost == 255) {
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 2000,
        "Goal is in unknown space: target=(%.3f, %.3f) frame='%s'; allow_unknown=%s "
        "goal_stamp=%d.%09u",
        goal_in_costmap.pose.position.x, goal_in_costmap.pose.position.y,
        costmap.header.frame_id.c_str(), allow_unknown ? "true" : "false",
        goal.header.stamp.sec, goal.header.stamp.nanosec);
      return allow_unknown ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
    if (static_cast<int>(cost) >= max_cost_exclusive) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "Goal cost exceeds recovery threshold: target=(%.3f, %.3f) frame='%s', "
        "raw cost=%u, exclusive threshold=%d goal_stamp=%d.%09u; "
        "stopping recovery for this goal",
        goal_in_costmap.pose.position.x, goal_in_costmap.pose.position.y,
        costmap.header.frame_id.c_str(), static_cast<unsigned int>(cost), max_cost_exclusive,
        goal.header.stamp.sec, goal.header.stamp.nanosec);
      return BT::NodeStatus::FAILURE;
    }

    return BT::NodeStatus::SUCCESS;
  }

private:
  bool isSnapshotFresh(
    const nav2_msgs::msg::Costmap & costmap,
    const SteadyClock::time_point & received_at) const
  {
    const auto stamp = rclcpp::Time(
      costmap.header.stamp, node_->get_clock()->get_clock_type());
    const auto ros_age_ns = (node_->now() - stamp).nanoseconds();
    const auto max_age_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      kCostmapSnapshotFreshnessLimit).count();
    const auto steady_age = SteadyClock::now() - received_at;
    return ros_age_ns >= 0 && ros_age_ns <= max_age_ns &&
      steady_age >= SteadyClock::duration::zero() &&
      steady_age <= kCostmapSnapshotFreshnessLimit;
  }

  void costmapCallback(nav2_msgs::msg::Costmap::ConstSharedPtr message)
  {
    std::lock_guard<std::mutex> lock(costmap_mutex_);
    costmap_snapshot_ = std::move(message);
    costmap_received_at_ = SteadyClock::now();
  }

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr costmap_sub_;
  std::mutex costmap_mutex_;
  nav2_msgs::msg::Costmap::ConstSharedPtr costmap_snapshot_;
  SteadyClock::time_point costmap_received_at_{};
};

}  // namespace bringup

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<bringup::GoalCostSafeCondition>("GoalCostSafe");
}
