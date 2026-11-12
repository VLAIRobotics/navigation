#include <algorithm>
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
#include <vector>

#include <behaviortree_cpp_v3/bt_factory.h>
#include "bringup/path_collision_escape.hpp"
#include <behaviortree_cpp_v3/condition_node.h>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <nav2_costmap_2d/cost_values.hpp>
#include <nav2_msgs/msg/costmap.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>

namespace bringup
{
namespace
{
using SteadyClock = std::chrono::steady_clock;
using GridPolygon = std::vector<tf2::Vector3>;
using path_collision::Point2D;
constexpr auto kInputFreshnessLimit = std::chrono::milliseconds(1500);
constexpr auto kCurrentPoseFreshnessLimit = std::chrono::milliseconds(500);
constexpr auto kMaxFutureStampOffset = std::chrono::milliseconds(100);
constexpr auto kFootprintTransformWait = std::chrono::milliseconds(500);
constexpr double kMaxAngularStep = 0.08726646259971647;  // 5 degrees
constexpr double kMaxSampleDisplacementCells = path_collision::kMaxSampleDisplacementCells;
constexpr size_t kMaxSamplesPerSegment = 200000;
constexpr size_t kMaxCandidateCellsPerPose = 200000;
bool finiteTransform(const tf2::Transform & transform)
{
  const auto & origin = transform.getOrigin();
  const auto & rotation = transform.getRotation();
  return std::isfinite(origin.x()) && std::isfinite(origin.y()) &&
         std::isfinite(origin.z()) && std::isfinite(rotation.x()) &&
         std::isfinite(rotation.y()) && std::isfinite(rotation.z()) &&
         std::isfinite(rotation.w()) && rotation.length2() > 1.0e-12;
}

}  // namespace

class PathCollisionFreeCondition : public BT::ConditionNode
{
public:
  PathCollisionFreeCondition(
    const std::string & name,
    const BT::NodeConfiguration & config)
  : BT::ConditionNode(name, config)
  {
    node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
    tf_buffer_ = config.blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
    if (!node_->has_parameter("start_unknown_escape_radius")) {
      node_->declare_parameter("start_unknown_escape_radius", 0.4411);
    }
    start_unknown_escape_radius_ = node_->get_parameter(
      "start_unknown_escape_radius").as_double();
    if (!std::isfinite(start_unknown_escape_radius_) || start_unknown_escape_radius_ <= 0.0) {
      throw std::invalid_argument("start_unknown_escape_radius must be finite and positive");
    }

    const auto costmap_topic = getInput<std::string>("costmap_topic");
    const auto footprint_topic = getInput<std::string>("footprint_topic");
    if (!costmap_topic || costmap_topic.value().empty() ||
      !footprint_topic || footprint_topic.value().empty())
    {
      throw std::invalid_argument(
              "PathCollisionFree requires costmap_topic and footprint_topic");
    }

    callback_group_ = node_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive, false);
    callback_group_executor_.add_callback_group(
      callback_group_, node_->get_node_base_interface());
    rclcpp::SubscriptionOptions options;
    options.callback_group = callback_group_;

    costmap_subscriber_ = node_->create_subscription<nav2_msgs::msg::Costmap>(
      costmap_topic.value(), rclcpp::QoS(1).reliable().transient_local(),
      std::bind(&PathCollisionFreeCondition::costmapCallback, this,
        std::placeholders::_1), options);
    footprint_subscriber_ = node_->create_subscription<geometry_msgs::msg::PolygonStamped>(
      footprint_topic.value(), rclcpp::SystemDefaultsQoS(),
      std::bind(&PathCollisionFreeCondition::footprintCallback, this,
        std::placeholders::_1), options);
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<nav_msgs::msg::Path>(
        "path", "Path to validate against the current robot footprint"),
      BT::InputPort<nav_msgs::msg::Path>(
        "alternate_path", "Optional second path checked against the same snapshot"),
      BT::InputPort<geometry_msgs::msg::PoseStamped>(
        "goal", "Optional goal pose checked against the same costmap snapshot"),
      BT::InputPort<bool>(
        "check_path", true, "Validate the primary path and optional alternate path"),
      BT::InputPort<bool>(
        "check_alternate_path", false, "Also validate alternate_path"),
      BT::InputPort<bool>("check_goal", false, "Also validate goal occupancy"),
      BT::InputPort<std::string>(
        "costmap_topic", "/global_costmap/costmap_raw", "Fresh raw Nav2 costmap topic"),
      BT::InputPort<std::string>(
        "footprint_topic", "/global_costmap/published_footprint",
        "Current robot footprint topic"),
      BT::InputPort<bool>(
        "require_new_snapshot", false,
        "Wait for a raw costmap snapshot newer than this tick before checking"),
      BT::InputPort<int>("max_cost_exclusive", 254, "Reject goal cells at or above this cost"),
      BT::InputPort<bool>("allow_unknown", false, "Allow a goal in unknown space")
    };
  }

  BT::NodeStatus tick() override
  {
    bool check_path = true;
    if (!getInput("check_path", check_path)) {
      RCLCPP_WARN(node_->get_logger(), "PathCollisionFree could not read path check option");
      return BT::NodeStatus::FAILURE;
    }
    nav_msgs::msg::Path path;
    if (check_path && (!getInput("path", path) || path.poses.empty())) {
      RCLCPP_DEBUG(
        node_->get_logger(),
        "PathCollisionFree skipped because planning or path preparation produced no path");
      return BT::NodeStatus::FAILURE;
    }

    nav_msgs::msg::Path alternate_path;
    bool has_alternate_path = false;
    if (check_path && !getInput("check_alternate_path", has_alternate_path)) {
      RCLCPP_WARN(node_->get_logger(), "PathCollisionFree could not read alternate path option");
      return BT::NodeStatus::FAILURE;
    }
    if (has_alternate_path &&
      (!getInput("alternate_path", alternate_path) || alternate_path.poses.empty()))
    {
      RCLCPP_WARN(node_->get_logger(), "PathCollisionFree received an empty alternate path");
      return BT::NodeStatus::FAILURE;
    }

    geometry_msgs::msg::PoseStamped goal;
    bool has_goal = false;
    if (!getInput("check_goal", has_goal)) {
      RCLCPP_WARN(node_->get_logger(), "PathCollisionFree could not read goal check option");
      return BT::NodeStatus::FAILURE;
    }
    if (has_goal && !getInput("goal", goal)) {
      RCLCPP_WARN(node_->get_logger(), "PathCollisionFree could not read the goal pose");
      return BT::NodeStatus::FAILURE;
    }

    bool require_new_snapshot = false;
    int max_cost_exclusive = 254;
    bool allow_unknown = false;
    if (!getInput("require_new_snapshot", require_new_snapshot) ||
      !getInput("max_cost_exclusive", max_cost_exclusive) ||
      !getInput("allow_unknown", allow_unknown) ||
      max_cost_exclusive <= 0 || max_cost_exclusive > 255)
    {
      RCLCPP_WARN(node_->get_logger(), "PathCollisionFree received invalid gate options");
      return BT::NodeStatus::FAILURE;
    }

    nav2_msgs::msg::Costmap::ConstSharedPtr costmap;
    SteadyClock::time_point costmap_received_at;
    geometry_msgs::msg::PolygonStamped::ConstSharedPtr footprint;
    SteadyClock::time_point footprint_received_at;
    const auto check_started = node_->now();
    const auto check_started_steady = SteadyClock::now();
    const auto snapshot_deadline = SteadyClock::now() + std::chrono::milliseconds(2500);
    do {
      callback_group_executor_.spin_some();
      {
        std::lock_guard<std::mutex> lock(snapshot_mutex_);
        costmap = costmap_snapshot_;
        costmap_received_at = costmap_received_at_;
        footprint = footprint_snapshot_;
        footprint_received_at = footprint_received_at_;
      }
      const bool costmap_is_fresh = costmap && isFresh(
        rclcpp::Time(costmap->header.stamp, node_->get_clock()->get_clock_type()),
        costmap_received_at);
      const bool costmap_is_new = costmap && rclcpp::Time(
        costmap->header.stamp, node_->get_clock()->get_clock_type()) > check_started &&
        costmap_received_at > check_started_steady;
      const bool footprint_is_fresh = footprint && isFresh(
        rclcpp::Time(footprint->header.stamp, node_->get_clock()->get_clock_type()),
        footprint_received_at);
      if (costmap_is_fresh && (!require_new_snapshot || costmap_is_new) && footprint_is_fresh) {
        break;
      }
      if (!require_new_snapshot || SteadyClock::now() >= snapshot_deadline) {
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    } while (true);

    if (!costmap || !isFresh(
        rclcpp::Time(costmap->header.stamp, node_->get_clock()->get_clock_type()),
        costmap_received_at) ||
      (require_new_snapshot && rclcpp::Time(
        costmap->header.stamp, node_->get_clock()->get_clock_type()) <= check_started) ||
      (require_new_snapshot && costmap_received_at <= check_started_steady))
    {
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 2000,
        "PathCollisionFree rejected the path because no fresh required costmap snapshot arrived");
      return BT::NodeStatus::FAILURE;
    }
    if (!footprint || !isFresh(
        rclcpp::Time(footprint->header.stamp, node_->get_clock()->get_clock_type()),
        footprint_received_at))
    {
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 2000,
        "PathCollisionFree rejected the path because the robot footprint is missing or stale");
      return BT::NodeStatus::FAILURE;
    }

    const auto & metadata = costmap->metadata;
    const uint64_t cell_count = static_cast<uint64_t>(metadata.size_x) * metadata.size_y;
    if (costmap->header.frame_id.empty() || metadata.layer != "master" ||
      metadata.size_x == 0 || metadata.size_y == 0 ||
      !std::isfinite(metadata.resolution) || metadata.resolution <= 0.0F ||
      static_cast<uint64_t>(costmap->data.size()) != cell_count)
    {
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 2000,
        "PathCollisionFree rejected an invalid raw costmap snapshot");
      return BT::NodeStatus::FAILURE;
    }
    if (check_path && (!pathHasFrame(path, costmap->header.frame_id) ||
      (has_alternate_path && !pathHasFrame(alternate_path, costmap->header.frame_id))))
    {
      RCLCPP_WARN(node_->get_logger(), "PathCollisionFree rejected a path/costmap frame mismatch");
      return BT::NodeStatus::FAILURE;
    }
    if (footprint->header.frame_id.empty() ||
      footprint->header.frame_id != costmap->header.frame_id)
    {
      RCLCPP_WARN(
        node_->get_logger(),
        "PathCollisionFree rejected footprint frame '%s' for costmap frame '%s'",
        footprint->header.frame_id.c_str(), costmap->header.frame_id.c_str());
      return BT::NodeStatus::FAILURE;
    }

    tf2::Transform world_from_grid;
    if (!makeCostmapTransform(metadata.origin, world_from_grid)) {
      RCLCPP_WARN(node_->get_logger(), "PathCollisionFree rejected invalid costmap origin");
      return BT::NodeStatus::FAILURE;
    }
    const tf2::Transform grid_from_world = world_from_grid.inverse();

    GridPolygon footprint_in_base;
    geometry_msgs::msg::Pose actual_start_pose;
    tf2::Transform world_from_base;
    try {
      tf2::Transform base_from_world;
      const auto footprint_transform = tf_buffer_->lookupTransform(
        "base_link", costmap->header.frame_id,
        rclcpp::Time(footprint->header.stamp, node_->get_clock()->get_clock_type()),
        rclcpp::Duration::from_nanoseconds(
          std::chrono::duration_cast<std::chrono::nanoseconds>(
            kFootprintTransformWait).count()));
      tf2::fromMsg(footprint_transform.transform, base_from_world);
      if (!finiteTransform(base_from_world)) {
        RCLCPP_WARN(node_->get_logger(), "PathCollisionFree received an invalid footprint TF");
        return BT::NodeStatus::FAILURE;
      }

      const auto current_robot_transform = tf_buffer_->lookupTransform(
        costmap->header.frame_id, "base_link", tf2::TimePointZero);
      const auto current_transform_stamp = rclcpp::Time(
        current_robot_transform.header.stamp, node_->get_clock()->get_clock_type());
      if (!isCurrentPoseFresh(current_transform_stamp)) {
        RCLCPP_WARN_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 2000,
          "PathCollisionFree rejected a stale current robot transform");
        return BT::NodeStatus::FAILURE;
      }
      tf2::fromMsg(current_robot_transform.transform, world_from_base);
      if (!finiteTransform(world_from_base)) {
        RCLCPP_WARN(node_->get_logger(), "PathCollisionFree received an invalid current robot TF");
        return BT::NodeStatus::FAILURE;
      }
      auto start_rotation = world_from_base.getRotation();
      start_rotation.normalize();
      actual_start_pose.position.x = world_from_base.getOrigin().x();
      actual_start_pose.position.y = world_from_base.getOrigin().y();
      actual_start_pose.position.z = world_from_base.getOrigin().z();
      actual_start_pose.orientation = tf2::toMsg(start_rotation);
      footprint_in_base.reserve(footprint->polygon.points.size());
      for (const auto & point : footprint->polygon.points) {
        if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
          RCLCPP_WARN(node_->get_logger(), "PathCollisionFree rejected non-finite footprint point");
          return BT::NodeStatus::FAILURE;
        }
        footprint_in_base.push_back(
          base_from_world * tf2::Vector3(point.x, point.y, point.z));
      }
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 2000,
        "PathCollisionFree could not transform the published footprint: %s", error.what());
      return BT::NodeStatus::FAILURE;
    }

    std::vector<Point2D> footprint_shape;
    footprint_shape.reserve(footprint_in_base.size());
    double max_footprint_radius = 0.0;
    for (const auto & point : footprint_in_base) {
      if (!std::isfinite(point.x()) || !std::isfinite(point.y())) {
        RCLCPP_WARN(node_->get_logger(), "PathCollisionFree rejected invalid transformed footprint");
        return BT::NodeStatus::FAILURE;
      }
      footprint_shape.push_back({point.x(), point.y()});
      max_footprint_radius = std::max(max_footprint_radius, std::hypot(point.x(), point.y()));
    }
    if (!path_collision::hasUsableArea(footprint_shape)) {
      RCLCPP_WARN(node_->get_logger(), "PathCollisionFree rejected a degenerate footprint");
      return BT::NodeStatus::FAILURE;
    }

    if (check_path && (!checkPath(
        path, actual_start_pose, footprint_in_base, footprint_shape, max_footprint_radius,
        world_from_grid, *costmap,
        start_unknown_escape_radius_) ||
      (has_alternate_path && !checkPath(
        alternate_path, actual_start_pose, footprint_in_base, footprint_shape,
        max_footprint_radius,
        world_from_grid, *costmap,
        start_unknown_escape_radius_))))
    {
      return BT::NodeStatus::FAILURE;
    }

    if (has_goal && !checkGoal(
        goal, *costmap, grid_from_world, max_cost_exclusive, allow_unknown))
    {
      return BT::NodeStatus::FAILURE;
    }
    return inputsStillFresh(*costmap, costmap_received_at, *footprint, footprint_received_at) ?
      BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

private:
  bool pathHasFrame(const nav_msgs::msg::Path & path, const std::string & frame) const
  {
    if (path.header.frame_id.empty() || path.header.frame_id != frame) {
      return false;
    }
    for (const auto & pose : path.poses) {
      if (!pose.header.frame_id.empty() && pose.header.frame_id != frame) {
        return false;
      }
    }
    return true;
  }

  bool checkGoal(
    const geometry_msgs::msg::PoseStamped & goal,
    const nav2_msgs::msg::Costmap & costmap,
    const tf2::Transform & grid_from_world,
    int max_cost_exclusive,
    bool allow_unknown) const
  {
    if (goal.header.frame_id.empty()) {
      RCLCPP_WARN(node_->get_logger(),
        "Navigation gate: category=static_goal_invalid reason=empty_frame "
        "goal_stamp=%d.%09u", goal.header.stamp.sec, goal.header.stamp.nanosec);
      return false;
    }
    geometry_msgs::msg::PoseStamped goal_in_costmap;
    try {
      if (goal.header.frame_id == costmap.header.frame_id) {
        goal_in_costmap = goal;
      } else {
        const auto transform = tf_buffer_->lookupTransform(
          costmap.header.frame_id, goal.header.frame_id,
          rclcpp::Time(goal.header.stamp, node_->get_clock()->get_clock_type()),
          rclcpp::Duration::from_seconds(0.05));
        tf2::doTransform(goal, goal_in_costmap, transform);
      }
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN(node_->get_logger(),
        "Navigation gate: category=goal_transform_unavailable reason=%s", error.what());
      return false;
    }

    const auto & position = goal_in_costmap.pose.position;
    if (!std::isfinite(position.x) || !std::isfinite(position.y) ||
      !std::isfinite(position.z))
    {
      RCLCPP_WARN(node_->get_logger(),
        "Navigation gate: category=static_goal_invalid reason=non_finite_pose "
        "goal_stamp=%d.%09u", goal.header.stamp.sec, goal.header.stamp.nanosec);
      return false;
    }
    const auto grid_point = grid_from_world * tf2::Vector3(position.x, position.y, position.z);
    const double cell_x = grid_point.x() / costmap.metadata.resolution;
    const double cell_y = grid_point.y() / costmap.metadata.resolution;
    if (!std::isfinite(cell_x) || !std::isfinite(cell_y) || cell_x < 0.0 || cell_y < 0.0 ||
      cell_x >= costmap.metadata.size_x || cell_y >= costmap.metadata.size_y)
    {
      RCLCPP_WARN(node_->get_logger(),
        "Navigation gate: category=static_goal_out_of_bounds point=(%.3f,%.3f) "
        "goal_stamp=%d.%09u", position.x, position.y,
        goal.header.stamp.sec, goal.header.stamp.nanosec);
      return false;
    }
    const auto x = static_cast<uint32_t>(std::floor(cell_x));
    const auto y = static_cast<uint32_t>(std::floor(cell_y));
    const auto index = static_cast<size_t>(y) * costmap.metadata.size_x + x;
    const auto cost = costmap.data[index];
    if (cost == nav2_costmap_2d::NO_INFORMATION) {
      if (allow_unknown) {
        return true;
      }
      RCLCPP_WARN(node_->get_logger(),
        "Navigation gate: category=static_goal_unknown cell=(%u,%u) point=(%.3f,%.3f) "
        "cost=%u goal_stamp=%d.%09u",
        x, y, position.x, position.y, static_cast<unsigned int>(cost),
        goal.header.stamp.sec, goal.header.stamp.nanosec);
      return false;
    }
    if (static_cast<int>(cost) >= max_cost_exclusive) {
      RCLCPP_WARN(
        node_->get_logger(),
        "Navigation gate: category=static_goal_invalid cell=(%u,%u) "
        "point=(%.3f,%.3f) cost=%u goal_stamp=%d.%09u",
        x, y, position.x, position.y, static_cast<unsigned int>(cost),
        goal.header.stamp.sec, goal.header.stamp.nanosec);
      return false;
    }
    return true;
  }

  bool checkPath(
    const nav_msgs::msg::Path & path,
    const geometry_msgs::msg::Pose & actual_start_pose,
    const GridPolygon & footprint_in_base,
    const std::vector<Point2D> & footprint_shape,
    double max_footprint_radius,
    const tf2::Transform & world_from_grid,
    const nav2_msgs::msg::Costmap & costmap,
    double start_unknown_escape_radius) const
  {
    std::vector<double> path_yaws;
    path_yaws.reserve(path.poses.size());
    for (size_t i = 0; i < path.poses.size(); ++i) {
      const auto & pose = path.poses[i];
      const auto & position = pose.pose.position;
      const auto & orientation = pose.pose.orientation;
      if (!std::isfinite(position.x) || !std::isfinite(position.y) ||
        !std::isfinite(position.z) || !std::isfinite(orientation.x) ||
        !std::isfinite(orientation.y) || !std::isfinite(orientation.z) ||
        !std::isfinite(orientation.w))
      {
        RCLCPP_WARN(node_->get_logger(), "PathCollisionFree rejected invalid pose %zu", i);
        return false;
      }
      const double norm = std::hypot(
        std::hypot(orientation.x, orientation.y),
        std::hypot(orientation.z, orientation.w));
      if (!std::isfinite(norm) || norm < 1.0e-6) {
        RCLCPP_WARN(node_->get_logger(), "PathCollisionFree rejected invalid orientation at pose %zu", i);
        return false;
      }
      tf2::Quaternion normalized(
        orientation.x / norm, orientation.y / norm, orientation.z / norm, orientation.w / norm);
      path_yaws.push_back(tf2::getYaw(normalized));
    }

    const double resolution = costmap.metadata.resolution;
    const auto & metadata = costmap.metadata;
    const auto & costs = costmap.data;
    const auto grid_from_world = world_from_grid.inverse();
    const auto & start_position = actual_start_pose.position;
    const auto start_grid = grid_from_world * tf2::Vector3(
      start_position.x, start_position.y, start_position.z);
    const double start_cell_x = start_grid.x() / resolution;
    const double start_cell_y = start_grid.y() / resolution;
    if (!std::isfinite(start_cell_x) || !std::isfinite(start_cell_y) ||
      start_cell_x < 0.0 || start_cell_y < 0.0 ||
      start_cell_x >= metadata.size_x || start_cell_y >= metadata.size_y)
    {
      RCLCPP_WARN(node_->get_logger(), "PathCollisionFree rejected a path start outside the costmap");
      return false;
    }
    const auto & start_orientation = actual_start_pose.orientation;
    const double start_orientation_norm = std::hypot(
      std::hypot(start_orientation.x, start_orientation.y),
      std::hypot(start_orientation.z, start_orientation.w));
    if (!std::isfinite(start_orientation_norm) || start_orientation_norm < 1.0e-6) {
      RCLCPP_WARN(node_->get_logger(), "PathCollisionFree rejected an invalid actual start orientation");
      return false;
    }
    tf2::Quaternion normalized_start_orientation(
      start_orientation.x / start_orientation_norm,
      start_orientation.y / start_orientation_norm,
      start_orientation.z / start_orientation_norm,
      start_orientation.w / start_orientation_norm);
    const tf2::Transform world_from_start(
      normalized_start_orientation,
      tf2::Vector3(start_position.x, start_position.y, start_position.z));
    const tf2::Transform grid_from_start = grid_from_world * world_from_start;
    std::vector<Point2D> start_footprint_polygon;
    if (!path_collision::transformFootprintToGrid(
        footprint_shape, start_cell_x, start_cell_y,
        tf2::getYaw(grid_from_start.getRotation()), resolution, start_footprint_polygon))
    {
      RCLCPP_WARN(node_->get_logger(),
        "PathCollisionFree rejected an invalid footprint at the current start pose");
      return false;
    }
    std::vector<std::size_t> initial_unknown_cells;
    if (!path_collision::collectUnknownCells(
        costs, metadata.size_x, metadata.size_y, start_footprint_polygon,
        kMaxSampleDisplacementCells,
        kMaxCandidateCellsPerPose, initial_unknown_cells))
    {
      RCLCPP_WARN(node_->get_logger(),
        "PathCollisionFree rejected an invalid initial footprint for unknown-cell handling");
      return false;
    }

    // Center the escape disk on the current TF pose. Match the footprint collection
    // margin to checkPose so nearby unknown cells can activate a known start's escape.
    path_collision::StartEscapeMask start_escape_mask(
      costs, metadata.size_x, metadata.size_y, resolution, start_cell_x, start_cell_y,
      start_unknown_escape_radius, initial_unknown_cells);

    const double max_linear_step = resolution * kMaxSampleDisplacementCells;
    const double max_angular_step = max_footprint_radius > path_collision::kGeometryEpsilon ?
      std::min(kMaxAngularStep, 2.0 * std::asin(std::min(
        1.0, max_linear_step / (2.0 * max_footprint_radius)))) : kMaxAngularStep;
    if (!std::isfinite(max_linear_step) || max_linear_step <= 0.0 ||
      !std::isfinite(max_angular_step) || max_angular_step <= 0.0)
    {
      RCLCPP_WARN(node_->get_logger(), "PathCollisionFree rejected invalid sampling geometry");
      return false;
    }

    geometry_msgs::msg::Pose previous_pose = actual_start_pose;
    double previous_yaw = tf2::getYaw(normalized_start_orientation);

    for (size_t i = 0; i < path.poses.size(); ++i) {
      const auto & target_pose = path.poses[i].pose;
      const auto & from = previous_pose.position;
      const auto & to = target_pose.position;
      const double dx = to.x - from.x;
      const double dy = to.y - from.y;
      const double distance = std::hypot(dx, dy);
      const double yaw_delta = std::atan2(
        std::sin(path_yaws[i] - previous_yaw),
        std::cos(path_yaws[i] - previous_yaw));
      const double requested_linear_steps = distance / max_linear_step;
      const double requested_angular_steps = std::abs(yaw_delta) / max_angular_step;
      if (!std::isfinite(requested_linear_steps) || !std::isfinite(requested_angular_steps) ||
        requested_linear_steps > static_cast<double>(kMaxSamplesPerSegment) ||
        requested_angular_steps > static_cast<double>(kMaxSamplesPerSegment))
      {
        RCLCPP_WARN(node_->get_logger(), "PathCollisionFree rejected oversized segment %zu", i);
        return false;
      }
      const size_t steps = std::max<size_t>(1, static_cast<size_t>(std::ceil(std::max(
        requested_linear_steps, requested_angular_steps))));
      if (steps > kMaxSamplesPerSegment) {
        RCLCPP_WARN(node_->get_logger(), "PathCollisionFree rejected oversized segment %zu", i);
        return false;
      }
      for (size_t step = 0; step <= steps; ++step) {
        const double ratio = static_cast<double>(step) / static_cast<double>(steps);
        geometry_msgs::msg::Pose pose;
        pose.position.x = from.x + ratio * dx;
        pose.position.y = from.y + ratio * dy;
        pose.position.z = from.z + ratio * (to.z - from.z);
        const double yaw = previous_yaw + ratio * yaw_delta;
        // The actual start pose has no interpolation gap and matches footprint map clearing.
        const double footprint_margin = (i == 0 && step == 0) ?
          0.0 : kMaxSampleDisplacementCells;
        if (!checkPose(
            pose, yaw, footprint_in_base, world_from_grid, costmap, resolution,
            footprint_margin, i, ratio, start_escape_mask))
        {
          return false;
        }
      }
      previous_pose = target_pose;
      previous_yaw = path_yaws[i];
    }
    return true;
  }

  static bool makeCostmapTransform(
    const geometry_msgs::msg::Pose & origin, tf2::Transform & world_from_grid)
  {
    const auto & position = origin.position;
    const auto & orientation = origin.orientation;
    if (!std::isfinite(position.x) || !std::isfinite(position.y) ||
      !std::isfinite(position.z) || !std::isfinite(orientation.x) ||
      !std::isfinite(orientation.y) || !std::isfinite(orientation.z) ||
      !std::isfinite(orientation.w))
    {
      return false;
    }
    const double norm = std::hypot(
      std::hypot(orientation.x, orientation.y),
      std::hypot(orientation.z, orientation.w));
    if (!std::isfinite(norm) || norm < 1.0e-6) {
      return false;
    }
    tf2::Quaternion rotation(
      orientation.x / norm, orientation.y / norm, orientation.z / norm, orientation.w / norm);
    world_from_grid = tf2::Transform(
      rotation, tf2::Vector3(position.x, position.y, position.z));
    return finiteTransform(world_from_grid);
  }

  bool isFresh(const rclcpp::Time & stamp, SteadyClock::time_point received_at) const
  {
    const auto ros_age_ns = (node_->now() - stamp).nanoseconds();
    const auto max_age_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      kInputFreshnessLimit).count();
    const auto max_future_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      kMaxFutureStampOffset).count();
    const auto steady_age = SteadyClock::now() - received_at;
    return ros_age_ns >= -max_future_ns && ros_age_ns <= max_age_ns &&
           steady_age >= SteadyClock::duration::zero() && steady_age <= kInputFreshnessLimit;
  }

  bool isCurrentPoseFresh(const rclcpp::Time & stamp) const
  {
    const auto ros_age_ns = (node_->now() - stamp).nanoseconds();
    const auto max_age_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      kCurrentPoseFreshnessLimit).count();
    const auto max_future_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      kMaxFutureStampOffset).count();
    return ros_age_ns >= -max_future_ns && ros_age_ns <= max_age_ns;
  }

  bool inputsStillFresh(
    const nav2_msgs::msg::Costmap & costmap,
    SteadyClock::time_point costmap_received_at,
    const geometry_msgs::msg::PolygonStamped & footprint,
    SteadyClock::time_point footprint_received_at) const
  {
    const bool fresh = isFresh(
      rclcpp::Time(costmap.header.stamp, node_->get_clock()->get_clock_type()),
      costmap_received_at) && isFresh(
      rclcpp::Time(footprint.header.stamp, node_->get_clock()->get_clock_type()),
      footprint_received_at);
    if (!fresh) {
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 2000,
        "PathCollisionFree rejected the path because an input became stale during checking");
    }
    return fresh;
  }

  bool checkPose(
    const geometry_msgs::msg::Pose & pose,
    double yaw,
    const GridPolygon & footprint_in_base,
    const tf2::Transform & world_from_grid,
    const nav2_msgs::msg::Costmap & costmap,
    double resolution,
    double sampling_margin_cells,
    size_t segment_index,
    double segment_ratio,
    path_collision::StartEscapeMask & start_escape_mask) const
  {
    tf2::Quaternion rotation;
    rotation.setRPY(0.0, 0.0, yaw);
    const tf2::Transform world_from_base(
      rotation, tf2::Vector3(pose.position.x, pose.position.y, pose.position.z));
    const tf2::Transform grid_from_world = world_from_grid.inverse();
    std::vector<Point2D> polygon;
    polygon.reserve(footprint_in_base.size());
    for (const auto & point : footprint_in_base) {
      const auto grid_point = grid_from_world * world_from_base * point;
      const double x = grid_point.x() / resolution;
      const double y = grid_point.y() / resolution;
      if (!std::isfinite(x) || !std::isfinite(y)) {
        RCLCPP_WARN(node_->get_logger(), "PathCollisionFree rejected non-finite transformed footprint");
        return false;
      }
      polygon.push_back({x, y});
    }

    const double map_width = static_cast<double>(costmap.metadata.size_x);
    const double map_height = static_cast<double>(costmap.metadata.size_y);
    for (const auto & point : polygon) {
      if (point.x < -path_collision::kGeometryEpsilon ||
        point.y < -path_collision::kGeometryEpsilon ||
        point.x >= map_width || point.y >= map_height)
      {
        RCLCPP_WARN_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 1000,
          "PathCollisionFree found footprint outside costmap at segment %zu (t=%.3f)",
          segment_index, segment_ratio);
        return false;
      }
    }

    double min_x = polygon.front().x;
    double max_x = polygon.front().x;
    double min_y = polygon.front().y;
    double max_y = polygon.front().y;
    for (const auto & point : polygon) {
      min_x = std::min(min_x, point.x);
      max_x = std::max(max_x, point.x);
      min_y = std::min(min_y, point.y);
      max_y = std::max(max_y, point.y);
    }
    const auto & metadata = costmap.metadata;
    const int64_t first_x = std::max<int64_t>(0, static_cast<int64_t>(
      std::floor(min_x - 1.0 - sampling_margin_cells)));
    const int64_t last_x = std::min<int64_t>(metadata.size_x - 1, static_cast<int64_t>(
      std::ceil(max_x + sampling_margin_cells)));
    const int64_t first_y = std::max<int64_t>(0, static_cast<int64_t>(
      std::floor(min_y - 1.0 - sampling_margin_cells)));
    const int64_t last_y = std::min<int64_t>(metadata.size_y - 1, static_cast<int64_t>(
      std::ceil(max_y + sampling_margin_cells)));
    if (last_x < first_x || last_y < first_y) {
      RCLCPP_WARN(node_->get_logger(), "PathCollisionFree rejected footprint outside costmap");
      return false;
    }
    const uint64_t candidate_cells = static_cast<uint64_t>(last_x - first_x + 1) *
      static_cast<uint64_t>(last_y - first_y + 1);
    if (candidate_cells > kMaxCandidateCellsPerPose) {
      RCLCPP_WARN(node_->get_logger(), "PathCollisionFree rejected oversized footprint bounds");
      return false;
    }

    bool footprint_overlaps_start_escape = false;
    for (int64_t y = first_y; y <= last_y; ++y) {
      for (int64_t x = first_x; x <= last_x; ++x) {
        if (!path_collision::polygonIntersectsCell(
            polygon, x, y, sampling_margin_cells))
        {
          continue;
        }
        const auto index = static_cast<size_t>(y) * metadata.size_x +
          static_cast<size_t>(x);
        const auto cost = costmap.data[index];
        if (start_escape_mask.contains_start_escape(index)) {
          footprint_overlaps_start_escape = true;
        }
        if (path_collision::isBlockedCell(cost, start_escape_mask.allows_unknown(index))) {
          const char * reason = cost == nav2_costmap_2d::NO_INFORMATION ?
            "unknown cell outside bounded start escape" : "lethal cell";
          RCLCPP_WARN_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 1000,
            "PathCollisionFree found %s at segment %zu (t=%.3f), "
            "grid=(%lld,%lld), cost=%u",
            reason, segment_index, segment_ratio, static_cast<long long>(x),
            static_cast<long long>(y), static_cast<unsigned int>(cost));
          return false;
        }
      }
    }
    start_escape_mask.finish_pose(footprint_overlaps_start_escape);
    return true;
  }

  void costmapCallback(nav2_msgs::msg::Costmap::ConstSharedPtr message)
  {
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    costmap_snapshot_ = std::move(message);
    costmap_received_at_ = SteadyClock::now();
  }

  void footprintCallback(geometry_msgs::msg::PolygonStamped::ConstSharedPtr message)
  {
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    footprint_snapshot_ = std::move(message);
    footprint_received_at_ = SteadyClock::now();
  }

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr costmap_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr footprint_subscriber_;
  std::mutex snapshot_mutex_;
  double start_unknown_escape_radius_{0.4411};
  nav2_msgs::msg::Costmap::ConstSharedPtr costmap_snapshot_;
  SteadyClock::time_point costmap_received_at_{};
  geometry_msgs::msg::PolygonStamped::ConstSharedPtr footprint_snapshot_;
  SteadyClock::time_point footprint_received_at_{};
};

}  // namespace bringup

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<bringup::PathCollisionFreeCondition>("PathCollisionFree");
}
