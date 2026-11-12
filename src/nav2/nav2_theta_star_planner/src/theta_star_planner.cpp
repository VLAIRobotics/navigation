// Copyright 2020 Anshumaan Singh
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <cstddef>
#include <chrono>
#include <cmath>
#include <limits>
#include <vector>
#include <memory>
#include <random>
#include <string>
#include <iomanip>
#include <sstream>
#include <builtin_interfaces/msg/time.hpp>
#include "nav2_theta_star_planner/theta_star_planner.hpp"
#include "nav2_theta_star_planner/theta_star.hpp"
#include "nav2_theta_star_planner/goal_cell_endpoint.hpp"

namespace nav2_theta_star_planner
{
void ThetaStarPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  planner_ = std::make_unique<theta_star::ThetaStar>();
  parent_node_ = parent;
  auto node = parent_node_.lock();
  logger_ = node->get_logger();
  clock_ = node->get_clock();
  name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;
  planner_->costmap_ = costmap_ros->getCostmap();
  global_frame_ = costmap_ros->getGlobalFrameID();
  std::random_device entropy;
  std::ostringstream instance_id;
  instance_id << std::hex << std::setfill('0');
  for (int i = 0; i < 4; ++i) instance_id << std::setw(8) << entropy();
  plan_instance_id_ = instance_id.str();
  plan_sequence_ = 0;
  plan_status_publisher_ = node->create_publisher<std_msgs::msg::String>(
    "/navigation/global_plan_status", rclcpp::QoS(1).reliable().transient_local());

  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".how_many_corners", rclcpp::ParameterValue(8));

  node->get_parameter(name_ + ".how_many_corners", planner_->how_many_corners_);

  if (planner_->how_many_corners_ != 8 && planner_->how_many_corners_ != 4) {
    planner_->how_many_corners_ = 8;
    RCLCPP_WARN(logger_, "Your value for - .how_many_corners  was overridden, and is now set to 8");
  }

  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".allow_unknown", rclcpp::ParameterValue(true));
  node->get_parameter(name_ + ".allow_unknown", planner_->allow_unknown_);

  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".start_unknown_escape_radius", rclcpp::ParameterValue(0.0));
  node->get_parameter(
    name_ + ".start_unknown_escape_radius", planner_->start_unknown_escape_radius_);
  if (!std::isfinite(planner_->start_unknown_escape_radius_) ||
    planner_->start_unknown_escape_radius_ < 0.0)
  {
    planner_->start_unknown_escape_radius_ = 0.0;
    RCLCPP_WARN(logger_, "Invalid start_unknown_escape_radius; using 0.0 m");
  }

  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".w_euc_cost", rclcpp::ParameterValue(1.0));
  node->get_parameter(name_ + ".w_euc_cost", planner_->w_euc_cost_);

  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".w_traversal_cost", rclcpp::ParameterValue(2.0));
  node->get_parameter(name_ + ".w_traversal_cost", planner_->w_traversal_cost_);

  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".w_clearance_cost", rclcpp::ParameterValue(0.0));
  node->get_parameter(name_ + ".w_clearance_cost", planner_->w_clearance_cost_);
  if (!std::isfinite(planner_->w_clearance_cost_) || planner_->w_clearance_cost_ < 0.0 ||
    planner_->w_clearance_cost_ > 10.0)
  {
    planner_->w_clearance_cost_ = 0.0;
    RCLCPP_WARN(logger_, "Invalid w_clearance_cost; disabling clearance preference");
  }

  planner_->w_heuristic_cost_ = planner_->w_euc_cost_ < 1.0 ? planner_->w_euc_cost_ : 1.0;

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".use_final_approach_orientation", rclcpp::ParameterValue(false));
  node->get_parameter(name + ".use_final_approach_orientation", use_final_approach_orientation_);

  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".path_hysteresis_ratio", rclcpp::ParameterValue(0.15));
  node->get_parameter(name_ + ".path_hysteresis_ratio", path_hysteresis_ratio_);
  if (!std::isfinite(path_hysteresis_ratio_) || path_hysteresis_ratio_ < 0.0) {
    path_hysteresis_ratio_ = 0.0;
    RCLCPP_WARN(logger_, "Invalid path_hysteresis_ratio; disabling path hysteresis");
  }
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".path_hysteresis_max_rejoin_distance", rclcpp::ParameterValue(0.20));
  node->get_parameter(
    name_ + ".path_hysteresis_max_rejoin_distance", path_hysteresis_max_rejoin_distance_);
  if (!std::isfinite(path_hysteresis_max_rejoin_distance_) ||
    path_hysteresis_max_rejoin_distance_ < 0.0)
  {
    path_hysteresis_max_rejoin_distance_ = 0.20;
    RCLCPP_WARN(
      logger_, "Invalid path_hysteresis_max_rejoin_distance; using 0.20 m");
  }
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".path_hysteresis_max_length_ratio", rclcpp::ParameterValue(1.25));
  node->get_parameter(
    name_ + ".path_hysteresis_max_length_ratio", path_hysteresis_max_length_ratio_);
  if (!std::isfinite(path_hysteresis_max_length_ratio_) ||
    path_hysteresis_max_length_ratio_ < 1.0)
  {
    path_hysteresis_max_length_ratio_ = 1.25;
    RCLCPP_WARN(logger_, "Invalid path_hysteresis_max_length_ratio; using 1.25");
  }
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".path_hysteresis_min_clearance_gain", rclcpp::ParameterValue(0.20));
  node->get_parameter(
    name_ + ".path_hysteresis_min_clearance_gain", path_hysteresis_min_clearance_gain_);
  if (!std::isfinite(path_hysteresis_min_clearance_gain_) ||
    path_hysteresis_min_clearance_gain_ < 0.0)
  {
    path_hysteresis_min_clearance_gain_ = 0.20;
    RCLCPP_WARN(logger_, "Invalid path_hysteresis_min_clearance_gain; using 0.20 m");
  }
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".path_hysteresis_confirmations", rclcpp::ParameterValue(1));
  node->get_parameter(
    name_ + ".path_hysteresis_confirmations", path_hysteresis_confirmations_);
  if (path_hysteresis_confirmations_ < 1) {
    path_hysteresis_confirmations_ = 1;
    RCLCPP_WARN(logger_, "Invalid path_hysteresis_confirmations; using 1");
  }
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".path_clearance_search_radius", rclcpp::ParameterValue(1.0));
  node->get_parameter(name_ + ".path_clearance_search_radius", path_clearance_search_radius_);
  if (!std::isfinite(path_clearance_search_radius_) || path_clearance_search_radius_ <= 0.0 ||
    path_clearance_search_radius_ > 10.0)
  {
    path_clearance_search_radius_ = 1.0;
    RCLCPP_WARN(logger_, "Invalid path_clearance_search_radius; using 1.0 m");
  }
  RCLCPP_INFO(
    logger_,
    "Theta* route scoring: w_euc=%.2f w_traversal=%.2f w_clearance=%.2f clearance_cap=%.2f m "
    "retain_length_ratio=%.2f switch_confirmations=%d",
    planner_->w_euc_cost_, planner_->w_traversal_cost_, planner_->w_clearance_cost_,
    path_clearance_search_radius_,
    path_hysteresis_max_length_ratio_, path_hysteresis_confirmations_);
}

void ThetaStarPlanner::cleanup()
{
  RCLCPP_INFO(logger_, "CleaningUp plugin %s of type nav2_theta_star_planner", name_.c_str());
  have_cached_path_ = false;
  cached_path_.poses.clear();
  pending_better_candidate_count_ = 0;
  current_goal_pose_ = geometry_msgs::msg::PoseStamped();
  cached_goal_pose_ = geometry_msgs::msg::PoseStamped();
  planner_.reset();
  costmap_ros_.reset();
  plan_status_publisher_.reset();
}

void ThetaStarPlanner::activate()
{
  RCLCPP_INFO(logger_, "Activating plugin %s of type nav2_theta_star_planner", name_.c_str());
  // Add callback for dynamic parameters
  auto node = parent_node_.lock();
  if (plan_status_publisher_) plan_status_publisher_->on_activate();
  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(&ThetaStarPlanner::dynamicParametersCallback, this, std::placeholders::_1));
}

void ThetaStarPlanner::deactivate()
{
  RCLCPP_INFO(logger_, "Deactivating plugin %s of type nav2_theta_star_planner", name_.c_str());
  if (plan_status_publisher_ && plan_status_publisher_->is_activated()) {
    plan_status_publisher_->on_deactivate();
  }
  have_cached_path_ = false;
  cached_path_.poses.clear();
  pending_better_candidate_count_ = 0;
  current_goal_pose_ = geometry_msgs::msg::PoseStamped();
  cached_goal_pose_ = geometry_msgs::msg::PoseStamped();
}

void ThetaStarPlanner::publishPlanOutcome(
  const char * state, const geometry_msgs::msg::PoseStamped & goal)
{
  if (!plan_status_publisher_ || !plan_status_publisher_->is_activated()) return;
  std::ostringstream output;
  const auto event_stamp = static_cast<builtin_interfaces::msg::Time>(clock_->now());
  const auto event_steady_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
  const auto sequence = ++plan_sequence_;
  const auto request_id = "plan:" + std::to_string(goal.header.stamp.sec) + ':' +
    std::to_string(goal.header.stamp.nanosec);
  output << std::setprecision(17)
         << "instance=" << plan_instance_id_ << ";seq=" << sequence
         << ";request_id=" << request_id
         << ";state=" << state << ";frame=" << goal.header.frame_id
         << ";target_sec=" << goal.header.stamp.sec
         << ";target_nanosec=" << goal.header.stamp.nanosec
         << ";x=" << goal.pose.position.x << ";y=" << goal.pose.position.y
         << ";z=" << goal.pose.position.z
         << ";qx=" << goal.pose.orientation.x << ";qy=" << goal.pose.orientation.y
         << ";qz=" << goal.pose.orientation.z << ";qw=" << goal.pose.orientation.w
         << ";event_sec=" << event_stamp.sec << ";event_nanosec=" << event_stamp.nanosec
         << ";event_steady_ns=" << event_steady_ns;
  std_msgs::msg::String message;
  message.data = output.str();
  plan_status_publisher_->publish(message);
}

nav_msgs::msg::Path ThetaStarPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  nav_msgs::msg::Path global_path;
  const auto start_time = std::chrono::steady_clock::now();
  current_goal_pose_ = goal;

  const auto lock_wait_start = std::chrono::steady_clock::now();
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(planner_->costmap_->getMutex()));
  const auto lock_acquired = std::chrono::steady_clock::now();
  obstacle_field_build_ms_ = 0.0;
  const auto log_planning_timing = [this, start_time, lock_wait_start, lock_acquired](
      const char * result) {
      const auto finished = std::chrono::steady_clock::now();
      const double lock_wait_ms = std::chrono::duration<double, std::milli>(
        lock_acquired - lock_wait_start).count();
      const double planning_ms = std::chrono::duration<double, std::milli>(
        finished - lock_acquired).count();
      const double total_ms = std::chrono::duration<double, std::milli>(
        finished - start_time).count();
      RCLCPP_INFO_THROTTLE(
        logger_, *clock_, 5000,
        "Theta* timing: result=%s costmap_lock_wait=%.3f ms planning=%.3f ms "
        "distance_field=%.3f ms total=%.3f ms map=%ux%u nodes_opened=%d",
        result, lock_wait_ms, planning_ms, obstacle_field_build_ms_, total_ms,
        planner_->costmap_->getSizeInCellsX(), planner_->costmap_->getSizeInCellsY(),
        planner_->nodes_opened);
    };

  // Corner case of start and goal beeing on the same cell
  unsigned int mx_start, my_start, mx_goal, my_goal;
  if (!planner_->costmap_->worldToMap(
      start.pose.position.x, start.pose.position.y, mx_start, my_start))
  {
    RCLCPP_WARN(logger_,
      "Navigation planner: category=global_start_out_of_bounds point=(%.3f,%.3f)",
      start.pose.position.x, start.pose.position.y);
    have_cached_path_ = false;
    cached_path_.poses.clear();
    pending_better_candidate_count_ = 0;
    log_planning_timing("start_outside_costmap");
    publishPlanOutcome("no_path", goal);
    return global_path;
  }

  if (!planner_->costmap_->worldToMap(
      goal.pose.position.x, goal.pose.position.y, mx_goal, my_goal))
  {
    RCLCPP_WARN(logger_,
      "Navigation planner: category=static_goal_out_of_bounds point=(%.3f,%.3f) "
      "goal_stamp=%d.%09u",
      goal.pose.position.x, goal.pose.position.y,
      goal.header.stamp.sec, goal.header.stamp.nanosec);
    have_cached_path_ = false;
    cached_path_.poses.clear();
    pending_better_candidate_count_ = 0;
    log_planning_timing("goal_outside_costmap");
    publishPlanOutcome("invalid_goal", goal);
    return global_path;
  }

  if (mx_start == mx_goal && my_start == my_goal) {
    const auto cost = planner_->costmap_->getCost(mx_start, my_start);
    if (!((cost == UNKNOWN_COST && planner_->allow_unknown_) ||
      cost <= MAX_NON_OBSTACLE_COST ||
      cost == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE))
    {
      if (cost == nav2_costmap_2d::LETHAL_OBSTACLE)
      {
        RCLCPP_ERROR(
          logger_,
          "Navigation planner: category=static_goal_invalid "
          "goal=(%.2f,%.2f) cost=%u goal_stamp=%d.%09u; "
          "choose another goal",
          goal.pose.position.x, goal.pose.position.y, static_cast<unsigned int>(cost),
          goal.header.stamp.sec, goal.header.stamp.nanosec);
      } else {
        RCLCPP_WARN(
          logger_,
          "Navigation planner: category=static_start_goal_cell_invalid "
          "cell=(%u,%u) world=(%.3f,%.3f) cost=%u goal_stamp=%d.%09u",
          mx_start, my_start, start.pose.position.x, start.pose.position.y,
          static_cast<unsigned int>(cost), goal.header.stamp.sec, goal.header.stamp.nanosec);
      }
      have_cached_path_ = false;
      cached_path_.poses.clear();
      pending_better_candidate_count_ = 0;
      log_planning_timing("same_cell_unsafe");
      publishPlanOutcome("no_path", goal);
      return global_path;
    }
    global_path.header.stamp = clock_->now();
    global_path.header.frame_id = global_frame_;
    geometry_msgs::msg::PoseStamped pose;
    pose.header = global_path.header;
    pose.pose.position.z = 0.0;

    pose.pose = start.pose;
    // if we have a different start and goal orientation, set the unique path pose to the goal
    // orientation, unless use_final_approach_orientation=true where we need it to be the start
    // orientation to avoid movement from the local planner
    if (start.pose.orientation != goal.pose.orientation && !use_final_approach_orientation_) {
      pose.pose.orientation = goal.pose.orientation;
    }
    global_path.poses.push_back(pose);
    // A one-pose plan means this goal is already in the robot's current cell. Do not
    // leave the previous goal's multi-pose route available to the next replan.
    have_cached_path_ = false;
    cached_path_.poses.clear();
    pending_better_candidate_count_ = 0;
    log_planning_timing("same_cell_goal");
    publishPlanOutcome("planned", goal);
    return global_path;
  }

  planner_->robot_footprint_ = costmap_ros_->getRobotFootprint();
  planner_->setStartAndGoal(start, goal);
  obstacle_field_ready_for_plan_ = false;
  buildObstacleDistanceField();
  obstacle_field_ready_for_plan_ = true;
  RCLCPP_DEBUG(
    logger_, "Got the src and dst... (%i, %i) && (%i, %i)",
    planner_->src_.x, planner_->src_.y, planner_->dst_.x, planner_->dst_.y);
  getPlan(global_path);
  // check if a plan is generated
  size_t plan_size = global_path.poses.size();
  if (plan_size > 0) {
    global_path.poses.back().pose.orientation = goal.pose.orientation;
  }

  // If use_final_approach_orientation=true, interpolate the last pose orientation from the
  // previous pose to set the orientation to the 'final approach' orientation of the robot so
  // it does not rotate.
  // And deal with corner case of plan of length 1
  if (use_final_approach_orientation_) {
    if (plan_size == 1) {
      global_path.poses.back().pose.orientation = start.pose.orientation;
    } else if (plan_size > 1) {
      double dx, dy, theta;
      auto last_pose = global_path.poses.back().pose.position;
      auto approach_pose = global_path.poses[plan_size - 2].pose.position;
      dx = last_pose.x - approach_pose.x;
      dy = last_pose.y - approach_pose.y;
      theta = atan2(dy, dx);
      global_path.poses.back().pose.orientation =
        nav2_util::geometry_utils::orientationAroundZAxis(theta);
    }
  }

  // The discrete search ends at the goal cell center. Normalize that endpoint
  // to the transformed request pose so downstream consumers can enforce one
  // exact endpoint contract without guessing the costmap resolution or origin.
  // First prove that the generated endpoint and requested pose occupy the same
  // searched costmap cell. applyPathHysteresis() then collision-checks the exact
  // final segment before it can adopt or retain this route.
  bool invalid_endpoint = false;
  if (global_path.poses.size() > 1 &&
    global_path.header.frame_id == goal.header.frame_id)
  {
    auto & endpoint = global_path.poses.back().pose.position;
    unsigned int endpoint_mx = 0;
    unsigned int endpoint_my = 0;
    if (!planner_->costmap_->worldToMap(endpoint.x, endpoint.y, endpoint_mx, endpoint_my) ||
      endpoint_mx != mx_goal || endpoint_my != my_goal)
    {
      RCLCPP_WARN(
        logger_, "Theta* goal endpoint cell mismatch: endpoint=(%.3f,%.3f) "
        "goal=(%.3f,%.3f) endpoint_cell=(%u,%u) goal_cell=(%u,%u)",
        endpoint.x, endpoint.y, goal.pose.position.x, goal.pose.position.y,
        endpoint_mx, endpoint_my, mx_goal, my_goal);
      global_path.poses.clear();
      have_cached_path_ = false;
      cached_path_.poses.clear();
      pending_better_candidate_count_ = 0;
      pending_better_candidate_path_.poses.clear();
      invalid_endpoint = true;
    } else {
      endpoint.x = goal.pose.position.x;
      endpoint.y = goal.pose.position.y;
    }
  }

  // getPlan() has already dropped the cache for every genuine planning failure. If it
  // survived an empty plan, only the robot's own cell was unsafe, so hysteresis may
  // still serve the cache and keep the robot moving off that cell.
  if (plan_size > 0 || have_cached_path_) {
    applyPathHysteresis(start, global_path);
  }

  if (!global_path.poses.empty()) {
    const auto & endpoint = global_path.poses.back().pose.position;
    const auto endpoint_result = checkGoalCellEndpoint(
      global_path.header.frame_id, global_path.poses.size(), endpoint.x, endpoint.y,
      goal.header.frame_id, goal.pose.position.x, goal.pose.position.y);
    if (endpoint_result == GoalCellEndpointResult::ENDPOINT_MISMATCH)
    {
      RCLCPP_WARN(
        logger_, "Theta* goal endpoint contract failed: result=endpoint_mismatch "
        "endpoint=(%.3f,%.3f) goal=(%.3f,%.3f) tolerance=%.3f frame=%s goal_frame=%s",
        endpoint.x, endpoint.y, goal.pose.position.x, goal.pose.position.y,
        kGoalEndpointPositionTolerance,
        global_path.header.frame_id.c_str(), goal.header.frame_id.c_str());
      global_path.poses.clear();
      have_cached_path_ = false;
      cached_path_.poses.clear();
      pending_better_candidate_count_ = 0;
      pending_better_candidate_path_.poses.clear();
      invalid_endpoint = true;
    }
  }

  const auto stop_time = std::chrono::steady_clock::now();
  auto dur = std::chrono::duration_cast<std::chrono::microseconds>(stop_time - start_time);
  RCLCPP_DEBUG(logger_, "the time taken is : %i", static_cast<int>(dur.count()));
  RCLCPP_DEBUG(logger_, "the nodes_opened are:  %i", planner_->nodes_opened);
  log_planning_timing(global_path.poses.empty() ? "failed" : "planned");
  publishPlanOutcome(
    invalid_endpoint ? "invalid_result" :
    (global_path.poses.empty() ? "no_path" : "planned"), goal);
  return global_path;
}

void ThetaStarPlanner::getPlan(nav_msgs::msg::Path & global_path)
{
  std::vector<coordsW> path;
  if (planner_->isUnsafeToPlan()) {
    double start_x, start_y, goal_x, goal_y;
    planner_->costmap_->mapToWorld(
      planner_->src_.x, planner_->src_.y, start_x, start_y);
    planner_->costmap_->mapToWorld(
      planner_->dst_.x, planner_->dst_.y, goal_x, goal_y);
    const auto start_cost = planner_->costmap_->getCost(
      planner_->src_.x, planner_->src_.y);
    const auto goal_cost = planner_->costmap_->getCost(
      planner_->dst_.x, planner_->dst_.y);
    if (goal_cost == nav2_costmap_2d::LETHAL_OBSTACLE)
    {
      RCLCPP_ERROR(
        logger_,
        "Navigation planner: category=static_goal_invalid "
        "goal=(%.2f,%.2f) cost=%u goal_stamp=%d.%09u; "
        "choose another goal",
        goal_x, goal_y, static_cast<unsigned int>(goal_cost),
        current_goal_pose_.header.stamp.sec, current_goal_pose_.header.stamp.nanosec);
    } else {
      RCLCPP_ERROR(
        logger_,
        "Navigation planner: category=global_start_or_goal_invalid "
        "start=(%.2f,%.2f cost=%u) goal=(%.2f,%.2f cost=%u) goal_stamp=%d.%09u",
        start_x, start_y, static_cast<unsigned int>(start_cost),
        goal_x, goal_y, static_cast<unsigned int>(goal_cost),
        current_goal_pose_.header.stamp.sec, current_goal_pose_.header.stamp.nanosec);
    }
    global_path.poses.clear();
    // A cached path may be used to leave only a cell known to be lethal under the
    // robot. Unknown starts are not recoverable this way when allow_unknown is false.
    // The goal must remain safe so the cache cannot bypass goal validation.
    if (start_cost != nav2_costmap_2d::LETHAL_OBSTACLE || !planner_->isGoalSafe()) {
      have_cached_path_ = false;
      cached_path_.poses.clear();
      pending_better_candidate_count_ = 0;
    }
  } else if (planner_->generatePath(path)) {
    global_path = linearInterpolation(path, planner_->costmap_->getResolution());
  } else {
    double start_x, start_y, goal_x, goal_y;
    planner_->costmap_->mapToWorld(
      planner_->src_.x, planner_->src_.y, start_x, start_y);
    planner_->costmap_->mapToWorld(
      planner_->dst_.x, planner_->dst_.y, goal_x, goal_y);
    const auto start_cost = planner_->costmap_->getCost(
      planner_->src_.x, planner_->src_.y);
    const auto goal_cost = planner_->costmap_->getCost(
      planner_->dst_.x, planner_->dst_.y);
    RCLCPP_ERROR(
      logger_,
      "Navigation planner: category=global_no_path "
      "start=(%.2f,%.2f cost=%u) goal=(%.2f,%.2f cost=%u) goal_stamp=%d.%09u",
      start_x, start_y, static_cast<unsigned int>(start_cost),
      goal_x, goal_y, static_cast<unsigned int>(goal_cost),
      current_goal_pose_.header.stamp.sec, current_goal_pose_.header.stamp.nanosec);
    const auto & rejection = planner_->first_search_rejection_;
    if (rejection.valid) {
      RCLCPP_WARN(logger_,
        "Navigation planner: category=global_no_path_first_rejection "
        "cell=(%d,%d) world=(%.3f,%.3f) cost=%d kind=%s",
        rejection.x, rejection.y, rejection.world_x, rejection.world_y,
        rejection.cost, rejection.kind);
    } else {
      RCLCPP_WARN(logger_,
        "Navigation planner: category=global_no_path_first_rejection "
        "kind=none reason=search_exhausted_without_rejected_transition");
    }
    std::size_t free_cells = 0;
    std::size_t soft_cost_cells = 0;
    std::size_t inscribed_cells = 0;
    std::size_t lethal_cells = 0;
    std::size_t unknown_cells = 0;
    for (unsigned int y = 0; y < planner_->costmap_->getSizeInCellsY(); ++y) {
      for (unsigned int x = 0; x < planner_->costmap_->getSizeInCellsX(); ++x) {
        const auto cell_cost = planner_->costmap_->getCost(x, y);
        if (cell_cost == nav2_costmap_2d::NO_INFORMATION) {
          ++unknown_cells;
        } else if (cell_cost == nav2_costmap_2d::LETHAL_OBSTACLE) {
          ++lethal_cells;
        } else if (cell_cost == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
          ++inscribed_cells;
        } else if (cell_cost == nav2_costmap_2d::FREE_SPACE) {
          ++free_cells;
        } else {
          ++soft_cost_cells;
        }
      }
    }
    RCLCPP_WARN(
      logger_,
      "Theta* disconnected search summary: map=%ux%u resolution=%.3f m "
      "cells{free=%zu soft=%zu inscribed=%zu lethal=%zu unknown=%zu} nodes_opened=%d",
      planner_->costmap_->getSizeInCellsX(), planner_->costmap_->getSizeInCellsY(),
      planner_->costmap_->getResolution(), free_cells, soft_cost_cells, inscribed_cells,
      lethal_cells, unknown_cells, planner_->nodes_opened);
    global_path.poses.clear();
    // A genuine search failure: never serve a stale path in its place, the recovery
    // behaviours upstream have to see it.
    have_cached_path_ = false;
    cached_path_.poses.clear();
    pending_better_candidate_count_ = 0;
    pending_better_candidate_path_.poses.clear();
  }
  global_path.header.stamp = clock_->now();
  global_path.header.frame_id = global_frame_;
}

bool ThetaStarPlanner::pathTraversable(
  const nav_msgs::msg::Path & path,
  const geometry_msgs::msg::PoseStamped & robot,
  PathMetrics * metrics, theta_star::ThetaStar::Rejection * rejection) const
{
  if (rejection != nullptr) {
    *rejection = {};
  }
  if (metrics != nullptr) {
    *metrics = PathMetrics{};
    metrics->length = nav2_util::geometry_utils::calculate_path_length(path);
    metrics->minimum_obstacle_gap = path_clearance_search_radius_;
  }
  if (path.poses.empty()) {
    return false;
  }
  unsigned int robot_x, robot_y;
  if (!planner_->costmap_->worldToMap(
      robot.pose.position.x, robot.pose.position.y, robot_x, robot_y))
  {
    const auto * map = planner_->costmap_;
    const int x = static_cast<int>(std::floor(
        (robot.pose.position.x - map->getOriginX()) / map->getResolution()));
    const int y = static_cast<int>(std::floor(
        (robot.pose.position.y - map->getOriginY()) / map->getResolution()));
    planner_->checkCell(x, y, false, false, rejection);
    return false;
  }
  const auto map_pose = [&](const geometry_msgs::msg::PoseStamped & pose, int & x, int & y) {
      const auto * map = planner_->costmap_;
      x = static_cast<int>(std::floor((pose.pose.position.x - map->getOriginX()) /
          map->getResolution()));
      y = static_cast<int>(std::floor((pose.pose.position.y - map->getOriginY()) /
          map->getResolution()));
      return x >= 0 && y >= 0 && x < static_cast<int>(map->getSizeInCellsX()) &&
             y < static_cast<int>(map->getSizeInCellsY());
    };
  int x, y;
  if (!map_pose(path.poses.front(), x, y)) {
    planner_->checkCell(x, y, false, false, rejection);
    return false;
  }
  bool left = !planner_->isStartExitCell(x, y);
  // The robot may already occupy a lethal cell while a retained route is used to escape.
  if ((x != static_cast<int>(robot_x) || y != static_cast<int>(robot_y)) &&
    !planner_->checkCell(x, y, left, false, rejection))
  {
    return false;
  }
  const auto visit = [&](int mx, int my) {
      if (metrics != nullptr) {
        metrics->traversal_cost += planner_->traversalCostAtCell(mx, my);
        metrics->clearance_cost += planner_->clearanceDeficitAtCell(mx, my) *
          planner_->costmap_->getResolution();
        bool capped = true;
        const double gap = obstacleGapAtCell(mx, my, capped);
        if (gap < metrics->minimum_obstacle_gap) {
          metrics->minimum_obstacle_gap = gap;
          metrics->obstacle_gap_capped = capped;
        }
      }
    };
  for (std::size_t i = 1; i < path.poses.size(); ++i) {
    int next_x, next_y;
    if (!map_pose(path.poses[i], next_x, next_y)) {
      planner_->checkCell(next_x, next_y, left, false, rejection);
      return false;
    }
    bool terminal_goal = next_x == planner_->dst_.x && next_y == planner_->dst_.y;
    for (std::size_t j = i + 1; terminal_goal && j < path.poses.size(); ++j) {
      int future_x, future_y;
      terminal_goal = map_pose(path.poses[j], future_x, future_y) &&
        future_x == next_x && future_y == next_y;
    }
    const auto & from = path.poses[i - 1].pose.position;
    const auto & to = path.poses[i].pose.position;
    if (!planner_->traverseWorldSegment(
        from.x, from.y, to.x, to.y, left, rejection, visit, terminal_goal))
    {
      return false;
    }
    x = next_x;
    y = next_y;
  }
  return true;
}

void ThetaStarPlanner::buildObstacleDistanceField()
{
  const auto build_started = std::chrono::steady_clock::now();
  auto * costmap = planner_->costmap_;
  const auto width = costmap->getSizeInCellsX();
  const auto height = costmap->getSizeInCellsY();
  const double resolution = costmap->getResolution();
  obstacle_distance_field_width_ = width;
  obstacle_distance_field_height_ = height;
  obstacle_distance_field_resolution_ = resolution;
  const auto cell_count = static_cast<std::size_t>(width) * height;
  obstacle_distance_field_.assign(cell_count, static_cast<float>(path_clearance_search_radius_));
  planner_->setObstacleDistanceField(
    &obstacle_distance_field_, width, height, resolution, path_clearance_search_radius_);
  if (width == 0 || height == 0 || !std::isfinite(resolution) || resolution <= 0.0) {
    obstacle_field_build_ms_ = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - build_started).count();
    return;
  }

  constexpr double infinity = 1.0e20;
  std::vector<double> horizontal(cell_count, infinity);
  std::vector<double> line_values(std::max(width, height), infinity);
  std::vector<double> line_distances;
  std::vector<std::size_t> sites(std::max(width, height));
  std::vector<double> boundaries(std::max(width, height) + 1);

  const auto transform_1d = [&](std::size_t size) {
      line_distances.assign(size, infinity);
      std::size_t envelope_size = 0;
      for (std::size_t q = 0; q < size; ++q) {
        if (line_values[q] >= infinity * 0.5) continue;
        if (envelope_size == 0) {
          sites[0] = q;
          boundaries[0] = -infinity;
          boundaries[1] = infinity;
          envelope_size = 1;
          continue;
        }
        double intersection = 0.0;
        while (true) {
          const auto previous = sites[envelope_size - 1];
          const double q_value = static_cast<double>(q);
          const double previous_value = static_cast<double>(previous);
          intersection = ((line_values[q] + q_value * q_value) -
            (line_values[previous] + previous_value * previous_value)) /
            (2.0 * (q_value - previous_value));
          if (intersection > boundaries[envelope_size - 1] || envelope_size == 1) break;
          --envelope_size;
        }
        sites[envelope_size] = q;
        boundaries[envelope_size] = intersection;
        ++envelope_size;
        boundaries[envelope_size] = infinity;
      }
      if (envelope_size == 0) return;
      std::size_t envelope_index = 0;
      for (std::size_t q = 0; q < size; ++q) {
        while (envelope_index + 1 < envelope_size &&
          boundaries[envelope_index + 1] < static_cast<double>(q))
        {
          ++envelope_index;
        }
        const double delta = static_cast<double>(q) -
          static_cast<double>(sites[envelope_index]);
        line_distances[q] = delta * delta + line_values[sites[envelope_index]];
      }
    };

  for (std::uint32_t y = 0; y < height; ++y) {
    std::fill(line_values.begin(), line_values.begin() + width, infinity);
    for (std::uint32_t x = 0; x < width; ++x) {
      const auto cost = costmap->getCost(x, y);
      if (cost == nav2_costmap_2d::LETHAL_OBSTACLE ||
        cost == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE ||
        (cost == nav2_costmap_2d::NO_INFORMATION && !planner_->allow_unknown_))
      {
        line_values[x] = 0.0;
      }
    }
    transform_1d(width);
    for (std::uint32_t x = 0; x < width; ++x) {
      horizontal[static_cast<std::size_t>(y) * width + x] = line_distances[x];
    }
  }

  const double half_cell_diagonal = resolution * std::sqrt(0.5);
  for (std::uint32_t x = 0; x < width; ++x) {
    for (std::uint32_t y = 0; y < height; ++y) {
      line_values[y] = horizontal[static_cast<std::size_t>(y) * width + x];
    }
    transform_1d(height);
    for (std::uint32_t y = 0; y < height; ++y) {
      const auto index = static_cast<std::size_t>(y) * width + x;
      const double center_distance = std::sqrt(line_distances[y]) * resolution;
      const double cell_gap = std::max(0.0, center_distance - half_cell_diagonal);
      obstacle_distance_field_[index] = static_cast<float>(std::min(
          path_clearance_search_radius_, cell_gap));
    }
  }
  obstacle_field_build_ms_ = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - build_started).count();
}

double ThetaStarPlanner::obstacleGapAtCell(
  unsigned int mx, unsigned int my, bool & capped) const
{
  if (mx >= obstacle_distance_field_width_ || my >= obstacle_distance_field_height_ ||
    obstacle_distance_field_resolution_ <= 0.0 ||
    obstacle_distance_field_.size() != static_cast<std::size_t>(
      obstacle_distance_field_width_) * obstacle_distance_field_height_)
  {
    capped = true;
    return path_clearance_search_radius_;
  }
  const auto gap = static_cast<double>(
    obstacle_distance_field_[static_cast<std::size_t>(my) * obstacle_distance_field_width_ + mx]);
  capped = gap >= path_clearance_search_radius_;
  return gap;
}

void ThetaStarPlanner::measureExclusivePath(
  const nav_msgs::msg::Path & path, const nav_msgs::msg::Path & other,
  PathMetrics & metrics) const
{
  metrics.exclusive_length = 0.0;
  metrics.exclusive_traversal_cost = 0.0;
  metrics.exclusive_clearance_cost = 0.0;
  metrics.exclusive_minimum_obstacle_gap = path_clearance_search_radius_;
  metrics.exclusive_high_cost_length = 0.0;
  metrics.exclusive_score = 0.0;
  metrics.has_exclusive_segment = false;
  metrics.exclusive_obstacle_gap_capped = true;
  if (path.poses.size() < 2 || other.poses.size() < 2 ||
    path.header.frame_id != other.header.frame_id)
  {
    return;
  }

  const double resolution = planner_->costmap_->getResolution();
  const double sample_step = std::max(0.025, resolution * 0.5);
  const double shared_route_tolerance = std::max(0.10, 2.0 * resolution);
  const auto distance_to_path = [](double x, double y, const nav_msgs::msg::Path & route) {
      double nearest = std::numeric_limits<double>::max();
      for (std::size_t i = 1; i < route.poses.size(); ++i) {
        const auto & first = route.poses[i - 1].pose.position;
        const auto & last = route.poses[i].pose.position;
        const double dx = last.x - first.x;
        const double dy = last.y - first.y;
        const double squared_length = dx * dx + dy * dy;
        const double fraction = squared_length > 1e-12 ? std::clamp(
          ((x - first.x) * dx + (y - first.y) * dy) / squared_length, 0.0, 1.0) : 0.0;
        nearest = std::min(nearest, std::hypot(
            x - (first.x + fraction * dx), y - (first.y + fraction * dy)));
      }
      return nearest;
    };

  for (std::size_t segment = 1; segment < path.poses.size(); ++segment) {
    const auto & first = path.poses[segment - 1].pose.position;
    const auto & last = path.poses[segment].pose.position;
    const double dx = last.x - first.x;
    const double dy = last.y - first.y;
    const double length = std::hypot(dx, dy);
    if (length <= 1e-9) continue;
    const auto steps = std::max<std::size_t>(
      1, static_cast<std::size_t>(std::ceil(length / sample_step)));
    const double step_length = length / static_cast<double>(steps);
    for (std::size_t step = 0; step < steps; ++step) {
      const double fraction = (static_cast<double>(step) + 0.5) / steps;
      const double x = first.x + fraction * dx;
      const double y = first.y + fraction * dy;
      if (distance_to_path(x, y, other) <= shared_route_tolerance) continue;
      metrics.has_exclusive_segment = true;
      metrics.exclusive_length += step_length;
      unsigned int mx = 0;
      unsigned int my = 0;
      if (!planner_->costmap_->worldToMap(x, y, mx, my)) continue;
      const auto cost = planner_->costmap_->getCost(mx, my);
      metrics.exclusive_traversal_cost +=
        planner_->traversalCostAtCell(static_cast<int>(mx), static_cast<int>(my)) * step_length;
      metrics.exclusive_clearance_cost +=
        planner_->clearanceDeficitAtCell(static_cast<int>(mx), static_cast<int>(my)) *
        step_length;
      if (cost >= 128 && cost <= MAX_NON_OBSTACLE_COST) {
        metrics.exclusive_high_cost_length += step_length;
      }
      bool capped = true;
      const double gap = obstacleGapAtCell(mx, my, capped);
      if (gap < metrics.exclusive_minimum_obstacle_gap) {
        metrics.exclusive_minimum_obstacle_gap = gap;
        metrics.exclusive_obstacle_gap_capped = capped;
      }
    }
  }
  if (metrics.has_exclusive_segment) {
    metrics.exclusive_score = planner_->w_euc_cost_ * metrics.exclusive_length +
      metrics.exclusive_traversal_cost;
  } else {
    metrics.exclusive_minimum_obstacle_gap = -1.0;
  }
}

void ThetaStarPlanner::logPathDecision(
  const char * reason, const char * selected,
  const PathMetrics * retained, const PathMetrics * candidate,
  double retained_score, double candidate_score) const
{
  const auto length = [](const PathMetrics * metrics) {
      return metrics == nullptr ? -1.0 : metrics->length;
    };
  const auto traversal = [](const PathMetrics * metrics) {
      return metrics == nullptr ? -1.0 : metrics->traversal_cost;
    };
  const auto clearance = [](const PathMetrics * metrics) {
      return metrics == nullptr ? -1.0 : metrics->clearance_cost;
    };
  const auto gap = [](const PathMetrics * metrics) {
      return metrics == nullptr ? -1.0 : metrics->minimum_obstacle_gap;
    };
  const auto cap = [](const PathMetrics * metrics) {
      return metrics == nullptr || metrics->obstacle_gap_capped ? " (capped)" : "";
    };
  const auto exclusive_length = [](const PathMetrics * metrics) {
      return metrics == nullptr ? -1.0 : metrics->exclusive_length;
    };
  const auto exclusive_gap = [](const PathMetrics * metrics) {
      return metrics == nullptr || !metrics->has_exclusive_segment ?
             -1.0 : metrics->exclusive_minimum_obstacle_gap;
    };
  const auto exclusive_high_cost_length = [](const PathMetrics * metrics) {
      return metrics == nullptr ? -1.0 : metrics->exclusive_high_cost_length;
    };
  const auto exclusive_clearance = [](const PathMetrics * metrics) {
      return metrics == nullptr ? -1.0 : metrics->exclusive_clearance_cost;
    };
  const auto exclusive_score = [](const PathMetrics * metrics) {
      return metrics == nullptr ? -1.0 : metrics->exclusive_score;
    };
  const auto exclusive_gap_status = [](const PathMetrics * metrics) {
      if (metrics == nullptr) return "unavailable";
      return metrics->has_exclusive_segment ?
             (metrics->exclusive_obstacle_gap_capped ? "capped" : "measured") : "shared";
    };
  RCLCPP_INFO(
    logger_,
    "Theta* route decision: reason=%s selected=%s "
    "candidate={length=%.2f m traversal=%.3f clearance_bias=%.3f m "
    "min_lethal_gap=%.2f m%s score=%.3f "
    "exclusive={length=%.2f m gap=%.2f m(%s) clearance_bias=%.3f m "
    "high_cost=%.2f m score=%.3f}} "
    "retained={length=%.2f m traversal=%.3f clearance_bias=%.3f m "
    "min_lethal_gap=%.2f m%s score=%.3f "
    "exclusive={length=%.2f m gap=%.2f m(%s) clearance_bias=%.3f m "
    "high_cost=%.2f m score=%.3f}} "
    "switch_evidence=%d/%d",
    reason, selected, length(candidate), traversal(candidate), clearance(candidate),
    gap(candidate), cap(candidate), candidate_score, exclusive_length(candidate),
    exclusive_gap(candidate), exclusive_gap_status(candidate), exclusive_clearance(candidate),
    exclusive_high_cost_length(candidate), exclusive_score(candidate),
    length(retained), traversal(retained), clearance(retained), gap(retained), cap(retained),
    retained_score, exclusive_length(retained), exclusive_gap(retained),
    exclusive_gap_status(retained), exclusive_clearance(retained),
    exclusive_high_cost_length(retained), exclusive_score(retained),
    pending_better_candidate_count_, path_hysteresis_confirmations_);
}

bool ThetaStarPlanner::candidateRouteMatchesPending(
  const nav_msgs::msg::Path & candidate) const
{
  const auto & pending = pending_better_candidate_path_;
  if (candidate.header.frame_id != pending.header.frame_id ||
    candidate.poses.size() < 2 || pending.poses.size() < 2)
  {
    return false;
  }
  const double tolerance = std::max(
    0.25, 2.0 * planner_->costmap_->getResolution());
  const double candidate_length =
    nav2_util::geometry_utils::calculate_path_length(candidate);
  if (candidate_length <= tolerance) {
    return false;
  }

  const auto point_at_distance = [](
    const nav_msgs::msg::Path & path, double distance, double & x, double & y) {
      for (std::size_t i = 1; i < path.poses.size(); ++i) {
        const auto & previous = path.poses[i - 1].pose.position;
        const auto & current = path.poses[i].pose.position;
        const double segment_length = std::hypot(
          current.x - previous.x, current.y - previous.y);
        if (segment_length <= 1e-9) continue;
        if (distance <= segment_length) {
          const double fraction = distance / segment_length;
          x = previous.x + fraction * (current.x - previous.x);
          y = previous.y + fraction * (current.y - previous.y);
          return true;
        }
        distance -= segment_length;
      }
      if (distance <= 1e-6) {
        x = path.poses.back().pose.position.x;
        y = path.poses.back().pose.position.y;
        return true;
      }
      return false;
    };

  const auto distance_to_path = [](double x, double y, const nav_msgs::msg::Path & path) {
      double nearest = std::numeric_limits<double>::max();
      for (std::size_t i = 1; i < path.poses.size(); ++i) {
        const auto & first = path.poses[i - 1].pose.position;
        const auto & last = path.poses[i].pose.position;
        const double dx = last.x - first.x;
        const double dy = last.y - first.y;
        const double segment_length_squared = dx * dx + dy * dy;
        const double fraction = segment_length_squared > 1e-12 ? std::clamp(
          ((x - first.x) * dx + (y - first.y) * dy) / segment_length_squared,
          0.0, 1.0) : 0.0;
        nearest = std::min(nearest, std::hypot(
            x - (first.x + fraction * dx), y - (first.y + fraction * dy)));
      }
      return nearest;
    };

  for (double station = 0.25; station < candidate_length; station += 0.50) {
    double x = 0.0;
    double y = 0.0;
    if (!point_at_distance(candidate, station, x, y) ||
      distance_to_path(x, y, pending) > tolerance)
    {
      return false;
    }
  }
  const auto & endpoint = candidate.poses.back().pose.position;
  return distance_to_path(endpoint.x, endpoint.y, pending) <= tolerance;
}

void ThetaStarPlanner::applyPathHysteresis(
  const geometry_msgs::msg::PoseStamped & start,
  nav_msgs::msg::Path & global_path)
{
  if (!obstacle_field_ready_for_plan_) {
    buildObstacleDistanceField();
  }
  obstacle_field_ready_for_plan_ = false;
  // getPlan() calls in with an empty path when only the start cell was unsafe, so that
  // the cache can still be served. With no candidate there is nothing to compare against
  // and nothing worth caching.
  const bool have_candidate = global_path.poses.size() >= 2;

  const auto adopt = [&]() {
      if (have_candidate) {
        cached_path_ = global_path;
        cached_goal_pose_ = current_goal_pose_;
        have_cached_path_ = true;
      } else {
        have_cached_path_ = false;
        cached_path_.poses.clear();
      }
      pending_better_candidate_count_ = 0;
      pending_better_candidate_path_.poses.clear();
    };
  const auto discard = [&]() {
      have_cached_path_ = false;
      cached_path_.poses.clear();
      pending_better_candidate_count_ = 0;
      pending_better_candidate_path_.poses.clear();
    };

  const double no_score = std::numeric_limits<double>::quiet_NaN();
  PathMetrics candidate_metrics;
  theta_star::ThetaStar::Rejection candidate_rejection;
  const bool candidate_is_safe = have_candidate &&
    pathTraversable(global_path, start, &candidate_metrics, &candidate_rejection);
  if (have_candidate && !candidate_is_safe && candidate_rejection.valid) {
    RCLCPP_WARN(logger_,
      "Navigation planner: category=global_candidate_rejected "
      "cell=(%d,%d) world=(%.3f,%.3f) cost=%d kind=%s",
      candidate_rejection.x, candidate_rejection.y, candidate_rejection.world_x,
      candidate_rejection.world_y, candidate_rejection.cost, candidate_rejection.kind);
  }
  if (path_hysteresis_ratio_ <= 0.0 && candidate_is_safe) {
    adopt();
    logPathDecision("hysteresis_disabled", have_candidate ? "candidate" : "none",
      nullptr, have_candidate ? &candidate_metrics : nullptr, no_score, no_score);
    return;
  }

  // Compare the requested world goal, not rolling costmap cell indices. The costmap
  // origin moves with the robot, so a fixed world goal is expected to get new indices.
  const auto same_goal = [](const geometry_msgs::msg::PoseStamped & lhs,
      const geometry_msgs::msg::PoseStamped & rhs) {
      constexpr double epsilon = 1e-6;
      return lhs.header.frame_id == rhs.header.frame_id &&
             std::abs(lhs.pose.position.x - rhs.pose.position.x) <= epsilon &&
             std::abs(lhs.pose.position.y - rhs.pose.position.y) <= epsilon &&
             std::abs(lhs.pose.orientation.x - rhs.pose.orientation.x) <= epsilon &&
             std::abs(lhs.pose.orientation.y - rhs.pose.orientation.y) <= epsilon &&
             std::abs(lhs.pose.orientation.z - rhs.pose.orientation.z) <= epsilon &&
             std::abs(lhs.pose.orientation.w - rhs.pose.orientation.w) <= epsilon;
    };

  if (!have_cached_path_ || cached_path_.poses.size() < 2) {
    if (candidate_is_safe) {
      adopt();
      logPathDecision("no_retained_route", "candidate",
        nullptr, &candidate_metrics, no_score, no_score);
    } else {
      discard();
      global_path.poses.clear();
      logPathDecision("candidate_failed_safety_check", "none", nullptr,
        have_candidate ? &candidate_metrics : nullptr, no_score, no_score);
    }
    return;
  }
  if (!same_goal(current_goal_pose_, cached_goal_pose_)) {
    if (candidate_is_safe) {
      adopt();
      logPathDecision("goal_changed", "candidate", nullptr, &candidate_metrics,
        no_score, no_score);
    } else {
      discard();
      global_path.poses.clear();
      logPathDecision("candidate_failed_for_new_goal", "none", nullptr,
        have_candidate ? &candidate_metrics : nullptr, no_score, no_score);
    }
    return;
  }

  // Re-anchor at the closest point on the cached polyline, not its closest sampled
  // pose. Using a sampled pose can make the robot backtrack to the previous sample;
  // inserting that pose both before and after the retained suffix also creates a
  // zero-length segment.
  const auto & poses = cached_path_.poses;
  std::size_t closest_segment = 0;
  double closest_fraction = 0.0;
  double min_distance_squared = std::numeric_limits<double>::max();
  for (std::size_t i = 0; i + 1 < poses.size(); ++i) {
    const double dx = poses[i + 1].pose.position.x - poses[i].pose.position.x;
    const double dy = poses[i + 1].pose.position.y - poses[i].pose.position.y;
    const double length_squared = dx * dx + dy * dy;
    const double fraction = length_squared > 1e-12 ? std::clamp(
      ((start.pose.position.x - poses[i].pose.position.x) * dx +
      (start.pose.position.y - poses[i].pose.position.y) * dy) / length_squared,
      0.0, 1.0) : 0.0;
    const double projected_x = poses[i].pose.position.x + fraction * dx;
    const double projected_y = poses[i].pose.position.y + fraction * dy;
    const double offset_x = start.pose.position.x - projected_x;
    const double offset_y = start.pose.position.y - projected_y;
    const double distance_squared = offset_x * offset_x + offset_y * offset_y;
    if (distance_squared < min_distance_squared) {
      min_distance_squared = distance_squared;
      closest_segment = i;
      closest_fraction = fraction;
    }
  }

  // Start the retained path at the robot itself, so TEB's unconditional overwrite of
  // transformed_plan.front() with the robot position is idempotent instead of implicitly
  // creating an unchecked robot -> second-point segment.
  // Keep the rejoin leg as one geometric segment; pathTraversable() rasterizes every
  // costmap cell crossed by it, including cells touched at grid corners.
  nav_msgs::msg::Path retained;
  retained.header = global_path.header;
  auto start_pose = start;
  start_pose.header = global_path.header;
  retained.poses.push_back(start_pose);
  const auto & segment_start = poses[closest_segment];
  const auto & segment_end = poses[closest_segment + 1];
  geometry_msgs::msg::PoseStamped rejoin_pose = segment_start;
  rejoin_pose.pose.position.x = segment_start.pose.position.x + closest_fraction *
    (segment_end.pose.position.x - segment_start.pose.position.x);
  rejoin_pose.pose.position.y = segment_start.pose.position.y + closest_fraction *
    (segment_end.pose.position.y - segment_start.pose.position.y);
  rejoin_pose.pose.position.z = segment_start.pose.position.z + closest_fraction *
    (segment_end.pose.position.z - segment_start.pose.position.z);
  rejoin_pose.header = global_path.header;

  const auto append_if_distinct = [&retained](const geometry_msgs::msg::PoseStamped & pose) {
      const auto & previous = retained.poses.back().pose.position;
      const auto & current = pose.pose.position;
      if (std::hypot(current.x - previous.x, current.y - previous.y) >= 1e-6) {
        retained.poses.push_back(pose);
      }
    };

  append_if_distinct(rejoin_pose);
  for (std::size_t i = closest_segment + 1; i < poses.size(); ++i) {
    auto pose = poses[i];
    pose.header = global_path.header;
    append_if_distinct(pose);
  }

  PathMetrics retained_metrics;
  theta_star::ThetaStar::Rejection retained_rejection;
  const bool retained_is_safe = retained.poses.size() >= 2 &&
    pathTraversable(retained, start, &retained_metrics, &retained_rejection);
  if (retained.poses.size() >= 2 && !retained_is_safe && retained_rejection.valid) {
    RCLCPP_WARN(logger_,
      "Navigation planner: category=retained_path_rejected "
      "cell=(%d,%d) world=(%.3f,%.3f) cost=%d kind=%s",
      retained_rejection.x, retained_rejection.y, retained_rejection.world_x,
      retained_rejection.world_y, retained_rejection.cost, retained_rejection.kind);
  }
  if (!retained_is_safe) {
    if (candidate_is_safe) {
      adopt();
      logPathDecision("retained_route_invalid", "candidate", nullptr,
        &candidate_metrics, no_score, no_score);
    } else {
      discard();
      global_path.poses.clear();
      logPathDecision("both_candidate_and_retained_route_invalid", "none", nullptr,
        have_candidate ? &candidate_metrics : nullptr, no_score, no_score);
    }
    return;
  }

  // The cap is an exceedance limit, so a robot sitting exactly on it still rejoins.
  constexpr double rejoin_distance_tolerance = 1e-9;
  if (std::sqrt(min_distance_squared) >
    path_hysteresis_max_rejoin_distance_ + rejoin_distance_tolerance)
  {
    if (candidate_is_safe) {
      adopt();
      logPathDecision("rejoin_distance_exceeded", "candidate",
        &retained_metrics, &candidate_metrics, no_score, no_score);
    } else {
      discard();
      global_path.poses.clear();
      logPathDecision("invalid_candidate_and_rejoin_distance_exceeded", "none",
        &retained_metrics, have_candidate ? &candidate_metrics : nullptr,
        no_score, no_score);
    }
    return;
  }

  if (have_candidate && !candidate_is_safe) {
    pending_better_candidate_count_ = 0;
    pending_better_candidate_path_.poses.clear();
    logPathDecision("candidate_failed_keep_safe_retained_route", "retained",
      &retained_metrics, &candidate_metrics, no_score, no_score);
    global_path = retained;
    cached_path_.header = retained.header;
    cached_path_.poses.assign(retained.poses.begin() + 1, retained.poses.end());
    return;
  }

  if (!have_candidate && path_hysteresis_ratio_ <= 0.0) {
    discard();
    global_path.poses.clear();
    logPathDecision("hysteresis_disabled_without_candidate", "none",
      &retained_metrics, nullptr, no_score, no_score);
    return;
  }

  // Charge the rejoin gap to the retained path: calculate_path_length now includes the
  // start → closest segment, so a cache the robot has drifted away from loses the
  // comparison naturally without a separate deviation threshold.
  if (have_candidate) {
    measureExclusivePath(global_path, retained, candidate_metrics);
    measureExclusivePath(retained, global_path, retained_metrics);
    const double resolution = planner_->costmap_->getResolution();
    const double retained_score = planner_->w_euc_cost_ * retained_metrics.length +
      retained_metrics.traversal_cost * resolution;
    const double candidate_score = planner_->w_euc_cost_ * candidate_metrics.length +
      candidate_metrics.traversal_cost * resolution;
    const bool within_length_budget = candidate_metrics.length <=
      retained_metrics.length * path_hysteresis_max_length_ratio_;
    const bool have_route_difference = candidate_metrics.has_exclusive_segment &&
      retained_metrics.has_exclusive_segment;
    const bool meaningfully_wider = have_route_difference &&
      candidate_metrics.exclusive_minimum_obstacle_gap >=
      retained_metrics.exclusive_minimum_obstacle_gap + path_hysteresis_min_clearance_gain_;
    const bool materially_less_high_cost = have_route_difference &&
      retained_metrics.exclusive_high_cost_length >=
      candidate_metrics.exclusive_high_cost_length +
      std::max(path_hysteresis_min_clearance_gain_, resolution);
    const bool candidate_clearance_is_comparable = have_route_difference &&
      candidate_metrics.exclusive_minimum_obstacle_gap +
      path_hysteresis_min_clearance_gain_ >=
      retained_metrics.exclusive_minimum_obstacle_gap;
    const bool high_cost_span_is_safer = materially_less_high_cost &&
      candidate_clearance_is_comparable;
    const bool meaningfully_more_centered = have_route_difference &&
      retained_metrics.exclusive_clearance_cost >=
      candidate_metrics.exclusive_clearance_cost + path_hysteresis_min_clearance_gain_;
    const bool meaningfully_safer = within_length_budget &&
      (meaningfully_wider ||
      high_cost_span_is_safer || meaningfully_more_centered);
    const bool meaningfully_cheaper = within_length_budget &&
      candidate_metrics.has_exclusive_segment && retained_metrics.has_exclusive_segment &&
      retained_metrics.exclusive_score >
      candidate_metrics.exclusive_score * (1.0 + path_hysteresis_ratio_);
    const bool much_shorter_without_material_safety_loss =
      retained_metrics.length > candidate_metrics.length * path_hysteresis_max_length_ratio_ &&
      (!candidate_metrics.has_exclusive_segment || !retained_metrics.has_exclusive_segment ||
      candidate_metrics.exclusive_minimum_obstacle_gap + path_hysteresis_min_clearance_gain_ >=
      retained_metrics.exclusive_minimum_obstacle_gap);
    if (meaningfully_safer || meaningfully_cheaper ||
      much_shorter_without_material_safety_loss)
    {
      if (pending_better_candidate_count_ > 0 &&
        candidateRouteMatchesPending(global_path))
      {
        ++pending_better_candidate_count_;
      } else {
        pending_better_candidate_count_ = 1;
      }
      pending_better_candidate_path_ = global_path;
      if (pending_better_candidate_count_ >= path_hysteresis_confirmations_) {
        adopt();
        const char * switch_reason = meaningfully_wider ? "wider_candidate_persisted" :
          (meaningfully_more_centered ? "more_centered_candidate_persisted" :
          (high_cost_span_is_safer ? "lower_high_cost_span_candidate_persisted" :
          (meaningfully_cheaper ? "lower_cost_candidate_persisted" :
          "shorter_candidate_persisted")));
        logPathDecision(
          switch_reason,
          "candidate", &retained_metrics, &candidate_metrics,
          retained_score, candidate_score);
        return;
      }
      logPathDecision("better_candidate_pending_confirmation", "retained",
        &retained_metrics, &candidate_metrics, retained_score, candidate_score);
    } else {
      pending_better_candidate_count_ = 0;
      pending_better_candidate_path_.poses.clear();
      logPathDecision("retain_safe_route", "retained",
        &retained_metrics, &candidate_metrics, retained_score, candidate_score);
    }
    if (!use_final_approach_orientation_) {
      retained.poses.back().pose.orientation = global_path.poses.back().pose.orientation;
    } else if (retained.poses.size() >= 2) {
      const auto & previous = retained.poses[retained.poses.size() - 2].pose.position;
      const auto & last = retained.poses.back().pose.position;
      retained.poses.back().pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(
        std::atan2(last.y - previous.y, last.x - previous.x));
    }
  } else {
    pending_better_candidate_count_ = 0;
    pending_better_candidate_path_.poses.clear();
    logPathDecision("no_fresh_candidate_retain_safe_route", "retained",
      &retained_metrics, nullptr, no_score, no_score);
  }

  global_path = retained;
  // Keep only the route after the temporary robot-to-route connector. Otherwise the
  // next replan can select this old robot pose as the nearest point and connect back
  // to it after the robot has already moved forward.
  cached_path_.header = retained.header;
  cached_path_.poses.assign(retained.poses.begin() + 1, retained.poses.end());
}

nav_msgs::msg::Path ThetaStarPlanner::linearInterpolation(
  const std::vector<coordsW> & raw_path,
  const double & dist_bw_points)
{
  nav_msgs::msg::Path pa;

  if (raw_path.empty()) {
    return pa;
  }

  geometry_msgs::msg::PoseStamped p1;
  for (unsigned int j = 0; j < raw_path.size() - 1; j++) {
    coordsW pt1 = raw_path[j];
    coordsW pt2 = raw_path[j + 1];
    const double dx = pt2.x - pt1.x;
    const double dy = pt2.y - pt1.y;
    p1.pose.position.x = pt1.x;
    p1.pose.position.y = pt1.y;
    if (std::hypot(dx, dy) > 1e-9) {
      p1.pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(std::atan2(dy, dx));
    }
    pa.poses.push_back(p1);

    double distance = std::hypot(pt2.x - pt1.x, pt2.y - pt1.y);
    if (distance <= 1e-9) {
      continue;
    }
    int loops = static_cast<int>(distance / dist_bw_points);
    double sin_alpha = (pt2.y - pt1.y) / distance;
    double cos_alpha = (pt2.x - pt1.x) / distance;
    for (int k = 1; k < loops; k++) {
      p1.pose.position.x = pt1.x + k * dist_bw_points * cos_alpha;
      p1.pose.position.y = pt1.y + k * dist_bw_points * sin_alpha;
      p1.pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(std::atan2(dy, dx));
      pa.poses.push_back(p1);
    }
  }

  // Include the final waypoint. Its orientation is replaced by the requested goal
  // orientation (or the final approach orientation) in createPlan().
  p1.pose.position.x = raw_path.back().x;
  p1.pose.position.y = raw_path.back().y;
  if (raw_path.size() > 1) {
    const coordsW & previous = raw_path[raw_path.size() - 2];
    const double dx = raw_path.back().x - previous.x;
    const double dy = raw_path.back().y - previous.y;
    if (std::hypot(dx, dy) > 1e-9) {
      p1.pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(std::atan2(dy, dx));
    }
  }
  pa.poses.push_back(p1);

  return pa;
}

rcl_interfaces::msg::SetParametersResult
ThetaStarPlanner::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  for (auto parameter : parameters) {
    const auto & type = parameter.get_type();
    const auto & name = parameter.get_name();

    if (type == ParameterType::PARAMETER_INTEGER) {
      if (name == name_ + ".how_many_corners") {
        planner_->how_many_corners_ = parameter.as_int();
      } else if (name == name_ + ".path_hysteresis_confirmations" &&
        parameter.as_int() >= 1)
      {
        path_hysteresis_confirmations_ = parameter.as_int();
      }
    } else if (type == ParameterType::PARAMETER_DOUBLE) {
      if (name == name_ + ".w_euc_cost") {
        planner_->w_euc_cost_ = parameter.as_double();
      } else if (name == name_ + ".w_traversal_cost") {
        planner_->w_traversal_cost_ = parameter.as_double();
      } else if (name == name_ + ".w_clearance_cost") {
        const double weight = parameter.as_double();
        if (std::isfinite(weight) && weight >= 0.0 && weight <= 10.0) {
          planner_->w_clearance_cost_ = weight;
        }
      } else if (name == name_ + ".path_hysteresis_ratio") {
        path_hysteresis_ratio_ = std::max(0.0, parameter.as_double());
      } else if (name == name_ + ".path_hysteresis_max_rejoin_distance") {
        const double distance = parameter.as_double();
        if (std::isfinite(distance) && distance >= 0.0) {
          path_hysteresis_max_rejoin_distance_ = distance;
        }
      } else if (name == name_ + ".path_hysteresis_max_length_ratio") {
        const double ratio = parameter.as_double();
        if (std::isfinite(ratio) && ratio >= 1.0) {
          path_hysteresis_max_length_ratio_ = ratio;
        }
      } else if (name == name_ + ".path_hysteresis_min_clearance_gain") {
        const double gain = parameter.as_double();
        if (std::isfinite(gain) && gain >= 0.0) {
          path_hysteresis_min_clearance_gain_ = gain;
        }
      } else if (name == name_ + ".path_clearance_search_radius") {
        const double radius = parameter.as_double();
        if (std::isfinite(radius) && radius > 0.0 && radius <= 10.0) {
          path_clearance_search_radius_ = radius;
        }
      } else if (name == name_ + ".start_unknown_escape_radius") {
        const double radius = parameter.as_double();
        if (std::isfinite(radius) && radius >= 0.0) {
          planner_->start_unknown_escape_radius_ = radius;
        }
      }
    } else if (type == ParameterType::PARAMETER_BOOL) {
      if (name == name_ + ".use_final_approach_orientation") {
        use_final_approach_orientation_ = parameter.as_bool();
      } else if (name == name_ + ".allow_unknown") {
        planner_->allow_unknown_ = parameter.as_bool();
      }
    }
  }

  result.successful = true;
  return result;
}

}  // namespace nav2_theta_star_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_theta_star_planner::ThetaStarPlanner, nav2_core::GlobalPlanner)
