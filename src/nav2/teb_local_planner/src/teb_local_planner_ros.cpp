/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016,
 *  TU Dortmund - Institute of Control Theory and Systems Engineering.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the institute nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Christoph Rösmann
 *********************************************************************/

#include "teb_local_planner/teb_local_planner_ros.h"
#include "teb_local_planner/costmap_obstacle_tiling.hpp"
#include "teb_local_planner/obstacles.h"
#include "teb_local_planner/numerical_safety.hpp"
#include "goal_change_reset.hpp"

//#include <tf_conversions/tf_eigen.h>
#include <boost/algorithm/string.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <exception>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>
#include <stdexcept>
#include <utility>
#include <vector>

// pluginlib macros
#include <pluginlib/class_list_macros.hpp>

#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"

#include <nav2_core/exceptions.hpp>
#include <dwb_critics/line_iterator.hpp>
#include <nav2_costmap_2d/cost_values.hpp>
#include <nav2_costmap_2d/costmap_layer.hpp>
#include <nav2_costmap_2d/footprint.hpp>
#include <nav_2d_utils/tf_help.hpp>
#include <tf2/utils.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

using nav2_util::declare_parameter_if_not_declared;

namespace teb_local_planner
{
  
namespace
{

constexpr double kProjectionBacktrackWindow = 0.50;
constexpr double kProjectionLookaheadWindow = 0.50;
constexpr double kRouteProgressRenewalDistance = 0.02;
constexpr double kHeadingProgressRenewalAngle = 0.05;

struct RouteProjection
{
  double station{std::numeric_limits<double>::quiet_NaN()};
  double remaining{std::numeric_limits<double>::quiet_NaN()};
  double heading{std::numeric_limits<double>::quiet_NaN()};
  bool valid{false};
};

template<typename Callback>
class ScopeExit
{
public:
  explicit ScopeExit(Callback callback)
  : callback_(std::move(callback))
  {}

  ~ScopeExit()
  {
    callback_();
  }

  ScopeExit(const ScopeExit &) = delete;
  ScopeExit & operator=(const ScopeExit &) = delete;

private:
  Callback callback_;
};

template<typename Callback>
ScopeExit<Callback> makeScopeExit(Callback callback)
{
  return ScopeExit<Callback>(std::move(callback));
}

class ForwardOnlyOptimizationGuard
{
public:
  ForwardOnlyOptimizationGuard(TebConfig & config, const bool enabled)
  : config_(config),
    max_vel_x_backwards_(config.robot.max_vel_x_backwards),
    allow_init_with_backwards_motion_(config.trajectory.allow_init_with_backwards_motion),
    enabled_(enabled)
  {
    if (enabled_) {
      config_.robot.max_vel_x_backwards = 0.0;
      config_.trajectory.allow_init_with_backwards_motion = false;
    }
  }

  ~ForwardOnlyOptimizationGuard()
  {
    if (enabled_) {
      config_.robot.max_vel_x_backwards = max_vel_x_backwards_;
      config_.trajectory.allow_init_with_backwards_motion =
        allow_init_with_backwards_motion_;
    }
  }

  ForwardOnlyOptimizationGuard(const ForwardOnlyOptimizationGuard &) = delete;
  ForwardOnlyOptimizationGuard & operator=(const ForwardOnlyOptimizationGuard &) = delete;

private:
  TebConfig & config_;
  double max_vel_x_backwards_;
  bool allow_init_with_backwards_motion_;
  bool enabled_;
};

double absoluteYawError(double target_yaw, double current_yaw)
{
  return std::abs(std::atan2(
      std::sin(target_yaw - current_yaw), std::cos(target_yaw - current_yaw)));
}

RouteProjection projectOntoPath(
  const std::vector<geometry_msgs::msg::PoseStamped>& path,
  const geometry_msgs::msg::PoseStamped& robot_pose,
  double previous_station)
{
  RouteProjection result;
  if (path.empty()) {
    return result;
  }

  const auto& robot = robot_pose.pose.position;
  if (path.size() == 1) {
    const auto& goal = path.front().pose.position;
    result.remaining = std::hypot(robot.x - goal.x, robot.y - goal.y);
    result.station = -result.remaining;
    result.heading = tf2::getYaw(path.front().pose.orientation);
    result.valid = true;
    return result;
  }

  double total_length = 0.0;
  double best_distance_squared = std::numeric_limits<double>::infinity();
  double best_progress = 0.0;
  double best_heading = std::numeric_limits<double>::quiet_NaN();
  const bool constrain_station = std::isfinite(previous_station);
  const double min_station = constrain_station ?
    std::max(0.0, previous_station - kProjectionBacktrackWindow) : 0.0;
  const double max_station = constrain_station ?
    previous_station + kProjectionLookaheadWindow :
    std::numeric_limits<double>::infinity();
  for (std::size_t i = 1; i < path.size(); ++i) {
    const auto& start = path[i - 1].pose.position;
    const auto& end = path[i].pose.position;
    const double dx = end.x - start.x;
    const double dy = end.y - start.y;
    const double segment_length_squared = dx * dx + dy * dy;
    const double segment_length = std::sqrt(segment_length_squared);
    if (segment_length_squared <= 1e-12) {
      continue;
    }
    const double raw_projection = segment_length_squared > 0.0
      ? ((robot.x - start.x) * dx + (robot.y - start.y) * dy) / segment_length_squared
      : 0.0;
    const double projection = std::max(0.0, std::min(1.0, raw_projection));
    const double candidate_progress = total_length + projection * segment_length;
    const double offset_x = robot.x - (start.x + projection * dx);
    const double offset_y = robot.y - (start.y + projection * dy);
    const double distance_squared = offset_x * offset_x + offset_y * offset_y;
    if (candidate_progress >= min_station && candidate_progress <= max_station &&
      distance_squared < best_distance_squared)
    {
      best_distance_squared = distance_squared;
      best_progress = candidate_progress;
      best_heading = std::atan2(dy, dx);
    }
    total_length += segment_length;
  }
  if (!std::isfinite(best_distance_squared)) {
    return result;
  }

  result.station = best_progress;
  result.remaining = std::max(0.0, total_length - best_progress);
  result.heading = best_heading;
  result.valid = true;
  return result;
}

}  // namespace


TebLocalPlannerROS::TebLocalPlannerROS() 
    : costmap_ros_(nullptr), tf_(nullptr), cfg_(new TebConfig()), costmap_model_(nullptr), intra_proc_node_(nullptr),
                                           
                                           custom_via_points_active_(false), no_infeasible_plans_(0),
                                           last_preferred_rotdir_(RotType::none), initialized_(false)
{
}


TebLocalPlannerROS::~TebLocalPlannerROS()
{
}

void TebLocalPlannerROS::initialize(nav2_util::LifecycleNode::SharedPtr node)
{
  // check if the plugin is already initialized
  if(!initialized_)
  {	
    // declare parameters (ros2-dashing)
    intra_proc_node_.reset( 
            new rclcpp::Node("teb_local_planner", node->get_namespace(), 
              rclcpp::NodeOptions()));
    failure_diagnostic_pub_ = intra_proc_node_->create_publisher<std_msgs::msg::String>(
      "/navigation/teb_failure_diagnostic",
      rclcpp::QoS(1).reliable().transient_local());
    cfg_->declareParameters(node, name_);

    // get parameters of TebConfig via the nodehandle and override the default config
    cfg_->loadRosParamFromNodeHandle(node, name_);
    
    // reserve some memory for obstacles
    obstacles_.reserve(500);
        
    // create the planner instance
    if (cfg_->hcp.enable_homotopy_class_planning)
    {
      planner_ = PlannerInterfacePtr(new HomotopyClassPlanner(node, *cfg_.get(), &obstacles_, visualization_, &via_points_));
      RCLCPP_INFO(logger_, "Parallel planning in distinctive topologies enabled.");
    }
    else
    {
      planner_ = PlannerInterfacePtr(new TebOptimalPlanner(node, *cfg_.get(), &obstacles_, visualization_, &via_points_));
      RCLCPP_INFO(logger_, "Parallel planning in distinctive topologies disabled.");
    }
    
    // init other variables
    costmap_ = costmap_ros_->getCostmap(); // locking should be done in MoveBase.
    
    costmap_model_ = std::make_shared<dwb_critics::ObstacleFootprintCritic>();
    std::string costmap_model_name("costmap_model");
    costmap_model_->initialize(node, costmap_model_name, name_, costmap_ros_);

    cfg_->map_frame = costmap_ros_->getGlobalFrameID(); // TODO

   
    
    // Get footprint of the robot and minimum and maximum distance from the center of the robot to its footprint vertices.
    footprint_spec_ = costmap_ros_->getRobotFootprint();
    nav2_costmap_2d::calculateMinAndMaxDistances(
    footprint_spec_, robot_inscribed_radius_, robot_circumscribed_radius);

    // Add callback for dynamic parameters
    dyn_params_handler = node->add_on_set_parameters_callback(
      std::bind(&TebConfig::dynamicParametersCallback, std::ref(cfg_), std::placeholders::_1));

    // validate optimization footprint and costmap footprint
    validateFootprints(cfg_->robot_model->getInscribedRadius(), robot_inscribed_radius_, cfg_->obstacles.min_obstacle_dist);
        
    

    // setup callback for custom via-points
    via_points_sub_ = node->create_subscription<nav_msgs::msg::Path>(
                "via_points", 
                rclcpp::SystemDefaultsQoS(),
                std::bind(&TebLocalPlannerROS::customViaPointsCB, this, std::placeholders::_1));

    declare_parameter_if_not_declared(
      node, name_ + ".enable_reverse_suppression", rclcpp::ParameterValue(false));
    enable_reverse_suppression_ =
      node->get_parameter(name_ + ".enable_reverse_suppression").as_bool();
    if (enable_reverse_suppression_) {
      declare_parameter_if_not_declared(
        node, name_ + ".navigation_goal_status_topic",
        rclcpp::ParameterValue("/navigate_to_pose/_action/status"));
      const std::string navigation_goal_status_topic = node->get_parameter(
        name_ + ".navigation_goal_status_topic").as_string();
      if (navigation_goal_status_topic.empty()) {
        throw std::invalid_argument("navigation_goal_status_topic must not be empty");
      }
      navigation_goal_status_sub_ =
        node->create_subscription<action_msgs::msg::GoalStatusArray>(
        navigation_goal_status_topic, rclcpp::QoS(10).reliable(),
        [this](action_msgs::msg::GoalStatusArray::SharedPtr message) {
          const action_msgs::msg::GoalStatus * newest_active_goal = nullptr;
          for (const auto & status : message->status_list) {
            if (status.status != action_msgs::msg::GoalStatus::STATUS_ACCEPTED &&
              status.status != action_msgs::msg::GoalStatus::STATUS_EXECUTING)
            {
              continue;
            }
            if (newest_active_goal == nullptr ||
              status.goal_info.stamp.sec > newest_active_goal->goal_info.stamp.sec ||
              (status.goal_info.stamp.sec == newest_active_goal->goal_info.stamp.sec &&
              status.goal_info.stamp.nanosec >= newest_active_goal->goal_info.stamp.nanosec))
            {
              newest_active_goal = &status;
            }
          }
          if (newest_active_goal == nullptr) {
            return;
          }
          const auto & goal_id = newest_active_goal->goal_info.goal_id.uuid;
          std::lock_guard<std::mutex> lock(navigation_goal_identity_mutex_);
          pending_navigation_goal_identity_.assign(goal_id.begin(), goal_id.end());
        });

      declare_parameter_if_not_declared(
        node, name_ + ".reverse_blocked_topic",
        rclcpp::ParameterValue("/chassis/reverse_blocked"));
      const std::string reverse_blocked_topic =
        node->get_parameter(name_ + ".reverse_blocked_topic").as_string();
      if (reverse_blocked_topic.empty()) {
        throw std::invalid_argument("reverse_blocked_topic must not be empty");
      }
      reverse_blocked_sub_ = node->create_subscription<std_msgs::msg::Bool>(
        reverse_blocked_topic, rclcpp::QoS(1).reliable().transient_local(),
        [this](std_msgs::msg::Bool::SharedPtr message) {
          reverse_blocked_state_.store(message->data);
          if (message->data) {
            pending_reverse_blocked_.store(true);
          }
        });
      declare_parameter_if_not_declared(
        node, name_ + ".reverse_suppression_attempt_topic",
        rclcpp::ParameterValue("/navigation/follow_path_attempt"));
      const std::string attempt_topic = node->get_parameter(
        name_ + ".reverse_suppression_attempt_topic").as_string();
      if (attempt_topic.empty()) {
        throw std::invalid_argument("reverse_suppression_attempt_topic must not be empty");
      }
      follow_path_attempt_ack_pub_ = intra_proc_node_->create_publisher<std_msgs::msg::UInt64>(
        attempt_topic + "_ack", rclcpp::QoS(10).reliable());
      declare_parameter_if_not_declared(
        node, name_ + ".follow_path_progress_topic",
        rclcpp::ParameterValue("/navigation/follow_path_progress"));
      const std::string progress_topic = node->get_parameter(
        name_ + ".follow_path_progress_topic").as_string();
      if (progress_topic.empty()) {
        throw std::invalid_argument("follow_path_progress_topic must not be empty");
      }
      follow_path_progress_pub_ = intra_proc_node_->create_publisher<std_msgs::msg::UInt64>(
        progress_topic, rclcpp::QoS(10).reliable());
      follow_path_attempt_sub_ = node->create_subscription<std_msgs::msg::UInt64>(
        attempt_topic, rclcpp::QoS(10).reliable(),
        [this](std_msgs::msg::UInt64::SharedPtr message) {
          follow_path_attempt_id_.store(message->data);
          follow_path_attempt_ack_pub_->publish(*message);
        });
      declare_parameter_if_not_declared(
        node, name_ + ".reverse_suppression_timeout",
        rclcpp::ParameterValue(reverse_suppression_timeout_));
      reverse_suppression_timeout_ =
        node->get_parameter(name_ + ".reverse_suppression_timeout").as_double();
      if (!std::isfinite(reverse_suppression_timeout_) || reverse_suppression_timeout_ <= 0.0) {
        throw std::invalid_argument("reverse_suppression_timeout must be finite and positive");
      }
    }
    
    // initialize failure detector
    //rclcpp::Node::SharedPtr nh_move_base("~");
    double controller_frequency = 5;
    node->get_parameter("controller_frequency", controller_frequency);
    if (!std::isfinite(controller_frequency) || controller_frequency <= 0.0) {
      throw std::invalid_argument("controller_frequency must be finite and positive");
    }
    controller_frequency_hz_ = controller_frequency;
    failure_detector_.setBufferLength(std::round(
        cfg_->recovery.oscillation_filter_duration * controller_frequency_hz_));
    
    // set initialized flag
    initialized_ = true;

    // This should be called since to prevent different time sources exception
    time_last_infeasible_plan_ = clock_->now();
    time_last_oscillation_ = clock_->now();
    RCLCPP_DEBUG(logger_, "teb_local_planner plugin initialized.");
  }
  else
  {
    RCLCPP_INFO(logger_, "teb_local_planner has already been initialized, doing nothing.");
  }
}

std::string TebLocalPlannerROS::describeFeasibilityFailure(
  const PlannerInterface::FeasibilityFailure & failure,
  std::string * failure_key)
{
  std::ostringstream output;
  output << std::fixed << std::setprecision(3)
         << "category=teb_trajectory_infeasible"
         << " pose_index=" << failure.pose_index
         << " interpolated=" << (failure.interpolated ? "true" : "false")
         << " pose_x=" << failure.pose.x
         << " pose_y=" << failure.pose.y
         << " heading_rad=" << failure.pose.theta;

  if (failure.cause == PlannerInterface::FeasibilityFailure::Cause::NONE || !costmap_) {
    output << " cause=no_feasibility_sample";
    if (failure_key) {
      *failure_key = "no_feasibility_sample:" + std::to_string(failure.pose_index);
    }
    return output.str();
  }

  std::unique_lock<std::recursive_mutex> lock(*costmap_->getMutex());
  const auto oriented_footprint = dwb_critics::getOrientedFootprint(
    failure.pose, footprint_spec_);
  struct FailureSample
  {
    bool valid{false};
    unsigned int x{0};
    unsigned int y{0};
    unsigned char cost{0};
    std::size_t edge{0};
    std::size_t sample{0};
  } contact;
  enum class Cause {UNCLASSIFIED, LETHAL, UNKNOWN, OUT_OF_MAP};
  Cause cause = Cause::UNCLASSIFIED;
  std::size_t failed_edge = 0;
  double outside_x = 0.0;
  double outside_y = 0.0;

  // Mirror ObstacleFootprintCritic::scorePose in Humble while the same
  // costmap mutex is held: polygon edges in order, LineIterator samples in
  // order, and only lethal (254) or unknown (255) cells reject a sample.
  for (std::size_t edge = 0; edge < oriented_footprint.size(); ++edge) {
    const auto & start = oriented_footprint[edge];
    const auto & end = oriented_footprint[(edge + 1) % oriented_footprint.size()];
    unsigned int start_x = 0, start_y = 0, end_x = 0, end_y = 0;
    if (!costmap_->worldToMap(start.x, start.y, start_x, start_y)) {
      cause = Cause::OUT_OF_MAP;
      failed_edge = edge;
      outside_x = start.x;
      outside_y = start.y;
      break;
    }
    if (!costmap_->worldToMap(end.x, end.y, end_x, end_y)) {
      cause = Cause::OUT_OF_MAP;
      failed_edge = edge;
      outside_x = end.x;
      outside_y = end.y;
      break;
    }
    dwb_critics::LineIterator line(
      static_cast<int>(start_x), static_cast<int>(start_y),
      static_cast<int>(end_x), static_cast<int>(end_y));
    std::size_t sample_index = 0;
    for (; line.isValid(); line.advance(), ++sample_index) {
      const auto x = static_cast<unsigned int>(line.getX());
      const auto y = static_cast<unsigned int>(line.getY());
      const auto cost = costmap_->getCost(x, y);
      if (cost == nav2_costmap_2d::LETHAL_OBSTACLE ||
        cost == nav2_costmap_2d::NO_INFORMATION)
      {
        contact = {true, x, y, cost, edge, sample_index};
        cause = cost == nav2_costmap_2d::LETHAL_OBSTACLE ? Cause::LETHAL : Cause::UNKNOWN;
        failed_edge = edge;
        break;
      }
    }
    if (contact.valid) break;
  }

  unsigned int robot_cell_x = 0, robot_cell_y = 0;
  const bool robot_center_in_map = costmap_->worldToMap(
    failure.pose.x, failure.pose.y, robot_cell_x, robot_cell_y);
  const auto heading_bucket = std::llround(failure.pose.theta * 20.0);
  const char * cause_name = "unclassified_rejection";
  if (cause == Cause::LETHAL) cause_name = "lethal_obstacle";
  if (cause == Cause::UNKNOWN) cause_name = "unknown_space";
  if (cause == Cause::OUT_OF_MAP) cause_name = "footprint_outside_map";
  output << " cause=" << cause_name << " footprint_edge=" << failed_edge;
  if (cause == Cause::OUT_OF_MAP) {
    output << " touch_world=(" << outside_x << ',' << outside_y << ')'
           << " collision_cell=false outside_map=true";
    if (failure_key) {
      std::ostringstream key;
      key << failure.pose_index << ':' << failure.interpolated << ':'
          << robot_center_in_map << ':' << robot_cell_x << ':' << robot_cell_y << ':'
          << heading_bucket << ':' << cause_name << ':' << outside_x << ':' << outside_y;
      *failure_key = key.str();
    }
    return output.str();
  }
  if (!contact.valid) {
    if (failure_key) {
      std::ostringstream key;
      key << failure.pose_index << ':' << failure.interpolated << ':'
          << robot_center_in_map << ':' << robot_cell_x << ':' << robot_cell_y << ':'
          << heading_bucket << ':' << cause_name;
      *failure_key = key.str();
    }
    return output.str();
  }

  double cell_world_x = 0.0, cell_world_y = 0.0;
  costmap_->mapToWorld(contact.x, contact.y, cell_world_x, cell_world_y);
  output << " touch_cell=(" << contact.x << ',' << contact.y << ')'
         << " master_cost=" << static_cast<unsigned int>(contact.cost)
         << " edge_sample=" << contact.sample
         << " touch_world=(" << cell_world_x << ',' << cell_world_y << ')'
         << " collision_cell=true outside_map=false layer_costs=[";
  bool first_layer = true;
  std::ostringstream layer_signature;
  const auto * plugins = costmap_ros_->getLayeredCostmap()->getPlugins();
  for (const auto & plugin : *plugins) {
    const auto layer = std::dynamic_pointer_cast<nav2_costmap_2d::CostmapLayer>(plugin);
    if (!layer) {
      continue;
    }
    unsigned int layer_x = 0, layer_y = 0;
    const unsigned char layer_cost = layer->worldToMap(cell_world_x, cell_world_y, layer_x, layer_y) ?
      layer->getCost(layer_x, layer_y) : nav2_costmap_2d::NO_INFORMATION;
    if (!first_layer) output << ',';
    output << layer->getName() << ':' << static_cast<unsigned int>(layer_cost);
    layer_signature << layer->getName() << ':' << static_cast<unsigned int>(layer_cost) << ',';
    first_layer = false;
  }
  output << ']';
  if (failure_key) {
    std::ostringstream key;
    key << failure.pose_index << ':' << failure.interpolated << ':'
        << robot_center_in_map << ':' << robot_cell_x << ':' << robot_cell_y << ':'
        << heading_bucket << ':' << cause_name << ':' << contact.edge << ':'
        << contact.sample << ':' << contact.x << ':' << contact.y << ':'
        << static_cast<unsigned int>(contact.cost) << ':' << layer_signature.str();
    *failure_key = key.str();
  }
  return output.str();
}

void TebLocalPlannerROS::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) {
  nh_ = parent;

  auto node = nh_.lock();
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  costmap_ros_ = costmap_ros;
  tf_ = tf;
  name_ = name;

  initialize(node);
  visualization_ = std::make_shared<TebVisualization>(node, *cfg_);
  visualization_->on_configure();
  planner_->setVisualization(visualization_);
  
  return;
}

void TebLocalPlannerROS::setPlan(const nav_msgs::msg::Path & orig_global_plan)
{
  // check if plugin is initialized
  if(!initialized_)
  {
    RCLCPP_ERROR(logger_, "teb_local_planner has not been initialized, please call initialize() before using this planner");
    return;
  }

  if (enable_reverse_suppression_ && applyNavigationGoalIdentityUpdate()) {
    last_feasibility_failure_key_.clear();
    route_progress_reference_path_.clear();
    route_progress_last_projected_station_ = std::numeric_limits<double>::quiet_NaN();
    path_change_tracker_.reset();
    route_progress_change_tracker_.reset();
    resetReverseSuppressionRoute();
    pending_reverse_blocked_.store(false);
    detail::resetForGoalChange(
      *planner_, failure_detector_, last_preferred_rotdir_, no_infeasible_plans_, last_cmd_);
    const auto reset_time = clock_->now();
    time_last_infeasible_plan_ = reset_time -
      rclcpp::Duration::from_seconds(cfg_->recovery.shrink_horizon_min_duration);
    time_last_oscillation_ = reset_time -
      rclcpp::Duration::from_seconds(cfg_->recovery.oscillation_recovery_min_duration);
    RCLCPP_INFO(
      logger_,
      "Navigation action goal identity changed; reverse suppression and goal recovery state reset.");
  }

  if (orig_global_plan.poses.empty()) {
    last_feasibility_failure_key_.clear();
    global_plan_.clear();
    route_progress_reference_path_.clear();
    route_progress_last_projected_station_ = std::numeric_limits<double>::quiet_NaN();
    path_change_tracker_.reset();
    route_progress_change_tracker_.reset();
    resetReverseSuppressionRoute();
    pending_reverse_blocked_.store(false);
    detail::resetForGoalChange(
      *planner_, failure_detector_, last_preferred_rotdir_, no_infeasible_plans_, last_cmd_);
    return;
  }

  const bool planner_route_changed = path_change_tracker_.update(orig_global_plan);
  const bool progress_route_changed = enable_reverse_suppression_ &&
    route_progress_change_tracker_.update(orig_global_plan);
  if (planner_route_changed || progress_route_changed) {
    if (progress_route_changed) {
      route_progress_reference_path_.clear();
      route_progress_last_projected_station_ = std::numeric_limits<double>::quiet_NaN();
      follow_path_progress_best_path_station_ = std::numeric_limits<double>::quiet_NaN();
      follow_path_progress_target_yaw_ = std::numeric_limits<double>::quiet_NaN();
      follow_path_progress_best_yaw_error_ = std::numeric_limits<double>::quiet_NaN();
      reverse_suppression_rebase_progress_ = route_reverse_suppressed_;
    }
    if (enable_reverse_suppression_) {
      // Reinitialize the local trajectory for a changed route while retaining
      // the rejected-reverse latch and its current no-progress deadline.
      planner_->clearPlanner();
      last_preferred_rotdir_ = RotType::none;
      planner_->setPreferredTurningDir(last_preferred_rotdir_);
      RCLCPP_DEBUG(
        logger_, "Global route changed; navigation-goal reverse suppression is retained.");
    } else {
      // Preserve the established reset behavior for planners without route-level
      // reverse suppression (for example, the odom-frame navigation profile).
      detail::resetForGoalChange(
        *planner_, failure_detector_, last_preferred_rotdir_, no_infeasible_plans_, last_cmd_);
      const auto reset_time = clock_->now();
      time_last_infeasible_plan_ = reset_time -
        rclcpp::Duration::from_seconds(cfg_->recovery.shrink_horizon_min_duration);
      time_last_oscillation_ = reset_time -
        rclcpp::Duration::from_seconds(cfg_->recovery.oscillation_recovery_min_duration);
    }
  }

  if (enable_reverse_suppression_ && route_progress_reference_path_.empty()) {
    // This path remains the measurement reference across ordinary replans, whose
    // polyline lengths can differ even when the robot has not moved.
    route_progress_reference_path_.reserve(orig_global_plan.poses.size());
    for (const auto & pose : orig_global_plan.poses) {
      auto stamped_pose = pose;
      stamped_pose.header = orig_global_plan.header;
      route_progress_reference_path_.push_back(std::move(stamped_pose));
    }
  }

  if (enable_reverse_suppression_ && reverse_blocked_state_.load()) {
    pending_reverse_blocked_.store(true);
  }

  // store the global plan
  global_plan_.clear();
  global_plan_.reserve(orig_global_plan.poses.size());
  for(const auto &in_pose :orig_global_plan.poses) {
    geometry_msgs::msg::PoseStamped out_pose;
    out_pose.pose = in_pose.pose;
    out_pose.header = orig_global_plan.header;
    global_plan_.push_back(out_pose);
  }

  // we do not clear the local planner here, since setPlan is called frequently whenever the global planner updates the plan.
  // the local planner checks whether it is required to reinitialize the trajectory or not within each velocity computation step.  
            
  return;
}


geometry_msgs::msg::TwistStamped TebLocalPlannerROS::computeVelocityCommands(const geometry_msgs::msg::PoseStamped &pose,
  const geometry_msgs::msg::Twist &velocity, nav2_core::GoalChecker *goal_checker)
{
  // check if plugin initialized
  if(!initialized_)
  {
    throw std::runtime_error(
      std::string("teb_local_planner has not been initialized, please call initialize() before using this planner")
    );
  }
  const auto timing_started = std::chrono::steady_clock::now();
  const auto exception_count = std::uncaught_exceptions();
  const auto control_cycle_guard = makeScopeExit([this, timing_started, exception_count]() {
      const double elapsed_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - timing_started).count();
      const double budget_ms = 1000.0 / controller_frequency_hz_;
      ++control_cycle_count_;
      if (elapsed_ms > budget_ms) {
        ++control_cycle_overruns_window_;
        control_cycle_overrun_max_ms_ = std::max(control_cycle_overrun_max_ms_, elapsed_ms);
      }
      if (control_cycle_count_ % 10 == 0) {
        const auto outcome = std::uncaught_exceptions() > exception_count ? "exception" : "return";
        if (control_cycle_overruns_window_ > 0) {
          RCLCPP_WARN(
            logger_, "TEB control cycle budget: samples=10 over_budget=%llu period=%.3f ms "
            "last=%.3f ms max=%.3f ms outcome=%s",
            static_cast<unsigned long long>(control_cycle_overruns_window_), budget_ms, elapsed_ms,
            control_cycle_overrun_max_ms_, outcome);
        } else {
          RCLCPP_INFO(
            logger_, "TEB control cycle budget: samples=10 over_budget=%llu period=%.3f ms "
            "last=%.3f ms max=%.3f ms outcome=%s",
            static_cast<unsigned long long>(control_cycle_overruns_window_), budget_ms, elapsed_ms,
            control_cycle_overrun_max_ms_, outcome);
        }
        control_cycle_overruns_window_ = 0;
        control_cycle_overrun_max_ms_ = 0.0;
      }
    });
  geometry_msgs::msg::TwistStamped cmd_vel;
  
  cmd_vel.header.stamp = clock_->now();
  cmd_vel.header.frame_id = costmap_ros_->getBaseFrameID();
  cmd_vel.twist.linear.x = 0;
  cmd_vel.twist.linear.y = 0;
  cmd_vel.twist.angular.z = 0;

  // Update for the current goal checker's state
  geometry_msgs::msg::Pose pose_tolerance;
  geometry_msgs::msg::Twist vel_tolerance;
  if (!goal_checker->getTolerances(pose_tolerance, vel_tolerance)) {
    RCLCPP_WARN(logger_, "Unable to retrieve goal checker's tolerances!");
  } else {
    cfg_->goal_tolerance.xy_goal_tolerance = pose_tolerance.position.x;
  }
  
  // Get robot pose
  robot_pose_ = PoseSE2(pose.pose);
  geometry_msgs::msg::PoseStamped robot_pose;
  robot_pose.header = pose.header;
  robot_pose_.toPoseMsg(robot_pose.pose);
  
  // Get robot velocity
  robot_vel_ = velocity;
  
  // prune global plan to cut off parts of the past (spatially before the robot)
  pruneGlobalPlan(robot_pose, global_plan_, cfg_->trajectory.global_plan_prune_distance);

  // Transform global plan to the frame of interest (w.r.t. the local costmap)
  std::vector<geometry_msgs::msg::PoseStamped> transformed_plan;
  int goal_idx;
  geometry_msgs::msg::TransformStamped tf_plan_to_global;
  if (!transformGlobalPlan(global_plan_, robot_pose, *costmap_, cfg_->map_frame, cfg_->trajectory.max_global_plan_lookahead_dist,
                           transformed_plan, &goal_idx, &tf_plan_to_global))
  {
    throw std::runtime_error(
      std::string("Could not transform the global plan to the frame of the controller")
    );
  }

  RouteProjection route_projection;
  double remaining_path_distance = std::numeric_limits<double>::quiet_NaN();
  double path_station = std::numeric_limits<double>::quiet_NaN();
  double path_heading = std::numeric_limits<double>::quiet_NaN();
  double robot_yaw = std::numeric_limits<double>::quiet_NaN();
  if (enable_reverse_suppression_ && !route_progress_reference_path_.empty()) {
    try {
      const auto robot_in_plan = tf_->transform(
        robot_pose, route_progress_reference_path_.front().header.frame_id);
      robot_yaw = tf2::getYaw(robot_in_plan.pose.orientation);
      route_projection = projectOntoPath(
        route_progress_reference_path_, robot_in_plan, route_progress_last_projected_station_);
      if (route_projection.valid) {
        route_progress_last_projected_station_ = route_projection.station;
        path_station = route_projection.station;
        remaining_path_distance = route_projection.remaining;
        path_heading = route_projection.heading;
        if (route_projection.remaining <= cfg_->goal_tolerance.xy_goal_tolerance + 0.05) {
          path_heading = tf2::getYaw(route_progress_reference_path_.back().pose.orientation);
        }
      }
    } catch (const tf2::TransformException& error) {
      RCLCPP_DEBUG(logger_, "Cannot measure route progress for reverse suppression: %s", error.what());
    }
    applyFollowPathAttemptUpdate(path_station, path_heading, robot_yaw);
    rebaseReverseSuppressionProgress(path_station, path_heading, robot_yaw);
    publishMeasuredFollowPathProgress(path_station, path_heading, robot_yaw);
    enforceReverseSuppressionDeadline(path_station, path_heading, robot_yaw);
  }

  // update via-points container
  if (!custom_via_points_active_)
    updateViaPointsContainer(transformed_plan, cfg_->trajectory.global_plan_viapoint_sep);

  // check if we should enter any backup mode and apply settings
  configureBackupModes(
    transformed_plan, goal_idx, remaining_path_distance,
    path_station, path_heading, robot_yaw);
    
  // Return false if the transformed global plan is empty
  if (transformed_plan.empty())
  {
    throw std::runtime_error(
      std::string("Transformed plan is empty. Cannot determine a local plan.")
    );
  }
              
  // Get current goal point (last point of the transformed plan)
  const geometry_msgs::msg::PoseStamped &goal_point = transformed_plan.back();
  robot_goal_.x() = goal_point.pose.position.x;
  robot_goal_.y() = goal_point.pose.position.y;
  if (cfg_->trajectory.global_plan_overwrite_orientation)
  {
    robot_goal_.theta() = estimateLocalGoalOrientation(global_plan_, goal_point, goal_idx, tf_plan_to_global);
    // overwrite/update goal orientation of the transformed plan with the actual goal (enable using the plan as initialization)
    //transformed_plan.back().pose.orientation = tf::createQuaternionMsgFromYaw(robot_goal_.theta());
    tf2::Quaternion q;
    q.setRPY(0, 0, robot_goal_.theta());
    transformed_plan.back().pose.orientation = tf2::toMsg(q);
  }  
  else
  {
    robot_goal_.theta() = tf2::getYaw(goal_point.pose.orientation);
  }

  // overwrite/update start of the transformed plan with the actual robot position (allows using the plan as initial trajectory)
  if (transformed_plan.size()==1) // plan only contains the goal
  {
    transformed_plan.insert(transformed_plan.begin(), geometry_msgs::msg::PoseStamped()); // insert start (not yet initialized)
  }
  //tf::poseTFToMsg(robot_pose, transformed_plan.front().pose); // update start;
  transformed_plan.front().pose = robot_pose.pose;

  const auto preparation_finished = std::chrono::steady_clock::now();
  // clear currently existing obstacles
  obstacles_.clear();

  // Update obstacle container with costmap information or polygons provided by a costmap_converter plugin
  double costmap_lock_wait_ms = 0.0;
  double costmap_scan_ms = 0.0;
  double obstacle_tiling_ms = 0.0;
  std::size_t lethal_cells = 0;
  std::size_t obstacle_tiles = 0;
  const auto obstacle_extraction_started = std::chrono::steady_clock::now();
  updateObstacleContainerWithCostmap(
    costmap_lock_wait_ms, costmap_scan_ms, obstacle_tiling_ms,
    lethal_cells, obstacle_tiles);
  const auto obstacle_extraction_finished = std::chrono::steady_clock::now();
  
    
  // Do not allow config changes during the following optimization step
  std::lock_guard<std::mutex> cfg_lock(cfg_->configMutex());
    
  // Now perform the actual planning
//   bool success = planner_->plan(robot_pose_, robot_goal_, robot_vel_, cfg_->goal_tolerance.free_goal_vel); // straight line init
  bool success = false;
  ForwardOnlyOptimizationGuard forward_only_guard(*cfg_, route_reverse_suppressed_);
  const auto optimization_started = std::chrono::steady_clock::now();
  try {
    success = planner_->plan(transformed_plan, &robot_vel_, cfg_->goal_tolerance.free_goal_vel);
  } catch (const std::exception & error) {
    planner_->clearPlanner();
    ++no_infeasible_plans_;
    time_last_infeasible_plan_ = clock_->now();
    last_cmd_ = cmd_vel.twist;
    RCLCPP_ERROR(logger_, "TEB planning exception; planner reset: %s", error.what());
    throw std::runtime_error(
      std::string("teb_local_planner planning exception; planner reset: ") + error.what());
  }
  const auto optimization_finished = std::chrono::steady_clock::now();
  if (!success)
  {
    planner_->clearPlanner(); // force reinitialization for next time
    
    ++no_infeasible_plans_; // increase number of infeasible solutions in a row
    time_last_infeasible_plan_ = clock_->now();
    last_cmd_ = cmd_vel.twist;
    
    throw std::runtime_error(
      std::string("teb_local_planner was not able to obtain a local plan for the current setting.")
    );
  }

  // Check for divergence
  if (planner_->hasDiverged())
  {
    cmd_vel.twist.linear.x = cmd_vel.twist.linear.y = cmd_vel.twist.angular.z = 0;

    // Reset everything to start again with the initialization of new trajectories.
    planner_->clearPlanner();
    RCLCPP_WARN_THROTTLE(logger_, *(clock_), 1, "TebLocalPlannerROS: the trajectory has diverged. Resetting planner...");

    ++no_infeasible_plans_; // increase number of infeasible solutions in a row
    time_last_infeasible_plan_ = clock_->now();
    last_cmd_ = cmd_vel.twist;
    throw std::runtime_error(
      std::string("TebLocalPlannerROS: velocity command invalid (hasDiverged). Resetting planner...")
    );
  }
         
  // Check feasibility (but within the first few states only)
  if(cfg_->robot.is_footprint_dynamic)
  {
    // Update footprint of the robot and minimum and maximum distance from the center of the robot to its footprint vertices.
    std::vector<geometry_msgs::msg::Point> updated_footprint_spec_ = costmap_ros_->getRobotFootprint();
    if (updated_footprint_spec_ != footprint_spec_) {
      updated_footprint_spec_ = footprint_spec_;
       nav2_costmap_2d::calculateMinAndMaxDistances(
    updated_footprint_spec_, robot_inscribed_radius_, robot_circumscribed_radius);
    }
  }

  const auto collision_check_started = std::chrono::steady_clock::now();
  std::unique_lock<std::recursive_mutex> costmap_lock(*costmap_->getMutex());
  PlannerInterface::FeasibilityFailure feasibility_failure;
  bool feasible = planner_->isTrajectoryFeasible(
    costmap_model_.get(), footprint_spec_, robot_inscribed_radius_, robot_circumscribed_radius,
    cfg_->trajectory.feasibility_check_no_poses,
    cfg_->trajectory.feasibility_check_lookahead_distance, &feasibility_failure);
  const auto collision_check_finished = std::chrono::steady_clock::now();
  if (!feasible)
  {
    std::string diagnostic_key;
    const std::string diagnostic = describeFeasibilityFailure(
      feasibility_failure, &diagnostic_key);
    if (diagnostic_key != last_feasibility_failure_key_) {
      RCLCPP_WARN(logger_, "%s", diagnostic.c_str());
      if (failure_diagnostic_pub_) {
        std_msgs::msg::String message;
        message.data = diagnostic;
        failure_diagnostic_pub_->publish(message);
      }
      last_feasibility_failure_key_ = diagnostic_key;
    }
    costmap_lock.unlock();
    cmd_vel.twist.linear.x = cmd_vel.twist.linear.y = cmd_vel.twist.angular.z = 0;
   
    // now we reset everything to start again with the initialization of new trajectories.
    planner_->clearPlanner();

    ++no_infeasible_plans_; // increase number of infeasible solutions in a row
    time_last_infeasible_plan_ = clock_->now();
    last_cmd_ = cmd_vel.twist;
    
    throw std::runtime_error(
      std::string("TebLocalPlannerROS: trajectory is not feasible. Resetting planner...")
    );
  }

  costmap_lock.unlock();
  last_feasibility_failure_key_.clear();

  // Get the velocity command for this sampling interval
  const auto velocity_command_started = std::chrono::steady_clock::now();
  if (!planner_->getVelocityCommand(cmd_vel.twist.linear.x, cmd_vel.twist.linear.y, cmd_vel.twist.angular.z, cfg_->trajectory.control_look_ahead_poses))
  {
    cmd_vel.twist.linear.x = cmd_vel.twist.linear.y = cmd_vel.twist.angular.z = 0;
    planner_->clearPlanner();
    ++no_infeasible_plans_; // increase number of infeasible solutions in a row
    time_last_infeasible_plan_ = clock_->now();
    last_cmd_ = cmd_vel.twist;
    
    throw std::runtime_error(
      std::string("TebLocalPlannerROS: velocity command invalid. Resetting planner...")
    );
  }

  if (!detail::finite_velocity_command(
      cmd_vel.twist.linear.x, cmd_vel.twist.linear.y, cmd_vel.twist.angular.z))
  {
    cmd_vel.twist.linear.x = cmd_vel.twist.linear.y = cmd_vel.twist.angular.z = 0;
    planner_->clearPlanner();
    ++no_infeasible_plans_;
    time_last_infeasible_plan_ = clock_->now();
    last_cmd_ = cmd_vel.twist;
    RCLCPP_ERROR(logger_, "TebLocalPlannerROS: non-finite velocity command. Resetting planner...");
    throw std::runtime_error(
      std::string("TebLocalPlannerROS: velocity command is not finite. Resetting planner...")
    );
  }
  const auto velocity_command_finished = std::chrono::steady_clock::now();

  if (route_reverse_suppressed_ && cmd_vel.twist.linear.x < 0.0) {
    const double rejected_linear_x = cmd_vel.twist.linear.x;
    // Drop only the translation. Zeroing the whole twist froze the robot: with no
    // translation and no rotation the progress checker expired and FollowPath
    // aborted, so a suppressed reverse could never be escaped. Keeping the
    // angular component lets the robot turn in place and leave forwards.
    cmd_vel.twist.linear.x = 0.0;
    cmd_vel.twist.linear.y = 0.0;
    // This path returns before the saturation below, so clamp the kept rotation here.
    saturateVelocity(
      cmd_vel.twist.linear.x, cmd_vel.twist.linear.y, cmd_vel.twist.angular.z,
      cfg_->robot.max_vel_x, cfg_->robot.max_vel_y, cfg_->robot.max_vel_theta,
      cfg_->robot.max_vel_x_backwards);
    planner_->clearPlanner();
    ++no_infeasible_plans_;
    time_last_infeasible_plan_ = clock_->now();
    last_cmd_ = cmd_vel.twist;
    RCLCPP_WARN_THROTTLE(
      logger_, *(clock_), 1000,
      "Rejecting reverse component of TEB candidate (vx=%.6f) while reverse is suppressed; "
      "keeping wz=%.6f so the robot can turn in place instead of stalling.",
      rejected_linear_x, cmd_vel.twist.angular.z);
    return cmd_vel;
  }
  
  // Saturate velocity, if the optimization results violates the constraints (could be possible due to soft constraints).
  saturateVelocity(cmd_vel.twist.linear.x, cmd_vel.twist.linear.y, cmd_vel.twist.angular.z, cfg_->robot.max_vel_x, cfg_->robot.max_vel_y,
                   cfg_->robot.max_vel_theta, cfg_->robot.max_vel_x_backwards);

  // convert rot-vel to steering angle if desired (carlike robot).
  // The min_turning_radius is allowed to be slighly smaller since it is a soft-constraint
  // and opposed to the other constraints not affected by penalty_epsilon. The user might add a safety margin to the parameter itself.
  if (cfg_->robot.cmd_angle_instead_rotvel)
  {
    cmd_vel.twist.angular.z = convertTransRotVelToSteeringAngle(cmd_vel.twist.linear.x, cmd_vel.twist.angular.z, cfg_->robot.wheelbase, 0.95*cfg_->robot.min_turning_radius);
    if (!std::isfinite(cmd_vel.twist.angular.z))
    {
      cmd_vel.twist.linear.x = cmd_vel.twist.linear.y = cmd_vel.twist.angular.z = 0;
      last_cmd_ = cmd_vel.twist;
      planner_->clearPlanner();

      ++no_infeasible_plans_; // increase number of infeasible solutions in a row
      time_last_infeasible_plan_ = clock_->now();
      
      throw std::runtime_error(
        std::string("TebLocalPlannerROS: Resulting steering angle is not finite. Resetting planner...")
      );
    }
  }

  // a feasible solution should be found, reset counter
  no_infeasible_plans_ = 0;
  
  // store last command (for recovery analysis etc.)
  last_cmd_ = cmd_vel.twist;
  
  // Now visualize everything    
  const auto visualization_started = std::chrono::steady_clock::now();
  planner_->visualize();
  visualization_->publishObstacles(obstacles_);
  visualization_->publishViaPoints(via_points_);
  visualization_->publishGlobalPlan(global_plan_);
  const auto visualization_finished = std::chrono::steady_clock::now();
  if (++control_timing_count_ % 10 == 0) {
    const auto milliseconds = [](const auto duration) {
        return std::chrono::duration<double, std::milli>(duration).count();
      };
    RCLCPP_INFO(
      logger_,
      "TEB timing: total=%.3f ms preparation=%.3f ms obstacle_extract=%.3f ms "
      "costmap_lock_wait=%.3f ms costmap_scan=%.3f ms obstacle_tiling=%.3f ms "
      "optimization=%.3f ms feasibility=%.3f ms velocity=%.3f ms visualization=%.3f ms "
      "lethal_cells=%zu tiles=%zu obstacles=%zu",
      milliseconds(visualization_finished - timing_started),
      milliseconds(preparation_finished - timing_started),
      milliseconds(obstacle_extraction_finished - obstacle_extraction_started),
      costmap_lock_wait_ms, costmap_scan_ms, obstacle_tiling_ms,
      milliseconds(optimization_finished - optimization_started),
      milliseconds(collision_check_finished - collision_check_started),
      milliseconds(velocity_command_finished - velocity_command_started),
      milliseconds(visualization_finished - visualization_started),
      lethal_cells, obstacle_tiles, obstacles_.size());
  }
  return cmd_vel;
}

void TebLocalPlannerROS::updateObstacleContainerWithCostmap(
  double & lock_wait_ms, double & scan_ms, double & tiling_ms,
  std::size_t & lethal_cells, std::size_t & obstacle_tiles)
{  
  // Add costmap obstacles if desired
  if (cfg_->obstacles.include_costmap_obstacles)
  {
    const auto lock_requested = std::chrono::steady_clock::now();
    std::unique_lock<std::recursive_mutex> lock(*costmap_->getMutex());
    const auto lock_acquired = std::chrono::steady_clock::now();
    lock_wait_ms = std::chrono::duration<double, std::milli>(
      lock_acquired - lock_requested).count();
    Eigen::Vector2d robot_orient = robot_pose_.orientationUnitVec();

    const auto width = costmap_->getSizeInCellsX();
    const auto height = costmap_->getSizeInCellsY();
    const auto cell_count = static_cast<std::size_t>(width) * height;
    const auto scan_started = std::chrono::steady_clock::now();
    std::vector<std::uint8_t> occupied(cell_count, 0);
    for (unsigned int y = 0; y < height; ++y) {
      for (unsigned int x = 0; x < width; ++x) {
        if (costmap_->getCost(x, y) != nav2_costmap_2d::LETHAL_OBSTACLE) {
          continue;
        }
        Eigen::Vector2d cell_center;
        costmap_->mapToWorld(x, y, cell_center.x(), cell_center.y());
        const Eigen::Vector2d obstacle_direction = cell_center - robot_pose_.position();
        if (obstacle_direction.dot(robot_orient) < 0.0 &&
          obstacle_direction.norm() > cfg_->obstacles.costmap_obstacles_behind_robot_dist)
        {
          continue;
        }
        occupied[static_cast<std::size_t>(y) * width + x] = 1;
        ++lethal_cells;
      }
    }
    const auto scan_finished = std::chrono::steady_clock::now();
    scan_ms = std::chrono::duration<double, std::milli>(scan_finished - scan_started).count();

    // Tile lethal cells with exact rectangles. This keeps free cells inside
    // U-shaped, ring-shaped, and diagonal obstacles free. A 0.5 m side cap
    // also keeps large lethal regions from becoming distant, ignored obstacles.
    const double half_cell = costmap_->getResolution() * 0.5;
    const auto max_cells_per_side = std::max(
      1U, static_cast<unsigned int>(std::floor(0.5 / costmap_->getResolution())));
    const auto tiling_started = std::chrono::steady_clock::now();
    const auto tiles = detail::tileLethalCells(occupied, width, height, max_cells_per_side);
    obstacle_tiles = tiles.size();
    for (const auto & tile : tiles) {
      double min_x;
      double min_y;
      double max_x;
      double max_y;
      costmap_->mapToWorld(tile.min_x, tile.min_y, min_x, min_y);
      costmap_->mapToWorld(tile.max_x, tile.max_y, max_x, max_y);
      Point2dContainer vertices{
        Eigen::Vector2d(min_x - half_cell, min_y - half_cell),
        Eigen::Vector2d(min_x - half_cell, max_y + half_cell),
        Eigen::Vector2d(max_x + half_cell, max_y + half_cell),
        Eigen::Vector2d(max_x + half_cell, min_y - half_cell)};
      obstacles_.push_back(ObstaclePtr(new PolygonObstacle(vertices)));
    }
    tiling_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - tiling_started).count();
  }
}

//void TebLocalPlannerROS::updateObstacleContainerWithCostmapConverter()
//{
  //if (!costmap_converter_)
    //return;
    
  //Get obstacles from costmap converter
  //costmap_converter::ObstacleArrayConstPtr obstacles = costmap_converter_->getObstacles();
  //if (!obstacles)
    //return;

  //for (std::size_t i=0; i<obstacles->obstacles.size(); ++i)
  //{
    //const costmap_converter_msgs::msg::ObstacleMsg* obstacle = &obstacles->obstacles.at(i);
    //const geometry_msgs::msg::Polygon* polygon = &obstacle->polygon;

    //if (polygon->points.size()==1 && obstacle->radius > 0) // Circle
    //{
      //obstacles_.push_back(ObstaclePtr(new CircularObstacle(polygon->points[0].x, polygon->points[0].y, obstacle->radius)));
    //}
    //else if (polygon->points.size()==1) // Point
    //{
      //obstacles_.push_back(ObstaclePtr(new PointObstacle(polygon->points[0].x, polygon->points[0].y)));
    //}
    //else if (polygon->points.size()==2) // Line
    //{
      //obstacles_.push_back(ObstaclePtr(new LineObstacle(polygon->points[0].x, polygon->points[0].y,
                                                        //polygon->points[1].x, polygon->points[1].y )));
    //}
    //else if (polygon->points.size()>2) // Real polygon
    //{
        //PolygonObstacle* polyobst = new PolygonObstacle;
        //for (std::size_t j=0; j<polygon->points.size(); ++j)
        //{
            //polyobst->pushBackVertex(polygon->points[j].x, polygon->points[j].y);
        //}
        //polyobst->finalizePolygon();
        //obstacles_.push_back(ObstaclePtr(polyobst));
    //}

    // Set velocity, if obstacle is moving
    //if(!obstacles_.empty())
      //obstacles_.back()->setCentroidVelocity(obstacles->obstacles[i].velocities, obstacles->obstacles[i].orientation);
  //}
//}




void TebLocalPlannerROS::updateViaPointsContainer(const std::vector<geometry_msgs::msg::PoseStamped>& transformed_plan, double min_separation)
{
  via_points_.clear();
  
  if (min_separation<=0)
    return;
  
  std::size_t prev_idx = 0;
  for (std::size_t i=1; i < transformed_plan.size(); ++i) // skip first one, since we do not need any point before the first min_separation [m]
  {
    // check separation to the previous via-point inserted
    if (distance_points2d( transformed_plan[prev_idx].pose.position, transformed_plan[i].pose.position ) < min_separation)
      continue;
        
    // add via-point
    via_points_.push_back( Eigen::Vector2d( transformed_plan[i].pose.position.x, transformed_plan[i].pose.position.y ) );
    prev_idx = i;
  }
  
}
      
//Eigen::Vector2d TebLocalPlannerROS::tfPoseToEigenVector2dTransRot(const tf::Pose& tf_vel)
//{
//  Eigen::Vector2d vel;
//  vel.coeffRef(0) = std::sqrt( tf_vel.getOrigin().getX() * tf_vel.getOrigin().getX() + tf_vel.getOrigin().getY() * tf_vel.getOrigin().getY() );
//  vel.coeffRef(1) = tf::getYaw(tf_vel.getRotation());
//  return vel;
//}
      
      
bool TebLocalPlannerROS::pruneGlobalPlan(const geometry_msgs::msg::PoseStamped& global_pose, std::vector<geometry_msgs::msg::PoseStamped>& global_plan, double dist_behind_robot)
{
  if (global_plan.empty())
    return true;
  
  try
  {
    // transform robot pose into the plan frame (we do not wait here, since pruning not crucial, if missed a few times)
    //geometry_msgs::msg::TransformStamped global_to_plan_transform = tf_->lookupTransform(global_plan.front().header.frame_id, global_pose.header.frame_id, tf2::timeFromSec(0));
    geometry_msgs::msg::PoseStamped robot = tf_->transform(
              global_pose,
              global_plan.front().header.frame_id);

    //robot.setData( global_to_plan_transform * global_pose );
    
    double dist_thresh_sq = dist_behind_robot*dist_behind_robot;
    
    // iterate plan until a pose close the robot is found
    std::vector<geometry_msgs::msg::PoseStamped>::iterator it = global_plan.begin();
    std::vector<geometry_msgs::msg::PoseStamped>::iterator erase_end = it;
    while (it != global_plan.end())
    {
      double dx = robot.pose.position.x - it->pose.position.x;
      double dy = robot.pose.position.y - it->pose.position.y;
      double dist_sq = dx * dx + dy * dy;
      if (dist_sq < dist_thresh_sq)
      {
         erase_end = it;
         break;
      }
      ++it;
    }
    if (erase_end == global_plan.end())
      return false;
    
    if (erase_end != global_plan.begin())
      global_plan.erase(global_plan.begin(), erase_end);
  }
  catch (const tf2::TransformException& ex)
  {
    RCLCPP_DEBUG(logger_, "Cannot prune path since no transform is available: %s\n", ex.what());
    return false;
  }
  return true;
}
      

bool TebLocalPlannerROS::transformGlobalPlan(const std::vector<geometry_msgs::msg::PoseStamped>& global_plan,
                  const geometry_msgs::msg::PoseStamped& global_pose, const nav2_costmap_2d::Costmap2D& costmap, const std::string& global_frame, double max_plan_length,
                  std::vector<geometry_msgs::msg::PoseStamped>& transformed_plan, int* current_goal_idx, geometry_msgs::msg::TransformStamped* tf_plan_to_global) const
{
  // this method is a slightly modified version of base_local_planner/goal_functions.h

  const geometry_msgs::msg::PoseStamped& plan_pose = global_plan[0];

  transformed_plan.clear();

  try 
  {
    if (global_plan.empty())
    {
      RCLCPP_ERROR(logger_, "Received plan with zero length");
      *current_goal_idx = 0;
      return false;
    }

    // get plan_to_global_transform from plan frame to global_frame
    geometry_msgs::msg::TransformStamped plan_to_global_transform = tf_->lookupTransform(
                global_frame, tf2_ros::fromMsg(plan_pose.header.stamp),
                plan_pose.header.frame_id, tf2::timeFromSec(0),
                plan_pose.header.frame_id, tf2::durationFromSec(0.5));

//    tf_->waitForTransform(global_frame, ros::Time::now(),
//    plan_pose.header.frame_id, plan_pose.header.stamp,
//    plan_pose.header.frame_id, ros::Duration(0.5));
//    tf_->lookupTransform(global_frame, ros::Time(),
//    plan_pose.header.frame_id, plan_pose.header.stamp,
//    plan_pose.header.frame_id, plan_to_global_transform);

    //let's get the pose of the robot in the frame of the plan
    geometry_msgs::msg::PoseStamped robot_pose = tf_->transform(global_pose, plan_pose.header.frame_id);

    //we'll discard points on the plan that are outside the local costmap
    double dist_threshold = std::max(costmap.getSizeInCellsX() * costmap.getResolution() / 2.0,
                                     costmap.getSizeInCellsY() * costmap.getResolution() / 2.0);
    dist_threshold *= 0.85; // just consider 85% of the costmap size to better incorporate point obstacle that are
                           // located on the border of the local costmap
    

    int i = 0;
    double sq_dist_threshold = dist_threshold * dist_threshold;
    double sq_dist = 1e10;
    
    //we need to loop to a point on the plan that is within a certain distance of the robot
    bool robot_reached = false;
    for(int j=0; j < (int)global_plan.size(); ++j)
    {
      double x_diff = robot_pose.pose.position.x - global_plan[j].pose.position.x;
      double y_diff = robot_pose.pose.position.y - global_plan[j].pose.position.y;
      double new_sq_dist = x_diff * x_diff + y_diff * y_diff;
      if (new_sq_dist > sq_dist_threshold)
        break;  // force stop if we have reached the costmap border

      if (robot_reached && new_sq_dist > sq_dist)
        break;

      if (new_sq_dist < sq_dist) // find closest distance
      {
        sq_dist = new_sq_dist;
        i = j;
        if (sq_dist < 0.05)      // 2.5 cm to the robot; take the immediate local minima; if it's not the global
          robot_reached = true;  // minima, probably means that there's a loop in the path, and so we prefer this
      }
    }

    geometry_msgs::msg::PoseStamped newer_pose;
    
    double plan_length = 0; // check cumulative Euclidean distance along the plan
    
    //now we'll transform until points are outside of our distance threshold
    while(i < (int)global_plan.size() && sq_dist <= sq_dist_threshold && (max_plan_length<=0 || plan_length <= max_plan_length))
    {
      //const geometry_msgs::msg::PoseStamped& pose = global_plan[i];
      //tf::poseStampedMsgToTF(pose, tf_pose);
      //tf_pose.setData(plan_to_global_transform * tf_pose);
      tf2::doTransform(global_plan[i], newer_pose, plan_to_global_transform);

//      tf_pose.stamp_ = plan_to_global_transform.stamp_;
//      tf_pose.frame_id_ = global_frame;
//      tf::poseStampedTFToMsg(tf_pose, newer_pose);

      transformed_plan.push_back(newer_pose);

      double x_diff = robot_pose.pose.position.x - global_plan[i].pose.position.x;
      double y_diff = robot_pose.pose.position.y - global_plan[i].pose.position.y;
      sq_dist = x_diff * x_diff + y_diff * y_diff;
      
      // caclulate distance to previous pose
      if (i>0 && max_plan_length>0)
        plan_length += distance_points2d(global_plan[i-1].pose.position, global_plan[i].pose.position);

      ++i;
    }
        
    // if we are really close to the goal (<sq_dist_threshold) and the goal is not yet reached (e.g. orientation error >>0)
    // the resulting transformed plan can be empty. In that case we explicitly inject the global goal.
    if (transformed_plan.empty())
    {
//      tf::poseStampedMsgToTF(global_plan.back(), tf_pose);
//      tf_pose.setData(plan_to_global_transform * tf_pose);
//      tf_pose.stamp_ = plan_to_global_transform.stamp_;
//      tf_pose.frame_id_ = global_frame;
//      tf::poseStampedTFToMsg(tf_pose, newer_pose);
      tf2::doTransform(global_plan.back(), newer_pose, plan_to_global_transform);

      transformed_plan.push_back(newer_pose);
      
      // Return the index of the current goal point (inside the distance threshold)
      if (current_goal_idx) *current_goal_idx = int(global_plan.size())-1;
    }
    else
    {
      // Return the index of the current goal point (inside the distance threshold)
      if (current_goal_idx) *current_goal_idx = i-1; // subtract 1, since i was increased once before leaving the loop
    }
    
    // Return the transformation from the global plan to the global planning frame if desired
    if (tf_plan_to_global) *tf_plan_to_global = plan_to_global_transform;
  }
  catch(tf2::LookupException& ex)
  {
    RCLCPP_ERROR(logger_, "No Transform available Error: %s\n", ex.what());
    return false;
  }
  catch(tf2::ConnectivityException& ex)
  {
    RCLCPP_ERROR(logger_, "Connectivity Error: %s\n", ex.what());
    return false;
  }
  catch(tf2::ExtrapolationException& ex)
  {
    RCLCPP_ERROR(logger_, "Extrapolation Error: %s\n", ex.what());
    if (global_plan.size() > 0)
      RCLCPP_ERROR(logger_, "Global Frame: %s Plan Frame size %d: %s\n", global_frame.c_str(), (unsigned int)global_plan.size(), global_plan[0].header.frame_id.c_str());

    return false;
  }

  return true;
}

    
      
      
double TebLocalPlannerROS::estimateLocalGoalOrientation(const std::vector<geometry_msgs::msg::PoseStamped>& global_plan, const geometry_msgs::msg::PoseStamped& local_goal,
                    int current_goal_idx, const geometry_msgs::msg::TransformStamped& tf_plan_to_global, int moving_average_length) const
{
  int n = (int)global_plan.size();
  
  // check if we are near the global goal already
  if (current_goal_idx > n-moving_average_length-2)
  {
    if (current_goal_idx >= n-1) // we've exactly reached the goal
    {
      return tf2::getYaw(local_goal.pose.orientation);
    }
    else
    {
//      tf::Quaternion global_orientation;
//      tf::quaternionMsgToTF(global_plan.back().pose.orientation, global_orientation);
//      return  tf2::getYaw(tf_plan_to_global.getRotation() *  global_orientation );

        tf2::Quaternion global_orientation, tf_plan_to_global_orientation;
        tf2::fromMsg(global_plan.back().pose.orientation, global_orientation);
        tf2::fromMsg(tf_plan_to_global.transform.rotation, tf_plan_to_global_orientation);

        return tf2::getYaw(tf_plan_to_global_orientation * global_orientation);
    }     
  }
  
  // reduce number of poses taken into account if the desired number of poses is not available
  moving_average_length = std::min(moving_average_length, n-current_goal_idx-1 ); // maybe redundant, since we have checked the vicinity of the goal before
  
  std::vector<double> candidates;
  geometry_msgs::msg::PoseStamped tf_pose_k = local_goal;
  geometry_msgs::msg::PoseStamped tf_pose_kp1;
  
  int range_end = current_goal_idx + moving_average_length;
  for (int i = current_goal_idx; i < range_end; ++i)
  {
    // Transform pose of the global plan to the planning frame
    const geometry_msgs::msg::PoseStamped& pose = global_plan.at(i+1);
    tf2::doTransform(global_plan.at(i+1), tf_pose_kp1, tf_plan_to_global);
      
    // calculate yaw angle  
    candidates.push_back( std::atan2(tf_pose_kp1.pose.position.y - tf_pose_k.pose.position.y,
              tf_pose_kp1.pose.position.x - tf_pose_k.pose.position.x ) );
    
    if (i<range_end-1) 
      tf_pose_k = tf_pose_kp1;
  }
  return average_angles(candidates);
}
      
      
void TebLocalPlannerROS::saturateVelocity(double& vx, double& vy, double& omega, double max_vel_x, double max_vel_y, double max_vel_theta, double max_vel_x_backwards) const
{
  double ratio_x = 1, ratio_omega = 1, ratio_y = 1;
  // Limit translational velocity for forward driving
  if (vx > max_vel_x)
    ratio_x = max_vel_x / vx;
  
  // limit strafing velocity
  if (vy > max_vel_y || vy < -max_vel_y)
    ratio_y = std::abs(max_vel_y / vy);
  
  // Limit angular velocity
  if (omega > max_vel_theta || omega < -max_vel_theta)
    ratio_omega = std::abs(max_vel_theta / omega);
  
  // Limit backwards velocity
  if (max_vel_x_backwards <= 0.0 && vx < 0.0)
    ratio_x = 0.0;
  else if (vx < -max_vel_x_backwards)
    ratio_x = - max_vel_x_backwards / vx;

  if (cfg_->robot.use_proportional_saturation)
  {
    double ratio = std::min(std::min(ratio_x, ratio_y), ratio_omega);
    vx *= ratio;
    vy *= ratio;
    omega *= ratio;
  }
  else
  {
    vx *= ratio_x;
    vy *= ratio_y;
    omega *= ratio_omega;
  }
}
     
     
double TebLocalPlannerROS::convertTransRotVelToSteeringAngle(double v, double omega, double wheelbase, double min_turning_radius) const
{
  if (omega==0 || v==0)
    return 0;
    
  double radius = v/omega;
  
  if (fabs(radius) < min_turning_radius)
    radius = double(sign(radius)) * min_turning_radius;

  return std::atan(wheelbase / radius);
}
     

void TebLocalPlannerROS::validateFootprints(double opt_inscribed_radius, double costmap_inscribed_radius, double min_obst_dist)
{
    RCLCPP_WARN_EXPRESSION(
                logger_, opt_inscribed_radius + min_obst_dist < costmap_inscribed_radius,
                  "The inscribed radius of the footprint specified for TEB optimization (%f) + min_obstacle_dist (%f) are smaller "
                  "than the inscribed radius of the robot's footprint in the costmap parameters (%f, including 'footprint_padding'). "
                  "Infeasible optimziation results might occur frequently!", opt_inscribed_radius, min_obst_dist, costmap_inscribed_radius);
}
   
   
   
void TebLocalPlannerROS::configureBackupModes(
    std::vector<geometry_msgs::msg::PoseStamped>& transformed_plan, int& goal_idx,
    const double remaining_path_distance, const double path_station,
    const double path_heading, const double robot_yaw)
{
    rclcpp::Time current_time = clock_->now();

    if (enable_reverse_suppression_ && pending_reverse_blocked_.exchange(false) &&
        !route_reverse_suppressed_) {
        route_reverse_suppressed_ = true;
        beginReverseSuppression(path_station, path_heading, robot_yaw);
        failure_detector_.clear();
        planner_->clearPlanner();
        last_cmd_ = geometry_msgs::msg::Twist();
        last_preferred_rotdir_ = RotType::none;
        planner_->setPreferredTurningDir(last_preferred_rotdir_);
        RCLCPP_WARN(
          logger_,
          "Navigation controller: category=reverse_safety "
          "event=reverse_blocked clearing_local_plan=true reverse_suppressed=true");
    }
    
    // reduced horizon backup mode
    if (cfg_->recovery.shrink_horizon_backup && 
        goal_idx < (int)transformed_plan.size()-1 && // we do not reduce if the goal is already selected (because the orientation might change -> can introduce oscillations)
       (no_infeasible_plans_>0 || (current_time - time_last_infeasible_plan_).seconds() < cfg_->recovery.shrink_horizon_min_duration )) // keep short horizon for at least a few seconds
    {
        RCLCPP_INFO_EXPRESSION(
                    logger_,
                    no_infeasible_plans_==1,
                    "Activating reduced horizon backup mode for at least %.2f sec (infeasible trajectory detected).", cfg_->recovery.shrink_horizon_min_duration);


        // Shorten horizon if requested
        // reduce to 50 percent:
        int horizon_reduction = goal_idx/2;
        
        if (no_infeasible_plans_ > 9)
        {
            RCLCPP_INFO_EXPRESSION(
                        logger_,
                        no_infeasible_plans_==10,
                        "Infeasible trajectory detected 10 times in a row: further reducing horizon...");
            horizon_reduction /= 2;
        }
        
        // we have a small overhead here, since we already transformed 50% more of the trajectory.
        // But that's ok for now, since we do not need to make transformGlobalPlan more complex 
        // and a reduced horizon should occur just rarely.
        int new_goal_idx_transformed_plan = int(transformed_plan.size()) - horizon_reduction - 1;
        goal_idx -= horizon_reduction;
        if (new_goal_idx_transformed_plan>0 && goal_idx >= 0)
            transformed_plan.erase(transformed_plan.begin()+new_goal_idx_transformed_plan, transformed_plan.end());
        else
            goal_idx += horizon_reduction; // this should not happen, but safety first ;-) 
    }
    
    
    // detect and resolve oscillations
    if (cfg_->recovery.oscillation_recovery)
    {
        double max_vel_theta;
        double max_vel_current = last_cmd_.linear.x >= 0 ? cfg_->robot.max_vel_x : cfg_->robot.max_vel_x_backwards;
        if (cfg_->robot.min_turning_radius!=0 && max_vel_current>0)
            max_vel_theta = std::max( max_vel_current/std::abs(cfg_->robot.min_turning_radius),  cfg_->robot.max_vel_theta );
        else
            max_vel_theta = cfg_->robot.max_vel_theta;
        
        failure_detector_.update(last_cmd_, cfg_->robot.max_vel_x, cfg_->robot.max_vel_x_backwards, max_vel_theta,
                               cfg_->recovery.oscillation_v_eps, cfg_->recovery.oscillation_omega_eps,
                               remaining_path_distance);
        
        bool oscillating = failure_detector_.isOscillating();
        const bool longitudinally_oscillating =
          failure_detector_.isLongitudinallyOscillating();
        bool recently_oscillated = (clock_->now()-time_last_oscillation_).seconds() < cfg_->recovery.oscillation_recovery_min_duration; // check if we have already detected an oscillation recently

        bool longitudinal_suppression_triggered = false;
        if (enable_reverse_suppression_ && longitudinally_oscillating &&
            !route_reverse_suppressed_) {
            longitudinal_suppression_triggered = true;
            route_reverse_suppressed_ = true;
            beginReverseSuppression(path_station, path_heading, robot_yaw);
            failure_detector_.clear();
            planner_->clearPlanner();
            last_cmd_ = geometry_msgs::msg::Twist();
            last_preferred_rotdir_ = RotType::none;
            planner_->setPreferredTurningDir(last_preferred_rotdir_);
            RCLCPP_WARN(
              logger_,
              "Repeated forward/reverse commands detected; clearing the local plan and suppressing reverse until the navigation goal changes.");
        }
        
        if (oscillating && !longitudinal_suppression_triggered)
        {
            if (!recently_oscillated)
            {
                // save current turning direction
                if (robot_vel_.angular.z > 0)
                    last_preferred_rotdir_ = RotType::left;
                else
                    last_preferred_rotdir_ = RotType::right;
                RCLCPP_INFO(logger_, "TebLocalPlannerROS: possible oscillation (of the robot or its local plan) detected. Activating recovery strategy (prefer current turning direction during optimization).");
            }
            time_last_oscillation_ = clock_->now();
            planner_->setPreferredTurningDir(last_preferred_rotdir_);
        }
        else if (!recently_oscillated && last_preferred_rotdir_ != RotType::none) // clear recovery behavior
        {
            last_preferred_rotdir_ = RotType::none;
            planner_->setPreferredTurningDir(last_preferred_rotdir_);
            RCLCPP_INFO(logger_, "TebLocalPlannerROS: oscillation recovery disabled/expired.");
        }
    }

}

void TebLocalPlannerROS::beginReverseSuppression(
  const double path_station, const double path_heading, const double robot_yaw)
{
  reverse_suppression_started_ = std::chrono::steady_clock::now();
  reverse_suppression_failed_ = false;
  reverse_suppression_best_path_station_ = path_station;
  reverse_suppression_target_yaw_ = path_heading;
  reverse_suppression_best_yaw_error_ =
    std::isfinite(path_heading) ? absoluteYawError(path_heading, robot_yaw) :
    std::numeric_limits<double>::quiet_NaN();
  reverse_suppression_rebase_progress_ = false;
}

void TebLocalPlannerROS::rebaseReverseSuppressionProgress(
  const double path_station, const double path_heading, const double robot_yaw)
{
  if (!route_reverse_suppressed_ || !reverse_suppression_rebase_progress_) {
    return;
  }
  reverse_suppression_best_path_station_ = path_station;
  reverse_suppression_target_yaw_ = path_heading;
  reverse_suppression_best_yaw_error_ = std::isfinite(path_heading)
    ? absoluteYawError(path_heading, robot_yaw) : std::numeric_limits<double>::quiet_NaN();
  reverse_suppression_rebase_progress_ = false;
}

void TebLocalPlannerROS::applyFollowPathAttemptUpdate(
  const double path_station, const double path_heading, const double robot_yaw)
{
  const std::uint64_t attempt_id = follow_path_attempt_id_.load();
  if (attempt_id == 0 || attempt_id == last_follow_path_attempt_id_) {
    return;
  }

  last_follow_path_attempt_id_ = attempt_id;
  follow_path_progress_best_path_station_ = path_station;
  follow_path_progress_target_yaw_ = path_heading;
  follow_path_progress_best_yaw_error_ = std::isfinite(path_heading)
    ? absoluteYawError(path_heading, robot_yaw) : std::numeric_limits<double>::quiet_NaN();
  RCLCPP_INFO(
    logger_, "Received FollowPath attempt %llu; route reverse suppression is %s.",
    static_cast<unsigned long long>(attempt_id),
    route_reverse_suppressed_ ? "active" : "inactive");
  if (!route_reverse_suppressed_) {
    return;
  }

  // A new BT FollowPath invocation starts a fresh bounded attempt. Ordinary setPlan
  // updates do not emit this signal and therefore cannot restart the deadline.
  reverse_suppression_started_ = std::chrono::steady_clock::now();
  reverse_suppression_failed_ = false;
  reverse_suppression_best_path_station_ = path_station;
  reverse_suppression_target_yaw_ = path_heading;
  reverse_suppression_best_yaw_error_ =
    std::isfinite(path_heading) ? absoluteYawError(path_heading, robot_yaw) :
    std::numeric_limits<double>::quiet_NaN();
}

void TebLocalPlannerROS::publishMeasuredFollowPathProgress(
  const double path_station, const double path_heading, const double robot_yaw)
{
  const std::uint64_t attempt_id = follow_path_attempt_id_.load();
  if (attempt_id == 0 || attempt_id != last_follow_path_attempt_id_ ||
    !follow_path_progress_pub_)
  {
    return;
  }

  bool made_progress = false;
  if (std::isfinite(path_station)) {
    if (!std::isfinite(follow_path_progress_best_path_station_)) {
      follow_path_progress_best_path_station_ = path_station;
    } else if (path_station - follow_path_progress_best_path_station_ >=
      kRouteProgressRenewalDistance)
    {
      follow_path_progress_best_path_station_ = path_station;
      made_progress = true;
    }
  }

  if (made_progress && std::isfinite(path_heading)) {
    follow_path_progress_target_yaw_ = path_heading;
    follow_path_progress_best_yaw_error_ = absoluteYawError(path_heading, robot_yaw);
  } else if (std::isfinite(follow_path_progress_target_yaw_)) {
    const double yaw_error = absoluteYawError(follow_path_progress_target_yaw_, robot_yaw);
    if (!std::isfinite(follow_path_progress_best_yaw_error_)) {
      follow_path_progress_best_yaw_error_ = yaw_error;
    } else if (follow_path_progress_best_yaw_error_ - yaw_error >=
      kHeadingProgressRenewalAngle)
    {
      follow_path_progress_best_yaw_error_ = yaw_error;
      made_progress = true;
    }
  }

  if (made_progress) {
    std_msgs::msg::UInt64 progress;
    progress.data = attempt_id;
    follow_path_progress_pub_->publish(progress);
  }
}

void TebLocalPlannerROS::enforceReverseSuppressionDeadline(
  const double path_station, const double path_heading, const double robot_yaw)
{
  if (!route_reverse_suppressed_) {
    return;
  }

  if (reverse_suppression_failed_) {
    throw std::runtime_error(
      "Reverse suppression deadline expired without measured route or heading progress.");
  }

  bool made_progress = false;
  if (std::isfinite(path_station)) {
    if (!std::isfinite(reverse_suppression_best_path_station_)) {
      reverse_suppression_best_path_station_ = path_station;
    } else if (path_station - reverse_suppression_best_path_station_ >=
      kRouteProgressRenewalDistance)
    {
      reverse_suppression_best_path_station_ = path_station;
      made_progress = true;
    }
  }

  if (made_progress && std::isfinite(path_heading)) {
    reverse_suppression_target_yaw_ = path_heading;
    reverse_suppression_best_yaw_error_ = absoluteYawError(path_heading, robot_yaw);
  } else if (std::isfinite(reverse_suppression_target_yaw_)) {
    const double yaw_error = absoluteYawError(reverse_suppression_target_yaw_, robot_yaw);
    if (!std::isfinite(reverse_suppression_best_yaw_error_)) {
      reverse_suppression_best_yaw_error_ = yaw_error;
    } else if (reverse_suppression_best_yaw_error_ - yaw_error >=
      kHeadingProgressRenewalAngle)
    {
      reverse_suppression_best_yaw_error_ = yaw_error;
      made_progress = true;
    }
  }

  if (made_progress) {
    reverse_suppression_started_ = std::chrono::steady_clock::now();
  }

  if (std::chrono::duration<double>(
      std::chrono::steady_clock::now() - reverse_suppression_started_).count() >=
    reverse_suppression_timeout_)
  {
    reverse_suppression_failed_ = true;
    planner_->clearPlanner();
    ++no_infeasible_plans_;
    time_last_infeasible_plan_ = clock_->now();
    last_cmd_ = geometry_msgs::msg::Twist();
    RCLCPP_ERROR(
      logger_,
      "Navigation controller: category=reverse_safety_timeout "
      "made no %.2f m route or %.2f rad heading progress for %.2f s; "
      "failing FollowPath.", kRouteProgressRenewalDistance,
      kHeadingProgressRenewalAngle, reverse_suppression_timeout_);
    throw std::runtime_error(
      "Reverse suppression deadline expired without measured route or heading progress.");
  }
}

bool TebLocalPlannerROS::applyNavigationGoalIdentityUpdate()
{
  std::vector<std::uint8_t> pending_identity;
  {
    std::lock_guard<std::mutex> lock(navigation_goal_identity_mutex_);
    pending_identity = pending_navigation_goal_identity_;
  }
  if (pending_identity.empty() || pending_identity == navigation_goal_identity_) {
    return false;
  }

  const bool changed = have_navigation_goal_identity_;
  navigation_goal_identity_ = std::move(pending_identity);
  have_navigation_goal_identity_ = true;
  return changed;
}

void TebLocalPlannerROS::resetReverseSuppressionRoute()
{
  route_reverse_suppressed_ = false;
  reverse_suppression_failed_ = false;
  reverse_suppression_best_path_station_ = std::numeric_limits<double>::quiet_NaN();
  reverse_suppression_target_yaw_ = std::numeric_limits<double>::quiet_NaN();
  reverse_suppression_best_yaw_error_ = std::numeric_limits<double>::quiet_NaN();
  reverse_suppression_started_ = std::chrono::steady_clock::time_point();
  reverse_suppression_rebase_progress_ = false;
  follow_path_progress_best_path_station_ = std::numeric_limits<double>::quiet_NaN();
  follow_path_progress_target_yaw_ = std::numeric_limits<double>::quiet_NaN();
  follow_path_progress_best_yaw_error_ = std::numeric_limits<double>::quiet_NaN();
}
     

void TebLocalPlannerROS::setSpeedLimit(
    const double & speed_limit, const bool & percentage)
{
  applySpeedLimit(cfg_->robot, speed_limit, percentage);
}

void TebLocalPlannerROS::applySpeedLimit(
    TebConfig::Robot & robot, const double & speed_limit, const bool & percentage)
{
  if (speed_limit == nav2_costmap_2d::NO_SPEED_LIMIT) {
    // Restore default value
    robot.max_vel_x = robot.base_max_vel_x;
    robot.max_vel_x_backwards = robot.base_max_vel_x_backwards;
    robot.max_vel_y = robot.base_max_vel_y;
    robot.max_vel_theta = robot.base_max_vel_theta;
  } else {
    if (percentage) {
      // Speed limit is expressed in % from maximum speed of robot
      robot.max_vel_x = robot.base_max_vel_x * speed_limit / 100.0;
      robot.max_vel_x_backwards = robot.base_max_vel_x_backwards * speed_limit / 100.0;
      robot.max_vel_y = robot.base_max_vel_y * speed_limit / 100.0;
      robot.max_vel_theta = robot.base_max_vel_theta * speed_limit / 100.0;
    } else {
      // Speed limit is expressed in absolute value
      double max_speed_xy = std::max(
            std::max(robot.base_max_vel_x, robot.base_max_vel_x_backwards), robot.base_max_vel_y);
      // Always recompute from the immutable baselines.  A later limit can be
      // less restrictive than an earlier one, in which case the current
      // values must be restored instead of retaining the old lower limit.
      const double ratio = std::max(0.0, std::min(1.0, speed_limit / max_speed_xy));
      robot.max_vel_x = robot.base_max_vel_x * ratio;
      robot.max_vel_x_backwards = robot.base_max_vel_x_backwards * ratio;
      robot.max_vel_y = robot.base_max_vel_y * ratio;
      robot.max_vel_theta = robot.base_max_vel_theta * ratio;
    }
  }
}

//void TebLocalPlannerROS::customObstacleCB(const costmap_converter_msgs::msg::ObstacleArrayMsg::ConstSharedPtr obst_msg)
//{
  //std::lock_guard<std::mutex> l(custom_obst_mutex_);
  //custom_obstacle_msg_ = *obst_msg;  
//}

void TebLocalPlannerROS::customViaPointsCB(const nav_msgs::msg::Path::ConstSharedPtr via_points_msg)
{
  RCLCPP_INFO_ONCE(logger_, "Via-points received. This message is printed once.");
  if (cfg_->trajectory.global_plan_viapoint_sep > 0)
  {
    RCLCPP_INFO(logger_, "Via-points are already obtained from the global plan (global_plan_viapoint_sep>0)."
             "Ignoring custom via-points.");
    custom_via_points_active_ = false;
    return;
  }

  std::lock_guard<std::mutex> l(via_point_mutex_);
  via_points_.clear();
  for (const geometry_msgs::msg::PoseStamped& pose : via_points_msg->poses)
  {
    via_points_.emplace_back(pose.pose.position.x, pose.pose.position.y);
  }
  custom_via_points_active_ = !via_points_.empty();
}

void TebLocalPlannerROS::activate() {
  visualization_->on_activate();

  return;
}
void TebLocalPlannerROS::deactivate() {
  visualization_->on_deactivate();

  return;
}
void TebLocalPlannerROS::cleanup() {
  path_change_tracker_.reset();
  route_progress_change_tracker_.reset();
  route_progress_reference_path_.clear();
  route_progress_last_projected_station_ = std::numeric_limits<double>::quiet_NaN();
  resetReverseSuppressionRoute();
  visualization_->on_cleanup();
  //costmap_converter_->stopWorker();
  
  return;
}

} // end namespace teb_local_planner

// register this planner as a nav2_core::Controller plugin
PLUGINLIB_EXPORT_CLASS(teb_local_planner::TebLocalPlannerROS, nav2_core::Controller)
