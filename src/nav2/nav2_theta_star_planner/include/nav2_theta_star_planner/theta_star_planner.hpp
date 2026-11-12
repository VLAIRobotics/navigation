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

#ifndef NAV2_THETA_STAR_PLANNER__THETA_STAR_PLANNER_HPP_
#define NAV2_THETA_STAR_PLANNER__THETA_STAR_PLANNER_HPP_

#include <iostream>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>
#include <chrono>
#include <queue>
#include <algorithm>
#include <memory>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_theta_star_planner/theta_star.hpp"
#include "nav2_util/geometry_utils.hpp"

using rcl_interfaces::msg::ParameterType;

namespace nav2_theta_star_planner
{

class ThetaStarPlanner : public nav2_core::GlobalPlanner
{
public:
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;

  void activate() override;

  void deactivate() override;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

protected:
  struct PathMetrics
  {
    double length{0.0};
    double traversal_cost{0.0};
    double clearance_cost{0.0};
    double minimum_obstacle_gap{0.0};
    bool obstacle_gap_capped{true};
    double exclusive_length{0.0};
    double exclusive_traversal_cost{0.0};
    double exclusive_clearance_cost{0.0};
    double exclusive_minimum_obstacle_gap{0.0};
    double exclusive_high_cost_length{0.0};
    double exclusive_score{0.0};
    bool has_exclusive_segment{false};
    bool exclusive_obstacle_gap_capped{true};
  };

  std::shared_ptr<tf2_ros::Buffer> tf_;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Logger logger_{rclcpp::get_logger("ThetaStarPlanner")};
  std::string global_frame_, name_;
  bool use_final_approach_orientation_;

  double path_hysteresis_ratio_{0.0};
  double path_hysteresis_max_rejoin_distance_{0.20};
  double path_hysteresis_max_length_ratio_{1.25};
  double path_hysteresis_min_clearance_gain_{0.20};
  double path_clearance_search_radius_{1.0};
  bool obstacle_field_ready_for_plan_{false};
  int path_hysteresis_confirmations_{1};
  int pending_better_candidate_count_{0};
  nav_msgs::msg::Path pending_better_candidate_path_;
  std::vector<float> obstacle_distance_field_;
  std::uint32_t obstacle_distance_field_width_{0};
  std::uint32_t obstacle_distance_field_height_{0};
  double obstacle_distance_field_resolution_{0.0};
  double obstacle_field_build_ms_{0.0};
  nav_msgs::msg::Path cached_path_;
  geometry_msgs::msg::PoseStamped current_goal_pose_;
  geometry_msgs::msg::PoseStamped cached_goal_pose_;
  bool have_cached_path_{false};

  // parent node weak ptr
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr plan_status_publisher_;
  std::string plan_instance_id_;
  std::uint64_t plan_sequence_{0};
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;

  std::unique_ptr<theta_star::ThetaStar> planner_;

  // Dynamic parameters handler
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;


  /**
   * @brief the function responsible for calling the algorithm and retrieving a path from it
   * @return global_path is the planned path to be taken
   */
  void getPlan(nav_msgs::msg::Path & global_path);
  void publishPlanOutcome(
    const char * state, const geometry_msgs::msg::PoseStamped & goal);

  /**
   * @brief interpolates points between the consecutive waypoints of the path
   * @param raw_path is used to send in the path received from the planner
   * @param dist_bw_points is used to send in the interpolation_resolution (which has been set as the costmap resolution)
   * @return the final path with waypoints at a distance of the value of interpolation_resolution of each other
   */
  static nav_msgs::msg::Path linearInterpolation(
    const std::vector<coordsW> & raw_path,
    const double & dist_bw_points);

  /**
   * @brief Keeps serving the previous path while it is traversable and no materially
   * safer, cheaper, or shorter candidate persists within the configured length bound.
   * Consecutive Theta* solutions can differ in topology after a single-cell shift of
   * the start cell, so without this the via-point consumers downstream see the path
   * jump every replan.
   * @param start current robot pose, used to trim the retained path and price the rejoin
   * @param global_path in: fresh candidate, out: the path to serve. The requested world
   * goal pose is used as the cache identity so rolling costmap indices do not invalidate it.
   */
  void applyPathHysteresis(
    const geometry_msgs::msg::PoseStamped & start,
    nav_msgs::msg::Path & global_path);

  /**
   * @brief Checks a retained path against the current costmap under the same rules the
   * search uses, exempting only the cell the robot itself stands in
   * @return false if any segment crosses outside the map, a lethal cell, inscribed
   * inflation outside the start's escape region, or unknown space while allow_unknown
   * is disabled
   */
  bool pathTraversable(
    const nav_msgs::msg::Path & path,
    const geometry_msgs::msg::PoseStamped & robot,
    PathMetrics * metrics = nullptr, theta_star::ThetaStar::Rejection * rejection = nullptr) const;

  double obstacleGapAtCell(unsigned int mx, unsigned int my, bool & capped) const;

  void logPathDecision(
    const char * reason, const char * selected,
    const PathMetrics * retained, const PathMetrics * candidate,
    double retained_score, double candidate_score) const;

  bool candidateRouteMatchesPending(const nav_msgs::msg::Path & candidate) const;

  void buildObstacleDistanceField();

  void measureExclusivePath(
    const nav_msgs::msg::Path & path, const nav_msgs::msg::Path & other,
    PathMetrics & metrics) const;

  /**
   * @brief Callback executed when a paramter change is detected
   * @param parameters list of changed parameters
   */
  rcl_interfaces::msg::SetParametersResult
  dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);
};
}   //  namespace nav2_theta_star_planner

#endif  //  NAV2_THETA_STAR_PLANNER__THETA_STAR_PLANNER_HPP_
