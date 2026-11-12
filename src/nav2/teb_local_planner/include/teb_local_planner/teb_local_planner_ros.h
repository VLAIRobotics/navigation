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

#ifndef TEB_LOCAL_PLANNER_ROS_H_
#define TEB_LOCAL_PLANNER_ROS_H_

#include <pluginlib/class_loader.hpp>

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <mutex>
#include <vector>

#include <rclcpp/rclcpp.hpp>

// Navigation2 local planner base class and utilities
#include <nav2_core/controller.hpp>

// timed-elastic-band related classes
#include "teb_local_planner/optimal_planner.h"
#include "teb_local_planner/homotopy_class_planner.h"
#include "teb_local_planner/visualization.h"
#include "teb_local_planner/recovery_behaviors.h"
#include "teb_local_planner/path_change_tracker.h"

// message types
#include <action_msgs/msg/goal_status_array.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int64.hpp>
//#include <costmap_converter_msgs/msg/obstacle_msg.hpp>

// transforms
#include <tf2_ros/transform_listener.h>
#include <tf2/transform_datatypes.h>

// costmap
//#include <costmap_converter/costmap_converter_interface.h>
#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"

#include <nav2_util/lifecycle_node.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav_2d_utils/parameters.hpp>
#include "rcl_interfaces/msg/set_parameters_result.hpp"
// dynamic reconfigure
//#include "teb_local_planner/TebLocalPlannerReconfigureConfig.h>
//#include <dynamic_reconfigure/server.h>


namespace teb_local_planner
{
using TFBufferPtr = std::shared_ptr<tf2_ros::Buffer>;
using CostmapROSPtr = std::shared_ptr<nav2_costmap_2d::Costmap2DROS>;

/**
  * @class TebLocalPlannerROS
  * @brief Implements the actual abstract navigation stack routines of the teb_local_planner plugin
  * @todo Escape behavior, more efficient obstacle handling
  */
class TebLocalPlannerROS : public nav2_core::Controller
{

public:
  /**
    * @brief Constructor of the teb plugin
    */
  TebLocalPlannerROS();

  /**
    * @brief  Destructor of the plugin
    */
  ~TebLocalPlannerROS();

  // Applies a temporary speed limit while preserving the configured baselines.
  static void applySpeedLimit(TebConfig::Robot & robot, const double & speed_limit,
    const bool & percentage);
  
  /**
   * @brief Configure the teb plugin
   * 
   * @param node The node of the instance
   * @param tf Pointer to a transform listener
   * @param costmap_ros Cost map representing occupied and free space
   */
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  void activate() override;
  void deactivate() override;
  void cleanup() override;

  /**
    * @brief Initializes the teb plugin
    */
  void initialize(nav2_util::LifecycleNode::SharedPtr node);

  /**
    * @brief Set the plan that the teb local planner is following
    * @param orig_global_plan The plan to pass to the local planner
    * @return
    */
  void setPlan(const nav_msgs::msg::Path & orig_global_plan) override;

  /**
    * @brief Given the current position, orientation, and velocity of the robot, compute velocity commands to send to the base
    * @param pose is the current position
    * @param velocity is the current velocity
    * @return velocity commands to send to the base
    */
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped &pose,
    const geometry_msgs::msg::Twist &velocity,
      nav2_core::GoalChecker * goal_checker);
  
    
  /** @name Public utility functions/methods */
  //@{
  
    /**
    * @brief  Transform a tf::Pose type into a Eigen::Vector2d containing the translational and angular velocities.
    * 
    * Translational velocities (x- and y-coordinates) are combined into a single translational velocity (first component).
    * @param tf_vel tf::Pose message containing a 1D or 2D translational velocity (x,y) and an angular velocity (yaw-angle)
    * @return Translational and angular velocity combined into an Eigen::Vector2d
    */
//  static Eigen::Vector2d tfPoseToEigenVector2dTransRot(const tf::Pose& tf_vel);

  /**
   * @brief Get the current robot footprint/contour model
   * @param nh const reference to the local rclcpp::Node::SharedPtr
   * @return Robot footprint model used for optimization
   */
  RobotFootprintModelPtr getRobotFootprintFromParamServer(nav2_util::LifecycleNode::SharedPtr node);
  
  /** 
   * @brief Set the footprint from the given XmlRpcValue.
   * @remarks This method is copied from costmap_2d/footprint.h, since it is not declared public in all ros distros
   * @remarks It is modified in order to return a container of Eigen::Vector2d instead of geometry_msgs::msg::Point
   * @param footprint_xmlrpc should be an array of arrays, where the top-level array should have 3 or more elements, and the
   * sub-arrays should all have exactly 2 elements (x and y coordinates).
   * @param full_param_name this is the full name of the rosparam from which the footprint_xmlrpc value came. 
   * It is used only for reporting errors. 
   * @return container of vertices describing the polygon
   */
// Using ROS2 parameter server
//  static Point2dContainer makeFootprintFromXMLRPC(XmlRpc::XmlRpcValue& footprint_xmlrpc, const std::string& full_param_name);
  
  /** 
   * @brief Get a number from the given XmlRpcValue.
   * @remarks This method is copied from costmap_2d/footprint.h, since it is not declared public in all ros distros
   * @remarks It is modified in order to return a container of Eigen::Vector2d instead of geometry_msgs::msg::Point
   * @param value double value type
   * @param full_param_name this is the full name of the rosparam from which the footprint_xmlrpc value came. 
   * It is used only for reporting errors. 
   * @returns double value
   */
// Using ROS2 parameter server
//  static double getNumberFromXMLRPC(XmlRpc::XmlRpcValue& value, const std::string& full_param_name);
  
  //@}

protected:

  /**
    * @brief Update internal obstacle vector based on occupied costmap cells
    * @remarks All occupied cells will be added as point obstacles.
    * @remarks All previous obstacles are cleared.
    * @sa updateObstacleContainerWithCostmapConverter
    * @todo Include temporal coherence among obstacle msgs (id vector)
    * @todo Include properties for dynamic obstacles (e.g. using constant velocity model)
    */
  void updateObstacleContainerWithCostmap(
    double & lock_wait_ms, double & scan_ms, double & tiling_ms,
    std::size_t & lethal_cells, std::size_t & obstacle_tiles);
  
  /**
   * @brief Update internal obstacle vector based on polygons provided by a costmap_converter plugin
   * @remarks Requires a loaded costmap_converter plugin.
   * @remarks All previous obstacles are cleared.
   * @sa updateObstacleContainerWithCostmap
   */
  
  void updateViaPointsContainer(const std::vector<geometry_msgs::msg::PoseStamped>& transformed_plan, double min_separation);
  
  
  /**
    * @brief Callback for the dynamic_reconfigure node.
    * 
    * This callback allows to modify parameters dynamically at runtime without restarting the node
    * @param config Reference to the dynamic reconfigure config
    * @param level Dynamic reconfigure level
    */
  // TODO : dynamic reconfigure is not supported on ROS2
//  void reconfigureCB(TebLocalPlannerReconfigureConfig& config, uint32_t level);
  
  
   /**
    * @brief Callback for custom obstacles that are not obtained from the costmap 
    * @param obst_msg pointer to the message containing a list of polygon shaped obstacles
    */
  
  
   /**
    * @brief Callback for custom via-points
    * @param via_points_msg pointer to the message containing a list of via-points
    */
  void customViaPointsCB(const nav_msgs::msg::Path::ConstSharedPtr via_points_msg);

   /**
    * @brief Prune global plan such that already passed poses are cut off
    * 
    * The pose of the robot is transformed into the frame of the global plan by taking the most recent tf transform.
    * If no valid transformation can be found, the method returns \c false.
    * The global plan is pruned until the distance to the robot is at least \c dist_behind_robot.
    * If no pose within the specified treshold \c dist_behind_robot can be found,
    * nothing will be pruned and the method returns \c false.
    * @remarks Do not choose \c dist_behind_robot too small (not smaller the cellsize of the map), otherwise nothing will be pruned.
    * @param global_pose The global pose of the robot
    * @param[in,out] global_plan The plan to be transformed
    * @param dist_behind_robot Distance behind the robot that should be kept [meters]
    * @return \c true if the plan is pruned, \c false in case of a transform exception or if no pose cannot be found inside the threshold
    */
  bool pruneGlobalPlan(const geometry_msgs::msg::PoseStamped& global_pose,
                       std::vector<geometry_msgs::msg::PoseStamped>& global_plan, double dist_behind_robot=1);
  
  /**
    * @brief  Transforms the global plan of the robot from the planner frame to the local frame (modified).
    * 
    * The method replaces transformGlobalPlan as defined in base_local_planner/goal_functions.h 
    * such that the index of the current goal pose is returned as well as 
    * the transformation between the global plan and the planning frame.
    * @param global_plan The plan to be transformed
    * @param global_pose The global pose of the robot
    * @param costmap A reference to the costmap being used so the window size for transforming can be computed
    * @param global_frame The frame to transform the plan to
    * @param max_plan_length Specify maximum length (cumulative Euclidean distances) of the transformed plan [if <=0: disabled; the length is also bounded by the local costmap size!]
    * @param[out] transformed_plan Populated with the transformed plan
    * @param[out] current_goal_idx Index of the current (local) goal pose in the global plan
    * @param[out] tf_plan_to_global Transformation between the global plan and the global planning frame
    * @return \c true if the global plan is transformed, \c false otherwise
    */
  bool transformGlobalPlan(const std::vector<geometry_msgs::msg::PoseStamped>& global_plan,
                           const geometry_msgs::msg::PoseStamped& global_pose,  const nav2_costmap_2d::Costmap2D& costmap,
                           const std::string& global_frame, double max_plan_length, std::vector<geometry_msgs::msg::PoseStamped>& transformed_plan,
                           int* current_goal_idx = NULL, geometry_msgs::msg::TransformStamped* tf_plan_to_global = NULL) const;
    
  /**
    * @brief Estimate the orientation of a pose from the global_plan that is treated as a local goal for the local planner.
    * 
    * If the current (local) goal point is not the final one (global)
    * substitute the goal orientation by the angle of the direction vector between 
    * the local goal and the subsequent pose of the global plan. 
    * This is often helpful, if the global planner does not consider orientations. \n
    * A moving average filter is utilized to smooth the orientation.
    * @param global_plan The global plan
    * @param local_goal Current local goal
    * @param current_goal_idx Index of the current (local) goal pose in the global plan
    * @param[out] tf_plan_to_global Transformation between the global plan and the global planning frame
    * @param moving_average_length number of future poses of the global plan to be taken into account
    * @return orientation (yaw-angle) estimate
    */
  double estimateLocalGoalOrientation(const std::vector<geometry_msgs::msg::PoseStamped>& global_plan, const geometry_msgs::msg::PoseStamped& local_goal,
                                      int current_goal_idx, const geometry_msgs::msg::TransformStamped& tf_plan_to_global, int moving_average_length=3) const;
        
        
  /**
   * @brief Saturate the translational and angular velocity to given limits.
   * 
   * The limit of the translational velocity for backwards driving can be changed independently.
   * Do not choose max_vel_x_backwards <= 0. If no backward driving is desired, change the optimization weight for
   * penalizing backwards driving instead.
   * @param[in,out] vx The translational velocity that should be saturated.
   * @param[in,out] vy Strafing velocity which can be nonzero for holonomic robots
   * @param[in,out] omega The angular velocity that should be saturated.
   * @param max_vel_x Maximum translational velocity for forward driving
   * @param max_vel_y Maximum strafing velocity (for holonomic robots)
   * @param max_vel_theta Maximum (absolute) angular velocity
   * @param max_vel_x_backwards Maximum translational velocity for backwards driving
   */
  void saturateVelocity(double& vx, double& vy, double& omega, double max_vel_x, double max_vel_y,
                        double max_vel_theta, double max_vel_x_backwards) const;

  
  /**
   * @brief Convert translational and rotational velocities to a steering angle of a carlike robot
   * 
   * The conversion is based on the following equations:
   * - The turning radius is defined by \f$ R = v/omega \f$
   * - For a car like robot withe a distance L between both axles, the relation is: \f$ tan(\phi) = L/R \f$
   * - phi denotes the steering angle.
   * @remarks You might provide distances instead of velocities, since the temporal information is not required.
   * @param v translational velocity [m/s]
   * @param omega rotational velocity [rad/s]
   * @param wheelbase distance between both axles (drive shaft and steering axle), the value might be negative for back_wheeled robots
   * @param min_turning_radius Specify a lower bound on the turning radius
   * @return Resulting steering angle in [rad] inbetween [-pi/2, pi/2]
   */
  double convertTransRotVelToSteeringAngle(double v, double omega, double wheelbase, double min_turning_radius = 0) const;
  
  /**
   * @brief Validate current parameter values of the footprint for optimization, obstacle distance and the costmap footprint
   * 
   * This method prints warnings if validation fails.
   * @remarks Currently, we only validate the inscribed radius of the footprints
   * @param opt_inscribed_radius Inscribed radius of the RobotFootprintModel for optimization
   * @param costmap_inscribed_radius Inscribed radius of the footprint model used for the costmap
   * @param min_obst_dist desired distance to obstacles
   */
  void validateFootprints(double opt_inscribed_radius, double costmap_inscribed_radius, double min_obst_dist);
  
  
  void configureBackupModes(
    std::vector<geometry_msgs::msg::PoseStamped>& transformed_plan, int& goal_idx,
    double remaining_path_distance, double path_station, double path_heading, double robot_yaw);
  void beginReverseSuppression(double path_station, double path_heading, double robot_yaw);
  void rebaseReverseSuppressionProgress(
    double path_station, double path_heading, double robot_yaw);
  void applyFollowPathAttemptUpdate(
    double path_station, double path_heading, double robot_yaw);
  void publishMeasuredFollowPathProgress(
    double path_station, double path_heading, double robot_yaw);
  void enforceReverseSuppressionDeadline(
    double path_station, double path_heading, double robot_yaw);
  bool applyNavigationGoalIdentityUpdate();
  void resetReverseSuppressionRoute();
  
  /**
   * @brief Limits the maximum linear speed of the robot.
   * @param speed_limit expressed in absolute value (in m/s)
   * or in percentage from maximum robot speed.
   * @param percentage Setting speed limit in percentage if true
   * or in absolute values in false case.
  */
  void setSpeedLimit(const double & speed_limit,  const bool & percentage);

  std::string describeFeasibilityFailure(
    const PlannerInterface::FeasibilityFailure & failure,
    std::string * failure_key);

private:
  // Definition of member variables
  rclcpp_lifecycle::LifecycleNode::WeakPtr nh_;
  rclcpp::Logger logger_{rclcpp::get_logger("TEBLocalPlanner")};
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Node::SharedPtr intra_proc_node_;
  // external objects (store weak pointers)
  CostmapROSPtr costmap_ros_; //!< Pointer to the costmap ros wrapper, received from the navigation stack
  nav2_costmap_2d::Costmap2D* costmap_; //!< Pointer to the 2d costmap (obtained from the costmap ros wrapper)
  TFBufferPtr tf_; //!< pointer to Transform Listener
  TebConfig::UniquePtr cfg_; //!< Config class that stores and manages all related parameters
    
  // internal objects (memory management owned)
  PlannerInterfacePtr planner_; //!< Instance of the underlying optimal planner class
  ObstContainer obstacles_; //!< Obstacle vector that should be considered during local trajectory optimization
  ViaPointContainer via_points_; //!< Container of via-points that should be considered during local trajectory optimization
  TebVisualizationPtr visualization_; //!< Instance of the visualization class (local/global plan, obstacles, ...)
  std::shared_ptr<dwb_critics::ObstacleFootprintCritic> costmap_model_;
  FailureDetector failure_detector_; //!< Detect if the robot got stucked
  
  std::vector<geometry_msgs::msg::PoseStamped> global_plan_; //!< Store the current global plan
  std::vector<geometry_msgs::msg::PoseStamped> route_progress_reference_path_;
  double route_progress_last_projected_station_{std::numeric_limits<double>::quiet_NaN()};
  PathChangeTracker path_change_tracker_;
  PathChangeTracker route_progress_change_tracker_{true};
  
 

  //std::shared_ptr< dynamic_reconfigure::Server<TebLocalPlannerReconfigureConfig> > dynamic_recfg_; //!< Dynamic reconfigure server to allow config modifications at runtime
   
  std::mutex custom_obst_mutex_; //!< Mutex that locks the obstacle array (multi-threaded)
  

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr via_points_sub_; //!< Subscriber for custom via-points received via a Path msg.
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr reverse_blocked_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt64>::SharedPtr follow_path_attempt_sub_;
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr follow_path_attempt_ack_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr follow_path_progress_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr failure_diagnostic_pub_;
  std::string last_feasibility_failure_key_;
  rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr navigation_goal_status_sub_;
  std::mutex navigation_goal_identity_mutex_;
  std::vector<std::uint8_t> pending_navigation_goal_identity_;
  std::vector<std::uint8_t> navigation_goal_identity_;
  bool have_navigation_goal_identity_{false};
  bool custom_via_points_active_; //!< Keep track whether valid via-points have been received from via_points_sub_
  std::mutex via_point_mutex_; //!< Mutex that locks the via_points container (multi-threaded)

  PoseSE2 robot_pose_; //!< Store current robot pose
  PoseSE2 robot_goal_; //!< Store current robot goal
  geometry_msgs::msg::Twist robot_vel_; //!< Store current robot translational and angular velocity (vx, vy, omega)
  rclcpp::Time time_last_infeasible_plan_; //!< Store at which time stamp the last infeasible plan was detected
  int no_infeasible_plans_; //!< Store how many times in a row the planner failed to find a feasible plan.
  rclcpp::Time time_last_oscillation_; //!< Store at which time stamp the last oscillation was detected
  RotType last_preferred_rotdir_; //!< Store recent preferred turning direction
  geometry_msgs::msg::Twist last_cmd_; //!< Store the last control command generated in computeVelocityCommands()
  bool enable_reverse_suppression_{false};
  bool route_reverse_suppressed_{false};
  std::atomic<bool> reverse_blocked_state_{false};
  std::atomic<bool> pending_reverse_blocked_{false};
  std::atomic<std::uint64_t> follow_path_attempt_id_{0};
  std::uint64_t last_follow_path_attempt_id_{0};
  double follow_path_progress_best_path_station_{std::numeric_limits<double>::quiet_NaN()};
  double follow_path_progress_target_yaw_{std::numeric_limits<double>::quiet_NaN()};
  double follow_path_progress_best_yaw_error_{std::numeric_limits<double>::quiet_NaN()};
  std::uint64_t control_timing_count_{0};
  double controller_frequency_hz_{5.0};
  std::uint64_t control_cycle_count_{0};
  std::uint64_t control_cycle_overruns_window_{0};
  double control_cycle_overrun_max_ms_{0.0};
  double reverse_suppression_timeout_{3.0};
  double reverse_suppression_sweep_command_delay_{0.15};
  bool reverse_suppression_failed_{false};
  double reverse_suppression_best_path_station_{std::numeric_limits<double>::quiet_NaN()};
  double reverse_suppression_target_yaw_{std::numeric_limits<double>::quiet_NaN()};
  double reverse_suppression_best_yaw_error_{std::numeric_limits<double>::quiet_NaN()};
  std::chrono::steady_clock::time_point reverse_suppression_started_{};
  bool reverse_suppression_rebase_progress_{false};
  
  std::vector<geometry_msgs::msg::Point> footprint_spec_; //!< Store the footprint of the robot 
  double robot_inscribed_radius_; //!< The radius of the inscribed circle of the robot (collision possible)
  double robot_circumscribed_radius; //!< The radius of the circumscribed circle of the robot
    
  // flags
  bool initialized_; //!< Keeps track about the correct initialization of this class
  std::string name_; //!< Name of plugin ID

protected:
  // Dynamic parameters handler
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler;
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
  
}; // end namespace teb_local_planner

#endif // TEB_LOCAL_PLANNER_ROS_H_
