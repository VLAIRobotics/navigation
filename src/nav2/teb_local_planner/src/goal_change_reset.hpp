#ifndef TEB_LOCAL_PLANNER__GOAL_CHANGE_RESET_HPP_
#define TEB_LOCAL_PLANNER__GOAL_CHANGE_RESET_HPP_

#include <geometry_msgs/msg/twist.hpp>

#include "teb_local_planner/misc.h"
#include "teb_local_planner/planner_interface.h"
#include "teb_local_planner/recovery_behaviors.h"

namespace teb_local_planner
{
namespace detail
{

inline void resetForGoalChange(
  PlannerInterface & planner, FailureDetector & failure_detector,
  RotType & last_preferred_rotdir, int & no_infeasible_plans,
  geometry_msgs::msg::Twist & last_cmd)
{
  planner.clearPlanner();
  planner.setPreferredTurningDir(RotType::none);
  failure_detector.clear();
  last_preferred_rotdir = RotType::none;
  no_infeasible_plans = 0;
  last_cmd = geometry_msgs::msg::Twist();
}

}  // namespace detail
}  // namespace teb_local_planner

#endif  // TEB_LOCAL_PLANNER__GOAL_CHANGE_RESET_HPP_
