#ifndef TEB_LOCAL_PLANNER__GOAL_CHANGE_TRACKER_H_
#define TEB_LOCAL_PLANNER__GOAL_CHANGE_TRACKER_H_

#include <cmath>

#include <geometry_msgs/msg/pose_stamped.hpp>

namespace teb_local_planner
{

class GoalChangeTracker
{
public:
  static constexpr double kGoalChangeThreshold = 0.20;

  bool update(const geometry_msgs::msg::PoseStamped & goal)
  {
    bool changed = false;
    if (have_goal_) {
      const double dx = goal.pose.position.x - last_goal_.pose.position.x;
      const double dy = goal.pose.position.y - last_goal_.pose.position.y;
      changed = std::hypot(dx, dy) > kGoalChangeThreshold;
    }
    last_goal_ = goal;
    have_goal_ = true;
    return changed;
  }

  void reset()
  {
    last_goal_ = geometry_msgs::msg::PoseStamped();
    have_goal_ = false;
  }

private:
  geometry_msgs::msg::PoseStamped last_goal_;
  bool have_goal_{false};
};

}  // namespace teb_local_planner

#endif  // TEB_LOCAL_PLANNER__GOAL_CHANGE_TRACKER_H_
