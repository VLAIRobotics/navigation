#ifndef TEB_LOCAL_PLANNER__PATH_CHANGE_TRACKER_H_
#define TEB_LOCAL_PLANNER__PATH_CHANGE_TRACKER_H_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/utils.h>

namespace teb_local_planner
{

class PathChangeTracker
{
public:
  static constexpr double kChangeThreshold = 0.20;
  static constexpr double kGoalOrientationChangeThreshold = 0.05;
  static constexpr double kSampleSpacing = 0.10;

  explicit PathChangeTracker(bool stable_reference = false)
  : stable_reference_(stable_reference)
  {}

  bool update(const nav_msgs::msg::Path & path)
  {
    if (path.poses.empty()) {
      reset();
      return false;
    }

    bool changed = false;
    if (have_path_) {
      const auto & old_goal = last_path_.poses.back().pose.position;
      const auto & new_goal = path.poses.back().pose.position;
      const double goal_position_threshold = stable_reference_ ?
        kStableEndpointChangeThreshold : kChangeThreshold;
      changed = std::hypot(
        new_goal.x - old_goal.x, new_goal.y - old_goal.y) > goal_position_threshold;
      if (stable_reference_ && !changed) {
        const double old_yaw = tf2::getYaw(last_path_.poses.back().pose.orientation);
        const double new_yaw = tf2::getYaw(path.poses.back().pose.orientation);
        const double yaw_delta = std::remainder(
          new_yaw - old_yaw, 2.0 * std::acos(-1.0));
        changed = std::abs(yaw_delta) > kGoalOrientationChangeThreshold;
      }
      for (std::size_t i = 0; !changed && i < path.poses.size(); ++i) {
        const auto & from = path.poses[i].pose.position;
        if (i + 1 == path.poses.size()) {
          changed = distanceToPath(from.x, from.y, last_path_) > kChangeThreshold;
          continue;
        }

        const auto & to = path.poses[i + 1].pose.position;
        const double length = std::hypot(to.x - from.x, to.y - from.y);
        const int samples = std::max(1, static_cast<int>(std::ceil(length / kSampleSpacing)));
        for (int sample = 0; !changed && sample <= samples; ++sample) {
          const double fraction = static_cast<double>(sample) / samples;
          const double x = from.x + fraction * (to.x - from.x);
          const double y = from.y + fraction * (to.y - from.y);
          changed = distanceToPath(x, y, last_path_) > kChangeThreshold;
        }
      }
    }

    // The route-progress tracker holds its accepted path as a stable reference;
    // the planner warm-start tracker keeps its original per-update behavior.
    if (!have_path_ || changed || !stable_reference_) {
      last_path_ = path;
    }
    have_path_ = true;
    return changed;
  }

  void reset()
  {
    last_path_ = nav_msgs::msg::Path();
    have_path_ = false;
  }

private:
  static double distanceToPath(double x, double y, const nav_msgs::msg::Path & path)
  {
    if (path.poses.empty()) {
      return std::numeric_limits<double>::infinity();
    }
    if (path.poses.size() == 1) {
      const auto & point = path.poses.front().pose.position;
      return std::hypot(x - point.x, y - point.y);
    }

    double min_distance = std::numeric_limits<double>::infinity();
    for (std::size_t i = 1; i < path.poses.size(); ++i) {
      const auto & start = path.poses[i - 1].pose.position;
      const auto & end = path.poses[i].pose.position;
      const double dx = end.x - start.x;
      const double dy = end.y - start.y;
      const double length_sq = dx * dx + dy * dy;
      const double raw_projection = length_sq > 0.0 ?
        ((x - start.x) * dx + (y - start.y) * dy) / length_sq : 0.0;
      const double projection = std::max(0.0, std::min(raw_projection, 1.0));
      min_distance = std::min(
        min_distance,
        std::hypot(x - (start.x + projection * dx), y - (start.y + projection * dy)));
    }
    return min_distance;
  }

  nav_msgs::msg::Path last_path_;
  bool have_path_{false};
  bool stable_reference_{false};

  static constexpr double kStableEndpointChangeThreshold = 1e-6;
};

}  // namespace teb_local_planner

#endif  // TEB_LOCAL_PLANNER__PATH_CHANGE_TRACKER_H_
