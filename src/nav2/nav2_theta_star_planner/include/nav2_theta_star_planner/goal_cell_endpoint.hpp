#pragma once

#include <cmath>
#include <cstddef>
#include <string>

namespace nav2_theta_star_planner
{

/**
 * Multi-pose plans end at the requested goal pose after the planner verifies
 * that it belongs to the searched goal cell. A one-pose plan is the planner's
 * same-cell case and keeps the start pose. Callers must verify action identity
 * before accepting a same-cell plan or a route whose frames differ.
 */
enum class GoalCellEndpointResult
{
  MATCHED_REQUESTED_GOAL,
  SAME_CELL_PLAN,
  DIFFERENT_FRAMES,
  EMPTY_PATH,
  ENDPOINT_MISMATCH
};

inline constexpr double kGoalEndpointPositionTolerance = 1.0e-3;

inline GoalCellEndpointResult checkGoalCellEndpoint(
  const std::string & path_frame,
  const std::size_t path_size,
  const double endpoint_x,
  const double endpoint_y,
  const std::string & goal_frame,
  const double goal_x,
  const double goal_y) noexcept
{
  if (path_size == 0) {
    return GoalCellEndpointResult::EMPTY_PATH;
  }
  if (!std::isfinite(endpoint_x) || !std::isfinite(endpoint_y) ||
    !std::isfinite(goal_x) || !std::isfinite(goal_y))
  {
    return GoalCellEndpointResult::ENDPOINT_MISMATCH;
  }
  if (path_frame != goal_frame) {
    return GoalCellEndpointResult::DIFFERENT_FRAMES;
  }
  if (path_size == 1) {
    return GoalCellEndpointResult::SAME_CELL_PLAN;
  }
  return std::hypot(endpoint_x - goal_x, endpoint_y - goal_y) <=
         kGoalEndpointPositionTolerance ?
         GoalCellEndpointResult::MATCHED_REQUESTED_GOAL :
         GoalCellEndpointResult::ENDPOINT_MISMATCH;
}

}  // namespace nav2_theta_star_planner
