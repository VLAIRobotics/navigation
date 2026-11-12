#include <cmath>

#include <gtest/gtest.h>

#include "nav2_theta_star_planner/goal_cell_endpoint.hpp"

namespace nav2_theta_star_planner
{

TEST(GoalCellEndpoint, acceptsExactGoalAtCellCorner)
{
  EXPECT_EQ(
    checkGoalCellEndpoint("map", 2, 0.0499, 0.0499, "map", 0.0499, 0.0499),
    GoalCellEndpointResult::MATCHED_REQUESTED_GOAL);
}

TEST(GoalCellEndpoint, rejectsQuantizedCellCenterInsteadOfRequestedGoal)
{
  EXPECT_EQ(
    checkGoalCellEndpoint("map", 2, 0.05, 0.05, "map", 0.099, 0.099),
    GoalCellEndpointResult::ENDPOINT_MISMATCH);
}

TEST(GoalCellEndpoint, acceptsOnePoseSameCellPlan)
{
  EXPECT_EQ(
    checkGoalCellEndpoint("map", 1, 0.14, 0.14, "map", 0.05, 0.05),
    GoalCellEndpointResult::SAME_CELL_PLAN);
}

TEST(GoalCellEndpoint, skipsGeometryWhenFramesDiffer)
{
  EXPECT_EQ(
    checkGoalCellEndpoint("map", 2, 10.0, 10.0, "odom", 0.0, 0.0),
    GoalCellEndpointResult::DIFFERENT_FRAMES);
}

TEST(GoalCellEndpoint, rejectsNonFiniteCoordinates)
{
  EXPECT_EQ(
    checkGoalCellEndpoint("map", 2, std::nan(""), 0.0, "map", 0.0, 0.0),
    GoalCellEndpointResult::ENDPOINT_MISMATCH);
}

}  // namespace nav2_theta_star_planner
