#include <gtest/gtest.h>

#include <initializer_list>
#include <limits>
#include <sstream>
#include <utility>

#include "goal_change_reset.hpp"
#include "teb_local_planner/g2o_types/vertex_timediff.h"
#include "teb_local_planner/numerical_safety.hpp"
#include "teb_local_planner/path_change_tracker.h"

namespace
{

class RecordingPlanner : public teb_local_planner::PlannerInterface
{
public:
  bool plan(
    const std::vector<geometry_msgs::msg::PoseStamped> &,
    const geometry_msgs::msg::Twist *, bool) override
  {
    return true;
  }

  bool plan(
    const teb_local_planner::PoseSE2 &, const teb_local_planner::PoseSE2 &,
    const geometry_msgs::msg::Twist *, bool) override
  {
    return true;
  }

  bool getVelocityCommand(double &, double &, double &, int) const override
  {
    return true;
  }

  void clearPlanner() override
  {
    ++clear_calls;
  }

  void setPreferredTurningDir(teb_local_planner::RotType dir) override
  {
    preferred_rotdir = dir;
  }

  void setVisualization(const teb_local_planner::TebVisualizationPtr &) override {}

  bool isTrajectoryFeasible(
    dwb_critics::ObstacleFootprintCritic *,
    const std::vector<geometry_msgs::msg::Point> &, double, double, int, double,
    FeasibilityFailure *) override
  {
    return true;
  }

  bool hasDiverged() const override
  {
    return false;
  }

  int clear_calls{0};
  teb_local_planner::RotType preferred_rotdir{teb_local_planner::RotType::left};
};

}  // namespace

geometry_msgs::msg::PoseStamped path_pose(double x, double y)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.orientation.w = 1.0;
  return pose;
}

nav_msgs::msg::Path path(std::initializer_list<std::pair<double, double>> points)
{
  nav_msgs::msg::Path result;
  for (const auto & point : points) {
    result.poses.push_back(path_pose(point.first, point.second));
  }
  return result;
}

TEST(PathChangeTracker, preservesWarmStartForSameRouteAndPassedPathPrefix)
{
  teb_local_planner::PathChangeTracker tracker;
  const auto original = path({{0.0, 0.0}, {1.0, 0.0}, {2.0, 0.0}});
  EXPECT_FALSE(tracker.update(original));
  EXPECT_FALSE(tracker.update(path({{0.4, 0.0}, {1.0, 0.0}, {1.6, 0.0}, {2.0, 0.0}})));
}

TEST(PathChangeTracker, detectsSignificantRouteChangeWithSameEndpoint)
{
  teb_local_planner::PathChangeTracker tracker;
  EXPECT_FALSE(tracker.update(path({{0.0, 0.0}, {1.0, 0.0}, {2.0, 0.0}})));
  EXPECT_TRUE(tracker.update(path({{0.0, 0.0}, {1.0, 0.25}, {2.0, 0.0}})));
  EXPECT_FALSE(tracker.update(path({{0.0, 0.0}, {1.0, 0.25}, {2.0, 0.0}})));
}

TEST(PathChangeTracker, detectsEndpointChangeAndResetsOnEmptyPath)
{
  teb_local_planner::PathChangeTracker tracker;
  EXPECT_FALSE(tracker.update(path({{0.0, 0.0}, {1.0, 0.0}})));
  EXPECT_TRUE(tracker.update(path({{0.0, 0.0}, {1.25, 0.0}})));

  EXPECT_FALSE(tracker.update(nav_msgs::msg::Path()));
  EXPECT_FALSE(tracker.update(path({{0.0, 0.0}, {1.25, 0.0}})));
}

TEST(FailureDetector, clearDropsOscillationHistoryBetweenGoals)
{
  teb_local_planner::FailureDetector detector;
  detector.setBufferLength(10);

  geometry_msgs::msg::Twist command;
  for (int i = 0; i < 6; ++i) {
    command.angular.z = (i % 2 == 0) ? 0.5 : -0.5;
    detector.update(command, 0.5, 0.2, 0.5, 0.1, 0.1);
  }
  ASSERT_TRUE(detector.isOscillating());

  detector.clear();
  EXPECT_FALSE(detector.isOscillating());

  command.angular.z = 0.5;
  detector.update(command, 0.5, 0.2, 0.5, 0.1, 0.1);
  EXPECT_FALSE(detector.isOscillating());
}

TEST(VertexTimeDiff, rejectsNonPositiveAndNonFiniteUpdates)
{
  teb_local_planner::VertexTimeDiff timediff(0.0);
  EXPECT_DOUBLE_EQ(timediff.estimate(), 1e-3);

  const double negative_update = -1.0;
  timediff.oplusImpl(&negative_update);
  EXPECT_DOUBLE_EQ(timediff.estimate(), 1e-3);

  const double nan_update = std::numeric_limits<double>::quiet_NaN();
  timediff.oplusImpl(&nan_update);
  EXPECT_DOUBLE_EQ(timediff.estimate(), 1e-3);

  const double positive_update = 0.25;
  timediff.oplusImpl(&positive_update);
  EXPECT_DOUBLE_EQ(timediff.estimate(), 0.251);

  const double infinity_update = std::numeric_limits<double>::infinity();
  timediff.oplusImpl(&infinity_update);
  EXPECT_DOUBLE_EQ(timediff.estimate(), 1e-3);

  const double negative_infinity_update = -std::numeric_limits<double>::infinity();
  timediff.oplusImpl(&negative_infinity_update);
  EXPECT_DOUBLE_EQ(timediff.estimate(), 1e-3);
}

TEST(VertexTimeDiff, validatesStreamBeforeSanitizing)
{
  teb_local_planner::VertexTimeDiff timediff(0.25);

  std::istringstream empty_stream;
  EXPECT_FALSE(timediff.read(empty_stream));
  EXPECT_DOUBLE_EQ(timediff.estimate(), 0.25);

  std::istringstream invalid_stream("not-a-time-diff");
  EXPECT_FALSE(timediff.read(invalid_stream));
  EXPECT_DOUBLE_EQ(timediff.estimate(), 0.25);

  std::istringstream valid_stream("0.4");
  EXPECT_TRUE(timediff.read(valid_stream));
  EXPECT_DOUBLE_EQ(timediff.estimate(), 0.4);
}

TEST(NumericalSafety, rejectsNonFiniteVelocityCommands)
{
  EXPECT_TRUE(teb_local_planner::detail::finite_velocity_command(0.1, 0.0, -0.2));
  EXPECT_FALSE(teb_local_planner::detail::finite_velocity_command(
    std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0));
  EXPECT_FALSE(teb_local_planner::detail::finite_velocity_command(
    0.0, std::numeric_limits<double>::infinity(), 0.0));
  EXPECT_FALSE(teb_local_planner::detail::finite_velocity_command(
    0.0, 0.0, -std::numeric_limits<double>::infinity()));
}

TEST(NumericalSafety, rejectsTimeDiffSafetyFloor)
{
  const double minimum = teb_local_planner::detail::minimum_time_diff();
  EXPECT_FALSE(teb_local_planner::detail::valid_time_diff(0.0));
  EXPECT_FALSE(teb_local_planner::detail::valid_time_diff(-0.1));
  EXPECT_FALSE(teb_local_planner::detail::valid_time_diff(minimum));
  EXPECT_FALSE(teb_local_planner::detail::valid_time_diff(
    std::numeric_limits<double>::quiet_NaN()));
  EXPECT_FALSE(teb_local_planner::detail::valid_time_diff(
    std::numeric_limits<double>::infinity()));
  EXPECT_TRUE(teb_local_planner::detail::valid_time_diff(minimum + 1e-6));
}

TEST(GoalChangeReset, clearsPlannerDirectionAndRecoveryState)
{
  RecordingPlanner planner;
  teb_local_planner::FailureDetector detector;
  detector.setBufferLength(10);
  geometry_msgs::msg::Twist command;
  for (int i = 0; i < 6; ++i) {
    command.angular.z = (i % 2 == 0) ? 0.5 : -0.5;
    detector.update(command, 0.5, 0.2, 0.5, 0.1, 0.1);
  }
  ASSERT_TRUE(detector.isOscillating());

  auto preferred_rotdir = teb_local_planner::RotType::right;
  int infeasible_plans = 3;
  command.linear.x = 0.2;
  teb_local_planner::detail::resetForGoalChange(
    planner, detector, preferred_rotdir, infeasible_plans, command);

  EXPECT_EQ(planner.clear_calls, 1);
  EXPECT_EQ(planner.preferred_rotdir, teb_local_planner::RotType::none);
  EXPECT_FALSE(detector.isOscillating());
  EXPECT_EQ(preferred_rotdir, teb_local_planner::RotType::none);
  EXPECT_EQ(infeasible_plans, 0);
  EXPECT_EQ(command.linear.x, 0.0);
  EXPECT_EQ(command.angular.z, 0.0);
}
