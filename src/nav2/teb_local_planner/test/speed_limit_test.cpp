#include <gtest/gtest.h>

#include "teb_local_planner/teb_config.h"
#include "teb_local_planner/teb_local_planner_ros.h"

namespace
{

teb_local_planner::TebConfig::Robot make_robot()
{
  teb_local_planner::TebConfig::Robot robot;
  robot.base_max_vel_x = 0.35;
  robot.base_max_vel_x_backwards = 0.20;
  robot.base_max_vel_y = 0.10;
  robot.base_max_vel_theta = 0.30;
  robot.max_vel_x = robot.base_max_vel_x;
  robot.max_vel_x_backwards = robot.base_max_vel_x_backwards;
  robot.max_vel_y = robot.base_max_vel_y;
  robot.max_vel_theta = robot.base_max_vel_theta;
  return robot;
}

void expect_current_equals(
  const teb_local_planner::TebConfig::Robot & robot,
  double x, double backwards, double y, double theta)
{
  EXPECT_DOUBLE_EQ(robot.max_vel_x, x);
  EXPECT_DOUBLE_EQ(robot.max_vel_x_backwards, backwards);
  EXPECT_DOUBLE_EQ(robot.max_vel_y, y);
  EXPECT_DOUBLE_EQ(robot.max_vel_theta, theta);
}

}  // namespace

TEST(TebSpeedLimit, PercentageLimitPreservesBaselines)
{
  auto robot = make_robot();
  teb_local_planner::TebLocalPlannerROS::applySpeedLimit(robot, 50.0, true);

  expect_current_equals(robot, 0.175, 0.10, 0.05, 0.15);
  EXPECT_DOUBLE_EQ(robot.base_max_vel_x, 0.35);
  EXPECT_DOUBLE_EQ(robot.base_max_vel_x_backwards, 0.20);
  EXPECT_DOUBLE_EQ(robot.base_max_vel_y, 0.10);
  EXPECT_DOUBLE_EQ(robot.base_max_vel_theta, 0.30);
}

TEST(TebSpeedLimit, AbsoluteLimitScalesAllComponentsFromBaselines)
{
  auto robot = make_robot();
  teb_local_planner::TebLocalPlannerROS::applySpeedLimit(robot, 0.175, false);

  const double ratio = 0.175 / 0.35;
  expect_current_equals(robot, 0.35 * ratio, 0.20 * ratio, 0.10 * ratio, 0.30 * ratio);
  EXPECT_DOUBLE_EQ(robot.base_max_vel_x, 0.35);
  EXPECT_DOUBLE_EQ(robot.base_max_vel_theta, 0.30);
}

TEST(TebSpeedLimit, NoLimitRestoresAllCurrentComponents)
{
  auto robot = make_robot();
  teb_local_planner::TebLocalPlannerROS::applySpeedLimit(robot, 50.0, true);
  teb_local_planner::TebLocalPlannerROS::applySpeedLimit(
    robot, nav2_costmap_2d::NO_SPEED_LIMIT, false);

  expect_current_equals(robot, 0.35, 0.20, 0.10, 0.30);
}

TEST(TebSpeedLimit, HigherAbsoluteLimitRestoresBaselineAfterLowerLimit)
{
  auto robot = make_robot();
  teb_local_planner::TebLocalPlannerROS::applySpeedLimit(robot, 0.175, false);
  teb_local_planner::TebLocalPlannerROS::applySpeedLimit(robot, 0.50, false);

  expect_current_equals(robot, 0.35, 0.20, 0.10, 0.30);
}

TEST(TebSpeedLimit, BaselineAbsoluteLimitRestoresBaselineAfterLowerLimit)
{
  auto robot = make_robot();
  teb_local_planner::TebLocalPlannerROS::applySpeedLimit(robot, 0.175, false);
  teb_local_planner::TebLocalPlannerROS::applySpeedLimit(robot, 0.35, false);

  expect_current_equals(robot, 0.35, 0.20, 0.10, 0.30);
}

TEST(TebSpeedLimit, DynamicVelocityParameterUpdatesBaselineAndCurrent)
{
  teb_local_planner::TebConfig config;
  config.node_name = "FollowPath";
  const auto result = config.dynamicParametersCallback({
      rclcpp::Parameter("FollowPath.max_vel_x", 0.40),
      rclcpp::Parameter("FollowPath.max_vel_x_backwards", 0.15),
      rclcpp::Parameter("FollowPath.max_vel_y", 0.0),
      rclcpp::Parameter("FollowPath.max_vel_theta", 0.25),
    });

  ASSERT_TRUE(result.successful);
  EXPECT_DOUBLE_EQ(config.robot.max_vel_x, 0.40);
  EXPECT_DOUBLE_EQ(config.robot.base_max_vel_x, 0.40);
  EXPECT_DOUBLE_EQ(config.robot.max_vel_x_backwards, 0.15);
  EXPECT_DOUBLE_EQ(config.robot.base_max_vel_x_backwards, 0.15);
  EXPECT_DOUBLE_EQ(config.robot.max_vel_y, 0.0);
  EXPECT_DOUBLE_EQ(config.robot.base_max_vel_y, 0.0);
  EXPECT_DOUBLE_EQ(config.robot.max_vel_theta, 0.25);
  EXPECT_DOUBLE_EQ(config.robot.base_max_vel_theta, 0.25);
}
