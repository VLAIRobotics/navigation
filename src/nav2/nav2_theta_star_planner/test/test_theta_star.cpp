//  Copyright 2020 Anshumaan Singh
//
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//
//  http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.

#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>
#include <memory>
#include <set>
#include <utility>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "tf2/utils.h"
#include "nav2_costmap_2d/footprint.hpp"
#include "nav2_theta_star_planner/theta_star.hpp"
#include "nav2_theta_star_planner/theta_star_planner.hpp"

class init_rclcpp
{
public:
  init_rclcpp() {rclcpp::init(0, nullptr);}
  ~init_rclcpp() {rclcpp::shutdown();}
};

/// class created to access the protected members of the ThetaStar class
/// u is used as shorthand for use
class test_theta_star : public theta_star::ThetaStar
{
public:
  int getSizeOfNodePosition()
  {
    return static_cast<int>(node_position_.size());
  }

  bool ulosCheck(const int & x0, const int & y0, const int & x1, const int & y1, double & sl_cost)
  {
    return losCheck(x0, y0, x1, y1, sl_cost);
  }

  bool uwithinLimits(const int & cx, const int & cy) {return withinLimits(cx, cy);}

  bool uisSafeToExpand(int from_x, int from_y, int to_x, int to_y)
  {
    return isSafeToExpand(from_x, from_y, to_x, to_y);
  }

  bool uisStartExitCell(int x, int y) const {return isStartExitCell(x, y);}

  bool uisGoal(const tree_node & this_node) {return isGoal(this_node);}

  void uinitializePosn(int size_inc = 0)
  {
    node_position_.reserve(size_x_ * size_y_);
    nodes_data_.reserve(size_x_ * size_y_);
    initializePosn(size_inc);
  }

  void uaddIndex(const int & cx, const int & cy) {addIndex(cx, cy, &nodes_data_[0]);}

  tree_node * ugetIndex(const int & cx, const int & cy) {return getIndex(cx, cy);}

  tree_node * test_getIndex() {return &nodes_data_[0];}

  void uaddToNodesData(const int & id) {addToNodesData(id);}

  void uresetContainers() {nodes_data_.clear(); resetContainers();}

  bool runAlgo(std::vector<coordsW> & path)
  {
    if (!isUnsafeToPlan()) {
      return generatePath(path);
    }
    return false;
  }
};

class test_theta_star_planner : public nav2_theta_star_planner::ThetaStarPlanner
{
public:
  using nav2_theta_star_planner::ThetaStarPlanner::linearInterpolation;
  using nav2_theta_star_planner::ThetaStarPlanner::applyPathHysteresis;
  using nav2_theta_star_planner::ThetaStarPlanner::path_hysteresis_ratio_;
  using nav2_theta_star_planner::ThetaStarPlanner::path_hysteresis_max_rejoin_distance_;
  using nav2_theta_star_planner::ThetaStarPlanner::path_hysteresis_max_length_ratio_;
  using nav2_theta_star_planner::ThetaStarPlanner::path_hysteresis_min_clearance_gain_;
  using nav2_theta_star_planner::ThetaStarPlanner::path_hysteresis_confirmations_;
  using nav2_theta_star_planner::ThetaStarPlanner::pending_better_candidate_count_;
  using nav2_theta_star_planner::ThetaStarPlanner::have_cached_path_;
  using nav2_theta_star_planner::ThetaStarPlanner::cached_path_;
  using nav2_theta_star_planner::ThetaStarPlanner::current_goal_pose_;
  using nav2_theta_star_planner::ThetaStarPlanner::use_final_approach_orientation_;
  using nav2_theta_star_planner::ThetaStarPlanner::pathTraversable;
  using nav2_theta_star_planner::ThetaStarPlanner::buildObstacleDistanceField;
  using nav2_theta_star_planner::ThetaStarPlanner::obstacleGapAtCell;
  using nav2_theta_star_planner::ThetaStarPlanner::measureExclusivePath;
  using PathMetrics = nav2_theta_star_planner::ThetaStarPlanner::PathMetrics;
  using nav2_theta_star_planner::ThetaStarPlanner::planner_;
};

geometry_msgs::msg::PoseStamped make_pose(double x, double y)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.orientation.w = 1.0;
  return pose;
}

/// Straight path along y, from x0 to x1 inclusive, spaced 0.1 m apart
nav_msgs::msg::Path straight_path(double x0, double x1, double y)
{
  nav_msgs::msg::Path path;
  for (int i = 0; i <= static_cast<int>(std::lround((x1 - x0) / 0.1)); ++i) {
    path.poses.push_back(make_pose(x0 + i * 0.1, y));
  }
  return path;
}

geometry_msgs::msg::PoseStamped make_cell_pose(
  nav2_costmap_2d::Costmap2D * costmap, unsigned int mx, unsigned int my)
{
  geometry_msgs::msg::PoseStamped pose;
  costmap->mapToWorld(mx, my, pose.pose.position.x, pose.pose.position.y);
  pose.pose.orientation.w = 1.0;
  return pose;
}

init_rclcpp node;

// Tests meant to test the algorithm itself and its helper functions
TEST(ThetaStarTest, test_theta_star) {
  auto planner_ = std::make_unique<test_theta_star>();
  planner_->costmap_ = new nav2_costmap_2d::Costmap2D(50, 50, 1.0, 0.0, 0.0, 0);
  for (int i = 7; i <= 14; i++) {
    for (int j = 7; j <= 14; j++) {
      planner_->costmap_->setCost(i, j, 253);
    }
  }
  coordsM s = {5, 5}, g = {18, 18};
  int size_x = 20, size_y = 20;
  planner_->size_x_ = size_x;
  planner_->size_y_ = size_y;
  geometry_msgs::msg::PoseStamped start, goal;
  start.pose.position.x = s.x;
  start.pose.position.y = s.y;
  start.pose.orientation.w = 1.0;
  goal.pose.position.x = g.x;
  goal.pose.position.y = g.y;
  goal.pose.orientation.w = 1.0;
  /// Check if the setStartAndGoal function works properly
  planner_->setStartAndGoal(start, goal);
  EXPECT_TRUE(planner_->src_.x == s.x && planner_->src_.y == s.y);
  EXPECT_TRUE(planner_->dst_.x == g.x && planner_->dst_.y == g.y);
  /// Check if the initializePosn function works properly
  planner_->uinitializePosn(size_x * size_y);
  EXPECT_EQ(planner_->getSizeOfNodePosition(), (size_x * size_y));

  /// Check if the withinLimits function works properly
  EXPECT_TRUE(planner_->uwithinLimits(18, 18));
  EXPECT_FALSE(planner_->uwithinLimits(120, 140));

  tree_node n = {g.x, g.y, 120, 0, NULL, false, 20};
  n.parent_id = &n;
  /// Check if the isGoal function works properly
  EXPECT_TRUE(planner_->uisGoal(n));           // both (x,y) are the goal coordinates
  n.x = 25;
  EXPECT_FALSE(planner_->uisGoal(n));          // only y coordinate matches with that of goal
  n.x = g.x;
  n.y = 20;
  EXPECT_FALSE(planner_->uisGoal(n));          // only x coordinate matches with that of goal
  n.x = 30;
  EXPECT_FALSE(planner_->uisGoal(n));          // both (x, y) are different from the goal coordinate

  /// Check if the isSafe functions work properly
  EXPECT_TRUE(planner_->isSafe(5, 5));         // cost at this point is 0
  EXPECT_FALSE(planner_->isSafe(10, 10));      // cost at this point is 253 (>LETHAL_COST)

  /// Check if the functions addIndex & getIndex work properly
  coordsM c = {18, 18};
  planner_->uaddToNodesData(0);
  planner_->uaddIndex(c.x, c.y);
  tree_node * c_node = planner_->ugetIndex(c.x, c.y);
  EXPECT_EQ(c_node, planner_->test_getIndex());

  double sl_cost = 0.0;
  /// Checking for the case where the losCheck should return the value as true
  EXPECT_TRUE(planner_->ulosCheck(2, 2, 7, 20, sl_cost));
  /// and as false
  EXPECT_FALSE(planner_->ulosCheck(2, 2, 18, 18, sl_cost));

  planner_->uresetContainers();
  std::vector<coordsW> path;
  /// Check if the planner returns a path for the case where a path exists
  EXPECT_TRUE(planner_->runAlgo(path));
  EXPECT_GT(static_cast<int>(path.size()), 0);
  /// and where it doesn't exist
  path.clear();
  planner_->src_ = {10, 10};
  EXPECT_FALSE(planner_->runAlgo(path));
  EXPECT_EQ(static_cast<int>(path.size()), 0);
}

TEST(ThetaStarTest, diagonal_checks_both_side_cells)
{
  auto planner = std::make_unique<test_theta_star>();
  planner->costmap_ = new nav2_costmap_2d::Costmap2D(5, 5, 1.0, 0.0, 0.0, 0);
  planner->dst_ = {4, 4};
  planner->costmap_->setCost(2, 1, nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_FALSE(planner->uisSafeToExpand(1, 1, 2, 2));
  double cost = 0.0;
  EXPECT_FALSE(planner->ulosCheck(1, 1, 2, 2, cost));
  planner->costmap_->setCost(2, 1, 0);
  planner->costmap_->setCost(1, 2, nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_FALSE(planner->uisSafeToExpand(1, 1, 2, 2));
  EXPECT_FALSE(planner->ulosCheck(1, 1, 2, 2, cost));
}

TEST(ThetaStarTest, world_segment_ending_at_grid_corner_stops_in_all_directions)
{
  auto planner = std::make_unique<test_theta_star>();
  planner->costmap_ = new nav2_costmap_2d::Costmap2D(5, 5, 0.1, 0.0, 0.0, 0);
  const double resolution = planner->costmap_->getResolution();
  const double corner_x = resolution;
  const double corner_y = resolution;
  const double offset = 0.5 * resolution;
  const std::vector<coordsW> starts{
    {corner_x - offset, corner_y - offset},
    {corner_x - offset, corner_y + offset},
    {corner_x + offset, corner_y - offset},
    {corner_x + offset, corner_y + offset}};
  unsigned int goal_x = 0, goal_y = 0;
  ASSERT_TRUE(planner->costmap_->worldToMap(corner_x, corner_y, goal_x, goal_y));
  planner->dst_ = {static_cast<int>(goal_x), static_cast<int>(goal_y)};

  for (const auto & start : starts) {
    std::set<std::pair<int, int>> visited;
    bool left_start_exit = false;
    theta_star::ThetaStar::Rejection rejection;
    EXPECT_TRUE(planner->traverseWorldSegment(
        start.x, start.y, corner_x, corner_y, left_start_exit, &rejection,
        [&visited](int x, int y) {visited.emplace(x, y);}))
      << "segment ending at a grid corner must stop at parameter t=1";
    EXPECT_EQ(visited.size(), 3U);
  }
}

TEST(ThetaStarTest, world_segment_ending_on_grid_line_stops_in_all_axis_directions)
{
  auto planner = std::make_unique<test_theta_star>();
  planner->costmap_ = new nav2_costmap_2d::Costmap2D(5, 5, 0.1, 0.0, 0.0, 0);
  const double resolution = planner->costmap_->getResolution();
  const double line = 2.0 * resolution;
  const double middle = 1.5 * resolution;
  struct SegmentCase
  {
    coordsW start;
    coordsW end;
    int touched_cell_x;
    int touched_cell_y;
  };
  const std::vector<SegmentCase> segments{
    {{line - 0.5 * resolution, middle}, {line, middle}, 2, 1},
    {{line + 0.5 * resolution, middle}, {line, middle}, 1, 1},
    {{middle, line - 0.5 * resolution}, {middle, line}, 1, 2},
    {{middle, line + 0.5 * resolution}, {middle, line}, 1, 1}};
  unsigned int goal_x = 0, goal_y = 0;

  for (const auto & segment : segments) {
    ASSERT_TRUE(planner->costmap_->worldToMap(
        segment.end.x, segment.end.y, goal_x, goal_y));
    planner->dst_ = {static_cast<int>(goal_x), static_cast<int>(goal_y)};
    bool left_start_exit = false;
    theta_star::ThetaStar::Rejection rejection;
    std::set<std::pair<int, int>> visited;
    EXPECT_TRUE(planner->traverseWorldSegment(
        segment.start.x, segment.start.y, segment.end.x, segment.end.y,
        left_start_exit, &rejection,
        [&visited](int x, int y) {visited.emplace(x, y);}))
      << "segment ending on a grid line must not step past its endpoint";
    EXPECT_EQ(visited.size(), 1U);
    EXPECT_EQ(visited.count({segment.touched_cell_x, segment.touched_cell_y}), 1U);
  }
}

TEST(ThetaStarTest, world_segment_rejects_obstacle_touching_endpoint_corner)
{
  auto planner = std::make_unique<test_theta_star>();
  planner->costmap_ = new nav2_costmap_2d::Costmap2D(5, 5, 0.1, 0.0, 0.0, 0);
  const double resolution = planner->costmap_->getResolution();
  const double corner = resolution;
  unsigned int goal_x = 0, goal_y = 0;
  ASSERT_TRUE(planner->costmap_->worldToMap(corner, corner, goal_x, goal_y));
  planner->dst_ = {static_cast<int>(goal_x), static_cast<int>(goal_y)};

  // This side cell only touches the closed segment at its endpoint vertex.
  planner->costmap_->setCost(0, 0, nav2_costmap_2d::LETHAL_OBSTACLE);
  bool left_start_exit = false;
  theta_star::ThetaStar::Rejection rejection;
  EXPECT_FALSE(planner->traverseWorldSegment(
      corner - 0.5 * resolution, corner + 0.5 * resolution, corner, corner,
      left_start_exit, &rejection, [](int, int) {}));
  ASSERT_TRUE(rejection.valid);
  EXPECT_STREQ(rejection.kind, "obstacle");
  EXPECT_EQ(rejection.x, 0);
  EXPECT_EQ(rejection.y, 0);
}

TEST(ThetaStarTest, endpoint_corner_keeps_inscribed_goal_exception)
{
  auto planner = std::make_unique<test_theta_star>();
  planner->costmap_ = new nav2_costmap_2d::Costmap2D(5, 5, 0.1, 0.0, 0.0, 0);
  const double resolution = planner->costmap_->getResolution();
  const double corner = resolution;
  unsigned int goal_x = 0, goal_y = 0;
  ASSERT_TRUE(planner->costmap_->worldToMap(corner, corner, goal_x, goal_y));
  planner->dst_ = {static_cast<int>(goal_x), static_cast<int>(goal_y)};
  planner->costmap_->setCost(goal_x, goal_y, nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);

  bool left_start_exit = false;
  EXPECT_TRUE(planner->traverseWorldSegment(
      corner - 0.5 * resolution, corner + 0.5 * resolution, corner, corner,
      left_start_exit, nullptr, [](int, int) {}));
}

TEST(ThetaStarTest, cost_252_is_soft_in_neighbor_and_los_checks)
{
  auto planner = std::make_unique<test_theta_star>();
  planner->costmap_ = new nav2_costmap_2d::Costmap2D(5, 5, 1.0, 0.0, 0.0, 0);
  planner->dst_ = {4, 4};
  planner->costmap_->setCost(2, 1, 252);
  planner->costmap_->setCost(2, 2, 252);

  EXPECT_TRUE(planner->uisSafeToExpand(1, 1, 2, 2));
  double line_cost = 0.0;
  EXPECT_TRUE(planner->ulosCheck(1, 1, 2, 2, line_cost));
}

TEST(ThetaStarTest, clearance_distance_adds_soft_route_cost_without_blocking)
{
  auto planner = std::make_unique<test_theta_star>();
  planner->costmap_ = new nav2_costmap_2d::Costmap2D(5, 5, 1.0, 0.0, 0.0, 0);
  planner->w_clearance_cost_ = 0.5;
  std::vector<float> distance_field(25, 1.0F);
  distance_field[1 * 5 + 1] = 0.1F;
  distance_field[3 * 5 + 3] = 0.8F;
  planner->setObstacleDistanceField(&distance_field, 5, 5, 1.0, 1.0);

  EXPECT_TRUE(planner->isSafe(1, 1));
  EXPECT_TRUE(planner->isSafe(3, 3));
  EXPECT_GT(planner->traversalCostAtCell(1, 1), planner->traversalCostAtCell(3, 3));
}

TEST(ThetaStarTest, corner_side_cells_use_state_before_the_step)
{
  auto planner = std::make_unique<test_theta_star>();
  planner->costmap_ = new nav2_costmap_2d::Costmap2D(4, 4, 1.0, 0.0, 0.0, 0);
  planner->dst_ = {3, 3};
  planner->start_exit_cells_.resize(16, 0);
  planner->start_exit_cells_[5] = 1;
  planner->start_exit_cells_[9] = 1;
  bool left = false;
  EXPECT_TRUE(planner->traverseCells(1, 1, 2, 2, left, nullptr, [](int, int) {}));
  EXPECT_TRUE(left);
}

TEST(ThetaStarTest, corner_touching_outside_start_escape_prevents_later_reentry)
{
  auto planner = std::make_unique<test_theta_star>();
  planner->costmap_ = new nav2_costmap_2d::Costmap2D(4, 4, 1.0, 0.0, 0.0, 0);
  planner->dst_ = {3, 3};
  planner->start_exit_cells_.resize(16, 0);
  planner->start_exit_cells_[5] = 1;
  planner->start_exit_cells_[10] = 1;
  planner->costmap_->setCost(1, 1, nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);

  bool left = false;
  EXPECT_TRUE(planner->traverseCells(1, 1, 2, 2, left, nullptr, [](int, int) {}));
  EXPECT_TRUE(left);

  theta_star::ThetaStar::Rejection rejection;
  EXPECT_FALSE(planner->traverseCells(
      2, 2, 1, 1, left, &rejection, [](int, int) {}));
  ASSERT_TRUE(rejection.valid);
  EXPECT_STREQ(rejection.kind, "start_escape");
}

TEST(ThetaStarTest, corner_side_outside_escape_blocks_inscribed_side_reentry)
{
  auto planner = std::make_unique<test_theta_star>();
  planner->costmap_ = new nav2_costmap_2d::Costmap2D(4, 4, 1.0, 0.0, 0.0, 0);
  planner->dst_ = {3, 3};
  planner->start_exit_cells_.resize(16, 0);
  planner->start_exit_cells_[5] = 1;
  planner->start_exit_cells_[9] = 1;
  planner->start_exit_cells_[10] = 1;
  planner->costmap_->setCost(1, 1, nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
  planner->costmap_->setCost(1, 2, nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);

  bool left = false;
  theta_star::ThetaStar::Rejection rejection;
  EXPECT_FALSE(planner->traverseCells(
      1, 1, 2, 2, left, &rejection, [](int, int) {}));
  ASSERT_TRUE(rejection.valid);
  EXPECT_STREQ(rejection.kind, "start_escape");
  EXPECT_EQ(rejection.x, 1);
  EXPECT_EQ(rejection.y, 2);
}

TEST(ThetaStarTest, rejects_reentry_into_start_escape_region)
{
  auto planner = std::make_unique<test_theta_star>();
  planner->costmap_ = new nav2_costmap_2d::Costmap2D(5, 3, 1.0, 0.0, 0.0, 0);
  planner->dst_ = {4, 1};
  planner->start_exit_cells_.resize(15, 0);
  planner->start_exit_cells_[5] = 1;
  planner->start_exit_cells_[7] = 1;
  planner->costmap_->setCost(2, 1, nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
  bool left = false;
  theta_star::ThetaStar::Rejection rejection;
  EXPECT_FALSE(planner->traverseCells(0, 1, 2, 1, left, &rejection, [](int, int) {}));
  EXPECT_TRUE(rejection.valid);
  EXPECT_STREQ(rejection.kind, "start_escape");
  EXPECT_EQ(rejection.x, 2);
  EXPECT_EQ(rejection.y, 1);
}

TEST(ThetaStarTest, reaches_goal_when_it_is_last_queued_node)
{
  auto planner = std::make_unique<test_theta_star>();
  planner->costmap_ = new nav2_costmap_2d::Costmap2D(2, 1, 1.0, 0.0, 0.0, 0);
  planner->size_x_ = 2;
  planner->size_y_ = 1;
  auto start = make_pose(0.5, 0.5);
  auto goal = make_pose(1.5, 0.5);
  planner->setStartAndGoal(start, goal);
  planner->uinitializePosn(2);

  std::vector<coordsW> path;
  ASSERT_TRUE(planner->runAlgo(path));
  ASSERT_FALSE(path.empty());
  EXPECT_NEAR(path.back().x, goal.pose.position.x, 1e-9);
  EXPECT_NEAR(path.back().y, goal.pose.position.y, 1e-9);
}

TEST(ThetaStarTest, allows_starting_inside_inscribed_inflation_and_leaving)
{
  auto planner = std::make_unique<test_theta_star>();
  planner->costmap_ = new nav2_costmap_2d::Costmap2D(30, 30, 1.0, 0.0, 0.0, 0);
  planner->size_x_ = 30;
  planner->size_y_ = 30;
  planner->allow_unknown_ = false;
  for (unsigned int x = 4; x <= 6; ++x) {
    for (unsigned int y = 4; y <= 6; ++y) {
      planner->costmap_->setCost(
        x, y, nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
    }
  }

  geometry_msgs::msg::PoseStamped start;
  start.pose.position.x = 5.5;
  start.pose.position.y = 5.5;
  start.pose.orientation.w = 1.0;
  auto goal = start;
  goal.pose.position.x = 20.5;
  goal.pose.position.y = 20.5;
  planner->setStartAndGoal(start, goal);
  planner->uinitializePosn(30 * 30);
  planner->costmap_->setCost(10, 10, nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);

  std::vector<coordsW> path;
  EXPECT_FALSE(planner->isUnsafeToPlan());
  EXPECT_FALSE(planner->isSafe(4, 4));
  EXPECT_TRUE(planner->uisSafeToExpand(5, 5, 4, 4));
  EXPECT_FALSE(planner->uisSafeToExpand(9, 9, 4, 4));
  EXPECT_TRUE(planner->uisSafeToExpand(4, 4, 3, 4));
  double line_cost = 0.0;
  EXPECT_FALSE(planner->ulosCheck(9, 5, 5, 5, line_cost));
  EXPECT_TRUE(planner->ulosCheck(5, 5, 8, 5, line_cost));
  EXPECT_FALSE(planner->isSafe(10, 10));
  ASSERT_TRUE(planner->runAlgo(path));
  ASSERT_FALSE(path.empty());
  EXPECT_NEAR(path.front().x, start.pose.position.x, 1e-9);
  EXPECT_NEAR(path.front().y, start.pose.position.y, 1e-9);
  bool left_start_exit = false;
  for (const auto & point : path) {
    unsigned int map_x, map_y;
    ASSERT_TRUE(planner->costmap_->worldToMap(point.x, point.y, map_x, map_y));
    const bool in_start_exit = planner->uisStartExitCell(map_x, map_y);
    EXPECT_FALSE(in_start_exit && left_start_exit);
    left_start_exit = left_start_exit || !in_start_exit;
  }
}

TEST(ThetaStarTest, allows_bounded_unknown_start_escape_only)
{
  auto planner = std::make_unique<test_theta_star>();
  planner->costmap_ = new nav2_costmap_2d::Costmap2D(30, 30, 1.0, 0.0, 0.0, 0);
  planner->size_x_ = 30;
  planner->size_y_ = 30;
  planner->allow_unknown_ = false;
  planner->start_unknown_escape_radius_ = 2.5;
  planner->costmap_->setCost(5, 5, nav2_costmap_2d::NO_INFORMATION);
  planner->costmap_->setCost(6, 5, nav2_costmap_2d::NO_INFORMATION);
  planner->costmap_->setCost(9, 5, nav2_costmap_2d::NO_INFORMATION);

  auto start = make_pose(5.5, 5.5);
  auto goal = make_pose(20.5, 20.5);
  planner->setStartAndGoal(start, goal);
  planner->uinitializePosn(30 * 30);

  EXPECT_FALSE(planner->isUnsafeToPlan());
  EXPECT_TRUE(planner->uisSafeToExpand(5, 5, 6, 5));
  EXPECT_TRUE(planner->uisSafeToExpand(6, 5, 7, 5));
  EXPECT_FALSE(planner->uisSafeToExpand(8, 5, 9, 5));

  std::vector<coordsW> path;
  ASSERT_TRUE(planner->runAlgo(path));
  ASSERT_FALSE(path.empty());
  EXPECT_NEAR(path.front().x, start.pose.position.x, 1e-9);
}

TEST(ThetaStarTest, allows_isolated_unknown_cells_inside_geometric_start_disk)
{
  auto planner = std::make_unique<test_theta_star>();
  planner->costmap_ = new nav2_costmap_2d::Costmap2D(20, 20, 0.1, 0.0, 0.0, 0);
  planner->size_x_ = 20;
  planner->size_y_ = 20;
  planner->allow_unknown_ = false;
  planner->start_unknown_escape_radius_ = 0.25;
  planner->costmap_->setCost(5, 5, nav2_costmap_2d::NO_INFORMATION);
  planner->costmap_->setCost(7, 5, nav2_costmap_2d::NO_INFORMATION);
  planner->costmap_->setCost(9, 5, nav2_costmap_2d::NO_INFORMATION);
  planner->costmap_->setCost(6, 6, nav2_costmap_2d::LETHAL_OBSTACLE);

  const auto start = make_pose(0.55, 0.55);
  const auto goal = make_pose(1.55, 1.55);
  planner->setStartAndGoal(start, goal);

  EXPECT_FALSE(planner->isUnsafeToPlan());
  EXPECT_TRUE(planner->uisStartExitCell(7, 5));
  EXPECT_TRUE(planner->uisSafeToExpand(6, 5, 7, 5));
  EXPECT_FALSE(planner->uisStartExitCell(9, 5));
  EXPECT_FALSE(planner->uisSafeToExpand(7, 5, 9, 5));
  EXPECT_FALSE(planner->uisSafeToExpand(5, 5, 6, 6));
}

TEST(ThetaStarTest, softInflatedStartNeedsUnknownUnderCurrentFootprint)
{
  auto planner = std::make_unique<test_theta_star>();
  planner->costmap_ = new nav2_costmap_2d::Costmap2D(20, 20, 0.1, 0.0, 0.0, 0);
  planner->size_x_ = 20;
  planner->size_y_ = 20;
  planner->allow_unknown_ = false;
  planner->start_unknown_escape_radius_ = 0.35;
  planner->costmap_->setCost(5, 5, 100);
  planner->costmap_->setCost(8, 5, nav2_costmap_2d::NO_INFORMATION);
  const auto make_point = [](double x, double y) {
      geometry_msgs::msg::Point point;
      point.x = x;
      point.y = y;
      return point;
    };
  planner->robot_footprint_ = {
    make_point(0.15, 0.15), make_point(0.15, -0.15),
    make_point(-0.15, -0.15), make_point(-0.15, 0.15)};

  planner->setStartAndGoal(make_pose(0.55, 0.55), make_pose(1.55, 1.55));

  EXPECT_FALSE(planner->uisStartExitCell(8, 5));
  EXPECT_FALSE(planner->uisSafeToExpand(5, 5, 8, 5));
}

TEST(ThetaStarTest, softInflatedStartEscapesWhenCurrentFootprintCoversUnknown)
{
  auto planner = std::make_unique<test_theta_star>();
  planner->costmap_ = new nav2_costmap_2d::Costmap2D(20, 20, 0.1, 0.0, 0.0, 0);
  planner->size_x_ = 20;
  planner->size_y_ = 20;
  planner->allow_unknown_ = false;
  planner->start_unknown_escape_radius_ = 0.35;
  planner->costmap_->setCost(5, 5, 100);
  planner->costmap_->setCost(6, 5, nav2_costmap_2d::NO_INFORMATION);
  const auto make_point = [](double x, double y) {
      geometry_msgs::msg::Point point;
      point.x = x;
      point.y = y;
      return point;
    };
  planner->robot_footprint_ = {
    make_point(0.15, 0.15), make_point(0.15, -0.15),
    make_point(-0.15, -0.15), make_point(-0.15, 0.15)};

  planner->setStartAndGoal(make_pose(0.55, 0.55), make_pose(1.55, 1.55));

  EXPECT_TRUE(planner->uisStartExitCell(6, 5));
  EXPECT_TRUE(planner->uisSafeToExpand(5, 5, 6, 5));
}

TEST(ThetaStarTest, unknown_start_is_rejected_without_escape_radius)
{
  auto planner = std::make_unique<test_theta_star>();
  planner->costmap_ = new nav2_costmap_2d::Costmap2D(10, 10, 1.0, 0.0, 0.0, 0);
  planner->size_x_ = 10;
  planner->size_y_ = 10;
  planner->allow_unknown_ = false;
  planner->costmap_->setCost(1, 1, nav2_costmap_2d::NO_INFORMATION);
  planner->setStartAndGoal(make_pose(1.5, 1.5), make_pose(8.5, 8.5));
  EXPECT_TRUE(planner->isUnsafeToPlan());
}

TEST(ThetaStarTest, rejects_lethal_start_and_occupied_goal)
{
  auto planner = std::make_unique<test_theta_star>();
  planner->costmap_ = new nav2_costmap_2d::Costmap2D(30, 30, 1.0, 0.0, 0.0, 0);
  planner->size_x_ = 30;
  planner->size_y_ = 30;
  planner->allow_unknown_ = false;

  geometry_msgs::msg::PoseStamped start;
  start.pose.position.x = 5.0;
  start.pose.position.y = 5.0;
  start.pose.orientation.w = 1.0;
  auto goal = start;
  goal.pose.position.x = 20.0;
  goal.pose.position.y = 20.0;
  planner->setStartAndGoal(start, goal);

  planner->costmap_->setCost(5, 5, nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_TRUE(planner->isUnsafeToPlan());

  planner->costmap_->setCost(5, 5, 0);
  planner->costmap_->setCost(20, 20, nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
  EXPECT_FALSE(planner->isUnsafeToPlan());
  planner->costmap_->setCost(20, 20, nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_TRUE(planner->isUnsafeToPlan());
}

TEST(ThetaStarTest, robot_radius_inflation_routes_around_synthetic_obstacle) {
  constexpr unsigned int map_cells = 80;
  constexpr double resolution = 0.1;
  constexpr double robot_radius = 0.30;
  constexpr double inflation_radius = 0.30;
  constexpr double cost_scaling_factor = 8.0;

  std::vector<geometry_msgs::msg::Point> circle_footprint =
    nav2_costmap_2d::makeFootprintFromRadius(robot_radius);
  double inscribed_radius = 0.0;
  double circumscribed_radius = 0.0;
  nav2_costmap_2d::calculateMinAndMaxDistances(
    circle_footprint, inscribed_radius, circumscribed_radius);
  EXPECT_NEAR(circumscribed_radius, robot_radius, 1e-6);
  EXPECT_NEAR(inscribed_radius, robot_radius * std::cos(std::acos(-1.0) / 16.0), 1e-6);

  auto make_costmap = [ = ](double inflation_inner_radius) {
      auto costmap = std::make_unique<nav2_costmap_2d::Costmap2D>(
        map_cells, map_cells, resolution, -4.0, -4.0, 0);
      const std::pair<unsigned int, unsigned int> obstacle{40, 40};

      for (unsigned int x = 0; x < map_cells; ++x) {
        for (unsigned int y = 0; y < map_cells; ++y) {
          const double dx = static_cast<double>(x) - obstacle.first;
          const double dy = static_cast<double>(y) - obstacle.second;
          const double distance = std::hypot(dx, dy) * resolution;

          unsigned char cost = 0;
          if (distance == 0.0) {
            cost = nav2_costmap_2d::LETHAL_OBSTACLE;
          } else if (distance <= inflation_inner_radius) {
            cost = nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
          } else if (distance <= inflation_radius) {
            const double factor = std::exp(
              -cost_scaling_factor * (distance - inflation_inner_radius));
            cost = static_cast<unsigned char>(
              (nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE - 1) * factor);
          }
          costmap->setCost(x, y, cost);
        }
      }
      return costmap;
    };

  auto make_planner = [ = ](nav2_costmap_2d::Costmap2D * costmap) {
      auto planner = std::make_unique<test_theta_star>();
      planner->costmap_ = costmap;
      planner->size_x_ = 40;
      planner->size_y_ = 40;
      planner->allow_unknown_ = false;
      geometry_msgs::msg::PoseStamped start;
      start.pose.position.x = -2.0;
      start.pose.position.y = 0.0;
      start.pose.orientation.w = 1.0;
      geometry_msgs::msg::PoseStamped goal = start;
      goal.pose.position.x = 2.0;
      planner->setStartAndGoal(start, goal);
      planner->uinitializePosn(40 * 40);
      return planner;
    };

  auto old_costmap = make_costmap(0.215);
  auto old_planner = make_planner(old_costmap.get());
  std::vector<coordsW> path;
  EXPECT_LE(old_costmap->getCost(43, 40), 252);
  EXPECT_TRUE(old_planner->isSafe(43, 40));

  auto inflated_costmap = make_costmap(inscribed_radius);
  auto inflated_planner = make_planner(inflated_costmap.get());
  path.clear();
  EXPECT_EQ(inflated_costmap->getCost(42, 40), nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
  ASSERT_TRUE(inflated_planner->runAlgo(path));
  ASSERT_FALSE(path.empty());
  bool path_clears_the_obstacle = false;
  for (const auto & point : path) {
    const double distance = std::hypot(point.x - 0.05, point.y - 0.05);
    EXPECT_GT(distance, inscribed_radius);
    path_clears_the_obstacle = path_clears_the_obstacle ||
      std::abs(point.y - 0.05) > inscribed_radius;
  }
  EXPECT_TRUE(path_clears_the_obstacle);
}

TEST(ThetaStarPlanner, interpolation_sets_path_orientation_and_endpoint) {
  std::vector<coordsW> raw_path{{0.0, 0.0}, {2.0, 0.0}, {2.0, 2.0}};
  const auto path = test_theta_star_planner::linearInterpolation(raw_path, 1.0);

  ASSERT_EQ(path.poses.size(), 5u);
  EXPECT_NEAR(path.poses.front().pose.position.x, 0.0, 1e-9);
  EXPECT_NEAR(path.poses.back().pose.position.x, 2.0, 1e-9);
  EXPECT_NEAR(path.poses.back().pose.position.y, 2.0, 1e-9);
  EXPECT_NEAR(tf2::getYaw(path.poses[0].pose.orientation), 0.0, 1e-9);
  EXPECT_NEAR(tf2::getYaw(path.poses[2].pose.orientation), M_PI_2, 1e-9);
}

class PathHysteresisTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    life_node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>("ThetaStarHysteresisTest");
    costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>("global_costmap");
    costmap_ros_->on_configure(rclcpp_lifecycle::State());
    plugin_ = std::make_unique<test_theta_star_planner>();
    plugin_->configure(life_node_, "test", nullptr, costmap_ros_);
    plugin_->activate();
  }

  void TearDown() override
  {
    plugin_->deactivate();
    plugin_->cleanup();
    plugin_.reset();
    costmap_ros_->on_cleanup(rclcpp_lifecycle::State());
    costmap_ros_.reset();
    life_node_.reset();
  }

  /// Stands in for the goal cell setStartAndGoal() would have resolved
  void setGoalCell(int x, int y)
  {
    plugin_->planner_->dst_ = coordsM{x, y};
    costmap_ros_->getCostmap()->mapToWorld(
      static_cast<unsigned int>(x), static_cast<unsigned int>(y),
      plugin_->current_goal_pose_.pose.position.x,
      plugin_->current_goal_pose_.pose.position.y);
    plugin_->current_goal_pose_.pose.orientation.w = 1.0;
  }

  /// Seeds the cache with `path`, as the first plan for the current goal cell would
  void seedCache(const nav_msgs::msg::Path & path)
  {
    auto adopted = path;
    plugin_->applyPathHysteresis(path.poses.front(), adopted);
    ASSERT_TRUE(plugin_->have_cached_path_);
  }

  void setCost(double x, double y, unsigned char cost)
  {
    unsigned int mx, my;
    ASSERT_TRUE(costmap_ros_->getCostmap()->worldToMap(x, y, mx, my));
    costmap_ros_->getCostmap()->setCost(mx, my, cost);
  }

  nav_msgs::msg::Path path_through_cells(
    const std::vector<std::pair<unsigned int, unsigned int>> & cells)
  {
    nav_msgs::msg::Path path;
    auto * costmap = costmap_ros_->getCostmap();
    for (const auto & cell : cells) {
      path.poses.push_back(make_cell_pose(costmap, cell.first, cell.second));
    }
    return path;
  }

  rclcpp_lifecycle::LifecycleNode::SharedPtr life_node_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::unique_ptr<test_theta_star_planner> plugin_;
};

TEST_F(PathHysteresisTest, keeps_previous_path_when_candidate_is_barely_shorter)
{
  setGoalCell(30, 20);
  seedCache(straight_path(1.0, 3.0, 2.0));

  // 1.9 m against the retained 2.0 m: inside the 15% margin, so the cache wins
  auto candidate = straight_path(1.0, 2.9, 2.0);
  plugin_->applyPathHysteresis(make_pose(1.0, 2.0), candidate);
  EXPECT_NEAR(candidate.poses.back().pose.position.x, 3.0, 1e-6);
}

TEST_F(PathHysteresisTest, cost_252_remains_traversable_in_final_path_validation)
{
  auto * costmap = costmap_ros_->getCostmap();
  setGoalCell(11, 11);
  costmap->setCost(11, 10, 252);
  costmap->setCost(11, 11, 252);
  const auto path = path_through_cells({{10, 10}, {11, 11}});
  const auto robot = make_cell_pose(costmap, 10, 10);

  EXPECT_TRUE(plugin_->pathTraversable(path, robot));
}

TEST_F(PathHysteresisTest, long_slanted_segment_matches_its_interpolated_traversal)
{
  auto * costmap = costmap_ros_->getCostmap();
  ASSERT_GT(costmap->getSizeInCellsX(), 65U);
  ASSERT_GT(costmap->getSizeInCellsY(), 45U);
  const auto start = make_cell_pose(costmap, 5, 20);
  const auto goal = make_cell_pose(costmap, 60, 42);
  setGoalCell(60, 42);
  const std::vector<coordsW> raw_path{{start.pose.position.x, start.pose.position.y},
    {goal.pose.position.x, goal.pose.position.y}};
  nav_msgs::msg::Path direct;
  direct.poses = {start, goal};
  const auto interpolated = test_theta_star_planner::linearInterpolation(
    raw_path, costmap->getResolution());

  std::set<std::pair<int, int>> direct_cells;
  std::set<std::pair<int, int>> interpolated_cells;
  bool direct_left = false;
  bool interpolated_left = false;
  ASSERT_TRUE(plugin_->planner_->traverseWorldSegment(
      start.pose.position.x, start.pose.position.y,
      goal.pose.position.x, goal.pose.position.y, direct_left, nullptr,
      [&direct_cells](int x, int y) {direct_cells.emplace(x, y);}));
  for (std::size_t i = 1; i < interpolated.poses.size(); ++i) {
    const auto & from = interpolated.poses[i - 1].pose.position;
    const auto & to = interpolated.poses[i].pose.position;
    ASSERT_TRUE(plugin_->planner_->traverseWorldSegment(
        from.x, from.y, to.x, to.y, interpolated_left, nullptr,
        [&interpolated_cells](int x, int y) {interpolated_cells.emplace(x, y);}));
  }
  EXPECT_EQ(direct_cells, interpolated_cells);
  ASSERT_FALSE(direct_cells.empty());
  const auto blocked_cell_it = std::find_if(
    direct_cells.begin(), direct_cells.end(), [](const auto & cell) {
      return cell.first != 5 && cell.second != 20 &&
             !(cell.first == 60 && cell.second == 42);
    });
  ASSERT_NE(blocked_cell_it, direct_cells.end());
  const auto blocked_cell = *blocked_cell_it;

  costmap->setCost(blocked_cell.first, blocked_cell.second, nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_FALSE(plugin_->pathTraversable(direct, start));
  EXPECT_FALSE(plugin_->pathTraversable(interpolated, start));
}

TEST_F(PathHysteresisTest, adopts_candidate_that_is_clearly_shorter)
{
  setGoalCell(30, 20);
  seedCache(straight_path(1.0, 3.0, 2.0));

  auto candidate = straight_path(1.0, 1.5, 2.0);
  plugin_->applyPathHysteresis(make_pose(1.0, 2.0), candidate);
  EXPECT_NEAR(candidate.poses.back().pose.position.x, 1.5, 1e-6);
}

TEST_F(PathHysteresisTest, adopts_candidate_when_robot_is_too_far_from_cached_path)
{
  setGoalCell(30, 20);
  seedCache(straight_path(1.0, 3.0, 2.0));

  // The fresh route is not 15% shorter, but a 0.25 m cross-track rejoin exceeds the
  // absolute 0.20 m cap and must not pull the robot back to the stale route.
  auto candidate = straight_path(1.0, 2.9, 2.0);
  plugin_->applyPathHysteresis(make_pose(1.5, 2.25), candidate);
  EXPECT_NEAR(candidate.poses.back().pose.position.x, 2.9, 1e-6);
}

TEST_F(PathHysteresisTest, drops_cache_when_goal_cell_changes)
{
  setGoalCell(30, 20);
  seedCache(straight_path(1.0, 3.0, 2.0));

  // One cell across, which is less than a full resolution of metric movement
  setGoalCell(31, 20);
  auto candidate = straight_path(1.0, 3.5, 2.0);
  plugin_->applyPathHysteresis(make_pose(1.0, 2.0), candidate);
  EXPECT_NEAR(candidate.poses.back().pose.position.x, 3.5, 1e-6);
}

TEST_F(PathHysteresisTest, keeps_cache_when_rolling_origin_changes)
{
  setGoalCell(30, 20);
  seedCache(straight_path(1.0, 3.0, 2.0));
  const auto world_goal = plugin_->current_goal_pose_;
  auto * costmap = costmap_ros_->getCostmap();
  costmap->updateOrigin(costmap->getOriginX() + 0.20, costmap->getOriginY());

  unsigned int goal_x, goal_y;
  ASSERT_TRUE(
    costmap->worldToMap(
      world_goal.pose.position.x, world_goal.pose.position.y, goal_x, goal_y));
  plugin_->planner_->dst_ = coordsM{static_cast<int>(goal_x), static_cast<int>(goal_y)};
  auto candidate = straight_path(1.0, 2.9, 2.0);
  plugin_->applyPathHysteresis(make_pose(1.0, 2.0), candidate);
  EXPECT_NEAR(candidate.poses.back().pose.position.x, 3.0, 1e-6);
}

TEST_F(PathHysteresisTest, switches_to_free_candidate_when_cached_route_has_soft_cost)
{
  setGoalCell(30, 20);
  auto cached = straight_path(1.0, 3.0, 2.0);
  seedCache(cached);
  for (const auto & pose : cached.poses) {
    setCost(pose.pose.position.x, pose.pose.position.y, 252);
  }

  nav_msgs::msg::Path candidate;
  candidate.poses = {
    make_pose(1.0, 2.0), make_pose(1.0, 3.0), make_pose(3.0, 3.0), make_pose(3.0, 2.0)};
  plugin_->applyPathHysteresis(make_pose(1.0, 2.0), candidate);
  EXPECT_NEAR(candidate.poses.back().pose.position.y, 2.0, 1e-6);
  EXPECT_EQ(candidate.poses.size(), 4u);
}

TEST_F(PathHysteresisTest, retained_route_recomputes_final_approach_orientation)
{
  setGoalCell(30, 20);
  plugin_->use_final_approach_orientation_ = true;
  seedCache(straight_path(1.0, 3.0, 2.0));

  nav_msgs::msg::Path candidate;
  candidate.poses = {
    make_pose(1.0, 2.0), make_pose(3.0, 1.0), make_pose(3.0, 2.0)};
  candidate.poses.back().pose.orientation =
    nav2_util::geometry_utils::orientationAroundZAxis(M_PI_2);
  plugin_->applyPathHysteresis(make_pose(1.0, 2.0), candidate);
  ASSERT_FALSE(candidate.poses.empty());
  EXPECT_NEAR(tf2::getYaw(candidate.poses.back().pose.orientation), 0.0, 1e-9);
}

TEST_F(PathHysteresisTest, new_obstacle_invalidates_old_path_and_uses_safe_candidate)
{
  setGoalCell(30, 20);
  auto retained = straight_path(1.0, 3.0, 2.0);
  seedCache(retained);

  auto * costmap = costmap_ros_->getCostmap();
  unsigned int obstacle_x, obstacle_y;
  ASSERT_TRUE(costmap->worldToMap(2.0, 2.0, obstacle_x, obstacle_y));
  costmap->setCost(obstacle_x, obstacle_y, nav2_costmap_2d::LETHAL_OBSTACLE);
  unsigned int start_x, start_y, upper_start_x, upper_start_y;
  unsigned int upper_mid_x, upper_mid_y, upper_end_x, upper_end_y, goal_x, goal_y;
  ASSERT_TRUE(costmap->worldToMap(1.0, 2.0, start_x, start_y));
  ASSERT_TRUE(costmap->worldToMap(1.0, 2.5, upper_start_x, upper_start_y));
  ASSERT_TRUE(costmap->worldToMap(2.0, 2.5, upper_mid_x, upper_mid_y));
  ASSERT_TRUE(costmap->worldToMap(3.0, 2.5, upper_end_x, upper_end_y));
  ASSERT_TRUE(costmap->worldToMap(3.0, 2.0, goal_x, goal_y));
  auto candidate = path_through_cells({
    {start_x, start_y}, {upper_start_x, upper_start_y},
    {upper_mid_x, upper_mid_y}, {upper_end_x, upper_end_y}, {goal_x, goal_y}});
  plugin_->applyPathHysteresis(make_cell_pose(costmap, start_x, start_y), candidate);
  ASSERT_FALSE(candidate.poses.empty());
  EXPECT_NEAR(candidate.poses.back().pose.position.x, 3.0, 1e-6);
  EXPECT_GT(candidate.poses[1].pose.position.y, 2.2);
  EXPECT_TRUE(plugin_->have_cached_path_);
}

TEST_F(PathHysteresisTest, invalid_candidate_keeps_a_still_traversable_retained_route)
{
  setGoalCell(30, 20);
  seedCache(straight_path(1.0, 3.0, 2.0));
  auto * costmap = costmap_ros_->getCostmap();
  unsigned int obstacle_x, obstacle_y;
  ASSERT_TRUE(costmap->worldToMap(2.0, 2.5, obstacle_x, obstacle_y));
  costmap->setCost(obstacle_x, obstacle_y, nav2_costmap_2d::LETHAL_OBSTACLE);

  unsigned int start_x, start_y, upper_start_x, upper_start_y;
  unsigned int upper_mid_x, upper_mid_y, upper_end_x, upper_end_y, goal_x, goal_y;
  ASSERT_TRUE(costmap->worldToMap(1.0, 2.0, start_x, start_y));
  ASSERT_TRUE(costmap->worldToMap(1.0, 2.5, upper_start_x, upper_start_y));
  ASSERT_TRUE(costmap->worldToMap(2.0, 2.5, upper_mid_x, upper_mid_y));
  ASSERT_TRUE(costmap->worldToMap(3.0, 2.5, upper_end_x, upper_end_y));
  ASSERT_TRUE(costmap->worldToMap(3.0, 2.0, goal_x, goal_y));
  auto candidate = path_through_cells({
    {start_x, start_y}, {upper_start_x, upper_start_y},
    {upper_mid_x, upper_mid_y}, {upper_end_x, upper_end_y}, {goal_x, goal_y}});
  const auto robot = make_cell_pose(costmap, start_x, start_y);
  EXPECT_FALSE(plugin_->pathTraversable(candidate, robot));

  plugin_->applyPathHysteresis(robot, candidate);
  ASSERT_FALSE(candidate.poses.empty());
  EXPECT_NEAR(candidate.poses.front().pose.position.y, 2.0, 1e-6);
  EXPECT_NEAR(candidate.poses.back().pose.position.x, 3.0, 1e-6);
  EXPECT_NEAR(candidate.poses.back().pose.position.y, 2.0, 1e-6);
  EXPECT_TRUE(plugin_->have_cached_path_);
}

TEST_F(PathHysteresisTest, shared_narrow_terminal_does_not_hide_clearance_gain_on_route_difference)
{
  auto * costmap = costmap_ros_->getCostmap();
  ASSERT_GT(costmap->getSizeInCellsX(), 35U);
  ASSERT_GT(costmap->getSizeInCellsY(), 31U);
  const auto retained = path_through_cells({{5, 25}, {35, 25}});
  auto candidate = path_through_cells({
    {5, 25}, {11, 31}, {19, 31}, {25, 25}, {35, 25}});
  setGoalCell(35, 25);
  plugin_->path_hysteresis_confirmations_ = 3;
  seedCache(retained);

  // One lethal cell narrows the routes' shared terminal. Another narrows only the
  // retained middle segment; neither cell intersects either route centerline.
  costmap->setCost(33, 24, nav2_costmap_2d::LETHAL_OBSTACLE);
  costmap->setCost(15, 26, nav2_costmap_2d::LETHAL_OBSTACLE);
  plugin_->buildObstacleDistanceField();
  const auto robot = make_cell_pose(costmap, 5, 25);
  test_theta_star_planner::PathMetrics retained_metrics;
  test_theta_star_planner::PathMetrics candidate_metrics;
  ASSERT_TRUE(plugin_->pathTraversable(retained, robot, &retained_metrics));
  ASSERT_TRUE(plugin_->pathTraversable(candidate, robot, &candidate_metrics));
  EXPECT_NEAR(
    retained_metrics.minimum_obstacle_gap,
    candidate_metrics.minimum_obstacle_gap, costmap->getResolution());
  plugin_->measureExclusivePath(retained, candidate, retained_metrics);
  plugin_->measureExclusivePath(candidate, retained, candidate_metrics);
  ASSERT_TRUE(retained_metrics.has_exclusive_segment);
  ASSERT_TRUE(candidate_metrics.has_exclusive_segment);
  EXPECT_GT(
    candidate_metrics.exclusive_minimum_obstacle_gap,
    retained_metrics.exclusive_minimum_obstacle_gap +
    plugin_->path_hysteresis_min_clearance_gain_);

  const double retained_y = make_cell_pose(costmap, 5, 25).pose.position.y;
  for (int confirmation = 1; confirmation <= 3; ++confirmation) {
    auto next_candidate = candidate;
    plugin_->applyPathHysteresis(robot, next_candidate);
    ASSERT_FALSE(next_candidate.poses.empty());
    const auto max_y = std::max_element(
      next_candidate.poses.begin(), next_candidate.poses.end(),
      [](const auto & lhs, const auto & rhs) {
        return lhs.pose.position.y < rhs.pose.position.y;
      })->pose.position.y;
    if (confirmation < 3) {
      EXPECT_NEAR(max_y, retained_y, costmap->getResolution());
    } else {
      EXPECT_GT(max_y, retained_y + 0.2);
    }
  }
}

TEST_F(PathHysteresisTest, adopts_more_centered_route_from_obstacle_distance_score)
{
  auto * costmap = costmap_ros_->getCostmap();
  ASSERT_GT(costmap->getSizeInCellsX(), 70U);
  ASSERT_GT(costmap->getSizeInCellsY(), 35U);
  const auto retained = path_through_cells({{5, 25}, {65, 25}});
  const auto candidate = path_through_cells({{5, 25}, {15, 29}, {55, 29}, {65, 25}});
  setGoalCell(65, 25);
  seedCache(retained);

  // A wall runs beside the retained route. The candidate spends more distance away
  // from it, while its equal endpoints and small length increase keep the switch bounded.
  for (unsigned int x = 18; x <= 52; ++x) {
    costmap->setCost(x, 20, nav2_costmap_2d::LETHAL_OBSTACLE);
  }
  plugin_->planner_->w_clearance_cost_ = 0.5;
  plugin_->path_hysteresis_min_clearance_gain_ = 0.8;
  plugin_->buildObstacleDistanceField();
  const auto robot = make_cell_pose(costmap, 5, 25);
  test_theta_star_planner::PathMetrics retained_metrics;
  test_theta_star_planner::PathMetrics candidate_metrics;
  ASSERT_TRUE(plugin_->pathTraversable(retained, robot, &retained_metrics));
  ASSERT_TRUE(plugin_->pathTraversable(candidate, robot, &candidate_metrics));
  plugin_->measureExclusivePath(retained, candidate, retained_metrics);
  plugin_->measureExclusivePath(candidate, retained, candidate_metrics);
  EXPECT_GT(
    retained_metrics.exclusive_clearance_cost,
    candidate_metrics.exclusive_clearance_cost +
    plugin_->path_hysteresis_min_clearance_gain_);

  auto selected = candidate;
  plugin_->applyPathHysteresis(robot, selected);
  ASSERT_FALSE(selected.poses.empty());
  EXPECT_GT(selected.poses[1].pose.position.y, retained.poses.front().pose.position.y + 0.2);
}

TEST_F(PathHysteresisTest, shared_bottleneck_allows_route_with_less_exclusive_high_cost_length)
{
  auto * costmap = costmap_ros_->getCostmap();
  ASSERT_GT(costmap->getSizeInCellsX(), 35U);
  ASSERT_GT(costmap->getSizeInCellsY(), 29U);
  const auto retained = path_through_cells({{5, 25}, {35, 25}});
  auto candidate = path_through_cells({
    {5, 25}, {11, 29}, {19, 29}, {25, 25}, {35, 25}});
  setGoalCell(35, 25);
  plugin_->path_hysteresis_confirmations_ = 3;
  seedCache(retained);

  // Both routes share the same narrow terminal. The retained route also crosses a
  // longer high-cost middle section, while the candidate passes above it.
  costmap->setCost(33, 24, nav2_costmap_2d::LETHAL_OBSTACLE);
  for (unsigned int x = 12; x <= 22; ++x) {
    costmap->setCost(x, 25, 180);
  }
  plugin_->buildObstacleDistanceField();
  const auto robot = make_cell_pose(costmap, 5, 25);
  test_theta_star_planner::PathMetrics retained_metrics;
  test_theta_star_planner::PathMetrics candidate_metrics;
  ASSERT_TRUE(plugin_->pathTraversable(retained, robot, &retained_metrics));
  ASSERT_TRUE(plugin_->pathTraversable(candidate, robot, &candidate_metrics));
  EXPECT_NEAR(
    retained_metrics.minimum_obstacle_gap,
    candidate_metrics.minimum_obstacle_gap, costmap->getResolution());
  plugin_->measureExclusivePath(retained, candidate, retained_metrics);
  plugin_->measureExclusivePath(candidate, retained, candidate_metrics);
  EXPECT_GT(retained_metrics.exclusive_high_cost_length, 0.0);
  EXPECT_LT(candidate_metrics.exclusive_high_cost_length, retained_metrics.exclusive_high_cost_length);

  for (int confirmation = 1; confirmation <= 3; ++confirmation) {
    auto next_candidate = candidate;
    plugin_->applyPathHysteresis(robot, next_candidate);
    ASSERT_FALSE(next_candidate.poses.empty());
    if (confirmation < 3) {
      EXPECT_NEAR(next_candidate.poses[1].pose.position.y,
        make_cell_pose(costmap, 5, 25).pose.position.y, costmap->getResolution());
    } else {
      EXPECT_GT(next_candidate.poses[1].pose.position.y,
        make_cell_pose(costmap, 5, 25).pose.position.y + 0.2);
    }
  }
}

TEST_F(PathHysteresisTest, distance_field_refreshes_when_a_new_obstacle_enters_the_snapshot)
{
  auto * costmap = costmap_ros_->getCostmap();
  plugin_->buildObstacleDistanceField();
  bool initially_capped = false;
  const auto before = plugin_->obstacleGapAtCell(20, 20, initially_capped);
  EXPECT_TRUE(initially_capped);

  costmap->setCost(25, 20, nav2_costmap_2d::LETHAL_OBSTACLE);
  plugin_->buildObstacleDistanceField();
  bool after_capped = true;
  const auto after = plugin_->obstacleGapAtCell(20, 20, after_capped);
  EXPECT_FALSE(after_capped);
  EXPECT_LT(after, before);
}

TEST_F(PathHysteresisTest, drops_cache_when_retained_path_enters_inscribed_inflation)
{
  setGoalCell(30, 20);
  seedCache(straight_path(1.0, 3.0, 2.0));
  // No escape region was computed, so 253 is off limits everywhere, exactly as it is
  // for the search itself
  setCost(2.0, 2.0, nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);

  auto candidate = nav_msgs::msg::Path{};
  candidate.poses = {
    make_pose(1.0, 2.0), make_pose(1.0, 2.5), make_pose(3.0, 2.5), make_pose(3.0, 2.0)};
  plugin_->applyPathHysteresis(make_pose(1.0, 2.0), candidate);
  EXPECT_NEAR(candidate.poses.back().pose.position.x, 3.0, 1e-6);
  EXPECT_GT(candidate.poses[1].pose.position.y, 2.2);
}

TEST_F(PathHysteresisTest, drops_cache_when_retained_path_re_enters_the_escape_region)
{
  setGoalCell(30, 20);
  seedCache(straight_path(1.0, 3.0, 2.0));

  const auto width = costmap_ros_->getCostmap()->getSizeInCellsX();
  const auto height = costmap_ros_->getCostmap()->getSizeInCellsY();
  plugin_->planner_->start_exit_cells_.assign(width * height, 0);

  // The robot starts inside the escape region and the path leaves it, so the search
  // would refuse to re-enter an inscribed cell later on, even though that cell is also
  // flagged as part of the same region.
  const auto mark_escape_cell = [&](const geometry_msgs::msg::PoseStamped & pose) {
      unsigned int mx, my;
      ASSERT_TRUE(
        costmap_ros_->getCostmap()->worldToMap(
          pose.pose.position.x, pose.pose.position.y, mx, my));
      costmap_ros_->getCostmap()->setCost(
        mx, my, nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
      plugin_->planner_->start_exit_cells_[my * width + mx] = 1;
    };
  mark_escape_cell(plugin_->cached_path_.poses.front());
  // Use the generated path pose itself so its floating-point representation maps to
  // the exact grid cell that is then marked as the later re-entry point.
  mark_escape_cell(plugin_->cached_path_.poses[14]);

  // Route around both marked escape cells; the stale straight route must not be reused.
  auto candidate = nav_msgs::msg::Path{};
  candidate.poses = {
    make_pose(1.0, 2.0), make_pose(1.0, 2.5), make_pose(3.0, 2.5), make_pose(3.0, 2.0)};
  plugin_->applyPathHysteresis(make_pose(1.0, 2.0), candidate);
  EXPECT_NEAR(candidate.poses.back().pose.position.x, 3.0, 1e-6);
  EXPECT_GT(candidate.poses[1].pose.position.y, 2.2);
}

TEST_F(PathHysteresisTest, retained_path_is_anchored_on_the_robot_pose)
{
  setGoalCell(30, 20);
  seedCache(straight_path(1.0, 3.0, 2.0));

  // TEB overwrites the first pose of whatever plan it receives with the robot position,
  // so the retained path has to already start there or that overwrite silently creates
  // an unchecked segment to the second point.
  auto candidate = straight_path(1.0, 2.9, 2.0);
  plugin_->applyPathHysteresis(make_pose(1.5, 2.2), candidate);

  ASSERT_GE(candidate.poses.size(), 2u);
  EXPECT_NEAR(candidate.poses.front().pose.position.x, 1.5, 1e-6);
  EXPECT_NEAR(candidate.poses.front().pose.position.y, 2.2, 1e-6);
  EXPECT_NEAR(candidate.poses.back().pose.position.x, 3.0, 1e-6);
}

TEST_F(PathHysteresisTest, rejoins_at_polyline_projection_without_duplicate_points)
{
  setGoalCell(30, 20);
  seedCache(straight_path(1.0, 3.0, 2.0));

  auto candidate = straight_path(1.0, 2.9, 2.0);
  plugin_->applyPathHysteresis(make_pose(1.55, 2.2), candidate);

  ASSERT_GE(candidate.poses.size(), 3u);
  EXPECT_NEAR(candidate.poses.front().pose.position.x, 1.55, 1e-6);
  EXPECT_NEAR(candidate.poses.front().pose.position.y, 2.2, 1e-6);
  EXPECT_NEAR(candidate.poses[1].pose.position.x, 1.55, 1e-6);
  EXPECT_NEAR(candidate.poses[1].pose.position.y, 2.0, 1e-6);
  for (std::size_t i = 1; i < candidate.poses.size(); ++i) {
    const auto & previous = candidate.poses[i - 1].pose.position;
    const auto & current = candidate.poses[i].pose.position;
    EXPECT_GE(std::hypot(current.x - previous.x, current.y - previous.y), 1e-6);
  }
}

TEST_F(PathHysteresisTest, straight_rejoin_does_not_backtrack_to_previous_sample)
{
  setGoalCell(30, 20);
  seedCache(straight_path(1.0, 3.0, 2.0));

  auto candidate = straight_path(1.0, 2.9, 2.0);
  plugin_->applyPathHysteresis(make_pose(1.55, 2.2), candidate);

  ASSERT_GE(candidate.poses.size(), 2u);
  for (std::size_t i = 1; i < candidate.poses.size(); ++i) {
    EXPECT_GE(
      candidate.poses[i].pose.position.x + 1e-6,
      candidate.poses[i - 1].pose.position.x);
  }
}

TEST_F(PathHysteresisTest, consecutive_replans_keep_forward_progress_on_a_straight_path)
{
  setGoalCell(30, 20);
  seedCache(straight_path(1.0, 3.0, 2.0));

  for (const double robot_x : {1.55, 1.75, 2.05}) {
    auto candidate = straight_path(1.0, 2.9, 2.0);
    plugin_->applyPathHysteresis(make_pose(robot_x, 2.2), candidate);
    ASSERT_GE(candidate.poses.size(), 2u);
    EXPECT_NEAR(candidate.poses.front().pose.position.x, robot_x, 1e-6);
    for (std::size_t i = 1; i < candidate.poses.size(); ++i) {
      const auto & previous = candidate.poses[i - 1].pose.position;
      const auto & current = candidate.poses[i].pose.position;
      EXPECT_GE(current.x + 1e-6, previous.x);
      EXPECT_GE(std::hypot(current.x - previous.x, current.y - previous.y), 1e-6);
    }
  }
}

TEST_F(PathHysteresisTest, same_cell_goal_clears_previous_cached_route)
{
  const auto start = make_pose(0.05, 0.05);
  ASSERT_FALSE(plugin_->createPlan(start, make_pose(1.0, 1.0)).poses.empty());
  ASSERT_TRUE(plugin_->have_cached_path_);

  unsigned int start_mx = 0;
  unsigned int start_my = 0;
  ASSERT_TRUE(costmap_ros_->getCostmap()->worldToMap(
      start.pose.position.x, start.pose.position.y, start_mx, start_my));
  const auto same_cell_goal = make_cell_pose(costmap_ros_->getCostmap(), start_mx, start_my);
  const auto completed_path = plugin_->createPlan(start, same_cell_goal);
  ASSERT_EQ(completed_path.poses.size(), 1u);
  EXPECT_FALSE(plugin_->have_cached_path_);
  EXPECT_TRUE(plugin_->cached_path_.poses.empty());
}

TEST_F(PathHysteresisTest, multi_pose_plan_ends_at_exact_goal_near_cell_corner)
{
  auto * costmap = costmap_ros_->getCostmap();
  const double resolution = costmap->getResolution();
  unsigned int goal_mx = 0;
  unsigned int goal_my = 0;
  ASSERT_TRUE(costmap->worldToMap(1.0, 1.0, goal_mx, goal_my));
  double center_x = 0.0;
  double center_y = 0.0;
  costmap->mapToWorld(goal_mx, goal_my, center_x, center_y);
  auto goal = make_pose(center_x + resolution * 0.45, center_y + resolution * 0.45);
  goal.header.frame_id = costmap_ros_->getGlobalFrameID();
  unsigned int corner_goal_mx = 0;
  unsigned int corner_goal_my = 0;
  ASSERT_TRUE(costmap->worldToMap(
      goal.pose.position.x, goal.pose.position.y, corner_goal_mx, corner_goal_my));
  ASSERT_EQ(corner_goal_mx, goal_mx);
  ASSERT_EQ(corner_goal_my, goal_my);

  const auto path = plugin_->createPlan(make_pose(0.05, 0.05), goal);
  ASSERT_GT(path.poses.size(), 1u);
  EXPECT_NEAR(path.poses.back().pose.position.x, goal.pose.position.x, 1e-9);
  EXPECT_NEAR(path.poses.back().pose.position.y, goal.pose.position.y, 1e-9);
}

TEST_F(PathHysteresisTest, drops_cache_when_retained_path_leaves_the_costmap)
{
  setGoalCell(30, 20);
  // The default costmap spans 5 m, so the tail of this path has no cost to check
  seedCache(straight_path(4.5, 5.5, 2.0));

  auto candidate = straight_path(1.0, 3.5, 2.0);
  plugin_->applyPathHysteresis(make_pose(4.5, 2.0), candidate);
  EXPECT_NEAR(candidate.poses.back().pose.position.x, 3.5, 1e-6);
}

TEST_F(PathHysteresisTest, tolerates_lethal_cell_under_the_robot_itself)
{
  // Establish a cache via the real planning pipeline, then mark the robot cell lethal
  // and verify that createPlan() still returns the cached path (not empty).
  // This exercises the full createPlan() → getPlan() → isUnsafeToPlan() path, unlike
  // a direct applyPathHysteresis() call that bypasses isStartSafe().
  unsigned int rx, ry;
  ASSERT_TRUE(costmap_ros_->getCostmap()->worldToMap(0.05, 0.05, rx, ry));
  const auto robot = make_cell_pose(costmap_ros_->getCostmap(), rx, ry);
  auto path = plugin_->createPlan(robot, make_pose(1.0, 1.0));
  ASSERT_FALSE(path.poses.empty());
  ASSERT_TRUE(plugin_->have_cached_path_);

  costmap_ros_->getCostmap()->setCost(rx, ry, nav2_costmap_2d::LETHAL_OBSTACLE);

  path = plugin_->createPlan(robot, make_pose(1.0, 1.0));
  EXPECT_FALSE(path.poses.empty());
}

TEST_F(PathHysteresisTest, unknown_start_does_not_reuse_cached_path_when_unknown_is_disallowed)
{
  unsigned int rx, ry;
  ASSERT_TRUE(costmap_ros_->getCostmap()->worldToMap(0.05, 0.05, rx, ry));
  const auto robot = make_cell_pose(costmap_ros_->getCostmap(), rx, ry);
  const auto goal = make_pose(1.0, 1.0);
  ASSERT_FALSE(plugin_->createPlan(robot, goal).poses.empty());
  ASSERT_TRUE(plugin_->have_cached_path_);

  plugin_->planner_->allow_unknown_ = false;
  costmap_ros_->getCostmap()->setCost(rx, ry, nav2_costmap_2d::NO_INFORMATION);
  const auto path = plugin_->createPlan(robot, goal);
  EXPECT_TRUE(path.poses.empty());
  EXPECT_FALSE(plugin_->have_cached_path_);
}

TEST_F(PathHysteresisTest, robot_start_cell_is_not_exempted_after_path_leaves_and_reenters)
{
  auto * costmap = costmap_ros_->getCostmap();
  const auto start = make_cell_pose(costmap, 10, 10);
  nav_msgs::msg::Path path;
  path.poses = {
    start,
    make_cell_pose(costmap, 11, 10),
    start};
  costmap->setCost(10, 10, nav2_costmap_2d::LETHAL_OBSTACLE);

  EXPECT_FALSE(plugin_->pathTraversable(path, start));
}

TEST_F(PathHysteresisTest, rejects_cache_when_rejoin_supercover_touches_lethal_cell)
{
  auto * costmap = costmap_ros_->getCostmap();
  const unsigned int mid_x = costmap->getSizeInCellsX() / 2;
  const unsigned int mid_y = costmap->getSizeInCellsY() / 2;
  const unsigned int start_x = mid_x + 3;
  const unsigned int start_y = mid_y + 3;
  const unsigned int cache_start_x = mid_x - 3;
  const unsigned int cache_start_y = mid_y - 3;
  const unsigned int goal_x = mid_x - 10;
  const unsigned int goal_y = mid_y - 3;
  setGoalCell(static_cast<int>(goal_x), static_cast<int>(goal_y));
  seedCache(path_through_cells({{cache_start_x, cache_start_y}, {goal_x, goal_y}}));

  auto candidate = path_through_cells(
  {
    {start_x, start_y}, {start_x, mid_y - 10}, {mid_x + 15, mid_y - 10},
    {mid_x + 15, goal_y}, {goal_x, goal_y}});
  const auto candidate_second = candidate.poses[1].pose.position;

  // The robot-to-cache diagonal touches this side cell exactly at its first grid
  // corner; the cell is not represented by endpoint-only sampling.
  costmap->setCost(start_x - 1, start_y, nav2_costmap_2d::LETHAL_OBSTACLE);
  const auto rejoin = path_through_cells(
    {{start_x, start_y}, {cache_start_x, cache_start_y}, {goal_x, goal_y}});
  EXPECT_EQ(costmap->getCost(start_x - 1, start_y), nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_FALSE(plugin_->pathTraversable(rejoin, candidate.poses.front()));
  plugin_->applyPathHysteresis(candidate.poses.front(), candidate);

  ASSERT_GE(candidate.poses.size(), 2u);
  EXPECT_NEAR(candidate.poses[1].pose.position.x, candidate_second.x, 1e-9);
  EXPECT_NEAR(candidate.poses[1].pose.position.y, candidate_second.y, 1e-9);
}

TEST_F(PathHysteresisTest, rejects_cache_when_rejoin_supercover_touches_inscribed_cell)
{
  auto * costmap = costmap_ros_->getCostmap();
  const unsigned int mid_x = costmap->getSizeInCellsX() / 2;
  const unsigned int mid_y = costmap->getSizeInCellsY() / 2;
  const unsigned int start_x = mid_x + 3;
  const unsigned int start_y = mid_y + 3;
  const unsigned int cache_start_x = mid_x - 3;
  const unsigned int cache_start_y = mid_y - 3;
  const unsigned int goal_x = mid_x - 10;
  const unsigned int goal_y = mid_y - 3;
  setGoalCell(static_cast<int>(goal_x), static_cast<int>(goal_y));
  seedCache(path_through_cells({{cache_start_x, cache_start_y}, {goal_x, goal_y}}));

  auto candidate = path_through_cells(
  {
    {start_x, start_y}, {start_x, mid_y - 10}, {mid_x + 15, mid_y - 10},
    {mid_x + 15, goal_y}, {goal_x, goal_y}});
  const auto candidate_second = candidate.poses[1].pose.position;
  costmap->setCost(
    start_x - 1, start_y, nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
  const auto rejoin = path_through_cells(
    {{start_x, start_y}, {cache_start_x, cache_start_y}, {goal_x, goal_y}});
  EXPECT_EQ(
    costmap->getCost(start_x - 1, start_y), nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
  EXPECT_FALSE(plugin_->pathTraversable(rejoin, candidate.poses.front()));
  plugin_->applyPathHysteresis(candidate.poses.front(), candidate);

  ASSERT_GE(candidate.poses.size(), 2u);
  EXPECT_NEAR(candidate.poses[1].pose.position.x, candidate_second.x, 1e-9);
  EXPECT_NEAR(candidate.poses[1].pose.position.y, candidate_second.y, 1e-9);
}

TEST_F(PathHysteresisTest, charges_the_rejoin_gap_to_the_cache)
{
  setGoalCell(30, 20);
  seedCache(straight_path(1.0, 3.0, 2.0));

  // Robot is 1.5 m off the lane: rejoining costs 1.5 m plus the 1.0 m that remains,
  // which loses to a 2.0 m candidate even though the retained tail alone is shorter
  auto candidate = straight_path(1.0, 3.0, 2.0);
  plugin_->applyPathHysteresis(make_pose(2.0, 0.5), candidate);
  EXPECT_NEAR(candidate.poses.front().pose.position.x, 1.0, 1e-6);
  EXPECT_EQ(candidate.poses.size(), straight_path(1.0, 3.0, 2.0).poses.size());
}

TEST_F(PathHysteresisTest, zero_ratio_always_adopts_candidate)
{
  setGoalCell(30, 20);
  seedCache(straight_path(1.0, 3.0, 2.0));
  plugin_->path_hysteresis_ratio_ = 0.0;

  auto candidate = straight_path(1.0, 2.9, 2.0);
  plugin_->applyPathHysteresis(make_pose(1.0, 2.0), candidate);
  EXPECT_NEAR(candidate.poses.back().pose.position.x, 2.9, 1e-6);
}

TEST_F(PathHysteresisTest, planning_failure_clears_the_cache)
{
  auto path = plugin_->createPlan(make_pose(0.05, 0.05), make_pose(1.0, 1.0));
  ASSERT_FALSE(path.poses.empty());
  ASSERT_TRUE(plugin_->have_cached_path_);

  for (unsigned int i = 7; i <= 14; ++i) {
    for (unsigned int j = 7; j <= 14; ++j) {
      costmap_ros_->getCostmap()->setCost(i, j, nav2_costmap_2d::LETHAL_OBSTACLE);
    }
  }
  path = plugin_->createPlan(make_pose(0.05, 0.05), make_pose(1.0, 1.0));
  EXPECT_TRUE(path.poses.empty());
  EXPECT_FALSE(plugin_->have_cached_path_);
}

// Smoke tests meant to detect issues arising from the plugin part rather than the algorithm
TEST(ThetaStarPlanner, test_theta_star_planner) {
  rclcpp_lifecycle::LifecycleNode::SharedPtr life_node =
    std::make_shared<rclcpp_lifecycle::LifecycleNode>("ThetaStarPlannerTest");

  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros =
    std::make_shared<nav2_costmap_2d::Costmap2DROS>("global_costmap");
  costmap_ros->on_configure(rclcpp_lifecycle::State());

  geometry_msgs::msg::PoseStamped start, goal;
  start.pose.position.x = 0.0;
  start.pose.position.y = 0.0;
  start.pose.orientation.w = 1.0;
  goal = start;
  auto planner_2d = std::make_unique<nav2_theta_star_planner::ThetaStarPlanner>();
  planner_2d->configure(life_node, "test", nullptr, costmap_ros);
  planner_2d->activate();

  nav_msgs::msg::Path path = planner_2d->createPlan(start, goal);
  EXPECT_GT(static_cast<int>(path.poses.size()), 0);

  unsigned int start_x, start_y;
  ASSERT_TRUE(
    costmap_ros->getCostmap()->worldToMap(
      start.pose.position.x, start.pose.position.y, start_x, start_y));
  costmap_ros->getCostmap()->setCost(
    start_x, start_y, nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
  path = planner_2d->createPlan(start, start);
  EXPECT_FALSE(path.poses.empty());
  costmap_ros->getCostmap()->setCost(start_x, start_y, nav2_costmap_2d::FREE_SPACE);

  // test if the goal is unsafe
  for (int i = 7; i <= 14; i++) {
    for (int j = 7; j <= 14; j++) {
      costmap_ros->getCostmap()->setCost(i, j, 254);
    }
  }
  goal.pose.position.x = 1.0;
  goal.pose.position.y = 1.0;
  path = planner_2d->createPlan(start, goal);
  EXPECT_EQ(static_cast<int>(path.poses.size()), 0);

  planner_2d->deactivate();
  planner_2d->cleanup();

  planner_2d.reset();
  costmap_ros->on_cleanup(rclcpp_lifecycle::State());
  life_node.reset();
  costmap_ros.reset();
}

TEST(ThetaStarPlanner, test_theta_star_reconfigure)
{
  rclcpp_lifecycle::LifecycleNode::SharedPtr life_node =
    std::make_shared<rclcpp_lifecycle::LifecycleNode>("ThetaStarPlannerTest");

  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros =
    std::make_shared<nav2_costmap_2d::Costmap2DROS>("global_costmap");
  costmap_ros->on_configure(rclcpp_lifecycle::State());

  auto planner = std::make_unique<nav2_theta_star_planner::ThetaStarPlanner>();
  try {
    // Expect to throw due to invalid prims file in param
    planner->configure(life_node, "test", nullptr, costmap_ros);
  } catch (...) {
  }
  planner->activate();

  auto rec_param = std::make_shared<rclcpp::AsyncParametersClient>(
    life_node->get_node_base_interface(), life_node->get_node_topics_interface(),
    life_node->get_node_graph_interface(),
    life_node->get_node_services_interface());

  auto results = rec_param->set_parameters_atomically(
    {rclcpp::Parameter("test.how_many_corners", 8),
      rclcpp::Parameter("test.w_euc_cost", 1.0),
      rclcpp::Parameter("test.w_traversal_cost", 2.0),
      rclcpp::Parameter("test.use_final_approach_orientation", false),
      rclcpp::Parameter("test.allow_unknown", false)});

  rclcpp::spin_until_future_complete(
    life_node->get_node_base_interface(),
    results);

  EXPECT_EQ(life_node->get_parameter("test.how_many_corners").as_int(), 8);
  EXPECT_EQ(
    life_node->get_parameter("test.w_euc_cost").as_double(),
    1.0);
  EXPECT_EQ(life_node->get_parameter("test.w_traversal_cost").as_double(), 2.0);
  EXPECT_EQ(life_node->get_parameter("test.use_final_approach_orientation").as_bool(), false);
  EXPECT_EQ(life_node->get_parameter("test.allow_unknown").as_bool(), false);

  rclcpp::spin_until_future_complete(
    life_node->get_node_base_interface(),
    results);
}
