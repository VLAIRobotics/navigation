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

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>
#include "nav2_theta_star_planner/theta_star.hpp"

namespace theta_star
{
namespace
{
struct Point2D
{
  double x;
  double y;
};

double cross(const Point2D & a, const Point2D & b, const Point2D & c)
{
  return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}

bool pointOnSegment(const Point2D & point, const Point2D & a, const Point2D & b)
{
  constexpr double epsilon = 1.0e-9;
  return std::abs(cross(a, b, point)) <= epsilon &&
         point.x >= std::min(a.x, b.x) - epsilon &&
         point.x <= std::max(a.x, b.x) + epsilon &&
         point.y >= std::min(a.y, b.y) - epsilon &&
         point.y <= std::max(a.y, b.y) + epsilon;
}

bool segmentsIntersect(
  const Point2D & a, const Point2D & b, const Point2D & c, const Point2D & d)
{
  constexpr double epsilon = 1.0e-9;
  const double ab_c = cross(a, b, c);
  const double ab_d = cross(a, b, d);
  const double cd_a = cross(c, d, a);
  const double cd_b = cross(c, d, b);
  if (((ab_c > epsilon && ab_d < -epsilon) || (ab_c < -epsilon && ab_d > epsilon)) &&
    ((cd_a > epsilon && cd_b < -epsilon) || (cd_a < -epsilon && cd_b > epsilon)))
  {
    return true;
  }
  return pointOnSegment(c, a, b) || pointOnSegment(d, a, b) ||
         pointOnSegment(a, c, d) || pointOnSegment(b, c, d);
}

bool pointInPolygon(const Point2D & point, const std::vector<Point2D> & polygon)
{
  bool inside = false;
  for (std::size_t i = 0, previous = polygon.size() - 1;
    i < polygon.size(); previous = i++)
  {
    const auto & a = polygon[previous];
    const auto & b = polygon[i];
    if (pointOnSegment(point, a, b)) {
      return true;
    }
    if ((a.y > point.y) != (b.y > point.y)) {
      const double crossing_x = a.x + (point.y - a.y) * (b.x - a.x) / (b.y - a.y);
      if (point.x < crossing_x) {
        inside = !inside;
      }
    }
  }
  return inside;
}

bool polygonIntersectsCell(const std::vector<Point2D> & polygon, int x, int y)
{
  const double left = x;
  const double bottom = y;
  const double right = static_cast<double>(x) + 1.0;
  const double top = static_cast<double>(y) + 1.0;
  for (const auto & point : polygon) {
    if (point.x >= left - 1.0e-9 && point.x <= right + 1.0e-9 &&
      point.y >= bottom - 1.0e-9 && point.y <= top + 1.0e-9)
    {
      return true;
    }
  }

  const Point2D corners[] = {{left, bottom}, {right, bottom}, {right, top}, {left, top}};
  for (const auto & corner : corners) {
    if (pointInPolygon(corner, polygon)) {
      return true;
    }
  }
  for (std::size_t i = 0; i < polygon.size(); ++i) {
    for (std::size_t edge = 0; edge < 4; ++edge) {
      if (segmentsIntersect(
          polygon[i], polygon[(i + 1) % polygon.size()],
          corners[edge], corners[(edge + 1) % 4]))
      {
        return true;
      }
    }
  }
  return false;
}

bool footprintCoversUnknown(
  const nav2_costmap_2d::Costmap2D & costmap,
  const std::vector<geometry_msgs::msg::Point> & footprint,
  const geometry_msgs::msg::PoseStamped & start)
{
  if (footprint.size() < 3) {
    return false;
  }
  const double resolution = costmap.getResolution();
  if (!std::isfinite(resolution) || resolution <= 0.0) {
    return false;
  }

  const auto & orientation = start.pose.orientation;
  const double quaternion_norm = std::hypot(
    std::hypot(orientation.x, orientation.y), std::hypot(orientation.z, orientation.w));
  if (!std::isfinite(quaternion_norm) || quaternion_norm < 1.0e-6) {
    return false;
  }
  const double qx = orientation.x / quaternion_norm;
  const double qy = orientation.y / quaternion_norm;
  const double qz = orientation.z / quaternion_norm;
  const double qw = orientation.w / quaternion_norm;
  const double yaw = std::atan2(2.0 * (qw * qz + qx * qy),
      1.0 - 2.0 * (qy * qy + qz * qz));
  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);
  const double start_x = (start.pose.position.x - costmap.getOriginX()) / resolution;
  const double start_y = (start.pose.position.y - costmap.getOriginY()) / resolution;
  if (!std::isfinite(start_x) || !std::isfinite(start_y)) {
    return false;
  }

  std::vector<Point2D> polygon;
  polygon.reserve(footprint.size());
  for (const auto & point : footprint) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y)) {
      return false;
    }
    polygon.push_back({
      start_x + (cos_yaw * point.x - sin_yaw * point.y) / resolution,
      start_y + (sin_yaw * point.x + cos_yaw * point.y) / resolution});
  }

  double min_x = polygon.front().x;
  double max_x = min_x;
  double min_y = polygon.front().y;
  double max_y = min_y;
  for (const auto & point : polygon) {
    min_x = std::min(min_x, point.x);
    max_x = std::max(max_x, point.x);
    min_y = std::min(min_y, point.y);
    max_y = std::max(max_y, point.y);
  }
  if (costmap.getSizeInCellsX() > static_cast<unsigned int>(std::numeric_limits<int>::max()) ||
    costmap.getSizeInCellsY() > static_cast<unsigned int>(std::numeric_limits<int>::max()))
  {
    return false;
  }
  const int width = static_cast<int>(costmap.getSizeInCellsX());
  const int height = static_cast<int>(costmap.getSizeInCellsY());
  if (width <= 0 || height <= 0 || max_x < 0.0 || max_y < 0.0 ||
    min_x >= width || min_y >= height)
  {
    return false;
  }
  const int first_x = static_cast<int>(std::max(0.0, std::floor(min_x) - 1.0));
  const int first_y = static_cast<int>(std::max(0.0, std::floor(min_y) - 1.0));
  const int last_x = static_cast<int>(std::min(
      static_cast<double>(width - 1), std::ceil(max_x)));
  const int last_y = static_cast<int>(std::min(
      static_cast<double>(height - 1), std::ceil(max_y)));
  for (int y = first_y; y <= last_y; ++y) {
    for (int x = first_x; x <= last_x; ++x) {
      if (polygonIntersectsCell(polygon, x, y) &&
        costmap.getCost(static_cast<unsigned int>(x), static_cast<unsigned int>(y)) == UNKNOWN_COST)
      {
        return true;
      }
    }
  }
  return false;
}
}  // namespace

ThetaStar::ThetaStar()
: w_traversal_cost_(1.0),
  w_clearance_cost_(0.0),
  w_euc_cost_(2.0),
  w_heuristic_cost_(1.0),
  how_many_corners_(8),
  allow_unknown_(true),
  start_unknown_escape_radius_(0.0),
  size_x_(0),
  size_y_(0),
  index_generated_(0)
{
  exp_node = new tree_node;
}

void ThetaStar::setStartAndGoal(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  unsigned int s[2] = {0, 0}, d[2] = {0, 0};
  const bool start_in_map = costmap_->worldToMap(
    start.pose.position.x, start.pose.position.y, s[0], s[1]);
  const bool goal_in_map = costmap_->worldToMap(
    goal.pose.position.x, goal.pose.position.y, d[0], d[1]);

  src_ = start_in_map ? coordsM{static_cast<int>(s[0]), static_cast<int>(s[1])} : coordsM{-1, -1};
  dst_ = goal_in_map ? coordsM{static_cast<int>(d[0]), static_cast<int>(d[1])} : coordsM{-1, -1};

  start_exit_cells_.clear();
  const auto map_width = costmap_->getSizeInCellsX();
  const auto map_height = costmap_->getSizeInCellsY();
  const bool valid_start = src_.x >= 0 && src_.y >= 0 &&
    src_.x < static_cast<int>(map_width) && src_.y < static_cast<int>(map_height);
  const auto start_cost = valid_start ? costmap_->getCost(src_.x, src_.y) : 0;
  const bool unknown_start = start_cost == UNKNOWN_COST &&
    !allow_unknown_ && start_unknown_escape_radius_ > 0.0;
  const bool footprint_unknown_start =
    start_cost != nav2_costmap_2d::LETHAL_OBSTACLE &&
    !allow_unknown_ && start_unknown_escape_radius_ > 0.0 &&
    footprintCoversUnknown(*costmap_, robot_footprint_, start);
  const bool inscribed_start = start_cost == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
  if (valid_start && !allow_unknown_ && start_unknown_escape_radius_ > 0.0 &&
    start_cost != nav2_costmap_2d::LETHAL_OBSTACLE &&
    (unknown_start || footprint_unknown_start))
  {
    double start_world_x = 0.0;
    double start_world_y = 0.0;
    costmap_->mapToWorld(src_.x, src_.y, start_world_x, start_world_y);
    const double resolution = costmap_->getResolution();
    const double start_x = static_cast<double>(src_.x) + 0.5 +
      (start.pose.position.x - start_world_x) / resolution;
    const double start_y = static_cast<double>(src_.y) + 0.5 +
      (start.pose.position.y - start_world_y) / resolution;
    const double radius_cells = start_unknown_escape_radius_ / resolution;
    const auto first_x = static_cast<std::size_t>(std::max(
      0.0, std::floor(start_x - radius_cells) - 1.0));
    const auto first_y = static_cast<std::size_t>(std::max(
      0.0, std::floor(start_y - radius_cells) - 1.0));
    const auto last_x = static_cast<std::size_t>(std::min(
      static_cast<double>(map_width - 1), std::ceil(start_x + radius_cells)));
    const auto last_y = static_cast<std::size_t>(std::min(
      static_cast<double>(map_height - 1), std::ceil(start_y + radius_cells)));

    bool has_unknown_in_disk = unknown_start;
    for (std::size_t y = first_y; y <= last_y; ++y) {
      for (std::size_t x = first_x; x <= last_x; ++x) {
        const double dx = std::max({0.0, static_cast<double>(x) - start_x,
          start_x - static_cast<double>(x + 1)}) * resolution;
        const double dy = std::max({0.0, static_cast<double>(y) - start_y,
          start_y - static_cast<double>(y + 1)}) * resolution;
        if (dx * dx + dy * dy <=
          start_unknown_escape_radius_ * start_unknown_escape_radius_ + 1.0e-12 &&
          costmap_->getCost(static_cast<unsigned int>(x), static_cast<unsigned int>(y)) == UNKNOWN_COST)
        {
          has_unknown_in_disk = true;
        }
      }
    }

    if (has_unknown_in_disk) {
      start_exit_cells_.assign(map_width * map_height, 0);
      for (std::size_t y = first_y; y <= last_y; ++y) {
        for (std::size_t x = first_x; x <= last_x; ++x) {
          const double dx = std::max({0.0, static_cast<double>(x) - start_x,
            start_x - static_cast<double>(x + 1)}) * resolution;
          const double dy = std::max({0.0, static_cast<double>(y) - start_y,
            start_y - static_cast<double>(y + 1)}) * resolution;
          if (dx * dx + dy * dy <=
            start_unknown_escape_radius_ * start_unknown_escape_radius_ + 1.0e-12)
          {
            start_exit_cells_[y * map_width + x] = 1;
          }
        }
      }
    }
  }

  if (valid_start && inscribed_start) {
    if (start_exit_cells_.empty()) {
      start_exit_cells_.assign(map_width * map_height, 0);
    }
    std::vector<std::uint8_t> inscribed_visited(map_width * map_height, 0);
    std::queue<coordsM> pending;
    pending.push(src_);
    inscribed_visited[static_cast<std::size_t>(src_.y) * map_width + src_.x] = 1;
    start_exit_cells_[static_cast<std::size_t>(src_.y) * map_width + src_.x] = 1;
    while (!pending.empty()) {
      const auto cell = pending.front();
      pending.pop();
      for (const auto & move : moves) {
        const int next_x = cell.x + move.x;
        const int next_y = cell.y + move.y;
        if (next_x < 0 || next_y < 0 || next_x >= static_cast<int>(map_width) ||
          next_y >= static_cast<int>(map_height))
        {
          continue;
        }
        if (costmap_->getCost(next_x, next_y) !=
          nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
        {
          continue;
        }
        const auto index = static_cast<std::size_t>(next_y) * map_width + next_x;
        if (!inscribed_visited[index]) {
          inscribed_visited[index] = 1;
          start_exit_cells_[index] = 1;
          pending.push({next_x, next_y});
        }
      }
    }
  }
}

bool ThetaStar::checkCell(
  int x, int y, bool left_start_exit, bool terminal, Rejection * rejection) const
{
  const bool in_map = x >= 0 && y >= 0 &&
    x < static_cast<int>(costmap_->getSizeInCellsX()) &&
    y < static_cast<int>(costmap_->getSizeInCellsY());
  const int cost = in_map ? costmap_->getCost(x, y) : -1;
  const bool in_exit = in_map && isStartExitCell(x, y);
  const char * kind = nullptr;
  if (!in_map) {
    kind = "out_of_bounds";
  } else if (cost == UNKNOWN_COST && !allow_unknown_) {
    if (in_exit && left_start_exit) {
      kind = "start_escape";
    } else if (!in_exit) {
      kind = "unknown";
    }
  } else if (cost == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
    if (!terminal && in_exit && left_start_exit) {
      kind = "start_escape";
    } else if (!terminal && !in_exit) {
      kind = "obstacle";
    }
  } else if (cost == nav2_costmap_2d::LETHAL_OBSTACLE) {
    kind = "obstacle";
  }
  if (kind != nullptr && rejection != nullptr && !rejection->valid) {
    rejection->valid = true;
    rejection->x = x; rejection->y = y; rejection->cost = cost;
    rejection->world_x = costmap_->getOriginX() + (x + 0.5) * costmap_->getResolution();
    rejection->world_y = costmap_->getOriginY() + (y + 0.5) * costmap_->getResolution();
    rejection->kind = kind;
  }
  return kind == nullptr;
}

bool ThetaStar::generatePath(std::vector<coordsW> & raw_path)
{
  resetContainers();
  first_search_rejection_ = Rejection{};
  addToNodesData(index_generated_);
  double src_g_cost = getTraversalCost(src_.x, src_.y), src_h_cost = getHCost(src_.x, src_.y);
  nodes_data_[index_generated_] =
  {src_.x, src_.y, src_g_cost, src_h_cost, &nodes_data_[index_generated_], true,
    src_g_cost + src_h_cost};
  queue_.push({&nodes_data_[index_generated_]});
  addIndex(src_.x, src_.y, &nodes_data_[index_generated_]);
  tree_node * curr_data = nullptr;
  index_generated_++;
  nodes_opened = 0;

  while (!queue_.empty()) {
    curr_data = queue_.top();
    queue_.pop();
    nodes_opened++;

    if (isGoal(*curr_data)) {
      break;
    }

    resetParent(curr_data);
    setNeighbors(curr_data);
  }

  if (curr_data == nullptr || !isGoal(*curr_data)) {
    raw_path.clear();
    return false;
  }

  backtrace(raw_path, curr_data);
  clearQueue();

  return true;
}

void ThetaStar::resetParent(tree_node * curr_data)
{
  double g_cost, los_cost = 0;
  curr_data->is_in_queue = false;
  const tree_node * curr_par = curr_data->parent_id;
  const tree_node * maybe_par = curr_par->parent_id;

  if (losCheck(maybe_par->x, maybe_par->y, curr_data->x, curr_data->y, los_cost)) {
    g_cost = maybe_par->g +
      getEuclideanCost(curr_data->x, curr_data->y, maybe_par->x, maybe_par->y) + los_cost;

    if (g_cost < curr_data->g) {
      curr_data->parent_id = maybe_par;
      curr_data->g = g_cost;
      curr_data->f = g_cost + curr_data->h;
    }
  }
}

void ThetaStar::setNeighbors(const tree_node * curr_data)
{
  int mx, my;
  tree_node * m_id = nullptr;
  double g_cost, h_cost, cal_cost;

  for (int i = 0; i < how_many_corners_; i++) {
    mx = curr_data->x + moves[i].x;
    my = curr_data->y + moves[i].y;

    bool left = !isStartExitCell(curr_data->x, curr_data->y);
    double edge_traversal_cost = 0.0;
    if (!traverseCells(
        curr_data->x, curr_data->y, mx, my, left, &first_search_rejection_,
        [this, &edge_traversal_cost](int x, int y) {
          edge_traversal_cost += getTraversalCost(x, y);
        }))
    {
      continue;
    }

    g_cost = curr_data->g + getEuclideanCost(curr_data->x, curr_data->y, mx, my) +
      edge_traversal_cost;

    m_id = getIndex(mx, my);

    if (m_id == nullptr) {
      addToNodesData(index_generated_);
      m_id = &nodes_data_[index_generated_];
      addIndex(mx, my, m_id);
      index_generated_++;
    }

    exp_node = m_id;

    h_cost = getHCost(mx, my);
    cal_cost = g_cost + h_cost;
    if (exp_node->f > cal_cost) {
      exp_node->g = g_cost;
      exp_node->h = h_cost;
      exp_node->f = cal_cost;
      exp_node->parent_id = curr_data;
      if (!exp_node->is_in_queue) {
        exp_node->x = mx;
        exp_node->y = my;
        exp_node->is_in_queue = true;
        queue_.push({m_id});
      }
    }
  }
}

void ThetaStar::backtrace(std::vector<coordsW> & raw_points, const tree_node * curr_n) const
{
  std::vector<coordsW> path_rev;
  coordsW world{};
  do {
    costmap_->mapToWorld(curr_n->x, curr_n->y, world.x, world.y);
    path_rev.push_back(world);
    if (path_rev.size() > 1) {
      curr_n = curr_n->parent_id;
    }
  } while (curr_n->parent_id != curr_n);
  costmap_->mapToWorld(curr_n->x, curr_n->y, world.x, world.y);
  path_rev.push_back(world);

  raw_points.reserve(path_rev.size());
  for (int i = static_cast<int>(path_rev.size()) - 1; i >= 0; i--) {
    raw_points.push_back(path_rev[i]);
  }
}

bool ThetaStar::losCheck(
  const int & x0, const int & y0, const int & x1, const int & y1,
  double & sl_cost) const
{
  sl_cost = 0.0;
  bool left = !isStartExitCell(x0, y0);
  return traverseCells(
    x0, y0, x1, y1, left, &first_search_rejection_,
    [this, &sl_cost](int x, int y) {
      sl_cost += getTraversalCost(x, y);
    });
}

void ThetaStar::resetContainers()
{
  index_generated_ = 0;
  int last_size_x = size_x_;
  int last_size_y = size_y_;
  int curr_size_x = static_cast<int>(costmap_->getSizeInCellsX());
  int curr_size_y = static_cast<int>(costmap_->getSizeInCellsY());
  if (((last_size_x != curr_size_x) || (last_size_y != curr_size_y)) &&
    static_cast<int>(node_position_.size()) < (curr_size_x * curr_size_y))
  {
    initializePosn(curr_size_y * curr_size_x - last_size_y * last_size_x);
    nodes_data_.reserve(curr_size_x * curr_size_y);
  } else {
    initializePosn();
  }
  size_x_ = curr_size_x;
  size_y_ = curr_size_y;
}

void ThetaStar::initializePosn(int size_inc)
{
  if (!node_position_.empty()) {
    for (int i = 0; i < size_x_ * size_y_; i++) {
      node_position_[i] = nullptr;
    }
  }

  for (int i = 0; i < size_inc; i++) {
    node_position_.push_back(nullptr);
  }
}
}  //  namespace theta_star
