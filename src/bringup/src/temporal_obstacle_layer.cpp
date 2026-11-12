#include "bringup/temporal_obstacle_layer.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <stdexcept>
#include <vector>

#include <nav2_costmap_2d/cost_values.hpp>
#include <nav2_costmap_2d/footprint.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace bringup
{

std::size_t TemporalObstacleLayer::CellKeyHash::operator()(const CellKey & key) const
{
  const auto hx = std::hash<std::int64_t>{}(key.x);
  const auto hy = std::hash<std::int64_t>{}(key.y);
  return hx ^ (hy + static_cast<std::size_t>(0x9e3779b9) + (hx << 6) + (hx >> 2));
}

void TemporalObstacleLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("TemporalObstacleLayer lifecycle node expired during initialization");
  }

  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("observation_topic", rclcpp::ParameterValue("/obstacles/blocking"));
  declareParameter("point_retention", rclcpp::ParameterValue(1.0));
  declareParameter("observation_timeout", rclcpp::ParameterValue(1.0));
  declareParameter("future_tolerance", rclcpp::ParameterValue(0.1));
  declareParameter("footprint_clearing_enabled", rclcpp::ParameterValue(true));
  node->get_parameter(getFullName("enabled"), enabled_);
  node->get_parameter(getFullName("observation_topic"), observation_topic_);
  double point_retention = 1.0;
  double observation_timeout = 1.0;
  double future_tolerance = 0.1;
  node->get_parameter(getFullName("point_retention"), point_retention);
  node->get_parameter(getFullName("observation_timeout"), observation_timeout);
  node->get_parameter(getFullName("future_tolerance"), future_tolerance);
  node->get_parameter(getFullName("footprint_clearing_enabled"), footprint_clearing_enabled_);
  if (observation_topic_.empty() || !std::isfinite(point_retention) || point_retention <= 0.0 ||
    !std::isfinite(observation_timeout) || observation_timeout <= 0.0 ||
    !std::isfinite(future_tolerance) || future_tolerance < 0.0)
  {
    throw std::runtime_error("TemporalObstacleLayer topic and timeouts must be valid");
  }
  point_retention_ = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(point_retention));
  observation_timeout_ = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(observation_timeout));
  point_retention_ns_ = static_cast<std::int64_t>(std::llround(point_retention * 1e9));
  future_tolerance_ns_ = static_cast<std::int64_t>(std::llround(future_tolerance * 1e9));

  default_value_ = nav2_costmap_2d::FREE_SPACE;
  matchSize();
  current_ = false;
  const auto diagnostic_qos = rclcpp::QoS(1).reliable().transient_local();
  diagnostic_costmap_publisher_ = node->create_publisher<nav2_msgs::msg::Costmap>(
    getName() + "/costmap_raw", diagnostic_qos);
  diagnostic_points_publisher_ = node->create_publisher<sensor_msgs::msg::PointCloud2>(
    getName() + "/source_points", diagnostic_qos);
  cloud_subscriber_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
    observation_topic_, rclcpp::SensorDataQoS(),
    std::bind(&TemporalObstacleLayer::cloudCallback, this, std::placeholders::_1));
}

void TemporalObstacleLayer::cloudCallback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr message)
{
  const auto now_steady = std::chrono::steady_clock::now();
  const auto now_ros_ns = clock_->now().nanoseconds();
  std::uint64_t generation;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    handleClock(now_ros_ns);
    generation = ++incoming_generation_;
    latest_cloud_valid_ = false;
  }
  const auto invalidate = [this, now_ros_ns](const char * reason) {
      std::lock_guard<std::mutex> lock(mutex_);
      handleClock(now_ros_ns);
      latest_cloud_valid_ = false;
      RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000, "%s", reason);
    };
  if (!message || message->header.frame_id != layered_costmap_->getGlobalFrameID() ||
    message->header.stamp.sec < 0 || message->header.stamp.nanosec >= 1000000000U ||
    (message->header.stamp.sec == 0 && message->header.stamp.nanosec == 0))
  {
    invalidate("Temporal obstacle cloud has an invalid timestamp or frame");
    return;
  }

  const auto stamp_ns = static_cast<std::int64_t>(message->header.stamp.sec) * 1000000000LL +
    message->header.stamp.nanosec;
  struct ReceivedPoint
  {
    double x;
    double y;
    double z;
    bool clear;
  };
  std::vector<ReceivedPoint> received_points;
  const auto operation_field = std::find_if(
    message->fields.begin(), message->fields.end(),
    [](const sensor_msgs::msg::PointField & field) {return field.name == "operation";});
  const bool has_operation = operation_field != message->fields.end();
  if (has_operation &&
    (operation_field->datatype != sensor_msgs::msg::PointField::UINT8 ||
    operation_field->count != 1))
  {
    invalidate("Temporal obstacle cloud has an invalid operation field");
    return;
  }
  try {
    sensor_msgs::PointCloud2ConstIterator<float> x(*message, "x");
    sensor_msgs::PointCloud2ConstIterator<float> y(*message, "y");
    sensor_msgs::PointCloud2ConstIterator<float> z(*message, "z");
    if (has_operation) {
      sensor_msgs::PointCloud2ConstIterator<std::uint8_t> operation(*message, "operation");
      for (; x != x.end(); ++x, ++y, ++z, ++operation) {
        if (!std::isfinite(*x) || !std::isfinite(*y) || !std::isfinite(*z)) {
          continue;
        }
        received_points.push_back({*x, *y, *z, *operation != 0U});
      }
    } else {
      for (; x != x.end(); ++x, ++y, ++z) {
        if (!std::isfinite(*x) || !std::isfinite(*y) || !std::isfinite(*z)) {
          continue;
        }
        received_points.push_back({*x, *y, *z, false});
      }
    }
  } catch (const std::runtime_error & error) {
    const std::string reason = std::string("Cannot read temporal obstacle cloud: ") + error.what();
    invalidate(reason.c_str());
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  handleClock(now_ros_ns);
  if (generation != incoming_generation_) {
    return;
  }
  const auto source_age_ns = now_ros_ns - stamp_ns;
  if (now_ros_ns <= 0 || source_age_ns > point_retention_ns_ ||
    source_age_ns < -future_tolerance_ns_)
  {
    latest_cloud_valid_ = false;
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 2000,
      "Rejecting temporal obstacle cloud outside the source-time freshness window");
    return;
  }
  if (have_cloud_ && stamp_ns <= last_cloud_stamp_ns_) {
    latest_cloud_valid_ = false;
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 2000,
      "Temporal obstacle cloud timestamp did not advance; rejecting repeated scan");
    return;
  }

  for (const auto & received : received_points) {
    const CellKey key{
      static_cast<std::int64_t>(std::floor(received.x / resolution_)),
      static_cast<std::int64_t>(std::floor(received.y / resolution_))};
    if (received.clear) {
      const auto found = points_.find(key);
      if (found != points_.end() && found->second.source_stamp_ns <= stamp_ns) {
        points_.erase(found);
      }
    } else {
      points_[key] = Point{received.x, received.y, received.z, now_steady, stamp_ns};
    }
  }
  last_cloud_stamp_ns_ = stamp_ns;
  last_cloud_received_ = now_steady;
  have_cloud_ = true;
  latest_cloud_valid_ = true;
  latest_generation_ = generation;
}

void TemporalObstacleLayer::handleClock(const std::int64_t now_ros_ns)
{
  if (last_ros_now_ns_ > 0 && now_ros_ns < last_ros_now_ns_) {
    points_.clear();
    have_cloud_ = false;
    latest_cloud_valid_ = false;
    latest_generation_ = 0;
    last_cloud_stamp_ns_ = 0;
    last_cloud_received_ = {};
  }
  last_ros_now_ns_ = now_ros_ns;
}

bool TemporalObstacleLayer::observationIsFresh(
  std::chrono::steady_clock::time_point now_steady, std::int64_t now_ros_ns) const
{
  if (!latest_cloud_valid_ || !have_cloud_ || now_ros_ns <= 0 ||
    now_steady - last_cloud_received_ > observation_timeout_)
  {
    return false;
  }
  const auto source_age_ns = now_ros_ns - last_cloud_stamp_ns_;
  return source_age_ns <= point_retention_ns_ && source_age_ns >= -future_tolerance_ns_;
}

void TemporalObstacleLayer::expirePoints(
  std::chrono::steady_clock::time_point now_steady, std::int64_t now_ros_ns)
{
  for (auto it = points_.begin(); it != points_.end();) {
    const bool source_expired = now_ros_ns > 0 && now_ros_ns >= it->second.source_stamp_ns &&
      now_ros_ns - it->second.source_stamp_ns >= point_retention_ns_;
    const bool receive_expired = now_steady - it->second.last_seen >= point_retention_;
    if (source_expired || receive_expired) {
      it = points_.erase(it);
    } else {
      ++it;
    }
  }
}

bool TemporalObstacleLayer::insidePolygon(
  double x, double y, const std::vector<geometry_msgs::msg::Point> & polygon)
{
  if (polygon.size() < 3) {
    return false;
  }
  bool inside = false;
  for (std::size_t i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++) {
    const auto & a = polygon[j];
    const auto & b = polygon[i];
    const double cross = (x - a.x) * (b.y - a.y) - (y - a.y) * (b.x - a.x);
    if (std::abs(cross) <= 1e-9 &&
      x >= std::min(a.x, b.x) && x <= std::max(a.x, b.x) &&
      y >= std::min(a.y, b.y) && y <= std::max(a.y, b.y))
    {
      return true;
    }
    if ((a.y > y) != (b.y > y) &&
      x < (b.x - a.x) * (y - a.y) / (b.y - a.y) + a.x)
    {
      inside = !inside;
    }
  }
  return inside;
}

void TemporalObstacleLayer::clearFootprint(double robot_x, double robot_y, double robot_yaw)
{
  if (!footprint_clearing_enabled_) {
    return;
  }
  std::vector<geometry_msgs::msg::Point> world_footprint;
  nav2_costmap_2d::transformFootprint(
    robot_x, robot_y, robot_yaw, layered_costmap_->getFootprint(), world_footprint);
  if (world_footprint.size() < 3) {
    return;
  }
  for (auto it = points_.begin(); it != points_.end();) {
    if (insidePolygon(it->second.x, it->second.y, world_footprint)) {
      it = points_.erase(it);
    } else {
      ++it;
    }
  }
}

void TemporalObstacleLayer::matchSize()
{
  CostmapLayer::matchSize();
  resolution_ = getResolution();
  std::lock_guard<std::mutex> lock(mutex_);
  resetMaps();
}

void TemporalObstacleLayer::activate()
{
  if (diagnostic_costmap_publisher_) {
    diagnostic_costmap_publisher_->on_activate();
  }
  if (diagnostic_points_publisher_) {
    diagnostic_points_publisher_->on_activate();
  }
}

void TemporalObstacleLayer::deactivate()
{
  if (diagnostic_costmap_publisher_) {
    diagnostic_costmap_publisher_->on_deactivate();
  }
  if (diagnostic_points_publisher_) {
    diagnostic_points_publisher_->on_deactivate();
  }
}

void TemporalObstacleLayer::updateBounds(
  double robot_x, double robot_y, double robot_yaw, double * min_x, double * min_y,
  double * max_x, double * max_y)
{
  if (layered_costmap_->isRolling()) {
    updateOrigin(robot_x - getSizeInMetersX() / 2.0, robot_y - getSizeInMetersY() / 2.0);
  }

  const auto now_steady = std::chrono::steady_clock::now();
  const auto now_ros_ns = clock_->now().nanoseconds();
  std::lock_guard<std::mutex> lock(mutex_);
  handleClock(now_ros_ns);
  robot_x_ = robot_x;
  robot_y_ = robot_y;
  robot_yaw_ = robot_yaw;
  expirePoints(now_steady, now_ros_ns);
  clearFootprint(robot_x, robot_y, robot_yaw);
  if (!observationIsFresh(now_steady, now_ros_ns) ||
    latest_generation_ != incoming_generation_ ||
    latest_generation_ != integrated_generation_)
  {
    current_ = false;
  }

  *min_x = std::min(*min_x, getOriginX());
  *min_y = std::min(*min_y, getOriginY());
  *max_x = std::max(*max_x, getOriginX() + getSizeInMetersX());
  *max_y = std::max(*max_y, getOriginY() + getSizeInMetersY());
}

void TemporalObstacleLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j, int max_i, int max_j)
{
  const auto now_steady = std::chrono::steady_clock::now();
  const auto now_ros_ns = clock_->now().nanoseconds();
  std::lock_guard<std::mutex> lock(mutex_);
  handleClock(now_ros_ns);
  if (latest_generation_ != incoming_generation_ ||
    latest_generation_ != integrated_generation_ ||
    !observationIsFresh(now_steady, now_ros_ns))
  {
    current_ = false;
  }
  expirePoints(now_steady, now_ros_ns);
  clearFootprint(robot_x_, robot_y_, robot_yaw_);
  resetMaps();
  for (const auto & [unused, point] : points_) {
    unsigned int mx;
    unsigned int my;
    if (worldToMap(point.x, point.y, mx, my)) {
      setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);
    }
  }
  updateWithOverwrite(master_grid, min_i, min_j, max_i, max_j);
  if (latest_generation_ == incoming_generation_ &&
    observationIsFresh(now_steady, now_ros_ns))
  {
    integrated_generation_ = latest_generation_;
    current_ = true;
  } else {
    current_ = false;
  }
  publishDiagnosticSnapshot();
}

void TemporalObstacleLayer::publishDiagnosticSnapshot()
{
  if (diagnostic_costmap_publisher_ && diagnostic_costmap_publisher_->is_activated()) {
    nav2_msgs::msg::Costmap snapshot;
    snapshot.header.frame_id = layered_costmap_->getGlobalFrameID();
    snapshot.header.stamp = clock_->now();
    snapshot.metadata.map_load_time = snapshot.header.stamp;
    snapshot.metadata.update_time = snapshot.header.stamp;
    snapshot.metadata.layer = getName();
    snapshot.metadata.resolution = static_cast<float>(getResolution());
    snapshot.metadata.size_x = getSizeInCellsX();
    snapshot.metadata.size_y = getSizeInCellsY();
    snapshot.metadata.origin.position.x = getOriginX();
    snapshot.metadata.origin.position.y = getOriginY();
    snapshot.metadata.origin.orientation.w = 1.0;
    const auto cell_count = static_cast<std::size_t>(snapshot.metadata.size_x) *
      snapshot.metadata.size_y;
    const auto * source = getCharMap();
    snapshot.data.assign(source, source + cell_count);
    diagnostic_costmap_publisher_->publish(snapshot);
  }
  if (!diagnostic_points_publisher_ || !diagnostic_points_publisher_->is_activated() ||
    diagnostic_points_publisher_->get_subscription_count() == 0)
  {
    return;
  }
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.frame_id = layered_costmap_->getGlobalFrameID();
  cloud.header.stamp = clock_->now();
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(points_.size());
  sensor_msgs::PointCloud2Iterator<float> x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> z(cloud, "z");
  for (const auto & [unused, point] : points_) {
    *x = static_cast<float>(point.x);
    *y = static_cast<float>(point.y);
    *z = static_cast<float>(point.z);
    ++x;
    ++y;
    ++z;
  }
  diagnostic_points_publisher_->publish(cloud);
}

void TemporalObstacleLayer::clearArea(
  int start_x, int start_y, int end_x, int end_y, bool invert)
{
  std::lock_guard<std::mutex> lock(mutex_);
  for (auto it = points_.begin(); it != points_.end();) {
    unsigned int mx;
    unsigned int my;
    const bool in_rectangle = worldToMap(it->second.x, it->second.y, mx, my) &&
      static_cast<int>(mx) >= start_x && static_cast<int>(mx) <= end_x &&
      static_cast<int>(my) >= start_y && static_cast<int>(my) <= end_y;
    if (in_rectangle == invert) {
      ++it;
    } else {
      it = points_.erase(it);
    }
  }
  CostmapLayer::clearArea(start_x, start_y, end_x, end_y, invert);
}

void TemporalObstacleLayer::reset()
{
  std::lock_guard<std::mutex> lock(mutex_);
  points_.clear();
  have_cloud_ = false;
  latest_cloud_valid_ = false;
  latest_generation_ = 0;
  integrated_generation_ = 0;
  last_cloud_stamp_ns_ = 0;
  last_cloud_received_ = {};
  current_ = false;
  resetMaps();
}

}  // namespace bringup

PLUGINLIB_EXPORT_CLASS(bringup::TemporalObstacleLayer, nav2_costmap_2d::Layer)
