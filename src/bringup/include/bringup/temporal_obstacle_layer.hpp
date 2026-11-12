#ifndef BRINGUP__TEMPORAL_OBSTACLE_LAYER_HPP_
#define BRINGUP__TEMPORAL_OBSTACLE_LAYER_HPP_

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <nav2_costmap_2d/costmap_layer.hpp>
#include <nav2_msgs/msg/costmap.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace bringup
{

class TemporalObstacleLayer : public nav2_costmap_2d::CostmapLayer
{
public:
  TemporalObstacleLayer() = default;
  ~TemporalObstacleLayer() override = default;

  void reset() override;
  void clearArea(int start_x, int start_y, int end_x, int end_y, bool invert) override;
  bool isClearable() override {return true;}
  void updateBounds(
    double robot_x, double robot_y, double robot_yaw, double * min_x, double * min_y,
    double * max_x, double * max_y) override;
  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j, int max_i,
    int max_j) override;
  void matchSize() override;
  void activate() override;
  void deactivate() override;

protected:
  void onInitialize() override;

private:
  struct CellKey
  {
    std::int64_t x;
    std::int64_t y;

    bool operator==(const CellKey & other) const
    {
      return x == other.x && y == other.y;
    }
  };

  struct CellKeyHash
  {
    std::size_t operator()(const CellKey & key) const;
  };

  struct Point
  {
    double x;
    double y;
    double z;
    std::chrono::steady_clock::time_point last_seen;
    std::int64_t source_stamp_ns;
  };

  void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr message);
  void handleClock(const std::int64_t now_ros_ns);
  bool observationIsFresh(
    std::chrono::steady_clock::time_point now_steady, std::int64_t now_ros_ns) const;
  void expirePoints(
    std::chrono::steady_clock::time_point now_steady, std::int64_t now_ros_ns);
  void clearFootprint(double robot_x, double robot_y, double robot_yaw);
  void publishDiagnosticSnapshot();
  static bool insidePolygon(
    double x, double y, const std::vector<geometry_msgs::msg::Point> & polygon);

  std::mutex mutex_;
  std::unordered_map<CellKey, Point, CellKeyHash> points_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscriber_;
  rclcpp_lifecycle::LifecyclePublisher<nav2_msgs::msg::Costmap>::SharedPtr
    diagnostic_costmap_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr
    diagnostic_points_publisher_;
  std::string observation_topic_;
  std::chrono::steady_clock::duration point_retention_{std::chrono::seconds(1)};
  std::chrono::steady_clock::duration observation_timeout_{std::chrono::seconds(1)};
  std::chrono::steady_clock::time_point last_cloud_received_{};
  std::int64_t last_cloud_stamp_ns_{0};
  std::int64_t last_ros_now_ns_{0};
  std::int64_t point_retention_ns_{1000000000LL};
  std::int64_t future_tolerance_ns_{100000000LL};
  std::uint64_t incoming_generation_{0};
  std::uint64_t latest_generation_{0};
  std::uint64_t integrated_generation_{0};
  double robot_x_{0.0};
  double robot_y_{0.0};
  double robot_yaw_{0.0};
  double resolution_{0.10};
  bool footprint_clearing_enabled_{true};
  bool have_cloud_{false};
  bool latest_cloud_valid_{false};
};

}  // namespace bringup

#endif  // BRINGUP__TEMPORAL_OBSTACLE_LAYER_HPP_
