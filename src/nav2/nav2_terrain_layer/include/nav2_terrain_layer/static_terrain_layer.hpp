#ifndef NAV2_TERRAIN_LAYER__STATIC_TERRAIN_LAYER_HPP_
#define NAV2_TERRAIN_LAYER__STATIC_TERRAIN_LAYER_HPP_

#include <cstdint>
#include <mutex>
#include <string>
#include <unordered_map>
#include <unordered_set>

#include <fastdem/elevation_map.hpp>
#include <fastdem/bridge/ros2.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav2_costmap_2d/costmap_layer.hpp>
#include <nav2_terrain_layer/terrain_cost.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>

namespace nav2_terrain_layer
{

class StaticTerrainLayer : public nav2_costmap_2d::CostmapLayer
{
public:
  StaticTerrainLayer() = default;
  ~StaticTerrainLayer() override = default;

  void reset() override;
  bool isClearable() override {return false;}
  void updateBounds(
    double robot_x, double robot_y, double robot_yaw, double * min_x, double * min_y,
    double * max_x, double * max_y) override;
  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j, int max_i,
    int max_j) override;
  void matchSize() override;

protected:
  void onInitialize() override;

private:
  struct LiveResidualEvidence
  {
    std::uint64_t low_residual_history{0};
    std::uint64_t hard_step_history{0};
    std::uint64_t last_source_sequence{0};
    std::int64_t last_fresh_observation_stamp_ns{0};
    double residual_step{0.0};
    float elevation{0.0F};
    float slope{0.0F};
    float roughness{0.0F};
    float points{0.0F};
    float observation_age{0.0F};
    float variance{0.0F};
    bool cost_expired{false};
  };

  void addMapBounds(double * min_x, double * min_y, double * max_x, double * max_y) const;
  void onlineTerrainCallback(const grid_map_msgs::msg::GridMap::SharedPtr message);
  void fusedMapCallback(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr message);
  bool validMap() const;
  bool costmapGeometryMatches() const;

  bool map_loaded_{false};
  bool geometry_valid_{false};
  bool hard_feature_constraints_{true};
  bool residual_step_hard_constraint_{true};
  bool use_live_terrain_{true};
  std::string terrain_map_file_;
  std::string global_frame_{"map"};
  std::string live_terrain_topic_{"/fastdem/postprocess/gridmap"};
  std::string global_terrain_topic_{"/navigation/global_terrain_map"};
  fastdem::ElevationMap terrain_map_;
  TerrainCostConfig cost_config_;
  int min_observations_{1};
  int live_max_observation_age_frames_{3};
  int live_confirmation_frames_{3};
  int live_confirmation_window_frames_{5};
  int live_min_neighbor_support_{5};
  double live_residual_timeout_{1.5};
  std::int64_t live_residual_timeout_ns_{1500000000LL};
  std::uint64_t last_live_source_stamp_ns_{0};
  std::uint64_t live_source_sequence_{0};
  mutable std::mutex terrain_map_mutex_;
  std::mutex live_evidence_mutex_;
  std::unordered_map<std::int64_t, LiveResidualEvidence> live_residual_evidence_;
  std::unordered_set<std::int64_t> dirty_terrain_cells_;
  bool full_rebuild_needed_{true};
  std::uint64_t cost_update_count_{0};
  std::uint64_t terrain_snapshot_publish_count_{0};
  double last_update_bounds_ms_{0.0};
  grid_map_msgs::msg::GridMap terrain_snapshot_cache_;
  bool terrain_snapshot_cache_valid_{false};
  rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr live_terrain_subscription_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr fused_map_subscription_;
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr global_terrain_publisher_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr terrain_changed_publisher_;
};

}  // namespace nav2_terrain_layer

#endif  // NAV2_TERRAIN_LAYER__STATIC_TERRAIN_LAYER_HPP_
