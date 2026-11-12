#ifndef NAV2_TERRAIN_LAYER__TERRAIN_LAYER_HPP_
#define NAV2_TERRAIN_LAYER__TERRAIN_LAYER_HPP_

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <unordered_set>

#include <grid_map_msgs/msg/grid_map.hpp>
#include <nanogrid/GridMap.hpp>
#include <nav2_costmap_2d/costmap_layer.hpp>
#include <nav2_msgs/msg/costmap.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <std_msgs/msg/bool.hpp>

#include "nav2_terrain_layer/terrain_cost.hpp"

namespace nav2_terrain_layer
{
class TerrainLayer : public nav2_costmap_2d::CostmapLayer
{
public:
  TerrainLayer() = default;
  ~TerrainLayer() override = default;

  void reset() override;
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
  struct WorldCell
  {
    std::int64_t x;
    std::int64_t y;

    bool operator==(const WorldCell & other) const {return x == other.x && y == other.y;}
  };

  struct WorldCellHash
  {
    std::size_t operator()(const WorldCell & cell) const
    {
      const auto hx = std::hash<std::int64_t>{}(cell.x);
      const auto hy = std::hash<std::int64_t>{}(cell.y);
      return hx ^ (hy + static_cast<std::size_t>(0x9e3779b9) + (hx << 6) + (hx >> 2));
    }
  };

  struct ConfirmationState
  {
    std::uint64_t hit_history{0};
    int frames_in_window{0};
    // Missing frames are counted in the rolling window and are not reset by a hit.
    int missing_frames{0};
    bool confirmed{false};
    std::int64_t last_hit_time_ns{0};
    std::int64_t confirmed_since_ns{0};
  };

  void gridMapCallback(const grid_map_msgs::msg::GridMap::SharedPtr msg);
  bool mapIsFresh(const rclcpp::Time & now) const;
  void addFullCostmapBounds(double * min_x, double * min_y, double * max_x, double * max_y) const;
  void clearConfirmationStateLocked();
  bool expireConfirmationsLocked(std::int64_t now_ns);
  void invalidateMapLocked();
  void suspendMapLocked();
  void publishTerrainStatusLocked(bool valid, bool force_publish = false);
  void publishDiagnosticSnapshot(bool source_is_current);
  WorldCell worldCell(const nanogrid::Position & position, double resolution) const;

  mutable std::mutex mutex_;
  nanogrid::GridMap latest_map_;
  bool have_map_{false};
  bool clear_requested_{false};
  bool full_bounds_requested_{false};
  bool last_update_fresh_{false};
  rclcpp::Time last_message_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr terrain_status_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<nav2_msgs::msg::Costmap>::SharedPtr
    diagnostic_costmap_publisher_;
  std::string terrain_status_topic_{"/fastdem/terrain_valid"};
  bool have_published_terrain_status_{false};
  bool terrain_status_{false};
  std::string global_frame_{"odom"};
  double map_timeout_{0.5};
  TerrainCostConfig cost_config_;
  bool slope_hard_constraint_{true};
  bool residual_step_hard_constraint_{true};
  bool roughness_hard_constraint_{true};
  int min_observations_{3};
  int max_observation_age_frames_{3};
  // Minimum lethal hits and maximum missing frames in the recent window.
  int lethal_confirmation_frames_{3};
  int lethal_confirmation_window_frames_{5};
  int lethal_clear_frames_{2};
  double lethal_max_age_{2.0};
  std::uint64_t confirmation_history_mask_{31};
  std::int64_t lethal_max_age_ns_{2000000000LL};
  int min_confident_cells_{20};
  double max_lethal_fraction_{0.25};
  double max_future_stamp_offset_{0.1};
  std::unordered_map<WorldCell, ConfirmationState, WorldCellHash> confirmations_;
  bool have_source_geometry_{false};
  int source_rows_{0};
  int source_cols_{0};
  double source_resolution_{0.0};
  double source_length_x_{0.0};
  double source_length_y_{0.0};
  bool have_source_stamp_{false};
  std::uint64_t last_source_stamp_ns_{0};
};
}  // namespace nav2_terrain_layer

#endif  // NAV2_TERRAIN_LAYER__TERRAIN_LAYER_HPP_
