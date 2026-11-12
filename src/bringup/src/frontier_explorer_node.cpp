#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <iterator>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <action_msgs/msg/goal_status_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <map_msgs/msg/occupancy_grid_update.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include "bringup/chassis_driver.hpp"
#include "bringup/frontier_selection.hpp"
#include "bringup/status_heartbeat.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class FrontierExplorerNode : public rclcpp::Node
{
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;
  using Clock = std::chrono::steady_clock;
  using GoalStatus = action_msgs::msg::GoalStatus;

public:
  FrontierExplorerNode()
  : Node("frontier_explorer"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
  {
    map_topic_ = declare_parameter<std::string>("map_topic", "/map");
    map_updates_topic_ = declare_parameter<std::string>("map_updates_topic", "/map_updates");
    costmap_topic_ = declare_parameter<std::string>("costmap_topic", "/global_costmap/costmap");
    prior_map_topic_ = declare_parameter<std::string>("prior_map_topic", "/map_prior");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    action_name_ = declare_parameter<std::string>("action_name", "navigate_to_pose");
    navigation_ready_topic_ = declare_parameter<std::string>(
      "navigation_ready_topic", "/navigation_ready");
    health_topic_ = declare_parameter<std::string>("health_topic", "/fastlio/health");
    observation_topic_ = declare_parameter<std::string>(
      "observation_topic", "/obstacles/blocking");
    terrain_topic_ = declare_parameter<std::string>(
      "terrain_topic", "/fastdem/postprocess/gridmap");
    status_topic_ = declare_parameter<std::string>(
      "status_topic", "/navigation/frontier_exploration_status");
    stop_service_ = declare_parameter<std::string>(
      "stop_service", "/navigation/frontier_exploration/stop");
    resume_service_ = declare_parameter<std::string>(
      "resume_service", "/navigation/frontier_exploration/resume");
    min_frontier_cells_ = declare_parameter<int>("min_frontier_cells", 4);
    search_period_ = declare_parameter<double>("search_period", 2.0);
    tf_timeout_ = declare_parameter<double>("tf_timeout", 0.10);
    exclusion_radius_ = declare_parameter<double>("exclusion_radius", 0.60);
    max_map_extension_m_ = declare_parameter<double>("max_map_extension_m", 20.0);
    require_terrain_ = declare_parameter<bool>("require_terrain", true);
    terrain_config_.frame_id = declare_parameter<std::string>("terrain_frame", "odom");
    terrain_config_.max_age = declare_parameter<double>("terrain_max_age", 1.5);
    terrain_config_.max_future_stamp_offset = declare_parameter<double>(
      "terrain_max_future_stamp_offset", 0.1);
    terrain_config_.min_observations = declare_parameter<int>("terrain_min_observations", 3);
    terrain_config_.max_observation_age_frames = declare_parameter<int>(
      "terrain_max_observation_age_frames", 3);
    terrain_config_.min_confident_cells = declare_parameter<int>(
      "terrain_min_confident_cells", 20);
    terrain_config_.max_lethal_fraction = declare_parameter<double>(
      "terrain_max_lethal_fraction", 0.90);
    terrain_config_.slope_lethal = declare_parameter<double>(
      "terrain_slope_lethal_threshold", 35.0);
    terrain_config_.residual_step_lethal = declare_parameter<double>(
      "terrain_residual_step_lethal_threshold", 0.25);
    terrain_config_.roughness_lethal = declare_parameter<double>(
      "terrain_roughness_lethal_threshold", 0.25);
    const auto footprint_vertices = declare_parameter<std::vector<double>>(
      "robot_footprint", {0.275, 0.225, 0.275, -0.225,
        -0.35, -0.225, -0.35, 0.225});
    if (map_topic_.empty() || map_updates_topic_.empty() || costmap_topic_.empty() ||
      prior_map_topic_.empty() || base_frame_.empty() || action_name_.empty() ||
      navigation_ready_topic_.empty() || health_topic_.empty() || observation_topic_.empty() ||
      terrain_topic_.empty() || status_topic_.empty() || stop_service_.empty() ||
      resume_service_.empty() || min_frontier_cells_ < 1 ||
      !std::isfinite(search_period_) || search_period_ <= 0.0 ||
      !std::isfinite(tf_timeout_) || tf_timeout_ <= 0.0 ||
      !std::isfinite(exclusion_radius_) || exclusion_radius_ < 0.0 ||
      !std::isfinite(max_map_extension_m_) || max_map_extension_m_ < 0.0 ||
      terrain_config_.frame_id.empty() || !std::isfinite(terrain_config_.max_age) ||
      terrain_config_.max_age <= 0.0 ||
      footprint_vertices.size() < 6 || footprint_vertices.size() % 2 != 0)
    {
      throw std::invalid_argument("frontier explorer parameters are invalid");
    }
    double footprint_area = 0.0;
    for (std::size_t i = 0; i < footprint_vertices.size(); i += 2) {
      const bringup::FrontierFootprintPoint point{
        footprint_vertices[i], footprint_vertices[i + 1]};
      if (!std::isfinite(point.x) || !std::isfinite(point.y)) {
        throw std::invalid_argument("robot_footprint contains a non-finite coordinate");
      }
      footprint_.push_back(point);
    }
    for (std::size_t i = 0; i < footprint_.size(); ++i) {
      const auto & current = footprint_[i];
      const auto & next = footprint_[(i + 1) % footprint_.size()];
      footprint_area += current.x * next.y - next.x * current.y;
    }
    if (!std::isfinite(footprint_area) || std::abs(footprint_area) <= 1.0e-9) {
      throw std::invalid_argument("robot_footprint must enclose a non-zero area");
    }

    status_pub_ = create_publisher<std_msgs::msg::String>(
      status_topic_, rclcpp::QoS(1).reliable().transient_local());
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      map_topic_, rclcpp::QoS(1).reliable().transient_local(),
      [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr message) {
        if (!validGrid(*message)) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Ignoring invalid map snapshot");
          return;
        }
        if (!map_ || !sameMapSnapshot(*map_, *message)) {
          ++map_revision_;
          if (map_ && awaiting_map_update_) {
            recordNewlyKnownFreeCells(*map_, *message);
          }
          map_received_ = Clock::now();
        }
        map_ = *message;
        for (const auto & update : pending_map_updates_) {
          applyMapUpdateToGrid(update);
        }
        pending_map_updates_.clear();
      });
    map_updates_sub_ = create_subscription<map_msgs::msg::OccupancyGridUpdate>(
      map_updates_topic_, rclcpp::QoS(10).reliable(),
      [this](map_msgs::msg::OccupancyGridUpdate::ConstSharedPtr update) {
        applyMapUpdate(*update);
      });
    costmap_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      costmap_topic_, rclcpp::QoS(1).reliable().transient_local(),
      [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr message) {
        if (!validGrid(*message)) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 5000, "Ignoring invalid global costmap snapshot");
          return;
        }
        global_costmap_ = *message;
        global_costmap_received_ = Clock::now();
        for (const auto & update : pending_costmap_updates_) {
          applyCostmapUpdateToGrid(update);
        }
        pending_costmap_updates_.clear();
      });
    costmap_updates_sub_ = create_subscription<map_msgs::msg::OccupancyGridUpdate>(
      costmap_topic_ + "_updates", rclcpp::QoS(10).reliable(),
      [this](map_msgs::msg::OccupancyGridUpdate::ConstSharedPtr update) {
        applyCostmapUpdate(*update);
      });
    prior_map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      prior_map_topic_, rclcpp::QoS(1).reliable().transient_local(),
      [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr message) {
        if (!validGrid(*message)) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 5000, "Ignoring invalid prior map snapshot");
          return;
        }
        prior_map_ = *message;
      });
    navigation_ready_sub_ = create_subscription<std_msgs::msg::Bool>(
      navigation_ready_topic_, rclcpp::QoS(1).reliable().transient_local(),
      [this](std_msgs::msg::Bool::ConstSharedPtr message) {
        navigation_ready_ = message->data;
        have_navigation_ready_ = true;
        navigation_ready_received_ = Clock::now();
        if (!navigation_ready_ && goal_in_flight_) {
          requestGoalCancel("导航门控已暂停");
          publishStatus("导航门控已暂停；正在结束探索目标");
        }
      });
    health_sub_ = create_subscription<std_msgs::msg::Bool>(
      health_topic_, rclcpp::QoS(1).best_effort(),
      [this](std_msgs::msg::Bool::ConstSharedPtr message) {
        healthy_ = message->data;
        have_health_ = true;
        health_received_ = Clock::now();
      });
    observation_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      observation_topic_, rclcpp::QoS(1).best_effort(),
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr message) {
        have_observation_ = true;
        ++observation_count_;
        observation_received_ = Clock::now();
      });
    terrain_sub_ = create_subscription<grid_map_msgs::msg::GridMap>(
      terrain_topic_, rclcpp::QoS(1).reliable(),
      [this](grid_map_msgs::msg::GridMap::ConstSharedPtr message) {
        const auto elevation_layer = std::find(
          message->layers.begin(), message->layers.end(), "elevation");
        const auto elevation_index = elevation_layer == message->layers.end() ?
          message->data.size() : static_cast<std::size_t>(
          std::distance(message->layers.begin(), elevation_layer));
        const bool has_elevation = elevation_index < message->data.size() &&
          std::any_of(message->data[elevation_index].data.begin(),
          message->data[elevation_index].data.end(), [](float value) {
            return std::isfinite(value);
          });
        if (!has_elevation || !bringup::terrain_message_is_current(
            *message, get_clock()->now().nanoseconds(), terrain_config_))
        {
          have_terrain_ = false;
          terrain_message_.reset();
          return;
        }

        const auto stamp_ns = static_cast<std::int64_t>(message->header.stamp.sec) *
          1000000000LL + static_cast<std::int64_t>(message->header.stamp.nanosec);
        if (have_terrain_stamp_) {
          if (stamp_ns < last_terrain_stamp_ns_) {
            have_terrain_ = false;
            terrain_message_.reset();
            return;
          }
          if (stamp_ns == last_terrain_stamp_ns_) return;
        }

        last_terrain_stamp_ns_ = stamp_ns;
        have_terrain_stamp_ = true;
        terrain_message_ = std::move(message);
        have_terrain_ = true;
        terrain_received_ = Clock::now();
      });

    action_client_ = rclcpp_action::create_client<NavigateToPose>(this, action_name_);
    addGoalStatusMonitor(actionNameBase(action_name_), GoalAction::NavigateToPose);
    addGoalStatusMonitor("/follow_waypoints", GoalAction::FollowWaypoints);
    addGoalStatusMonitor("/navigate_through_poses", GoalAction::NavigateThroughPoses);

    stop_service_server_ = create_service<std_srvs::srv::Trigger>(
      stop_service_, [this](
        const std_srvs::srv::Trigger::Request::SharedPtr,
        std_srvs::srv::Trigger::Response::SharedPtr response)
      {
        operator_paused_ = true;
        operator_pause_reason_ = "用户停止探索";
        requestGoalCancel(operator_pause_reason_);
        publishStatus(goal_in_flight_ ? "正在取消探索目标：用户停止探索" : "探索已停止");
        response->success = true;
        response->message = goal_in_flight_ ?
          "已暂停派发，并请求取消探索目标" : "探索已停止";
      });
    resume_service_server_ = create_service<std_srvs::srv::Trigger>(
      resume_service_, [this](
        const std_srvs::srv::Trigger::Request::SharedPtr,
        std_srvs::srv::Trigger::Response::SharedPtr response)
      {
        if (externalGoalActive()) {
          response->success = false;
          response->message = "外部导航目标仍在执行；结束该目标后再恢复探索";
          return;
        }
        const auto gate_reason = dataGateFailure();
        if (!gate_reason.empty()) {
          response->success = false;
          response->message = gate_reason;
          return;
        }
        if (goal_in_flight_) {
          response->success = false;
          response->message = "探索目标仍在取消或执行；收到终态后再恢复探索";
          return;
        }
        operator_paused_ = false;
        operator_pause_reason_.clear();
        response->success = true;
        response->message = "前沿探索已恢复";
        publishStatus(response->message);
      });

    search_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(search_period_)),
      std::bind(&FrontierExplorerNode::searchAndNavigate, this));
    gate_timer_ = create_wall_timer(
      std::chrono::milliseconds(100), std::bind(&FrontierExplorerNode::checkDataGate, this));
    publishStatus("等待地图、定位、感知和导航门控状态");
    RCLCPP_INFO(
      get_logger(), "Frontier exploration ready; dispatch remains gated by fresh localization, "
      "perception, FastDEM, and navigation status");
  }

private:
  enum class GoalAction
  {
    NavigateToPose,
    FollowWaypoints,
    NavigateThroughPoses
  };

  static bool validGrid(const nav_msgs::msg::OccupancyGrid & grid)
  {
    return grid.info.width > 0 && grid.info.height > 0 &&
           std::isfinite(grid.info.resolution) && grid.info.resolution > 0.0F &&
           grid.data.size() == static_cast<std::size_t>(grid.info.width) * grid.info.height;
  }

  static bool sameMapSnapshot(
    const nav_msgs::msg::OccupancyGrid & lhs,
    const nav_msgs::msg::OccupancyGrid & rhs)
  {
    return lhs.header.frame_id == rhs.header.frame_id &&
           lhs.info.width == rhs.info.width && lhs.info.height == rhs.info.height &&
           lhs.info.resolution == rhs.info.resolution &&
           lhs.info.origin.position.x == rhs.info.origin.position.x &&
           lhs.info.origin.position.y == rhs.info.origin.position.y &&
           lhs.info.origin.position.z == rhs.info.origin.position.z &&
           lhs.info.origin.orientation.x == rhs.info.origin.orientation.x &&
           lhs.info.origin.orientation.y == rhs.info.origin.orientation.y &&
           lhs.info.origin.orientation.z == rhs.info.origin.orientation.z &&
           lhs.info.origin.orientation.w == rhs.info.origin.orientation.w &&
           lhs.data == rhs.data;
  }

  static std::string actionNameBase(const std::string & action_name)
  {
    return !action_name.empty() && action_name.front() == '/' ? action_name : "/" + action_name;
  }

  void addGoalStatusMonitor(const std::string & action_name, GoalAction kind)
  {
    goal_status_subs_.push_back(create_subscription<action_msgs::msg::GoalStatusArray>(
      action_name + "/_action/status", rclcpp::QoS(1).reliable().transient_local(),
      [this, kind](action_msgs::msg::GoalStatusArray::ConstSharedPtr message) {
        processGoalStatuses(kind, *message);
      }));
  }

  void processGoalStatuses(
    GoalAction kind, const action_msgs::msg::GoalStatusArray & message)
  {
    bool external_active = false;
    for (const auto & status : message.status_list) {
      if (status.status != GoalStatus::STATUS_ACCEPTED &&
        status.status != GoalStatus::STATUS_EXECUTING)
      {
        continue;
      }
      const auto & goal_id = status.goal_info.goal_id.uuid;
      if (kind == GoalAction::NavigateToPose &&
        ((goal_handle_ && goal_handle_->get_goal_id() == goal_id) ||
        (have_last_explorer_goal_id_ && last_explorer_goal_id_ == goal_id)))
      {
        continue;
      }
      if (kind == GoalAction::NavigateToPose && goal_response_pending_ && !goal_handle_) {
        continue;
      }
      external_active = true;
      break;
    }

    switch (kind) {
      case GoalAction::NavigateToPose:
        external_navigate_to_pose_active_ = external_active;
        latest_navigate_to_pose_statuses_ = message.status_list;
        break;
      case GoalAction::FollowWaypoints:
        external_follow_waypoints_active_ = external_active;
        break;
      case GoalAction::NavigateThroughPoses:
        external_navigate_through_poses_active_ = external_active;
        break;
    }
    if (externalGoalActive() && !operator_paused_) {
      operator_paused_ = true;
      operator_pause_reason_ = "检测到外部导航目标，探索已让出导航";
      requestGoalCancel(operator_pause_reason_);
      publishStatus(goal_in_flight_ ? "用户目标接管，正在取消探索目标" : operator_pause_reason_);
    }
  }

  bool externalGoalActive() const
  {
    return external_navigate_to_pose_active_ || external_follow_waypoints_active_ ||
           external_navigate_through_poses_active_;
  }

  void refreshExternalNavigateGoalState()
  {
    action_msgs::msg::GoalStatusArray message;
    message.status_list = latest_navigate_to_pose_statuses_;
    processGoalStatuses(GoalAction::NavigateToPose, message);
  }

  void applyMapUpdate(const map_msgs::msg::OccupancyGridUpdate & update)
  {
    if (!map_) {
      pending_map_updates_.push_back(update);
      return;
    }
    applyMapUpdateToGrid(update);
  }

  void applyMapUpdateToGrid(const map_msgs::msg::OccupancyGridUpdate & update)
  {
    auto & grid = *map_;
    if (!validGrid(grid) || update.header.frame_id != grid.header.frame_id ||
      update.width == 0 || update.height == 0 || update.x > grid.info.width ||
      update.y > grid.info.height || update.width > grid.info.width - update.x ||
      update.height > grid.info.height - update.y ||
      update.data.size() != static_cast<std::size_t>(update.width) * update.height)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Ignoring invalid /map update (frame, bounds, or data size mismatch)");
      return;
    }
    bool changed = false;
    for (std::uint32_t row = 0; row < update.height; ++row) {
      for (std::uint32_t col = 0; col < update.width; ++col) {
        const auto source = static_cast<std::size_t>(row) * update.width + col;
        const auto target = static_cast<std::size_t>(update.y + row) * grid.info.width +
          update.x + col;
        if (awaiting_map_update_ && grid.data[target] < 0 &&
          update.data[source] >= 0 && update.data[source] <= 50 &&
          recently_promoted_free_cells_.size() < kMaxTrackedPromotedCells)
        {
          const auto world = bringup::detail::gridCellCenter(
            grid, update.x + col, update.y + row);
          recently_promoted_free_cells_.emplace_back(world.x(), world.y());
        }
        if (grid.data[target] != update.data[source]) {
          grid.data[target] = update.data[source];
          changed = true;
        }
      }
    }
    if (changed) {
      ++map_revision_;
      map_received_ = Clock::now();
    }
  }

  void recordNewlyKnownFreeCells(
    const nav_msgs::msg::OccupancyGrid & previous,
    const nav_msgs::msg::OccupancyGrid & current)
  {
    if (!awaiting_map_update_) return;
    for (std::uint32_t y = 0; y < current.info.height; ++y) {
      for (std::uint32_t x = 0; x < current.info.width; ++x) {
        const auto current_index = static_cast<std::size_t>(y) * current.info.width + x;
        if (current.data[current_index] < 0 || current.data[current_index] > 50) continue;
        const auto world = bringup::detail::gridCellCenter(current, x, y);
        const auto previous_cell = bringup::detail::gridCellAtWorld(
          previous, world.x(), world.y());
        bool previously_unknown = !previous_cell ||
          !bringup::detail::inside(previous, *previous_cell);
        if (!previously_unknown) {
          const auto previous_index = static_cast<std::size_t>(previous_cell->second) *
            previous.info.width + static_cast<std::size_t>(previous_cell->first);
          previously_unknown = previous.data[previous_index] < 0;
        }
        if (previously_unknown && recently_promoted_free_cells_.size() <
          kMaxTrackedPromotedCells)
        {
          recently_promoted_free_cells_.emplace_back(world.x(), world.y());
        }
      }
    }
  }

  bool mapUpdateReachedCostmap() const
  {
    if (recently_promoted_free_cells_.empty()) return true;
    bool found_overlapping_cell = false;
    for (const auto & point : recently_promoted_free_cells_) {
      const auto cell = bringup::detail::gridCellAtWorld(
        *global_costmap_, point.first, point.second);
      if (!cell || !bringup::detail::inside(*global_costmap_, *cell)) continue;
      found_overlapping_cell = true;
      const auto index = static_cast<std::size_t>(cell->second) *
        global_costmap_->info.width + static_cast<std::size_t>(cell->first);
      if (global_costmap_->data[index] < 0) return false;
    }
    return found_overlapping_cell;
  }

  void applyCostmapUpdate(const map_msgs::msg::OccupancyGridUpdate & update)
  {
    if (!global_costmap_) {
      pending_costmap_updates_.push_back(update);
      return;
    }
    applyCostmapUpdateToGrid(update);
  }

  void applyCostmapUpdateToGrid(const map_msgs::msg::OccupancyGridUpdate & update)
  {
    auto & grid = *global_costmap_;
    const auto update_width = static_cast<std::size_t>(update.width);
    const auto update_height = static_cast<std::size_t>(update.height);
    if (!validGrid(grid) || update.header.frame_id != grid.header.frame_id ||
      update.width == 0 || update.height == 0 || update.x > grid.info.width ||
      update.y > grid.info.height || update.width > grid.info.width - update.x ||
      update.height > grid.info.height - update.y ||
      update_height > std::numeric_limits<std::size_t>::max() / update_width ||
      update.data.size() != update_width * update_height)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Ignoring invalid global costmap update (frame, bounds, or data size mismatch)");
      return;
    }
    for (std::uint32_t row = 0; row < update.height; ++row) {
      const auto source = static_cast<std::size_t>(row) * update_width;
      const auto target = static_cast<std::size_t>(update.y + row) * grid.info.width + update.x;
      std::copy_n(update.data.begin() + static_cast<std::ptrdiff_t>(source), update.width,
        grid.data.begin() + static_cast<std::ptrdiff_t>(target));
    }
    global_costmap_received_ = Clock::now();
  }

  std::string dataGateFailure()
  {
    const auto now = Clock::now();
    const double navigation_ready_age =
      std::chrono::duration<double>(now - navigation_ready_received_).count();
    if (!bringup::navigationReadyHeartbeatIsFresh(
        have_navigation_ready_, navigation_ready_age))
    {
      return "导航门控状态未收到或已超时";
    }
    if (!navigation_ready_) return "导航门控已暂停";
    if (!have_health_ || !healthy_ ||
      std::chrono::duration<double>(now - health_received_).count() > 0.5)
    {
      return "定位不可用或健康状态超时";
    }
    if (!have_observation_ ||
      std::chrono::duration<double>(now - observation_received_).count() > 1.0)
    {
      return "感知观测不可用或已超时";
    }
    const auto terrain_receive_age =
      std::chrono::duration<double>(now - terrain_received_).count();
    const bool terrain_fresh = have_terrain_ && terrain_message_ &&
      terrain_receive_age >= 0.0 && terrain_receive_age <= terrain_config_.max_age &&
      bringup::terrain_message_is_current(
        *terrain_message_, get_clock()->now().nanoseconds(), terrain_config_);
    if (require_terrain_ && !terrain_fresh) {
      return "FastDEM 地形数据不可用或已超时";
    }
    if (!global_costmap_ ||
      std::chrono::duration<double>(now - global_costmap_received_).count() > 2.0)
    {
      return "全局代价地图不可用或已超时";
    }
    return {};
  }

  void checkDataGate()
  {
    if (operator_paused_) {
      publishStatus("探索已暂停：" + operator_pause_reason_);
      return;
    }
    const auto reason = dataGateFailure();
    if (!reason.empty()) {
      if (goal_in_flight_) requestGoalCancel(reason);
      publishStatus(goal_in_flight_ ?
        "数据门控暂停派发：" + reason + "；正在结束探索目标" :
        "数据门控暂停派发：" + reason);
    }
  }

  void searchAndNavigate()
  {
    if (operator_paused_) {
      publishStatus("探索已暂停：" + operator_pause_reason_);
      return;
    }
    const auto gate_reason = dataGateFailure();
    if (!gate_reason.empty()) {
      if (goal_in_flight_) requestGoalCancel(gate_reason);
      publishStatus("数据门控暂停派发：" + gate_reason);
      return;
    }
    if (goal_in_flight_) return;
    if (!map_ || !global_costmap_ || !prior_map_) {
      publishStatus("等待 /map、全局代价地图和先验地图");
      return;
    }
    if (!action_client_->action_server_is_ready()) {
      publishStatus("Nav2 导航服务不可用");
      return;
    }
    if (awaiting_map_update_) {
      const bool new_observation = observation_count_ > observations_at_arrival_;
      const bool map_updated = map_revision_ > map_revision_at_arrival_ &&
        map_received_ > map_received_at_arrival_;
      const bool costmap_synchronized = map_updated &&
        global_costmap_received_ > map_received_ && mapUpdateReachedCostmap();
      if (!new_observation || !map_updated || !costmap_synchronized) {
        const auto status = !new_observation ? "已到达前沿，等待新感知观测" :
          !map_updated ? "已收到新感知观测，等待 /map 内容或范围更新" :
          "/map 已更新，等待全局代价地图同步新已知区域";
        publishStatus(status);
        return;
      }
      awaiting_map_update_ = false;
      recently_promoted_free_cells_.clear();
      publishStatus("前沿区域已获得新观测、地图更新和代价地图同步，继续搜索");
    }

    geometry_msgs::msg::TransformStamped pose;
    try {
      pose = tf_buffer_.lookupTransform(
        map_->header.frame_id, base_frame_, tf2::TimePointZero,
        tf2::durationFromSec(tf_timeout_));
    } catch (const tf2::TransformException & error) {
      publishStatus(std::string("定位不可用：等待 map→") + base_frame_ + " TF");
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000, "Waiting for robot pose in map frame: %s", error.what());
      return;
    }

    const auto candidate = bringup::selectFrontierGoal(
      *map_, *global_costmap_, *prior_map_,
      pose.transform.translation.x, pose.transform.translation.y,
      static_cast<std::size_t>(min_frontier_cells_), max_map_extension_m_,
      recent_goals_, exclusion_radius_, footprint_);
    if (!candidate) {
      publishStatus("无可达前沿；等待新地图或新的可达候选");
      return;
    }

    NavigateToPose::Goal goal;
    goal.pose.header.frame_id = map_->header.frame_id;
    goal.pose.header.stamp = now();
    goal.pose.pose.position.x = candidate->x;
    goal.pose.pose.position.y = candidate->y;
    goal.pose.pose.position.z = 0.0;
    const tf2::Quaternion orientation(
      tf2::Vector3(0.0, 0.0, 1.0), candidate->yaw);
    goal.pose.pose.orientation = tf2::toMsg(orientation);

    goal_in_flight_ = true;
    goal_response_pending_ = true;
    cancel_requested_ = false;
    cancel_request_sent_ = false;
    cancel_reason_.clear();
    publishStatus("正在前往已知空闲侧前沿 (" + format(candidate->x) + "," +
      format(candidate->y) + ")");
    auto options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    options.goal_response_callback = [this, candidate](GoalHandle::SharedPtr handle) {
        goal_response_pending_ = false;
        if (!handle) {
          goal_in_flight_ = false;
          rememberGoal(candidate->x, candidate->y);
          if (operator_paused_) {
            publishStatus("探索已暂停：" + operator_pause_reason_);
          } else {
            publishStatus("Nav2 拒绝该前沿目标，尝试附近其他候选");
          }
          refreshExternalNavigateGoalState();
          return;
        }
        goal_handle_ = handle;
        last_explorer_goal_id_ = handle->get_goal_id();
        have_last_explorer_goal_id_ = true;
        rememberGoal(candidate->x, candidate->y);
        refreshExternalNavigateGoalState();
        if (operator_paused_) {
          requestGoalCancel(operator_pause_reason_);
        } else {
          const auto gate_reason = dataGateFailure();
          if (!gate_reason.empty()) requestGoalCancel(gate_reason);
        }
        if (!cancel_requested_) {
          publishStatus("正在探索前沿 (" + format(candidate->x) + "," +
            format(candidate->y) + ")");
        }
      };
    options.result_callback = [this, candidate](const GoalHandle::WrappedResult & result) {
        if (goal_handle_ && goal_handle_->get_goal_id() != result.goal_id) return;
        goal_handle_.reset();
        goal_in_flight_ = false;
        goal_response_pending_ = false;
        const bool canceled_by_node = cancel_requested_;
        const auto cancel_reason = cancel_reason_;
        cancel_requested_ = false;
        cancel_request_sent_ = false;
        cancel_reason_.clear();
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
          awaiting_map_update_ = true;
          map_revision_at_arrival_ = map_revision_;
          map_received_at_arrival_ = map_received_;
          observations_at_arrival_ = observation_count_;
          recently_promoted_free_cells_.clear();
          publishStatus("已到达前沿，等待新感知观测和 /map 更新");
        } else if (result.code == rclcpp_action::ResultCode::CANCELED &&
          !canceled_by_node && !operator_paused_)
        {
          operator_paused_ = true;
          operator_pause_reason_ = "探索目标被用户取消";
          publishStatus("探索已暂停：探索目标被用户取消");
        } else if (operator_paused_) {
          publishStatus("探索已暂停：" + operator_pause_reason_ + "；探索目标已结束");
        } else if (canceled_by_node) {
          publishStatus("探索目标已结束：" + cancel_reason);
        } else {
          publishStatus("前沿目标失败，已排除该位置附近并尝试其他候选");
          RCLCPP_WARN(
            get_logger(), "Frontier goal at (%.2f, %.2f) ended with result code %d",
            candidate->x, candidate->y, static_cast<int>(result.code));
        }
      };
    try {
      action_client_->async_send_goal(goal, options);
    } catch (const std::exception & error) {
      goal_in_flight_ = false;
      goal_response_pending_ = false;
      rememberGoal(candidate->x, candidate->y);
      publishStatus(std::string("无法提交前沿目标：") + error.what());
    }
  }

  void requestGoalCancel(const std::string & reason)
  {
    if (!goal_in_flight_) return;
    if (!cancel_requested_) {
      cancel_requested_ = true;
      cancel_reason_ = reason;
    }
    if (!goal_handle_ || cancel_request_sent_) return;
    cancel_request_sent_ = true;
    const auto cancel_future = action_client_->async_cancel_goal(goal_handle_);
    (void)cancel_future;
    publishStatus("正在取消探索目标：" + cancel_reason_);
  }

  void rememberGoal(double x, double y)
  {
    const bool already_excluded = std::any_of(
      recent_goals_.begin(), recent_goals_.end(), [&](const auto & previous) {
        const double dx = x - previous.first;
        const double dy = y - previous.second;
        return dx * dx + dy * dy < 1.0e-6;
      });
    if (already_excluded) return;
    recent_goals_.emplace_back(x, y);
    constexpr std::size_t kMaxRememberedGoals = 24;
    if (recent_goals_.size() > kMaxRememberedGoals) recent_goals_.erase(recent_goals_.begin());
  }

  static std::string format(double value)
  {
    char buffer[32];
    std::snprintf(buffer, sizeof(buffer), "%.2f", value);
    return buffer;
  }

  void publishStatus(const std::string & status)
  {
    if (status == last_status_) return;
    last_status_ = status;
    std_msgs::msg::String message;
    message.data = status;
    status_pub_->publish(message);
    RCLCPP_INFO(get_logger(), "Frontier exploration: %s", status.c_str());
  }

  std::string map_topic_;
  std::string map_updates_topic_;
  std::string costmap_topic_;
  std::string prior_map_topic_;
  std::string base_frame_;
  std::string action_name_;
  std::string navigation_ready_topic_;
  std::string health_topic_;
  std::string observation_topic_;
  std::string terrain_topic_;
  std::string status_topic_;
  std::string stop_service_;
  std::string resume_service_;
  std::string last_status_;
  std::string operator_pause_reason_;
  std::string cancel_reason_;
  int min_frontier_cells_{4};
  double search_period_{2.0};
  double tf_timeout_{0.10};
  double exclusion_radius_{0.60};
  double max_map_extension_m_{20.0};
  bringup::TerrainGateConfig terrain_config_;
  bool require_terrain_{true};
  bool navigation_ready_{false};
  bool have_navigation_ready_{false};
  bool healthy_{false};
  bool have_health_{false};
  bool have_observation_{false};
  bool have_terrain_{false};
  bool have_terrain_stamp_{false};
  bool goal_in_flight_{false};
  bool goal_response_pending_{false};
  bool cancel_requested_{false};
  bool cancel_request_sent_{false};
  bool operator_paused_{false};
  bool external_navigate_to_pose_active_{false};
  bool external_follow_waypoints_active_{false};
  bool external_navigate_through_poses_active_{false};
  bool have_last_explorer_goal_id_{false};
  bool awaiting_map_update_{false};
  std::uint64_t map_revision_{0};
  std::uint64_t map_revision_at_arrival_{0};
  std::uint64_t observation_count_{0};
  std::uint64_t observations_at_arrival_{0};
  std::int64_t last_terrain_stamp_ns_{0};
  static constexpr std::size_t kMaxTrackedPromotedCells = 4096;
  std::optional<nav_msgs::msg::OccupancyGrid> map_;
  std::optional<nav_msgs::msg::OccupancyGrid> global_costmap_;
  std::optional<nav_msgs::msg::OccupancyGrid> prior_map_;
  grid_map_msgs::msg::GridMap::ConstSharedPtr terrain_message_;
  std::vector<map_msgs::msg::OccupancyGridUpdate> pending_map_updates_;
  std::vector<map_msgs::msg::OccupancyGridUpdate> pending_costmap_updates_;
  std::vector<std::pair<double, double>> recent_goals_;
  std::vector<std::pair<double, double>> recently_promoted_free_cells_;
  std::vector<bringup::FrontierFootprintPoint> footprint_;
  std::vector<GoalStatus> latest_navigate_to_pose_statuses_;
  rclcpp_action::GoalUUID last_explorer_goal_id_{};
  Clock::time_point navigation_ready_received_{};
  Clock::time_point map_received_{};
  Clock::time_point map_received_at_arrival_{};
  Clock::time_point health_received_{};
  Clock::time_point observation_received_{};
  Clock::time_point terrain_received_{};
  Clock::time_point global_costmap_received_{};
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<map_msgs::msg::OccupancyGridUpdate>::SharedPtr map_updates_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
  rclcpp::Subscription<map_msgs::msg::OccupancyGridUpdate>::SharedPtr costmap_updates_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr prior_map_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr navigation_ready_sub_, health_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr observation_sub_;
  rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr terrain_sub_;
  std::vector<rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr>
    goal_status_subs_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service_server_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr resume_service_server_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
  GoalHandle::SharedPtr goal_handle_;
  rclcpp::TimerBase::SharedPtr search_timer_, gate_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FrontierExplorerNode>());
  rclcpp::shutdown();
  return 0;
}
