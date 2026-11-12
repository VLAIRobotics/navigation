#include "nav2_terrain_layer/terrain_layer.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <unordered_set>
#include <vector>

#include <nanogrid/bridge/ros2.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace nav2_terrain_layer
{
namespace
{
int hitCount(std::uint64_t history)
{
  int count = 0;
  while (history != 0) {
    history &= history - 1;
    ++count;
  }
  return count;
}

bool validGridMapGeometry(const grid_map_msgs::msg::GridMap & message)
{
  const auto & orientation = message.info.pose.orientation;
  const double quaternion_norm_squared =
    orientation.x * orientation.x + orientation.y * orientation.y +
    orientation.z * orientation.z + orientation.w * orientation.w;
  constexpr double kGeometryTolerance = 1e-6;
  if (message.layers.size() != message.data.size() ||
    !std::isfinite(message.info.resolution) || message.info.resolution <= 0.0 ||
    !std::isfinite(message.info.length_x) || !std::isfinite(message.info.length_y) ||
    message.info.length_x <= 0.0 || message.info.length_y <= 0.0 ||
    !std::isfinite(message.info.pose.position.x) ||
    !std::isfinite(message.info.pose.position.y) ||
    !std::isfinite(message.info.pose.position.z) ||
    message.info.pose.position.z != 0.0 ||
    !std::isfinite(quaternion_norm_squared) ||
    std::abs(quaternion_norm_squared - 1.0) > kGeometryTolerance ||
    std::abs(orientation.x) > kGeometryTolerance ||
    std::abs(orientation.y) > kGeometryTolerance ||
    std::abs(orientation.z) > kGeometryTolerance ||
    std::abs(std::abs(orientation.w) - 1.0) > kGeometryTolerance)
  {
    return false;
  }
  const auto rows = std::llround(message.info.length_x / message.info.resolution);
  const auto cols = std::llround(message.info.length_y / message.info.resolution);
  if (rows < 1 || cols < 1 || rows > std::numeric_limits<int>::max() ||
    cols > std::numeric_limits<int>::max() ||
    message.outer_start_index >= static_cast<uint32_t>(rows) ||
    message.inner_start_index >= static_cast<uint32_t>(cols))
  {
    return false;
  }
  const auto cell_count = static_cast<std::size_t>(rows) * static_cast<std::size_t>(cols);
  std::unordered_set<std::string> layer_names;
  for (std::size_t i = 0; i < message.layers.size(); ++i) {
    const auto & data = message.data[i];
    if (!layer_names.insert(message.layers[i]).second || data.layout.dim.size() != 2 ||
      data.layout.data_offset != 0 ||
      data.layout.dim[0].label != "column_index" ||
      data.layout.dim[1].label != "row_index" ||
      data.layout.dim[0].size != static_cast<uint32_t>(cols) ||
      data.layout.dim[1].size != static_cast<uint32_t>(rows) ||
      data.layout.dim[0].stride != cell_count ||
      data.layout.dim[1].stride != static_cast<uint32_t>(rows) ||
      data.data.size() != cell_count)
    {
      return false;
    }
  }
  return true;
}
}  // namespace

void TerrainLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("TerrainLayer lifecycle node expired during initialization");
  }

  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("gridmap_topic", rclcpp::ParameterValue("/fastdem/postprocess/gridmap"));
  declareParameter("terrain_status_topic", rclcpp::ParameterValue("/fastdem/terrain_valid"));
  declareParameter("global_frame", rclcpp::ParameterValue("odom"));
  declareParameter("map_timeout", rclcpp::ParameterValue(0.5));
  declareParameter("slope_soft_threshold", rclcpp::ParameterValue(5.0));
  declareParameter("slope_lethal_threshold", rclcpp::ParameterValue(10.0));
  declareParameter("slope_hard_constraint", rclcpp::ParameterValue(true));
  declareParameter("residual_step_soft_threshold", rclcpp::ParameterValue(0.015));
  declareParameter("residual_step_lethal_threshold", rclcpp::ParameterValue(0.05));
  declareParameter("residual_step_hard_constraint", rclcpp::ParameterValue(true));
  declareParameter("roughness_soft_threshold", rclcpp::ParameterValue(0.03));
  declareParameter("roughness_lethal_threshold", rclcpp::ParameterValue(0.10));
  declareParameter("soft_cost_scale", rclcpp::ParameterValue(1.0));
  declareParameter("roughness_hard_constraint", rclcpp::ParameterValue(true));
  declareParameter("min_observations", rclcpp::ParameterValue(3));
  declareParameter("max_observation_age_frames", rclcpp::ParameterValue(3));
  declareParameter("lethal_confirmation_frames", rclcpp::ParameterValue(3));
  declareParameter("lethal_confirmation_window_frames", rclcpp::ParameterValue(5));
  declareParameter("lethal_clear_frames", rclcpp::ParameterValue(2));
  declareParameter("lethal_max_age", rclcpp::ParameterValue(2.0));
  declareParameter("min_confident_cells", rclcpp::ParameterValue(20));
  declareParameter("max_lethal_fraction", rclcpp::ParameterValue(0.25));
  declareParameter("max_future_stamp_offset", rclcpp::ParameterValue(0.1));

  node->get_parameter(getFullName("enabled"), enabled_);
  std::string topic;
  node->get_parameter(getFullName("gridmap_topic"), topic);
  node->get_parameter(getFullName("terrain_status_topic"), terrain_status_topic_);
  node->get_parameter(getFullName("global_frame"), global_frame_);
  node->get_parameter(getFullName("map_timeout"), map_timeout_);
  node->get_parameter(getFullName("slope_soft_threshold"), cost_config_.slope_soft);
  node->get_parameter(getFullName("slope_lethal_threshold"), cost_config_.slope_lethal);
  node->get_parameter(getFullName("slope_hard_constraint"), slope_hard_constraint_);
  node->get_parameter(
    getFullName("residual_step_soft_threshold"), cost_config_.residual_step_soft);
  node->get_parameter(
    getFullName("residual_step_lethal_threshold"), cost_config_.residual_step_lethal);
  node->get_parameter(
    getFullName("residual_step_hard_constraint"), residual_step_hard_constraint_);
  node->get_parameter(getFullName("roughness_soft_threshold"), cost_config_.roughness_soft);
  node->get_parameter(getFullName("roughness_lethal_threshold"), cost_config_.roughness_lethal);
  node->get_parameter(getFullName("soft_cost_scale"), cost_config_.soft_cost_scale);
  node->get_parameter(getFullName("roughness_hard_constraint"), roughness_hard_constraint_);
  node->get_parameter(getFullName("min_observations"), min_observations_);
  node->get_parameter(
    getFullName("max_observation_age_frames"), max_observation_age_frames_);
  node->get_parameter(
    getFullName("lethal_confirmation_frames"), lethal_confirmation_frames_);
  node->get_parameter(
    getFullName("lethal_confirmation_window_frames"), lethal_confirmation_window_frames_);
  node->get_parameter(getFullName("lethal_clear_frames"), lethal_clear_frames_);
  node->get_parameter(getFullName("lethal_max_age"), lethal_max_age_);
  node->get_parameter(getFullName("min_confident_cells"), min_confident_cells_);
  node->get_parameter(getFullName("max_lethal_fraction"), max_lethal_fraction_);
  node->get_parameter(
    getFullName("max_future_stamp_offset"), max_future_stamp_offset_);

  if (map_timeout_ <= 0.0) {
    throw std::runtime_error("TerrainLayer map_timeout must be greater than zero");
  }
  if (!std::isfinite(cost_config_.soft_cost_scale) || cost_config_.soft_cost_scale < 0.0 ||
    cost_config_.soft_cost_scale > 1.0 || min_observations_ < 1 ||
    max_observation_age_frames_ < 0 ||
    lethal_confirmation_frames_ < 1 || lethal_confirmation_window_frames_ < 1 ||
    lethal_confirmation_window_frames_ > 64 ||
    lethal_confirmation_frames_ > lethal_confirmation_window_frames_ ||
    lethal_clear_frames_ < 1 || lethal_clear_frames_ >= lethal_confirmation_window_frames_ ||
    min_confident_cells_ < 1 || max_lethal_fraction_ <= 0.0 || max_lethal_fraction_ > 1.0 ||
    !std::isfinite(lethal_max_age_) || lethal_max_age_ <= 0.0 ||
    lethal_max_age_ >=
    static_cast<double>(std::numeric_limits<std::int64_t>::max()) / 1e9 ||
    !std::isfinite(max_future_stamp_offset_) || max_future_stamp_offset_ < 0.0)
  {
    throw std::runtime_error("TerrainLayer confidence parameters are invalid");
  }
  confirmation_history_mask_ = lethal_confirmation_window_frames_ == 64 ?
    std::numeric_limits<std::uint64_t>::max() :
    (std::uint64_t{1} << lethal_confirmation_window_frames_) - 1;
  lethal_max_age_ns_ = static_cast<std::int64_t>(std::llround(lethal_max_age_ * 1e9));
  // Costmap2D's default constructor leaves default_value_ indeterminate. Because
  // updateWithMax() skips NO_INFORMATION, this keeps unobserved cells from raising
  // the master costmap when a full refresh clears the internal map.
  default_value_ = nav2_costmap_2d::NO_INFORMATION;
  // CostmapLayer owns a separate internal map.  Nav2 has already sized the
  // parent costmap before plugin initialization, so initialize ours now.
  matchSize();
  terrain_status_publisher_ = node->create_publisher<std_msgs::msg::Bool>(
    terrain_status_topic_, rclcpp::QoS(1).reliable().transient_local());
  diagnostic_costmap_publisher_ = node->create_publisher<nav2_msgs::msg::Costmap>(
    getName() + "/costmap_raw", rclcpp::QoS(1).reliable().transient_local());
  {
    std::lock_guard<std::mutex> lock(mutex_);
    publishTerrainStatusLocked(false);
  }
  subscription_ = node->create_subscription<grid_map_msgs::msg::GridMap>(
    topic, rclcpp::QoS(1),
    std::bind(&TerrainLayer::gridMapCallback, this, std::placeholders::_1));
  // Costmap currency reports whether this layer completed its update cycle. The
  // status heartbeat follows accepted source messages and is checked by the gate.
  current_ = true;
}

void TerrainLayer::gridMapCallback(const grid_map_msgs::msg::GridMap::SharedPtr msg)
{
  if (msg->header.frame_id != global_frame_) {
    std::lock_guard<std::mutex> lock(mutex_);
    invalidateMapLocked();
    RCLCPP_ERROR_THROTTLE(
      logger_, *clock_, 2000, "FastDEM terrain frame mismatch: got '%s', expected '%s'; stopping",
      msg->header.frame_id.c_str(), global_frame_.c_str());
    return;
  }
  if (!validGridMapGeometry(*msg)) {
    std::lock_guard<std::mutex> lock(mutex_);
    invalidateMapLocked();
    RCLCPP_ERROR_THROTTLE(
      logger_, *clock_, 2000, "Malformed FastDEM terrain grid map geometry; stopping");
    return;
  }

  nanogrid::GridMap map;
  try {
    map = nanogrid::ros2::fromMsg(*msg);
  } catch (const std::exception & error) {
    std::lock_guard<std::mutex> lock(mutex_);
    invalidateMapLocked();
    RCLCPP_ERROR_THROTTLE(
      logger_, *clock_, 2000, "Malformed FastDEM terrain grid map: %s; stopping", error.what());
    return;
  }

  const auto has_layer = [&map](const char * name) {
      return std::find(map.getLayers().begin(), map.getLayers().end(), name) !=
             map.getLayers().end();
    };
  if (!has_layer("slope") || !has_layer("roughness") || !has_layer("step_residual") ||
    !has_layer("n_points") || !has_layer("observation_age"))
  {
    // FastDEM strips the feature layers whenever its own source goes stale, so this
    // is a liveness gap rather than a data fault: stop publishing and retain only
    // confirmation history that has not expired by the rolling window or absolute age.
    std::lock_guard<std::mutex> lock(mutex_);
    suspendMapLocked();
    RCLCPP_ERROR_THROTTLE(
      logger_, *clock_, 2000,
      "FastDEM terrain grid map lacks required feature/confidence layers; stopping");
    return;
  }

  if (msg->header.stamp.sec < 0 || msg->header.stamp.nanosec >= 1000000000U ||
    (msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0)) {
    std::lock_guard<std::mutex> lock(mutex_);
    invalidateMapLocked();
    RCLCPP_ERROR_THROTTLE(
      logger_, *clock_, 2000, "FastDEM terrain grid map has zero timestamp; stopping");
    return;
  }

  const auto & slope = map.get("slope");
  const auto & residual_step = map.get("step_residual");
  const auto & roughness = map.get("roughness");
  const auto & n_points = map.get("n_points");
  const auto & observation_age = map.get("observation_age");
  std::array<std::size_t, 3> soft_counts{};
  std::array<std::size_t, 3> lethal_counts{};
  std::array<std::size_t, 3> lethal_candidate_counts{};
  std::unordered_set<WorldCell, WorldCellHash> lethal_cells;
  std::unordered_set<WorldCell, WorldCellHash> lethal_candidate_cells;
  std::size_t confident_cells = 0;
  std::size_t filtered_cells = 0;
  for (auto cell : map.cells()) {
    const std::array<double, 3> features = {
      slope(cell.index), residual_step(cell.index), roughness(cell.index)};
    const bool has_feature = std::any_of(
      features.begin(), features.end(), [](double value) {return std::isfinite(value);});
    if (!has_feature) {
      continue;
    }
    const bool confident = std::isfinite(n_points(cell.index)) &&
      n_points(cell.index) >= static_cast<float>(min_observations_) &&
      std::isfinite(observation_age(cell.index)) && observation_age(cell.index) >= 0.0f &&
      observation_age(cell.index) <= static_cast<float>(max_observation_age_frames_);
    if (!confident) {
      ++filtered_cells;
      continue;
    }
    ++confident_cells;
    bool lethal = false;
    bool lethal_candidate = false;
    for (std::size_t i = 0; i < features.size(); ++i) {
      const auto feature_cost = featureCost(
        features[i], i == 0 ? cost_config_.slope_soft :
        (i == 1 ? cost_config_.residual_step_soft : cost_config_.roughness_soft),
        i == 0 ? cost_config_.slope_lethal :
        (i == 1 ? cost_config_.residual_step_lethal : cost_config_.roughness_lethal));
      const bool hard_constraint = i == 0 ? slope_hard_constraint_ :
        (i == 1 ? residual_step_hard_constraint_ : roughness_hard_constraint_);
      if (feature_cost == nav2_costmap_2d::LETHAL_OBSTACLE) {
        lethal_candidate = true;
        ++lethal_candidate_counts[i];
        if (hard_constraint) {
          ++lethal_counts[i];
          lethal = true;
        } else {
          ++soft_counts[i];
        }
      } else if (feature_cost > 0) {
        ++soft_counts[i];
      }
    }
    if (lethal || lethal_candidate) {
      const auto position = map.position(cell);
      if (position) {
        const auto key = worldCell(*position, map.getResolution());
        if (lethal) {
          lethal_cells.insert(key);
        }
        if (lethal_candidate) {
          lethal_candidate_cells.insert(key);
        }
      }
    }
  }

  if (confident_cells < static_cast<std::size_t>(min_confident_cells_)) {
    // Treat as a liveness gap (same as missing layers): retain only recent
    // confirmation history, which expires by the rolling window or absolute age.
    std::lock_guard<std::mutex> lock(mutex_);
    suspendMapLocked();
    RCLCPP_ERROR_THROTTLE(
      logger_, *clock_, 2000,
      "FastDEM terrain frame rejected: insufficient confidence cells=%zu (min=%d); stopping",
      confident_cells, min_confident_cells_);
    return;
  }

  const double lethal_candidate_fraction =
    static_cast<double>(lethal_candidate_cells.size()) / static_cast<double>(confident_cells);
  if (lethal_candidate_fraction > max_lethal_fraction_) {
    std::lock_guard<std::mutex> lock(mutex_);
    suspendMapLocked();
    RCLCPP_ERROR_THROTTLE(
      logger_, *clock_, 2000,
      "FastDEM terrain frame rejected: confident=%zu, lethal_candidate_fraction=%.3f (max=%.3f); stopping",
      confident_cells, lethal_candidate_fraction, max_lethal_fraction_);
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  const auto stamp_ns = static_cast<std::uint64_t>(msg->header.stamp.sec) * 1000000000ULL +
    static_cast<std::uint64_t>(msg->header.stamp.nanosec);
  const auto now_ns = clock_->now().nanoseconds();
  const auto max_age_ns = static_cast<std::uint64_t>(std::llround(map_timeout_ * 1e9));
  const auto max_future_ns = static_cast<std::uint64_t>(
    std::llround(max_future_stamp_offset_ * 1e9));
  if (now_ns <= 0 ||
    (stamp_ns > static_cast<std::uint64_t>(now_ns) &&
    stamp_ns - static_cast<std::uint64_t>(now_ns) > max_future_ns) ||
    (stamp_ns < static_cast<std::uint64_t>(now_ns) &&
    static_cast<std::uint64_t>(now_ns) - stamp_ns > max_age_ns))
  {
    // A stamp that has merely aged out means the stream is late; the terrain geometry
    // recorded per odom cell is still valid. A real discontinuity is caught by the
    // backwards-timestamp check below, which does clear confirmations.
    suspendMapLocked();
    RCLCPP_ERROR_THROTTLE(
      logger_, *clock_, 2000,
      "FastDEM terrain timestamp is outside the accepted time window; stopping");
    return;
  }
  if (have_source_stamp_ && stamp_ns < last_source_stamp_ns_) {
    invalidateMapLocked();
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 2000,
      "FastDEM terrain timestamp moved backwards; clearing map and confirmations");
    return;
  }
  if (have_source_stamp_ && stamp_ns == last_source_stamp_ns_) {
    return;
  }
  const bool geometry_changed = have_source_geometry_ &&
    (map.getSize()(0) != source_rows_ || map.getSize()(1) != source_cols_ ||
    std::abs(map.getResolution() - source_resolution_) > 1e-9 ||
    std::abs(map.getLength()(0) - source_length_x_) > 1e-9 ||
    std::abs(map.getLength()(1) - source_length_y_) > 1e-9);
  if (geometry_changed) {
    clearConfirmationStateLocked();
  }
  have_source_geometry_ = true;
  source_rows_ = map.getSize()(0);
  source_cols_ = map.getSize()(1);
  source_resolution_ = map.getResolution();
  source_length_x_ = map.getLength()(0);
  source_length_y_ = map.getLength()(1);
  have_source_stamp_ = true;
  last_source_stamp_ns_ = stamp_ns;

  expireConfirmationsLocked(now_ns);
  for (const auto & cell : lethal_cells) {
    confirmations_.try_emplace(cell);
  }
  for (auto it = confirmations_.begin(); it != confirmations_.end();) {
    auto & state = it->second;
    const bool lethal_this_frame = lethal_cells.count(it->first) != 0;
    state.hit_history = (state.hit_history << 1) & confirmation_history_mask_;
    state.frames_in_window = std::min(
      lethal_confirmation_window_frames_, state.frames_in_window + 1);
    if (lethal_this_frame) {
      state.hit_history |= 1;
      state.last_hit_time_ns = static_cast<std::int64_t>(stamp_ns);
    }
    const int hits = hitCount(state.hit_history);
    state.missing_frames = state.frames_in_window - hits;
    const bool confirmed = hits >= lethal_confirmation_frames_ &&
      state.missing_frames < lethal_clear_frames_;
    if (confirmed && !state.confirmed) {
      state.confirmed_since_ns = static_cast<std::int64_t>(stamp_ns);
    } else if (!confirmed) {
      state.confirmed_since_ns = 0;
    }
    state.confirmed = confirmed;
    if (hits == 0) {
      it = confirmations_.erase(it);
    } else {
      ++it;
    }
  }
  expireConfirmationsLocked(now_ns);

  latest_map_ = std::move(map);
  have_map_ = true;
  clear_requested_ = true;
  last_message_time_ = rclcpp::Time(msg->header.stamp);
  publishTerrainStatusLocked(mapIsFresh(clock_->now()), true);
  std::size_t pending = 0;
  std::size_t confirmed = 0;
  double latched_age_max = 0.0;
  for (const auto & [unused, state] : confirmations_) {
    if (state.confirmed) {
      ++confirmed;
      if (state.confirmed_since_ns > 0 && now_ns >= state.confirmed_since_ns) {
        latched_age_max = std::max(
          latched_age_max,
          static_cast<double>(now_ns - state.confirmed_since_ns) / 1e9);
      }
    } else {
      ++pending;
    }
  }
  RCLCPP_INFO_THROTTLE(
    logger_, *clock_, 2000,
    "FastDEM terrain: enabled=%s confident=%zu filtered=%zu slope soft/lethal/candidates=%zu/%zu/%zu residual soft/lethal/candidates=%zu/%zu/%zu roughness soft/lethal/candidates=%zu/%zu/%zu pending=%zu confirmed=%zu latched_age_max=%.2f s",
    enabled_ ? "true" : "false", confident_cells, filtered_cells,
    soft_counts[0], lethal_counts[0],
    lethal_candidate_counts[0], soft_counts[1], lethal_counts[1],
    lethal_candidate_counts[1], soft_counts[2], lethal_counts[2],
    lethal_candidate_counts[2], pending, confirmed, latched_age_max);
}

bool TerrainLayer::mapIsFresh(const rclcpp::Time & now) const
{
  return have_map_ && (now - last_message_time_).seconds() >= 0.0 &&
         (now - last_message_time_).seconds() <= map_timeout_;
}

void TerrainLayer::addFullCostmapBounds(
  double * min_x, double * min_y, double * max_x, double * max_y) const
{
  *min_x = std::min(*min_x, getOriginX());
  *min_y = std::min(*min_y, getOriginY());
  *max_x = std::max(*max_x, getOriginX() + getSizeInMetersX());
  *max_y = std::max(*max_y, getOriginY() + getSizeInMetersY());
}

void TerrainLayer::updateBounds(
  double robot_x, double robot_y, double, double * min_x, double * min_y, double * max_x,
  double * max_y)
{
  bool origin_changed = false;
  if (layered_costmap_ != nullptr && layered_costmap_->isRolling()) {
    const double old_origin_x = getOriginX();
    const double old_origin_y = getOriginY();
    updateOrigin(
      robot_x - getSizeInMetersX() / 2.0,
      robot_y - getSizeInMetersY() / 2.0);
    origin_changed = getOriginX() != old_origin_x || getOriginY() != old_origin_y;
  }
  useExtraBounds(min_x, min_y, max_x, max_y);
  std::lock_guard<std::mutex> lock(mutex_);
  full_bounds_requested_ = false;
  const bool fresh = mapIsFresh(clock_->now());
  if (expireConfirmationsLocked(clock_->now().nanoseconds())) {
    clear_requested_ = true;
  }
  if (origin_changed || (last_update_fresh_ && !fresh)) {
    clear_requested_ = true;
  }
  last_update_fresh_ = fresh;
  publishTerrainStatusLocked(fresh, true);
  // The source GridMap is immutable between callbacks. Recompute the layer over
  // the full local map for each new source map, clear, or rolling-origin shift;
  // unchanged cycles can use bounds contributed by other costmap layers.
  if (enabled_ && clear_requested_) {
    addFullCostmapBounds(min_x, min_y, max_x, max_y);
    full_bounds_requested_ = true;
  }
}

void TerrainLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j, int max_i, int max_j)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (expireConfirmationsLocked(clock_->now().nanoseconds())) {
    clear_requested_ = true;
  }
  const bool full_refresh = full_bounds_requested_;
  full_bounds_requested_ = false;
  if (full_refresh) {
    resetMaps();
  }
  if (!enabled_ || !mapIsFresh(clock_->now())) {
    if (full_refresh) {
      clear_requested_ = false;
    } else if (enabled_ && last_update_fresh_) {
      // Freshness can expire between updateBounds() and updateCosts(). Ask for a
      // full clear on the next cycle if this cycle did not reserve full bounds.
      clear_requested_ = true;
    }
    last_update_fresh_ = false;
    publishDiagnosticSnapshot(false);
    return;
  }

  if (!full_refresh) {
    // Preserve costs outside the current update window. Clear only the cells
    // that the layered costmap will recompute this cycle.
    for (int j = min_j; j < max_j; ++j) {
      for (int i = min_i; i < max_i; ++i) {
        setCost(static_cast<unsigned int>(i), static_cast<unsigned int>(j),
          nav2_costmap_2d::NO_INFORMATION);
      }
    }
  }

  const auto & slope = latest_map_.get("slope");
  const auto & residual_step = latest_map_.get("step_residual");
  const auto & roughness = latest_map_.get("roughness");
  const auto & n_points = latest_map_.get("n_points");
  const auto & observation_age = latest_map_.get("observation_age");
  std::vector<WorldCell> confirmed_cells;
  for (auto cell : latest_map_.cells()) {
    const auto position = latest_map_.position(cell);
    if (!position) {
      continue;
    }
    unsigned int mx;
    unsigned int my;
    if (!master_grid.worldToMap(position->x(), position->y(), mx, my) ||
      static_cast<int>(mx) < min_i || static_cast<int>(mx) >= max_i ||
      static_cast<int>(my) < min_j || static_cast<int>(my) >= max_j)
    {
      continue;
    }
    const auto key = worldCell(*position, latest_map_.getResolution());
    const auto confirmation = confirmations_.find(key);
    if (confirmation != confirmations_.end() && confirmation->second.confirmed) {
      confirmed_cells.push_back(key);
      continue;
    }
    const bool confident = std::isfinite(n_points(cell.index)) &&
      n_points(cell.index) >= static_cast<float>(min_observations_) &&
      std::isfinite(observation_age(cell.index)) && observation_age(cell.index) >= 0.0f &&
      observation_age(cell.index) <= static_cast<float>(max_observation_age_frames_);
    if (!confident) {
      continue;
    }
    const bool has_feature = std::isfinite(slope(cell.index)) ||
      std::isfinite(residual_step(cell.index)) || std::isfinite(roughness(cell.index));
    if (has_feature) {
      unsigned char cost = terrainCost(
        slope(cell.index), residual_step(cell.index), roughness(cell.index), cost_config_,
        slope_hard_constraint_, residual_step_hard_constraint_, roughness_hard_constraint_);
      if (cost == nav2_costmap_2d::LETHAL_OBSTACLE) {
        cost = confirmation != confirmations_.end() && confirmation->second.confirmed ?
          nav2_costmap_2d::LETHAL_OBSTACLE : 252;
      }
      cost = scaleSoftTerrainCost(cost, cost_config_.soft_cost_scale);
      setCost(mx, my, cost);
    }
  }
  for (const auto & key : confirmed_cells) {
    unsigned int mx = 0;
    unsigned int my = 0;
    if (master_grid.worldToMap(
        static_cast<double>(key.x) * latest_map_.getResolution(),
        static_cast<double>(key.y) * latest_map_.getResolution(), mx, my) &&
      static_cast<int>(mx) >= min_i && static_cast<int>(mx) < max_i &&
      static_cast<int>(my) >= min_j && static_cast<int>(my) < max_j)
    {
      setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);
    }
  }
  if (full_refresh) {
    clear_requested_ = false;
  }
  updateWithMax(master_grid, min_i, min_j, max_i, max_j);
  publishDiagnosticSnapshot(true);
}

void TerrainLayer::activate()
{
  if (diagnostic_costmap_publisher_) {
    diagnostic_costmap_publisher_->on_activate();
  }
}

void TerrainLayer::deactivate()
{
  if (diagnostic_costmap_publisher_) {
    diagnostic_costmap_publisher_->on_deactivate();
  }
}

void TerrainLayer::publishDiagnosticSnapshot(const bool source_is_current)
{
  if (!diagnostic_costmap_publisher_ || !diagnostic_costmap_publisher_->is_activated()) {
    return;
  }
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
  if (source_is_current) {
    const auto * source = getCharMap();
    snapshot.data.assign(source, source + cell_count);
  } else {
    snapshot.data.assign(cell_count, nav2_costmap_2d::NO_INFORMATION);
  }
  diagnostic_costmap_publisher_->publish(snapshot);
}

void TerrainLayer::reset()
{
  std::lock_guard<std::mutex> lock(mutex_);
  resetMaps();
  have_map_ = false;
  clear_requested_ = true;
  full_bounds_requested_ = false;
  last_update_fresh_ = false;
  publishTerrainStatusLocked(false);
  clearConfirmationStateLocked();
  have_source_stamp_ = false;
  have_source_geometry_ = false;
}

void TerrainLayer::matchSize()
{
  CostmapLayer::matchSize();
  std::lock_guard<std::mutex> lock(mutex_);
  clear_requested_ = true;
}

void TerrainLayer::clearConfirmationStateLocked()
{
  confirmations_.clear();
}

bool TerrainLayer::expireConfirmationsLocked(const std::int64_t now_ns)
{
  bool expired = false;
  for (auto it = confirmations_.begin(); it != confirmations_.end();) {
    const auto last_hit_ns = it->second.last_hit_time_ns;
    if (last_hit_ns > 0 && now_ns >= last_hit_ns &&
      now_ns - last_hit_ns > lethal_max_age_ns_)
    {
      it = confirmations_.erase(it);
      expired = true;
    } else {
      ++it;
    }
  }
  return expired;
}

void TerrainLayer::invalidateMapLocked()
{
  suspendMapLocked();
  clearConfirmationStateLocked();
  have_source_stamp_ = false;
  have_source_geometry_ = false;
}

void TerrainLayer::suspendMapLocked()
{
  have_map_ = false;
  clear_requested_ = true;
  last_update_fresh_ = false;
  publishTerrainStatusLocked(false);
}

void TerrainLayer::publishTerrainStatusLocked(const bool valid, const bool force_publish)
{
  if (!terrain_status_publisher_ ||
    (!force_publish && have_published_terrain_status_ && terrain_status_ == valid))
  {
    return;
  }
  std_msgs::msg::Bool message;
  message.data = valid;
  terrain_status_publisher_->publish(message);
  terrain_status_ = valid;
  have_published_terrain_status_ = true;
}

TerrainLayer::WorldCell TerrainLayer::worldCell(
  const nanogrid::Position & position, const double resolution) const
{
  return {
    static_cast<std::int64_t>(std::llround(position.x() / resolution)),
    static_cast<std::int64_t>(std::llround(position.y() / resolution))};
}
}  // namespace nav2_terrain_layer

PLUGINLIB_EXPORT_CLASS(nav2_terrain_layer::TerrainLayer, nav2_costmap_2d::Layer)
