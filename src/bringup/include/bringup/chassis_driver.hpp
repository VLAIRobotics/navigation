#ifndef BRINGUP__CHASSIS_DRIVER_HPP_
#define BRINGUP__CHASSIS_DRIVER_HPP_

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <string>
#include <iterator>
#include <unordered_set>
#include <utility>

#include <geometry_msgs/msg/twist.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <linux/can.h>

namespace bringup
{

struct ChassisConfig
{
  int64_t can_id{0x050};
  int64_t max_raw_speed{800};
  // Independent navigation limits; max_*_speed below remains CAN calibration.
  double max_command_forward_speed{0.30};
  double max_command_reverse_speed{0.10};
  double max_command_angular_speed{0.30};
  double max_linear_speed{0.5};
  double max_angular_speed{1.8};
  double send_rate{200.0};
  double command_timeout{0.5};
  double fastlio_health_loss_grace{0.3};
};

inline void validate_config(const ChassisConfig & config)
{
  if (config.can_id < 0 || config.can_id > 0x7ff) {
    throw std::invalid_argument("can_id must be in range 0..0x7FF");
  }
  if (config.max_raw_speed < 1 || config.max_raw_speed > 32767) {
    throw std::invalid_argument("max_raw_speed must be in range 1..32767");
  }
  const std::array<std::pair<const char *, double>, 8> positive_values{{
      {"max_command_forward_speed", config.max_command_forward_speed},
      {"max_command_reverse_speed", config.max_command_reverse_speed},
      {"max_command_angular_speed", config.max_command_angular_speed},
      {"max_linear_speed", config.max_linear_speed},
      {"max_angular_speed", config.max_angular_speed},
      {"send_rate", config.send_rate},
      {"command_timeout", config.command_timeout},
      {"fastlio_health_loss_grace", config.fastlio_health_loss_grace},
    }};
  for (const auto & item : positive_values) {
    if (!std::isfinite(item.second) || item.second <= 0.0) {
      throw std::invalid_argument(std::string(item.first) + " must be finite and greater than zero");
    }
  }
  if (config.fastlio_health_loss_grace >= config.command_timeout) {
    throw std::invalid_argument("fastlio_health_loss_grace must be less than command_timeout");
  }
}

struct TerrainGateConfig
{
  std::string frame_id{"odom"};
  double max_age{0.5};
  double max_future_stamp_offset{0.1};
  int min_observations{3};
  int max_observation_age_frames{3};
  int min_confident_cells{20};
  double max_lethal_fraction{0.25};
  double slope_lethal{20.0};
  double residual_step_lethal{0.10};
  double roughness_lethal{0.08};
};

inline bool terrain_message_is_current(
  const grid_map_msgs::msg::GridMap & message, const int64_t now_ns,
  const TerrainGateConfig & config)
{
  if (message.header.frame_id != config.frame_id || now_ns <= 0 ||
    !std::isfinite(config.max_age) || config.max_age <= 0.0 ||
    !std::isfinite(config.max_future_stamp_offset) || config.max_future_stamp_offset < 0.0 ||
    config.min_observations < 1 || config.max_observation_age_frames < 0 ||
    config.min_confident_cells < 1 || !std::isfinite(config.max_lethal_fraction) ||
    config.max_lethal_fraction <= 0.0 || config.max_lethal_fraction > 1.0 ||
    !std::isfinite(config.slope_lethal) || !std::isfinite(config.residual_step_lethal) ||
    !std::isfinite(config.roughness_lethal))
  {
    return false;
  }

  const auto stamp_ns = static_cast<int64_t>(message.header.stamp.sec) * 1000000000LL +
    static_cast<int64_t>(message.header.stamp.nanosec);
  if (stamp_ns <= 0 || message.header.stamp.nanosec >= 1000000000U) {
    return false;
  }
  const auto age_ns = now_ns - stamp_ns;
  if (age_ns > static_cast<int64_t>(std::llround(config.max_age * 1e9)) ||
    age_ns < -static_cast<int64_t>(std::llround(config.max_future_stamp_offset * 1e9)))
  {
    return false;
  }

  if (message.layers.size() != message.data.size() ||
    !std::isfinite(message.info.resolution) || message.info.resolution <= 0.0 ||
    !std::isfinite(message.info.length_x) || !std::isfinite(message.info.length_y) ||
    message.info.length_x <= 0.0 || message.info.length_y <= 0.0 ||
    !std::isfinite(message.info.pose.position.x) ||
    !std::isfinite(message.info.pose.position.y) ||
    !std::isfinite(message.info.pose.position.z) ||
    message.info.pose.position.z != 0.0 ||
    !std::isfinite(message.info.pose.orientation.x) ||
    !std::isfinite(message.info.pose.orientation.y) ||
    !std::isfinite(message.info.pose.orientation.z) ||
    !std::isfinite(message.info.pose.orientation.w))
  {
    return false;
  }
  const auto & orientation = message.info.pose.orientation;
  constexpr double kGeometryTolerance = 1e-6;
  const double quaternion_norm_squared =
    orientation.x * orientation.x + orientation.y * orientation.y +
    orientation.z * orientation.z + orientation.w * orientation.w;
  if (!std::isfinite(quaternion_norm_squared) ||
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
    if (!layer_names.insert(message.layers[i]).second ||
      message.data[i].layout.dim.size() != 2 ||
      message.data[i].layout.data_offset != 0 ||
      message.data[i].layout.dim[0].label != "column_index" ||
      message.data[i].layout.dim[1].label != "row_index" ||
      message.data[i].layout.dim[0].size != static_cast<uint32_t>(cols) ||
      message.data[i].layout.dim[1].size != static_cast<uint32_t>(rows) ||
      message.data[i].layout.dim[0].stride != cell_count ||
      message.data[i].layout.dim[1].stride != static_cast<uint32_t>(rows) ||
      message.data[i].data.size() != cell_count)
    {
      return false;
    }
  }

  constexpr const char * required_layers[] = {
    "slope", "step_residual", "roughness", "n_points", "observation_age"};
  std::array<const std::vector<float> *, 5> layers{};
  for (std::size_t i = 0; i < std::size(required_layers); ++i) {
    const auto it = std::find(message.layers.begin(), message.layers.end(), required_layers[i]);
    if (it == message.layers.end()) {
      return false;
    }
    layers[i] = &message.data[static_cast<std::size_t>(std::distance(message.layers.begin(), it))].data;
  }

  std::size_t confident_cells = 0;
  std::size_t lethal_cells = 0;
  for (std::size_t i = 0; i < cell_count; ++i) {
    const bool has_feature = std::isfinite((*layers[0])[i]) ||
      std::isfinite((*layers[1])[i]) || std::isfinite((*layers[2])[i]);
    const bool confident = has_feature && std::isfinite((*layers[3])[i]) &&
      (*layers[3])[i] >= static_cast<float>(config.min_observations) &&
      std::isfinite((*layers[4])[i]) && (*layers[4])[i] >= 0.0f &&
      (*layers[4])[i] <= static_cast<float>(config.max_observation_age_frames);
    if (!confident) {
      continue;
    }
    ++confident_cells;
    if ((std::isfinite((*layers[0])[i]) && (*layers[0])[i] >= config.slope_lethal) ||
      (std::isfinite((*layers[1])[i]) &&
      (*layers[1])[i] >= config.residual_step_lethal) ||
      (std::isfinite((*layers[2])[i]) && (*layers[2])[i] >= config.roughness_lethal))
    {
      ++lethal_cells;
    }
  }
  return confident_cells >= static_cast<std::size_t>(config.min_confident_cells) &&
         static_cast<double>(lethal_cells) / static_cast<double>(confident_cells) <=
         config.max_lethal_fraction;
}

inline bool fastlio_health_within_grace(
  bool health_ready, bool has_ever_been_healthy,
  double unhealthy_duration_sec, double loss_grace_sec)
{
  return health_ready || (has_ever_been_healthy && unhealthy_duration_sec >= 0.0 &&
         unhealthy_duration_sec < loss_grace_sec);
}

inline int16_t velocity_to_raw(double value, double max_speed, int64_t max_raw_speed)
{
  if (!std::isfinite(value)) {
    throw std::invalid_argument("velocity must be finite");
  }
  const auto scaled = std::clamp(
    value / max_speed * max_raw_speed,
    -static_cast<double>(max_raw_speed), static_cast<double>(max_raw_speed));
  const auto limited = std::llround(scaled);
  return static_cast<int16_t>(limited);
}

inline void put_int16_le(can_frame & frame, int offset, int16_t value)
{
  const auto raw = static_cast<uint16_t>(value);
  frame.data[offset] = static_cast<uint8_t>(raw & 0xff);
  frame.data[offset + 1] = static_cast<uint8_t>((raw >> 8) & 0xff);
}

inline can_frame build_frame(int64_t can_id, int16_t vy, int16_t vw)
{
  can_frame frame{};
  frame.can_id = static_cast<canid_t>(can_id);
  frame.can_dlc = 8;
  put_int16_le(frame, 0, 0);
  put_int16_le(frame, 2, vy);
  put_int16_le(frame, 4, vw);
  frame.data[6] = 0;
  frame.data[7] = 0;
  return frame;
}

inline geometry_msgs::msg::Twist limit_navigation_command(
  const geometry_msgs::msg::Twist & input, const ChassisConfig & config)
{
  auto output = input;
  output.linear.x = std::clamp(
    input.linear.x, -config.max_command_reverse_speed, config.max_command_forward_speed);
  output.angular.z = std::clamp(
    input.angular.z, -config.max_command_angular_speed, config.max_command_angular_speed);
  return output;
}

inline bool motion_safety_check_required(
  const geometry_msgs::msg::Twist & command,
  double measured_linear_x, double measured_linear_y, double measured_angular_z,
  double threshold)
{
  if (!std::isfinite(measured_linear_x) || !std::isfinite(measured_linear_y) ||
    !std::isfinite(measured_angular_z) || !std::isfinite(threshold) || threshold < 0.0)
  {
    return true;
  }
  return std::hypot(command.linear.x, command.linear.y) > threshold ||
         std::abs(command.angular.z) > threshold ||
         std::hypot(measured_linear_x, measured_linear_y) > threshold ||
         std::abs(measured_angular_z) > threshold;
}

class CommandState
{
public:
  explicit CommandState(const ChassisConfig & config)
  : config_(config) {}

  bool update(const geometry_msgs::msg::Twist & twist, std::chrono::steady_clock::time_point now)
  {
    if (!finite_twist(twist)) {
      clear();
      safety_fault_latched_ = true;
      return false;
    }
    // The current CAN definition only has verified forward and yaw axes.
    // Never silently drop lateral or other unsupported Twist components.
    if (std::abs(twist.linear.y) > 1e-6 || std::abs(twist.linear.z) > 1e-6 ||
      std::abs(twist.angular.x) > 1e-6 || std::abs(twist.angular.y) > 1e-6)
    {
      clear();
      safety_fault_latched_ = true;
      return false;
    }
    if (std::abs(twist.linear.x) > 2.0 * std::max(
        config_.max_command_forward_speed, config_.max_command_reverse_speed) ||
      std::abs(twist.angular.z) > 2.0 * config_.max_command_angular_speed)
    {
      clear();
      safety_fault_latched_ = true;
      return false;
    }
    if (safety_fault_latched_) {
      clear();
      return true;
    }
    if (health_loss_hold_ && (twist.linear.x != 0.0 || twist.angular.z != 0.0)) {
      return true;
    }
    const auto limited = limit_navigation_command(twist, config_);
    frame_ = build_frame(
      config_.can_id,
      velocity_to_raw(limited.linear.x, config_.max_linear_speed, config_.max_raw_speed),
      velocity_to_raw(-limited.angular.z, config_.max_angular_speed, config_.max_raw_speed));
    command_linear_x_ = limited.linear.x;
    command_angular_z_ = limited.angular.z;
    last_command_ = now;
    have_command_ = true;
    return true;
  }

  void set_health_loss_hold(const bool hold)
  {
    health_loss_hold_ = hold;
    if (hold) {
      clear();
    }
  }

  bool safety_fault_latched() const
  {
    return safety_fault_latched_;
  }

  void clear()
  {
    have_command_ = false;
    command_linear_x_ = 0.0;
    command_angular_z_ = 0.0;
  }

  bool has_active_reverse_command(
    std::chrono::steady_clock::time_point now, double motion_threshold) const
  {
    return have_command_ && now - last_command_ <=
      std::chrono::duration<double>(config_.command_timeout) &&
           command_linear_x_ < -motion_threshold;
  }

  geometry_msgs::msg::Twist command(std::chrono::steady_clock::time_point now) const
  {
    geometry_msgs::msg::Twist twist;
    if (!have_command_ || now - last_command_ >
      std::chrono::duration<double>(config_.command_timeout))
    {
      return twist;
    }
    twist.linear.x = command_linear_x_;
    twist.angular.z = command_angular_z_;
    return twist;
  }

  void block_reverse_motion()
  {
    if (!have_command_ || command_linear_x_ >= 0.0) {
      return;
    }
    command_linear_x_ = 0.0;
    frame_ = build_frame(
      config_.can_id, 0,
      velocity_to_raw(-command_angular_z_, config_.max_angular_speed, config_.max_raw_speed));
  }

  can_frame frame(std::chrono::steady_clock::time_point now) const
  {
    if (!have_command_ || now - last_command_ > std::chrono::duration<double>(config_.command_timeout)) {
      return build_frame(config_.can_id, 0, 0);
    }
    return frame_;
  }

private:
  static bool finite_twist(const geometry_msgs::msg::Twist & twist)
  {
    return std::isfinite(twist.linear.x) && std::isfinite(twist.linear.y) &&
           std::isfinite(twist.linear.z) && std::isfinite(twist.angular.x) &&
           std::isfinite(twist.angular.y) && std::isfinite(twist.angular.z);
  }

  ChassisConfig config_;
  can_frame frame_{};
  double command_linear_x_{0.0};
  double command_angular_z_{0.0};
  std::chrono::steady_clock::time_point last_command_{};
  bool have_command_{false};
  bool health_loss_hold_{false};
  bool safety_fault_latched_{false};
};

class ReverseMotionLatch
{
public:
  explicit ReverseMotionLatch(std::size_t reset_cycles = 5)
  : reset_cycles_(std::max<std::size_t>(1, reset_cycles)) {}

  bool update(bool confirmed_reverse_motion, bool forward_or_zero_command)
  {
    if (confirmed_reverse_motion) {
      active_ = true;
      release_cycles_ = 0;
    } else if (active_ && forward_or_zero_command) {
      ++release_cycles_;
      if (release_cycles_ >= reset_cycles_) {
        active_ = false;
        release_cycles_ = 0;
      }
    } else if (active_) {
      release_cycles_ = 0;
    }
    return active_;
  }

  bool active() const
  {
    return active_;
  }

private:
  std::size_t reset_cycles_;
  std::size_t release_cycles_{0};
  bool active_{false};
};

class ReverseSafetyTripLatch
{
public:
  explicit ReverseSafetyTripLatch(std::chrono::milliseconds hold_duration =
    std::chrono::milliseconds(500))
  : hold_duration_(hold_duration) {}

  void trip(std::chrono::steady_clock::time_point now)
  {
    active_ = true;
    hold_until_ = now + hold_duration_;
  }

  bool update(std::chrono::steady_clock::time_point now, bool reverse_gate_active)
  {
    if (active_ && !reverse_gate_active && now >= hold_until_) {
      active_ = false;
    }
    return active_;
  }

private:
  std::chrono::milliseconds hold_duration_;
  std::chrono::steady_clock::time_point hold_until_{};
  bool active_{false};
};

inline bool reverse_cost_is_blocked(
  std::uint8_t cost, int block_cost, bool unknown_is_blocked)
{
  return cost == 255 ? unknown_is_blocked : cost >= block_cost;
}

inline bool reverse_safety_check_required(bool reverse_command_active, bool reverse_latched)
{
  return reverse_command_active || reverse_latched;
}

}  // namespace bringup

#endif  // BRINGUP__CHASSIS_DRIVER_HPP_
