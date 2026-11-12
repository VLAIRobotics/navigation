#include <chrono>
#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>
#include <string>
#include <utility>

#include <gtest/gtest.h>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include "bringup/chassis_driver.hpp"

namespace
{
geometry_msgs::msg::Twist twist(double linear_x, double angular_z)
{
  geometry_msgs::msg::Twist message;
  message.linear.x = linear_x;
  message.angular.z = angular_z;
  return message;
}

int16_t decode_le(const can_frame & frame, int offset)
{
  return static_cast<int16_t>(static_cast<uint16_t>(frame.data[offset]) |
         (static_cast<uint16_t>(frame.data[offset + 1]) << 8));
}

grid_map_msgs::msg::GridMap terrain_message(int64_t stamp_ns)
{
  grid_map_msgs::msg::GridMap message;
  message.header.frame_id = "odom";
  message.header.stamp.sec = static_cast<int32_t>(stamp_ns / 1000000000LL);
  message.header.stamp.nanosec = static_cast<uint32_t>(stamp_ns % 1000000000LL);
  message.info.resolution = 0.1;
  message.info.length_x = 2.0;
  message.info.length_y = 1.0;
  message.info.pose.orientation.w = 1.0;
  constexpr std::size_t rows = 20;
  constexpr std::size_t cols = 10;
  for (const auto * name : {"slope", "step_residual", "roughness", "n_points", "observation_age"}) {
    std_msgs::msg::Float32MultiArray data;
    data.layout.dim.resize(2);
    data.layout.dim[0].label = "column_index";
    data.layout.dim[0].size = cols;
    data.layout.dim[0].stride = rows * cols;
    data.layout.dim[1].label = "row_index";
    data.layout.dim[1].size = rows;
    data.layout.dim[1].stride = rows;
    data.data.assign(rows * cols, 0.0F);
    if (std::string(name) == "n_points") {
      data.data.assign(rows * cols, 3.0F);
    }
    message.layers.emplace_back(name);
    message.data.push_back(std::move(data));
  }
  return message;
}
}  // namespace

TEST(ChassisDriver, AppliesNavigationLimitsBeforeCanCalibration)
{
  bringup::ChassisConfig config;
  bringup::CommandState state(config);
  const auto now = std::chrono::steady_clock::now();
  ASSERT_TRUE(state.update(twist(0.5, 0.0), now));
  EXPECT_EQ(decode_le(state.frame(now), 2), 480);
  ASSERT_TRUE(state.update(twist(-0.5, 0.0), now));
  EXPECT_EQ(decode_le(state.frame(now), 2), -160);
  ASSERT_TRUE(state.update(twist(0.0, 1.0), now));
  EXPECT_EQ(
    decode_le(state.frame(now), 4),
    bringup::velocity_to_raw(-0.3, config.max_angular_speed, config.max_raw_speed));
  ASSERT_TRUE(state.update(twist(0.5, -0.5), now));
  EXPECT_EQ(decode_le(state.frame(now), 2), 480);
  EXPECT_EQ(decode_le(state.frame(now), 4), 133);
}

TEST(ChassisDriver, StopFrameAndLittleEndianNegative)
{
  const auto frame = bringup::build_frame(0x050, -1, -800);
  EXPECT_EQ(frame.can_id, 0x050U);
  EXPECT_EQ(frame.can_dlc, 8);
  EXPECT_EQ(frame.data[0], 0);
  EXPECT_EQ(frame.data[2], 0xff);
  EXPECT_EQ(frame.data[3], 0xff);
  EXPECT_EQ(frame.data[4], 0xe0);
  EXPECT_EQ(frame.data[5], 0xfc);
  const auto stop = bringup::build_frame(0x050, 0, 0);
  EXPECT_TRUE(std::all_of(std::begin(stop.data), std::begin(stop.data) + 8,
                          [](uint8_t value) { return value == 0; }));
}

TEST(ChassisDriver, InvalidCommandImmediatelyStops)
{
  bringup::ChassisConfig config;
  bringup::CommandState state(config);
  const auto now = std::chrono::steady_clock::now();
  ASSERT_TRUE(state.update(twist(0.5, 0.0), now));
  auto invalid = twist(0.0, 0.0);
  invalid.linear.x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(state.update(invalid, now));
  EXPECT_EQ(decode_le(state.frame(now), 2), 0);
  ASSERT_TRUE(state.safety_fault_latched());
  ASSERT_TRUE(state.update(twist(0.1, 0.0), now));
  EXPECT_EQ(decode_le(state.frame(now), 2), 0);
}

TEST(ChassisDriver, SevereFiniteCommandLatchesStop)
{
  bringup::ChassisConfig config;
  bringup::CommandState state(config);
  const auto now = std::chrono::steady_clock::now();
  ASSERT_FALSE(state.update(twist(2.0, -2.0), now));
  EXPECT_TRUE(state.safety_fault_latched());
  EXPECT_EQ(decode_le(state.frame(now), 2), 0);
  EXPECT_EQ(decode_le(state.frame(now), 4), 0);
  ASSERT_TRUE(state.update(twist(0.1, 0.0), now));
  EXPECT_EQ(decode_le(state.frame(now), 2), 0);
}

TEST(ChassisDriver, StopsAfterCommandTimeout)
{
  bringup::ChassisConfig config;
  bringup::CommandState state(config);
  const auto command_time = std::chrono::steady_clock::now();
  ASSERT_TRUE(state.update(twist(0.5, 0.0), command_time));
  EXPECT_EQ(decode_le(state.frame(command_time + std::chrono::milliseconds(400)), 2), 480);
  EXPECT_EQ(decode_le(state.frame(command_time + std::chrono::milliseconds(600)), 2), 0);
}

TEST(ChassisDriver, ReverseGateFailurePreservesForwardAndRotationCommands)
{
  bringup::ChassisConfig config;
  bringup::CommandState state(config);
  const auto now = std::chrono::steady_clock::now();

  ASSERT_TRUE(state.update(twist(0.20, 0.30), now));
  for (int cycle = 0; cycle < 20; ++cycle) {
    state.block_reverse_motion();
  }
  const auto command = state.command(now);
  EXPECT_DOUBLE_EQ(command.linear.x, 0.20);
  EXPECT_DOUBLE_EQ(command.angular.z, 0.30);
  EXPECT_EQ(decode_le(state.frame(now), 2), 320);
}

TEST(ChassisDriver, ReverseGateFailureStopsOnlyReverseTranslation)
{
  bringup::ChassisConfig config;
  bringup::CommandState state(config);
  const auto now = std::chrono::steady_clock::now();
  ASSERT_TRUE(state.update(twist(-0.20, 0.30), now));

  state.block_reverse_motion();

  const auto command = state.command(now);
  EXPECT_DOUBLE_EQ(command.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(command.angular.z, 0.30);
  EXPECT_EQ(decode_le(state.frame(now), 2), 0);
  EXPECT_NE(decode_le(state.frame(now), 4), 0);
}

TEST(ChassisDriver, ActiveReverseCommandRequiresSafetyCheckBeforeConfirmation)
{
  bringup::ChassisConfig config;
  bringup::CommandState state(config);
  const auto now = std::chrono::steady_clock::now();
  ASSERT_TRUE(state.update(twist(-0.01, 0.30), now));

  const bool reverse_active = state.has_active_reverse_command(now, 0.0);
  EXPECT_TRUE(reverse_active);
  EXPECT_TRUE(bringup::reverse_safety_check_required(reverse_active, false));
}

TEST(ChassisDriver, ReverseMotionLatchResetsAfterForwardOrZeroCycles)
{
  bringup::ReverseMotionLatch latch(3);
  EXPECT_TRUE(latch.update(true, false));
  EXPECT_TRUE(latch.active());
  EXPECT_TRUE(latch.update(false, true));
  EXPECT_TRUE(latch.update(false, true));
  EXPECT_FALSE(latch.update(false, true));
  EXPECT_FALSE(latch.active());
}

TEST(ChassisDriver, ReverseBlockedStatusHoldsThroughZeroCommandAndLatchRecovery)
{
  bringup::ChassisConfig config;
  bringup::CommandState state(config);
  bringup::ReverseMotionLatch motion_latch(2);
  bringup::ReverseSafetyTripLatch safety_trip;
  const auto start = std::chrono::steady_clock::now();

  ASSERT_TRUE(state.update(twist(-0.05, 0.0), start));
  ASSERT_TRUE(motion_latch.update(true, false));
  safety_trip.trip(start);
  state.clear();
  EXPECT_EQ(decode_le(state.frame(start), 2), 0);
  EXPECT_TRUE(safety_trip.update(start + std::chrono::milliseconds(100), true));

  const auto recovery_time = start + std::chrono::milliseconds(600);
  EXPECT_TRUE(motion_latch.update(false, true));
  EXPECT_TRUE(safety_trip.update(recovery_time, motion_latch.active()));
  EXPECT_FALSE(motion_latch.update(false, true));
  EXPECT_FALSE(safety_trip.update(recovery_time, motion_latch.active()));
}

TEST(ChassisDriver, ReverseBlockedStatusHonorsMinimumTripHold)
{
  bringup::ReverseSafetyTripLatch safety_trip;
  const auto start = std::chrono::steady_clock::now();
  safety_trip.trip(start);

  EXPECT_TRUE(safety_trip.update(start + std::chrono::milliseconds(499), false));
  EXPECT_FALSE(safety_trip.update(start + std::chrono::milliseconds(500), false));
}

TEST(ChassisDriver, ReverseCostThresholdAndUnknownPolicyAreIndependent)
{
  EXPECT_FALSE(bringup::reverse_cost_is_blocked(252, 253, false));
  EXPECT_TRUE(bringup::reverse_cost_is_blocked(253, 253, false));
  EXPECT_FALSE(bringup::reverse_cost_is_blocked(255, 253, false));
  EXPECT_TRUE(bringup::reverse_cost_is_blocked(255, 253, true));
}

TEST(ChassisDriver, HealthGraceRequiresPriorHealthyStateAndExpires)
{
  constexpr double grace = 0.3;
  EXPECT_TRUE(bringup::fastlio_health_within_grace(true, true, 0.0, grace));
  EXPECT_TRUE(bringup::fastlio_health_within_grace(false, true, 0.2, grace));
  EXPECT_FALSE(bringup::fastlio_health_within_grace(false, true, grace, grace));
  EXPECT_FALSE(bringup::fastlio_health_within_grace(false, false, 0.0, grace));
}

TEST(ChassisDriver, FastlioHealthLossImmediatelyStopsAndHoldsMotion)
{
  bringup::ChassisConfig config;
  bringup::CommandState state(config);
  const auto now = std::chrono::steady_clock::now();
  ASSERT_TRUE(state.update(twist(0.20, 0.30), now));

  state.set_health_loss_hold(true);
  EXPECT_EQ(decode_le(state.frame(now), 2), 0);
  EXPECT_EQ(decode_le(state.frame(now), 4), 0);

  EXPECT_TRUE(state.update(twist(0.20, 0.30), now + std::chrono::milliseconds(10)));
  EXPECT_EQ(decode_le(state.frame(now + std::chrono::milliseconds(10)), 2), 0);
  EXPECT_EQ(decode_le(state.frame(now + std::chrono::milliseconds(10)), 4), 0);

  state.set_health_loss_hold(false);
  EXPECT_TRUE(state.update(twist(0.10, 0.0), now + std::chrono::milliseconds(20)));
  EXPECT_NE(decode_le(state.frame(now + std::chrono::milliseconds(20)), 2), 0);
}

TEST(ChassisDriver, ValidatesSafetyParameters)
{
  bringup::ChassisConfig config;
  config.command_timeout = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(bringup::validate_config(config), std::invalid_argument);
  config = bringup::ChassisConfig{};
  config.max_raw_speed = 32768;
  EXPECT_THROW(bringup::validate_config(config), std::invalid_argument);
  config = bringup::ChassisConfig{};
  config.can_id = 0x800;
  EXPECT_THROW(bringup::validate_config(config), std::invalid_argument);
  config = bringup::ChassisConfig{};
  config.fastlio_health_loss_grace = -0.1;
  EXPECT_THROW(bringup::validate_config(config), std::invalid_argument);
  config = bringup::ChassisConfig{};
  config.fastlio_health_loss_grace = config.command_timeout;
  EXPECT_THROW(bringup::validate_config(config), std::invalid_argument);
}

TEST(ChassisDriver, TerrainGateRequiresCurrentTrustedGeometry)
{
  const bringup::TerrainGateConfig config;
  constexpr int64_t now_ns = 10'000'000'000LL;
  auto message = terrain_message(now_ns);
  EXPECT_TRUE(bringup::terrain_message_is_current(message, now_ns, config));

  message.header.frame_id = "base_link";
  EXPECT_FALSE(bringup::terrain_message_is_current(message, now_ns, config));
  message = terrain_message(0);
  EXPECT_FALSE(bringup::terrain_message_is_current(message, now_ns, config));
  message = terrain_message(now_ns + 200'000'000LL);
  EXPECT_FALSE(bringup::terrain_message_is_current(message, now_ns, config));
  message = terrain_message(now_ns - 600'000'000LL);
  EXPECT_FALSE(bringup::terrain_message_is_current(message, now_ns, config));

  message = terrain_message(now_ns);
  message.data.front().data.pop_back();
  EXPECT_FALSE(bringup::terrain_message_is_current(message, now_ns, config));

  message = terrain_message(now_ns);
  for (auto & data : message.data) {
    data.layout.dim[0].size = 20;
  }
  EXPECT_FALSE(bringup::terrain_message_is_current(message, now_ns, config));
}

TEST(ChassisDriver, TerrainGateUsesConfiguredFreshnessWindow)
{
  auto config = bringup::TerrainGateConfig{};
  config.max_age = 1.2;
  constexpr int64_t now_ns = 10'000'000'000LL;

  auto message = terrain_message(now_ns - 800'000'000LL);
  EXPECT_TRUE(bringup::terrain_message_is_current(message, now_ns, config));

  message = terrain_message(now_ns - 1'300'000'000LL);
  EXPECT_FALSE(bringup::terrain_message_is_current(message, now_ns, config));
}

TEST(ChassisDriver, TerrainGateRejectsInvalidPoseAndOffsetGeometry)
{
  const bringup::TerrainGateConfig config;
  constexpr int64_t now_ns = 10'000'000'000LL;
  auto message = terrain_message(now_ns);
  message.info.pose.orientation.z = std::sin(0.25);
  message.info.pose.orientation.w = std::cos(0.25);
  EXPECT_FALSE(bringup::terrain_message_is_current(message, now_ns, config));

  message = terrain_message(now_ns);
  message.info.pose.orientation.w = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(bringup::terrain_message_is_current(message, now_ns, config));

  message = terrain_message(now_ns);
  message.info.pose.position.z = 0.01;
  EXPECT_FALSE(bringup::terrain_message_is_current(message, now_ns, config));

  message = terrain_message(now_ns);
  message.data.front().layout.data_offset = 1;
  EXPECT_FALSE(bringup::terrain_message_is_current(message, now_ns, config));
}

TEST(ChassisDriver, TerrainGateRejectsSparseAndLethalOutlierFrames)
{
  const bringup::TerrainGateConfig config;
  constexpr int64_t now_ns = 10'000'000'000LL;
  auto message = terrain_message(now_ns);
  for (std::size_t i = 0; i < 3; ++i) {
    std::fill(message.data[i].data.begin(), message.data[i].data.end(), NAN);
  }
  EXPECT_FALSE(bringup::terrain_message_is_current(message, now_ns, config));

  message = terrain_message(now_ns);
  std::fill(message.data[0].data.begin(), message.data[0].data.end(), 20.0F);
  EXPECT_FALSE(bringup::terrain_message_is_current(message, now_ns, config));
}

TEST(ChassisDriver, TerrainGateRejectsAnAllNanMap)
{
  const bringup::TerrainGateConfig config;
  constexpr int64_t now_ns = 10'000'000'000LL;
  auto message = terrain_message(now_ns);
  for (auto & layer : message.data) {
    std::fill(layer.data.begin(), layer.data.end(),
      std::numeric_limits<float>::quiet_NaN());
  }
  EXPECT_FALSE(bringup::terrain_message_is_current(message, now_ns, config));
}
