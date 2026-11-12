#include <algorithm>
#include <chrono>
#include <cstddef>
#include <limits>
#include <memory>
#include <thread>

#include <gtest/gtest.h>
#include <nanogrid/bridge/ros2.hpp>
#include <nav2_costmap_2d/layered_costmap.hpp>
#include <nav2_util/lifecycle_node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_ros/buffer.h>

#include "nav2_terrain_layer/terrain_layer.hpp"

namespace
{
class TerrainLayerTest : public ::testing::Test
{
protected:
  virtual bool useSoftResidual() const {return false;}

  static void SetUpTestSuite()
  {
    int argc = 0;
    char ** argv = nullptr;
    rclcpp::init(argc, argv);
  }

  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }

  void SetUp() override
  {
    rclcpp::NodeOptions options;
    options.append_parameter_override(
      "terrain_layer.gridmap_topic", std::string("/test/terrain_gridmap"));
    options.append_parameter_override(
      "terrain_layer.residual_step_hard_constraint", !useSoftResidual());
    lifecycle_node_ = std::make_shared<nav2_util::LifecycleNode>(
      "terrain_layer_test", "", options);
    publisher_node_ = std::make_shared<rclcpp::Node>("terrain_layer_test_publisher");
    publisher_ = publisher_node_->create_publisher<grid_map_msgs::msg::GridMap>(
      "/test/terrain_gridmap", rclcpp::QoS(10));
    terrain_status_subscription_ = publisher_node_->create_subscription<std_msgs::msg::Bool>(
      "/fastdem/terrain_valid", rclcpp::QoS(10).reliable().transient_local(),
      [this](const std_msgs::msg::Bool::SharedPtr message) {
        terrain_status_received_ = true;
        terrain_status_valid_ = message->data;
        ++terrain_status_message_count_;
      });

    layered_costmap_ = std::make_unique<nav2_costmap_2d::LayeredCostmap>("odom", true, false);
    layered_costmap_->resizeMap(20, 20, 0.1, -1.0, -1.0);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(lifecycle_node_->get_clock());
    layer_ = std::make_shared<nav2_terrain_layer::TerrainLayer>();
    layer_->initialize(
      layered_costmap_.get(), "terrain_layer", tf_buffer_.get(), lifecycle_node_,
      lifecycle_node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive));
    layered_costmap_->addPlugin(layer_);

    executor_.add_node(lifecycle_node_->get_node_base_interface());
    executor_.add_node(publisher_node_->get_node_base_interface());
  }

  void TearDown() override
  {
    executor_.remove_node(publisher_node_->get_node_base_interface());
    executor_.remove_node(lifecycle_node_->get_node_base_interface());
    layer_.reset();
    tf_buffer_.reset();
    layered_costmap_.reset();
    publisher_.reset();
    terrain_status_subscription_.reset();
    publisher_node_.reset();
    lifecycle_node_.reset();
  }

  grid_map_msgs::msg::GridMap makeTerrainMessage(
    int64_t stamp_ns, bool lethal_center = false, bool low_observation_center = false,
    bool stale_center = false, double lethal_fraction = 0.0,
    bool no_feature_center = false, bool residual_center = false) const
  {
    nanogrid::GridMap map({
      "slope", "step_residual", "roughness", "n_points", "observation_age"});
    map.setGeometry(nanogrid::Length(2.0, 1.0), 0.1, nanogrid::Position(0.0, 0.0));
    map.setFrameId("odom");
    map.setTimestamp(static_cast<uint64_t>(stamp_ns));
    map.get("slope").setZero();
    map.get("step_residual").setZero();
    map.get("roughness").setZero();
    map.get("n_points").setConstant(3.0f);
    map.get("observation_age").setZero();

    if (lethal_center) {
      const auto center = map.index(nanogrid::Position(0.05, 0.05));
      EXPECT_TRUE(center.has_value());
      if (!center) return toStandardMessage(map);
      map.at("slope", *center) = 20.0f;
    }
    if (residual_center) {
      const auto center = map.index(nanogrid::Position(0.05, 0.05));
      EXPECT_TRUE(center.has_value());
      if (!center) return toStandardMessage(map);
      map.at("step_residual", *center) = 0.12f;
    }
    if (low_observation_center || stale_center) {
      const auto center = map.index(nanogrid::Position(0.05, 0.05));
      EXPECT_TRUE(center.has_value());
      if (!center) return toStandardMessage(map);
      map.at("n_points", *center) = low_observation_center ? 2.0f : 3.0f;
      map.at("observation_age", *center) = stale_center ? 4.0f : 0.0f;
      map.at("slope", *center) = 20.0f;
    }
    if (no_feature_center) {
      const auto center = map.index(nanogrid::Position(0.05, 0.05));
      EXPECT_TRUE(center.has_value());
      if (!center) return toStandardMessage(map);
      map.at("slope", *center) = NAN;
      map.at("step_residual", *center) = NAN;
      map.at("roughness", *center) = NAN;
    }
    if (lethal_fraction > 0.0) {
      int lethal_cells = static_cast<int>(map.getSize().prod() * lethal_fraction);
      for (auto cell : map.cells()) {
        if (lethal_cells-- <= 0) break;
        map.at("slope", cell) = 20.0f;
      }
    }
    return toStandardMessage(map);
  }

  grid_map_msgs::msg::GridMap toStandardMessage(const nanogrid::GridMap & map) const
  {
    auto message = nanogrid::ros2::toMsg(map);
    for (auto & data : message.data) {
      data.layout.dim[0].label = "column_index";
      data.layout.dim[0].size = static_cast<std::uint32_t>(map.getSize()(1));
      data.layout.dim[0].stride = static_cast<std::uint32_t>(map.getSize().prod());
      data.layout.dim[1].label = "row_index";
      data.layout.dim[1].size = static_cast<std::uint32_t>(map.getSize()(0));
      data.layout.dim[1].stride = static_cast<std::uint32_t>(map.getSize()(0));
    }
    return message;
  }

  grid_map_msgs::msg::GridMap makeMessageWithoutLayer(int64_t stamp_ns)
  {
    auto message = makeTerrainMessage(stamp_ns);
    const auto it = std::find(
      message.layers.begin(), message.layers.end(), "observation_age");
    EXPECT_NE(it, message.layers.end());
    if (it == message.layers.end()) return message;
    const auto index = static_cast<std::size_t>(std::distance(message.layers.begin(), it));
    message.layers.erase(it);
    message.data.erase(message.data.begin() + static_cast<std::ptrdiff_t>(index));
    return message;
  }

  void publishAndUpdate(const grid_map_msgs::msg::GridMap & message)
  {
    publisher_->publish(message);
    for (int i = 0; i < 20; ++i) {
      executor_.spin_some();
      layered_costmap_->updateMap(0.0, 0.0, 0.0);
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  }

  unsigned char centerCost() const
  {
    const auto * costmap = layered_costmap_->getCostmap();
    unsigned int mx;
    unsigned int my;
    if (!costmap->worldToMap(0.0, 0.0, mx, my)) return 0;
    return costmap->getCost(mx, my);
  }

  rclcpp::executors::SingleThreadedExecutor executor_;
  std::shared_ptr<nav2_util::LifecycleNode> lifecycle_node_;
  std::shared_ptr<rclcpp::Node> publisher_node_;
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr terrain_status_subscription_;
  bool terrain_status_received_{false};
  bool terrain_status_valid_{false};
  std::size_t terrain_status_message_count_{0};
  std::unique_ptr<nav2_costmap_2d::LayeredCostmap> layered_costmap_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<nav2_terrain_layer::TerrainLayer> layer_;
};

TEST_F(TerrainLayerTest, InitializesAndAppliesGridMap)
{
  ASSERT_EQ(layer_->getSizeInCellsX(), 20U);
  ASSERT_EQ(layer_->getSizeInCellsY(), 20U);

  publishAndUpdate(makeTerrainMessage(
    lifecycle_node_->get_clock()->now().nanoseconds(), true));
  EXPECT_TRUE(layer_->isCurrent());
  EXPECT_EQ(centerCost(), 252);
}

TEST_F(TerrainLayerTest, RequiresThreeLethalFrames)
{
  for (int i = 0; i < 2; ++i) {
    publishAndUpdate(makeTerrainMessage(
      lifecycle_node_->get_clock()->now().nanoseconds(), true));
    EXPECT_EQ(centerCost(), 252);
  }
  publishAndUpdate(makeTerrainMessage(
    lifecycle_node_->get_clock()->now().nanoseconds(), true));
  EXPECT_EQ(centerCost(), nav2_costmap_2d::LETHAL_OBSTACLE);
}

TEST_F(TerrainLayerTest, DuplicateTimestampDoesNotConfirmAnotherFrame)
{
  const auto message = makeTerrainMessage(
    lifecycle_node_->get_clock()->now().nanoseconds(), true);
  publishAndUpdate(message);
  publishAndUpdate(message);
  publishAndUpdate(message);
  EXPECT_EQ(centerCost(), 252);

  const auto next_stamp = message.header.stamp.sec * 1000000000LL +
    message.header.stamp.nanosec + 100000000LL;
  publishAndUpdate(makeTerrainMessage(next_stamp, true));
  EXPECT_EQ(centerCost(), 252);
  publishAndUpdate(makeTerrainMessage(next_stamp + 100000000LL, true));
  EXPECT_EQ(centerCost(), nav2_costmap_2d::LETHAL_OBSTACLE);
}

TEST_F(TerrainLayerTest, ClearsConfirmedLethalAfterTwoMissingFrames)
{
  for (int i = 0; i < 3; ++i) {
    publishAndUpdate(makeTerrainMessage(
      lifecycle_node_->get_clock()->now().nanoseconds(), true));
  }
  ASSERT_EQ(centerCost(), nav2_costmap_2d::LETHAL_OBSTACLE);
  publishAndUpdate(makeTerrainMessage(lifecycle_node_->get_clock()->now().nanoseconds()));
  EXPECT_EQ(centerCost(), nav2_costmap_2d::LETHAL_OBSTACLE);
  publishAndUpdate(makeTerrainMessage(lifecycle_node_->get_clock()->now().nanoseconds()));
  EXPECT_EQ(centerCost(), 0);
}

TEST_F(TerrainLayerTest, FiltersLowConfidenceCells)
{
  publishAndUpdate(makeTerrainMessage(
    lifecycle_node_->get_clock()->now().nanoseconds(), false, true));
  EXPECT_TRUE(layer_->isCurrent());
  EXPECT_EQ(centerCost(), 0);

  publishAndUpdate(makeTerrainMessage(
    lifecycle_node_->get_clock()->now().nanoseconds(), false, false, true));
  EXPECT_TRUE(layer_->isCurrent());
  EXPECT_EQ(centerCost(), 0);
}

TEST_F(TerrainLayerTest, RetainsConfirmedLethalForLowConfidenceCell)
{
  for (int i = 0; i < 3; ++i) {
    publishAndUpdate(makeTerrainMessage(
      lifecycle_node_->get_clock()->now().nanoseconds(), true));
  }
  ASSERT_EQ(centerCost(), nav2_costmap_2d::LETHAL_OBSTACLE);

  publishAndUpdate(makeTerrainMessage(
    lifecycle_node_->get_clock()->now().nanoseconds(), false, true));
  EXPECT_EQ(centerCost(), nav2_costmap_2d::LETHAL_OBSTACLE);
  publishAndUpdate(makeTerrainMessage(
    lifecycle_node_->get_clock()->now().nanoseconds(), false, false, false, 0.0, true));
  EXPECT_EQ(centerCost(), 0);
}

TEST_F(TerrainLayerTest, RetainsConfirmedLethalForStaleCell)
{
  for (int i = 0; i < 3; ++i) {
    publishAndUpdate(makeTerrainMessage(
      lifecycle_node_->get_clock()->now().nanoseconds(), true));
  }
  ASSERT_EQ(centerCost(), nav2_costmap_2d::LETHAL_OBSTACLE);

  publishAndUpdate(makeTerrainMessage(
    lifecycle_node_->get_clock()->now().nanoseconds(), false, false, true));
  EXPECT_EQ(centerCost(), nav2_costmap_2d::LETHAL_OBSTACLE);
}

TEST_F(TerrainLayerTest, MissingConfidenceLayerStops)
{
  publishAndUpdate(makeMessageWithoutLayer(lifecycle_node_->get_clock()->now().nanoseconds()));
  EXPECT_TRUE(layer_->isCurrent());
  EXPECT_EQ(centerCost(), 0);
}

TEST_F(TerrainLayerTest, MalformedArrayGeometryStops)
{
  auto message = makeTerrainMessage(lifecycle_node_->get_clock()->now().nanoseconds());
  ASSERT_FALSE(message.data.empty());
  message.data.front().data.pop_back();
  publishAndUpdate(message);
  EXPECT_TRUE(layer_->isCurrent());
  EXPECT_EQ(centerCost(), 0);
}

TEST_F(TerrainLayerTest, RotatedGridMapStops)
{
  auto message = makeTerrainMessage(lifecycle_node_->get_clock()->now().nanoseconds());
  message.info.pose.orientation.z = std::sin(0.25);
  message.info.pose.orientation.w = std::cos(0.25);
  publishAndUpdate(message);
  EXPECT_TRUE(layer_->isCurrent());
  EXPECT_EQ(centerCost(), 0);
}

TEST_F(TerrainLayerTest, NonFiniteQuaternionStops)
{
  auto message = makeTerrainMessage(lifecycle_node_->get_clock()->now().nanoseconds());
  message.info.pose.orientation.w = std::numeric_limits<double>::quiet_NaN();
  publishAndUpdate(message);
  EXPECT_TRUE(layer_->isCurrent());
  EXPECT_EQ(centerCost(), 0);
}

TEST_F(TerrainLayerTest, NonZeroHeightAndDataOffsetStop)
{
  auto message = makeTerrainMessage(lifecycle_node_->get_clock()->now().nanoseconds());
  message.info.pose.position.z = 0.01;
  publishAndUpdate(message);
  EXPECT_TRUE(layer_->isCurrent());
  EXPECT_EQ(centerCost(), 0);

  message = makeTerrainMessage(lifecycle_node_->get_clock()->now().nanoseconds());
  ASSERT_FALSE(message.data.empty());
  message.data.front().layout.data_offset = 1;
  publishAndUpdate(message);
  EXPECT_TRUE(layer_->isCurrent());
  EXPECT_EQ(centerCost(), 0);
}

TEST_F(TerrainLayerTest, SparseFrameStops)
{
  auto message = makeTerrainMessage(lifecycle_node_->get_clock()->now().nanoseconds());
  for (auto & data : message.data) {
    data.data.assign(data.data.size(), NAN);
  }
  publishAndUpdate(message);
  EXPECT_TRUE(layer_->isCurrent());
}

TEST_F(TerrainLayerTest, ExcessiveLethalFractionStops)
{
  publishAndUpdate(makeTerrainMessage(
    lifecycle_node_->get_clock()->now().nanoseconds(), false, false, false, 0.5));
  EXPECT_TRUE(layer_->isCurrent());
  EXPECT_EQ(centerCost(), 0);
}

TEST_F(TerrainLayerTest, TimestampRollbackStops)
{
  const auto now = lifecycle_node_->get_clock()->now().nanoseconds();
  publishAndUpdate(makeTerrainMessage(now, true));
  ASSERT_TRUE(layer_->isCurrent());
  publishAndUpdate(makeTerrainMessage(now - 1000000000, true));
  EXPECT_TRUE(layer_->isCurrent());
  EXPECT_EQ(centerCost(), 0);
}

TEST_F(TerrainLayerTest, StaleSourceClearsTerrainCostsWithoutBlockingCostmap)
{
  for (int i = 0; i < 3; ++i) {
    publishAndUpdate(makeTerrainMessage(
      lifecycle_node_->get_clock()->now().nanoseconds(), true));
  }
  ASSERT_EQ(centerCost(), nav2_costmap_2d::LETHAL_OBSTACLE);

  const auto stale_stamp = lifecycle_node_->get_clock()->now().nanoseconds() - 1000000000LL;
  publishAndUpdate(makeTerrainMessage(stale_stamp, true));

  EXPECT_TRUE(layer_->isCurrent());
  EXPECT_EQ(centerCost(), nav2_costmap_2d::FREE_SPACE);
}

TEST_F(TerrainLayerTest, TerrainStatusReflectsAcceptedQualityAndFreshness)
{
  publishAndUpdate(makeTerrainMessage(lifecycle_node_->get_clock()->now().nanoseconds()));
  ASSERT_TRUE(terrain_status_received_);
  EXPECT_TRUE(terrain_status_valid_);

  auto malformed = makeTerrainMessage(lifecycle_node_->get_clock()->now().nanoseconds());
  ASSERT_FALSE(malformed.data.empty());
  malformed.data.front().data.pop_back();
  publishAndUpdate(malformed);
  EXPECT_FALSE(terrain_status_valid_);

  auto low_confidence = makeTerrainMessage(lifecycle_node_->get_clock()->now().nanoseconds());
  const auto n_points = std::find(
    low_confidence.layers.begin(), low_confidence.layers.end(), "n_points");
  ASSERT_NE(n_points, low_confidence.layers.end());
  const auto n_points_index = static_cast<std::size_t>(
    std::distance(low_confidence.layers.begin(), n_points));
  std::fill(
    low_confidence.data[n_points_index].data.begin(),
    low_confidence.data[n_points_index].data.end(), 0.0F);
  publishAndUpdate(low_confidence);
  EXPECT_FALSE(terrain_status_valid_);

  publishAndUpdate(makeTerrainMessage(lifecycle_node_->get_clock()->now().nanoseconds()));
  EXPECT_TRUE(terrain_status_valid_);

  const auto stale_stamp = lifecycle_node_->get_clock()->now().nanoseconds() - 1000000000LL;
  publishAndUpdate(makeTerrainMessage(stale_stamp));
  EXPECT_FALSE(terrain_status_valid_);
}

TEST_F(TerrainLayerTest, TerrainStatusHeartbeatFollowsAcceptedSourceMessages)
{
  publishAndUpdate(makeTerrainMessage(lifecycle_node_->get_clock()->now().nanoseconds()));
  ASSERT_TRUE(terrain_status_valid_);
  const auto first_message_count = terrain_status_message_count_;
  publishAndUpdate(makeTerrainMessage(lifecycle_node_->get_clock()->now().nanoseconds()));
  EXPECT_GT(terrain_status_message_count_, first_message_count);
}

TEST_F(TerrainLayerTest, ResetClearsLethalConfirmation)
{
  for (int i = 0; i < 3; ++i) {
    publishAndUpdate(makeTerrainMessage(
      lifecycle_node_->get_clock()->now().nanoseconds(), true));
  }
  ASSERT_EQ(centerCost(), nav2_costmap_2d::LETHAL_OBSTACLE);
  layer_->reset();
  publishAndUpdate(makeTerrainMessage(
    lifecycle_node_->get_clock()->now().nanoseconds(), true));
  EXPECT_EQ(centerCost(), 252);
}

TEST_F(TerrainLayerTest, SoftCostAppliesBeforeLethalConfirmation)
{
  auto message = makeTerrainMessage(lifecycle_node_->get_clock()->now().nanoseconds());
  const auto it = std::find(message.layers.begin(), message.layers.end(), "slope");
  ASSERT_NE(it, message.layers.end());
  const auto index = static_cast<std::size_t>(std::distance(message.layers.begin(), it));
  std::fill(message.data[index].data.begin(), message.data[index].data.end(), 7.0f);
  publishAndUpdate(message);
  EXPECT_GT(centerCost(), 0);
  EXPECT_LT(centerCost(), nav2_costmap_2d::LETHAL_OBSTACLE);
}

class TerrainLayerSoftResidualTest : public TerrainLayerTest
{
protected:
  bool useSoftResidual() const override {return true;}
};

TEST_F(TerrainLayerSoftResidualTest, ResidualNeverConfirmsButSlopeStillDoes)
{
  for (int i = 0; i < 4; ++i) {
    publishAndUpdate(makeTerrainMessage(
      lifecycle_node_->get_clock()->now().nanoseconds(), false, false, false,
      0.0, false, true));
  }
  EXPECT_EQ(centerCost(), 252);
  for (int i = 0; i < 3; ++i) {
    publishAndUpdate(makeTerrainMessage(
      lifecycle_node_->get_clock()->now().nanoseconds(), true));
  }
  EXPECT_EQ(centerCost(), nav2_costmap_2d::LETHAL_OBSTACLE);
}

TEST_F(TerrainLayerTest, FollowsRollingCostmapOrigin)
{
  const auto message = makeTerrainMessage(lifecycle_node_->get_clock()->now().nanoseconds());
  publishAndUpdate(message);
  ASSERT_TRUE(layer_->isCurrent());

  layered_costmap_->updateMap(0.7, 0.0, 0.0);
  const auto * costmap = layered_costmap_->getCostmap();
  EXPECT_DOUBLE_EQ(layer_->getOriginX(), costmap->getOriginX());
  EXPECT_DOUBLE_EQ(layer_->getOriginY(), costmap->getOriginY());
}

TEST_F(TerrainLayerTest, RollingOriginPreservesWorldLethalConfirmation)
{
  for (int i = 0; i < 2; ++i) {
    publishAndUpdate(makeTerrainMessage(
      lifecycle_node_->get_clock()->now().nanoseconds(), true));
  }
  layered_costmap_->updateMap(0.7, 0.0, 0.0);
  publishAndUpdate(makeTerrainMessage(
    lifecycle_node_->get_clock()->now().nanoseconds(), true));
  EXPECT_EQ(centerCost(), nav2_costmap_2d::LETHAL_OBSTACLE);
}

// Costmap2D's default constructor leaves default_value_ indeterminate.  A full layer
// refresh uses resetMaps(), so any value other than NO_INFORMATION can be stamped onto
// the master costmap by updateWithMax().
TEST_F(TerrainLayerTest, DefaultValueIsNoInformation)
{
  EXPECT_EQ(layer_->getDefaultValue(), nav2_costmap_2d::NO_INFORMATION);
}

TEST_F(TerrainLayerTest, CellsOutsideGridMapKeepMasterCost)
{
  // The terrain message spans 2.0 x 1.0 m about the origin, so y = -0.8 lies outside it
  // while staying inside the 2 x 2 m costmap.
  publishAndUpdate(makeTerrainMessage(lifecycle_node_->get_clock()->now().nanoseconds()));
  ASSERT_TRUE(layer_->isCurrent());

  const auto * costmap = layered_costmap_->getCostmap();
  unsigned int mx;
  unsigned int my;
  ASSERT_TRUE(costmap->worldToMap(0.0, -0.8, mx, my));
  EXPECT_EQ(costmap->getCost(mx, my), nav2_costmap_2d::FREE_SPACE);
}

TEST_F(TerrainLayerTest, FullBoundsOnlyForNewMapOrRollingOriginChange)
{
  publishAndUpdate(makeTerrainMessage(lifecycle_node_->get_clock()->now().nanoseconds()));
  ASSERT_TRUE(layer_->isCurrent());

  double min_x = 0.2;
  double min_y = 0.3;
  double max_x = 0.4;
  double max_y = 0.5;
  layer_->updateBounds(0.0, 0.0, 0.0, &min_x, &min_y, &max_x, &max_y);
  EXPECT_DOUBLE_EQ(min_x, 0.2);
  EXPECT_DOUBLE_EQ(min_y, 0.3);
  EXPECT_DOUBLE_EQ(max_x, 0.4);
  EXPECT_DOUBLE_EQ(max_y, 0.5);

  const auto stamp = lifecycle_node_->get_clock()->now().nanoseconds() - 10000000;
  publisher_->publish(makeTerrainMessage(stamp));
  executor_.spin_some();
  min_x = 0.0;
  min_y = 0.0;
  max_x = 0.0;
  max_y = 0.0;
  layer_->updateBounds(0.0, 0.0, 0.0, &min_x, &min_y, &max_x, &max_y);
  EXPECT_DOUBLE_EQ(min_x, layer_->getOriginX());
  EXPECT_DOUBLE_EQ(min_y, layer_->getOriginY());
  EXPECT_DOUBLE_EQ(max_x, layer_->getOriginX() + layer_->getSizeInMetersX());
  EXPECT_DOUBLE_EQ(max_y, layer_->getOriginY() + layer_->getSizeInMetersY());

  layer_->updateCosts(*layered_costmap_->getCostmap(), 0, 0, 20, 20);
  min_x = 0.0;
  min_y = 0.0;
  max_x = 0.0;
  max_y = 0.0;
  layer_->updateBounds(0.7, 0.0, 0.0, &min_x, &min_y, &max_x, &max_y);
  EXPECT_DOUBLE_EQ(min_x, layer_->getOriginX());
  EXPECT_DOUBLE_EQ(min_y, layer_->getOriginY());
  EXPECT_DOUBLE_EQ(max_x, layer_->getOriginX() + layer_->getSizeInMetersX());
  EXPECT_DOUBLE_EQ(max_y, layer_->getOriginY() + layer_->getSizeInMetersY());
}

TEST_F(TerrainLayerTest, PartialUpdatePreservesTerrainOutsideItsBounds)
{
  for (int i = 0; i < 3; ++i) {
    publishAndUpdate(makeTerrainMessage(
      lifecycle_node_->get_clock()->now().nanoseconds(), true));
  }
  unsigned int mx;
  unsigned int my;
  ASSERT_TRUE(layered_costmap_->getCostmap()->worldToMap(0.05, 0.05, mx, my));
  ASSERT_EQ(layer_->getCost(mx, my), nav2_costmap_2d::LETHAL_OBSTACLE);

  layer_->updateCosts(*layered_costmap_->getCostmap(), 0, 0, 1, 1);
  EXPECT_EQ(layer_->getCost(mx, my), nav2_costmap_2d::LETHAL_OBSTACLE);
}
}  // namespace
