#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <fastdem/bridge/ros2.hpp>
#include <fastdem/io/npz.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <gtest/gtest.h>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <nanogrid/bridge/ros2.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav2_costmap_2d/layered_costmap.hpp>
#include <nav2_util/lifecycle_node.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
#include <tf2_ros/buffer.h>

#include "nav2_terrain_layer/static_terrain_layer.hpp"

namespace
{
class StaticTerrainLayerTest : public ::testing::Test
{
protected:
  virtual bool useSoftFeatureCosts() const {return false;}
  virtual float staticResidual() const {return 0.0F;}
  virtual float staticSlope() const {return 35.0F;}
  virtual double liveResidualTimeout() const {return 1.5;}
  virtual void configureStaticMap(fastdem::ElevationMap &) {}

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
    terrain_map_file_ = std::filesystem::temp_directory_path() /
      "static_terrain_layer_test.npz";
    fastdem::ElevationMap map(2.0f, 2.0f, 0.1f, "map");
    for (const auto & layer : {std::string("slope"), std::string("step_residual"),
      std::string("roughness"), std::string("n_points"), std::string("class"),
      std::string("ground_source"), std::string("clearance")}) {
      if (!map.exists(layer)) map.add(layer, NAN);
    }
    map.get("elevation").setZero();
    map.get("slope").setConstant(staticSlope());
    map.get("step_residual").setConstant(staticResidual());
    map.get("roughness").setZero();
    map.get("n_points").setConstant(3.0f);
    map.get("class").setConstant(1.0f);
    map.get("ground_source").setConstant(1.0f);
    map.get("clearance").setConstant(std::numeric_limits<float>::infinity());
    configureStaticMap(map);
    ASSERT_TRUE(fastdem::io::saveNpz(terrain_map_file_.string(), map));

    rclcpp::NodeOptions options;
    options.append_parameter_override(
      "static_terrain_layer.terrain_map_file", terrain_map_file_.string());
    options.append_parameter_override(
      "static_terrain_layer.hard_feature_constraints", !useSoftFeatureCosts());
    options.append_parameter_override("static_terrain_layer.min_observations", 3);
    options.append_parameter_override(
      "static_terrain_layer.live_terrain_topic", std::string("/test/static_live_terrain"));
    options.append_parameter_override(
      "static_terrain_layer.live_residual_timeout", liveResidualTimeout());
    lifecycle_node_ = std::make_shared<nav2_util::LifecycleNode>(
      "static_terrain_layer_test", "", options);
    layered_costmap_ = std::make_unique<nav2_costmap_2d::LayeredCostmap>("map", false, false);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(lifecycle_node_->get_clock());
    layer_ = std::make_shared<nav2_terrain_layer::StaticTerrainLayer>();
    layer_->initialize(
      layered_costmap_.get(), "static_terrain_layer", tf_buffer_.get(), lifecycle_node_,
      lifecycle_node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive));
    layered_costmap_->addPlugin(layer_);
    layered_costmap_->getCostmap()->resizeMap(20, 20, 0.1, -1.0, -1.0);
    publisher_node_ = std::make_shared<rclcpp::Node>("static_terrain_layer_test_publisher");
    publisher_ = publisher_node_->create_publisher<grid_map_msgs::msg::GridMap>(
      "/test/static_live_terrain", rclcpp::QoS(10));
    global_terrain_subscription_ =
      publisher_node_->create_subscription<grid_map_msgs::msg::GridMap>(
      "/navigation/global_terrain_map", rclcpp::QoS(1).reliable().transient_local(),
      [this](grid_map_msgs::msg::GridMap::ConstSharedPtr message) {
        global_terrain_snapshot_ = *message;
        global_terrain_revisions_.emplace_back(
          message->header.stamp.sec, message->header.stamp.nanosec);
      });
    map_publisher_ = publisher_node_->create_publisher<nav_msgs::msg::OccupancyGrid>(
      "/map", rclcpp::QoS(1).reliable().transient_local());
    terrain_changed_subscription_ = publisher_node_->create_subscription<std_msgs::msg::Empty>(
      "/navigation/terrain_map_changed", rclcpp::QoS(1).reliable(),
      [this](std_msgs::msg::Empty::ConstSharedPtr) {
        nav_msgs::msg::OccupancyGrid map;
        map.header.frame_id = "map";
        const auto stamp_ns = lifecycle_node_->get_clock()->now().nanoseconds();
        map.header.stamp.sec = static_cast<std::int32_t>(stamp_ns / 1000000000LL);
        map.header.stamp.nanosec = static_cast<std::uint32_t>(stamp_ns % 1000000000LL);
        map.info.resolution = 0.1F;
        map.info.width = 20;
        map.info.height = 20;
        map.info.origin.position.x = -1.0;
        map.info.origin.position.y = -1.0;
        map.info.origin.orientation.w = 1.0;
        map.data.assign(400, 0);
        map_publisher_->publish(map);
      });
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
    lifecycle_node_.reset();
    publisher_.reset();
    global_terrain_subscription_.reset();
    terrain_changed_subscription_.reset();
    map_publisher_.reset();
    publisher_node_.reset();
    std::error_code error;
    std::filesystem::remove(terrain_map_file_, error);
  }

  unsigned char centerCost() const
  {
    const auto * costmap = layered_costmap_->getCostmap();
    unsigned int mx;
    unsigned int my;
    if (!costmap->worldToMap(0.0, 0.0, mx, my)) return 0;
    return costmap->getCost(mx, my);
  }

  float centerGroundSource() const
  {
    return groundSourceAt(nanogrid::Position(0.0, 0.0));
  }

  float groundSourceAt(const nanogrid::Position & position) const
  {
    if (!global_terrain_snapshot_) return NAN;
    const auto map = nanogrid::ros2::fromMsg(*global_terrain_snapshot_);
    const auto index = map.index(position);
    if (!index || !map.exists("ground_source")) return NAN;
    return map.at("ground_source", *index);
  }

  float groundSourceAtLinearIndex(const Eigen::Index linear_index) const
  {
    if (!global_terrain_snapshot_) return NAN;
    const auto map = nanogrid::ros2::fromMsg(*global_terrain_snapshot_);
    if (!map.exists("ground_source")) return NAN;
    for (const auto cell : map.cells()) {
      if (cell.index == linear_index) return map.at("ground_source", cell);
    }
    return NAN;
  }

  float centerElevation() const
  {
    if (!global_terrain_snapshot_) return NAN;
    const auto map = nanogrid::ros2::fromMsg(*global_terrain_snapshot_);
    const auto center = map.index(nanogrid::Position(0.0, 0.0));
    if (!center || !map.exists("elevation")) return NAN;
    return map.at("elevation", *center);
  }

  float centerSlope() const
  {
    if (!global_terrain_snapshot_) return NAN;
    const auto map = nanogrid::ros2::fromMsg(*global_terrain_snapshot_);
    const auto center = map.index(nanogrid::Position(0.0, 0.0));
    if (!center || !map.exists("slope")) return NAN;
    return map.at("slope", *center);
  }

  float centerResidual() const
  {
    if (!global_terrain_snapshot_) return NAN;
    const auto map = nanogrid::ros2::fromMsg(*global_terrain_snapshot_);
    const auto center = map.index(nanogrid::Position(0.0, 0.0));
    if (!center || !map.exists("step_residual")) return NAN;
    return map.at("step_residual", *center);
  }

  float centerRoughness() const
  {
    if (!global_terrain_snapshot_) return NAN;
    const auto map = nanogrid::ros2::fromMsg(*global_terrain_snapshot_);
    const auto center = map.index(nanogrid::Position(0.0, 0.0));
    if (!center || !map.exists("roughness")) return NAN;
    return map.at("roughness", *center);
  }

  void setStaticMapFromOdomTransform(
    const double roll, const double translation_z, const double translation_y = 0.0)
  {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.frame_id = "map";
    transform.child_frame_id = "odom";
    const auto stamp_ns = lifecycle_node_->get_clock()->now().nanoseconds();
    transform.header.stamp.sec = static_cast<std::int32_t>(stamp_ns / 1000000000LL);
    transform.header.stamp.nanosec = static_cast<std::uint32_t>(stamp_ns % 1000000000LL);
    transform.transform.translation.z = translation_z;
    transform.transform.translation.y = translation_y;
    transform.transform.rotation.x = std::sin(roll * 0.5);
    transform.transform.rotation.w = std::cos(roll * 0.5);
    ASSERT_TRUE(tf_buffer_->setTransform(transform, "static_terrain_layer_test", true));
  }

  grid_map_msgs::msg::GridMap makeLiveTerrainMessage(
    const std::int64_t stamp_ns, const float residual_step,
    const bool sufficient_observations = true, const float slope = 0.0F,
    const float roughness = 0.0F, const float elevation = 0.0F,
    const std::string & frame_id = "map", const double length_x = 2.0,
    const double center_x = 0.0) const
  {
    fastdem::ElevationMap map(static_cast<float>(length_x), 2.0F, 0.1F, frame_id);
    for (const char * layer : {
        fastdem::layer::slope, fastdem::layer::step_residual, fastdem::layer::roughness,
        fastdem::layer::n_points, fastdem::layer::normal_x, fastdem::layer::normal_y,
        fastdem::layer::normal_z})
    {
      if (!map.exists(layer)) map.add(layer, NAN);
    }
    map.setPosition(nanogrid::Position(center_x, 0.0));
    map.setFrameId(frame_id);
    map.setTimestamp(static_cast<std::uint64_t>(stamp_ns));
    map.get(fastdem::layer::elevation).setConstant(elevation);
    map.get(fastdem::layer::slope).setConstant(slope);
    map.get(fastdem::layer::step_residual).setConstant(residual_step);
    map.get(fastdem::layer::roughness).setConstant(roughness);
    map.get(fastdem::layer::n_points).setConstant(sufficient_observations ? 3.0F : 2.0F);
    map.get(fastdem::layer::observation_age).setZero();
    const float slope_radians = slope * static_cast<float>(M_PI / 180.0);
    map.get(fastdem::layer::normal_x).setConstant(std::sin(slope_radians));
    map.get(fastdem::layer::normal_y).setZero();
    map.get(fastdem::layer::normal_z).setConstant(std::cos(slope_radians));
    return fastdem::ros2::toGridMap(
      map, fastdem::layer::elevation, /*include_surface_normals=*/true);
  }

  void publishLiveAndUpdate(const grid_map_msgs::msg::GridMap & message)
  {
    publisher_->publish(message);
    for (int i = 0; i < 20; ++i) {
      executor_.spin_some();
      layered_costmap_->updateMap(0.0, 0.0, 0.0);
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  }

  rclcpp::executors::SingleThreadedExecutor executor_;
  std::filesystem::path terrain_map_file_;
  std::shared_ptr<nav2_util::LifecycleNode> lifecycle_node_;
  std::shared_ptr<rclcpp::Node> publisher_node_;
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr publisher_;
  rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr global_terrain_subscription_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_publisher_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr terrain_changed_subscription_;
  std::optional<grid_map_msgs::msg::GridMap> global_terrain_snapshot_;
  std::vector<std::pair<std::int32_t, std::uint32_t>> global_terrain_revisions_;
  std::unique_ptr<nav2_costmap_2d::LayeredCostmap> layered_costmap_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<nav2_terrain_layer::StaticTerrainLayer> layer_;
};

class StaticTerrainLayerSoftResidualTest : public StaticTerrainLayerTest
{
protected:
  bool useSoftFeatureCosts() const override {return true;}
  float staticResidual() const override {return 0.04F;}
  float staticSlope() const override {return 0.0F;}
};

class StaticTerrainLayerShortResidualExpiryTest : public StaticTerrainLayerSoftResidualTest
{
protected:
  double liveResidualTimeout() const override {return 0.8;}
};

class StaticTerrainLayerUnknownGroundTest : public StaticTerrainLayerTest
{
protected:
  void configureStaticMap(fastdem::ElevationMap & map) override
  {
    const auto center = map.index(nanogrid::Position(0.0, 0.0));
    ASSERT_TRUE(center);
    map.at("class", *center) = 0.0F;
    map.at("ground_source", *center) = 0.0F;
  }
};

class StaticTerrainLayerResizeEvidenceTest : public StaticTerrainLayerTest
{
protected:
  void configureStaticMap(fastdem::ElevationMap & map) override
  {
    map.get("class")(15, 1) = 0.0F;
    map.get("ground_source")(15, 1) = 0.0F;
  }
};

TEST_F(StaticTerrainLayerTest, LoadsStaticMapAndAppliesLethalFeature)
{
  EXPECT_FALSE(layer_->isCurrent());
  layered_costmap_->updateMap(0.0, 0.0, 0.0);
  EXPECT_TRUE(layer_->isCurrent());
  EXPECT_EQ(layer_->getCost(10, 10), nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_EQ(centerCost(), nav2_costmap_2d::LETHAL_OBSTACLE);
}

TEST_F(StaticTerrainLayerUnknownGroundTest, KeepsUnknownBlockedUntilFreshMeasuredFrames)
{
  layered_costmap_->updateMap(0.0, 0.0, 0.0);
  EXPECT_EQ(centerCost(), nav2_costmap_2d::NO_INFORMATION);

  for (int frame = 0; frame < 3; ++frame) {
    publishLiveAndUpdate(makeLiveTerrainMessage(
      lifecycle_node_->get_clock()->now().nanoseconds(), 0.0F));
  }
  EXPECT_LT(centerCost(), nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_FLOAT_EQ(centerGroundSource(), 1.0F);
}

TEST_F(StaticTerrainLayerUnknownGroundTest, KeepsHighStepUnknownAndUnpromoted)
{
  layered_costmap_->updateMap(0.0, 0.0, 0.0);
  EXPECT_EQ(centerCost(), nav2_costmap_2d::NO_INFORMATION);

  for (int frame = 0; frame < 3; ++frame) {
    publishLiveAndUpdate(makeLiveTerrainMessage(
      lifecycle_node_->get_clock()->now().nanoseconds(), 0.04F));
  }

  EXPECT_EQ(centerCost(), nav2_costmap_2d::NO_INFORMATION);
  EXPECT_FLOAT_EQ(centerGroundSource(), 0.0F);
}

TEST_F(StaticTerrainLayerResizeEvidenceTest, ClearsLiveEvidenceWhenFusedMapGeometryChanges)
{
  layered_costmap_->updateMap(0.0, 0.0, 0.0);
  for (int frame = 0; frame < 2; ++frame) {
    publishLiveAndUpdate(makeLiveTerrainMessage(
      lifecycle_node_->get_clock()->now().nanoseconds(), 0.0F, true, 0.0F, 0.0F,
      0.0F, "map", 4.0, 0.5));
  }

  nav_msgs::msg::OccupancyGrid expanded_map;
  expanded_map.header.frame_id = "map";
  const auto stamp_ns = lifecycle_node_->get_clock()->now().nanoseconds();
  expanded_map.header.stamp.sec = static_cast<std::int32_t>(stamp_ns / 1000000000LL);
  expanded_map.header.stamp.nanosec = static_cast<std::uint32_t>(stamp_ns % 1000000000LL);
  expanded_map.info.resolution = 0.1F;
  expanded_map.info.width = 30;
  expanded_map.info.height = 20;
  expanded_map.info.origin.position.x = -1.0;
  expanded_map.info.origin.position.y = -1.0;
  expanded_map.info.origin.orientation.w = 1.0;
  expanded_map.data.assign(600, 0);
  map_publisher_->publish(expanded_map);
  for (int i = 0; i < 20; ++i) {
    executor_.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  layered_costmap_->resizeMap(30, 20, 0.1, -1.0, -1.0);
  layered_costmap_->updateMap(0.0, 0.0, 0.0);

  publishLiveAndUpdate(makeLiveTerrainMessage(
    lifecycle_node_->get_clock()->now().nanoseconds(), 0.0F, true, 0.0F, 0.0F,
    0.0F, "map", 4.0, 0.5));

  EXPECT_FLOAT_EQ(groundSourceAtLinearIndex(35), 0.0F);
}

TEST_F(StaticTerrainLayerUnknownGroundTest, PromotesMeasuredTraversableThreeDimensionalTransform)
{
  constexpr double kRoll = 0.08;
  constexpr double kTranslationZ = 1.0;
  constexpr float kOdomElevation = 2.0F;
  const auto source_message = makeLiveTerrainMessage(
    lifecycle_node_->get_clock()->now().nanoseconds(), 0.02F, true, 0.0F, 0.015F,
    kOdomElevation, "odom");
  const auto source_map = nanogrid::ros2::fromMsg(source_message);
  const auto source_index = source_map.index(nanogrid::Position(0.0, 0.0));
  ASSERT_TRUE(source_index);
  const auto source_position = source_map.position(*source_index);
  ASSERT_TRUE(source_position);
  const double source_y = source_position->y();
  const double translation_y =
    std::sin(kRoll) * kOdomElevation - std::cos(kRoll) * source_y;
  setStaticMapFromOdomTransform(kRoll, kTranslationZ, translation_y);
  layered_costmap_->updateMap(0.0, 0.0, 0.0);

  for (int frame = 0; frame < 3; ++frame) {
    publishLiveAndUpdate(makeLiveTerrainMessage(
      lifecycle_node_->get_clock()->now().nanoseconds(), 0.02F, true, 0.0F, 0.015F,
      kOdomElevation, "odom"));
  }

  const auto expected_map_elevation = static_cast<float>(
    kTranslationZ + std::sin(kRoll) * source_y + std::cos(kRoll) * kOdomElevation);
  EXPECT_NEAR(centerElevation(), expected_map_elevation, 1.0e-4F);
  EXPECT_GT(centerElevation(), kOdomElevation + 0.5F);
  EXPECT_NEAR(centerSlope(), kRoll * 180.0 / M_PI, 0.05);
  EXPECT_NEAR(centerResidual(), 0.02F / std::cos(kRoll), 1.0e-4F);
  EXPECT_NEAR(centerRoughness(), 0.015F, 1.0e-5F);
  EXPECT_FLOAT_EQ(centerGroundSource(), 1.0F);
}

TEST_F(StaticTerrainLayerUnknownGroundTest, RejectsOverLimitThreeDimensionalSlope)
{
  constexpr double kRoll = 0.4;
  constexpr double kTranslationZ = 1.0;
  constexpr float kOdomElevation = 2.0F;
  const auto source_message = makeLiveTerrainMessage(
    lifecycle_node_->get_clock()->now().nanoseconds(), 0.02F, true, 0.0F, 0.015F,
    kOdomElevation, "odom");
  const auto source_map = nanogrid::ros2::fromMsg(source_message);
  const auto source_index = source_map.index(nanogrid::Position(0.0, 0.0));
  ASSERT_TRUE(source_index);
  const auto source_position = source_map.position(*source_index);
  ASSERT_TRUE(source_position);
  const double source_y = source_position->y();
  const double translation_y =
    std::sin(kRoll) * kOdomElevation - std::cos(kRoll) * source_y;
  setStaticMapFromOdomTransform(kRoll, kTranslationZ, translation_y);
  layered_costmap_->updateMap(0.0, 0.0, 0.0);

  for (int frame = 0; frame < 3; ++frame) {
    publishLiveAndUpdate(makeLiveTerrainMessage(
      lifecycle_node_->get_clock()->now().nanoseconds(), 0.02F, true, 0.0F, 0.015F,
      kOdomElevation, "odom"));
  }

  EXPECT_FLOAT_EQ(centerGroundSource(), 0.0F);
}

TEST_F(StaticTerrainLayerTest, AppliesStaticTerrainToAnExpandedAlignedCostmap)
{
  EXPECT_NO_THROW(layered_costmap_->resizeMap(30, 20, 0.1, -1.0, -1.0));
  layered_costmap_->updateMap(0.0, 0.0, 0.0);
  EXPECT_TRUE(layer_->isCurrent());
  EXPECT_EQ(centerCost(), nav2_costmap_2d::LETHAL_OBSTACLE);
}

TEST_F(StaticTerrainLayerTest, ResizesGlobalTerrainToFusedMapAndMatchesItsRevision)
{
  nav_msgs::msg::OccupancyGrid map;
  map.header.frame_id = "map";
  const auto stamp_ns = lifecycle_node_->get_clock()->now().nanoseconds();
  map.header.stamp.sec = static_cast<std::int32_t>(stamp_ns / 1000000000LL);
  map.header.stamp.nanosec = static_cast<std::uint32_t>(stamp_ns % 1000000000LL);
  map.info.resolution = 0.1F;
  map.info.width = 30;
  map.info.height = 20;
  map.info.origin.position.x = -1.5;
  map.info.origin.position.y = -1.0;
  map.info.origin.orientation.w = 1.0;
  map.data.assign(600, 0);
  map_publisher_->publish(map);
  for (int i = 0; i < 20; ++i) {
    executor_.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  ASSERT_TRUE(global_terrain_snapshot_);
  EXPECT_EQ(global_terrain_snapshot_->header.stamp.sec, map.header.stamp.sec);
  EXPECT_EQ(global_terrain_snapshot_->header.stamp.nanosec, map.header.stamp.nanosec);
  const auto terrain = nanogrid::ros2::fromMsg(*global_terrain_snapshot_);
  EXPECT_EQ(terrain.getSize()(0), 30);
  EXPECT_EQ(terrain.getSize()(1), 20);
  EXPECT_NEAR(terrain.getPosition().x(), 0.0, 1.0e-5);
  EXPECT_NEAR(terrain.getPosition().y(), 0.0, 1.0e-5);
}

TEST_F(StaticTerrainLayerTest, PublishesEachFusedMapRevisionWithItsMatchingTerrainSnapshot)
{
  nav_msgs::msg::OccupancyGrid map;
  map.header.frame_id = "map";
  map.info.resolution = 0.1F;
  map.info.width = 20;
  map.info.height = 20;
  map.info.origin.position.x = -1.0;
  map.info.origin.position.y = -1.0;
  map.info.origin.orientation.w = 1.0;
  map.data.assign(400, 0);

  const auto publish_revision = [this, &map](std::int64_t stamp_ns) {
      map.header.stamp.sec = static_cast<std::int32_t>(stamp_ns / 1000000000LL);
      map.header.stamp.nanosec = static_cast<std::uint32_t>(stamp_ns % 1000000000LL);
      map_publisher_->publish(map);
      for (int i = 0; i < 20; ++i) {
        executor_.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
      }
      ASSERT_TRUE(global_terrain_snapshot_);
      EXPECT_EQ(
        global_terrain_snapshot_->header.stamp.sec, map.header.stamp.sec);
      EXPECT_EQ(
        global_terrain_snapshot_->header.stamp.nanosec, map.header.stamp.nanosec);
    };

  const auto first_revision = lifecycle_node_->get_clock()->now().nanoseconds();
  publish_revision(first_revision);
  const auto second_revision = first_revision + 1000000LL;
  publish_revision(second_revision);
}

TEST_F(
  StaticTerrainLayerUnknownGroundTest,
  KeepsFusedMapRevisionsMatchedWhileGroundPromotionRuns)
{
  layered_costmap_->updateMap(0.0, 0.0, 0.0);
  for (int frame = 0; frame < 2; ++frame) {
    publishLiveAndUpdate(makeLiveTerrainMessage(
      lifecycle_node_->get_clock()->now().nanoseconds(), 0.0F));
  }
  EXPECT_EQ(centerGroundSource(), 0.0F);

  std::atomic<bool> keep_updating{true};
  std::thread updater([this, &keep_updating]() {
      while (keep_updating.load()) {
        layered_costmap_->updateMap(0.0, 0.0, 0.0);
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
      }
    });

  // The live callback and costmap thread can promote this unknown ground
  // cell while the executor processes the following fused-map revisions.
  publisher_->publish(makeLiveTerrainMessage(
    lifecycle_node_->get_clock()->now().nanoseconds(), 0.0F));
  executor_.spin_some();

  nav_msgs::msg::OccupancyGrid map;
  map.header.frame_id = "map";
  map.info.resolution = 0.1F;
  map.info.width = 20;
  map.info.height = 20;
  map.info.origin.position.x = -1.0;
  map.info.origin.position.y = -1.0;
  map.info.origin.orientation.w = 1.0;
  map.data.assign(400, 0);
  const auto first_stamp_ns = lifecycle_node_->get_clock()->now().nanoseconds();
  std::vector<std::pair<std::int32_t, std::uint32_t>> expected_revisions;
  for (int revision = 0; revision < 12; ++revision) {
    const auto stamp_ns = first_stamp_ns + revision * 1000000LL;
    map.header.stamp.sec = static_cast<std::int32_t>(stamp_ns / 1000000000LL);
    map.header.stamp.nanosec = static_cast<std::uint32_t>(stamp_ns % 1000000000LL);
    expected_revisions.emplace_back(map.header.stamp.sec, map.header.stamp.nanosec);
    map_publisher_->publish(map);
    for (int spin = 0; spin < 10; ++spin) {
      executor_.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
  }

  keep_updating.store(false);
  updater.join();
  for (int spin = 0; spin < 20; ++spin) {
    executor_.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  EXPECT_EQ(centerGroundSource(), 1.0F);
  for (const auto & revision : expected_revisions) {
    EXPECT_NE(
      std::find(
        global_terrain_revisions_.begin(), global_terrain_revisions_.end(), revision),
      global_terrain_revisions_.end());
  }
}

TEST_F(StaticTerrainLayerTest, RejectsAlignedCostmapThatDoesNotCoverTerrainMap)
{
  layered_costmap_->resizeMap(10, 20, 0.1, -0.5, -1.0);
  layered_costmap_->updateMap(0.0, 0.0, 0.0);
  EXPECT_FALSE(layer_->isCurrent());
}

TEST_F(StaticTerrainLayerTest, RejectsMisalignedCostmapOrigin)
{
  layered_costmap_->resizeMap(20, 20, 0.1, -0.95, -1.0);
  layered_costmap_->updateMap(0.0, 0.0, 0.0);
  EXPECT_FALSE(layer_->isCurrent());

  layered_costmap_->resizeMap(20, 20, 0.1, -1.0, -1.0);
  layered_costmap_->updateMap(0.0, 0.0, 0.0);
  EXPECT_TRUE(layer_->isCurrent());
}

TEST_F(StaticTerrainLayerTest, RejectsResolutionMismatch)
{
  layered_costmap_->resizeMap(10, 10, 0.2, -1.0, -1.0);
  layered_costmap_->updateMap(0.0, 0.0, 0.0);
  EXPECT_FALSE(layer_->isCurrent());

  layered_costmap_->resizeMap(20, 20, 0.1, -1.0, -1.0);
  layered_costmap_->updateMap(0.0, 0.0, 0.0);
  EXPECT_TRUE(layer_->isCurrent());
}

// Costmap2D's default constructor leaves default_value_ indeterminate, and resetMaps()
// refills the layer with it.  Only NO_INFORMATION keeps updateWithMax() from stamping it
// onto cells the loaded terrain map does not cover.
TEST_F(StaticTerrainLayerTest, DefaultValueIsNoInformation)
{
  EXPECT_EQ(layer_->getDefaultValue(), nav2_costmap_2d::NO_INFORMATION);
}

TEST_F(StaticTerrainLayerTest, CellsOutsideTerrainMapKeepMasterCost)
{
  // The terrain map spans 2 x 2 m about the origin; widening the costmap to 3 m in x
  // leaves x > 1.0 uncovered.
  ASSERT_NO_THROW(layered_costmap_->resizeMap(30, 20, 0.1, -1.0, -1.0));
  layered_costmap_->updateMap(0.0, 0.0, 0.0);
  ASSERT_TRUE(layer_->isCurrent());

  const auto * costmap = layered_costmap_->getCostmap();
  unsigned int mx;
  unsigned int my;
  ASSERT_TRUE(costmap->worldToMap(1.5, 0.0, mx, my));
  EXPECT_EQ(costmap->getCost(mx, my), nav2_costmap_2d::FREE_SPACE);
}

TEST_F(StaticTerrainLayerSoftResidualTest, LiveLowResidualReplacesStaticResidualAfterThreeFrames)
{
  layered_costmap_->updateMap(0.0, 0.0, 0.0);
  ASSERT_EQ(centerCost(), 252);
  for (int frame = 0; frame < 2; ++frame) {
    publishLiveAndUpdate(makeLiveTerrainMessage(
      lifecycle_node_->get_clock()->now().nanoseconds(), 0.0F));
    EXPECT_EQ(centerCost(), 252);
  }
  publishLiveAndUpdate(makeLiveTerrainMessage(
    lifecycle_node_->get_clock()->now().nanoseconds(), 0.0F));
  EXPECT_EQ(centerCost(), nav2_costmap_2d::FREE_SPACE);
}

TEST_F(StaticTerrainLayerSoftResidualTest, LiveFiveCentimeterStepBecomesLethalAfterConfirmation)
{
  layered_costmap_->updateMap(0.0, 0.0, 0.0);
  ASSERT_EQ(centerCost(), 252);
  for (int frame = 0; frame < 2; ++frame) {
    publishLiveAndUpdate(makeLiveTerrainMessage(
        lifecycle_node_->get_clock()->now().nanoseconds(), 0.05F));
    EXPECT_EQ(centerCost(), 252);
  }
  publishLiveAndUpdate(makeLiveTerrainMessage(
      lifecycle_node_->get_clock()->now().nanoseconds(), 0.05F));
  EXPECT_EQ(centerCost(), nav2_costmap_2d::LETHAL_OBSTACLE);
}

TEST_F(StaticTerrainLayerSoftResidualTest, OppositeConfirmedEvidenceRestoresStaticResidualCost)
{
  layered_costmap_->updateMap(0.0, 0.0, 0.0);
  for (int frame = 0; frame < 3; ++frame) {
    publishLiveAndUpdate(makeLiveTerrainMessage(
      lifecycle_node_->get_clock()->now().nanoseconds(), 0.0F));
  }
  ASSERT_EQ(centerCost(), nav2_costmap_2d::FREE_SPACE);

  publishLiveAndUpdate(makeLiveTerrainMessage(
    lifecycle_node_->get_clock()->now().nanoseconds(), 0.05F));
  EXPECT_EQ(centerCost(), 252);
}

TEST_F(StaticTerrainLayerShortResidualExpiryTest, ExpiredLiveEvidenceRestoresStaticResidualCost)
{
  layered_costmap_->updateMap(0.0, 0.0, 0.0);
  for (int frame = 0; frame < 3; ++frame) {
    publishLiveAndUpdate(makeLiveTerrainMessage(
      lifecycle_node_->get_clock()->now().nanoseconds(), 0.0F));
  }
  ASSERT_EQ(centerCost(), nav2_costmap_2d::FREE_SPACE);

  std::this_thread::sleep_for(std::chrono::milliseconds(900));
  layered_costmap_->updateMap(0.0, 0.0, 0.0);
  EXPECT_EQ(centerCost(), 252);
}

TEST_F(StaticTerrainLayerSoftResidualTest, ResetClearsLiveEvidenceAndRebuildsStaticTerrainCost)
{
  layered_costmap_->updateMap(0.0, 0.0, 0.0);
  for (int frame = 0; frame < 3; ++frame) {
    publishLiveAndUpdate(makeLiveTerrainMessage(
      lifecycle_node_->get_clock()->now().nanoseconds(), 0.0F));
  }
  ASSERT_EQ(centerCost(), nav2_costmap_2d::FREE_SPACE);

  layer_->reset();
  layered_costmap_->updateMap(0.0, 0.0, 0.0);
  EXPECT_EQ(centerCost(), 252);
}

TEST_F(StaticTerrainLayerSoftResidualTest, MissingLiveSupportRetainsTheCachedTerrainCost)
{
  layered_costmap_->updateMap(0.0, 0.0, 0.0);
  for (int frame = 0; frame < 3; ++frame) {
    publishLiveAndUpdate(makeLiveTerrainMessage(
      lifecycle_node_->get_clock()->now().nanoseconds(), 0.0F));
  }
  ASSERT_EQ(centerCost(), nav2_costmap_2d::FREE_SPACE);

  for (int frame = 0; frame < 3; ++frame) {
    publishLiveAndUpdate(makeLiveTerrainMessage(
      lifecycle_node_->get_clock()->now().nanoseconds(), 0.0F, false));
  }
  EXPECT_EQ(centerCost(), 252);
}

}  // namespace
