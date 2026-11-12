#include <cmath>

#include <gtest/gtest.h>

#include <small_gicp_relocalization/pose_utils.hpp>

namespace {

geometry_msgs::msg::Pose makePose(double x, double y, double z, double yaw) {
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  pose.orientation.w = std::cos(yaw / 2.0);
  pose.orientation.z = std::sin(yaw / 2.0);
  return pose;
}

}  // namespace

TEST(PoseUtils, InitialPoseIsPlanarMapOdomCorrection) {
  const auto result = small_gicp_relocalization::planarPoseToMapOdom(
    makePose(1.0, -2.0, 4.0, 0.35));

  EXPECT_DOUBLE_EQ(result.translation().x(), 1.0);
  EXPECT_DOUBLE_EQ(result.translation().y(), -2.0);
  EXPECT_DOUBLE_EQ(result.translation().z(), 0.0);
  EXPECT_NEAR(result.rotation().eulerAngles(0, 1, 2).z(), 0.35, 1e-12);
  EXPECT_NEAR(result.rotation().eulerAngles(0, 1, 2).x(), 0.0, 1e-12);
  EXPECT_NEAR(result.rotation().eulerAngles(0, 1, 2).y(), 0.0, 1e-12);
}

TEST(PoseUtils, PredictionRetainsCurrentBodyAttitude) {
  const Eigen::Isometry3d map_odom =
    Eigen::Translation3d(1.0, 2.0, 0.0) *
    Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitZ());
  const Eigen::Isometry3d odom_body =
    Eigen::Translation3d(3.0, 4.0, 0.5) *
    Eigen::AngleAxisd(-0.7, Eigen::Vector3d::UnitY());

  const Eigen::Isometry3d prediction =
    small_gicp_relocalization::predictMapBody(map_odom, odom_body);
  EXPECT_NEAR(prediction.translation().x(), 1.0 + 3.0 * std::cos(0.2) - 4.0 * std::sin(0.2), 1e-12);
  EXPECT_NEAR(prediction.translation().y(), 2.0 + 3.0 * std::sin(0.2) + 4.0 * std::cos(0.2), 1e-12);
  EXPECT_NEAR(
    small_gicp_relocalization::rotationCorrection(prediction, map_odom * odom_body), 0.0, 1e-12);
}

TEST(PoseUtils, InitialPoseUsesSameTimePlanarOdomBody) {
  const Eigen::Isometry3d odom_body =
    Eigen::Translation3d(3.0, 1.0, 0.5) *
    Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(-0.7, Eigen::Vector3d::UnitY());
  const Eigen::Isometry3d initial_map_odom =
    small_gicp_relocalization::initialMapOdomFromClickedPose(
    makePose(10.0, 2.0, 0.0, 0.3), odom_body);
  const Eigen::Isometry3d predicted_map_body =
    small_gicp_relocalization::predictMapBody(initial_map_odom, odom_body);

  EXPECT_NEAR(predicted_map_body.translation().x(), 10.0, 1e-12);
  EXPECT_NEAR(predicted_map_body.translation().y(), 2.0, 1e-12);
  EXPECT_NEAR(predicted_map_body.translation().z(), 0.5, 1e-12);
  const Eigen::Isometry3d clicked_map_body =
    small_gicp_relocalization::planarPoseToMapOdom(makePose(10.0, 2.0, 0.0, 0.3));
  EXPECT_NEAR(
    small_gicp_relocalization::rotationCorrection(predicted_map_body, clicked_map_body),
    0.7, 1e-12);
}

TEST(PoseUtils, MapOdomUsesSameTimeOdomBody) {
  Eigen::Isometry3d map_body = Eigen::Isometry3d::Identity();
  map_body.translation() = Eigen::Vector3d(5.0, 2.0, 1.0);
  Eigen::Isometry3d odom_body = Eigen::Isometry3d::Identity();
  odom_body.translation() = Eigen::Vector3d(3.0, 1.0, 0.5);
  const Eigen::Isometry3d map_odom =
    small_gicp_relocalization::mapOdomFromMapBody(map_body, odom_body);

  EXPECT_TRUE((map_odom * odom_body).isApprox(map_body));
}

TEST(PoseUtils, CorrectionMetricsCompareAgainstPrediction) {
  const Eigen::Isometry3d prediction = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d candidate = Eigen::Isometry3d::Identity();
  candidate.translation().x() = 0.4;
  candidate.linear() = Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitZ()).toRotationMatrix();

  EXPECT_NEAR(
    small_gicp_relocalization::translationCorrection(candidate, prediction), 0.4, 1e-12);
  EXPECT_NEAR(
    small_gicp_relocalization::rotationCorrection(candidate, prediction), 0.2, 1e-12);
}

TEST(PoseUtils, RegistrationGateAppliesToTheFirstResult) {
  EXPECT_FALSE(small_gicp_relocalization::registrationPassesGate(
    true, 100, 50, 2.1, 0.1, 2.0, 0.5));
  EXPECT_FALSE(small_gicp_relocalization::registrationPassesGate(
    true, 100, 50, 0.1, 0.6, 2.0, 0.5));
  EXPECT_FALSE(small_gicp_relocalization::registrationPassesGate(
    false, 100, 50, 0.1, 0.1, 2.0, 0.5));
  EXPECT_TRUE(small_gicp_relocalization::registrationPassesGate(
    true, 50, 50, 0.1, 0.1, 2.0, 0.5));
}
