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

Eigen::Isometry3d poseTransform(const geometry_msgs::msg::Pose & pose) {
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.translation() = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
  result.linear() = Eigen::Quaterniond(
    pose.orientation.w, pose.orientation.x, pose.orientation.y,
    pose.orientation.z).normalized().toRotationMatrix();
  return result;
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

TEST(PoseUtils, ClickedBasePoseUsesSameTimeFullOdometryAndBaseOffset) {
  geometry_msgs::msg::Pose clicked = makePose(4.0, -2.0, 1.3, 0.6);
  const Eigen::Isometry3d odom_base =
    Eigen::Translation3d(1.2, 0.7, 0.8) *
    Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(-0.08, Eigen::Vector3d::UnitY());
  const auto map_odom = small_gicp_relocalization::mapOdomFromClickedBasePose(
    clicked, odom_base);
  const auto recovered_map_base = map_odom * odom_base;

  EXPECT_NEAR(recovered_map_base.translation().x(), clicked.position.x, 1e-12);
  EXPECT_NEAR(recovered_map_base.translation().y(), clicked.position.y, 1e-12);
  EXPECT_NEAR(recovered_map_base.translation().z(), clicked.position.z, 1e-12);
  EXPECT_NEAR(
    small_gicp_relocalization::rotationCorrection(
      recovered_map_base, poseTransform(clicked)),
    0.0, 1e-12);
}

TEST(PoseUtils, ClickedPlanarPoseKeepsTrustedHeightAndTilt) {
  const Eigen::Isometry3d trusted_map_base =
    Eigen::Translation3d(1.0, 2.0, 0.73) *
    Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(-0.12, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(0.06, Eigen::Vector3d::UnitX());
  const auto clicked = makePose(3.0, -1.0, 0.0, -0.8);
  const auto result = small_gicp_relocalization::clickedPlanarPoseWithReferenceHeightAndTilt(
    clicked, trusted_map_base);
  const auto result_transform = poseTransform(result);

  EXPECT_DOUBLE_EQ(result.position.x, 3.0);
  EXPECT_DOUBLE_EQ(result.position.y, -1.0);
  EXPECT_NEAR(result.position.z, 0.73, 1e-12);
  Eigen::Isometry3d expected_transform = Eigen::Isometry3d::Identity();
  expected_transform.linear() =
    Eigen::AngleAxisd(-0.8, Eigen::Vector3d::UnitZ()).toRotationMatrix() *
    (Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(-0.12, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(0.06, Eigen::Vector3d::UnitX())).toRotationMatrix();
  EXPECT_NEAR(
    small_gicp_relocalization::rotationCorrection(
      result_transform, expected_transform),
    0.0, 1e-12);
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

TEST(PoseUtils, MapOdomCandidateIsIndependentOfRobotMotionForContinuityChecks) {
  const Eigen::Isometry3d common_map_odom =
    Eigen::Translation3d(1.1, -0.4, 0.2) *
    Eigen::AngleAxisd(0.16, Eigen::Vector3d::UnitZ());
  const Eigen::Isometry3d first_odom_body =
    Eigen::Translation3d(0.5, 0.2, 0.0) *
    Eigen::AngleAxisd(-0.3, Eigen::Vector3d::UnitZ());
  const Eigen::Isometry3d second_odom_body =
    Eigen::Translation3d(4.2, -1.7, 0.0) *
    Eigen::AngleAxisd(1.1, Eigen::Vector3d::UnitZ());

  const auto first_candidate = small_gicp_relocalization::mapOdomFromMapBody(
    common_map_odom * first_odom_body, first_odom_body);
  const auto second_candidate = small_gicp_relocalization::mapOdomFromMapBody(
    common_map_odom * second_odom_body, second_odom_body);

  EXPECT_TRUE(first_candidate.isApprox(second_candidate, 1e-12));
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
