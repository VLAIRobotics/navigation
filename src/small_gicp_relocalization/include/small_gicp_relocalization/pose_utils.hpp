#pragma once

#include <cmath>
#include <cstddef>

#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose.hpp>

namespace small_gicp_relocalization {

inline Eigen::Isometry3d planarPoseToMapOdom(const geometry_msgs::msg::Pose &pose) {
  const double yaw = std::atan2(
    2.0 * (pose.orientation.w * pose.orientation.z + pose.orientation.x * pose.orientation.y),
    1.0 - 2.0 * (pose.orientation.y * pose.orientation.y + pose.orientation.z * pose.orientation.z));

  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.translation() = Eigen::Vector3d(pose.position.x, pose.position.y, 0.0);
  result.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  return result;
}

inline Eigen::Isometry3d predictMapBody(
  const Eigen::Isometry3d &map_odom, const Eigen::Isometry3d &odom_body) {
  return map_odom * odom_body;
}

inline Eigen::Isometry3d planarizeTransform(const Eigen::Isometry3d &transform) {
  const double yaw = std::atan2(transform.rotation()(1, 0), transform.rotation()(0, 0));
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.translation() = Eigen::Vector3d(transform.translation().x(), transform.translation().y(), 0.0);
  result.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  return result;
}

inline Eigen::Isometry3d initialMapOdomFromClickedPose(
  const geometry_msgs::msg::Pose &clicked_pose, const Eigen::Isometry3d &odom_body) {
  return planarPoseToMapOdom(clicked_pose) * planarizeTransform(odom_body).inverse();
}

inline Eigen::Isometry3d mapOdomFromMapBody(
  const Eigen::Isometry3d &map_body, const Eigen::Isometry3d &odom_body) {
  return map_body * odom_body.inverse();
}

inline double translationCorrection(
  const Eigen::Isometry3d &candidate, const Eigen::Isometry3d &prediction) {
  return (candidate.translation() - prediction.translation()).norm();
}

inline double rotationCorrection(
  const Eigen::Isometry3d &candidate, const Eigen::Isometry3d &prediction) {
  const Eigen::AngleAxisd delta(candidate.rotation() * prediction.rotation().transpose());
  return std::abs(delta.angle());
}

inline bool registrationPassesGate(
  bool converged, std::size_t num_inliers, std::size_t min_inliers,
  double correction_translation, double correction_rotation,
  double max_translation_jump, double max_rotation_jump) {
  return converged && num_inliers >= min_inliers &&
    correction_translation <= max_translation_jump && correction_rotation <= max_rotation_jump;
}

}  // namespace small_gicp_relocalization
