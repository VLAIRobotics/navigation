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

// RViz's /initialpose describes the robot base frame in map.  Keep the full
// timestamped odometry transform here so body/base extrinsics and height are
// accounted for instead of treating the click as the LiDAR body origin.
inline Eigen::Isometry3d mapOdomFromClickedBasePose(
  const geometry_msgs::msg::Pose &clicked_map_base,
  const Eigen::Isometry3d &odom_base)
{
  return Eigen::Isometry3d(
    Eigen::Translation3d(clicked_map_base.position.x, clicked_map_base.position.y,
      clicked_map_base.position.z) *
    Eigen::Quaterniond(clicked_map_base.orientation.w, clicked_map_base.orientation.x,
      clicked_map_base.orientation.y, clicked_map_base.orientation.z).normalized()) *
    odom_base.inverse();
}

// Apply the clicked XY/yaw while carrying the trusted base height and tilt.
inline geometry_msgs::msg::Pose clickedPlanarPoseWithReferenceHeightAndTilt(
  const geometry_msgs::msg::Pose & clicked_map_base,
  const Eigen::Isometry3d & reference_map_base)
{
  geometry_msgs::msg::Pose result = clicked_map_base;
  const double clicked_yaw = std::atan2(
    2.0 * (clicked_map_base.orientation.w * clicked_map_base.orientation.z +
    clicked_map_base.orientation.x * clicked_map_base.orientation.y),
    1.0 - 2.0 * (clicked_map_base.orientation.y * clicked_map_base.orientation.y +
    clicked_map_base.orientation.z * clicked_map_base.orientation.z));
  const double reference_yaw = std::atan2(
    reference_map_base.rotation()(1, 0), reference_map_base.rotation()(0, 0));
  const Eigen::Matrix3d rotation = Eigen::AngleAxisd(
    clicked_yaw - reference_yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix() *
    reference_map_base.rotation();
  const Eigen::Quaterniond orientation(rotation);
  result.position.z = reference_map_base.translation().z();
  result.orientation.x = orientation.x();
  result.orientation.y = orientation.y();
  result.orientation.z = orientation.z();
  result.orientation.w = orientation.w();
  return result;
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
