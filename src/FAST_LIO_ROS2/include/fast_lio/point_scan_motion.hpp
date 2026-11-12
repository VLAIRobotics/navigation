#ifndef FAST_LIO__POINT_SCAN_MOTION_HPP_
#define FAST_LIO__POINT_SCAN_MOTION_HPP_

#include <array>
#include <cstddef>

namespace fast_lio
{

struct PointBodyMotionInScanEnd
{
  // Row-major rotation from the point-time IMU frame into the scan-end IMU frame.
  std::array<double, 9> relative_rotation{{
    1.0, 0.0, 0.0,
    0.0, 1.0, 0.0,
    0.0, 0.0, 1.0}};
  std::array<double, 3> relative_translation{{0.0, 0.0, 0.0}};
  double time_from_scan_end_seconds{0.0};
  bool valid{false};
};

inline std::array<double, 3> sensorOriginInScanEndBody(
  const PointBodyMotionInScanEnd & motion,
  const std::array<double, 3> & lidar_translation_in_imu)
{
  std::array<double, 3> origin = motion.relative_translation;
  for (std::size_t row = 0; row < 3; ++row) {
    for (std::size_t col = 0; col < 3; ++col) {
      origin[row] += motion.relative_rotation[row * 3 + col] *
        lidar_translation_in_imu[col];
    }
  }
  return origin;
}

}  // namespace fast_lio

#endif  // FAST_LIO__POINT_SCAN_MOTION_HPP_
