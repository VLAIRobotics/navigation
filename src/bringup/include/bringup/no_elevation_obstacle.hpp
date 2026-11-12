#ifndef BRINGUP__NO_ELEVATION_OBSTACLE_HPP_
#define BRINGUP__NO_ELEVATION_OBSTACLE_HPP_

#include <cmath>

namespace bringup
{
inline bool isNoElevationBlocking(
  double point_z, double sensor_height, double minimum_height, double maximum_height)
{
  const double height_above_ground = point_z + sensor_height;
  return std::isfinite(height_above_ground) && height_above_ground > minimum_height &&
         height_above_ground < maximum_height;
}
}  // namespace bringup

#endif  // BRINGUP__NO_ELEVATION_OBSTACLE_HPP_
