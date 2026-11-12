#ifndef TEB_LOCAL_PLANNER__NUMERICAL_SAFETY_HPP_
#define TEB_LOCAL_PLANNER__NUMERICAL_SAFETY_HPP_

#include <cmath>

namespace teb_local_planner
{
namespace detail
{

constexpr double minimum_time_diff()
{
  return 1e-3;
}

inline bool valid_time_diff(const double value)
{
  return std::isfinite(value) && value > minimum_time_diff();
}

inline bool finite_velocity_command(const double vx, const double vy, const double omega)
{
  return std::isfinite(vx) && std::isfinite(vy) && std::isfinite(omega);
}

}  // namespace detail
}  // namespace teb_local_planner

#endif  // TEB_LOCAL_PLANNER__NUMERICAL_SAFETY_HPP_
