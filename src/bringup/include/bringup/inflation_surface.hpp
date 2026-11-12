#ifndef BRINGUP__INFLATION_SURFACE_HPP_
#define BRINGUP__INFLATION_SURFACE_HPP_

#include <array>
#include <cstdint>
#include <vector>

#include <map_msgs/msg/occupancy_grid_update.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

namespace bringup
{

struct InflationCell
{
  std::uint32_t x;
  std::uint32_t y;
  std::int8_t cost;
};

struct InflationColor
{
  float r;
  float g;
  float b;
  float a;
};

std::vector<InflationCell> inflationCells(
  const nav_msgs::msg::OccupancyGrid & grid, double radius);
InflationColor inflationColor(std::int8_t cost);
bool applyCostmapUpdate(
  nav_msgs::msg::OccupancyGrid & grid, const map_msgs::msg::OccupancyGridUpdate & update,
  bool * changed = nullptr);

}  // namespace bringup

#endif  // BRINGUP__INFLATION_SURFACE_HPP_
