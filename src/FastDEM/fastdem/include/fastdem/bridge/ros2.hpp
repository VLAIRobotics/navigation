// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024 Ikhyeon Cho <tre0430@korea.ac.kr>

#ifndef FASTDEM_BRIDGE_ROS2_HPP
#define FASTDEM_BRIDGE_ROS2_HPP

#include <algorithm>
#include <cstdint>
#include <string>
#include <vector>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <rclcpp/time.hpp>

#include <nanogrid/bridge/ros2.hpp>

#include <fastdem/bridge/ros/impl.hpp>
#include <fastdem/bridge/ros/impl_visualization.hpp>

namespace fastdem::ros2 {

namespace detail {
inline rclcpp::Time toStamp(uint64_t ns) {
  return rclcpp::Time(static_cast<int64_t>(ns));
}
}  // namespace detail

/// Convert ElevationMap to PointCloud2 with all non-internal layers as fields.
inline sensor_msgs::msg::PointCloud2 toPointCloud2(
    const ElevationMap& map,
    const char* elevation_layer = layer::elevation) {
  return fastdem::detail::toPointCloud2Impl<sensor_msgs::msg::PointCloud2,
                                   sensor_msgs::msg::PointField>(
      map, detail::toStamp(map.getTimestamp()), elevation_layer);
}

/// Convert a submap region to PointCloud2 (zero-copy from parent map).
inline sensor_msgs::msg::PointCloud2 toPointCloud2(
    const ElevationMap& map, const nanogrid::Position& center,
    const nanogrid::Length& length,
    const char* elevation_layer = layer::elevation) {
  auto sub = map.subRegion(center, length);
  if (!sub) return {};
  return fastdem::detail::toPointCloud2Impl<sensor_msgs::msg::PointCloud2,
                                   sensor_msgs::msg::PointField>(
      map, detail::toStamp(map.getTimestamp()), elevation_layer,
      sub->startIndex, sub->size);
}

/// Convert ElevationMap to GridMap; optionally include internal surface normals.
inline grid_map_msgs::msg::GridMap toGridMap(
    const ElevationMap& map,
    const char* elevation_layer = layer::elevation,
    bool include_surface_normals = false,
    const std::vector<std::string>* selected_layers = nullptr) {
  auto layers = selected_layers ? std::vector<std::string>{} :
                                  fastdem::detail::visibleLayers(map);
  if (selected_layers) {
    layers.reserve(selected_layers->size() + 3);
    for (const auto& name : *selected_layers) {
      if (map.exists(name)) layers.push_back(name);
    }
  }
  if (include_surface_normals) {
    for (const char* normal_layer : {
        layer::normal_x, layer::normal_y, layer::normal_z})
    {
      if (map.exists(normal_layer) &&
          std::find(layers.begin(), layers.end(), normal_layer) == layers.end())
      {
        layers.emplace_back(normal_layer);
      }
    }
  }
  auto msg = nanogrid::ros2::toMsg(map, layers);
  fastdem::detail::normalizeGridMapLayout(msg, map);
  msg.basic_layers = {elevation_layer};
  return msg;
}

/// Create map boundary marker for RViz visualization.
inline visualization_msgs::msg::Marker toMapBoundary(const ElevationMap& map) {
  return nanogrid::ros2::toBoundaryMarker(map);
}

/// Convert ElevationMap normal vectors to MarkerArray for RViz visualization.
inline visualization_msgs::msg::MarkerArray toNormalMarkers(
    const ElevationMap& map, float arrow_length = 0.15f, int stride = 1) {
  return fastdem::detail::toNormalMarkersImpl<
      visualization_msgs::msg::MarkerArray, visualization_msgs::msg::Marker,
      geometry_msgs::msg::Point, std_msgs::msg::ColorRGBA>(
      map, detail::toStamp(map.getTimestamp()), arrow_length, stride);
}

}  // namespace fastdem::ros2

#endif  // FASTDEM_BRIDGE_ROS2_HPP
