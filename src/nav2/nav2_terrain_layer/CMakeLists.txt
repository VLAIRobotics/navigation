cmake_minimum_required(VERSION 3.8)
project(nav2_terrain_layer)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(fastdem REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/terrain_layer.cpp
  src/static_terrain_layer.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC include)
ament_target_dependencies(${PROJECT_NAME}
  grid_map_msgs
  nav_msgs
  nav2_costmap_2d
  nav2_msgs
  nav2_util
  pluginlib
  rclcpp
  rclcpp_lifecycle
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)
target_link_libraries(${PROJECT_NAME} fastdem::fastdem)

pluginlib_export_plugin_description_file(nav2_costmap_2d nav2_terrain_layer_plugin.xml)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
install(FILES nav2_terrain_layer_plugin.xml DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(visualization_msgs REQUIRED)
  ament_add_gtest(test_terrain_cost test/test_terrain_cost.cpp)
  target_include_directories(test_terrain_cost PRIVATE include)

  ament_add_gtest(test_terrain_layer test/test_terrain_layer.cpp)
  target_include_directories(test_terrain_layer PRIVATE include)
  ament_target_dependencies(test_terrain_layer
    grid_map_msgs
    nav2_costmap_2d
    nav2_util
    rclcpp
  )
  target_link_libraries(test_terrain_layer ${PROJECT_NAME} fastdem::fastdem)

  ament_add_gtest(test_static_terrain_layer test/test_static_terrain_layer.cpp)
  target_include_directories(test_static_terrain_layer PRIVATE include)
  ament_target_dependencies(test_static_terrain_layer
    grid_map_msgs
    nav_msgs
    nav2_costmap_2d
    nav2_util
    rclcpp
    sensor_msgs
    std_msgs
    tf2_ros
    visualization_msgs
  )
  target_link_libraries(test_static_terrain_layer ${PROJECT_NAME} fastdem::fastdem)
endif()

ament_package()
