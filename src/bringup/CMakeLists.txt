cmake_minimum_required(VERSION 3.8)
project(bringup)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(fastdem REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(action_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rmw REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rviz_common REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
set(EIGEN_FUNCTORS_PLUGIN "grid_map_core/eigen_plugins/FunctorsPlugin.hpp")
set(EIGEN_DENSEBASE_PLUGIN "grid_map_core/eigen_plugins/DenseBasePlugin.hpp")
find_package(grid_map_core REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(map_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_theta_star_planner REQUIRED)
find_package(small_gicp_relocalization REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(pcl_conversions REQUIRED)

add_library(navigation_status_panel SHARED src/navigation_status_panel.cpp)
target_include_directories(navigation_status_panel PRIVATE include)
ament_target_dependencies(navigation_status_panel
  action_msgs pluginlib rcl_interfaces rclcpp rclcpp_action rviz_common sensor_msgs std_msgs
  std_srvs)
target_link_libraries(navigation_status_panel Qt5::Widgets)
pluginlib_export_plugin_description_file(rviz_common navigation_panel_plugin.xml)
install(TARGETS navigation_status_panel LIBRARY DESTINATION lib)

add_library(bringup_goal_cost_condition_bt_node SHARED src/goal_cost_condition_bt_node.cpp)
target_compile_definitions(bringup_goal_cost_condition_bt_node PRIVATE BT_PLUGIN_EXPORT)
ament_target_dependencies(bringup_goal_cost_condition_bt_node
  behaviortree_cpp_v3 geometry_msgs nav2_msgs rclcpp tf2 tf2_geometry_msgs tf2_ros)
install(TARGETS bringup_goal_cost_condition_bt_node LIBRARY DESTINATION lib)

add_library(bringup_path_collision_condition_bt_node SHARED
  src/path_collision_condition_bt_node.cpp)
target_include_directories(bringup_path_collision_condition_bt_node PRIVATE include)
target_compile_definitions(bringup_path_collision_condition_bt_node PRIVATE BT_PLUGIN_EXPORT)
ament_target_dependencies(bringup_path_collision_condition_bt_node
  behaviortree_cpp_v3 geometry_msgs nav2_costmap_2d nav2_msgs nav_msgs rclcpp tf2
  tf2_geometry_msgs tf2_ros)
install(TARGETS bringup_path_collision_condition_bt_node LIBRARY DESTINATION lib)

add_library(bringup_follow_path_retry_bt_node SHARED src/follow_path_retry_bt_node.cpp)
target_include_directories(bringup_follow_path_retry_bt_node PRIVATE include)
target_compile_definitions(bringup_follow_path_retry_bt_node PRIVATE BT_PLUGIN_EXPORT)
ament_target_dependencies(bringup_follow_path_retry_bt_node
  action_msgs behaviortree_cpp_v3 builtin_interfaces geometry_msgs nav2_theta_star_planner
  nav_msgs rclcpp std_msgs)
install(TARGETS bringup_follow_path_retry_bt_node LIBRARY DESTINATION lib)

add_library(bringup_global_plan_retry_bt_node SHARED src/global_plan_retry_bt_node.cpp)
target_compile_definitions(bringup_global_plan_retry_bt_node PRIVATE BT_PLUGIN_EXPORT)
ament_target_dependencies(bringup_global_plan_retry_bt_node
  action_msgs behaviortree_cpp_v3 builtin_interfaces geometry_msgs nav2_msgs nav2_theta_star_planner
  nav_msgs rclcpp std_msgs)
install(TARGETS bringup_global_plan_retry_bt_node LIBRARY DESTINATION lib)

add_library(bringup_temporal_obstacle_layer SHARED src/temporal_obstacle_layer.cpp)
target_include_directories(bringup_temporal_obstacle_layer PRIVATE include)
ament_target_dependencies(bringup_temporal_obstacle_layer
  geometry_msgs nav2_costmap_2d nav2_msgs pluginlib rclcpp rclcpp_lifecycle sensor_msgs)
pluginlib_export_plugin_description_file(
  nav2_costmap_2d temporal_obstacle_layer_plugin.xml)
install(TARGETS bringup_temporal_obstacle_layer LIBRARY DESTINATION lib)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY behavior_trees
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(PROGRAMS scripts/gravity_leveler.py scripts/pcd_to_occupancy_grid.py
  scripts/publish_initial_pose.py
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(pcd_to_terrain_map src/pcd_to_terrain_map.cpp)
target_include_directories(pcd_to_terrain_map PRIVATE include ${PCL_INCLUDE_DIRS})
target_link_libraries(pcd_to_terrain_map fastdem::fastdem ${PCL_LIBRARIES})
install(TARGETS pcd_to_terrain_map DESTINATION lib/${PROJECT_NAME})

add_executable(static_terrain_map_publisher
  src/static_terrain_map_publisher.cpp
  src/static_terrain_map_utils.cpp
  src/inflation_surface.cpp)
target_include_directories(static_terrain_map_publisher PRIVATE include ${PCL_INCLUDE_DIRS})
ament_target_dependencies(static_terrain_map_publisher
  geometry_msgs grid_map_msgs map_msgs nav_msgs rclcpp sensor_msgs std_msgs tf2 tf2_geometry_msgs
  tf2_ros visualization_msgs)
target_link_libraries(static_terrain_map_publisher fastdem::fastdem ${PCL_LIBRARIES})
install(TARGETS static_terrain_map_publisher DESTINATION lib/${PROJECT_NAME})

add_executable(navigation_cloud_display_node src/navigation_cloud_display_node.cpp)
target_include_directories(navigation_cloud_display_node PRIVATE include)
target_compile_definitions(navigation_cloud_display_node PRIVATE
  EIGEN_FUNCTORS_PLUGIN="grid_map_core/eigen_plugins/FunctorsPlugin.hpp"
  EIGEN_DENSEBASE_PLUGIN="grid_map_core/eigen_plugins/DenseBasePlugin.hpp")
ament_target_dependencies(navigation_cloud_display_node
  geometry_msgs grid_map_core grid_map_msgs grid_map_ros nav_msgs rclcpp sensor_msgs tf2
  std_msgs tf2_geometry_msgs tf2_ros)
install(TARGETS navigation_cloud_display_node DESTINATION lib/${PROJECT_NAME})

add_executable(nonground_height_filter src/nonground_height_filter.cpp)
target_include_directories(nonground_height_filter PRIVATE include)
ament_target_dependencies(nonground_height_filter
  grid_map_msgs message_filters rclcpp sensor_msgs std_srvs tf2 tf2_geometry_msgs tf2_ros
  visualization_msgs)
target_link_libraries(nonground_height_filter fastdem::fastdem)
install(TARGETS nonground_height_filter DESTINATION lib/${PROJECT_NAME})

add_executable(path_gated_obstacle_node src/path_gated_obstacle_node.cpp)
ament_target_dependencies(path_gated_obstacle_node
  geometry_msgs nav_msgs rclcpp sensor_msgs std_msgs tf2 tf2_geometry_msgs tf2_ros)
install(TARGETS path_gated_obstacle_node DESTINATION lib/${PROJECT_NAME})

add_executable(global_map_fusion_node src/global_map_fusion_node.cpp)
target_include_directories(global_map_fusion_node PRIVATE include)
ament_target_dependencies(global_map_fusion_node
  builtin_interfaces geometry_msgs grid_map_msgs map_msgs message_filters nav2_costmap_2d nav_msgs
  rclcpp sensor_msgs std_msgs std_srvs rmw tf2 tf2_geometry_msgs tf2_ros visualization_msgs)
target_link_libraries(global_map_fusion_node fastdem::fastdem)
install(TARGETS global_map_fusion_node DESTINATION lib/${PROJECT_NAME})

add_executable(map_snapshot_saver_node src/map_snapshot_saver_node.cpp)
target_include_directories(map_snapshot_saver_node PRIVATE include)
ament_target_dependencies(map_snapshot_saver_node
  builtin_interfaces geometry_msgs grid_map_msgs nav_msgs rclcpp std_srvs tf2 tf2_geometry_msgs
  visualization_msgs)
target_link_libraries(map_snapshot_saver_node fastdem::fastdem)
install(TARGETS map_snapshot_saver_node DESTINATION lib/${PROJECT_NAME})

add_executable(frontier_explorer_node src/frontier_explorer_node.cpp)
target_include_directories(frontier_explorer_node PRIVATE include)
ament_target_dependencies(frontier_explorer_node
  action_msgs geometry_msgs grid_map_msgs map_msgs nav2_costmap_2d nav2_msgs nav_msgs rclcpp
  rclcpp_action sensor_msgs std_msgs std_srvs tf2 tf2_geometry_msgs tf2_ros)
install(TARGETS frontier_explorer_node DESTINATION lib/${PROJECT_NAME})

add_executable(chassis_driver src/chassis_driver.cpp)
target_include_directories(chassis_driver PRIVATE include)
ament_target_dependencies(chassis_driver geometry_msgs grid_map_msgs nav2_msgs nav_msgs rclcpp
  sensor_msgs small_gicp_relocalization std_msgs tf2 tf2_geometry_msgs tf2_ros)
install(TARGETS chassis_driver
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(navigation_gate_node src/navigation_gate_node.cpp)
target_include_directories(navigation_gate_node PRIVATE include)
ament_target_dependencies(navigation_gate_node
  action_msgs grid_map_msgs lifecycle_msgs nav2_msgs nav_msgs rclcpp sensor_msgs
  small_gicp_relocalization std_msgs)
install(TARGETS navigation_gate_node DESTINATION lib/${PROJECT_NAME})

add_executable(navigation_goal_proxy_node src/navigation_goal_proxy_node.cpp)
target_include_directories(navigation_goal_proxy_node PRIVATE include)
ament_target_dependencies(navigation_goal_proxy_node
  geometry_msgs map_msgs nav2_costmap_2d nav2_msgs nav_msgs rclcpp rclcpp_action sensor_msgs
  std_msgs tf2 tf2_geometry_msgs tf2_ros)
install(TARGETS navigation_goal_proxy_node DESTINATION lib/${PROJECT_NAME})

add_executable(map_odom_handoff_node src/map_odom_handoff_node.cpp)
target_include_directories(map_odom_handoff_node PRIVATE include)
ament_target_dependencies(map_odom_handoff_node
  geometry_msgs nav_msgs rclcpp small_gicp_relocalization std_msgs tf2 tf2_geometry_msgs tf2_ros)
install(TARGETS map_odom_handoff_node DESTINATION lib/${PROJECT_NAME})

add_executable(nav2_path_display_node src/nav2_path_display_node.cpp)
target_include_directories(nav2_path_display_node PRIVATE include)
ament_target_dependencies(nav2_path_display_node
  action_msgs builtin_interfaces geometry_msgs nav2_theta_star_planner nav_msgs rclcpp
  rclcpp_action std_msgs)
install(TARGETS nav2_path_display_node
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(static_map_filter_node src/static_map_filter_node.cpp)
target_include_directories(static_map_filter_node PRIVATE include ${PCL_INCLUDE_DIRS})
target_link_libraries(static_map_filter_node ${PCL_LIBRARIES})
ament_target_dependencies(static_map_filter_node
  rclcpp nav_msgs sensor_msgs std_msgs std_srvs pcl_conversions)
install(TARGETS static_map_filter_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_library(NAV2_PIPELINE_SEQUENCE_BT_NODE_LIBRARY nav2_pipeline_sequence_bt_node REQUIRED)
  ament_add_pytest_test(test_pcd_to_occupancy_grid test/test_pcd_to_occupancy_grid.py)
  ament_add_pytest_test(test_navigation_config test/test_navigation_config.py)
  ament_add_pytest_test(test_navigation_rewrites test/test_navigation_rewrites.py)
  ament_add_gtest(test_chassis_driver test/test_chassis_driver.cpp)
  target_include_directories(test_chassis_driver PRIVATE include)
  ament_target_dependencies(test_chassis_driver geometry_msgs grid_map_msgs std_msgs)

  ament_add_gtest(test_path_display_state test/test_path_display_state.cpp)
  target_include_directories(test_path_display_state PRIVATE include)
  ament_target_dependencies(test_path_display_state
    action_msgs builtin_interfaces rclcpp rclcpp_action)

  ament_add_gtest(test_static_map_filter test/test_static_map_filter.cpp)
  target_include_directories(test_static_map_filter PRIVATE include ${PCL_INCLUDE_DIRS})
  target_link_libraries(test_static_map_filter ${PCL_LIBRARIES})

  ament_add_gtest(test_temporal_obstacle_grid test/test_temporal_obstacle_grid.cpp)
  target_include_directories(test_temporal_obstacle_grid PRIVATE include)

  ament_add_gtest(test_terrain_map_utils test/test_terrain_map_utils.cpp)
  target_include_directories(test_terrain_map_utils PRIVATE include)

  ament_add_gtest(test_ground_provenance test/test_ground_provenance.cpp)
  target_include_directories(test_ground_provenance PRIVATE include)
  ament_target_dependencies(test_ground_provenance
    builtin_interfaces nav_msgs tf2 tf2_geometry_msgs)

  ament_add_gtest(test_map_fusion_evidence test/test_map_fusion_evidence.cpp)
  target_include_directories(test_map_fusion_evidence PRIVATE include)
  ament_target_dependencies(test_map_fusion_evidence
    nav2_costmap_2d nav_msgs tf2 tf2_geometry_msgs)

  ament_add_gtest(test_inflation_surface
    test/test_inflation_surface.cpp
    src/inflation_surface.cpp)
  target_include_directories(test_inflation_surface PRIVATE include)
  ament_target_dependencies(test_inflation_surface map_msgs nav_msgs)

  ament_add_gtest(test_static_terrain_map_publisher
    test/test_static_terrain_map_publisher.cpp
    src/static_terrain_map_utils.cpp)
  target_include_directories(test_static_terrain_map_publisher PRIVATE include)
  target_compile_definitions(test_static_terrain_map_publisher PRIVATE
    EIGEN_FUNCTORS_PLUGIN="grid_map_core/eigen_plugins/FunctorsPlugin.hpp"
    EIGEN_DENSEBASE_PLUGIN="grid_map_core/eigen_plugins/DenseBasePlugin.hpp")
  ament_target_dependencies(test_static_terrain_map_publisher
    grid_map_msgs grid_map_ros map_msgs nav_msgs rclcpp sensor_msgs tf2 tf2_geometry_msgs
    std_msgs visualization_msgs)
  target_link_libraries(test_static_terrain_map_publisher fastdem::fastdem)

  ament_add_gtest(test_navigation_gate_state test/test_navigation_gate_state.cpp)
  target_include_directories(test_navigation_gate_state PRIVATE include)
  ament_target_dependencies(test_navigation_gate_state lifecycle_msgs)

  ament_add_gtest(test_goal_rejection_cache test/test_goal_rejection_cache.cpp)
  target_include_directories(test_goal_rejection_cache PRIVATE include)

  ament_add_gtest(test_follow_path_retry_budget test/test_follow_path_retry_budget.cpp)
  target_include_directories(test_follow_path_retry_budget PRIVATE include)

  ament_add_gtest(test_global_plan_retry_bt_node test/test_global_plan_retry_bt_node.cpp)
  ament_target_dependencies(test_global_plan_retry_bt_node
    action_msgs behaviortree_cpp_v3 builtin_interfaces geometry_msgs nav_msgs rclcpp std_msgs)
  add_dependencies(test_global_plan_retry_bt_node
    bringup_follow_path_retry_bt_node bringup_global_plan_retry_bt_node)
  target_compile_definitions(test_global_plan_retry_bt_node PRIVATE
    FOLLOW_PATH_RETRY_BT_PLUGIN="$<TARGET_FILE:bringup_follow_path_retry_bt_node>"
    GLOBAL_PLAN_RETRY_BT_PLUGIN="$<TARGET_FILE:bringup_global_plan_retry_bt_node>"
    NAV2_PIPELINE_SEQUENCE_BT_PLUGIN="${NAV2_PIPELINE_SEQUENCE_BT_NODE_LIBRARY}"
    BRINGUP_MAP_NAV_TREE_PATH="${CMAKE_CURRENT_SOURCE_DIR}/behavior_trees/navigate_to_pose_map_guarded_recovery.xml")

  ament_add_gtest(test_navigation_goal_proxy test/test_navigation_goal_proxy.cpp)
  target_include_directories(test_navigation_goal_proxy PRIVATE include)
  target_compile_definitions(test_navigation_goal_proxy PRIVATE
    BRINGUP_NAVIGATION_GOAL_PROXY_NO_MAIN)
  ament_target_dependencies(test_navigation_goal_proxy
    action_msgs builtin_interfaces geometry_msgs map_msgs nav2_costmap_2d nav2_msgs nav_msgs
    rclcpp rclcpp_action sensor_msgs std_msgs tf2 tf2_geometry_msgs tf2_ros)

  ament_add_gtest(test_path_collision_escape test/test_path_collision_escape.cpp)
  target_include_directories(test_path_collision_escape PRIVATE include)
  ament_target_dependencies(test_path_collision_escape nav2_costmap_2d)

  ament_add_gtest(test_online_goal_route test/test_online_goal_route.cpp)
  target_include_directories(test_online_goal_route PRIVATE include)
  ament_target_dependencies(test_online_goal_route
    nav2_costmap_2d nav_msgs tf2 tf2_geometry_msgs)

  ament_add_gtest(test_online_map_state test/test_online_map_state.cpp)
  target_include_directories(test_online_map_state PRIVATE include)
  ament_target_dependencies(test_online_map_state builtin_interfaces map_msgs nav_msgs std_msgs)
endif()

ament_package()
