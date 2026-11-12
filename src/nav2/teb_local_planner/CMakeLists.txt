cmake_minimum_required(VERSION 3.5)
project(teb_local_planner)

# Set to Release in order to speed up the program significantly
set(CMAKE_BUILD_TYPE Release) #None, Debug, Release, RelWithDebInfo, MinSizeRel

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

#if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#  add_compile_options(-Wall -Wextra -Wpedantic)
#endif()

## Find catkin macros and libraries
find_package(ament_cmake REQUIRED)

find_package(dwb_critics REQUIRED)
find_package(nav_2d_utils REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
#find_package(interactive_markers REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(teb_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

message(STATUS "System: ${CMAKE_SYSTEM}")
## System dependencies are found with CMake's conventions
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake_modules)
message(STATUS "${CMAKE_MODULE_PATH}")
find_package(SUITESPARSE REQUIRED)
find_package(G2O REQUIRED)
find_package(Boost REQUIRED)
find_package(fmt REQUIRED)

find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
set(Eigen_INCLUDE_DIRS ${Eigen3_INCLUDE_DIRS})

set(EXTERNAL_INCLUDE_DIRS ${Eigen_INCLUDE_DIRS} ${SUITESPARSE_INCLUDE_DIRS} ${G2O_INCLUDE_DIR})
set(EXTERNAL_LIBS ${Boost_LIBRARIES} ${SUITESPARSE_LIBRARIES} ${G2O_LIBRARIES})

###########
## Build ##
###########

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${EXTERNAL_INCLUDE_DIRS}
)

## Build the teb_local_planner library

add_library(teb_local_planner SHARED
   src/timed_elastic_band.cpp
   src/optimal_planner.cpp
   src/obstacles.cpp
   src/visualization.cpp
   src/recovery_behaviors.cpp
   src/teb_config.cpp
   src/homotopy_class_planner.cpp
   src/teb_local_planner_ros.cpp
   src/graph_search.cpp
)

set(ament_dependencies
    
    dwb_critics
    nav_2d_utils
    nav2_core
    nav2_costmap_2d
    nav2_util
    geometry_msgs
    nav_msgs
    rclcpp
    rclcpp_action
    rclcpp_lifecycle
    std_msgs
    pluginlib
    teb_msgs
    tf2
    tf2_eigen
    tf2_geometry_msgs
    tf2_ros
    visualization_msgs
    builtin_interfaces
)
ament_target_dependencies(teb_local_planner
   ${ament_dependencies}
)
target_link_libraries(teb_local_planner
   Eigen3::Eigen
   ${EXTERNAL_LIBS}
   fmt::fmt
)

target_compile_definitions(teb_local_planner PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")


#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS teb_local_planner
   DESTINATION lib
)

## Mark cpp header files for installation
install(DIRECTORY include/
   DESTINATION include/
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES
  teb_local_planner_plugin.xml
  DESTINATION share
  )

install(PROGRAMS scripts/cmd_vel_to_ackermann_drive.py DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY params DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(teb_local_planner)
ament_export_dependencies(${ament_dependencies} eigen3_cmake_module Eigen3)
pluginlib_export_plugin_description_file(nav2_core teb_local_planner_plugin.xml)

#############
## Testing ##
#############

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(costmap_obstacle_tiling_test test/costmap_obstacle_tiling_test.cpp)
  target_include_directories(costmap_obstacle_tiling_test PRIVATE include)
  ament_target_dependencies(costmap_obstacle_tiling_test builtin_interfaces rclcpp)
  target_link_libraries(costmap_obstacle_tiling_test Eigen3::Eigen)

  ament_add_gtest(failure_detector_test test/failure_detector_test.cpp)
  target_include_directories(failure_detector_test PRIVATE include src)
  target_link_libraries(failure_detector_test teb_local_planner)

  ament_add_gtest(speed_limit_test test/speed_limit_test.cpp)
  target_include_directories(speed_limit_test PRIVATE include src)
  target_link_libraries(speed_limit_test teb_local_planner)
endif()

ament_package()
