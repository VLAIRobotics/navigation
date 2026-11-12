cmake_minimum_required(VERSION 3.16)
project(fastdem_ros2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Default to Release build
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
  message(STATUS "Setting build type to '${CMAKE_BUILD_TYPE}' as none was specified.")
endif()

###############
## Dependencies
###############

# ROS2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(visualization_msgs REQUIRED)

# Pure CMake dependency (our core library)
find_package(fastdem REQUIRED)

###########
## Build ##
###########

# Main elevation mapping node
add_executable(fastdem_node
  src/fastdem_ros_node.cpp
)

target_include_directories(fastdem_node PRIVATE
  include
)

ament_target_dependencies(fastdem_node
  rclcpp
  tf2_ros
  tf2_eigen
  sensor_msgs
  std_srvs
  grid_map_msgs
  visualization_msgs
)

target_link_libraries(fastdem_node
  fastdem::fastdem
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(Threads REQUIRED)
  ament_add_gtest(
    test_postprocess_publication_guard
    test/test_postprocess_publication_guard.cpp
  )
  target_include_directories(test_postprocess_publication_guard PRIVATE include)
  target_link_libraries(test_postprocess_publication_guard Threads::Threads)

  find_package(launch_testing_ament_cmake QUIET)
  if(launch_testing_ament_cmake_FOUND)
    add_launch_test(test/test_fastdem_runtime.py TIMEOUT 60)
  else()
    message(STATUS
      "fastdem_ros2: launch_testing_ament_cmake not found; "
      "runtime launch test skipped")
  endif()
endif()

#############
## Install ##
#############

install(TARGETS fastdem_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
