cmake_minimum_required(VERSION 3.8)
project(small_gicp_relocalization)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(small_gicp REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RelocalizationStatus.msg"
  "msg/MapOdomTarget.msg"
  "msg/NavigationPauseRequest.msg"
  "msg/NavigationPauseStatus.msg"
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs
)

find_package(OpenMP)
find_package(Threads REQUIRED)

add_executable(small_gicp_relocalization_node src/small_gicp_relocalization_node.cpp)
target_include_directories(small_gicp_relocalization_node PRIVATE include ${PCL_INCLUDE_DIRS})
target_link_libraries(small_gicp_relocalization_node
  small_gicp::small_gicp
  ${PCL_LIBRARIES}
  Eigen3::Eigen
  Threads::Threads
)
if(OpenMP_CXX_FOUND)
  target_link_libraries(small_gicp_relocalization_node OpenMP::OpenMP_CXX)
endif()
ament_target_dependencies(small_gicp_relocalization_node
  builtin_interfaces
  geometry_msgs
  nav_msgs
  pcl_conversions
  rclcpp
  sensor_msgs
  std_msgs
  std_srvs
  tf2
  tf2_eigen
  tf2_msgs
  tf2_ros
)
rosidl_target_interfaces(small_gicp_relocalization_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(gicp_offline_benchmark src/offline_gicp_benchmark.cpp)
target_include_directories(gicp_offline_benchmark PRIVATE include ${PCL_INCLUDE_DIRS})
target_link_libraries(gicp_offline_benchmark
  small_gicp::small_gicp
  ${PCL_LIBRARIES}
  Eigen3::Eigen
)
if(OpenMP_CXX_FOUND)
  target_link_libraries(gicp_offline_benchmark OpenMP::OpenMP_CXX)
endif()
ament_target_dependencies(gicp_offline_benchmark
  pcl_conversions
)

install(TARGETS small_gicp_relocalization_node
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS gicp_offline_benchmark
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_pose_utils test/test_pose_utils.cpp)
  target_include_directories(test_pose_utils PRIVATE include)
  ament_target_dependencies(test_pose_utils geometry_msgs)
  target_link_libraries(test_pose_utils Eigen3::Eigen)

  ament_add_gtest(test_relocalization_hysteresis test/test_relocalization_hysteresis.cpp)
  target_include_directories(test_relocalization_hysteresis PRIVATE include)

  ament_add_gtest(test_fastlio_health test/test_fastlio_health.cpp)
  target_include_directories(test_fastlio_health PRIVATE include)

  ament_add_gtest(test_manual_seed_deadline test/test_manual_seed_deadline.cpp)
  target_include_directories(test_manual_seed_deadline PRIVATE include)

  ament_add_gtest(test_relocalization_generation test/test_relocalization_generation.cpp)
  target_include_directories(test_relocalization_generation PRIVATE include)

  ament_add_gtest(test_snapshot_storage test/test_snapshot_storage.cpp)
  target_include_directories(test_snapshot_storage PRIVATE include)

  ament_add_gtest(test_registration_termination test/test_registration_termination.cpp)
  target_include_directories(test_registration_termination PRIVATE include)
endif()

ament_package()
