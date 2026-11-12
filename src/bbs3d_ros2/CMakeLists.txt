cmake_minimum_required(VERSION 3.14)
project(bbs3d_ros2)

# Default to Release: 3D-BBS is performance-critical and PCL/Eigen rely on
# -DNDEBUG for assert removal. Override with -DCMAKE_BUILD_TYPE=Debug etc.
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem system)
find_package(yaml-cpp REQUIRED)
# cpu_bbs3d parallelizes its score evaluation with OpenMP.
find_package(OpenMP)

add_library(cpu_bbs3d SHARED
  3d_bbs/bbs3d/src/cpu_bbs3d/bbs3d.cpp
  3d_bbs/bbs3d/src/cpu_bbs3d/voxelmaps.cpp
  3d_bbs/bbs3d/src/cpu_bbs3d/voxelmaps_io.cpp
)
target_include_directories(cpu_bbs3d PUBLIC
  ${PROJECT_SOURCE_DIR}/3d_bbs/bbs3d/include
)
target_link_libraries(cpu_bbs3d PUBLIC
  Eigen3::Eigen
  Boost::filesystem
  Boost::system
)
if(OpenMP_CXX_FOUND)
  target_link_libraries(cpu_bbs3d PUBLIC OpenMP::OpenMP_CXX)
endif()
install(TARGETS cpu_bbs3d LIBRARY DESTINATION lib)

ament_auto_add_executable(bbs3d_ros2_node
  src/bbs3d_node_main.cpp
  src/bbs3d_node.cpp
  src/bbs3d_backend.cpp
  src/bbs3d_backend_cpu.cpp
)
target_include_directories(bbs3d_ros2_node PUBLIC
  ${PROJECT_SOURCE_DIR}/include
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(bbs3d_ros2_node
  cpu_bbs3d
  ${PCL_LIBRARIES}
  Boost::filesystem
  Boost::system
  yaml-cpp
)

ament_auto_package(
  INSTALL_TO_SHARE
  config
)
