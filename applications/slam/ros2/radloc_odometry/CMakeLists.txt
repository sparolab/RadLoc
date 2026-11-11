cmake_minimum_required(VERSION 3.8)
project(radloc_odometry)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(radloc_interfaces REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem)
find_package(OpenMP)

# The descriptor comes from the repository's C++ core (cpp/), the estimation from the ROS-free sources shared
# with the ROS 1 port (applications/slam/common).
set(RADLOC_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../../.." CACHE PATH "RadLoc repository root")
if(NOT EXISTS "${RADLOC_ROOT}/cpp/CMakeLists.txt")
  message(FATAL_ERROR "radloc core not found under ${RADLOC_ROOT}/cpp")
endif()
set(RADLOC_BUILD_TOOLS OFF CACHE BOOL "" FORCE)
add_subdirectory("${RADLOC_ROOT}/cpp" "${CMAKE_CURRENT_BINARY_DIR}/radloc_core" EXCLUDE_FROM_ALL)
set(COMMON "${RADLOC_ROOT}/applications/slam/common")

# Estimation, vendored from yeti_radar_odometry. No ROS API inside.
add_library(radloc_odometry_core STATIC
  ${COMMON}/src/radar_utils.cpp
  ${COMMON}/src/features.cpp
  ${COMMON}/src/association.cpp
  ${COMMON}/src/key_operation.cpp)
target_include_directories(radloc_odometry_core PUBLIC ${COMMON}/include ${PCL_INCLUDE_DIRS})
target_link_libraries(radloc_odometry_core PUBLIC
  ${OpenCV_LIBS} ${PCL_LIBRARIES} Eigen3::Eigen Boost::system Boost::filesystem)
if(OpenMP_CXX_FOUND)
  target_link_libraries(radloc_odometry_core PUBLIC OpenMP::OpenMP_CXX)
endif()

add_executable(odometry_node src/odometry_node.cpp)
target_link_libraries(odometry_node radloc_odometry_core radloc::radloc)
ament_target_dependencies(odometry_node
  rclcpp nav_msgs sensor_msgs geometry_msgs tf2 tf2_geometry_msgs pcl_conversions
  radloc_interfaces)

install(TARGETS odometry_node DESTINATION lib/${PROJECT_NAME})
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
endif()

ament_package()
