cmake_minimum_required(VERSION 3.8)
project(radloc_slam)

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
find_package(Eigen3 REQUIRED)
find_package(GTSAM REQUIRED)

# The descriptor comes from the repository's C++ core (cpp/), the estimation from the ROS-free sources shared
# with the ROS 1 port (applications/slam/common).
set(RADLOC_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../../.." CACHE PATH "RadLoc repository root")
if(NOT EXISTS "${RADLOC_ROOT}/cpp/CMakeLists.txt")
  message(FATAL_ERROR "radloc core not found under ${RADLOC_ROOT}/cpp")
endif()
set(RADLOC_BUILD_TOOLS OFF CACHE BOOL "" FORCE)
add_subdirectory("${RADLOC_ROOT}/cpp" "${CMAKE_CURRENT_BINARY_DIR}/radloc_core" EXCLUDE_FROM_ALL)
set(COMMON "${RADLOC_ROOT}/applications/slam/common")

add_executable(pose_graph_node
  src/pose_graph_node.cpp
  ${COMMON}/src/loop_registration.cpp
  ${COMMON}/src/graph_io.cpp)
target_include_directories(pose_graph_node PRIVATE
  ${COMMON}/include ${PCL_INCLUDE_DIRS} ${GTSAM_INCLUDE_DIR})
target_link_libraries(pose_graph_node radloc::radloc gtsam ${PCL_LIBRARIES} Eigen3::Eigen)
ament_target_dependencies(pose_graph_node
  rclcpp nav_msgs sensor_msgs geometry_msgs tf2 tf2_geometry_msgs pcl_conversions
  radloc_interfaces)

add_executable(reoptimize ${COMMON}/src/reoptimize.cpp ${COMMON}/src/graph_io.cpp)
target_include_directories(reoptimize PRIVATE ${COMMON}/include ${GTSAM_INCLUDE_DIR})
target_link_libraries(reoptimize gtsam Eigen3::Eigen)

install(TARGETS pose_graph_node reoptimize DESTINATION lib/${PROJECT_NAME})
foreach(dir launch config)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${dir}")
    install(DIRECTORY ${dir} DESTINATION share/${PROJECT_NAME})
  endif()
endforeach()

ament_package()
