cmake_minimum_required(VERSION 3.16)
project(radloc_ros)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs nav_msgs sensor_msgs geometry_msgs tf2
             tf2_geometry_msgs pcl_conversions message_generation)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem)
find_package(GTSAM REQUIRED)
find_package(OpenMP)

add_message_files(FILES RadLocDescriptor.msg)
generate_messages(DEPENDENCIES std_msgs)
catkin_package(CATKIN_DEPENDS roscpp std_msgs nav_msgs sensor_msgs geometry_msgs message_runtime)

# The RadLoc core (cpp/) and the ROS-free estimation shared with the ROS 2 packages.
set(RADLOC_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../../.." CACHE PATH "RadLoc repository root")
if(NOT EXISTS "${RADLOC_ROOT}/cpp/CMakeLists.txt")
  message(FATAL_ERROR "radloc core not found under ${RADLOC_ROOT}/cpp")
endif()
set(RADLOC_BUILD_TOOLS OFF CACHE BOOL "" FORCE)
add_subdirectory("${RADLOC_ROOT}/cpp" "${CMAKE_CURRENT_BINARY_DIR}/radloc_core" EXCLUDE_FROM_ALL)
set(COMMON "${RADLOC_ROOT}/applications/slam/common")

include_directories(${catkin_INCLUDE_DIRS} ${COMMON}/include ${PCL_INCLUDE_DIRS} ${GTSAM_INCLUDE_DIR})

add_library(radloc_odometry_core STATIC
  ${COMMON}/src/radar_utils.cpp ${COMMON}/src/features.cpp
  ${COMMON}/src/association.cpp ${COMMON}/src/key_operation.cpp)
target_link_libraries(radloc_odometry_core PUBLIC ${OpenCV_LIBS} ${PCL_LIBRARIES} Eigen3::Eigen
                      Boost::system Boost::filesystem)
if(OpenMP_CXX_FOUND)
  target_link_libraries(radloc_odometry_core PUBLIC OpenMP::OpenMP_CXX)
endif()

add_executable(odometry_node src/odometry_node.cpp)
target_link_libraries(odometry_node radloc_odometry_core radloc::radloc ${catkin_LIBRARIES})
add_dependencies(odometry_node ${PROJECT_NAME}_generate_messages_cpp)

add_executable(pose_graph_node src/pose_graph_node.cpp ${COMMON}/src/loop_registration.cpp
               ${COMMON}/src/graph_io.cpp)
target_link_libraries(pose_graph_node radloc::radloc gtsam ${PCL_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(pose_graph_node ${PROJECT_NAME}_generate_messages_cpp)

add_executable(reoptimize ${COMMON}/src/reoptimize.cpp ${COMMON}/src/graph_io.cpp)
target_link_libraries(reoptimize gtsam)

install(TARGETS odometry_node pose_graph_node reoptimize RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
