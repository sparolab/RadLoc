# Multi-session radar SLAM (LT-SLAM style): aligns a query session to a central one through inter-session
# RadLoc loops. ROS-free: builds with plain CMake (any ROS 1 / ROS 2 image, or none) and, when ament is
# around, as a colcon package too.
#   cmake -S applications/multi-session-slam -B build/ms && cmake --build build/ms -j8
#   build/ms/multi_session_node applications/multi-session-slam/config/kaist03_split.yaml
cmake_minimum_required(VERSION 3.16)
project(radloc_multi_session)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake QUIET)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(GTSAM REQUIRED)
find_package(OpenMP)

set(RADLOC_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../.." CACHE PATH "RadLoc repository root")
if(NOT EXISTS "${RADLOC_ROOT}/cpp/CMakeLists.txt")
  message(FATAL_ERROR "radloc core not found under ${RADLOC_ROOT}/cpp")
endif()
set(RADLOC_BUILD_TOOLS OFF CACHE BOOL "" FORCE)
add_subdirectory("${RADLOC_ROOT}/cpp" "${CMAKE_CURRENT_BINARY_DIR}/radloc" EXCLUDE_FROM_ALL)

add_executable(multi_session_node
  src/main.cpp
  src/RosParamServer.cpp
  src/LTslam.cpp
  src/Session.cpp
  src/utility.cpp)
target_include_directories(multi_session_node PRIVATE
  include ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} ${GTSAM_INCLUDE_DIR})
target_link_libraries(multi_session_node radloc::radloc gtsam ${PCL_LIBRARIES} ${OpenCV_LIBS})
if(OpenMP_CXX_FOUND)
  target_link_libraries(multi_session_node OpenMP::OpenMP_CXX)
endif()

if(ament_cmake_FOUND)
  install(TARGETS multi_session_node DESTINATION lib/${PROJECT_NAME})
  install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
  ament_package()
else()
  install(TARGETS multi_session_node RUNTIME DESTINATION bin)
endif()
