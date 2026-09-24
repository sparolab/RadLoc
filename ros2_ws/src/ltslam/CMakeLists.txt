cmake_minimum_required(VERSION 3.8)
project(ltslam)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(GTSAM REQUIRED)
find_package(OpenMP)

# The RadLoc core is header-only and lives outside the workspace, so it is added
# as a plain CMake subdirectory rather than an ament package.
set(RADLOC_CORE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../cpp/radloc"
    CACHE PATH "Path to the radloc C++ core")
if(NOT EXISTS "${RADLOC_CORE_DIR}/CMakeLists.txt")
  message(FATAL_ERROR "radloc core not found at ${RADLOC_CORE_DIR}")
endif()
add_subdirectory("${RADLOC_CORE_DIR}" "${CMAKE_CURRENT_BINARY_DIR}/radloc"
                 EXCLUDE_FROM_ALL)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${GTSAM_INCLUDE_DIR}
)

add_executable(ltslam_node
  src/main.cpp
  src/RosParamServer.cpp
  src/LTslam.cpp
  src/Session.cpp
  src/utility.cpp
)
target_link_libraries(ltslam_node
  radloc
  gtsam
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
)
if(OpenMP_CXX_FOUND)
  target_link_libraries(ltslam_node OpenMP::OpenMP_CXX)
endif()

install(TARGETS ltslam_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
