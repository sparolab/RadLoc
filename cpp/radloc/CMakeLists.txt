cmake_minimum_required(VERSION 3.16)
project(radloc LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs)

# Header-only core: no ROS, no FFTW.
add_library(radloc INTERFACE)
target_include_directories(radloc INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(radloc INTERFACE ${OpenCV_LIBS})
target_include_directories(radloc INTERFACE ${OpenCV_INCLUDE_DIRS})

option(RADLOC_BUILD_TOOLS "Build validation/CLI tools" ON)
if(RADLOC_BUILD_TOOLS)
  add_executable(validate_descriptor tools/validate_descriptor.cpp)
  target_link_libraries(validate_descriptor PRIVATE radloc)
endif()

install(DIRECTORY include/ DESTINATION include)

if(RADLOC_BUILD_TOOLS)
  add_executable(validate_retrieval tools/validate_retrieval.cpp)
  target_link_libraries(validate_retrieval PRIVATE radloc)
endif()

if(RADLOC_BUILD_TOOLS)
  add_executable(test_phase_corr tools/test_phase_corr.cpp)
  target_link_libraries(test_phase_corr PRIVATE radloc)
endif()
