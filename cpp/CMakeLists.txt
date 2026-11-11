cmake_minimum_required(VERSION 3.16)
project(radloc VERSION 2.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs)

# Header-only core: OpenCV only. No ROS, no PCL, no FFTW.
add_library(radloc INTERFACE)
add_library(radloc::radloc ALIAS radloc)
target_include_directories(radloc INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS})
target_link_libraries(radloc INTERFACE ${OpenCV_LIBS})
target_compile_features(radloc INTERFACE cxx_std_17)

# Consumers that add_subdirectory() this project usually want the library only.
option(RADLOC_BUILD_TOOLS "Build the validation tools and the tests" ON)
if(RADLOC_BUILD_TOOLS)
  foreach(tool describe validate_descriptor validate_retrieval test_phase_corr make_session)
    add_executable(${tool} tools/${tool}.cpp)
    target_link_libraries(${tool} PRIVATE radloc)
  endforeach()
  enable_testing()
  add_executable(test_radloc tests/test_radloc.cpp)
  target_link_libraries(test_radloc PRIVATE radloc)
  add_test(NAME radloc COMMAND test_radloc)
endif()

include(GNUInstallDirs)
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(TARGETS radloc EXPORT radlocTargets)
install(EXPORT radlocTargets NAMESPACE radloc:: DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/radloc)
file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/radlocConfig.cmake
  "include(CMakeFindDependencyMacro)\nfind_dependency(OpenCV)\ninclude(\${CMAKE_CURRENT_LIST_DIR}/radlocTargets.cmake)\n")
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/radlocConfig.cmake DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/radloc)
