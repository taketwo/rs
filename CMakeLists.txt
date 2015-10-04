cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(rs)

option(BUILD_TESTS "Build tests." OFF)

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake/Modules/" ${CMAKE_MODULE_PATH})
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

find_package(PCL 1.7.2 REQUIRED)
find_package(RSSDK REQUIRED)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${RSSDK_INCLUDE_DIRS}
)

link_directories(
  ${PCL_LIBRARY_DIRS}
)

add_definitions(
  ${PCL_DEFINITIONS}
)

add_library(real_sense
  src/real_sense/real_sense_device_manager.cpp
  src/real_sense_grabber.cpp
  src/io_exception.cpp
  include/io_exception.h
  include/real_sense_grabber.h
  include/buffers.h
  include/impl/buffers.hpp
  include/real_sense/real_sense_device_manager.h
  include/real_sense/time.h
)

add_executable(real_sense_viewer
  src/real_sense_viewer.cpp
)
target_link_libraries(real_sense_viewer
  real_sense
  ${PCL_LIBRARIES}
  ${RSSDK_LIBRARIES}
)

if (BUILD_TESTS)
  enable_testing()
  add_subdirectory(test)
endif()

