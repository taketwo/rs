cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(3DScan)

option(BUILD_TESTS "Build tests." OFF)

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake/Modules/" ${CMAKE_MODULE_PATH})

find_package(PCL 1.7.2 REQUIRED)
find_package(RSSDK REQUIRED)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_DEBUG_POSTFIX "_d")
ADD_DEFINITIONS(-DUNICODE)
ADD_DEFINITIONS(-D_UNICODE)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /UMBCS /D_UNICODE /DUNICODE")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /MD")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /MDd")

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${RSSDK_INCLUDE_DIRS}
  ${RSSDK_INCLUDE_SAMPLE_DIRS}
)

link_directories(
  ${PCL_LIBRARY_DIRS}
)

add_definitions(
  ${PCL_DEFINITIONS}
)

add_executable(3DScan
  src/3dscan.cpp
  src/3dscan_obj_reader.cpp
  src/3dscan_obj_pcl.cpp)
  
target_link_libraries(3DScan
  ${PCL_LIBRARIES}
  ${RSSDK_LIBRARIES}
  #${RSSDK_PXUTIL_LIBRARY} # Use this if not using multi threading ( visualizer)
  ${RSSDK_PXUTILMD_LIBRARY}
)

set_target_properties(3DScan PROPERTIES OUTPUT_NAME "3DScan")
if ( MSVC )
    set_target_properties(3DScan PROPERTIES RUNTIME_OUTPUT_DIRECTORY D:/waqar-working/RS-PCL-Scan/Build/ )
    #set_target_properties(3DScan PROPERTIES RUNTIME_OUTPUT_DIRECTORY D:/waqar-working/RS-PCL-Scan/Build/Release/ )
    # etc for the other available configuration types (MinSizeRel, RelWithDebInfo)
endif ( MSVC )
if (BUILD_TESTS)
  enable_testing()
  add_subdirectory(test)
endif()

