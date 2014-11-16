###############################################################################
# Find Intel RealSense SDK
#
# This sets the following variables:
# RSSDK_FOUND - True if RealSense SDK was found.
# RSSDK_INCLUDE_DIRS - Directories containing the RealSense SDK include files.
# RSSDK_LIBRARIES - Libraries needed to use RealSense SDK.

find_path(RSSDK_DIR include/pxcversion.h
          PATHS "$ENV{RSSDK_DIR}"
		        "$ENV{PROGRAMFILES}/Intel/RSSDK"
				"$ENV{PROGRAMW6432}/Intel/RSSDK"
				"C:/Program Files (x86)/Intel/RSSDK"
				"C:/Program Files/Intel/RSSDK"
          DOC "RealSense SDK directory")

if(RSSDK_DIR)
  set(RSSDK_RELEASE_NAME libpxc.lib)
  set(RSSDK_DEBUG_NAME libpxc_d.lib)
  find_library(RSSDK_LIBRARY
               NAMES ${RSSDK_RELEASE_NAME}
               PATHS "${RSSDK_DIR}/lib/" NO_DEFAULT_PATH
               PATH_SUFFIXES x64 Win32)
  find_library(RSSDK_LIBRARY_DEBUG
               NAMES ${RSSDK_DEBUG_NAME} ${RSSDK_RELEASE_NAME}
               PATHS "${RSSDK_DIR}/lib/" NO_DEFAULT_PATH
               PATH_SUFFIXES x64 Win32)
  if(NOT RSSDK_LIBRARY_DEBUG)
    set(RSSDK_LIBRARY_DEBUG ${RSSDK_LIBRARY})
  endif()
  set(RSSDK_INCLUDE_DIRS ${RSSDK_DIR}/include)
  set(RSSDK_LIBRARIES optimized ${RSSDK_LIBRARY} debug ${RSSDK_LIBRARY_DEBUG})
  set(RSSDK_FOUND TRUE)
endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(RSSDK DEFAULT_MSG
                                  RSSDK_LIBRARY RSSDK_INCLUDE_DIRS)

mark_as_advanced(RSSDK_LIBRARY RSSDK_LIBRARY_DEBUG RSSDK_INCLUDE_DIRS)

if(RSSDK_FOUND)
  message(STATUS "RealSense SDK found (include: ${RSSDK_INCLUDE_DIRS}, libs: ${RSSDK_LIBRARIES})")
endif()

if(MSVC)
  set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} /NODEFAULTLIB:LIBCMT")
endif()