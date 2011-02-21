cmake_minimum_required (VERSION 2.4.6)

include ($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
include ($ENV{ROS_ROOT}/core/rosbuild/FindPkgConfig.cmake)

set (ROS_BUILD_TYPE RelWithDebInfo)
rosbuild_init ()
rosbuild_add_boost_directories ()
rosbuild_check_for_sse ()

set (EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
include_directories (${CMAKE_CURRENT_BINARY_DIR})

pkg_check_modules (LIBUSB REQUIRED libusb-1.0)
include_directories (${LIBUSB_INCLUDE_DIRS})
link_directories (${LIBUSB_LIBRARY_DIRS})

rosbuild_add_executable (nu_skeletontracker src/nu_skeletontracker.cpp)
