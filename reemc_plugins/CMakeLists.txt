cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#include (FindPkgConfig)
#if (PKG_CONFIG_FOUND)
#  pkg_check_modules(GAZEBO gazebo)
#endif()
#include_directories(${GAZEBO_INCLUDE_DIRS})
#link_directories(${GAZEBO_LIBRARY_DIRS})

# find osrf-common
#find_package(osrf-common)
#if (NOT osrf-common_FOUND)
#  message(STATUS "osrf-common not found, not building atlas with joint controllers.")
#  set(HAVE_OSRF_COMMON false)
#else ()
#  message(STATUS "osrf-common found [${OSRF_COMMON_ROS_PACKAGE_PATH}], building atlas with joint controllers.")
#  set(HAVE_OSRF_COMMON true)
#endif ()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(PalSimPlugin  src/PalSimPlugin.cpp)
#target_link_libraries(PalSimPlugin ${GAZEBO_libraries})

#rosbuild_add_library(REEMCPlugin  src/REEMCPlugin.cpp)
#target_link_libraries(REEMCPlugin ${GAZEBO_libraries})

#rosbuild_add_library(FTPlugin  src/gazebo_ros_forcetorque.cpp)
#target_link_libraries(FTPlugin ${GAZEBO_libraries})

#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
