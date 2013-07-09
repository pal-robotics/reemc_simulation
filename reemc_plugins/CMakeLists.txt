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

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
 pkg_check_modules(GAZEBO gazebo)
endif()
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_add_library(REEMCPlugin  src/REEMCPlugin.cpp)
target_link_libraries(REEMCPlugin ${GAZEBO_LIBRARIES})

rosbuild_add_library(FootContactModelPlugin  src/FootContactModelPlugin.cpp)
target_link_libraries(FootContactModelPlugin ${GAZEBO_LIBRARIES})

rosbuild_add_library(gazebo_ros_force_plugin  src/gazebo_ros_force.cpp)
target_link_libraries(gazebo_ros_force_plugin ${GAZEBO_LIBRARIES})
