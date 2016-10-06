cmake_minimum_required(VERSION 2.8.3)
project(reemc_hardware_gazebo)

find_package(catkin REQUIRED COMPONENTS pal_hardware_gazebo)

find_package(gazebo REQUIRED)

catkin_package(
    INCLUDE_DIRS
    CATKIN_DEPENDS
    DEPENDS
    LIBRARIES
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)

