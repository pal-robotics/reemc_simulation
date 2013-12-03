cmake_minimum_required(VERSION 2.8.3)
project(reemc_gazebo)

find_package(catkin REQUIRED)

catkin_package()

foreach(dir config launch models worlds)
    install(DIRECTORY ${dir}/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
