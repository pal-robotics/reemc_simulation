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

rosbuild_find_ros_package(xacro)
rosbuild_find_ros_package(reemc_description)

# Generate an up-to-date URDF robot description of REEMC
set(INPUT_XACRO_FILE ${reemc_description_PACKAGE_PATH}/robots/reemc_full.urdf.xacro)
set(OUTPUT_URDF_FILE ${CMAKE_CURRENT_SOURCE_DIR}/models/reemc_full/model.urdf)

add_custom_command(OUTPUT ${OUTPUT_URDF_FILE}
                   COMMAND ${xacro_PACKAGE_PATH}/xacro.py ${INPUT_XACRO_FILE} -o ${OUTPUT_URDF_FILE})

add_custom_target(generate_reemc_full ALL DEPENDS ${OUTPUT_URDF_FILE})

# Generate an up-to-date URDF robot description of REEMC lower body
set(INPUT_XACRO_FILE ${reemc_description_PACKAGE_PATH}/robots/reemc_lower_body.urdf.xacro)
set(OUTPUT_URDF_FILE ${CMAKE_CURRENT_SOURCE_DIR}/models/reemc_lower_body/model.urdf)

add_custom_command(OUTPUT ${OUTPUT_URDF_FILE}
                   COMMAND ${xacro_PACKAGE_PATH}/xacro.py ${INPUT_XACRO_FILE} -o ${OUTPUT_URDF_FILE})

add_custom_target(generate_reemc_lower_body ALL DEPENDS ${OUTPUT_URDF_FILE})

# Generate an up-to-date URDF robot description of REEMC lower body
set(INPUT_XACRO_FILE ${reemc_description_PACKAGE_PATH}/robots/reemc_foot.urdf.xacro)
set(OUTPUT_URDF_FILE ${CMAKE_CURRENT_SOURCE_DIR}/models/reemc_foot/model.urdf)

add_custom_command(OUTPUT ${OUTPUT_URDF_FILE}
                   COMMAND ${xacro_PACKAGE_PATH}/xacro.py ${INPUT_XACRO_FILE} -o ${OUTPUT_URDF_FILE})

add_custom_target(generate_reemc_foot ALL DEPENDS ${OUTPUT_URDF_FILE})

# Create symlink to robot model resources (meshes, images)
set(INPUT_MESHES_DIR ../../../../reemc_common/reemc_description/meshes)
set(OUTPUT_MESHES_SYMLINK ${CMAKE_CURRENT_SOURCE_DIR}/models/reemc_description/meshes)

# NOTE: Because of a cmake bug [1], we cannot add a custom command/target pair here, otherwise calling 'make clean'
# deletes the pointed-to folder contents!. This is fixed in CMake 2.8.11
# [1] http://public.kitware.com/Bug/view.php?id=10538
execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink ${INPUT_MESHES_DIR} ${OUTPUT_MESHES_SYMLINK})

# add_custom_command(OUTPUT ${OUTPUT_MESHES_SYMLINK}
#                    COMMAND ${CMAKE_COMMAND} -E create_symlink ${INPUT_MESHES_DIR} ${OUTPUT_MESHES_SYMLINK})
#
# add_custom_target(generate_meshes_link ALL DEPENDS ${OUTPUT_MESHES_SYMLINK})

