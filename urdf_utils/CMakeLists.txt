# Copyright (C) 2010 RobotCub Consortium
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

 
SET(PROJECTNAME urdf_utils)

PROJECT(${PROJECTNAME})

# YARP needs CMake 2.6 or greater
cmake_minimum_required(VERSION 2.6)

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)


find_package(urdfdom REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(kdl_format_io REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(TinyXML REQUIRED)
find_package(YARP REQUIRED)



# add YARP include directories
include_directories(${urdfdom_INCLUDE_DIR} ${kdl_format_io_INCLUDE_DIRS} ${orocos_kdl_INCLUDE_DIR}  ${EIGEN3_INCLUDE_DIR} ${YARP_INCLUDE_DIR})

add_executable(urdf_get_meshes urdf_get_meshes.cpp)
add_executable(urdf_get_limits urdf_get_limits.cpp)
add_executable(urdf_gazebo_cleanup urdf_gazebo_cleanup.cpp)

target_link_libraries(urdf_get_meshes ${urdfdom_LIBRARIES} ${orocos_kdl_LIBRARIES} ${TinyXML_LIBRARIES} ${kdl_format_io_LIBRARIES})
target_link_libraries(urdf_get_limits ${urdfdom_LIBRARIES} ${orocos_kdl_LIBRARIES} ${TinyXML_LIBRARIES} ${kdl_format_io_LIBRARIES})
target_link_libraries(urdf_gazebo_cleanup ${urdfdom_LIBRARIES} ${orocos_kdl_LIBRARIES} ${TinyXML_LIBRARIES} ${kdl_format_io_LIBRARIES} ${YARP_LIBRARIES})
