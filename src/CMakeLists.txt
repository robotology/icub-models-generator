# Copyright (C) 2010 RobotCub Consortium
# CopyPolicy: Released under the terms of the GNU GPL v2.0.


SET(PROJECTNAME urdf_utils)

PROJECT(${PROJECTNAME})

# YARP needs CMake 2.6 or greater
cmake_minimum_required(VERSION 2.6)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)


find_package(urdfdom REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(kdl_format_io REQUIRED)
find_package(Eigen3 REQUIRED)
#find_package(TinyXML REQUIRED)
find_package(YARP REQUIRED)
find_package(ICUB REQUIRED)
find_package(SDFormat REQUIRED)
find_package(iDynTree REQUIRED)

# add YARP include directories
include_directories(${urdfdom_INCLUDE_DIR}
                    ${kdl_format_io_INCLUDE_DIRS}
                    ${orocos_kdl_INCLUDE_DIR}
                    ${EIGEN3_INCLUDE_DIR} ${YARP_INCLUDE_DIR} ${iDynTree_INCLUDE_DIRS}
                    ${iDyn_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS}
                    ${skinDynLib_INCLUDE_DIRS} ${SDFormat_INCLUDE_DIRS})

add_library(urdf_utils STATIC urdf_utils.h urdf_utils.cpp)

add_executable(urdf_get_meshes urdf_get_meshes.cpp)
add_executable(urdf_get_limits urdf_get_limits.cpp)
add_executable(urdf_gazebo_cleanup urdf_gazebo_cleanup.cpp)
add_executable(urdf_get_total_mass urdf_get_total_mass.cpp)
add_executable(icub_urdf_from_iDyn icub_urdf_from_iDyn.cpp)
add_executable(icub_urdf_sdf_generator icub_urdf_sdf_generator.cpp)
add_executable(urdf_print_homogeneous_transformations urdf_print_homogeneous_transformations.cpp)
                                   

target_link_libraries(urdf_get_meshes urdf_utils ${urdfdom_LIBRARIES}  ${orocos_kdl_LIBRARIES} ${TinyXML_LIBRARIES} ${kdl_format_io_LIBRARIES})
target_link_libraries(urdf_get_limits urdf_utils ${urdfdom_LIBRARIES} ${orocos_kdl_LIBRARIES} ${TinyXML_LIBRARIES} ${kdl_format_io_LIBRARIES})
target_link_libraries(urdf_gazebo_cleanup urdf_utils ${urdfdom_LIBRARIES}  ${orocos_kdl_LIBRARIES} ${TinyXML_LIBRARIES} ${kdl_format_io_LIBRARIES} ${YARP_LIBRARIES})
target_link_libraries(urdf_get_total_mass urdf_utils ${urdfdom_LIBRARIES} ${orocos_kdl_LIBRARIES} ${TinyXML_LIBRARIES} ${kdl_format_io_LIBRARIES} ${YARP_LIBRARIES})
target_link_libraries(icub_urdf_from_iDyn ${YARP_LIBRARIES} ${ICUB_LIBRARIES} ${iDynTree_LIBRARIES} ${kdl_codyco_LIBRARIES} ${orocos_kdl_LIBRARIES} ${urdfdom_LIBRARIES} ${kdl_format_io_LIBRARIES} ${TinyXML_LIBRARIES})
target_link_libraries(icub_urdf_sdf_generator urdf_utils ${SDFormat_LIBRARIES} ${iDynTree_LIBRARIES} ${YARP_LIBRARIES} ${ICUB_LIBRARIES} ${kdl_codyco_LIBRARIES} ${orocos_kdl_LIBRARIES} ${urdfdom_LIBRARIES} ${kdl_format_io_LIBRARIES} ${TinyXML_LIBRARIES})
target_link_libraries(urdf_print_homogeneous_transformations  urdf_utils ${urdfdom_LIBRARIES} ${orocos_kdl_LIBRARIES} ${TinyXML_LIBRARIES} ${kdl_format_io_LIBRARIES} ${YARP_LIBRARIES})


INSTALL(TARGETS urdf_get_meshes DESTINATION bin)
INSTALL(TARGETS urdf_get_limits DESTINATION bin)
INSTALL(TARGETS urdf_gazebo_cleanup DESTINATION bin)
INSTALL(TARGETS urdf_get_total_mass DESTINATION bin)
INSTALL(TARGETS icub_urdf_from_iDyn DESTINATION bin)
INSTALL(TARGETS icub_urdf_sdf_generator DESTINATION bin)
INSTALL(TARGETS urdf_print_homogeneous_transformations  DESTINATION bin)

