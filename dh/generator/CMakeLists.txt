# Copyright (C) 2016 Fondazione Istituto Italiano di Tecnologia - iCub Facility
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

find_package(YCM REQUIRED)
find_package(YARP REQUIRED)
find_package(ICUB REQUIRED)

find_package(urdfdom REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(TinyXML REQUIRED)

find_package(SDFormat REQUIRED)
find_package(iDynTree 0.3.5 REQUIRED)

set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${ICUB_LINK_FLAGS}")
# Add Gazebo CXX flags, to support sdformat 3 reckless dependency on C++11
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${SDFormat_CXX_FLAGS}")

# add YARP include directories
include_directories(${urdfdom_INCLUDE_DIR}
                    ${orocos_kdl_INCLUDE_DIRS}
                    ${EIGEN3_INCLUDE_DIR} ${YARP_INCLUDE_DIR} ${iDynTree_INCLUDE_DIRS}
                    ${iDyn_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS}
                    ${skinDynLib_INCLUDE_DIRS} ${SDFormat_INCLUDE_DIRS}
                    ${urdfdom_INCLUDE_DIRS})

add_library(urdf_from_dh_utils STATIC urdf_utils.h
                                      urdf_utils.cpp
                                      urdf_sdf_from_dh_utils.h
                                      urdf_sdf_from_dh_utils.cpp)

add_executable(icub_urdf_sdf_from_dh_generator icub_urdf_sdf_from_dh_generator.cpp)

target_link_libraries(icub_urdf_sdf_from_dh_generator urdf_from_dh_utils ${SDFormat_LIBRARIES} ${iDynTree_LIBRARIES} ${YARP_LIBRARIES} ${ICUB_LIBRARIES} ${kdl_codyco_LIBRARIES} ${orocos_kdl_LIBRARIES} ${urdfdom_LIBRARIES} ${iDynTree_LIBRARIES} ${TinyXML_LIBRARIES})

install(TARGETS icub_urdf_sdf_from_dh_generator DESTINATION bin)
