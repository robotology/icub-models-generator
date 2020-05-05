# Copyright: (C) 2016 iCub Facility - Fondazione Istituto Italiano di Tecnologia
# Authors: Silvio Traversaro
# CopyPolicy: Released under the terms of the GNU LGPL v2+.

cmake_minimum_required(VERSION 2.8.11)

project(icub-model-generator)

# Give error if add_dependencies is called on a non-existing target
if(POLICY CMP0046)
  cmake_policy(SET CMP0046 NEW)
endif()

option(ICUB_MODEL_GENERATE_DH "Generate models using the model generation pipeline from Denativ Hartenberg models" ON)
option(ICUB_MODEL_GENERATE_SIMMECHANICS "Generate models using the model generation pipeline from simmechanics" ON)

set(ICUB_BUILD_PREFIX "iCub")
set(COMPONENTS_BUILD_PREFIX "components")

if( ICUB_MODEL_GENERATE_DH )
  add_subdirectory(dh)
endif()

if( ICUB_MODEL_GENERATE_SIMMECHANICS )
  add_subdirectory(simmechanics)
endif()

add_custom_target(generate-models ALL)

if(TARGET generate-models-dh)
  add_dependencies(generate-models generate-models-dh)
endif()

if(TARGET generate-models-simmechanics)
  add_dependencies(generate-models generate-models-simmechanics)
endif()

# Logic to copy the generated models to icub-models
# We need to have the location of a YARP source dir
set(ICUB_MODELS_SOURCE_DIR "" CACHE STRING "Location of the icub-models repository source code")

add_custom_target(copy-models-to-icub-models)
add_dependencies(copy-models-to-icub-models generate-models)
add_custom_command(TARGET copy-models-to-icub-models
                   POST_BUILD
                   COMMAND ${CMAKE_COMMAND} -E copy_directory "${CMAKE_BINARY_DIR}/${ICUB_BUILD_PREFIX}" "${ICUB_MODELS_SOURCE_DIR}/iCub"
                   COMMENT "Copying generated files to ${ICUB_MODELS_SOURCE_DIR}/iCub")
                   COMMAND ${CMAKE_COMMAND} -E copy_directory "${CMAKE_BINARY_DIR}/${COMPONENTS_BUILD_PREFIX}" "${ICUB_MODELS_SOURCE_DIR}/components"
                   COMMENT "Copying generated files to ${ICUB_MODELS_SOURCE_DIR}/components")
include( CTest )
enable_testing()
add_subdirectory(tests)
