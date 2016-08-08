# Copyright: (C) 2016 iCub Facility - Fondazione Istituto Italiano di Tecnologia
# Authors: Silvio Traversaro
# CopyPolicy: Released under the terms of the GNU LGPL v2+.

cmake_minimum_required(VERSION 2.8.11)

project(icub-model-generator)

# Give error if add_dependencies is called on a non-existing target
if(POLICY CMP0046)
  cmake_policy(SET CMP0046 NEW)
endif()

option(ICUB_MODEL_GENERATE_DH "Generate models using the model generation pipeline from Denativ Hartenberg models" OFF)
option(ICUB_MODEL_GENERATE_SIMMECHANICS "Generate models using the model generation pipeline from simmechanics" ON)

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

include( CTest )
enable_testing()
add_subdirectory(tests)
