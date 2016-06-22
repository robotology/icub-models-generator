# Copyright: (C) 2016 iCub Facility - Fondazione Istituto Italiano di Tecnologia
# Authors: Silvio Traversaro
# CopyPolicy: Released under the terms of the GNU LGPL v2+.

cmake_minimum_required(VERSION 2.8.11)

project(icub-model-generator)

# Give error if add_dependencies is called on a non-existing target
if(POLICY CMP0046)
  cmake_policy(SET CMP0046 NEW)
endif()

option(ICUB_MODEL_GENERATE_V1 "Generate models using the model generation pipeline v1" OFF)
option(ICUB_MODEL_GENERATE_V2 "Generate models using the model generation pipeline v2" ON)

if( ICUB_MODEL_GENERATE_V1 )
  add_subdirectory(v1)
endif()

if( ICUB_MODEL_GENERATE_V2 )
  add_subdirectory(v2)
endif()

add_custom_target(generate-models ALL)

if(TARGET generate-models-v1)
  add_dependencies(generate-models generate-models-v1)
endif()

if(TARGET generate-models-v2)
  add_dependencies(generate-models generate-models-v2)
endif()
