# Tests for generated models
# We use iDynTree & YARP for the tests
find_package(iDynTree REQUIRED)
find_package(YARP     REQUIRED)

add_executable(icub-model-test icub-model-test.cpp)
include_directories(${YARP_INCLUDE_DIRS})
target_link_libraries(icub-model-test ${iDynTree_LIBRARIES} ${YARP_LIBRARIES})

macro(add_icub_model_test yarpRobotName)
    add_test(NAME ${yarpRobotName}ConsistencyCheck
             COMMAND icub-model-test --model ${CMAKE_SOURCE_DIR}/generated/robots/${yarpRobotName}/model.urdf)
endmacro()

# Model generated from DH
if( ICUB_MODEL_GENERATE_DH )
    add_icub_model_test(iCubGenova03)
    add_icub_model_test(iCubNancy01)
    add_icub_model_test(iCubParis01)
    add_icub_model_test(iCubParis02)
endif()

# Model generated with simmechanics
if( ICUB_MODEL_GENERATE_SIMMECHANICS )
    add_icub_model_test(iCubGenova01)
    add_icub_model_test(iCubGenova02)
    add_icub_model_test(iCubGenova05)
    add_icub_model_test(iCubDarmstadt01)
endif()

