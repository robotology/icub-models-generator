# Tests for generated models
# We use iDynTree & YARP for the tests
find_package(iDynTree REQUIRED)
find_package(YARP     REQUIRED)

add_executable(icub-model-test icub-model-test.cpp)
target_link_libraries(icub-model-test ${iDynTree_LIBRARIES} YARP::YARP_os)

macro(add_icub_model_test yarpRobotName)
    add_test(NAME ${yarpRobotName}ConsistencyCheck
             COMMAND icub-model-test --model ${CMAKE_BINARY_DIR}/${BUILD_PREFIX}/robots/${yarpRobotName}/model.urdf)
endmacro()

# Model generated from DH
if( ICUB_MODEL_GENERATE_DH )
    add_icub_model_test(iCubGenova03)
    add_icub_model_test(iCubNancy01)
    add_icub_model_test(iCubParis01)
    add_icub_model_test(iCubParis02)
    add_icub_model_test(iCubLisboa01)
endif()

# Model generated with simmechanics
if( ICUB_MODEL_GENERATE_SIMMECHANICS )
    add_icub_model_test(iCubGazeboV2_5)
    add_icub_model_test(iCubGazeboV2_6)
    add_icub_model_test(iCubGazeboV2_7)
    add_icub_model_test(iCubGazeboV2_5_KIT_007)
    add_icub_model_test(iCubGazeboV3)
    add_icub_model_test(iCubGenova01)
    add_icub_model_test(iCubGenova02)
    add_icub_model_test(iCubGenova04)
    add_icub_model_test(iCubGenova09)
    add_icub_model_test(iCubErzelli02)
    add_icub_model_test(iCubDarmstadt01)
    add_icub_model_test(iCubWaterloo01)
endif()
