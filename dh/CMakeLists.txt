cmake_minimum_required(VERSION 2.8.11)

macro(generate_icub_dh)
    set(options)
    set(oneValueArgs YARP_ROBOT_NAME PARTS_VERSIONS)
    set(multiValueArgs)
    cmake_parse_arguments(GIDH "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

    add_custom_command(OUTPUT "${CMAKE_BINARY_DIR}/${BUILD_PREFIX}/robots/${GIDH_YARP_ROBOT_NAME}/model.urdf"
                       MAIN_DEPENDENCY ${CMAKE_CURRENT_SOURCE_DIR}/data/legacy_models/${GIDH_PARTS_VERSIONS}/model.urdf
                       COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/data/legacy_models/${GIDH_PARTS_VERSIONS}/model.urdf
                                                        ${CMAKE_BINARY_DIR}/${BUILD_PREFIX}/robots/${GIDH_YARP_ROBOT_NAME}/model.urdf)

    list(APPEND model-dh-generated-models "${CMAKE_BINARY_DIR}/${BUILD_PREFIX}/robots/${GIDH_YARP_ROBOT_NAME}/model.urdf")
endmacro()

set(model-dh-generated-models "")
generate_icub_dh(YARP_ROBOT_NAME iCubGenova03
                 PARTS_VERSIONS headV2-armsV2-legsV1-feetV2)

generate_icub_dh(YARP_ROBOT_NAME iCubNancy01
                 PARTS_VERSIONS headV2-armsV1-legsV2-feetV2)

generate_icub_dh(YARP_ROBOT_NAME iCubLisboa01
                 PARTS_VERSIONS headV2-armsV1-legsV1-feetV1)

add_custom_target(generate-models-dh
                  ALL
                  DEPENDS ${model-dh-generated-models})

# Copy the meshes in the meshes/upmc directory
add_custom_command(TARGET generate-models-dh
                   POST_BUILD
                   COMMAND ${CMAKE_COMMAND} -E make_directory "${CMAKE_BINARY_DIR}/${BUILD_PREFIX}/meshes/upmc"
                   COMMAND ${CMAKE_COMMAND} -E copy_directory "${CMAKE_CURRENT_SOURCE_DIR}/data/urdf_paris/icub/meshes" "${CMAKE_BINARY_DIR}/${BUILD_PREFIX}/meshes/upmc"
                   COMMENT "Copying UPMC meshes")