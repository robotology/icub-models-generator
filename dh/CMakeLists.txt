cmake_minimum_required(VERSION 2.8.11)

# Compile the actual dh --> URDF converter
add_subdirectory(generator)


# Generate URDF models for
# v1/v2 robots using the simmechanics-to-urdf script
macro(generate_icub_dh)
    set(options)
    set(oneValueArgs YARP_ROBOT_NAME)
    set(multiValueArgs PARTS_VERSIONS)
    cmake_parse_arguments(GIDH "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})


    add_custom_command(OUTPUT ${GIDH_YARP_ROBOT_NAME}.urdf
                       COMMAND icub_urdf_sdf_from_dh_generator
                       ARGS ${GIDH_PARTS_VERSIONS}
                            --outputfile ${GIDH_YARP_ROBOT_NAME}.urdf
                            --YARP_ROBOT_NAME ${GIDH_YARP_ROBOT_NAME}
                            --data_directory ${CMAKE_CURRENT_SOURCE_DIR}/data/
                       MAIN_DEPENDENCY "${CMAKE_CURRENT_SOURCE_DIR}/data/urdf_paris/icub/icub.xml")


    add_custom_command(OUTPUT "${CMAKE_BINARY_DIR}/${BUILD_PREFIX}/robots/${GIDH_YARP_ROBOT_NAME}/model.urdf"
                       MAIN_DEPENDENCY "${GIDH_YARP_ROBOT_NAME}.urdf"
                       COMMAND ${CMAKE_COMMAND} -E copy "${GIDH_YARP_ROBOT_NAME}.urdf"
                                                         "${CMAKE_BINARY_DIR}/${BUILD_PREFIX}/robots/${GIDH_YARP_ROBOT_NAME}/model.urdf")

    list(APPEND model-dh-generated-models "${CMAKE_BINARY_DIR}/${BUILD_PREFIX}/robots/${GIDH_YARP_ROBOT_NAME}/model.urdf")
endmacro()

set(model-dh-generated-models "")
generate_icub_dh(YARP_ROBOT_NAME iCubGenova03
                 PARTS_VERSIONS --headV2 --armsV2 --legsV1 --feetV2)

generate_icub_dh(YARP_ROBOT_NAME iCubNancy01
                 PARTS_VERSIONS --headV2 --armsV1 --legsV2 --feetV2)

generate_icub_dh(YARP_ROBOT_NAME iCubParis01
                 PARTS_VERSIONS --headV1 --armsV1 --legsV1 --feetV2)

generate_icub_dh(YARP_ROBOT_NAME iCubParis02
                 PARTS_VERSIONS --headV2 --armsV2 --legsV1 --feetV2 --iCubParis02)
                 
generate_icub_dh(YARP_ROBOT_NAME iCubLisboa01 
                 PARTS_VERSIONS --headV2 --armsV1 --legsV1 --feetV1)
                 
# Not a real commit that is meant to be merged to master, just a way to quickly generate all
# possible combinations of dh-generated models, so we can remove the generator that depends on
# iDynTree 1 and other deprecated software, and just copy the required configuration
set(headoptions headV1 headV2)
set(armsoptions armsV1 armsV2)
set(legsoptions legsV1 legsV2)
set(feetoptions feetV1 feetV2)
foreach(headflag headoptions)
    foreach(armsflag armsoptions)
        foreach(legsflag legsoptions)
            foreach(feetflag feetoptions)
                message(STATUS "icub-model-generator-dh: generating model: iCubModelLegacy-${headflag}-${armsflag}-${legsflag}-${feetflag}")
                generate_icub_dh(YARP_ROBOT_NAME iCubModelLegacy-${headflag}-${armsflag}-${legsflag}-${feetflag}
                                 PARTS_VERSIONS --${headflag} --${armsflag} --${legsflag} --${feetflag})
            endforeach()
        endforeach()
    endforeach()
endforeach()

message(STATUS "icub-model-generator-dh: generated models: ${model-dh-generated-models}")

add_custom_target(generate-models-dh
                  ALL
                  DEPENDS ${model-dh-generated-models})

# Copy the meshes in the meshes/upmc directory
add_custom_command(TARGET generate-models-dh
                   POST_BUILD
                   COMMAND ${CMAKE_COMMAND} -E make_directory "${CMAKE_BINARY_DIR}/${BUILD_PREFIX}/meshes/upmc"
                   COMMAND ${CMAKE_COMMAND} -E copy_directory "${CMAKE_CURRENT_SOURCE_DIR}/data/urdf_paris/icub/meshes" "${CMAKE_BINARY_DIR}/${BUILD_PREFIX}/meshes/upmc"
                   COMMENT "Copying UPMC meshes")
