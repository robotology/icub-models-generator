cmake_minimum_required(VERSION 2.8.11)

# Compile the actual dh --> URDF converter
add_subdirectory(generator)


# Generate URDF models for
# v1/v2 robots using the simmechanics-to-urdf script
macro(generate_icub_dh)
    set(options)
    set(oneValueArgs YARP_ROBOT_NAME PARTS_VERSIONS)
    set(multiValueArgs)
    cmake_parse_arguments(GIDH "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})


    add_custom_command(OUTPUT ${GIDH_YARP_ROBOT_NAME}.urdf
                       COMMAND icub_urdf_sdf_from_dh_generator
                       ARGS ${GIDH_PARTS_VERSIONS}
                            --outputfile ${GIDH_YARP_ROBOT_NAME}.urdf
                            --YARP_ROBOT_NAME ${GIDH_YARP_ROBOT_NAME}
                            --data_directory ${CMAKE_CURRENT_SOURCE_DIR}/data/
                       MAIN_DEPENDENCY "${CMAKE_CURRENT_SOURCE_DIR}/data/urdf_paris/icub/icub.xml")


    # instead of just copyng, we also modify the com on the fly
    add_custom_command(OUTPUT "${CMAKE_SOURCE_DIR}/generated/robots/${GIDH_YARP_ROBOT_NAME}/model.urdf"
                       MAIN_DEPENDENCY "${GIDH_YARP_ROBOT_NAME}.urdf"
                       COMMAND ${CMAKE_COMMAND} -E copy "${GIDH_YARP_ROBOT_NAME}.urdf"
                                                         "${CMAKE_SOURCE_DIR}/generated/robots/${GIDH_YARP_ROBOT_NAME}/model.urdf")


    list(APPEND model-dh-generated-models "${CMAKE_SOURCE_DIR}/generated/robots/${GIDH_YARP_ROBOT_NAME}/model.urdf")
endmacro()

set(model-dh-generated-models "")
generate_icub_dh(YARP_ROBOT_NAME iCubGenova03
                 PARTS_VERSIONS "--headV2 --armsV2 --legsV1 --feetV2")

generate_icub_dh(YARP_ROBOT_NAME iCubParis01
                 PARTS_VERSIONS "--headV1 --armsV1 --legsV1 --feetV2")

generate_icub_dh(YARP_ROBOT_NAME iCubParis02
                 PARTS_VERSIONS "--headV2 --armsV2 --legsV1 --feetV2 --iCubParis02")

add_custom_target(generate-models-dh
                  ALL
                  DEPENDS ${model-dh-generated-models})




