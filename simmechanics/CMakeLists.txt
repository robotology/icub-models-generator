find_package(PythonInterp REQUIRED)

# Generate URDF models for
# v2.5 robots using the simmechanics-to-urdf script
macro(generate_icub_simmechanics)
    set(options NO_BACKPACK BOGUS)
    set(oneValueArgs YARP_ROBOT_NAME SIMMECHANICS_XML YAML CSV)
    set(multiValueArgs)
    cmake_parse_arguments(GIVTWO "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})


    add_custom_command(OUTPUT ${GIVTWO_YARP_ROBOT_NAME}.urdf
                       COMMAND simmechanics_to_urdf
                       ARGS ${CMAKE_CURRENT_SOURCE_DIR}/data/${GIVTWO_SIMMECHANICS_XML}
                            --output xml
                            --yaml ${CMAKE_CURRENT_SOURCE_DIR}/data/${GIVTWO_YAML}
                            --csv-joints ${CMAKE_CURRENT_SOURCE_DIR}/data/${GIVTWO_CSV}
                            --outputfile ${GIVTWO_YARP_ROBOT_NAME}.urdf
                       MAIN_DEPENDENCY "${CMAKE_CURRENT_SOURCE_DIR}/data/${GIVTWO_SIMMECHANICS_XML}"
                       DEPENDS  "${CMAKE_CURRENT_SOURCE_DIR}/data/${GIVTWO_YAML}"
                                "${CMAKE_CURRENT_SOURCE_DIR}/data/${GIVTWO_CSV}")

    # If we are generating a model without backpack, we need to tweack the COM of the chest link,
    # and we have a custon python script for this
    if(${GIVTWO_NO_BACKPACK})
        # instead of just copyng, we also modify the com on the fly
        add_custom_command(OUTPUT "${CMAKE_BINARY_DIR}/${BUILD_PREFIX}/robots/${GIVTWO_YARP_ROBOT_NAME}/model.urdf"
                           MAIN_DEPENDENCY "${GIVTWO_YARP_ROBOT_NAME}.urdf"
                           COMMAND python ${CMAKE_CURRENT_SOURCE_DIR}/data/icub2_5/adjust_com_for_model_without_backpack.py
                                    --input_urdf  "${GIVTWO_YARP_ROBOT_NAME}.urdf"
                                    --output_urdf "${CMAKE_BINARY_DIR}/${BUILD_PREFIX}/robots/${GIVTWO_YARP_ROBOT_NAME}/model.urdf")
    else()
        add_custom_command(OUTPUT "${CMAKE_BINARY_DIR}/${BUILD_PREFIX}/robots/${GIVTWO_YARP_ROBOT_NAME}/model.urdf"
                           MAIN_DEPENDENCY "${GIVTWO_YARP_ROBOT_NAME}.urdf"
                           COMMAND ${CMAKE_COMMAND} -E
                                   copy "${GIVTWO_YARP_ROBOT_NAME}.urdf" "${CMAKE_BINARY_DIR}/${BUILD_PREFIX}/robots/${GIVTWO_YARP_ROBOT_NAME}/model.urdf")
    endif()

    list(APPEND model-simmechanics-generated-models "${CMAKE_BINARY_DIR}/${BUILD_PREFIX}/robots/${GIVTWO_YARP_ROBOT_NAME}/model.urdf")
endmacro()

set(model-simmechanics-generated-models "")
generate_icub_simmechanics(YARP_ROBOT_NAME iCubGenova02
                           SIMMECHANICS_XML "icub2_5/ICUB_2-5_BB_SIM_MODEL.xml"
                           YAML "icub2_5/ICUB_2-5_BB_simmechanics_options.yaml"
                           CSV "icub2_5/ICUB_2-5_BB_joint_parameters.csv")

generate_icub_simmechanics(YARP_ROBOT_NAME iCubGenova04
                           SIMMECHANICS_XML "icub2_5/ICUB_2-5_BB_SIM_MODEL.xml"
                           YAML "icub2_5/ICUB_2-5_BB_simmechanics_options.yaml"
                           CSV "icub2_5/ICUB_2-5_BB_joint_parameters.csv")

generate_icub_simmechanics(YARP_ROBOT_NAME iCubDarmstadt01
                           SIMMECHANICS_XML "icub2_5/ICUB_2-5_BB_SIM_MODEL.xml"
                           YAML "icub2_5/ICUB_2-5_BB_simmechanics_options_NO_BACKPACK.yaml"
                           CSV "icub2_5/ICUB_2-5_BB_joint_parameters.csv"
                           NO_BACKPACK)

generate_icub_simmechanics(YARP_ROBOT_NAME iCubGenova01
                           SIMMECHANICS_XML "icub2_5/ICUB_2-5_BB_SIM_MODEL.xml"
                           YAML "icub2_5/ICUB_2-5_BB_simmechanics_options_NO_BACKPACK.yaml"
                           CSV "icub2_5/ICUB_2-5_BB_joint_parameters.csv"
                           NO_BACKPACK)

generate_icub_simmechanics(YARP_ROBOT_NAME iCubGenova05
                           SIMMECHANICS_XML "icub2_5/ICUB_2-5_BB_SIM_MODEL.xml"
                           YAML "icub2_5/ICUB_2-5_BB_simmechanics_options_NO_BACKPACK.yaml"
                           CSV "icub2_5/ICUB_2-5_BB_joint_parameters.csv"
                           NO_BACKPACK)

add_custom_target(generate-models-simmechanics
                  ALL
                  DEPENDS ${model-simmechanics-generated-models})

# Copy the meshes in the meshes/simmechanics directory
add_custom_command(TARGET generate-models-simmechanics
                   POST_BUILD
                   COMMAND ${CMAKE_COMMAND} -E make_directory "${CMAKE_BINARY_DIR}/${BUILD_PREFIX}/meshes/simmechanics"
                   COMMAND ${CMAKE_COMMAND} -E copy_directory "${CMAKE_CURRENT_SOURCE_DIR}/data/icub2_5/simmechanics" "${CMAKE_BINARY_DIR}/${BUILD_PREFIX}/meshes/simmechanics"
                   COMMENT "Copying Simmechanics meshes")





