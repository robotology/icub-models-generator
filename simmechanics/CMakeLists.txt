find_package(PythonInterp REQUIRED)

# Generate URDF models for
# v2.5 robots using the simmechanics-to-urdf script
macro(generate_icub_simmechanics)
    set(options NO_BACKPACK BOGUS INCREASE_INERTIA_FOR_GAZEBO)
    set(oneValueArgs YARP_ROBOT_NAME SIMMECHANICS_XML YAML_TEMPLATE CSV)
    set(multiValueArgs)
    cmake_parse_arguments(GIVTWO "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

    # Generate the YAML file from the CMake configuration
    if(GIVTWO_NO_BACKPACK)
      set(CHEST_ASSIGNED_MASS 5.6380)
    else()
      set(CHEST_ASSIGNED_MASS 7.6380)
    endif()
    if(GIVTWO_INCREASE_INERTIA_FOR_GAZEBO)
      set(GAZEBO_ASSIGNED_INERTIAS
"assignedInertias:
  - linkName: r_hip_3
    xx: 0.01
    yy: 0.01
    zz: 0.01
  - linkName: r_ankle_2
    xx: 0.01
    yy: 0.01
    zz: 0.01
  - linkName: l_hip_3
    xx: 0.01
    yy: 0.01
    zz: 0.01
  - linkName: l_ankle_2
    xx: 0.01
    yy: 0.01
    zz: 0.01
  - linkName: r_shoulder_1
    xx: 0.01
    yy: 0.01
    zz: 0.01
  - linkName: r_shoulder_2
    xx: 0.01
    yy: 0.01
    zz: 0.01
  - linkName: r_shoulder_3
    xx: 0.01
    yy: 0.01
    zz: 0.01
  - linkName: r_elbow_1
    xx: 0.01
    yy: 0.01
    zz: 0.01
  - linkName: r_wrist_1
    xx: 0.01
    yy: 0.01
    zz: 0.01
  - linkName: r_hand
    xx: 0.01
    yy: 0.01
    zz: 0.01
  - linkName: l_shoulder_1
    xx: 0.01
    yy: 0.01
    zz: 0.01
  - linkName: l_shoulder_2
    xx: 0.01
    yy: 0.01
    zz: 0.01
  - linkName: l_shoulder_3
    xx: 0.01
    yy: 0.01
    zz: 0.01
  - linkName: l_elbow_1
    xx: 0.01
    yy: 0.01
    zz: 0.01
  - linkName: l_wrist_1
    xx: 0.01
    yy: 0.01
    zz: 0.01
  - linkName: l_hand
    xx: 0.01
    yy: 0.01
    zz: 0.01
  - linkName: neck_1
    xx: 0.01
    yy: 0.01
    zz: 0.01
  - linkName: neck_2
    xx: 0.01
    yy: 0.01
    zz: 0.01
")
    else()
      set(GAZEBO_ASSIGNED_INERTIAS "")
    endif()
    set(GENERATED_YAML_LOCATION ${CMAKE_CURRENT_BINARY_DIR}/${GIVTWO_YARP_ROBOT_NAME}.yaml)
    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/data/${GIVTWO_YAML_TEMPLATE}
                   ${GENERATED_YAML_LOCATION}
                   @ONLY)

    add_custom_command(OUTPUT ${GIVTWO_YARP_ROBOT_NAME}.urdf
                       COMMAND simmechanics_to_urdf
                       ARGS ${CMAKE_CURRENT_SOURCE_DIR}/data/${GIVTWO_SIMMECHANICS_XML}
                            --output xml
                            --yaml ${GENERATED_YAML_LOCATION}
                            --csv-joints ${CMAKE_CURRENT_SOURCE_DIR}/data/${GIVTWO_CSV}
                            --outputfile ${GIVTWO_YARP_ROBOT_NAME}.urdf
                       MAIN_DEPENDENCY "${CMAKE_CURRENT_SOURCE_DIR}/data/${GIVTWO_SIMMECHANICS_XML}"
                       DEPENDS  "${GENERATED_YAML_LOCATION}"
                                "${CMAKE_CURRENT_SOURCE_DIR}/data/${GIVTWO_CSV}")



    # If we are generating a model without backpack, we need to tweack the COM of the chest link,
    # and we have a custon python script for this
    if(${GIVTWO_NO_BACKPACK})
        # instead of just copyng, we also modify the com on the fly
        add_custom_command(OUTPUT "${CMAKE_BINARY_DIR}/${BUILD_PREFIX}/robots/${GIVTWO_YARP_ROBOT_NAME}/model.urdf"
                           MAIN_DEPENDENCY "${GIVTWO_YARP_ROBOT_NAME}.urdf"
                           COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/data/icub2_5/adjust_com_for_model_without_backpack.py
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
generate_icub_simmechanics(YARP_ROBOT_NAME iCubGazeboV2_5
                           SIMMECHANICS_XML "icub2_5/ICUB_2-5_BB_SIM_MODEL.xml"
                           YAML_TEMPLATE "icub2_5/ICUB_2-5_BB_simmechanics_options.yaml.in"
                           CSV "icub2_5/ICUB_2-5_BB_joint_parameters.csv"
                           INCREASE_INERTIA_FOR_GAZEBO)

generate_icub_simmechanics(YARP_ROBOT_NAME iCubGenova02
                           SIMMECHANICS_XML "icub2_5/ICUB_2-5_BB_SIM_MODEL.xml"
                           YAML_TEMPLATE "icub2_5/ICUB_2-5_BB_simmechanics_options.yaml.in"
                           CSV "icub2_5/ICUB_2-5_BB_joint_parameters.csv")

generate_icub_simmechanics(YARP_ROBOT_NAME iCubGenova04
                           SIMMECHANICS_XML "icub2_5/ICUB_2-5_BB_SIM_MODEL.xml"
                           YAML_TEMPLATE "icub2_5/ICUB_2-5_BB_simmechanics_options.yaml.in"
                           CSV "icub2_5/ICUB_2-5_BB_joint_parameters.csv")

generate_icub_simmechanics(YARP_ROBOT_NAME iCubDarmstadt01
                           SIMMECHANICS_XML "icub2_5/ICUB_2-5_BB_SIM_MODEL.xml"
                           YAML_TEMPLATE "icub2_5/ICUB_2-5_BB_simmechanics_options.yaml.in"
                           CSV "icub2_5/ICUB_2-5_BB_joint_parameters.csv"
                           NO_BACKPACK)

generate_icub_simmechanics(YARP_ROBOT_NAME iCubGenova01
                           SIMMECHANICS_XML "icub2_5/ICUB_2-5_BB_SIM_MODEL.xml"
                           YAML_TEMPLATE "icub2_5/ICUB_2-5_BB_simmechanics_options.yaml.in"
                           CSV "icub2_5/ICUB_2-5_BB_joint_parameters.csv"
                           NO_BACKPACK)

add_custom_target(generate-models-simmechanics
                  ALL
                  DEPENDS ${model-simmechanics-generated-models})

# Copy the meshes in the meshes/simmechanics directory
add_custom_command(TARGET generate-models-simmechanics
                   POST_BUILD
                   COMMAND ${CMAKE_COMMAND} -E copy_directory "${CMAKE_CURRENT_SOURCE_DIR}/data/icub2_5/meshes" "${CMAKE_BINARY_DIR}/${BUILD_PREFIX}/meshes"
                   COMMAND ${CMAKE_COMMAND} -E copy_directory "${CMAKE_CURRENT_SOURCE_DIR}/data/icub2_5/conf" "${CMAKE_BINARY_DIR}/${BUILD_PREFIX}/conf"
                   COMMENT "Copying Simmechanics meshes")





