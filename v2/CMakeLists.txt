# Generate URDF models for
# v2.5 robots using the simmechanics-to-urdf
macro(generate_icub_v2 yarpRobotName simmechanicsXML simmechanicsYAML simmechanicsCSV)
    add_custom_command(OUTPUT ${yarpRobotName}.urdf
                       COMMAND simmechanics_to_urdf
                       ARGS ${CMAKE_CURRENT_SOURCE_DIR}/data/${simmechanicsXML}
                            --output xml
                            --yaml ${CMAKE_CURRENT_SOURCE_DIR}/data/${simmechanicsYAML}
                            --csv-joints ${CMAKE_CURRENT_SOURCE_DIR}/data/${simmechanicsCSV}
                            --outputfile ${yarpRobotName}.urdf
                       MAIN_DEPENDENCY "${CMAKE_CURRENT_SOURCE_DIR}/data/${simmechanicsXML}"
                       DEPENDS  "${CMAKE_CURRENT_SOURCE_DIR}/data/${simmechanicsYAML}"
                                "${CMAKE_CURRENT_SOURCE_DIR}/data/${simmechanicsCSV}")
    add_custom_command(OUTPUT "${CMAKE_SOURCE_DIR}/generated/robots/${yarpRobotName}/model.urdf"
                       MAIN_DEPENDENCY "${yarpRobotName}.urdf"
                       COMMAND ${CMAKE_COMMAND} -E
                                copy "${yarpRobotName}.urdf" "${CMAKE_SOURCE_DIR}/generated/robots/${yarpRobotName}/model.urdf")
    list(APPEND model-v2-generated-models "${CMAKE_SOURCE_DIR}/generated/robots/${yarpRobotName}/model.urdf")
endmacro()

set(model-v2-generated-models "")
generate_icub_v2(iCubGenova02 "icub2_5/iCubGenova02_SIM_MODEL.xml"
                 "icub2_5/iCubGenova02_simmechanics_options.yaml" "icub2_5/iCubGenova02_joint_parameters.csv")
generate_icub_v2(iCubDarmstadt01 "icub2_5/iCubGenova02_SIM_MODEL.xml"
                 "icub2_5/iCubGenova02_simmechanics_options.yaml" "icub2_5/iCubGenova02_joint_parameters.csv")
generate_icub_v2(iCubGenova05 "icub2_5/iCubGenova02_SIM_MODEL.xml"
                 "icub2_5/iCubGenova02_simmechanics_options.yaml" "icub2_5/iCubGenova02_joint_parameters.csv")

add_custom_target(generate-models-v2
                  ALL
                  DEPENDS ${model-v2-generated-models})




