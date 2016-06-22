# Generate URDF models for
# v2.5 robots using the simmechanics-to-urdf

# todo transform to macro
add_custom_command(OUTPUT iCubGenova02.urdf
                   COMMAND simmechanics_to_urdf
                   ARGS ${CMAKE_CURRENT_SOURCE_DIR}/data/icub2_5/iCubGenova02_SIM_MODEL.xml
                        --output xml
                        --yaml ${CMAKE_CURRENT_SOURCE_DIR}/data/icub2_5/iCubGenova02_simmechanics_options.yaml
                        --csv-joints ${CMAKE_CURRENT_SOURCE_DIR}/data/icub2_5/iCubGenova02_joint_parameters.csv
                        --outputfile iCubGenova02.urdf
                   MAIN_DEPENDENCY "${CMAKE_CURRENT_SOURCE_DIR}/data/icub2_5/iCubGenova02_SIM_MODEL.xml"
                   DEPENDS  "${CMAKE_CURRENT_SOURCE_DIR}/data/icub2_5/iCubGenova02_simmechanics_options.yaml"
                            "${CMAKE_CURRENT_SOURCE_DIR}/data/icub2_5/iCubGenova02_joint_parameters.csv")


add_custom_target(generate-models-v2
                  ALL
                  DEPENDS iCubGenova02.urdf)


