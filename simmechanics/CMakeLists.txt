find_package(PythonInterp REQUIRED)

# Generate URDF models for
# v2.5 robots using the simmechanics-to-urdf script
macro(generate_icub_simmechanics)
    set(options NO_BACKPACK
                BOGUS
                INCREASE_INERTIA_FOR_GAZEBO
                ICUB_PLUS
                ICUB_2_6
                ICUB_2_7
                ICUB_3_COLLISION_GEOM_MOD)
    set(oneValueArgs YARP_ROBOT_NAME SIMMECHANICS_XML YAML_TEMPLATE CSV_TEMPLATE)
    set(multiValueArgs)
    cmake_parse_arguments(GIVTWO "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

    # Fallback if no assigned collision geometry is requested
    set(ASSIGNED_COLLISION_GEOMETRIES "")

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
      set(LOWERBODY_JOINT_DAMPING 1.0)
      set(UPPERBODY_JOINT_DAMPING 1.0)
      if(GIVTWO_ICUB_PLUS)
        set(ASSIGNED_COLLISION_GEOMETRIES
"assignedCollisionGeometry:
  - linkName: r_foot
    geometricShape:
      shape: box
      size: 0.16 0.072 0.001
      origin: \"0.0417858 0.00499780 0.000725075 3.14159264324 0.261799192098 0.0\"
  - linkName: l_foot
    geometricShape:
      shape: box
      size: 0.16 0.072 0.001
      origin: \"0.0417858 -0.00499780 0.000725075 3.14159264324 0.261799192098 0.0\"
")
      else()
        set(ASSIGNED_COLLISION_GEOMETRIES
"assignedCollisionGeometry:
  - linkName: r_foot
    geometricShape:
      shape: box
      size: 0.16 0.072 0.001
      origin: \"0.03 0.005 0.014 0.0 0.0 0.0\"
  - linkName: l_foot
    geometricShape:
      shape: box
      size: 0.16 0.072 0.001
      origin: \"0.03 -0.005 0.014 0.0 0.0 0.0\"
")
      endif()
    else()
      set(GAZEBO_ASSIGNED_INERTIAS "")
      set(LOWERBODY_JOINT_DAMPING 0.223)
      set(UPPERBODY_JOINT_DAMPING 0.06)
    endif()

    if (GIVTWO_ICUB_3_COLLISION_GEOM_MOD)
      set(ASSIGNED_COLLISION_GEOMETRIES
"assignedCollisionGeometry:
  - linkName: r_foot_front
    geometricShape:
      shape: box
      size: 0.117 0.100 0.006
      origin: \"0.0 0.0 0.003 0.0 0.0 0.0\"
  - linkName: r_foot_rear
    geometricShape:
      shape: box
      size: 0.117 0.100 0.006
      origin: \"0.0 0.0 0.003 0.0 0.0 0.0\"
  - linkName: l_foot_front
    geometricShape:
      shape: box
      size: 0.117 0.100 0.006
      origin: \"0.0 0.0 0.003 0.0 0.0 0.0\"
  - linkName: l_foot_rear
    geometricShape:
      shape: box
      size: 0.117 0.100 0.006
      origin: \"0.0 0.0 0.003 0.0 0.0 0.0\"
")
    endif()

    # If we are generating the icub2.5+ model we need to add the xsens IMU, change the mesh folder
    # and change the rotation axis list that need to be reversed.
    if(GIVTWO_ICUB_PLUS)
      set(MESH_FILE_FORMAT "filenameformatchangeext: \"package://iCub/meshes/simmechanics/2-5_plus/%s-binary.stl\"")
      set(ANKLE_PITCH_ROM "-50,20")
      set(CUSTOM_EPSILON "epsilon: 2e-7")
      set(XSENS_IMU_FRAME
"  - frameName: SCSYS_ROOT_LINK_XSENS_IMU
    frameReferenceLink: root_link
    exportedFrameName: root_link_imu_frame
")
      set(FT_IMU_SENSORS "")
      set(XSENS_IMU_SENSOR
"  - frameName: SCSYS_ROOT_LINK_XSENS_IMU
    linkName: root_link
    exportFrameInURDF: Yes
    sensorName: root_link_imu_acc
    sensorType: \"accelerometer\"
    updateRate: \"400\"
    sensorBlobs:
    - |
        <plugin name=\"root_link_xsens_imu_plugin\" filename=\"libgazebo_yarp_imu.so\">
            <yarpConfigurationFile>model://iCub/conf/gazebo_icub_xsens_inertial.ini</yarpConfigurationFile>
        </plugin>
")
      set(REVERSE_ROTATION_AXIS
"reverseRotationAxis:
  l_shoulder_roll
  l_elbow
  l_hip_yaw
  l_knee
  l_ankle_pitch
  l_hip_pitch
  r_ankle_roll
  r_shoulder_pitch
  r_shoulder_yaw
  r_elbow
  r_wrist_prosup
  torso_pitch
  neck_roll
  r_wrist_yaw
")
    else()
      set(MESH_FILE_FORMAT "filenameformatchangeext: \"package://iCub/meshes/simmechanics/%s-binary.stl\"")
      set(ANKLE_PITCH_ROM "-35,35")
      set(CUSTOM_EPSILON "")
      set(XSENS_IMU_FRAME "")
      set(FT_IMU_SENSORS
"  - frameName: SCSYS_R_UPPER_LEG_FT-IMU_3B11
    linkName: r_hip_2
    sensorName: r_upper_leg_ft_acc_3b11
    sensorType: \"accelerometer\"
    sensorBlobs: \"\"
  - frameName: SCSYS_R_UPPER_LEG_FT-IMU_3B11
    linkName: r_hip_2
    sensorName: r_upper_leg_ft_gyro_3b11
    sensorType: \"gyroscope\"
    sensorBlobs: \"\"
  - frameName: SCSYS_R_FOOT_FT-IMU_3B14
    linkName: r_ankle_2
    sensorName: r_foot_ft_acc_3b14
    sensorType: \"accelerometer\"
    sensorBlobs: \"\"
  - frameName: SCSYS_R_FOOT_FT-IMU_3B14
    linkName: r_ankle_2
    sensorName: r_foot_ft_gyro_3b14
    sensorType: \"gyroscope\"
    sensorBlobs: \"\"
  - frameName: SCSYS_L_UPPER_LEG_FT-IMU_3B12
    linkName: l_hip_2
    sensorName: l_upper_leg_ft_acc_3b12
    sensorType: \"accelerometer\"
    sensorBlobs: \"\"
  - frameName: SCSYS_L_UPPER_LEG_FT-IMU_3B12
    linkName: l_hip_2
    sensorName: l_upper_leg_ft_gyro_3b12
    sensorType: \"gyroscope\"
    sensorBlobs: \"\"
  - frameName: SCSYS_L_FOOT_FT-IMU_3B13
    linkName: l_ankle_2
    sensorName: l_foot_ft_acc_3b13
    sensorType: \"accelerometer\"
    sensorBlobs: \"\"
  - frameName: SCSYS_L_FOOT_FT-IMU_3B13
    linkName: l_ankle_2
    sensorName: l_foot_ft_gyro_3b13
    sensorType: \"gyroscope\"
    sensorBlobs: \"\"
")
      set(XSENS_IMU_SENSOR "")
      set(REVERSE_ROTATION_AXIS
"reverseRotationAxis:
  l_shoulder_roll
  l_elbow
  l_hip_yaw
  l_knee
  l_ankle_pitch
  r_hip_pitch
  r_ankle_roll
  r_shoulder_pitch
  r_shoulder_yaw
  r_elbow
  r_wrist_prosup
  torso_pitch
  neck_roll
  r_wrist_yaw
")
    endif()
    # The ICUB_2_6 option is used to modify the iCub 2.5.5 model to
    # add some details of the iCub 2.6 until a proper iCub 2.6 model
    # is produced
    set(RFE_ADDITIONAL_TRANSFORMATION "")
    if(GIVTWO_ICUB_2_6 OR GIVTWO_ICUB_2_7)
      set(RFE_ADDITIONAL_TRANSFORMATION
         "additionalTransformation: [0.0323779, -0.0139537, 0.072, -3.14159265358979, 0, -1.5707963267949]")

    endif()
    # The ICUB_2_7 option is used to modify the iCub 2.5.5 model to
    # add some details of the iCub 2.7 until a proper iCub 2.7 model
    # is produced
    set(ADDITIONAL_XML_BLOBS "")
    if(GIVTWO_ICUB_2_7)
      set(ADDITIONAL_XML_BLOBS
"  - |
         <link name=\"root_link_imu_frame\"/>

  - |
         <joint name=\"root_link_imu_frame_fixed_joint\" type=\"fixed\">
           <origin xyz=\"0.08515499999999998 -0.011000000000000022 -0.11230899999999999\" rpy=\"-2.0943952105869323 0 -1.5707963267948963\"/>
           <parent link=\"root_link\"/>
           <child link=\"root_link_imu_frame\"/>
         </joint>
  - |
         <gazebo reference=\"root_link\">
           <sensor name=\"root_link_imu_acc\" type=\"imu\">
             <always_on>1</always_on>
             <update_rate>400</update_rate>
             <pose>0.08515499999999998 -0.011000000000000022 -0.11230899999999999 -2.0943952105869323 -3.885780586188048e-16 -1.5707963267948963</pose>
             <plugin name=\"root_link_xsens_imu_plugin\" filename=\"libgazebo_yarp_imu.so\">
               <yarpConfigurationFile>model://iCub/conf/gazebo_icub_xsens_inertial.ini</yarpConfigurationFile>
             </plugin>
            </sensor>
          </gazebo>")
    endif()

    set(GENERATED_YAML_LOCATION ${CMAKE_CURRENT_BINARY_DIR}/${GIVTWO_YARP_ROBOT_NAME}.yaml)
    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/data/${GIVTWO_YAML_TEMPLATE}
                   ${GENERATED_YAML_LOCATION}
                   @ONLY)
    set(GENERATED_CSV_LOCATION ${CMAKE_CURRENT_BINARY_DIR}/${GIVTWO_YARP_ROBOT_NAME}.csv)
    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/data/${GIVTWO_CSV_TEMPLATE}
                   ${GENERATED_CSV_LOCATION}
                   @ONLY)

    add_custom_command(OUTPUT ${GIVTWO_YARP_ROBOT_NAME}.urdf
                       COMMAND simmechanics_to_urdf
                       ARGS ${CMAKE_CURRENT_SOURCE_DIR}/data/${GIVTWO_SIMMECHANICS_XML}
                            --output xml
                            --yaml ${GENERATED_YAML_LOCATION}
                            --csv-joints ${GENERATED_CSV_LOCATION}
                            --outputfile ${GIVTWO_YARP_ROBOT_NAME}.urdf
                       MAIN_DEPENDENCY "${CMAKE_CURRENT_SOURCE_DIR}/data/${GIVTWO_SIMMECHANICS_XML}"
                       DEPENDS  "${GENERATED_YAML_LOCATION}"
                                "${GENERATED_CSV_LOCATION}")


    # If we are generating a model without backpack, we need to tweak the COM of the chest link,
    # and we have a custom python script for this
    if(${GIVTWO_NO_BACKPACK})
        # instead of just copying, we also modify the com on the fly
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
                           CSV_TEMPLATE "icub2_5/ICUB_2-5_BB_joint_parameters.csv.in"
                           INCREASE_INERTIA_FOR_GAZEBO)

generate_icub_simmechanics(YARP_ROBOT_NAME iCubGazeboV2_6
                           SIMMECHANICS_XML "icub2_5/ICUB_2-5_BB_SIM_MODEL.xml"
                           YAML_TEMPLATE "icub2_5/ICUB_2-5_BB_simmechanics_options.yaml.in"
                           CSV_TEMPLATE "icub2_5/ICUB_2-5_BB_joint_parameters.csv.in"
                           INCREASE_INERTIA_FOR_GAZEBO ICUB_2_6)

generate_icub_simmechanics(YARP_ROBOT_NAME iCubGazeboV2_7
                           SIMMECHANICS_XML "icub2_5/ICUB_2-5_BB_SIM_MODEL.xml"
                           YAML_TEMPLATE "icub2_5/ICUB_2-5_BB_simmechanics_options.yaml.in"
                           CSV_TEMPLATE "icub2_5/ICUB_2-5_BB_joint_parameters.csv.in"
                           INCREASE_INERTIA_FOR_GAZEBO ICUB_2_7)

generate_icub_simmechanics(YARP_ROBOT_NAME iCubGazeboV3
                           SIMMECHANICS_XML "icub3/ICUB3_ALL_SIM_MODEL.xml"
                           YAML_TEMPLATE "icub3/ICUB_3_all_options_gazebo.yaml.in"
                           CSV_TEMPLATE "icub3/ICUB_3_joint_all_parameters.csv"
                           ICUB_3_COLLISION_GEOM_MOD)

generate_icub_simmechanics(YARP_ROBOT_NAME left_hand_mk3
                           SIMMECHANICS_XML "components/left_hand_mk3/SIM_LEFT_HAND.xml"
                           YAML_TEMPLATE "components/left_hand_mk3/simmechanics2urdf_configfile.yaml"
                           CSV_TEMPLATE "components/left_hand_mk3/simmechanics2urdf_joints_configfile.csv")

generate_icub_simmechanics(YARP_ROBOT_NAME left_wrist_mk2
                           SIMMECHANICS_XML "components/left_wrist_mk2/SIM_L_WRIST.xml"
                           YAML_TEMPLATE "components/left_wrist_mk2/simmechanics2urdf_configfile.yaml"
                           CSV_TEMPLATE "components/left_wrist_mk2/simmechanics2urdf_joints_configfile.csv")

generate_icub_simmechanics(YARP_ROBOT_NAME iCubGazeboV2_5_plus
                          SIMMECHANICS_XML "icub2_5/ICUB_2-5_plus_BB_SIM_MODEL.xml"
                          YAML_TEMPLATE "icub2_5/ICUB_2-5_BB_simmechanics_options.yaml.in"
                          CSV_TEMPLATE "icub2_5/ICUB_2-5_BB_joint_parameters.csv.in"
                          INCREASE_INERTIA_FOR_GAZEBO ICUB_PLUS)

generate_icub_simmechanics(YARP_ROBOT_NAME iCubGenova04_plus
                          SIMMECHANICS_XML "icub2_5/ICUB_2-5_plus_BB_SIM_MODEL.xml"
                          YAML_TEMPLATE "icub2_5/ICUB_2-5_BB_simmechanics_options.yaml.in"
                          CSV_TEMPLATE "icub2_5/ICUB_2-5_BB_joint_parameters.csv.in"
                          ICUB_PLUS)

generate_icub_simmechanics(YARP_ROBOT_NAME iCubGenova02
                           SIMMECHANICS_XML "icub2_5/ICUB_2-5_plus_BB_SIM_MODEL.xml"
                           YAML_TEMPLATE "icub2_5/ICUB_2-5_BB_simmechanics_options.yaml.in"
                           CSV_TEMPLATE "icub2_5/ICUB_2-5_BB_joint_parameters.csv.in"
                           ICUB_PLUS)

generate_icub_simmechanics(YARP_ROBOT_NAME iCubGenova04
                           SIMMECHANICS_XML "icub2_5/ICUB_2-5_BB_SIM_MODEL.xml"
                           YAML_TEMPLATE "icub2_5/ICUB_2-5_BB_simmechanics_options.yaml.in"
                           CSV_TEMPLATE "icub2_5/ICUB_2-5_BB_joint_parameters.csv.in")

generate_icub_simmechanics(YARP_ROBOT_NAME iCubDarmstadt01
                           SIMMECHANICS_XML "icub2_5/ICUB_2-5_BB_SIM_MODEL.xml"
                           YAML_TEMPLATE "icub2_5/ICUB_2-5_BB_simmechanics_options.yaml.in"
                           CSV_TEMPLATE "icub2_5/ICUB_2-5_BB_joint_parameters.csv.in"
                           NO_BACKPACK)

generate_icub_simmechanics(YARP_ROBOT_NAME iCubGenova01
                           SIMMECHANICS_XML "icub2_5/ICUB_2-5_BB_SIM_MODEL.xml"
                           YAML_TEMPLATE "icub2_5/ICUB_2-5_BB_simmechanics_options.yaml.in"
                           CSV_TEMPLATE "icub2_5/ICUB_2-5_BB_joint_parameters.csv.in"
                           NO_BACKPACK)
generate_icub_simmechanics(YARP_ROBOT_NAME iCubGenova09
                           SIMMECHANICS_XML "icub3/ICUB3_ALL_SIM_MODEL.xml"
                           YAML_TEMPLATE "icub3/ICUB_3_all_options.yaml"
                           CSV_TEMPLATE "icub3/ICUB_3_joint_all_parameters.csv")
generate_icub_simmechanics(YARP_ROBOT_NAME iCubErzelli02
                           SIMMECHANICS_XML "icub2_5/ICUB_2-5_BB_SIM_MODEL.xml"
                           YAML_TEMPLATE "icub2_5/ICUB_2-5_BB_simmechanics_options.yaml.in"
                           CSV_TEMPLATE "icub2_5/ICUB_2-5_BB_joint_parameters.csv.in")


add_custom_target(generate-models-simmechanics
                  ALL
                  DEPENDS ${model-simmechanics-generated-models})

# Copy the meshes in the meshes/simmechanics directory
add_custom_command(TARGET generate-models-simmechanics
                   POST_BUILD
                   COMMAND ${CMAKE_COMMAND} -E copy_directory "${CMAKE_CURRENT_SOURCE_DIR}/data/icub2_5/meshes" "${CMAKE_BINARY_DIR}/${BUILD_PREFIX}/meshes"
                   COMMAND ${CMAKE_COMMAND} -E copy_directory "${CMAKE_CURRENT_SOURCE_DIR}/data/icub2_5/conf" "${CMAKE_BINARY_DIR}/${BUILD_PREFIX}/conf"
                   COMMAND ${CMAKE_COMMAND} -E copy_directory "${CMAKE_CURRENT_SOURCE_DIR}/data/icub3/meshes" "${CMAKE_BINARY_DIR}/${BUILD_PREFIX}/meshes"
                   COMMAND ${CMAKE_COMMAND} -E copy_directory "${CMAKE_CURRENT_SOURCE_DIR}/data/icub3/conf" "${CMAKE_BINARY_DIR}/${BUILD_PREFIX}/conf_icub3"
                   COMMAND ${CMAKE_COMMAND} -E copy_directory "${CMAKE_CURRENT_SOURCE_DIR}/data/components/left_wrist_mk2/meshes" "${CMAKE_BINARY_DIR}/${BUILD_PREFIX}/meshes"
                   COMMAND ${CMAKE_COMMAND} -E copy_directory "${CMAKE_CURRENT_SOURCE_DIR}/data/components/left_wrist_mk2/conf" "${CMAKE_BINARY_DIR}/${BUILD_PREFIX}/conf_left_wrist_mk2"
                   COMMAND ${CMAKE_COMMAND} -E copy           "${CMAKE_CURRENT_SOURCE_DIR}/data/components/left_wrist_mk2/model.config" "${CMAKE_BINARY_DIR}/${BUILD_PREFIX}/robots/left_wrist_mk2/."
                   COMMAND ${CMAKE_COMMAND} -E copy_directory "${CMAKE_CURRENT_SOURCE_DIR}/data/components/left_hand_mk3/meshes" "${CMAKE_BINARY_DIR}/${BUILD_PREFIX}/meshes"
                   COMMAND ${CMAKE_COMMAND} -E copy_directory "${CMAKE_CURRENT_SOURCE_DIR}/data/components/left_hand_mk3/conf" "${CMAKE_BINARY_DIR}/${BUILD_PREFIX}/conf_left_hand_mk3"
                   COMMAND ${CMAKE_COMMAND} -E copy           "${CMAKE_CURRENT_SOURCE_DIR}/data/components/left_hand_mk3/model.config" "${CMAKE_BINARY_DIR}/${BUILD_PREFIX}/robots/left_hand_mk3/."
                   COMMENT "Copying Simmechanics meshes")

# Convert meshes from ASCII to binary
add_custom_command(TARGET generate-models-simmechanics
                   POST_BUILD
                   WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
                   COMMAND ruby ./scripts/convertSTL.rb ${CMAKE_BINARY_DIR}/${BUILD_PREFIX}/meshes/simmechanics/*.stl
                   COMMENT "Converting Simmechanics meshes to binary")
