 simmechanics_to_urdf ICUB3_UPPERBODY_SIM_MODEL.xml  --csv ICUB_3_joint_upperbody_parameters.csv --yaml ICUB_3_upperbody_options.yaml --outputfile model.urdf
gz sdf -p model.urdf > iCub3Upperbody.sdf


simmechanics_to_urdf ICUB3_L_LEG_SIM_MODEL.xml  --csv ICUB_3_joint_leftleg_parameters.csv --yaml ICUB_3_leftleg_options.yaml --outputfile model.urdf
gz sdf -p model.urdf > iCub3LeftLeg.sdf