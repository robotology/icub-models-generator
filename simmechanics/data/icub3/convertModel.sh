simmechanics_to_urdf SIM_ICUB3_UPPERBODY.xml --yaml icub3-upperbody-options.yaml --outputfile model.urdf
gz sdf -p model.urdf > iCub3Upperbody.sdf
