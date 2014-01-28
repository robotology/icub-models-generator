iCubModelGenerator
==================

Resources and programs to generated models (URDF,SDF) of the iCub robot

This folder should contain models of iCub - for example descriptions in the standard URDF format.

The folder urdf_paris contains the urdf models created at UPMC - ISIR by Joseph Salini ( https://github.com/XDE-ISIR/XDE-Resources ).

The folder urdf_idyn contains the urdf models extracted from the iDyn library, and the code necessary to create them.

The folder urdf_utils contains the programs used to generated the models combining meshes, limits and parameters into one urdf file.

Example
-------
Consider a directory where that contains the urdf_paris/ and icub_gazebo_data/ directory:
```
icub_urdf_from_iDyn --headV2 --legsV2 --output icubV2_raw.xml
urdf_get_meshes urdf_paris/icub/icub.xml icubV2_raw.xml icubV2_meshes.xml
urdf_get_limits urdf_paris/icub/icub.xml icubV2_meshes.xml icubV2_meshes_limits.xml
urdf_gazebo_cleanup --input icubV2_meshes_limits.xml --output icubV2_for_gazebo.xml
urdf_gazebo_cleanup --no_rule0 --no_rule1 --no_rule2 --no_rule4 --input icubV2_meshes_limits.xml --output icubV2_for_idyntree.xml
```
In this way two files where created, one suitable for gazebo conversion (icubV2_for_gazebo.xml) and one for loading in iDynTree (icubV2_for_idyntree.xml).
To create a proper sdf, create a directory with the necessary files from urdf_paris and icub_gazebo/ directories.