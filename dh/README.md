DH iCub Model Generator
==================

This software repository is still in alpha stage, not ready for normal use. 

Resources (./data/ folder) and programs (./src/) folder to generated models (URDF,SDF) of the iCub robot from DH parameters.

Installation
------------

##### Dependencies
- [YARP](https://github.com/robotology/yarp)
- [ICUB](https://github.com/robotology/icub-main)
- [iDynTree](https://github.com/robotology-playground/iDynTree)
- [SDFormat](https://bitbucket.org/osrf/sdformat)

The easiest way to install this repository with almost all its dependencies is to use the [codyco-superbuild](https://github.com/robotology/codyco-superbuild). The SDFormat dependency is not 
installed by the codyco-superbuild, but it should be already if you have installed the development files Gazebo (the
one that you need to compile [gazebo-yarp-plugins](https://github.com/robotology/gazebo-yarp-plugins).

 
Example
-------

### Model generation
After compiling and installing this repository, you can run:
```
icub_urdf_sdf_generator --data_directory /path/to/codyco-superbuild/data/ --output_directory /path/to/icub-models/
```
Where the `/path/to/codyco-superbuild/data/ ` directory is the path to the data subdirectory of this repository. 
`/path/to/icub-models/` is an arbitrary directory where you want to store the generated URDF and SDF models for the various iCubs (iCubGenova01, iCubGenova03, etc, etc).
The models available in `/path/to/icub-models/icub-models/urdf` are the one extracted directly from the iDyn models. 

### Use of generated models in Gazebo
In the directory `/path/to/icub-models/icub-models/gazebo-models` you will find the SDF models generated for use with the Gazebo simulator, and the URDF models that match the kinematic and dynamic parameters of the SDF models. As the `icub_urdf_sdf_generator` generates also the proper configuration files to create a Gazebo model database, you can simply add `/path/to/icub-models/icub-models/gazebo-models` to the enviroment variable `GAZEBO_MODEL_PATH`.

**Warning: currently to use the database of generated models of the iCub you should also install the [icub-gazebo](https://github.com/robotology-playground/icub-gazebo) repository, as the generated models take configuration files and meshes from the `iCub` model present in the `icub-gazebo` repository.**

