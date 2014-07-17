icub-model-generator
==================

This software repository is still in alpha stage, not ready for normal use. 

Resources (./data/ folder) and programs (./src/) folder to generated models (URDF,SDF) of the iCub robot .

Installation
------------

##### Dependencies
- [YARP](https://github.com/robotology/yarp)
- [ICUB](https://github.com/robotology/icub-main)
- [iDynTree](https://github.com/robotology-playground/iDynTree)

The easiest way to install this repository with all its dependencies is to use the [codyco-superbuild](https://github.com/robotology/yarp).
 
Example
-------
After compiling and installing this repository, you can run:
```
icub_urdf_sdf_generator --data_directory ./data/ --output_directory ./icub-models/
```
In icub-models, the URDF and SDF models for the various iCubs (iCubGenova01, iCubGenova03, etc, etc) will be created.
