# icub-model-generator

Resources and programs to generated models (URDF,SDF) of the iCub robot.

**Note: this repository is meant to streamline the process of producing iCub URDF/SDF models by iCub mantainers. It is not meant to be used directly by users of iCub. For offical info on the kinematic parameters of the iCub, please see [the documentation in iCub's wiki](http://wiki.icub.org/wiki/ICubForwardKinematics).**

There are currently two different pipelines to generated the models. 

The generation system from Denavit Hartenberg parametsrs based on data on robot models hardcoded in the iKin/iDyn source code and meshes postprocessed by ISIR at UPMC, Paris, available in `dh` directory.

The generation system from actual CAD models converted to simmechanics file, described in http://wiki.icub.org/wiki/Creo_Mechanism_to_URDF , available in the `simmechanics` directory.

Both generation pipelines are still a work in progress, and several issues need to be properly solved, for example: 
* [Meshes are not properly handled #28](https://github.com/robotology-playground/icub-model-generator/issues/28)
* [Gazebo SDF Models are not generated #25](https://github.com/robotology-playground/icub-model-generator/issues/25)
* [Hands and eyes are not generated #29](https://github.com/robotology-playground/icub-model-generator/issues/29)

## Generated models 

| `YARP_ROBOT_NAME` | Pipeline     | Notes                 |
|:-----------------:|:------------:|:---------------------:|
| `iCubDarmstadt01` | simmechanics | v2.5 without backpack |
| `iCubGenova01`    | simmechanics | v2.5 without backpack | 
| `iCubGenova02`    | simmechanics | v2.5   with backpack  | 
| `iCubGenova03`    | dh | v2 with legs v1 and feet v2.5   | 
| `iCubLisboa01`    | dh           | v1 with head v2       |
| `iCubNancy01`     | dh           | v2.5 with arms v2     |
| `iCubParis01`     | dh           | v1 with feet v2.5     | 
| `iCubParis02`     | dh           | v2 with feet v2.5     | 
