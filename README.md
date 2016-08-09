# icub-model-generator

Resources and programs to generated models (URDF,SDF) of the iCub robot. 

There are currently two different pipelines to generated the models. 

The generation system from Denavit Hartenberg parametsrs based on data on robot models hardcoded in the iKin/iDyn source code and meshes postprocessed by ISIR at UPMC, Paris, available in `dh` directory.

The generation system from actual CAD models converted to simmechanics file, described in http://wiki.icub.org/wiki/Creo_Mechanism_to_URDF , available in the `simmechanics` directory.

