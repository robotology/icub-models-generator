/**
 * Copyright  (C)  2014 CoDyCo Project
 * Author: Silvio Traversaro
 * website: http://www.codyco.eu
 */

#include "urdf_sdf_from_dh_utils.h"

int main(int argc, char* argv[])
{
    bool status = true;

    yarp::os::Property opt;

    opt.fromCommand(argc,argv);

    if( opt.check("help") ||
        !opt.check("data_directory") ||
        !opt.check("YARP_ROBOT_NAME") ||
        !opt.check("outputfile") )
    {
        std::cerr << "Utility for generating URDF and SDF models for iCub robots from DH parameters hardcoded in iKin/iDyn/iDynTree libraries." << std::endl;
        std::cerr << "Creating one URDF with data from CAD, and a SDF directory with parameters modified to work in Gazebo (with a URDF that matches the modified simulated model)" << std::endl;
        std::cerr << "Usage: \t icub_urdf_sdf_from_dh_generator --data_directory ./data/ --output_directory ./icub-models/" << std::endl;
        std::cerr << "Additional options: --min_mass and --min_inertia are used to select the minimum mass and minimum inertia to a give to a link in Gazebo model (default: mass 0.1, inertia: 0.01)" << std::endl;
        std::cerr << "\t\t --headV1/headV2 : specify the version of the head part" << std::endl;
        std::cerr << "\t\t --armsV1/armsV2 : specify the version of the arm parts" << std::endl;
        std::cerr << "\t\t --legsV1/legsV2 : specify the version of the leg parts" << std::endl;
        std::cerr << "\t\t --feetV1/feetV2 : specify the version of the leg parts" << std::endl;
        std::cerr << "\t\t --YARP_ROBOT_NAME : specify the YARP_ROBOT_NAME of the created robot." << std::endl;
        std::cerr << "\t\t --iCubParis02 : specify if the generated model is iCubParis02" << std::endl;
        std::cerr << "\t\t --outputfile  : specify the path of the generated file" << std::endl;

        return EXIT_FAILURE;
    }

    std::string output_file = opt.find("outputfile").asString();
    std::string data_directory = opt.find("data_directory").asString();

    if( *(data_directory.rbegin()) != '/' ) {
        data_directory = data_directory + "/";
    }

    double mass_epsilon = 0.1;
    double inertia_epsilon = 0.01;

    if( opt.check("min_mass" )  ) mass_epsilon = opt.find("min_mass").asDouble();
    if( opt.check("min_inertia" )  ) inertia_epsilon = opt.find("min_inertia").asDouble();

    // Parse part info
    int version_head = -1;
    if( opt.check("headV2") )
    {
        version_head = 2;
    }
    if( opt.check("headV1") )
    {
        version_head = 1;
    }
    if( version_head < 0 )
    {
        std::cerr << "icub_urdf_sdf_from_dh_generator : missing head version";
    }


    int version_arms = -1;
    if( opt.check("armsV2") )
    {
        version_arms = 2;
    }
    if( opt.check("armsV1") )
    {
        version_arms = 1;
    }
    if( version_arms < 0 )
    {
        std::cerr << "icub_urdf_sdf_from_dh_generator : missing arms version";
    }

    int version_legs = -1;
    if( opt.check("legsV2") )
    {
        version_legs = 2;
    }
    if( opt.check("legsV1") )
    {
        version_legs = 1;
    }
    if( version_legs < 0 )
    {
        std::cerr << "icub_urdf_sdf_from_dh_generator : missing legs version";
    }

    int version_feet = -1;
    if( opt.check("feetV2") )
    {
        version_feet = 2;
    }
    if( opt.check("feetV1") )
    {
        version_feet = 1;
    }
    if( version_feet < 0 )
    {
        std::cerr << "icub_urdf_sdf_from_dh_generator : missing feet version";
    }

    // iCubParis02 is a special robot, and needs a special options
    bool isiCubParis02 = opt.check("iCubParis02");


    //Generating model for black iCub
    //                          robot_name     directory    head   legs   feet    Paris02  gazeboSim
    bool simple_meshes = false;
    boost::shared_ptr<urdf::ModelInterface> model;
    bool ok = generate_iCub_urdf_model(opt.find("YARP_ROBOT_NAME").asString(),
                                       version_head , version_legs , version_feet,
                                       isiCubParis02, false, simple_meshes ,
                                       data_directory,mass_epsilon,inertia_epsilon,false,model,output_file);

    if( !ok )
    {
        std::cerr << "Failure in generating URDF model for " << opt.find("YARP_ROBOT_NAME").asString() << std::endl;
        return EXIT_FAILURE;
    }
    else
    {
        return EXIT_SUCCESS;
    }
}

