/**
 * Copyright  (C)  2014 CoDyCo Project
 * Author: Silvio Traversaro
 * website: http://www.codyco.eu
 */

#include "urdf_sdf_from_dh_utils.h"

void printTree(urdf::LinkSharedPtr link,int level = 0)
{
  level+=2;
  int count = 0;
  for (std::vector< urdf::LinkSharedPtr >::const_iterator child = link->child_links.begin(); child != link->child_links.end(); child++)
  {
    if (*child)
    {
      for(int j=0;j<level;j++) std::cout << "  "; //indent
      std::cout << "child(" << (count++)+1 << "):  " << (*child)->name  << " with joint: " << (*child)->parent_joint->name << std::endl;
      // first grandchild
      printTree(*child,level);
    }
    else
    {
      for(int j=0;j<level;j++) std::cout << " "; //indent
      std::cout << "root link: " << link->name << " has a null child!" << *child << std::endl;
    }
  }

}

void printJoints(boost::shared_ptr<urdf::ModelInterface> urdf)
{
    std::cout << "printing " << urdf->joints_.size() << " size" << std::endl;
    for(std::map<std::string,boost::shared_ptr<urdf::Joint> >::const_iterator it = urdf->joints_.begin(); it != urdf->joints_.end(); it++ ) {
        std::cout << it->first << " : " << it->second->name << std::endl;
    }
}

int main(int argc, char* argv[])
{
    bool status = true;

    yarp::os::Property opt;

    opt.fromCommand(argc,argv);

    if( opt.check("help") || !opt.check("output_directory") || !opt.check("data_directory") ) {
        std::cerr << "Utility for generating URDF and SDF models for iCub robots" << std::endl;
        std::cerr << "Creating one URDF with data from CAD, and a SDF directory with parameters modified to work in Gazebo (with a URDF that matches the modified simulated model)" << std::endl;
        std::cerr << "Usage: \t icub_urdf_sdf_generator --data_directory ./data/ --output_directory ./icub-models/" << std::endl;
        std::cerr << "Additional options: --min_mass and --min_inertia are used to select the minimum mass and minimum inertia to a give to a link in Gazebo model (default: mass 0.1, inertia: 0.01)" << std::endl;
        return EXIT_FAILURE;
    }

    std::string output_directory = opt.find("output_directory").asString();
    std::string data_directory = opt.find("data_directory").asString();

    if( *(output_directory.rbegin()) != '/' ) {
        output_directory = output_directory + "/";
    }

    if( *(data_directory.rbegin()) != '/' ) {
        data_directory = data_directory + "/";
    }

    double mass_epsilon = 0.1;
    double inertia_epsilon = 0.01;

    if( opt.check("min_mass" )  ) mass_epsilon = opt.find("min_mass").asDouble();
    if( opt.check("min_inertia" )  ) inertia_epsilon = opt.find("min_inertia").asDouble();

    //Generate model database
    std::vector<std::string> robot_names;

    //Generating model for black iCub
    //                          robot_name     directory    head   legs   feet    Paris02  gazeboSim
    bool simple_meshes = false;
    if( !generate_iCub_model("iCubGenova01",output_directory, 2    , 2    , 2   , false, false, simple_meshes , data_directory,mass_epsilon,inertia_epsilon) ) return EXIT_FAILURE;
    robot_names.push_back("iCubGenova01");

    //Generating model for red iCub
    if( !generate_iCub_model("iCubGenova03",output_directory, 2    , 1    , 2   , false, false, simple_meshes , data_directory,mass_epsilon,inertia_epsilon) ) return EXIT_FAILURE;
    robot_names.push_back("iCubGenova03");

    if( !generate_iCub_model("iCubParis01",output_directory, 1    , 1    , 2   , false, false, simple_meshes , data_directory,mass_epsilon,inertia_epsilon) ) return EXIT_FAILURE;
    robot_names.push_back("iCubParis01");

    //Generating model for red iCub
    if( !generate_iCub_model("iCubParis02",output_directory, 2    , 1    , 2   , true, false,  simple_meshes , data_directory,mass_epsilon,inertia_epsilon) ) return EXIT_FAILURE;
    robot_names.push_back("iCubParis02");

    //Generating model for Darmastad iCub
    if( !generate_iCub_model("iCubDarmstadt01",output_directory, 2    , 2    , 2  , false, false,  simple_meshes , data_directory,mass_epsilon,inertia_epsilon) ) return EXIT_FAILURE;
    robot_names.push_back("iCubDarmstad01");

    //Generating model for icubGazeboSim
    if( !generate_iCub_model("icubGazeboSim",output_directory, 2    , 1    , 2   , false, true, simple_meshes , data_directory,mass_epsilon,inertia_epsilon) ) return EXIT_FAILURE;
    robot_names.push_back("icubGazeboSim");

    //Generating model for icubGazeboSim
    if( !generate_iCub_model("icubGazeboSimNoFT",output_directory, 2    , 1    , 2   , false, true, simple_meshes , data_directory,mass_epsilon,inertia_epsilon,true) ) return EXIT_FAILURE;
    robot_names.push_back("icubGazeboSimNoFT");


    std::cerr << "iCub model files successfully created" << std::endl;

    std::cerr << "Generating gazebo database" << std::endl;
    generate_gazebo_database(robot_names,output_directory);

    return EXIT_SUCCESS;
}

