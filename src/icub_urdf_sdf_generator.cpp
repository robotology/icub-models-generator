/**
 * Copyright  (C)  2014 CoDyCo Project
 * Author: Silvio Traversaro
 * website: http://www.codyco.eu
 */
 
#include <iCub/iDynTree/idyn2kdl_icub.h>

#include <iostream>
#include <cstdlib>

#include <iCub/iDyn/iDyn.h>
#include <iCub/iDyn/iDynBody.h>

#include <kdl/tree.hpp>

#include <kdl_format_io/urdf_export.hpp>

#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>

#include <tinyxml.h>

#include <boost/function.hpp>

#include <yarp/os/Property.h>
#include <yarp/os/Os.h>

#include "urdf_utils.h"

#define NAME "icub_urdf_sdf_generator"

using namespace urdf;

void printTree(boost::shared_ptr<const Link> link,int level = 0)
{
  level+=2;
  int count = 0;
  for (std::vector<boost::shared_ptr<Link> >::const_iterator child = link->child_links.begin(); child != link->child_links.end(); child++)
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

/**
 * 
 * 
 * @param head_version can be 1 or 2
 * @param legs_version can be 1 or 2
 * @param feet_version can be 1 or 2
 * @param simple_meshes if true uses the simple visualization meshes, if false uses the heavy detailed one
 */
bool generate_iCub_model(std::string iCub_name, std::string root_directory, int head_version, int legs_version, int feet_version, bool simple_meshes, std::string data_directory, double mass_epsilon, double inertia_epsilon)
{
    bool ft_feet;
    if( feet_version == 1 ) {
        ft_feet = false;
    } else {
        ft_feet = true;
    }
    
    std::string paris_directory = data_directory+"urdf_paris/";
    
    std::string paris_subdirectory;
    if( simple_meshes ) {
        paris_subdirectory = "icub_simple";
    } else {
        paris_subdirectory = "icub";
    }
    
    std::string urdf_general_directory = root_directory+"urdf/";
    std::string urdf_robot_directory = urdf_general_directory+iCub_name+"/";
    std::string filename_urdf = urdf_robot_directory+"icub.xml";
    std::string gazebo_model_directory = root_directory + "gazebo_models/";
    std::string gazebo_robot_model_directory = gazebo_model_directory+iCub_name+"/";
    std::string filename_urdf_gazebo = gazebo_robot_model_directory+"icub.xml";
    std::string filename_urdf_gazebo_conversion = gazebo_robot_model_directory+"icub_conversion.xml";
    std::string gazebo_uri_prefix = "model://"+iCub_name+"/";
    std::string gazebo_sdf_filename = gazebo_robot_model_directory+"icub.sdf";
    
    //Creating needed directories
    yarp::os::mkdir(root_directory.c_str());
    yarp::os::mkdir(urdf_general_directory.c_str());
    yarp::os::mkdir(urdf_robot_directory.c_str());
    yarp::os::mkdir(gazebo_model_directory.c_str());
    yarp::os::mkdir(gazebo_robot_model_directory.c_str());

    //////////////////////////////////
    //Generating urdf from iDyn
    //////////////////////////////////
    iCub::iDyn::version_tag icub_type;

    icub_type.head_version = head_version;
    icub_type.legs_version = legs_version;  
    
    
    iCub::iDyn::iCubWholeBody icub_idyn(icub_type);

    KDL::Tree icub_kdl;
    
    KDL::JntArray dummy1,dummy2;
    
    if( ! toKDL(icub_idyn,icub_kdl,dummy1,dummy2,iCub::iDynTree::SKINDYNLIB_SERIALIZATION,false,true) ) {
        std::cerr << "Fatal error in iDyn - KDL conversion" << std::endl;
        return false;
    }
    
    boost::shared_ptr<urdf::ModelInterface> urdf_idyn(new urdf::ModelInterface);
    
    std::cout << "iCub KDL::Tree: " << std::endl;
    std::cout << icub_kdl << std::endl;
    
    
    if( ! kdl_format_io::treeToUrdfModel(icub_kdl,"test_icub",*urdf_idyn) ) {
        std::cerr << "Fatal error in KDL - URDF conversion" << std::endl;
        return false;
    }
    
    
    //////////////////////////////////////////////////////////////////
    //// Getting meshes and limits from urdf paris files
    //////////////////////////////////////////////////////////////////
    boost::shared_ptr<urdf::ModelInterface> urdf_paris;
    
    std::string filename_urdf_paris = paris_directory+paris_subdirectory+"/icub.xml";

    std::ifstream t_um(filename_urdf_paris.c_str());
    std::stringstream buffer_um;
    buffer_um << t_um.rdbuf();
    urdf_paris = parseURDF(buffer_um.str());
    
    urdf_import_limits(urdf_idyn,urdf_paris);
    urdf_import_meshes(urdf_idyn,urdf_paris);
    
    /////////////////////////////////////////////////////////////////////////
    ///// Exporting "normal" urdf file
    /////////////////////////////////////////////////////////////////////////
    TiXmlDocument* xml_doc;
    
    xml_doc = exportURDF(urdf_idyn);
    
    if( ! xml_doc->SaveFile(filename_urdf) ) {
        std::cerr << "Fatal error in URDF xml saving filename " << filename_urdf  << std::endl;
        return false;
    }
   
    
    //////////////////////////////////////////////////////////////////////////
    ///// Creating gazebo file
    /////////////////////////////////////////////////////////////////////////
    if( ! urdf_gazebo_cleanup_regularize_masses(urdf_idyn,mass_epsilon,inertia_epsilon) ) { std::cerr << "Error in regularizing masses root " << std::endl; return false; }
    
    //////////////////////////////////////////////////////////////////////////
    ////// Creating URDF file coherent with gazebo workarounds for use in iDynTree
    /////////////////////////////////////////////////////////////////////////    
    xml_doc = exportURDF(urdf_idyn);

    if( ! xml_doc->SaveFile(filename_urdf_gazebo) ) {
        std::cerr << "Fatal error in URDF xml saving" << std::endl;
        return false;
    }

    
    if( ! urdf_gazebo_cleanup_remove_massless_root(urdf_idyn) ) { std::cerr << "Error in removing massless root " << std::endl; return false; }
    printTree(urdf_idyn->getRoot());
    printJoints(urdf_idyn);
    if( ! urdf_gazebo_cleanup_remove_frames(urdf_idyn) ) { std::cerr << "Error in removing frames " << std::endl; return false; } 
    if( ! urdf_gazebo_cleanup_transform_FT_sensors(urdf_idyn) ) { std::cerr << "Error in transforming FT junctions " << std::endl; return false; } 
    if( ! urdf_gazebo_cleanup_regularize_masses(urdf_idyn,mass_epsilon,inertia_epsilon) ) { std::cerr << "Error in regularizing masses root " << std::endl; return false; }
    if( ! urdf_gazebo_cleanup_add_model_uri(urdf_idyn,gazebo_uri_prefix) ) { std::cerr << "Error in adding model URIs " << std::endl; return false; } 
    
    xml_doc = exportURDF(urdf_idyn);

    if( ! xml_doc->SaveFile(filename_urdf_gazebo_conversion) ) {
        std::cerr << "Fatal error in URDF xml saving" << std::endl;
        return false;
    }
    
    //Ugly workaround, I know, probably can be avoided by directly using sdfformat library
    std::string gazebo_conversion_command = "gzsdf print " + filename_urdf_gazebo_conversion + " > " + gazebo_sdf_filename; 
    std::cout << "Running command: " << gazebo_conversion_command << std::endl;
    system(gazebo_conversion_command.c_str());
    //if( !system(gazebo_conversion_command.c_str()) ) { std::cerr << "Error in urdf - sdf conversion" << std::endl; return false; }
    
    //\todo: adding plugins to generated sdf
    
    return true;
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
    
    double mass_epsilon = 0.1;
    double inertia_epsilon = 0.01;
    
    if( opt.check("min_mass" )  ) mass_epsilon = opt.find("min_mass").asDouble();
    if( opt.check("min_inertia" )  ) inertia_epsilon = opt.find("min_inertia").asDouble();
    
    //Generating model for black iCub
    //                  robot_name     directory         head   legs   feet  meshes 
    if( !generate_iCub_model("iCubGenova01",output_directory, 2    , 2    , 2   , false , data_directory,mass_epsilon,inertia_epsilon) ) return EXIT_FAILURE;
    
    //Generating model for red iCub
    if( !generate_iCub_model("iCubGenova03",output_directory, 2    , 1    , 2   , false , data_directory,mass_epsilon,inertia_epsilon) ) return EXIT_FAILURE;
        
    std::cerr << "iCub model files successfully created" << std::endl;
    
    return EXIT_SUCCESS;
}
