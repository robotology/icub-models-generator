/**
 * Copyright  (C)  2013 CoDyCo Project
 * Author: Silvio Traversaro
 * website: http://www.codyco.eu
 */
 
#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>

#include <kdl/treefksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>

#include "urdf_utils.h"

#include <kdl_format_io/urdf_import.hpp>

#include <iostream>
#include <sstream>
#include <fstream>

#include <tinyxml.h>

#include <yarp/os/Property.h>

using namespace kdl_format_io;
using namespace KDL;
using namespace std;

using namespace urdf;
using namespace boost;




int main(int argc, char* argv[])
{
    
    bool status = true;
        bool ret = true;
        
    bool verbose = true;
    
    
    yarp::os::Property opt;
    opt.fromCommand(argc,argv);
    
    double mass_epsilon = 0.1;
    double inertia_epsilon = 0.01;
    
    if( opt.check("min_mass" )  ) mass_epsilon = opt.find("min_mass").asDouble();
    if( opt.check("min_inertia" )  ) inertia_epsilon = opt.find("min_inertia").asDouble();
    
        std::string rule4_prefix =  "model://icub/";
    
    if( opt.check("help") || !opt.check("input") || !opt.check("output") )
    {
        std::cerr << "Usage: \t urdf_gazebo_cleanup --input urdf_input.xml --output urdf_output.xml" << std::endl;
        std::cerr << "This tool is used to make a urdf file suitable for conversion in SDF format used by Gazebo Simulator" << std::endl;
        std::cerr << "The following modification are made: " << std::endl;
        std::cerr << "  * (Rule 0) If a link without inertia is in the root, remove it (to handle base_link as commonly found in humanoid models for REP 120)" << std::endl;
        std::cerr << "  * (Rule 1) Links with zero mass, with no childrens and connected to their parent with a fixed joint are removed" << std::endl;
        std::cerr << "  * (Rule 2) The fixed joint connecting two non-leaf links (typically used in iCub for modeling FT sensors) are transformed in rotational joint with no range (to address this issue https://bitbucket.org/osrf/gazebo/issue/618/add-option-to-disable-auto-fixed-joint)" << std::endl;
        std::cerr << "  * (Rule 3) For the remaining  (also if the previous steps are not actually exectuted)  linkswith zero mass or zero diagonal inertia elements a small mass ( " << mass_epsilon << " ) and diagonal inertia elements " << inertia_epsilon << "  is added, to avoid stability problems in simulation" << std::endl;
        std::cerr << "  * (Rule 4) add a model:// uri prefix to all meshes uris" << std::endl;
        std::cerr << "Additional options: --no_rule0 avoid applyng rule0, --no_rule1 avoid applyng rule1 and so on" << std::endl;
        std::cerr << "Additional options: --min_mass and --min_inertia are used to select the minimum mass to a give to an object in Rule 3" << std::endl;
        std::cerr << "Additional options: --rule4_prefix is used to specify the prefix used in rule 4 (default: " << rule4_prefix << " ) " << std::endl;
        return EXIT_FAILURE;
    }
    
    if( opt.check("rule4_prefix") ) rule4_prefix = opt.find("rule4_prefix").asString().c_str();
    
  
    
    std::string file_name_urdf_input(opt.find("input").asString().c_str());
    std::string file_name_urdf_output(opt.find("output").asString().c_str());


    boost::shared_ptr<urdf::ModelInterface> urdf_input;
    
    std::ifstream t_im(file_name_urdf_input.c_str());
    std::stringstream buffer_im;
    buffer_im << t_im.rdbuf();
    urdf_input = parseURDF(buffer_im.str());
    if( !ret ) {
        std::cerr << "urdf_gazebo_cleanup: Fatal error in URDF xml parsing" << std::endl;
        return EXIT_FAILURE;
    }
    
    if( !opt.check("no_rule0") ) {
        urdf_gazebo_cleanup_remove_massless_root(urdf_input);
    
        std::cout << "urdf_gazebo_cleanup: Rule 0 applied successfully" << std::endl;
    }
    

    
    if( !opt.check("no_rule1") ) {
        if( urdf_gazebo_cleanup_remove_frames(urdf_input) ) {  
            std::cout << "urdf_gazebo_cleanup: Rule 1 applied successfully" << std::endl;
        } else {
            std::cout << "urdf_gazebo_cleanup: Rule 1 failed" << std::endl;
            return EXIT_FAILURE;
        }
    }

    
    if( !opt.check("no_rule2") ) {
        if( urdf_gazebo_cleanup_transform_FT_sensors(urdf_input) ) {  
            std::cout << "urdf_gazebo_cleanup: Rule 2 applied successfully" << std::endl;
        } else {
            std::cout << "urdf_gazebo_cleanup: Rule 2 failed" << std::endl;
            return EXIT_FAILURE;
        }    
    }
   
   
    if( !opt.check("no_rule3") ) {
        if( urdf_gazebo_cleanup_regularize_masses(urdf_input,mass_epsilon,inertia_epsilon) ) {  
            std::cout << "urdf_gazebo_cleanup: Rule 3 applied successfully" << std::endl;
        } else {
            std::cout << "urdf_gazebo_cleanup: Rule 3 failed" << std::endl;
            return EXIT_FAILURE;
        }            
    }

    if( !opt.check("no_rule4") ) {
        if( urdf_gazebo_cleanup_add_model_uri(urdf_input,rule4_prefix) ) {  
            std::cout << "urdf_gazebo_cleanup: Rule 4 applied successfully" << std::endl;
        } else {
            std::cout << "urdf_gazebo_cleanup: Rule 4 failed" << std::endl;
            return EXIT_FAILURE;
        }           
    }
    
    
    
   
    TiXmlDocument* xml_doc;
    
    xml_doc = exportURDF(urdf_input);
    
    if( ! xml_doc->SaveFile(file_name_urdf_output) ) {
        std::cerr << "urdf_gazebo_cleanup: Fatal error in URDF xml saving" << std::endl;
        return EXIT_FAILURE;
    }
    
    std::cerr << "urdf_gazebo_cleanup: URDF file successfully created" << std::endl;
    
    return EXIT_SUCCESS;
}
