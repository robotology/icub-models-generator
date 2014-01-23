/**
 * Copyright  (C)  2013 CoDyCo Project
 * Author: Silvio Traversaro
 * website: http://www.codyco.eu
 */
 
#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>

#include <kdl/treefksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>

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

///< \todo TODO add support for deleting a long chain of null link connected by fixed base

bool deleteLink(boost::shared_ptr<urdf::ModelInterface> urdf_input, std::string link_to_delete) 
{
    if( link_to_delete != urdf_input->getRoot()->name ) {
        //deleting normal link
        
        //delete links (remove from global and from parent (also for joint)
        boost::shared_ptr<urdf::Link> link_sptr = urdf_input->links_[link_to_delete];
        if( link_sptr->child_links.size() != 0 ) { std::cerr << "deleteLink error: tryng to delete a link with a child" << std::endl; return false;}
        boost::shared_ptr<urdf::Joint> joint_sptr = link_sptr->parent_joint;
        boost::shared_ptr<urdf::Link> parent_link = link_sptr->getParent();
        
        //remove links from cparent_link->child of parent
        for(int i=0; i < parent_link->child_links.size(); i++ ) {
            if( parent_link->child_links[i]->name == link_sptr->name ) {
                parent_link->child_links.erase( parent_link->child_links.begin()+i);
                parent_link->child_joints.erase( parent_link->child_joints.begin()+i);
            }
        }
    
        urdf_input->links_.erase(link_sptr->name);
        urdf_input->joints_.erase(joint_sptr->name);
    } else {
        //deleting root link
        
        boost::shared_ptr<urdf::Link> link_sptr = urdf_input->links_[link_to_delete];
        
        if( link_sptr->child_links.size() != 1 ) { std::cerr << "deleteLink error: tryng to delete a root link with more than a child" << std::endl; return false; }
        
        boost::shared_ptr<urdf::Joint> joint_sptr = link_sptr->child_joints[0];
        
        boost::shared_ptr<urdf::Link> new_root = link_sptr->child_links[0];
        
        urdf_input->root_link_ = new_root;
        
        urdf_input->links_.erase(link_sptr->name);
        urdf_input->joints_.erase(joint_sptr->name);

    }
    
    return true;
    
}

bool deleteLinks(boost::shared_ptr<urdf::ModelInterface> urdf_input, std::vector<std::string> linksToDelete) 
{
    for(int i=0; i < linksToDelete.size(); i++ ) {
        if( !deleteLink(urdf_input,linksToDelete[i]) ) return false;
    }
    return true;
}


int main(int argc, char* argv[])
{
    std::vector<std::string> linksToDelete;
    
    bool status = true;
        bool ret = true;
        
    bool verbose = true;
    
    
    yarp::os::Property opt;
    opt.fromCommand(argc,argv);
    
    double mass_epsilon = 0.005;
    double inertia_epsilon = 0.0001;
    
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
        if( !(urdf_input->getRoot()->inertial) ) {
            if( !deleteLink(urdf_input,urdf_input->getRoot()->name ) ) return EXIT_FAILURE;
        }
    
        std::cout << "urdf_gazebo_cleanup: Rule 0 applied successfully" << std::endl;
    }
    

    //Iterate over the links of urdf_input, and copy the mesh
    std::vector<boost::shared_ptr<Link> > input_links;
    
    if( !opt.check("no_rule1") ) {
            input_links.clear();
    urdf_input->getLinks(input_links);
    
    std::cout << "urdf_gazebo_cleanup: : Found " << input_links.size() << " links in input URDF " << std::endl;
    for(int i=0; i < input_links.size(); i++ )
    {
        //Rule 1
        double mass;
        double Ixx;
        double Iyy;
        double Izz;
        
        if( input_links[i]->inertial ) {
            mass = input_links[i]->inertial->mass;
            Ixx = input_links[i]->inertial->ixx;
            Iyy = input_links[i]->inertial->iyy;
            Izz = input_links[i]->inertial->izz;
        }
       
        int nrOfChildrens = input_links[i]->child_links.size();
        
        if( nrOfChildrens == 0 && mass == 0 && Ixx == 0.0 & Iyy == 0.0 && Izz == 0.0 && input_links[i]->parent_joint->type == urdf::Joint::FIXED ) {
            linksToDelete.push_back(input_links[i]->name);
        }
    }
    
    if( !deleteLinks(urdf_input,linksToDelete) ) return EXIT_FAILURE;
    linksToDelete.resize(0);
    
    std::cout << "urdf_gazebo_cleanup: Rule 1 applied successfully" << std::endl;
    }

    
    if( !opt.check("no_rule2") ) {
            input_links.clear();
    urdf_input->getLinks(input_links);
    
    for(int i=0; i < input_links.size(); i++ )
    {  
        //Rule 2
        int nrOfChildrens = input_links[i]->child_links.size();
       
        if( input_links[i]->parent_joint->type == urdf::Joint::FIXED && nrOfChildrens > 0) {
            input_links[i]->parent_joint->type = urdf::Joint::REVOLUTE;
            if( input_links[i]->parent_joint->limits ) {
                input_links[i]->parent_joint->limits->clear();
            } else {
                input_links[i]->parent_joint->limits.reset(new JointLimits());
            }
        }
        
    }
    std::cout << "urdf_gazebo_cleanup: Rule 2 applied successfully" << std::endl;
    }
   
   
    if( !opt.check("no_rule3") ) {
            input_links.clear();
    urdf_input->getLinks(input_links);
    for(int i=0; i < input_links.size(); i++ )
    {
     //Rule 3
        if( input_links[i]->parent_joint->type != urdf::Joint::FIXED ) {
            if( input_links[i]->inertial->mass <= mass_epsilon ) {
                input_links[i]->inertial->mass = mass_epsilon;
            }
            if( input_links[i]->inertial->ixx <= inertia_epsilon ) {
                input_links[i]->inertial->ixx = inertia_epsilon;
            }
            if( input_links[i]->inertial->iyy <= inertia_epsilon ) {
                input_links[i]->inertial->iyy = inertia_epsilon;
            }
            if( input_links[i]->inertial->izz <= inertia_epsilon ) {
                input_links[i]->inertial->izz = inertia_epsilon;
            }
        }
    }
        std::cout << "urdf_gazebo_cleanup: Rule 3 applied successfully" << std::endl;
        
    }

    if( !opt.check("no_rule4") ) {
        
    input_links.clear();
    urdf_input->getLinks(input_links);
    
    for(int i=0; i < input_links.size(); i++ )
    {
        //Rule 4
        for(int j=0; j < input_links[i]->visual_array.size(); j++ ) {
            if( input_links[i]->visual_array[j]->geometry->type == Geometry::MESH ) {
                (static_pointer_cast<urdf::Mesh>(input_links[i]->visual_array[j]->geometry))->filename = rule4_prefix + (static_pointer_cast<urdf::Mesh>(input_links[i]->visual_array[j]->geometry))->filename;         
            }
        }
        
        for(int j=0; j < input_links[i]->collision_array.size(); j++ ) {
            if( input_links[i]->collision_array[j]->geometry->type == Geometry::MESH ) {
                (static_pointer_cast<urdf::Mesh>(input_links[i]->collision_array[j]->geometry))->filename = rule4_prefix + (static_pointer_cast<urdf::Mesh>(input_links[i]->collision_array[j]->geometry))->filename;         
            }
        }
        std::cout << "urdf_gazebo_cleanup: Rule 4 applied successfully with prefix " <<  rule4_prefix << " on link " << input_links[i]->name << " ( i : " << i << " ) "  << std::endl;
        
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
