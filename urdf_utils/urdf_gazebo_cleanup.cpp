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

using namespace kdl_format_io;
using namespace KDL;
using namespace std;

using namespace urdf;
using namespace boost;

#define mass_epsilon 0.001
#define inertia_epsilon 0.001

///< \todo TODO add support for deleting a long chain of null link connected by fixed base

bool deleteLink(boost::shared_ptr<urdf::ModelInterface> urdf_input, std::string link_to_delete) 
{
    //delete links (remove from global and from parent (also for joint)
    boost::shared_ptr<urdf::Link> link_sptr = urdf_input->links_[link_to_delete];
    if( link_sptr->child_links.size() != 0 ) return false;
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
    
    if( argc != 3 ) {
        std::cerr << "Usage: \t urdf_gazebo_cleanup urdf_input.xml urdf_output.xml" << std::endl;
        std::cerr << "This tool is used to make a urdf file suitable for conversion in SDF format used by Gazebo Simulator" << std::endl;
        std::cerr << "The following modification are made: " << std::endl;
        std::cerr << "  * (Rule 1) Links with zero mass, with no childrens and connected to their parent with a fixed joint are removed" << std::endl;
        std::cerr << "  * (Rule 2) The fixed joint connecting two non-leaf links (typically used in iCub for modeling FT sensors) are transformed in rotational joint with no range (to address this issue https://bitbucket.org/osrf/gazebo/issue/618/add-option-to-disable-auto-fixed-joint)" << std::endl;
        std::cerr << "  * (Rule 3) For the remaining links with zero mass or zero diagonal inertia elements a small mass ( " << mass_epsilon << " ) and diagonal inertia elements " << inertia_epsilon << "  is added, to avoid stability problems in simulation" << std::endl;
        return EXIT_FAILURE;
    }
    
    std::string file_name_urdf_input(argv[1]);
    std::string file_name_urdf_output(argv[2]);


    boost::shared_ptr<urdf::ModelInterface> urdf_input;
    
    std::ifstream t_im(file_name_urdf_input.c_str());
    std::stringstream buffer_im;
    buffer_im << t_im.rdbuf();
    urdf_input = parseURDF(buffer_im.str());
    if( !ret ) {
        std::cerr << "urdf_gazebo_cleanup: Fatal error in URDF xml parsing" << std::endl;
        return EXIT_FAILURE;
    }

    //Iterate over the links of urdf_input, and copy the mesh
    std::vector<boost::shared_ptr<Link> > input_links;
    
    urdf_input->getLinks(input_links);
    
    std::cout << "urdf_gazebo_cleanup: : Found " << input_links.size() << " links in input URDF " << std::endl;
    for(int i=0; i < input_links.size(); i++ )
    {
        //Rule 1
        double mass = input_links[i]->inertial->mass;
        double Ixx = input_links[i]->inertial->ixx;
        double Iyy = input_links[i]->inertial->iyy;
        double Izz = input_links[i]->inertial->izz;
       
        int nrOfChildrens = input_links[i]->child_links.size();
        
        if( input_links[i]->parent_joint->type == urdf::Joint::FIXED && nrOfChildrens == 0 && mass == 0 && Ixx == 0.0 & Iyy == 0.0 && Izz == 0.0 ) {
            linksToDelete.push_back(input_links[i]->name);
        }
    }
    
    deleteLinks(urdf_input,linksToDelete);
    linksToDelete.resize(0);
    
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
        
        //Rule 3
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
    std::cout << "urdf_gazebo_cleanup: Rule 2 and 3 applied successfully" << std::endl;
    
    TiXmlDocument* xml_doc;
    
    xml_doc = exportURDF(urdf_input);
    
    if( ! xml_doc->SaveFile(file_name_urdf_output) ) {
        std::cerr << "urdf_gazebo_cleanup: Fatal error in URDF xml saving" << std::endl;
        return EXIT_FAILURE;
    }
    
    std::cerr << "urdf_gazebo_cleanup: URDF file successfully created" << std::endl;
    
    return EXIT_SUCCESS;
}
