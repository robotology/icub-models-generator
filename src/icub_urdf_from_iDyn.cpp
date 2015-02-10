/**
 * Copyright  (C)  2013 CoDyCo Project
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

#define NAME "icub_urdf_from_iDyn"

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
      std::cout << "child(" << (count++)+1 << "):  " << (*child)->name  << " FrameToTip " << urdf_export_helpers::values2str((*child)->parent_joint->parent_to_joint_origin_transform.position) << std::endl;
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


int main(int argc, char* argv[])
{
    bool status = true;

    yarp::os::Property opt;

    opt.fromCommand(argc,argv);

    if( opt.check("help") || !opt.check("output") ) {
        std::cerr << "Usage: \t icub_urdf_from_iDyn --headV2 --legsV2 --output output.xml" << std::endl;
        std::cerr << "Default values: head and legs v2" << std::endl;
        return EXIT_FAILURE;
    }

    std::string output_file = opt.find("output").asString().c_str();

    iCub::iDyn::version_tag icub_type;

    icub_type.head_version = 2;
    icub_type.legs_version = 2;

    if( opt.check("headV2") ) icub_type.head_version = 2;
    if( opt.check("legsV2") ) icub_type.legs_version = 2;
    if( opt.check("headV1") ) icub_type.head_version = 1;
    if( opt.check("legsV1") ) icub_type.legs_version = 1;


    iCub::iDyn::iCubWholeBody icub_idyn(icub_type);

    KDL::Tree icub_kdl;

    KDL::JntArray dummy1,dummy2;

    if( ! toKDL(icub_idyn,icub_kdl,dummy1,dummy2,iCub::iDynTree::SKINDYNLIB_SERIALIZATION,false,true) ) {
        std::cerr << "Fatal error in iDyn - KDL conversion" << std::endl;
        return EXIT_FAILURE;
    }

    boost::shared_ptr<urdf::ModelInterface> icub_ptr(new urdf::ModelInterface);

    //std::cout << "iCub KDL::Tree: " << std::endl;
    //std::cout << icub_kdl << std::endl;


    if( ! kdl_format_io::treeToUrdfModel(icub_kdl,"test_icub",*icub_ptr) ) {
        std::cerr << "Fatal error in KDL - URDF conversion" << std::endl;
        return EXIT_FAILURE;
    }

    boost::shared_ptr<const Link> root_link=icub_ptr->getRoot();
    printTree(root_link);

    TiXmlDocument*  xml_doc =  exportURDF(icub_ptr);
    if( ! xml_doc->SaveFile(output_file) ) {
        std::cerr << "Fatal error in URDF xml saving" << std::endl;
    }

    std::cerr << "URDF file successfully created" << std::endl;

    return EXIT_SUCCESS;
}
