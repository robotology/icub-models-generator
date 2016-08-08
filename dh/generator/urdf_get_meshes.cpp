/**
 * Copyright  (C)  2013 CoDyCo Project
 * Author: Silvio Traversaro
 * website: http://www.codyco.eu
 */
 
#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>

#include "urdf_utils.h"

#include <iDynTree/ModelIO/impl/urdf_import.hpp>

#include <iostream>
#include <sstream>
#include <fstream>

#include <tinyxml.h>

using namespace iDynTree;
using namespace KDL;
using namespace std;

using namespace urdf;
using namespace boost;



int main(int argc, char* argv[])
{
    bool status = true;
        bool ret = true;
        
    bool verbose = true;
    
    if( argc != 4 ) {
        std::cerr << "Usage: \t urdf_get_meshes urdf_meshes.xml urdf_input.xml urdf_output.xml" << std::endl;
        std::cerr << "In which the urdf_output.xml is the same model of urdf_input.xml, but with the meshes of urdf_meshes.xml" << std::endl;
        return EXIT_FAILURE;
    }
    
    std::string file_name_urdf_meshes(argv[1]);
    std::string file_name_urdf_input(argv[2]);
    std::string file_name_urdf_output(argv[3]);


    boost::shared_ptr<urdf::ModelInterface> urdf_meshes;
    boost::shared_ptr<urdf::ModelInterface> urdf_input;

    std::ifstream t_um(file_name_urdf_meshes.c_str());
    std::stringstream buffer_um;
    buffer_um << t_um.rdbuf();
    urdf_meshes = parseURDF(buffer_um.str());
    
    std::ifstream t_im(file_name_urdf_input.c_str());
    std::stringstream buffer_im;
    buffer_im << t_im.rdbuf();
    urdf_input = parseURDF(buffer_im.str());
    if( !ret ) {
        std::cerr << "Fatal error in URDF xml parsing" << std::endl;
        return EXIT_FAILURE;
    }

    if( !urdf_import_meshes(urdf_input,urdf_meshes) ) {
        std::cerr << "Fatal error in URDF mesh importing" << std::endl;
        return false;
    }
    
    TiXmlDocument* xml_doc;
    
    xml_doc = exportURDF(urdf_input);
    
    if( ! xml_doc->SaveFile(file_name_urdf_output) ) {
        std::cerr << "Fatal error in URDF xml saving" << std::endl;
        return EXIT_FAILURE;
    }
    
    std::cerr << "URDF file successfully created" << std::endl;
    
    return EXIT_SUCCESS;
}
