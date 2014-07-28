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

#include "urdf_utils.h"

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
    
    if( argc != 2 ) {
        std::cerr << "Usage: \t urdf_print_homegenous_transformation urdf_model.xml" << std::endl;
        std::cerr << "Prints the homogeneous transformation between the different reference frames of an the urdf model" << std::endl;
        return EXIT_FAILURE;
    }
    
    std::string file_name_urdf(argv[1]);

    boost::shared_ptr<urdf::ModelInterface> urdf_input;

    
    std::ifstream t_im(file_name_urdf.c_str());
    std::stringstream buffer_im;
    buffer_im << t_im.rdbuf();
    urdf_input = parseURDF(buffer_im.str());
    if( !urdf_input ) {
        std::cerr << "Fatal error in URDF xml parsing" << std::endl;
        return EXIT_FAILURE;
    }

    ret = urdf_print_hom_transformations(urdf_input);
    
    if(!ret ) {
        std::cerr << "Error in printing homogeneous matrices" << std::endl;
		return EXIT_FAILURE;
	}

    return EXIT_SUCCESS;
}
