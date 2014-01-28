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

double getTotalMass(urdf::ModelInterface & model)
{
    std::vector<boost::shared_ptr<Link> > input_links;
    
    model.getLinks(input_links);
    
    double total_mass = 0.0;
    
    std::cout << "Found " << input_links.size() << " links in input URDF " << std::endl;
    for(int i=0; i < input_links.size(); i++ )
    {
        if( input_links[i]->inertial ) {
            total_mass = total_mass + input_links[i]->inertial->mass;
        }
    }
    
    return total_mass;
}

int main(int argc, char* argv[])
{
    bool status = true;
        bool ret = true;
        
    bool verbose = true;
    
    if( argc != 2 ) {
        std::cerr << "Usage: \t urdf_get_total mass urdf_model.xml" << std::endl;
        std::cerr << "Prints the total mass of the urdf model" << std::endl;
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

    double mass = getTotalMass(*urdf_input);
    
    std::cout << "URDF file has a total mass of " << mass << std::endl;
    
    return EXIT_SUCCESS;
}
