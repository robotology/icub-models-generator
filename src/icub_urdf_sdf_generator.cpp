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


#include <sdf/parser.hh>
#include <sdf/parser_urdf.hh>

#include "urdf_utils.h"

#define NAME "icub_urdf_sdf_generator"

#define ERROR_FT_SENSOR_JOINT "__error_ft_sensor_joint__"

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

bool addGazeboYarpPluginsControlboard(sdf::ElementPtr model, std::string part_name, std::string robot_name)
{
    /*
     <plugin name="controlboard_right_arm" filename="libgazebo_yarp_controlboard.so">
        <yarpConfigurationFile>model://icub/conf/gazebo_icub_right_arm.ini</yarpConfigurationFile>
        <initialConfiguration>-0.52 0.52 0 0.785 0 0 0.698</initialConfiguration>
     </plugin>
     */
    sdf::ElementPtr plugin = model->AddElement("plugin");
    plugin->GetAttribute("name")->SetFromString("controlboard_"+part_name);
    plugin->GetAttribute("filename")->SetFromString("libgazebo_yarp_controlboard.so");
    
    sdf::ElementPtr yarpConfigurationFile_description(new sdf::Element);
    yarpConfigurationFile_description->SetName("yarpConfigurationFile");
    plugin->AddElementDescription(yarpConfigurationFile_description);
    sdf::ElementPtr yarpConfigurationFile = plugin->AddElement("yarpConfigurationFile");
    yarpConfigurationFile->AddValue("string","model://"+robot_name+"/conf/gazebo_icub_"+part_name+".ini",false);
               
    if( part_name == "right_arm" || part_name == "left_arm" ) {
        sdf::ElementPtr initialConfiguration_description(new sdf::Element);
        initialConfiguration_description->SetName("initialConfiguration");
        plugin->AddElementDescription(initialConfiguration_description);
        sdf::ElementPtr initialConfiguration = plugin->AddElement("initialConfiguration");
        initialConfiguration->AddValue("string","-0.52 0.52 0 0.785 0 0 0.698",false);
    }
    
    return true;
}

/**
 * 
 * @param part: head,torso,right_arm,left_arm,right_leg,left_leg
 */
bool addGazeboYarpPluginsControlboard(sdf::SDFPtr icub_sdf, std::string part_name, std::string robot_name)
{
    /// \todo add check on parts
    return addGazeboYarpPluginsControlboard(icub_sdf->root->GetElement("model"),part_name,robot_name);
}

/**
 * 
 * @param sensor: can be only imu_sensor
 */
bool addGazeboYarpPluginsIMU(sdf::SDFPtr icub_sdf, std::string sensor_name, std::string robot_name)
{
    assert( icub_sdf->root->HasElement("model"));
    /*
     <sensor name="imu_sensor" type="imu">
        <always_on>1</always_on>
        <update_rate>100</update_rate>
        <visualize>1</visualize>
        <imu/>
        <plugin filename="libgazebo_yarp_imu.so" name="iCub_yarp_gazebo_plugin_IMU">
          <yarpConfigurationFile>model://icub/conf/gazebo_icub_inertial.ini</yarpConfigurationFile>
        </plugin>
       <pose>0.0185 -0.1108 0.0066 1.5708 -0 0</pose>
      </sensor>
     */
    if( sensor_name == "imu_sensor" ) {
        //imu sensor is child of link head
        sdf::ElementPtr model_elem = icub_sdf->root->GetElement("model");
        for (sdf::ElementPtr link = model_elem->GetElement("link"); link; link = link->GetNextElement("link")) {
            if( link->GetAttribute("name")->GetAsString() == "head" ) {
                sdf::ElementPtr sensor = link->AddElement("sensor");
                sensor->GetAttribute("name")->SetFromString(sensor_name);
                sensor->GetAttribute("type")->SetFromString("imu");
                sdf::ElementPtr always_on = sensor->AddElement("always_on");
                always_on->AddValue("int","1",false);
                sdf::ElementPtr update_rate = sensor->AddElement("update_rate");
                update_rate->AddValue("int","100",false);
                sdf::ElementPtr visualize = sensor->AddElement("visualize");
                visualize->AddValue("int","1",false);
                sdf::ElementPtr imu = sensor->AddElement("imu");
                sdf::ElementPtr plugin = sensor->AddElement("plugin");
                plugin->GetAttribute("filename")->SetFromString("libgazebo_yarp_imu.so");
                plugin->GetAttribute("name")->SetFromString("iCub_yarp_gazebo_plugin_IMU");
                //plugin->AddAttribute("filename","string","libgazebo_yarp_imu.so",false);
                //plugin->AddAttribute("name","string","iCub_yarp_gazebo_plugin_IMU",false);
                sdf::ElementPtr yarpConfigurationFile_description(new sdf::Element);
                yarpConfigurationFile_description->SetName("yarpConfigurationFile");
                plugin->AddElementDescription(yarpConfigurationFile_description);
                sdf::ElementPtr yarpConfigurationFile = plugin->AddElement("yarpConfigurationFile");
                yarpConfigurationFile->AddValue("string","model://"+robot_name+"/conf/gazebo_icub_inertial.ini",false);
                sdf::ElementPtr pose = sensor->AddElement("pose");
                pose->AddValue("string","0.0185 -0.1108 0.0066 1.5708 -0 0",false);
            }
        }
        return true;
    } else {
        return false;
    }
}

bool addGazeboYarpPluginsFT(sdf::ElementPtr joint_element, std::string sensor_name, std::string robot_name)
{
    /*
     <sensor name="left_leg_ft" type="force_torque">
        <always_on>true</always_on>
        <visualize>true</visualize>
        <update_rate>100</update_rate>
        <pose>0 0 0 0 0 0</pose>
        <plugin filename="libgazebo_yarp_forcetorque.so" name="left_leg_ft_plugin">
          <yarpConfigurationFile>model://icub/conf/FT/gazebo_icub_left_leg_ft.ini</yarpConfigurationFile>
        </plugin>
      </sensor> 
     */
    sdf::ElementPtr sensor = joint_element->AddElement("sensor");
                    sensor->GetAttribute("name")->SetFromString(sensor_name+"_ft");
                    sensor->GetAttribute("type")->SetFromString("force_torque");
                    sdf::ElementPtr always_on = sensor->AddElement("always_on");
                    always_on->AddValue("int","1",false);
                    sdf::ElementPtr update_rate = sensor->AddElement("update_rate");
                    update_rate->AddValue("int","100",false);
                    sdf::ElementPtr pose = sensor->AddElement("pose");
                    pose->AddValue("string","0 0 0 0 0 0",false);
                    sdf::ElementPtr plugin = sensor->AddElement("plugin");
                    plugin->GetAttribute("filename")->SetFromString("libgazebo_yarp_forcetorque.so");
                    plugin->GetAttribute("name")->SetFromString(sensor_name+"_ft_plugin");
                    sdf::ElementPtr yarpConfigurationFile_description(new sdf::Element);
                    yarpConfigurationFile_description->SetName("yarpConfigurationFile");
                    plugin->AddElementDescription(yarpConfigurationFile_description);
                    sdf::ElementPtr yarpConfigurationFile = plugin->AddElement("yarpConfigurationFile");
                    yarpConfigurationFile->AddValue("string","model://"+robot_name+"/conf/FT/gazebo_icub_"+sensor_name+"_ft.ini",false);
    
    return true;
}

/**
 * 
 * @param sensor_name: left_arm,right_arm,left_leg,right_leg,left_foot,right_foot
 */
std::string getFTJointName(const std::string sensor_name)
{
    if( sensor_name == "left_arm" ) {
        return "l_arm_ft_sensor";
    } else if ( sensor_name == "right_arm" ) {
        return "r_arm_ft_sensor";
    } else if ( sensor_name == "left_leg" ) {
        return "l_leg_ft_sensor";
    } else if ( sensor_name == "right_leg" ) {
        return "r_leg_ft_sensor";
    } else if ( sensor_name == "left_foot" ) {
        return "l_foot_ft_sensor";
    } else if ( sensor_name == "right_foot" ) {
        return "r_foot_ft_sensor";
    } else {
        return ERROR_FT_SENSOR_JOINT;
    }
}

/**
 * Add the force/torque sensor description (with the relative gazebo_yarp_plugin)
 * @param sensor_name: left_arm,right_arm,left_leg,right_leg,left_foot,right_foot
 */
bool addGazeboYarpPluginsFT(sdf::SDFPtr icub_sdf, std::string sensor_name, std::string robot_name)
{
    bool ret;
    
    sdf::ElementPtr model_elem = icub_sdf->root->GetElement("model");
    
    //Get the gazebo joint that models the FT sensor
    std::string ft_sensor_joint = getFTJointName(sensor_name);
    
    if( ft_sensor_joint == ERROR_FT_SENSOR_JOINT ) {
        return false;
    }
    
    for (sdf::ElementPtr joint = model_elem->GetElement("joint"); joint; joint = joint->GetNextElement("joint")) {
        if( joint->GetAttribute("name")->GetAsString() == ft_sensor_joint ) {
            ret = addGazeboYarpPluginsFT(joint,sensor_name,robot_name);
            if( !ret ) { return false; }
        }
    }
    
    return true;   
}

bool addGazeboODEContactsProperty(sdf::ElementPtr collision_elem)
{
    /*
     * <surface>
         <contact>
            <ode>
                <kp>1e+06</kp>
                <kd>100</kd>
                <max_vel>1</max_vel>
                <min_depth>0</min_depth>
            </ode>
        </contact>
        <friction>
            <ode>
                <mu>1</mu>
                <mu2>1</mu2>
                <fdir1>1 0 0</fdir1>
            </ode>
        </friction>
        </surface>
    */
    sdf::ElementPtr surface = collision_elem->AddElement("surface");
    
    sdf::ElementPtr contact = surface->AddElement("contact");
    sdf::ElementPtr ode_contact = contact->AddElement("ode");
    
    sdf::ElementPtr kp = ode_contact->AddElement("kp");
    kp->AddValue("string","1e06",false);
    
    sdf::ElementPtr kd = ode_contact->AddElement("kd");
    kd->AddValue("string","100",false);
    
    sdf::ElementPtr max_vel = ode_contact->AddElement("max_vel");
    max_vel->AddValue("string","1",false);
    
    sdf::ElementPtr min_depth = ode_contact->AddElement("min_depth");
    min_depth->AddValue("string","0",false);

    sdf::ElementPtr friction = surface->AddElement("friction");
    sdf::ElementPtr ode_friction = friction->AddElement("ode");
    
    sdf::ElementPtr mu = ode_friction->AddElement("mu");
    mu->AddValue("string","1",false);
    
    sdf::ElementPtr mu2 = ode_friction->AddElement("mu2");
    mu2->AddValue("string","1",false);
    
    sdf::ElementPtr fdir1 = ode_friction->AddElement("fdir1");
    fdir1->AddValue("string","1 0 0",false);
    
    return true;
}

bool addGazeboODEContactsProperties(sdf::SDFPtr icub_sdf, std::string link_name, std::string collision_name)
{    
    sdf::ElementPtr model_elem = icub_sdf->root->GetElement("model");
    
    for (sdf::ElementPtr link = model_elem->GetElement("link"); link; link = link->GetNextElement("link")) {
        if( link->GetAttribute("name")->GetAsString() == link_name ) {
            for (sdf::ElementPtr collision = link->GetElement("collision"); collision; collision = collision->GetNextElement("collision")) {
                if( collision->GetAttribute("name")->GetAsString() == collision_name ) {
                    return addGazeboODEContactsProperty(collision);
                }
            }
        }
    }
    
    return false;
}

bool addGazeboODEJointProperties(sdf::SDFPtr icub_sdf)
{
    return false;
}


/**
 * Generate iCub URDF and SDF models, starting from UPMC meshes and kinematics/dynamics information from iDyn
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
    
    std::cout << "Generating iCub model for " << iCub_name << " (head: " << head_version << " , legs: " << legs_version << " , feet: " << feet_version << " )" << std::endl; 
    if( ft_feet ) { std::cout << "Generating FT sensor in the feet" << std::endl; }
    
    std::string paris_directory = data_directory+"urdf_paris/";
    
    std::string paris_subdirectory;
    if( simple_meshes ) {
        paris_subdirectory = "icub_simple";
    } else {
        paris_subdirectory = "icub";
    }
    
    std::string urdf_general_directory = root_directory+"urdf/";
    std::string urdf_robot_directory = urdf_general_directory+iCub_name+"/";
    std::string filename_urdf = urdf_robot_directory+"icub.urdf";
    std::string gazebo_model_directory = root_directory + "gazebo_models/";
    std::string gazebo_robot_model_directory = gazebo_model_directory+iCub_name+"/";
    std::string filename_urdf_gazebo = gazebo_robot_model_directory+"icub_simulation.urdf";
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
    
    if( ! toKDL(icub_idyn,icub_kdl,dummy1,dummy2,iCub::iDynTree::SKINDYNLIB_SERIALIZATION,ft_feet,true) ) {
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
    
    std::cout << "Loading file " << filename_urdf_paris << std::endl;
    
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

    /*
    if( ! xml_doc->SaveFile(filename_urdf_gazebo_conversion) ) {
        std::cerr << "Fatal error in URDF xml saving" << std::endl;
        return false;
    }*/
    
    std::stringstream ss;
    ss << *xml_doc;
    std::string urdf_for_gazebo_conversion_string = ss.str();
    
    //Ugly workaround, I know, probably can be avoided by directly using sdfformat library
    //std::string gazebo_conversion_command = "gzsdf print " + filename_urdf_gazebo_conversion + " > " + gazebo_sdf_filename; 
    //std::cout << "Running command: " << gazebo_conversion_command << std::endl;

    sdf::URDF2SDF urdf_converter;
    TiXmlDocument sdf_xml = urdf_converter.InitModelString(urdf_for_gazebo_conversion_string);
    
    //system(gazebo_conversion_command.c_str());
    //if( !system(gazebo_conversion_command.c_str()) ) { std::cerr << "Error in urdf - sdf conversion" << std::endl; return false; }
    
    //\todo: adding plugins to generated sdf
    sdf::SDFPtr icub_sdf(new sdf::SDF());
    sdf::init(icub_sdf);
    
    //if( ! sdf::readFile(gazebo_sdf_filename,icub_sdf) ) { std::cerr << "Problem in reading SDF file" << std::endl; return false; }
    if( ! sdf::readDoc(&sdf_xml,icub_sdf,"custom sdf xml") ) { std::cerr << "Problem in loading SDF file" << std::endl; return false; }
    
    if( ! icub_sdf->root->HasElement("model") ) { std::cerr << "Problem in parsing SDF dom" << std::endl; return false; }
    
    //Adding IMU sensor (in all robots)
    /// \todo add difference between head V1 and head V2
    if( ! addGazeboYarpPluginsIMU(icub_sdf,"imu_sensor",iCub_name) ) { std::cerr << "Problem in adding imu sensor" << std::endl; return false; }
    
    //Adding FT sensors (in all robots)
    bool ret=true;
    ret = ret && addGazeboYarpPluginsFT(icub_sdf,"left_arm",iCub_name);
    ret = ret && addGazeboYarpPluginsFT(icub_sdf,"right_arm",iCub_name);
    ret = ret && addGazeboYarpPluginsFT(icub_sdf,"left_leg",iCub_name);
    ret = ret && addGazeboYarpPluginsFT(icub_sdf,"right_leg",iCub_name);
    if( ft_feet ) {
        std::cerr << "Adding feet FT sensors" << std::endl;
        ret = ret && addGazeboYarpPluginsFT(icub_sdf,"left_foot",iCub_name);
        ret = ret && addGazeboYarpPluginsFT(icub_sdf,"right_foot",iCub_name);
    }
    if( !ret ) { std::cerr << "Problem in adding ft sensors" << std::endl; return false; }
    
    //Adding controlboards (in all robots, in the future add differences)
    ret = ret && addGazeboYarpPluginsControlboard(icub_sdf,"torso",iCub_name);
    if( !ret ) { std::cerr << "Problem in adding torso controlboard" << std::endl; return false; }
    ret = ret && addGazeboYarpPluginsControlboard(icub_sdf,"head",iCub_name);
    if( !ret ) { std::cerr << "Problem in adding head controlboard" << std::endl; return false; }
    ret = ret && addGazeboYarpPluginsControlboard(icub_sdf,"left_arm",iCub_name);
    if( !ret ) { std::cerr << "Problem in adding left_arm controlboards" << std::endl; return false; }
    ret = ret && addGazeboYarpPluginsControlboard(icub_sdf,"right_arm",iCub_name);
    if( !ret ) { std::cerr << "Problem in adding right_arm controlboards" << std::endl; return false; }
    ret = ret && addGazeboYarpPluginsControlboard(icub_sdf,"left_leg",iCub_name);
    if( !ret ) { std::cerr << "Problem in adding left_leg controlboards" << std::endl; return false; }
    ret = ret && addGazeboYarpPluginsControlboard(icub_sdf,"right_leg",iCub_name);
    if( !ret ) { std::cerr << "Problem in adding right_leg controlboards" << std::endl; return false; }
    
    //Adding parameters for properly handling of the contacts
    ret = ret && addGazeboODEContactsProperties(icub_sdf,"l_foot","l_foot_collision");
    if( !ret ) { std::cerr << "Problem in adding contact properties" << std::endl; return false; }
    ret = ret && addGazeboODEContactsProperties(icub_sdf,"r_foot","r_foot_collision");
    if( !ret ) { std::cerr << "Problem in adding contact properties" << std::endl; return false; }
    
    //Adding pose to avoid intersection with ground
    //<pose>0 0 0.70 0 0 0 </pose>
    sdf::ElementPtr pose = icub_sdf->root->GetElement("model")->AddElement("pose");
    pose->AddValue("string","0 0 0.70 0 0 0",false);
    
    
    icub_sdf->Write(gazebo_sdf_filename);
    
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
    
    //Generating model for black iCub
    //                  robot_name     directory         head   legs   feet  meshes 
    if( !generate_iCub_model("iCubGenova01",output_directory, 2    , 2    , 2   , false , data_directory,mass_epsilon,inertia_epsilon) ) return EXIT_FAILURE;
    
    //Generating model for red iCub
    if( !generate_iCub_model("iCubGenova03",output_directory, 2    , 1    , 2   , false , data_directory,mass_epsilon,inertia_epsilon) ) return EXIT_FAILURE;
        
    std::cerr << "iCub model files successfully created" << std::endl;
    
    return EXIT_SUCCESS;
}

