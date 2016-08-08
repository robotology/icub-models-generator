/**
 * Copyright  (C) 2013-2014 CoDyCo Project
 * Author: Silvio Traversaro
 * website: http://www.codyco.eu
 */

#ifndef URDF_SDF_FROM_DH_UTILS
#define URDF_SDF_FROM_DH_UTILS


#include <iCub/iDynTree/idyn2kdl_icub.h>

#include <iostream>
#include <fstream>
#include <cstdlib>

#include <iCub/iDyn/iDyn.h>
#include <iCub/iDyn/iDynBody.h>

#include <kdl/tree.hpp>

#include <iDynTree/ModelIO/impl/urdf_export.hpp>

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


bool addGazeboYarpPluginsControlboardToSDF(sdf::ElementPtr model, std::string part_name, std::string robot_name);

/**
 *
 * @param part: head,torso,right_arm,left_arm,right_leg,left_leg
 */
bool addGazeboYarpPluginsControlboardToSDF(sdf::SDFPtr icub_sdf, std::string part_name, std::string robot_name);

/**
 *
 * @param sensor: can be only imu_sensor
 */
bool addGazeboYarpPluginsIMUToSDF(sdf::SDFPtr icub_sdf, std::string sensor_name, std::string robot_name);

/**
 *
 * @param sensor_name: left_arm,right_arm,left_leg,right_leg,left_foot,right_foot
 */
std::string getFTJointName(const std::string sensor_name);
/**
 * Add the force/torque sensor description to the URDF (using gazebo extentions)
 * @param sensor_name: left_arm,right_arm,left_leg,right_leg,left_foot,right_foot
 */
bool addGazeboYarpPluginsFTToURDF(TiXmlDocument* urdf_doc, std::string sensor_name);

void AddElementAndSetValue(sdf::ElementPtr parent, std::string childType, std::string value_type, std::string value);

bool addGazeboSurfaceFrictionInformationToCollisionSDF(sdf::ElementPtr collision_sdf);

/**
 * For a given link add surface friction informations to the SDF
 *
 */
bool addGazeboSurfaceFrictionInformationToSDF(sdf::SDFPtr icub_sdf, std::string link_name);

bool addGazeboYarpPluginsFTToSDF(sdf::ElementPtr joint_element, std::string sensor_name, std::string robot_name);

/**
 * Add the force/torque sensor description (with the relative gazebo_yarp_plugin)
 * @param sensor_name: left_arm,right_arm,left_leg,right_leg,left_foot,right_foot
 */
bool addGazeboYarpPluginsFTToSDF(sdf::SDFPtr icub_sdf, std::string sensor_name, std::string robot_name);

bool addGazeboODEContactsProperty(sdf::ElementPtr collision_elem);
bool addGazeboODEContactsProperties(sdf::SDFPtr icub_sdf, std::string link_name, std::string collision_name);

bool substituteCollisionWithBoxInCollisionSDF(sdf::ElementPtr collision,
                                     std::string new_box_collision_pose,
                                     std::string box_size);

bool substituteCollisionWithBoxInSDF(sdf::SDFPtr icub_sdf,
                                     std::string link_name,
                                     std::string collision_name,
                                     std::string new_box_collision_pose,
                                     std::string box_size);

bool generate_model_config_file(std::string robot_name, std::string gazebo_robot_model_directory);

std::string get_gazebo_model_directory(std::string root_directory);

bool generate_iCub_urdf_model(std::string iCub_name,
                              int head_version,
                               int legs_version,
                               int feet_version,
                               bool is_iCubParis02,
                               bool is_icubGazeboSim,
                               bool simple_meshes,
                               std::string data_directory,
                               double mass_epsilon,
                               double inertia_epsilon,
                               bool noFTsimulation,
                               boost::shared_ptr<urdf::ModelInterface> urdf_file,
                               std::string outputfile );

bool generate_iCub_sdf_model(std::string iCub_name,
                              int head_version,
                               int legs_version,
                               int feet_version,
                               bool is_iCubParis02,
                               bool is_icubGazeboSim,
                               bool simple_meshes,
                               std::string data_directory,
                               double mass_epsilon,
                               double inertia_epsilon,
                               bool noFTsimulation,
                               boost::shared_ptr<urdf::ModelInterface> urdf_file);

/**
 * Generate iCub URDF and SDF models, starting from UPMC meshes and kinematics/dynamics information from iDyn
 *
 * @param head_version can be 1 or 2
 * @param legs_version can be 1 or 2
 * @param feet_version can be 1 or 2
 * @param simple_meshes if true uses the simple visualization meshes, if false uses the heavy detailed one
 */
/*
bool generate_iCub_model(std::string iCub_name,
                         std::string root_directory,
                         int head_version,
                         int legs_version,
                         int feet_version,
                         bool is_iCubParis02,
                         bool is_icubGazeboSim,
                         bool simple_meshes,
                         std::string data_directory,
                         double mass_epsilon,
                         double inertia_epsilon,
                         bool noFTsimulation = false
                        );*/

bool generate_gazebo_database(const std::vector<std::string> & robot_names, const std::string root_directory);

#endif
