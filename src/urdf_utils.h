/**
 * Copyright  (C) 2013-2014 CoDyCo Project
 * Author: Silvio Traversaro
 * website: http://www.codyco.eu
 */

#ifndef __URDF_UTILS_FUNCTIONS__
#define __URDF_UTILS_FUNCTIONS__

#include <urdf_model/model.h>


/**
 * Import meshes from a urdf model to another urdf model.
 * The match between diffent models is done based on link names, taking into account any diffence in the 
 * position of the reference frames of the links.
 * 
 * @param urdf_input the urdf model for which the mesh are added
 * @param urdf_meshes the urf model from which the mesh are imported
 * @return true if all went ok, false otherwise
 * 
 */
bool urdf_import_meshes(boost::shared_ptr<urdf::ModelInterface> urdf_input, boost::shared_ptr<urdf::ModelInterface> urdf_meshes);

/**
 * Get the total mass of a URDF model
 * 
 * @param model the considered URDF model
 * @return the total mass of the model
 * 
 */
double getTotalMass(urdf::ModelInterface & model);

/**
 * Import limits from a urdf model to another urdf model.
 * The match between diffent models is done based on joint names.
 * 
 * @param urdf_input the urdf model to which the limits are added
 * @param urdf_meshes the urf model from which the limits are imported
 * @return true if all went ok, false otherwise
 * 
 */
bool urdf_import_limits(boost::shared_ptr<urdf::ModelInterface> urdf_input, boost::shared_ptr<urdf::ModelInterface> urdf_meshes);

/**
 * If a link without inertia is in the root, remove it (to handle base_link as commonly found in humanoid models for REP 120)"
 * 
 */
bool urdf_gazebo_cleanup_remove_massless_root(boost::shared_ptr<urdf::ModelInterface> urdf_input);

/**
 * Links with zero mass, with no childrens and connected to their parent with a fixed joint are removed
 * 
 */
bool urdf_gazebo_cleanup_remove_frames(boost::shared_ptr<urdf::ModelInterface> urdf_input);


/**
 * Links with zero mass, with no childrens and connected to their parent with a fixed joint are removed
 * The fixed joint connecting two non-leaf links (typically used in iCub for modeling FT sensors) are transformed in rotational joint with no range (to address this issue https://bitbucket.org/osrf/gazebo/issue/618/add-option-to-disable-auto-fixed-joint)
 */
bool urdf_gazebo_cleanup_transform_FT_sensors(boost::shared_ptr<urdf::ModelInterface> urdf_input);

/**
 * For the links that are not massless_root, frames or subpart of FT sensors links but have zero mass or zero diagonal inertia elements a small mass and a small diagonal inertia matrix
 */
bool urdf_gazebo_cleanup_regularize_masses(boost::shared_ptr<urdf::ModelInterface> urdf_input, double mass_epsilon, double inertia_epsilon);

/**
 * Add a model uri to the meshes address
 * 
 */
bool urdf_gazebo_cleanup_add_model_uri(boost::shared_ptr<urdf::ModelInterface> urdf_input, std::string model_prefix);










#endif