/**
 * Copyright  (C) 2013-2014 CoDyCo Project
 * Author: Silvio Traversaro
 * website: http://www.codyco.eu
 */

#include "urdf_utils.h"

#include <kdl/tree.hpp>

#include <kdl/treefksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>

#include <iDynTree/ModelIO/impl/urdf_import.hpp>

using namespace urdf;
using namespace KDL;
using namespace std;
using namespace boost;

///< \todo TODO add support for deleting a long chain of null link connected by fixed base
bool deleteLink(boost::shared_ptr<urdf::ModelInterface> urdf_input, std::string link_to_delete, bool verbose=false)
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

        if( verbose ) {
            std::cout << "deleteLink: removing link  " << link_sptr->name << std::endl;
            std::cout << "deleteLink: removing joint " << joint_sptr->name << std::endl;
        }
        int deleted = 0;
        deleted = urdf_input->links_.erase(link_sptr->name);
        if( deleted != 1 ) { if( verbose ) { std::cout << "deleteLink error joint: Link not found" << std::endl; } return false; }
        deleted = urdf_input->joints_.erase(link_sptr->name);
        if( deleted != 1 ) { if( verbose ) { std::cout << "deleteLink error joint: Link not found" << std::endl; } return false; }

    } else {
        //deleting root link

        boost::shared_ptr<urdf::Link> link_sptr = urdf_input->links_[link_to_delete];

        if( link_sptr->child_links.size() != 1 ) { std::cerr << "deleteLink error: tryng to delete a root link with more than a child" << std::endl; return false; }

        boost::shared_ptr<urdf::Joint> joint_sptr = link_sptr->child_joints[0];

        boost::shared_ptr<urdf::Link> new_root = link_sptr->child_links[0];

        urdf_input->root_link_ = new_root;

        urdf_input->links_.erase(link_sptr->name);
        urdf_input->joints_.erase(new_root->name);

    }

    return true;

}

bool deleteLinks(boost::shared_ptr<urdf::ModelInterface> urdf_input, std::vector<std::string> linksToDelete, bool verbose=false)
{
    for(int i=0; i < linksToDelete.size(); i++ ) {
        if( !deleteLink(urdf_input,linksToDelete[i],verbose) ) return false;
    }
    return true;
}

// construct vector
urdf::Vector3 toUrdf(const KDL::Vector & v)
{
    return urdf::Vector3(v.x(), v.y(), v.z());
}

// construct rotation
urdf::Rotation toUrdf(const KDL::Rotation & r)
{
    double x,y,z,w;
    r.GetQuaternion(x,y,z,w);
    return urdf::Rotation(x,y,z,w);
}

// construct pose
urdf::Pose toUrdf(const KDL::Frame & p)
{
    urdf::Pose ret;
    ret.rotation = toUrdf(p.M);
    ret.position = toUrdf(p.p);
    return ret;
}

// construct vector
KDL::Vector toKdl(urdf::Vector3 v)
{
  return KDL::Vector(v.x, v.y, v.z);
}

// construct rotation
KDL::Rotation toKdl(urdf::Rotation r)
{
  return KDL::Rotation::Quaternion(r.x, r.y, r.z, r.w);
}

// construct pose
KDL::Frame toKdl(urdf::Pose p)
{
  return KDL::Frame(toKdl(p.rotation), toKdl(p.position));
}

double getTotalMass(urdf::ModelInterface & model)
{
    std::vector<boost::shared_ptr<urdf::Link> > input_links;

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

bool urdf_import_meshes(boost::shared_ptr<urdf::ModelInterface> urdf_input,
                        boost::shared_ptr<urdf::ModelInterface> urdf_meshes,
                        bool verbose)
{
    bool ret = true;
    KDL::Tree kdl_meshes, kdl_input;
    ret = ret && iDynTree::treeFromUrdfModel(*urdf_meshes,kdl_meshes,false);
    ret = ret && iDynTree::treeFromUrdfModel(*urdf_input,kdl_input,false);
    if( !ret ) {
        std::cerr << "Fatal error in URDF KDL conversion saving" << std::endl;
        return false;
    }

    if(kdl_meshes.getNrOfJoints() != kdl_input.getNrOfJoints())
    {
        std::cerr << "Fatal error, the two models have a different number of DOFs" << std::endl;
        return false;
    }

    KDL::JntArray q(kdl_meshes.getNrOfJoints());
    SetToZero(q);

    KDL::TreeFkSolverPos_recursive meshes_solver(kdl_meshes);
    KDL::TreeFkSolverPos_recursive input_solver(kdl_input);

    //Iterate over the links of urdf_input, and copy the mesh
    std::vector<boost::shared_ptr<urdf::Link> > input_links;

    urdf_input->getLinks(input_links);

    if( verbose ) std::cout << "Found " << input_links.size() << " links in input URDF " << std::endl;
    for(int i=0; i < input_links.size(); i++ )
    {
        KDL::Frame base_link_meshes, base_link_input;
        KDL::Frame link_visual_old, link_collision_old;
        KDL::Frame link_visual_new, link_collision_new;
        std::string input_name = input_links[i]->name;

        if( verbose ) std::cout << "Processing link " << input_name << std::endl;

        boost::shared_ptr<const urdf::Link> mesh_link_ptr = urdf_meshes->getLink(input_name);


        if( mesh_link_ptr ) {
            if( verbose ) std::cout << "Tryng to copy meshes of link " << input_name << std::endl;
            //if there is a link with the same name, copy the mesh
            //supporting only single mesh
            if( mesh_link_ptr->collision || mesh_link_ptr->visual ) {
                int retur;
                retur = meshes_solver.JntToCart(q,base_link_meshes,input_name);
                if( retur < 0 ) { return false; }
                retur = input_solver.JntToCart(q,base_link_input,input_name);
                if( retur < 0 ) { return false; }
            }
            if( mesh_link_ptr->collision ) {
                link_collision_old = toKdl(mesh_link_ptr->collision->origin);
                link_collision_new = base_link_input.Inverse()*base_link_meshes*link_collision_old;

                if( verbose ) std::cout << "Old collision origin " << link_collision_old << endl;
                if( verbose ) std::cout << "New collision origin " << link_collision_new << endl;


                input_links[i]->collision.reset(new urdf::Collision);

                input_links[i]->collision->geometry.reset(new urdf::Geometry);

                input_links[i]->collision->origin = toUrdf(link_collision_new);

                switch( mesh_link_ptr->collision->geometry->type ) {
                    case urdf::Geometry::SPHERE:
                        input_links[i]->collision->geometry.reset(new urdf::Sphere);
                        *(input_links[i]->collision->geometry) = *(mesh_link_ptr->collision->geometry);
                    break;
                    case Geometry::BOX:
                        input_links[i]->collision->geometry.reset(new urdf::Box);
                        *(input_links[i]->collision->geometry) = *(mesh_link_ptr->collision->geometry);
                    break;
                    case Geometry::CYLINDER:
                        input_links[i]->collision->geometry.reset(new urdf::Cylinder);
                        *(input_links[i]->collision->geometry) = *(mesh_link_ptr->collision->geometry);
                    break;
                    case Geometry::MESH:
                        input_links[i]->collision->geometry.reset(new urdf::Mesh);
                        if( verbose ) std::cout << "Copyng collision mesh with filename " << (boost::static_pointer_cast<urdf::Mesh>(mesh_link_ptr->collision->geometry))->filename << std::endl;
                        (boost::static_pointer_cast<urdf::Mesh>(input_links[i]->collision->geometry))->filename = (boost::static_pointer_cast<urdf::Mesh>(mesh_link_ptr->collision->geometry))->filename;
                        (boost::static_pointer_cast<urdf::Mesh>(input_links[i]->collision->geometry))->scale = (boost::static_pointer_cast<urdf::Mesh>(mesh_link_ptr->collision->geometry))->scale;
                    break;
                }

                input_links[i]->collision_array.push_back(input_links[i]->collision);

            }
            if( mesh_link_ptr->visual ) {
                link_visual_old = toKdl(mesh_link_ptr->visual->origin);
                link_visual_new = base_link_input.Inverse()*base_link_meshes*link_visual_old;

                input_links[i]->visual.reset(new urdf::Visual);

                input_links[i]->visual->origin = toUrdf(link_visual_new);

                input_links[i]->visual->material_name = mesh_link_ptr->visual->material_name;

                input_links[i]->visual->material.reset(new urdf::Material);

                *(input_links[i]->visual->material) = *(mesh_link_ptr->visual->material);

                input_links[i]->visual->geometry.reset(new urdf::Geometry);

                switch( mesh_link_ptr->visual->geometry->type ) {
                    case Geometry::SPHERE:
                        input_links[i]->visual->geometry.reset(new urdf::Sphere);
                        *(input_links[i]->visual->geometry) = *(mesh_link_ptr->visual->geometry);
                    break;
                    case Geometry::BOX:
                        input_links[i]->visual->geometry.reset(new urdf::Box);
                        *(input_links[i]->visual->geometry) = *(mesh_link_ptr->visual->geometry);
                    break;
                    case Geometry::CYLINDER:
                        input_links[i]->visual->geometry.reset(new urdf::Cylinder);
                        *(input_links[i]->visual->geometry) = *(mesh_link_ptr->visual->geometry);
                    break;
                    case Geometry::MESH:
                        input_links[i]->visual->geometry.reset(new urdf::Mesh);
                        if( verbose ) std::cout << "Copyng visual mesh with filename " << (boost::static_pointer_cast<urdf::Mesh>(mesh_link_ptr->visual->geometry))->filename << std::endl;
                        (boost::static_pointer_cast<urdf::Mesh>(input_links[i]->visual->geometry))->filename = (boost::static_pointer_cast<urdf::Mesh>(mesh_link_ptr->visual->geometry))->filename;
                        (boost::static_pointer_cast<urdf::Mesh>(input_links[i]->visual->geometry))->scale = (boost::static_pointer_cast<urdf::Mesh>(mesh_link_ptr->visual->geometry))->scale;
                    break;
                }

                input_links[i]->visual_array.push_back(input_links[i]->visual);


            }
        } else {
            std::cout << "No link found with name " << input_name << std::endl;
        }
    }
    return true;
}

bool urdf_import_limits(boost::shared_ptr<urdf::ModelInterface> urdf_input, boost::shared_ptr<urdf::ModelInterface> urdf_limits)
{
    //Iterate over the joints of urdf_input, and copy the limits
    for( std::map<std::string, boost::shared_ptr<urdf::Joint> >::iterator it = urdf_input->joints_.begin();
         it != urdf_input->joints_.end(); it++ )
    {
        std::string joint_name = it->second->name;
        std::string link_name = it->first;
        std::cout << "urdf_import_limits: Processing joint " << joint_name << " relative to link " << link_name << std::endl;

        boost::shared_ptr<const urdf::Joint> limits_joints_ptr = urdf_limits->getJoint(joint_name);


        if( limits_joints_ptr ) {
            std::cout << "urdf_import_limits: Tryng to copy limits of joint " << joint_name << std::endl;

            it->second->limits.reset(new urdf::JointLimits);
            *(it->second->limits) = *(limits_joints_ptr->limits);

            //Copy also the type to switch from continuous to revolute
            it->second->type = limits_joints_ptr->type;

        } else {
            std::cout << "urdf_import_limits: No joint found with link name " << link_name << std::endl;
        }
    }
    return true;
}

bool urdf_set_friction_parameters(boost::shared_ptr<urdf::ModelInterface> urdf_input, double damping, double friction)
{
    //Iterate over the joints of urdf_input, and copy the limits
    for( std::map<std::string, boost::shared_ptr<urdf::Joint> >::iterator it = urdf_input->joints_.begin();
         it != urdf_input->joints_.end(); it++ )
    {
        if( it->second->type == urdf::Joint::REVOLUTE ||
            it->second->type == urdf::Joint::CONTINUOUS )
        {
            it->second->dynamics.reset(new urdf::JointDynamics);
            it->second->dynamics->damping = damping;
            it->second->dynamics->friction = friction;
        }
    }

    return true;
}

bool urdf_print_hom_transformations(boost::shared_ptr<urdf::ModelInterface> urdf_input)
{
    //Iterate over the joints of urdf_input, and print the transformation matrix
    for( std::map<std::string, boost::shared_ptr<urdf::Joint> >::iterator it = urdf_input->joints_.begin();
         it != urdf_input->joints_.end(); it++ )
    {
        std::string joint_name = it->second->name;
        std::string father_link_name = it->second->parent_link_name;
        std::string child_link_name = it->second->child_link_name;
        std::cout << "Transformation matrix of frame of link " << child_link_name <<  " expressed in the frame of link " << father_link_name << " : " << std::endl;
        std::cout << toKdl(it->second->parent_to_joint_origin_transform) << std::endl;
        std::cout << "Quaternion: " << it->second->parent_to_joint_origin_transform.rotation.x
                  << " " << it->second->parent_to_joint_origin_transform.rotation.x
                  << " " << it->second->parent_to_joint_origin_transform.rotation.y
                  << " " << it->second->parent_to_joint_origin_transform.rotation.z
                  << " " << it->second->parent_to_joint_origin_transform.rotation.w << std::endl;
        std::cout << "Axis of joint " << joint_name << " : " << toKdl(it->second->axis) << std::endl;

    }
    return true;
}

bool urdf_gazebo_cleanup_remove_massless_root(boost::shared_ptr<urdf::ModelInterface> urdf_input)
{
    if( !(urdf_input->getRoot()->inertial) ) {
            if( !deleteLink(urdf_input,urdf_input->getRoot()->name) ) return EXIT_FAILURE;
    }
    return true;
}

bool urdf_gazebo_cleanup_remove_frames(boost::shared_ptr<urdf::ModelInterface> urdf_input)
{
    std::vector<std::string> linksToDelete;
    std::vector<boost::shared_ptr<Link> > input_links;

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

    if( !deleteLinks(urdf_input,linksToDelete,true) ) return false;
    linksToDelete.resize(0);
    return true;
}


bool hasEnding (std::string const &fullString, std::string const &ending)
{
    if (fullString.length() >= ending.length()) {
        return (0 == fullString.compare (fullString.length() - ending.length(), ending.length(), ending));
    } else {
        return false;
    }
}

bool urdf_gazebo_cleanup_transform_FT_sensors(boost::shared_ptr<urdf::ModelInterface> urdf_input)
{
    std::vector<boost::shared_ptr<Link> > input_links;
    input_links.clear();

    urdf_input->getLinks(input_links);

    for(int i=0; i < input_links.size(); i++ )
    {
        //Rule 2
        int nrOfChildrens = input_links[i]->child_links.size();

        std::string joint_name = input_links[i]->parent_joint->name;

        std::string ft_sensor_joint_suffix = "_ft_sensor";

        if( hasEnding(joint_name,ft_sensor_joint_suffix) ) {
            input_links[i]->parent_joint->type = urdf::Joint::REVOLUTE;
            if( input_links[i]->parent_joint->limits ) {
                input_links[i]->parent_joint->limits->clear();
            } else {
                input_links[i]->parent_joint->limits.reset(new JointLimits());
            }
        }

    }

    return true;
}

bool urdf_gazebo_cleanup_regularize_masses(boost::shared_ptr<urdf::ModelInterface> urdf_input, double mass_epsilon, double inertia_epsilon)
{
    std::vector<boost::shared_ptr<Link> > input_links;
    input_links.clear();
    urdf_input->getLinks(input_links);
    for(int i=0; i < input_links.size(); i++ )
    {
     //Rule 3
        if( !(input_links[i]->parent_joint) ) {
            //If the base_link was not removed, avoid adding masses
            continue;
        }

        int nrOfChildrens = input_links[i]->child_links.size();

        if( input_links[i]->parent_joint->type != urdf::Joint::FIXED || nrOfChildrens > 0 ) {

            std::cerr << "Corrected mass for link " << input_links[i]->name << std::endl;

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
    return true;
}


bool urdf_gazebo_cleanup_add_model_uri(boost::shared_ptr<urdf::ModelInterface> urdf_input,std::string rule4_prefix)
{
    std::vector<boost::shared_ptr<Link> > input_links;
    input_links.clear();
    urdf_input->getLinks(input_links);

    for(int i=0; i < input_links.size(); i++ )
    {
        //Rule 4
        for(int j=0; j < input_links[i]->visual_array.size(); j++ ) {
            if( input_links[i]->visual_array[j]->geometry->type == Geometry::MESH ) {
                (boost::static_pointer_cast<urdf::Mesh>(input_links[i]->visual_array[j]->geometry))->filename = rule4_prefix + (boost::static_pointer_cast<urdf::Mesh>(input_links[i]->visual_array[j]->geometry))->filename;
            }
        }

        for(int j=0; j < input_links[i]->collision_array.size(); j++ ) {
            if( input_links[i]->collision_array[j]->geometry->type == Geometry::MESH ) {
                (boost::static_pointer_cast<urdf::Mesh>(input_links[i]->collision_array[j]->geometry))->filename = rule4_prefix + (boost::static_pointer_cast<urdf::Mesh>(input_links[i]->collision_array[j]->geometry))->filename;
            }
        }
        std::cout << "urdf_gazebo_cleanup: Rule 4 applied successfully with prefix " <<  rule4_prefix << " on link " << input_links[i]->name << " ( i : " << i << " ) "  << std::endl;
    }

    return true;
}
