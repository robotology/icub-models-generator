#include "urdf_sdf_from_dh_utils.h"



bool addGazeboYarpPluginsControlboardToSDF(sdf::ElementPtr model, std::string part_name, std::string robot_name)
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
    yarpConfigurationFile->AddValue("string","model://icub/conf/gazebo_icub_"+part_name+".ini",false);

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
bool addGazeboYarpPluginsControlboardToSDF(sdf::SDFPtr icub_sdf, std::string part_name, std::string robot_name)
{
    /// \todo add check on parts
    return addGazeboYarpPluginsControlboardToSDF(icub_sdf->Root()->GetElement("model"),part_name,robot_name);
}

/**
 *
 * @param sensor: can be only imu_sensor
 */
bool addGazeboYarpPluginsIMUToSDF(sdf::SDFPtr icub_sdf, std::string sensor_name, std::string robot_name)
{
    assert( icub_sdf->Root()->HasElement("model"));
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
        sdf::ElementPtr model_elem = icub_sdf->Root()->GetElement("model");
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
                yarpConfigurationFile->AddValue("string","model://icub/conf/gazebo_icub_inertial.ini",false);
                sdf::ElementPtr pose = sensor->AddElement("pose");
                pose->AddValue("string","0.0185 -0.1108 0.0066 1.5708 -0 0",false);
            }
        }
        return true;
    } else {
        return false;
    }
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
 * Add the force/torque sensor description to the URDF (using gazebo extentions)
 * @param sensor_name: left_arm,right_arm,left_leg,right_leg,left_foot,right_foot
 */
bool addGazeboYarpPluginsFTToURDF(TiXmlDocument* urdf_doc, std::string sensor_name)
{
    // FT sensors extracted from iDyn are defined with respect to child link reference frame <frame>child</frame>
    // and they are measuring the wrench that the child link applies on the parent link <measure_direction>child_to_parent</measure_direction>

    //Get the URDF joint that models the FT sensor
    std::string ft_sensor_joint = getFTJointName(sensor_name);

    if( ft_sensor_joint == ERROR_FT_SENSOR_JOINT ) {
        return false;
    }

    /*
       <gazebo reference="l_foot_ft_sensor">
        <sensor name='left_foot_ft' type='force_torque'>
        <force_torque>
           <frame>child</frame>
           <measure_direction>child_to_parent</measure_direction>
        </force_torque>
        <always_on>1</always_on>
       <update_rate>100</update_rate>
       </sensor>
       </gazebo>
     */

    TiXmlElement * robot_element = urdf_doc->FirstChildElement( "robot" );

    TiXmlElement * gazebo_element = new TiXmlElement( "gazebo" );
    robot_element->LinkEndChild(gazebo_element);

    gazebo_element->SetAttribute("reference",ft_sensor_joint.c_str());

    TiXmlElement * sensor_element = new TiXmlElement( "sensor" );
    gazebo_element->LinkEndChild(sensor_element);

    std::string sdf_sensor_name = sensor_name+"_ft";
    sensor_element->SetAttribute("name",sdf_sensor_name.c_str());
    sensor_element->SetAttribute("type","force_torque");

    TiXmlElement * force_torque_element = new TiXmlElement( "force_torque" );
    sensor_element->LinkEndChild(force_torque_element);

    TiXmlElement * frame_element = new TiXmlElement( "frame" );
    force_torque_element->LinkEndChild(frame_element);

    TiXmlText * frame_text = new TiXmlText( "child" );
    frame_element->LinkEndChild(frame_text);

    TiXmlElement * measure_direction_element = new TiXmlElement( "measure_direction" );
    force_torque_element->LinkEndChild(measure_direction_element);

    TiXmlText * measure_direction_text = new TiXmlText( "child_to_parent" );
    measure_direction_element->LinkEndChild(measure_direction_text);

    TiXmlElement * update_rate_element = new TiXmlElement( "update_rate" );
    sensor_element->LinkEndChild(update_rate_element);

    TiXmlText * update_rate_text = new TiXmlText( "100" );
    update_rate_element->LinkEndChild(update_rate_text);

    TiXmlElement * always_on_element = new TiXmlElement( "always_on" );
    sensor_element->LinkEndChild(always_on_element);

    TiXmlText * always_on_text = new TiXmlText( "1" );
    always_on_element->LinkEndChild(always_on_text);

    return true;
}

bool add_iDynTreeFTToURDF(TiXmlDocument* urdf_doc, std::string sensor_name)
{
    // FT sensors extracted from iDyn are defined with respect to child link reference frame <frame>child</frame>
    // and they are measuring the wrench that the child link applies on the parent link <measure_direction>child_to_parent</measure_direction>

    //Get the URDF joint that models the FT sensor
    std::string ft_sensor_joint = getFTJointName(sensor_name);

    if( ft_sensor_joint == ERROR_FT_SENSOR_JOINT ) {
        return false;
    }

    /*
        <sensor name="l_leg_ft_sensor" type="force_torque">
            <parent joint="l_leg_ft_sensor"/>
            <force_torque>
                <frame>child</frame>
                <measure_direction>child_to_parent</measure_direction>
            </force_torque>
        </sensor>
     */

    TiXmlElement * robot_element = urdf_doc->FirstChildElement( "robot" );

    TiXmlElement * sensor_element = new TiXmlElement( "sensor" );
    robot_element->LinkEndChild(sensor_element);

    sensor_element->SetAttribute("type","force_torque");
    sensor_element->SetAttribute("name",ft_sensor_joint.c_str());

    sensor_element->SetAttribute("type","force_torque");

    TiXmlElement * parent_element = new TiXmlElement( "parent" );
    sensor_element->LinkEndChild(parent_element);

    parent_element->SetAttribute("joint",ft_sensor_joint);

    TiXmlElement * force_torque_element = new TiXmlElement( "force_torque" );
    sensor_element->LinkEndChild(force_torque_element);

    TiXmlElement * frame_element = new TiXmlElement( "frame" );
    force_torque_element->LinkEndChild(frame_element);

    TiXmlText * frame_text = new TiXmlText( "child" );
    frame_element->LinkEndChild(frame_text);

    TiXmlElement * measure_direction_element = new TiXmlElement( "measure_direction" );
    force_torque_element->LinkEndChild(measure_direction_element);

    TiXmlText * measure_direction_text = new TiXmlText( "child_to_parent" );
    measure_direction_element->LinkEndChild(measure_direction_text);


    return true;
}

void AddElementAndSetValue(sdf::ElementPtr parent, std::string childType, std::string value_type, std::string value)
{
    sdf::ElementPtr child = parent->AddElement(childType);
    child->AddValue(value_type,value,false);
}

bool addGazeboSurfaceFrictionInformationToCollisionSDF(sdf::ElementPtr collision_sdf)
{
        /*
        From iCub hand modified urdf,
        commit https://github.com/robotology-playground/icub-gazebo/commit/7cd9ee3059fef2344f9029bffa3a8b2b8895c059

        <collision name='l_foot_collision'>
                <!-- <pose>3.95315e-11 4.452e-07 -5.14839e-06 8.98039e-06 -1.36732e-05 -1.01971e-06</pose> -->
           <pose>-3e-02 0 3.5e-2 0 0 0</pose>
                <geometry>
<!--                     <mesh>
                        <scale>1 1 1</scale>
                        <uri>model://icub/meshes/collision/icub_simple_collision_l_foot.dae</uri>
                    </mesh> -->
                   <box>
                        <size>0.03 0.07 0.15</size>
                    </box>
                </geometry>
                <!-- <max_contacts> 10 </max_contacts> -->
               <max_contacts> 4 </max_contacts>
                <surface>
                    <contact>
                        <ode>
                            <soft_erp>  0.200  </soft_erp>
                            <soft_cfm>  0.000  </soft_cfm>
                            <kp>        100000.000  </kp>
                            <kd>        1.0     </kd>
                            <max_vel>   100.00  </max_vel>
                            <min_depth> 0.0010  </min_depth>
                        </ode>
                    </contact>
                    <friction>
                        <ode>
                            <mu>  1  </mu>
                            <mu2> 1  </mu2>
                            <fdir1>0.000000 0.000000 0.000000</fdir1>
                            <slip1> 0.00000 </slip1>
                            <slip2> 0.00000 </slip2>
                        </ode>
                    </friction>
                    <bounce>
                        <restitution_coefficient>0.000000</restitution_coefficient>
                        <threshold>100000.000000</threshold>
                    </bounce>
                </surface>
            </collision>

     */

    // handle max_contacts
    sdf::ElementPtr max_contacts = collision_sdf->AddElement("max_contacts");
    max_contacts->AddValue("int","4",false);

    // handle surface
    sdf::ElementPtr surface = collision_sdf->AddElement("surface");

    // contact group
    sdf::ElementPtr contact = surface->AddElement("contact");
    sdf::ElementPtr ode = contact->AddElement("ode");
    AddElementAndSetValue(ode,"soft_erp","double","0.2");
    AddElementAndSetValue(ode,"soft_cfm","double","0.0");
    AddElementAndSetValue(ode,"kp","double","100000.000");
    AddElementAndSetValue(ode,"kd","double","1.0");
    AddElementAndSetValue(ode,"max_vel","double","100.00");
    AddElementAndSetValue(ode,"min_depth","double","0.0010");

    // friction group
    sdf::ElementPtr friction = surface->AddElement("friction");
    sdf::ElementPtr friction_ode = friction->AddElement("ode");
    AddElementAndSetValue(friction_ode,"mu","double","1.000000");
    AddElementAndSetValue(friction_ode,"mu2","double","1.000000");
    AddElementAndSetValue(friction_ode,"fdir1","vector3","0.0 0.0 0.0");
    AddElementAndSetValue(friction_ode,"slip1","double","0.000000");
    AddElementAndSetValue(friction_ode,"slip2","double","0.000000");

    // bounce group
    sdf::ElementPtr bounce = surface->AddElement("bounce");
    AddElementAndSetValue(bounce,"restitution_coefficient","double","0.000000");
    AddElementAndSetValue(bounce,"threshold","double","100000.000000");

    return true;
}


/**
 * For a given link add surface friction informations to the SDF
 *
 */
bool addGazeboSurfaceFrictionInformationToSDF(sdf::SDFPtr icub_sdf, std::string link_name)
{

    sdf::ElementPtr model_elem = icub_sdf->Root()->GetElement("model");

    for (sdf::ElementPtr link = model_elem->GetElement("link"); link; link = link->GetNextElement("link"))
    {
        if( link->GetAttribute("name")->GetAsString() == link_name )
        {
            for (sdf::ElementPtr collision = link->GetElement("collision"); collision; collision = link->GetNextElement("collision"))
            {
                addGazeboSurfaceFrictionInformationToCollisionSDF(collision);
            }
        }
    }

    return true;
}



bool addGazeboYarpPluginsFTToSDF(sdf::ElementPtr joint_element, std::string sensor_name, std::string robot_name)
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
                    yarpConfigurationFile->AddValue("string","model://icub/conf/FT/gazebo_icub_"+sensor_name+"_ft.ini",false);

    return true;
}


/**
 * Add the force/torque sensor description (with the relative gazebo_yarp_plugin)
 * @param sensor_name: left_arm,right_arm,left_leg,right_leg,left_foot,right_foot
 */
bool addGazeboYarpPluginsFTToSDF(sdf::SDFPtr icub_sdf, std::string sensor_name, std::string robot_name)
{
    bool ret;

    sdf::ElementPtr model_elem = icub_sdf->Root()->GetElement("model");

    //Get the gazebo joint that models the FT sensor
    std::string ft_sensor_joint = getFTJointName(sensor_name);

    if( ft_sensor_joint == ERROR_FT_SENSOR_JOINT ) {
        return false;
    }

    for (sdf::ElementPtr joint = model_elem->GetElement("joint"); joint; joint = joint->GetNextElement("joint")) {
        if( joint->GetAttribute("name")->GetAsString() == ft_sensor_joint ) {
            ret = addGazeboYarpPluginsFTToSDF(joint,sensor_name,robot_name);
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
    sdf::ElementPtr model_elem = icub_sdf->Root()->GetElement("model");

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

bool substituteCollisionWithBoxInCollisionSDF(sdf::ElementPtr collision,
                                     std::string new_box_collision_pose,
                                     std::string box_size)
{
    sdf::ElementPtr geometry = collision->GetElement("geometry");

    // Remove existing mesh
    sdf::ElementPtr mesh = geometry->GetElement("mesh");
    geometry->RemoveChild(mesh);

    // Overwrite existing pose tag
    sdf::ElementPtr pose = collision->GetElement("pose");
    pose->Set(new_box_collision_pose);

    // Add a new box
    sdf::ElementPtr box = geometry->AddElement("box");
    sdf::ElementPtr size = box->GetElement("size");
    size->Set(box_size);

    return true;
}

bool substituteCollisionWithBoxInSDF(sdf::SDFPtr icub_sdf,
                                     std::string link_name,
                                     std::string collision_name,
                                     std::string new_box_collision_pose,
                                     std::string box_size)
{
    sdf::ElementPtr model_elem = icub_sdf->Root()->GetElement("model");

    for (sdf::ElementPtr link = model_elem->GetElement("link"); link; link = link->GetNextElement("link"))
    {
        if( link->GetAttribute("name")->GetAsString() == link_name )
        {
            for (sdf::ElementPtr collision = link->GetElement("collision"); collision; collision = link->GetNextElement("collision"))
            {
                if( collision->GetAttribute("name")->GetAsString() == collision_name )
                {
                    substituteCollisionWithBoxInCollisionSDF(collision,new_box_collision_pose,box_size);
                }
            }
        }
    }

    return true;
}

bool generate_model_config_file(std::string robot_name, std::string gazebo_robot_model_directory)
{

    std::ofstream model_config_file;
    std::string model_config_file_name = gazebo_robot_model_directory+"model.config";
    model_config_file.open(model_config_file_name.c_str());
    model_config_file << "<?xml version=\"1.0\"?>\n<model>\n<name>";
    model_config_file << robot_name;
    model_config_file << "</name><version>1.0</version><sdf version='1.4'>icub.sdf</sdf>\n</model>\n";
    model_config_file.close();

    return true;
}

std::string get_gazebo_model_directory(std::string root_directory)
{
    return root_directory+"gazebo_models/";
}

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
                               urdf::ModelInterfaceSharedPtr  urdf_file,
                               std::string outputfilename
                             )
{

    std::cout << "Generating iCub model for " << iCub_name << " (head: " << head_version << " , legs: " << legs_version << " , feet: " << feet_version << " )" << std::endl;
    if( feet_version == 2 ) { std::cout << "Generating FT sensor in the feet" << std::endl; }

    std::string paris_directory = data_directory+"urdf_paris/";

    std::string paris_subdirectory;
    if( simple_meshes ) {
        paris_subdirectory = "icub_simple";
    } else {
        paris_subdirectory = "icub";
    }

    //////////////////////////////////
    //Generating urdf from iDyn
    //////////////////////////////////
    iCub::iDyn::version_tag icub_type;

    icub_type.head_version = head_version;
    icub_type.legs_version = legs_version;


    iCub::iDyn::iCubWholeBody icub_idyn(icub_type);

    KDL::Tree icub_kdl;

    KDL::JntArray dummy1,dummy2;
    bool addRootWeight = true;
    bool debugFlag         = false;
    bool ft_feet = false;

    if( feet_version == 2 )
    {
        ft_feet = true;
    }

    if( ! toKDL(icub_idyn,icub_type,icub_kdl,dummy1,dummy2,iCub::iDynTree::SKINDYNLIB_SERIALIZATION,ft_feet,addRootWeight,debugFlag,is_iCubParis02,is_icubGazeboSim) ) {
        std::cerr << "Fatal error in iDyn - KDL conversion" << std::endl;
        return false;
    }

    urdf::ModelInterfaceSharedPtr  urdf_idyn(new urdf::ModelInterface);

    //std::cout << "iCub KDL::Tree: " << std::endl;
    //std::cout << icub_kdl << std::endl;

    if( ! iDynTree::treeToUrdfModel(icub_kdl,"iCub",*urdf_idyn) ) {
        std::cerr << "Fatal error in KDL - URDF conversion" << std::endl;
        return false;
    }


    //////////////////////////////////////////////////////////////////
    //// Getting meshes and limits from urdf paris files
    //////////////////////////////////////////////////////////////////
    urdf::ModelInterfaceSharedPtr  urdf_paris;

    std::string filename_urdf_paris = paris_directory+paris_subdirectory+"/icub.xml";


    std::ifstream t_um(filename_urdf_paris.c_str());
    std::stringstream buffer_um;
    buffer_um << t_um.rdbuf();
    urdf_paris = urdf::parseURDF(buffer_um.str());

    std::cout << "Loading file " << filename_urdf_paris << std::endl;


    bool ret_ok = urdf_import_limits(urdf_idyn,urdf_paris);

    if( !ret_ok ) {
        std::cerr << "Fatal error in importing limits from " << filename_urdf_paris  << std::endl;
        return false;
    }

    ret_ok = urdf_import_meshes(urdf_idyn,urdf_paris);

    if( !ret_ok ) {
        std::cerr << "Fatal error in importing meshes from " << filename_urdf_paris  << std::endl;
        return false;
    }

    //Add damping
    double damping_ie_viscous = 1.0;
    double friction_ie_coulomb = 0.0;
    ret_ok = urdf_set_friction_parameters(urdf_idyn,damping_ie_viscous,friction_ie_coulomb);

    if( !ret_ok ) {
        std::cerr << "Fatal error in setting friction parameters"  << std::endl;
        return false;
    }

    /////////////////////////////////////////////////////////////////////////
    ///// Exporting "normal" urdf file
    /////////////////////////////////////////////////////////////////////////
    TiXmlDocument* xml_doc;

    xml_doc = exportURDF(urdf_idyn);

    // Add sensors
    bool ret=true;
    ret = ret && addGazeboYarpPluginsFTToURDF(xml_doc,"left_arm");
    ret = ret && addGazeboYarpPluginsFTToURDF(xml_doc,"right_arm");
    ret = ret && addGazeboYarpPluginsFTToURDF(xml_doc,"left_leg");
    ret = ret && addGazeboYarpPluginsFTToURDF(xml_doc,"right_leg");
    if( ft_feet ) {
        std::cerr << "Adding feet FT sensors" << std::endl;
        ret = ret && addGazeboYarpPluginsFTToURDF(xml_doc,"left_foot");
        ret = ret && addGazeboYarpPluginsFTToURDF(xml_doc,"right_foot");
    }

    ret = ret && add_iDynTreeFTToURDF(xml_doc,"left_arm");
    ret = ret && add_iDynTreeFTToURDF(xml_doc,"right_arm");
    ret = ret && add_iDynTreeFTToURDF(xml_doc,"left_leg");
    ret = ret && add_iDynTreeFTToURDF(xml_doc,"right_leg");
    if( ft_feet ) {
        ret = ret && add_iDynTreeFTToURDF(xml_doc,"left_foot");
        ret = ret && add_iDynTreeFTToURDF(xml_doc,"right_foot");
    }

    if( !ret ) { std::cerr << "Problem in adding ft sensors to the URDF file" << std::endl; return false; }

    if( ! xml_doc->SaveFile(outputfilename) ) {
        std::cerr << "Fatal error in URDF xml saving filename " << outputfilename  << std::endl;
        return false;
    }

    urdf_file = urdf_idyn;

    return true;
}

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
                               urdf::ModelInterfaceSharedPtr  urdf_idyn,
                               std::string outputfilename
                            )
{
    std::string gazebo_mesh_model_name = "icub";
    std::string gazebo_uri_prefix = "model://icub/";
    std::string gazebo_sdf_filename = outputfilename;

    //////////////////////////////////////////////////////////////////////////
    ///// Creating gazebo file
    /////////////////////////////////////////////////////////////////////////
    if( ! urdf_gazebo_cleanup_regularize_masses(urdf_idyn,mass_epsilon,inertia_epsilon) ) { std::cerr << "Error in regularizing masses root " << std::endl; return false; }

    //////////////////////////////////////////////////////////////////////////
    ////// Creating URDF file coherent with gazebo workarounds for use in iDynTree
    ///////////////////////////////////////////////////////////////////////

    /*
    TiXmlDocument * xml_doc = exportURDF(urdf_idyn);

    if( ! xml_doc->SaveFile(outputfilename) ) {
        std::cerr << "Fatal error in URDF xml saving" << std::endl;
        return false;
    }*/

    if( ! urdf_gazebo_cleanup_remove_massless_root(urdf_idyn) ) { std::cerr << "Error in removing massless root " << std::endl; return false; }
    //printTree(urdf_idyn->getRoot());
    //printJoints(urdf_idyn);
    if( ! urdf_gazebo_cleanup_remove_frames(urdf_idyn) ) { std::cerr << "Error in removing frames " << std::endl; return false; }

    if( ! noFTsimulation )
    {
        if( ! urdf_gazebo_cleanup_transform_FT_sensors(urdf_idyn) ) { std::cerr << "Error in transforming FT junctions " << std::endl; return false; }
    }

    if( ! urdf_gazebo_cleanup_regularize_masses(urdf_idyn,mass_epsilon,inertia_epsilon) ) { std::cerr << "Error in regularizing masses root " << std::endl; return false; }
    if( ! urdf_gazebo_cleanup_add_model_uri(urdf_idyn,gazebo_uri_prefix) ) { std::cerr << "Error in adding model URIs " << std::endl; return false; }

    TiXmlDocument * xml_doc = xml_doc = exportURDF(urdf_idyn);

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

    //system(gazebo_conversion_command.c_str());
    //if( !system(gazebo_conversion_command.c_str()) ) { std::cerr << "Error in urdf - sdf conversion" << std::endl; return false; }

    //\todo: adding plugins to generated sdf
    sdf::SDFPtr icub_sdf(new sdf::SDF());
    sdf::init(icub_sdf);

    if( ! sdf::readString(urdf_for_gazebo_conversion_string,icub_sdf) ) { std::cerr << "Problem in reading SDF file" << std::endl; return false; }

    if( ! icub_sdf->Root()->HasElement("model") ) { std::cerr << "Problem in parsing SDF dom" << std::endl; return false; }

    // Substitute foot collisions with a box
    if( ! noFTsimulation )
    {
        substituteCollisionWithBoxInSDF(icub_sdf,"l_foot","l_foot_collision","0.03 0 -0.01 0 0 0","0.15 0.07 0.03");
        substituteCollisionWithBoxInSDF(icub_sdf,"r_foot","r_foot_collision","0.03 0 -0.01 0 0 0","0.15 0.07 0.03");
    }
    else
    {
        substituteCollisionWithBoxInSDF(icub_sdf,"l_ankle_2","l_ankle_2_collision","0.03 0 0.026 0 -0 0","0.15 0.07 0.03");
        substituteCollisionWithBoxInSDF(icub_sdf,"r_ankle_2","r_ankle_2_collision","0.03 0 0.026 0 -0 0","0.15 0.07 0.03");
    }

    //Adding IMU sensor (in all robots)
    /// \todo add difference between head V1 and head V2
    if( ! addGazeboYarpPluginsIMUToSDF(icub_sdf,"imu_sensor",iCub_name) ) { std::cerr << "Problem in adding imu sensor" << std::endl; return false; }

    //Adding FT sensors (in all robots)
    bool ret=true;
    ret = ret && addGazeboYarpPluginsFTToSDF(icub_sdf,"left_arm",iCub_name);
    ret = ret && addGazeboYarpPluginsFTToSDF(icub_sdf,"right_arm",iCub_name);
    ret = ret && addGazeboYarpPluginsFTToSDF(icub_sdf,"left_leg",iCub_name);
    ret = ret && addGazeboYarpPluginsFTToSDF(icub_sdf,"right_leg",iCub_name);
    if( feet_version == 2 ) {
        std::cerr << "Adding feet FT sensors" << std::endl;
        ret = ret && addGazeboYarpPluginsFTToSDF(icub_sdf,"left_foot",iCub_name);
        ret = ret && addGazeboYarpPluginsFTToSDF(icub_sdf,"right_foot",iCub_name);
    }
    if( !ret ) { std::cerr << "Problem in adding ft sensors" << std::endl; return false; }

    //Adding controlboards (in all robots, in the future add differences)
    ret = ret && addGazeboYarpPluginsControlboardToSDF(icub_sdf,"torso",iCub_name);
    if( !ret ) { std::cerr << "Problem in adding torso controlboard" << std::endl; return false; }
    ret = ret && addGazeboYarpPluginsControlboardToSDF(icub_sdf,"head",iCub_name);
    if( !ret ) { std::cerr << "Problem in adding head controlboard" << std::endl; return false; }
    ret = ret && addGazeboYarpPluginsControlboardToSDF(icub_sdf,"left_arm",iCub_name);
    if( !ret ) { std::cerr << "Problem in adding left_arm controlboards" << std::endl; return false; }
    ret = ret && addGazeboYarpPluginsControlboardToSDF(icub_sdf,"right_arm",iCub_name);
    if( !ret ) { std::cerr << "Problem in adding right_arm controlboards" << std::endl; return false; }
    ret = ret && addGazeboYarpPluginsControlboardToSDF(icub_sdf,"left_leg",iCub_name);
    if( !ret ) { std::cerr << "Problem in adding left_leg controlboards" << std::endl; return false; }
    ret = ret && addGazeboYarpPluginsControlboardToSDF(icub_sdf,"right_leg",iCub_name);
    if( !ret ) { std::cerr << "Problem in adding right_leg controlboards" << std::endl; return false; }

    //Adding parameters for properly handling of the contacts
    ret = ret && addGazeboSurfaceFrictionInformationToSDF(icub_sdf,"l_foot");
    if( !ret ) { std::cerr << "Problem in adding contact properties" << std::endl; return false; }
    ret = ret && addGazeboSurfaceFrictionInformationToSDF(icub_sdf,"r_foot");
    if( !ret ) { std::cerr << "Problem in adding contact properties" << std::endl; return false; }

    //Adding pose to avoid intersection with ground
    //<pose>0 0 0.70 0 0 0 </pose>
    sdf::ElementPtr pose = icub_sdf->Root()->GetElement("model")->AddElement("pose");
    pose->AddValue("string","0 0 0.70 0 0 0",false);

    icub_sdf->Write(gazebo_sdf_filename);

    return true;
}


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
                         int head_version,
                         int legs_version,
                         int feet_version,
                         bool is_iCubParis02,
                         bool is_icubGazeboSim,
                         bool simple_meshes,
                         std::string data_directory,
                         double mass_epsilon,
                         double inertia_epsilon,
                         bool noFTsimulation)
{
    bool ok = true;
    urdf::ModelInterfaceSharedPtr  urdf_file;
    ok = ok && generate_iCub_urdf_model(iCub_name,head_version,legs_version,feet_version,is_iCubParis02,is_icubGazeboSim,simple_meshes,data_directory,mass_epsilon,inertia_epsilon,noFTsimulation,urdf_file,root_directory+"/icub.urdf");
    ok = ok && generate_iCub_sdf_model(iCub_name,head_version,legs_version,feet_version,is_iCubParis02,is_icubGazeboSim,simple_meshes,data_directory,mass_epsilon,inertia_epsilon,noFTsimulation,urdf_file);
    return true;
}*/


bool generate_gazebo_database(const std::vector<std::string> & robot_names, const std::string root_directory)
{
    std::string gazebo_model_directory = get_gazebo_model_directory(root_directory);

    std::ofstream database_file;
    std::string database_file_name = gazebo_model_directory+"database.config";

    std::cerr << "generating gazebo database at : " << std::endl;;


    database_file.open(database_file_name.c_str());
    database_file << "<?xml version='1.0'?>\n<database>\n";
    database_file << "<name>icub-model-generator Database</name>\n";
    database_file << "<models>";
    for(int i=0; i < robot_names.size(); i++ )
    {
        database_file << "<uri>file://"+robot_names[i]+"</uri>\n";
    }

    database_file << "</models>\n</database>\n" << std::endl;
    database_file.close();

    return true;
}
