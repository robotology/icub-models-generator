
#include <iDynTree/Core/Axis.h>
#include <iDynTree/KinDynComputations.h>
#include <iDynTree/Model/JointState.h>
#include <iDynTree/Model/Indices.h>
#include <iDynTree/Model/Model.h>
#include <iDynTree/Model/Traversal.h>
#include <iDynTree/Model/RevoluteJoint.h>
#include <iDynTree/ModelIO/ModelLoader.h>
#include <iDynTree/Sensors/Sensors.h>
#include <iDynTree/Sensors/SixAxisForceTorqueSensor.h>


#include <yarp/os/Property.h>
#include <cmath>
#include <cstdlib>

inline bool checkDoubleAreEqual(const double & val1,
                                const double & val2,
                                const double tol)
{
    if( std::fabs(val1-val2) > tol )
    {
        return false;
    }

    return true;
}


template<typename VectorType1, typename VectorType2>
bool checkVectorAreEqual(const VectorType1 & dir1,
                         const VectorType2 & dir2,
                         const double tol)
{
    if( dir1.size() != dir2.size() )
    {
        return false;
    }

    for(int i=0; i < dir1.size(); i++)
    {
        if( std::fabs(dir1(i)-dir2(i)) > tol )
        {
            return false;
        }
    }
    return true;
}

template<typename MatrixType1, typename MatrixType2>
bool checkMatrixAreEqual(const MatrixType1 & mat1,
                         const MatrixType2 & mat2,
                         const double tol)
{
    if( mat1.rows() != mat2.rows() ||
        mat1.cols() != mat2.cols() )
    {
        return false;
    }

    for(int i=0; i < mat1.rows(); i++)
    {
        for(int j=0; j < mat1.cols(); j++ )
        {
            if( std::fabs(mat1(i,j)-mat2(i,j)) > tol )
            {
                return false;
            }
        }
    }
    return true;
}

bool checkTransformAreEqual(const iDynTree::Transform & t1,
                            const iDynTree::Transform & t2,
                            const double tol)
{
    return checkMatrixAreEqual(t1.getRotation(),t2.getRotation(),tol) &&
           checkVectorAreEqual(t1.getPosition(),t2.getPosition(),tol);
}


bool getAxisInRootLink(iDynTree::KinDynComputations & comp,
                                 const std::string jointName,
                                 iDynTree::Axis & axisInRootLink)
{
    iDynTree::LinkIndex rootLinkIdx = comp.getFrameIndex("root_link");

    if( rootLinkIdx == iDynTree::FRAME_INVALID_INDEX )
    {
        std::cerr << "icub-model-test error: impossible to find root_link in model" << std::endl;
        return false;
    }

    iDynTree::JointIndex jntIdx = comp.getRobotModel().getJointIndex(jointName);

    if( jntIdx == iDynTree::JOINT_INVALID_INDEX )
    {
        std::cerr << "icub-model-test error: impossible to find " << jointName << " in model" << std::endl;
        return false;
    }

    iDynTree::LinkIndex childLinkIdx = comp.getRobotModel().getJoint(jntIdx)->getSecondAttachedLink();

    // Check that the joint are actually revolute as all the joints in iCub
    const iDynTree::RevoluteJoint * revJoint = dynamic_cast<const iDynTree::RevoluteJoint *>(comp.getRobotModel().getJoint(jntIdx));

    if( !revJoint )
    {
        std::cerr << "icub-model-test error: " << jointName << " is not revolute " << std::endl;
        return false;
    }

    if( !revJoint->hasPosLimits() )
    {
        std::cerr << "icub-model-test error: " << jointName << " is a continous joint" << std::endl;
        return false;
    }

    axisInRootLink = comp.getRelativeTransform(rootLinkIdx,childLinkIdx)*(revJoint->getAxis(childLinkIdx));

    return true;
}

bool checkBaseLink(iDynTree::KinDynComputations & comp)
{
    iDynTree::LinkIndex rootLinkIdx = comp.getFrameIndex("root_link");

    if( rootLinkIdx == iDynTree::FRAME_INVALID_INDEX )
    {
        std::cerr << "icub-model-test error: impossible to find root_link in model" << std::endl;
        return false;
    }

    iDynTree::LinkIndex base_linkIdx = comp.getFrameIndex("base_link");

    if( rootLinkIdx == iDynTree::FRAME_INVALID_INDEX )
    {
        std::cerr << "icub-model-test error: impossible to find base_link in model" << std::endl;
        return false;
    }

    if( comp.getRobotModel().getFrameLink(base_linkIdx) != rootLinkIdx )
    {
        std::cerr << "icub-model-test error: base_link is not attached to root_link" << std::endl;
        return false;
    }

    if( !checkTransformAreEqual(comp.getRobotModel().getFrameTransform(base_linkIdx),iDynTree::Transform::Identity(),1e-6) )
    {
        std::cerr << "icub-model-test error: base_link <---> root_link transform is not an identity" << std::endl;
        return false;
    }

    std::cerr << "icub-model-test : base_link test performed correctly " << std::endl;

    return true;
}

bool checkSolesAreParallel(iDynTree::KinDynComputations & comp)
{
    iDynTree::LinkIndex rootLinkIdx = comp.getFrameIndex("root_link");

    if( rootLinkIdx == iDynTree::FRAME_INVALID_INDEX )
    {
        std::cerr << "icub-model-test error: impossible to find root_link in model" << std::endl;
        return false;
    }

    iDynTree::LinkIndex l_sole = comp.getFrameIndex("l_sole");

    if( rootLinkIdx == iDynTree::FRAME_INVALID_INDEX )
    {
        std::cerr << "icub-model-test error: impossible to find frame l_sole in model" << std::endl;
        return false;
    }

    iDynTree::LinkIndex r_sole = comp.getFrameIndex("r_sole");

    if( rootLinkIdx == iDynTree::FRAME_INVALID_INDEX )
    {
        std::cerr << "icub-model-test error: impossible to find frame r_sole in model" << std::endl;
        return false;
    }

    iDynTree::Transform root_H_l_sole = comp.getRelativeTransform(rootLinkIdx,l_sole);
    iDynTree::Transform root_H_r_sole = comp.getRelativeTransform(rootLinkIdx,r_sole);

    // height of the sole should be equal
    double l_sole_height = root_H_l_sole.getPosition().getVal(2);
    double r_sole_height = root_H_r_sole.getPosition().getVal(2);

    if( !checkDoubleAreEqual(l_sole_height,r_sole_height,1e-5) )
    {
        std::cerr << "icub-model-test error: l_sole_height is " << l_sole_height << ", while r_sole_height is " << r_sole_height << " (diff : " << std::fabs(l_sole_height-r_sole_height) <<  " )"  << std::endl;
        return false;
    }

    // x should also be equal
    double l_sole_x = root_H_l_sole.getPosition().getVal(0);
    double r_sole_x = root_H_r_sole.getPosition().getVal(0);

    // The increased threshold is a workaround for https://github.com/robotology/icub-model-generator/issues/125
    if( !checkDoubleAreEqual(l_sole_x,r_sole_x, 2e-4) )
    {
        std::cerr << "icub-model-test error: l_sole_x is " << l_sole_x << ", while r_sole_x is " << r_sole_x << " (diff : " << std::fabs(l_sole_x-r_sole_x) <<  " )"  << std::endl;
        return false;
    }

    // y should be simmetric
    double l_sole_y = root_H_l_sole.getPosition().getVal(1);
    double r_sole_y = root_H_r_sole.getPosition().getVal(1);

    // The increased threshold is a workaround for https://github.com/robotology/icub-model-generator/issues/125
    if( !checkDoubleAreEqual(l_sole_y,-r_sole_y,1e-4) )
    {
        std::cerr << "icub-model-test error: l_sole_y is " << l_sole_y << ", while r_sole_y is " << r_sole_y << " while they should be simmetric (diff : " << std::fabs(l_sole_y+r_sole_y) <<  " )"  << std::endl;
        return false;
    }


    std::cerr << "icub-model-test : sole are parallel test performed correctly " << std::endl;

    return true;
}



bool checkAxisDirectionsV2(iDynTree::KinDynComputations & comp)
{
    // Check axis for the first thing
    // see https://github.com/robotology/codyco-superbuild/issues/55#issuecomment-227872325 and
    // http://wiki.icub.org/wiki/ICub_Model_naming_conventions    // Check axis for the first thing
    // see https://github.com/robotology/codyco-superbuild/issues/55#issuecomment-227872325 and
    // http://wiki.icub.org/wiki/ICub_Model_naming_conventions

    std::vector<std::string> axisNames;
    std::vector<iDynTree::Direction> expectedDirectionInRootLink;

    axisNames.push_back("torso_pitch");
    expectedDirectionInRootLink.push_back(iDynTree::Direction(0.0,-1.0,0.0));
    axisNames.push_back("torso_roll");
    expectedDirectionInRootLink.push_back(iDynTree::Direction(1.0,0.0,0.0));
    axisNames.push_back("torso_yaw");
    expectedDirectionInRootLink.push_back(iDynTree::Direction(0.0,0.0,-1.0));
    axisNames.push_back("l_wrist_yaw");
    expectedDirectionInRootLink.push_back(iDynTree::Direction(0.0,-1.0,0.0));
    axisNames.push_back("r_wrist_yaw");
    expectedDirectionInRootLink.push_back(iDynTree::Direction(0.0,-1.0,0.0));

    for(int i=0; i < axisNames.size(); i++)
    {
        std::string axisToCheck = axisNames[i];
        iDynTree::Axis axisInRootLink;
        iDynTree::Direction expectedDirection = expectedDirectionInRootLink[i];
        bool getAxisOk = getAxisInRootLink(comp,axisToCheck,axisInRootLink);

        if( !getAxisOk )
        {
            return false;
        }

        if( !checkVectorAreEqual(axisInRootLink.getDirection(),expectedDirection,1e-5) )
        {
            std::cerr << "icub-model-test error:" << axisToCheck << " got direction of " << axisInRootLink.getDirection().toString()
                  << " instead of expected " << expectedDirection.toString() << std::endl;
            return false;
        }
    }

    std::cerr << "icub-model-test : checkAxisDirectionsV2 test performed correctly " << std::endl;

    return true;
}


/**
 * All the iCub have a odd and not null number of F/T sensors.
 */
bool checkFTSensorsAreOddAndNotNull(iDynTree::ModelLoader & mdlLoader)
{
    int nrOfFTSensors = mdlLoader.sensors().getNrOfSensors(iDynTree::SIX_AXIS_FORCE_TORQUE);

    if( nrOfFTSensors == 0 )
    {
        std::cerr << "icub-model-test error: no F/T sensor found in the model" << std::endl;
        return false;
    }

    if( nrOfFTSensors % 2 == 0 )
    {
        std::cerr << "icub-model-test : even number of F/T sensor found in the model" << std::endl;
        return false;
    }

    std::cerr << "icub-model-test : checkFTSensorsAreOddAndNotNull test performed correctly " << std::endl;

    return true;
}

/**
 * All the iCub have a even and not null number of F/T sensors.
 */
bool checkFTSensorsAreEvenAndNotNull(iDynTree::ModelLoader & mdlLoader)
{
    int nrOfFTSensors = mdlLoader.sensors().getNrOfSensors(iDynTree::SIX_AXIS_FORCE_TORQUE);

    if( nrOfFTSensors == 0 )
    {
        std::cerr << "icub-model-test error: no F/T sensor found in the model" << std::endl;
        return false;
    }

    if( nrOfFTSensors % 2 == 1 )
    {
        std::cerr << "icub-model-test : odd number of F/T sensor found in the model" << std::endl;
        return false;
    }

    std::cerr << "icub-model-test : checkFTSensorsAreEvenAndNotNull test performed correctly " << std::endl;

    return true;
}


bool checkFTSensorIsCorrectlyOriented(iDynTree::KinDynComputations & comp,
                                      const iDynTree::Rotation& expected,
                                      const std::string& sensorName)
{
    // Depending on the icub model, the sensor could be absent
    if (!comp.model().isFrameNameUsed(sensorName))
    {
        return true;
    }

    iDynTree::Rotation actual = comp.getRelativeTransform("root_link", sensorName).getRotation();

    if (!checkMatrixAreEqual(expected, actual, 1e-3))
    {
        std::cerr << "icub-model-test : transform between root_link and " << sensorName << " is not the expected one, test failed." << std::endl;
        std::cerr << "icub-model-test : Expected transform : " << expected.toString() << std::endl;
        std::cerr << "icub-model-test : Actual transform : " << actual.toString() << std::endl;
        return false;
    }
    std::cerr << "icub-model-test : checkFTSensorIsCorrectlyOriented test performed correctly " << std::endl;

    return true;
}


/**
 * Check that the F/T sensor have the correct orientation.
 *
 * See https://github.com/robotology/icub-model-generator/issues/92
 */
bool checkFTSensorsAreCorrectlyOrientedV2(iDynTree::KinDynComputations & comp)
{
    // The rotation of all the F/T sensors in the iCub robot when in zero position are the same
    iDynTree::Rotation rootLink_R_sensorFrameExpected =
        iDynTree::Rotation(-1.0,  0.0,  0.0,
                            0.0,  1.0,  0.0,
                            0.0,  0.0, -1.0);
    // The rotation of the foot F/T sensors in the iCub 2.5 + KIT_007 is different
    iDynTree::Rotation rootLink_R_foot_sensorFrameExpected_kit_007 =
        iDynTree::Rotation(-0.965926, 0.0,  0.258819,
                            0.0,      1.0,  0.0,
                           -0.258819, 0.0, -0.965926);

    bool isModelWitKit007 = checkFTSensorIsCorrectlyOriented(comp, rootLink_R_foot_sensorFrameExpected_kit_007, "l_foot_ft_sensor")
                            && checkFTSensorIsCorrectlyOriented(comp, rootLink_R_foot_sensorFrameExpected_kit_007, "r_foot_ft_sensor");

    bool ok = checkFTSensorIsCorrectlyOriented(comp, rootLink_R_sensorFrameExpected, "l_arm_ft_sensor");
    ok = checkFTSensorIsCorrectlyOriented(comp, rootLink_R_sensorFrameExpected, "r_arm_ft_sensor") && ok;
    ok = checkFTSensorIsCorrectlyOriented(comp, rootLink_R_sensorFrameExpected, "l_leg_ft_sensor") && ok;
    ok = checkFTSensorIsCorrectlyOriented(comp, rootLink_R_sensorFrameExpected, "r_leg_ft_sensor") && ok;
    ok = (checkFTSensorIsCorrectlyOriented(comp, rootLink_R_sensorFrameExpected, "l_foot_ft_sensor") || isModelWitKit007) && ok;
    ok = (checkFTSensorIsCorrectlyOriented(comp, rootLink_R_sensorFrameExpected, "r_foot_ft_sensor") || isModelWitKit007) && ok;

    return ok;
}


bool Model_isFrameNameUsed(const iDynTree::Model& model, const std::string frameName)
{
    for(size_t i=0; i < model.getNrOfLinks(); i++ )
    {
        if( frameName == model.getLinkName(i) )
        {
            return true;
        }
    }

    for(size_t i=model.getNrOfLinks(); i < model.getNrOfFrames(); i++ )
    {

        if( frameName == model.getFrameName(i) )
        {
            return true;
        }
    }

    return false;
}

bool checkFTMeasurementFrameGivenBySensorTagsIsCoherentWithMeasurementFrameGivenByFrame(const std::string& modelPath,
                                                                                        iDynTree::KinDynComputations& comp,
                                                                                        const iDynTree::SensorsList& sensors,
                                                                                        const std::string& sensorName)
{
    // As of mid 2023, for iCub 3 models the frame name is <prefix>_ft, while the sensor name is <prefix>_ft,
    // and the joint name is <prefix>_ft_sensor
    std::string frameName = sensorName;

    //std::cerr << comp.model().toString() << std::endl;

    // Check frame exist
    if (!comp.model().isFrameNameUsed(frameName))
    {
        std::cerr << "icub-model-test : model " << modelPath << " does not contain frame " << frameName << " as expected." << std::endl;
        return false;
    }

    // Check sensors exists
    std::ptrdiff_t sensorIdx;
    if (!sensors.getSensorIndex(iDynTree::SIX_AXIS_FORCE_TORQUE, sensorName, sensorIdx))
    {
        std::cerr << "icub-model-test : model " << modelPath << " does not contain FT sensor " << sensorName << " as expected." <<  std::endl;
        return false;
    }

    // Get root_H_link
    iDynTree::Transform root_H_frame = comp.getRelativeTransform("root_link", frameName);

    // Get root_H_sensor
    iDynTree::SixAxisForceTorqueSensor * sens
            = (::iDynTree::SixAxisForceTorqueSensor *) sensors.getSensor(::iDynTree::SIX_AXIS_FORCE_TORQUE, sensorIdx);
    if (!sens)
    {
        std::cerr << "icub-model-test : model " << modelPath << " error in reading sensor " << sensorName << "." <<  std::endl;
        return false;
    }

    std::string firstLinkName = sens->getFirstLinkName();
    iDynTree::Transform root_H_firstLink = comp.getRelativeTransform("root_link", firstLinkName);
    iDynTree::Transform firstLink_H_sensor;
    bool ok = sens->getLinkSensorTransform(sens->getFirstLinkIndex(), firstLink_H_sensor);

    if (!ok)
    {
        std::cerr << "icub-model-test : model " << modelPath << " error in reading transform of sensor " << sensorName << "." <<  std::endl;
        return false;
    }


    iDynTree::Transform root_H_sensor = root_H_firstLink*firstLink_H_sensor;

    // Check that the two transfom are equal equal
    if (!checkTransformAreEqual(root_H_frame, root_H_sensor, 1e-6))
    {
        std::cerr << "icub-model-test : transform between root_H_frame and root_H_sensor for " << sensorName << " is not the expected one, test failed." << std::endl;
        std::cerr << "icub-model-test : root_H_frame :" << root_H_frame.toString() << std::endl;
        std::cerr << "icub-model-test : root_H_sensor :" << root_H_sensor.toString() << std::endl;
        return false;
    }

    // Beside checking that root_H_frame and root_H_sensor, we should also check that the translation
    // between the child link of FT joint and the FT frame is the zero vector, as as of mid 2023 the SDF
    // standard always assume that the 6D FT measured by the sensor is expressed in the origin of the child link frame
    // See https://github.com/gazebosim/sdformat/issues/130 for more details
    iDynTree::Traversal traversalWithURDFBase;
    comp.model().computeFullTreeTraversal(traversalWithURDFBase);

    iDynTree::LinkIndex childLinkIdx = traversalWithURDFBase.getChildLinkIndexFromJointIndex(comp.model(), sens->getParentJointIndex());
    std::string childLinkName = comp.model().getLinkName(childLinkIdx);

    iDynTree::Transform childLink_H_sensorFrame = comp.getRelativeTransform(childLinkName, frameName);

    iDynTree::Vector3 zeroVector;
    zeroVector.zero();

    if (!checkVectorAreEqual(childLink_H_sensorFrame.getPosition(), zeroVector, 1e-6))
    {
        std::cerr << "icub-model-test : translation between link "  << childLinkName <<  " and sensor " << sensorName << " is non-zero, test failed, see  https://github.com/gazebosim/sdformat/issues/130  for more details." << std::endl;
        std::cerr << "icub-model-test : childLink_H_sensorFrame.getPosition(): "  << childLink_H_sensorFrame.getPosition().toString() << std::endl;
        return false;
    }

    return true;
}

bool isModelWithoutFTSensorInAnkles(const std::string& modelPath)
{
    return (modelPath.find("Genova03") != std::string::npos ||
            modelPath.find("Nancy01") != std::string::npos ||
            modelPath.find("Lisboa01") != std::string::npos);
}

std::vector<std::string> getFTSensorNames(const std::string& modelPath)
{
    std::vector<std::string> ftSensorNames;

    ftSensorNames.push_back("l_arm_ft");
    ftSensorNames.push_back("r_arm_ft");
    ftSensorNames.push_back("l_leg_ft");
    ftSensorNames.push_back("r_leg_ft");

    if(!isModelWithoutFTSensorInAnkles(modelPath))
    {
        ftSensorNames.push_back("l_foot_ft");
        ftSensorNames.push_back("r_foot_ft");
    }

    return ftSensorNames;
}

// Check that FT sensors have coherent sensor and frame information
// The frame information is used by RViz to display the wrench, while the sensor tags
// are used by iDynTree (there is also a sensor tag used by Gazebo, that is not tested here)
bool checkAllFTMeasurementFrameGivenBySensorTagsIsCoherentWithMeasurementFrameGivenByFrame(const std::string& modelPath)
{
    iDynTree::ModelLoader mdlLoader;

    // Open the model
    iDynTree::ModelParserOptions parserOptions;

    // By default iDynTree creates an additional frame with the same name of the sensor,
    // however in this case we have both the sensor and the urdf frame called <prefix>_ft,
    // and for this test we want to make sure that the <prefix>_ft additional frame is the
    // one in the URDF
    parserOptions.addSensorFramesAsAdditionalFrames = false;
    mdlLoader.setParsingOptions(parserOptions);

    mdlLoader.loadModelFromFile(modelPath);

    iDynTree::KinDynComputations comp;
    const bool modelLoaded = comp.loadRobotModel(mdlLoader.model());

    if (!modelLoaded)
    {
        std::cerr << "icub-model-test error: impossible to load model from " << modelLoaded << std::endl;
        return false;
    }

    iDynTree::Vector3 grav;
    grav.zero();
    iDynTree::JointPosDoubleArray qj(comp.getRobotModel());
    iDynTree::JointDOFsDoubleArray dqj(comp.getRobotModel());
    qj.zero();
    dqj.zero();

    comp.setRobotState(qj,dqj,grav);
    iDynTree::SensorsList sensors = mdlLoader.sensors();


    auto ftSensorNames = getFTSensorNames(modelPath);
    bool ok = true;

    for(auto& ftSensorName: ftSensorNames)
    {
        ok = checkFTMeasurementFrameGivenBySensorTagsIsCoherentWithMeasurementFrameGivenByFrame(modelPath, comp, sensors, ftSensorName) && ok;
    }

    if (ok)
    {
        std::cerr << "icub-model-test : checkAllFTMeasurementFrameGivenBySensorTagsIsCoherentWithMeasurementFrameGivenByFrame test performed correctly " << std::endl;
    }
    
    return ok;
}

int main(int argc, char ** argv)
{
    yarp::os::Property prop;

    // Parse command line options
    prop.fromCommand(argc,argv);

    // Skip upper body tests (to support testing iCubHeidelberg01 model)
    bool skipUpperBody = prop.check("skipUpperBody");

    // Get model name
    if( ! prop.find("model").isString() )
    {
        std::cerr << "icub-model-test error: model option not found" << std::endl;
        return EXIT_FAILURE;
    }

    const std::string modelPath = prop.find("model").asString().c_str();

    iDynTree::ModelLoader mdlLoader;
    mdlLoader.loadModelFromFile(modelPath);

    // Open the model
    iDynTree::KinDynComputations comp;
    const bool modelLoaded = comp.loadRobotModel(mdlLoader.model());

    if( !modelLoaded )
    {
        std::cerr << "icub-model-test error: impossible to load model from " << modelLoaded << std::endl;
        return EXIT_FAILURE;
    }

    iDynTree::Vector3 grav;
    grav.zero();
    iDynTree::JointPosDoubleArray qj(comp.getRobotModel());
    iDynTree::JointDOFsDoubleArray dqj(comp.getRobotModel());
    qj.zero();
    dqj.zero();

    comp.setRobotState(qj,dqj,grav);

    // Check axis
    if( !checkAxisDirectionsV2(comp) )
    {
        return EXIT_FAILURE;
    }


    // Check if base_link exist, and check that is a frame attached to root_link and if its
    // transform is the idyn
    if( !checkBaseLink(comp) )
    {
        return EXIT_FAILURE;
    }


    // Check if l_sole/r_sole have the same distance from the root_link
    if( !checkSolesAreParallel(comp) )
    {
        return EXIT_FAILURE;
    }

    // Now some test that test the sensors
    // The ft sensors orientation respect to the root_link are different to iCubV2 and they are under investigation.
    if( !checkFTSensorsAreEvenAndNotNull(mdlLoader) )
    {
        return EXIT_FAILURE;
    }

    if (!checkFTSensorsAreCorrectlyOrientedV2(comp))
    {
        return EXIT_FAILURE;
    }

    if (!checkAllFTMeasurementFrameGivenBySensorTagsIsCoherentWithMeasurementFrameGivenByFrame(modelPath))
    {
        return EXIT_FAILURE;
    }



    std::cerr << "Check for model " << modelPath << " concluded correctly!" << std::endl;

    return EXIT_SUCCESS;
}
