// Copyright (C) 2013 RobotCub Consortium
// Author: Francesco Romano
// CopyPolicy: Released under the terms of the GNU GPL v2.0.

/**
 
 XML read for the motor: * means the parameter is needed for the module

 <group name="MOTORS">
    <group name="Motor0">
        <param name="Resistance">10</param> <!-- * Motor Resistance [Ohm] -->
        <param name="Inductance">10</param> <!-- Motor Inductance [mH] -->
        <param name="BackEMF">10</param> <!-- * Motor Back ElectroMagnetic Force Constant [V/ rad/s] = [Nm/A] -->
        <param name="Weight">10</param> <!-- Motor (Frameless) weight [Kg] -->
        <param name="Inertia">10</param> <!-- Motor Inertia [Kg m^2] -->
        <param name="NumMagnPoles">10</param> <!-- Number of magnetic poles of the motor-->
        <param name="MaxStaticFriction">10</param> <!-- Max static friction [Nm] -->
        <param name="ViscousDamping">10</param> <!-- * Viscous damping [Nm / rad/s] -->
        <param name="PeakTorque">10</param> <!-- Peak torque [Nm] -->
        <param name="StallTorque">10</param> <!-- Stall torque [Nm] -->
        <param name="ReductionFactor">10</param> <!-- * Transmission reduction -->
        <param name="MaxVoltage">10</param> <!-- * Maximum voltage [V] -->
    </group>
 </group>
 
XML for coupled joints:
 
 <group name="COUPLED_JOINTS">
     <group name="Shoulder"><!-- Name of the coupling. It is not used-->
         <param name="JointNumber">3</param> <!-- Number of joints forming coupled -->
         <param name="JointIndexes">0 1 2</param><!-- Indexes of the joints forming the coupling. They must be present in the device xml file (e.g. in the PID section). The order must be consistent with the matrix row order -->
         <param name="VelocityTransfMatrixCoefficient">1</param> <!-- scalar coefficient for the velocity transformation matrix -->
         <param name="VelocityTransfMatrix">((1 0 0) (0 1 0) (1.625 -1.625 1))</param> <!-- Matrix transforming from encoder axis/coordinates to joint coordinates, i.e. JointPosition = T * encoderPositions-->
         <param name="TorqueTransfMatrixCoefficient">0.0001519</param> <!-- scalar coefficient for the torque transformation matrix -->
         <param name="TorqueTransfMatrix">((1 0 0) (-0.6153846154 0.6153846154 0) (-0.6153846154 0.6153846154 0.6153846154))</param> <!-- Matrix transforming from torque in motor coordinates to torque in joint coordinates, i.e. Torque_motor = T * Torque_joints-->
     </group>
 </group>
 
 NOTES on implementation:
 This application parses the iCub XML configuration file.
 In the iCub repository the ``robotInterface'' module does the same thing.
 Unfortunately it is not written as a library (linked by the module) but as a standalone module.
 Because I need the very same files I copied them (in the robotInterface dir in include and src). When the parser will be made as a standalone library it will be possible to remvove those files.
 Because I didn't want to include the debugStream file (I was not able to add it as a library but I should have added it hardcoded in the CMakeLists.txt) I created a "fake" file ``Debug.h'' which tries to hide the original debug library. Again, when the original iCub will (?) be more modular it will be possible to remove also the Debug.h file.
 */

#include "XMLReader.h"
#include "Robot.h"

#include "ICUBConfigParser.h"

#include <yarp/os/ResourceFinder.h>

using namespace RobotInterface;

int main(int argc, char ** argv) 
{
    //add ResourceFinder to locate iCub config file (?)
    yarp::os::ResourceFinder resourceFinder;
    resourceFinder.configure(argc, argv);
    
    yarp::os::Value file = resourceFinder.find("inputFile");
    if (file.isNull()) {
        std::cerr << "No input file provided." << std::endl;
        exit(-1);
    }
    
    std::cout << "Start parsing " << file.asString() << std::endl;
    
    XMLReader xmlReader;
    Robot robot = xmlReader.getRobot(file.asString());
    
    icub2gazebo::iCubConfigParser parser(robot);
    parser.startParsing();
    
    std::cout << std::endl << std::endl << "Computing updated PIDs" << std::endl << parser.outputPIDs();
    
    return 0;
    
    
}
