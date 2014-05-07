// Copyright (C) 2013 RobotCub Consortium
// Author: Francesco Romano
// CopyPolicy: Released under the terms of the GNU GPL v2.0.

#include "ICUBConfigParser.h"

#include <yarp/os/Property.h>
#include "Device.h"
#include "Robot.h"
#include "Types.h"
#include "Constants.h"
#include <string>
#include <sstream>
#include <cmath>
#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>

using namespace RobotInterface;
using namespace icub2gazebo;
using namespace yarp::math;

DeviceWrapper::DeviceWrapper():numberOfJoints(-1){}
PIDValues::PIDValues():p(0),d(0),i(0),shiftFactor(0),maxPWM(0),minPWM(0){}
MotorParameters::MotorParameters():motorResistance(0),motorInductance(0),backEMFConstant(0),weight(0),inertia(0),
numberOfPoles(0),maxStaticFriction(0),viscousDamping(0),peakTorque(0),stallTorque(0),maxVoltage(0){}
PIDsMotorWrapper::PIDsMotorWrapper():pidValues(),motorGain(0){}

CoupledJointInfo::CoupledJointInfo(unsigned int _numberOfJoints):
numberOfJoints(_numberOfJoints > 0 ? _numberOfJoints : 0),
velocityTransformationMatrix(numberOfJoints, numberOfJoints),
torqueTransformationMatrix(numberOfJoints, numberOfJoints){
    if (numberOfJoints > 0) {
        jointIndexes.reserve(numberOfJoints);
    }
}

iCubConfigParser::iCubConfigParser(RobotInterface::Robot &robot)
{
    _robot = robot;
}

void iCubConfigParser::startParsing()
{
    for (DeviceList::const_iterator it = _robot.devices().begin(); it != _robot.devices().end(); it++) {
        if (it->type().compare(DEVICE_TYPE_MOTIONCONTORL) == 0) {
            DeviceWrapper deviceWrapper;
            if (parseDevice(*it, &deviceWrapper)) {
                _devices[it->name()] = deviceWrapper;
            }
        }
    }
    for (DeviceList::const_iterator it = _robot.devices().begin(); it != _robot.devices().end(); it++) {
        if (it->type().compare(DEVICE_TYPE_MOTORSPECIFICATIONS) == 0) {
            //get relative device
            std::string relDevice = this->getRelativeDeviceName(*it);
            //find wrapper
            if (relDevice.empty())
                continue;
            DeviceWrapper wrapper = _devices.at(relDevice);
            std::cout << "Wrapper found: " << relDevice << "\n";
            parseMotors(*it, &wrapper);
            parseCoupledJoints(*it, &wrapper);
            _devices.at(relDevice) = wrapper;
        }
    }
    
    computeNewPIDsValues();
}

bool iCubConfigParser::parseDevice(const RobotInterface::Device &device, DeviceWrapper* wrapper)
{
    if (!wrapper) return false;
    wrapper->device = device;
    
    parseGeneralGroup(device, wrapper);
    parsePIDs(device, wrapper);
    return true;
}

bool iCubConfigParser::parseGeneralGroup(const RobotInterface::Device &device, DeviceWrapper* wrapper)
{
    if (!wrapper) return false;
    std::string generalProperties = device.findParam(GENERAL_GROUP_NAME);
    if (!generalProperties.empty()) {
        yarp::os::Property generals(generalProperties.c_str());
        wrapper->numberOfJoints = generals.find(GENERAL_JOINTS_PARAM_NAME).asInt();
        
        yarp::os::Bottle &encoders = generals.findGroup(GENERAL_ENCODERS_PARAM_NAME);
        for (int i = 1; i < encoders.size(); i++) {
            wrapper->encoders[i-1] = encoders.get(i).asDouble();
        }
        return true;
    }
    return false;
}

bool iCubConfigParser::parsePIDs(const RobotInterface::Device &device, DeviceWrapper* wrapper)
{
    if (!wrapper) return false;
    yarp::os::Property group;
    
    //first: find PIDs group
    std::string groupStr = findGroup(device.params(), PID_GROUP_NAME);
    if (!groupStr.empty()) {
        group.fromString(groupStr);
        
        int i = 0;
        while (true) {
            std::stringstream pidStream;
            pidStream << PID_PARAM_BASE_NAME << i;
            
            std::string name = pidStream.str();
            
            yarp::os::Bottle &pids = group.findGroup(pidStream.str());
            if (!pids.isNull()) {
                PIDValues pidValues;
                pidValues.p = pids.get(1).asDouble();
                pidValues.d = pids.get(2).asDouble();
                pidValues.i = pids.get(3).asDouble();
                pidValues.minPWM = pids.get(4).asInt();
                pidValues.maxPWM = pids.get(5).asInt();
                pidValues.shiftFactor = pids.get(6).asInt();
                wrapper->pids[i] = pidValues;
            }
            else {
                if (wrapper->numberOfJoints <= 0) {
                    //number of joints not defined (or 0). Because I didn't find the pid#i I break
                    break;
                }
            }
            i++;
            if (wrapper->numberOfJoints > 0 &&
                i >= wrapper->numberOfJoints) {
                break;
            }
        }
        return true;
    }
    return false;
}

bool iCubConfigParser::parseMotors(const RobotInterface::Device &device, DeviceWrapper* wrapper)
{
    if (!wrapper) return false;
    yarp::os::Property group;
    std::string groupStr = findGroup(device.params(), MOTOR_GROUP_NAME);
    if (!groupStr.empty()) {
        group.fromString(groupStr.c_str());
        int i = 0;
        while (true) {
            std::stringstream motorStream;
            motorStream << MOTOR_SUBGROUP_NAME << i;
            
            yarp::os::Bottle subGroupBottle = group.findGroup(motorStream.str());
            if (subGroupBottle.isNull()) {
                if (wrapper->numberOfJoints <= 0) {
                    //number of joints not defined (or 0). Because I didn't find the motor#i I break
                    break;
                }
                else if (wrapper->numberOfJoints > 0 &&
                         i >= wrapper->numberOfJoints) {
                    break;
                }
                i++;
                continue;
            }
            yarp::os::Property motor(subGroupBottle.toString().c_str());
            
            MotorParameters motorParameters;
            //fill structure
            yarp::os::Value val;
            motorParameters.motorResistance = motor.find(MOTOR_RESISTANCE_PARAM_NAME).asDouble();
            motorParameters.motorInductance = motor.find(MOTOR_INDUCTANCE_PARAM_NAME).asDouble();
            motorParameters.backEMFConstant = motor.find(MOTOR_BACKEMF_PARAM_NAME).asDouble();
            motorParameters.weight = motor.find(MOTOR_WEIGHT_PARAM_NAME).asDouble();
            motorParameters.inertia = motor.find(MOTOR_INERTIA_PARAM_NAME).asDouble();
            motorParameters.numberOfPoles = motor.find(MOTOR_NUMPOLES_PARAM_NAME).asDouble();
            motorParameters.maxStaticFriction = motor.find(MOTOR_STATICFRICTION_PARAM_NAME).asDouble();
            motorParameters.viscousDamping = motor.find(MOTOR_VISCOUSDAMPING_PARAM_NAME).asDouble();
            motorParameters.peakTorque = motor.find(MOTOR_PEAKTORQUE_PARAM_NAME).asDouble();
            motorParameters.stallTorque = motor.find(MOTOR_STALLTORQUE_PARAM_NAME).asDouble();
            motorParameters.reductionFactor = motor.find(MOTOR_REDUCTION_PARAM_NAME).asDouble();
            motorParameters.maxVoltage = motor.find(MOTOR_MAXVOLTAGE_PARAM_NAME).asDouble();
            
            wrapper->motors[i] = motorParameters;
            i++;
            if (wrapper->numberOfJoints > 0 &&
                i >= wrapper->numberOfJoints) {
                break;
            }
        }
        return true;
    }
    return false;
}

bool iCubConfigParser::parseCoupledJoints(const RobotInterface::Device &device, DeviceWrapper* wrapper)
{
    if (!wrapper) return false;
    yarp::os::Property group;
    std::string groupStr = findGroup(device.params(), COUPLEDJOINT_GROUP_NAME);
    if (!groupStr.empty()) {
        yarp::os::Bottle groupBottle(groupStr.c_str());
        //enumerate subgroups
        for (int i = 0; i < groupBottle.size(); i++) {
            yarp::os::Bottle *subGroup = groupBottle.get(i).asList();
            if (!subGroup) continue;
            int jointNumber = subGroup->find(COUPLEDJOINT_PARAM_JOINT_NUMBER_NAME).asInt();
            if (jointNumber <= 0) continue;
            CoupledJointInfo coupledJoint(jointNumber);
            yarp::os::Bottle& jointIndexes = subGroup->findGroup(COUPLEDJOINT_PARAM_JOINT_INDEXES_NAME);
            
            coupledJoint.velocityTransformationMatrixCoefficient = subGroup->find(COUPLEDJOINT_PARAM_JOINT_VELMATRIXCOEFF_NAME).asDouble();
            coupledJoint.torqueTransformationMatrixCoefficient = subGroup->find(COUPLEDJOINT_PARAM_JOINT_TORQUEMATRIXCOEFF_NAME).asDouble();
            
            yarp::os::Bottle& velMatrixBottle = subGroup->findGroup(COUPLEDJOINT_PARAM_JOINT_VELMATRIX_NAME);
            yarp::os::Bottle& torqueMatrixBottle = subGroup->findGroup(COUPLEDJOINT_PARAM_JOINT_TORQUEMATRIX_NAME);

            if (jointIndexes.isNull() ||
                (velMatrixBottle.isNull() || velMatrixBottle.size() < 2)  ||
                (torqueMatrixBottle.isNull() || torqueMatrixBottle.size() < 2)) continue;
            //remove outer wrapper of the matrix
            yarp::os::Bottle* velMatrix = velMatrixBottle.get(1).asList();
            yarp::os::Bottle* torqueMatrix = torqueMatrixBottle.get(1).asList();
            if (!velMatrix || !torqueMatrix) continue;
            
            for (int idx = 0; idx < jointNumber; idx++) {
                //I have to skip the first element..
                coupledJoint.jointIndexes.push_back(jointIndexes.get(idx + 1).asInt());
                
                yarp::os::Bottle* velMatrixRow = velMatrix->get(idx).asList();
                yarp::os::Bottle* torqueMatrixRow = torqueMatrix->get(idx).asList();
                if (!velMatrixRow || !torqueMatrix) continue;
                
                std::string row1 = velMatrixRow->toString();
                std::string row2 = torqueMatrixRow->toString();
                
                for (int col = 0; col < jointNumber; col++) {
                    coupledJoint.velocityTransformationMatrix(idx, col) = velMatrixRow->get(col).asDouble();
                    coupledJoint.torqueTransformationMatrix(idx, col) = torqueMatrixRow->get(col).asDouble();
                }
            }
            wrapper->coupledJoints.push_back(coupledJoint);
        }
        return true;
    }
    return false;
}

bool iCubConfigParser::computeNewPIDsValues()
{
    //For each device
    for (std::map<std::string, DeviceWrapper>::iterator device = _devices.begin(); device != _devices.end(); device++) {
        //iterate over original PIDS, and access using the same key the motor map.
        
        for (std::map<int, PIDValues>::const_iterator originalPID = device->second.pids.begin();
             originalPID != device->second.pids.end(); originalPID++) {
            
            std::map<int, MotorParameters>::const_iterator motorParameters = device->second.motors.find(originalPID->first);
            if (motorParameters != device->second.motors.end()) {
                //found motor parameters. Starting conversion
                PIDValues newPIDs;
                MotorParameters motorParams = motorParameters->second;
                
                //Common unit of measurement for the back EMF constant is Volt per k rpm.
                //But I need to convert it to SI unit=> V / rad/s
                //By the way: the constant in SI unit is equal to the motor current torque [Nm/A]
//                double k_e = motorParams.backEMFConstant * 1e-3 * 60 / (2*PI);
                
                //compute motor static gain (considering infinite inertia)
                //K_openloop = 1/R * (K_phi * N) [Nm / V]
                double motorOpenLoopGain = (motorParams.reductionFactor * motorParams.backEMFConstant) / motorParams.motorResistance;
                //K_feedback = K_phi * N * 1/B [V/Nm]
//                double motorFeedbackGain = (motorParams.reductionFactor * motorParams.backEMFConstant) / motorParams.viscousDamping;
                // For this gain I consider only the openloop gain => infinite inertia ~= stall torque
//                double motorGain = motorOpenLoopGain / (1 + motorOpenLoopGain * motorFeedbackGain); // [Nm / V]
                double motorGain = motorOpenLoopGain;
                
                //for each (p, d, and i)
                double shiftFactor = pow(2, -originalPID->second.shiftFactor);
                double originalP = originalPID->second.p * shiftFactor;
                double originalD = originalPID->second.d * shiftFactor;
                double originalI = originalPID->second.i * shiftFactor;
                
                double pwmFactor = motorParams.maxVoltage / originalPID->second.maxPWM;
                
                //reading encoder data
                std::map<int, double>::const_iterator encIterator = device->second.encoders.find(originalPID->first);
                double delta_enc = encIterator != device->second.encoders.end() ? encIterator->second : 0;
                
                //formula to be checked
                newPIDs.p = originalP * pwmFactor * delta_enc * RAD_2_DEG(1) //pid conversion to [V/rad]
                * motorGain; // pid in [Nm/rad]
                newPIDs.d = originalD * pwmFactor * delta_enc * RAD_2_DEG(1) //pid conversion to [V/rad]
                * motorGain; // pid in [Nm/rad]
                newPIDs.i = originalI * pwmFactor * delta_enc * RAD_2_DEG(1) //pid conversion to [V/rad]
                * motorGain; // pid in [Nm/rad]
                
                //copy original values for the other variables
                newPIDs.maxPWM = originalPID->second.maxPWM;
                newPIDs.minPWM = originalPID->second.minPWM;
                newPIDs.shiftFactor = originalPID->second.shiftFactor;
                                
                PIDsMotorWrapper motorWrapper;
                motorWrapper.pidValues = newPIDs;
                motorWrapper.motorGain = motorGain;
                
                device->second.newPIDs[motorParameters->first] = motorWrapper;
            }
        }
        //I have to check if there are some coupled joints for this device
        if (!device->second.coupledJoints.empty()) {
            //I have already computed the tau joint -> error (in radiants)
            //Usually tau joints = tau motor.
            //for coupled joints this is no longer true.
            //I want to map the pid (joint) constants to their motor constants and only at this point apply the motor constant. Then I have to remap this back to the joint space
            for (std::vector<CoupledJointInfo>::const_iterator cj = device->second.coupledJoints.begin();
                 cj != device->second.coupledJoints.end(); cj++) {
                //build a vector containing all the Ps (and Ds and Is) for the joints composing the coupling
                yarp::sig::Vector pValues(cj->numberOfJoints);
                yarp::sig::Vector dValues(cj->numberOfJoints);
                yarp::sig::Vector iValues(cj->numberOfJoints);
                
                for (int idx = 0; idx < cj->numberOfJoints; idx++) {
                    unsigned int jointIndex = cj->jointIndexes.at(idx);
                    
                    icub2gazebo::PIDsMotorWrapper pids = device->second.newPIDs.find(jointIndex)->second;
                    
                    pValues[idx] = pids.pidValues.p;
                    dValues[idx] = pids.pidValues.d;
                    iValues[idx] = pids.pidValues.i;
                }
                yarp::sig::Matrix transfMatrix = cj->torqueTransformationMatrix;

                yarp::sig::Vector pMotor = cj->torqueTransformationMatrixCoefficient * transfMatrix * pValues;
                yarp::sig::Vector dMotor = cj->torqueTransformationMatrixCoefficient * transfMatrix * dValues;
                yarp::sig::Vector iMotor = cj->torqueTransformationMatrixCoefficient * transfMatrix * iValues;
                
                //Now transform it back to joint coordinates.
                //simply invert the matrix (which should always be invertible and small so I do not care about performances)
                yarp::sig::Matrix invMatrix = yarp::math::luinv(transfMatrix);
                
                pValues = invMatrix * pMotor;
                dValues = invMatrix * dMotor;
                iValues = invMatrix * iMotor;
                
                //save it back to PIDs values
                for (int idx = 0; idx < cj->numberOfJoints; idx++) {
                    unsigned int jointIndex = cj->jointIndexes.at(idx);
                    
                    icub2gazebo::PIDsMotorWrapper pids = device->second.newPIDs.find(jointIndex)->second;
                    pids.pidValues.p = pValues[idx];
                    pids.pidValues.d = dValues[idx];
                    pids.pidValues.i = iValues[idx];
                }
            }
        }
    }
    return true;
}

std::string iCubConfigParser::getRelativeDeviceName(const RobotInterface::Device &device)
{
    std::string generalProperties = device.findParam(MOTOR_DEVICE_GROUP_NAME);
    if (!generalProperties.empty()) {
        yarp::os::Property generals(generalProperties.c_str());
        return generals.find(MOTOR_DEVICE_RELDEVICE_PARAM_NAME).asString();
    }
    return "";
}

std::string iCubConfigParser::outputPIDsForDevice(std::string deviceName)
{
    std::map<std::string, icub2gazebo::DeviceWrapper>::const_iterator device = _devices.find(deviceName);
    if (device == _devices.end())
        return std::string();
    
    std::ostringstream oss;
    for (std::map<int, PIDsMotorWrapper>::const_iterator pidWrapper = device->second.newPIDs.begin(); pidWrapper != device->second.newPIDs.end(); pidWrapper++) {
        oss <<  PID_PARAM_BASE_NAME << pidWrapper->first << " "
        << pidWrapper->second.pidValues.p << " " << pidWrapper->second.pidValues.d << " " << pidWrapper->second.pidValues.i << " "
        << pidWrapper->second.pidValues.maxPWM << " " << pidWrapper->second.pidValues.minPWM << " " << pidWrapper->second.pidValues.shiftFactor << " 0"
        << std::endl;
    }
    return oss.str();
}

std::string iCubConfigParser::outputPIDs()
{
    std::ostringstream oss;
    for (std::map<std::string, icub2gazebo::DeviceWrapper >::const_iterator it = _devices.begin(); it != _devices.end(); it++) {
        
        std::string newPids = this->outputPIDsForDevice(it->first);
        if (!newPids.empty()) {
            oss << "device = [" << it->first << "]" << std::endl;
            oss << newPids;
            oss << std::endl << std::endl;
        }
    }
    return oss.str();
}
