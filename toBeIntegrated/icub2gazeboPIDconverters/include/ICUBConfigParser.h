// Copyright (C) 2013 RobotCub Consortium
// Author: Francesco Romano
// CopyPolicy: Released under the terms of the GNU GPL v2.0.

#ifndef ICUB_CONFIG_PARSER_H
#define ICUB_CONFIG_PARSER_H

#include "Types.h"
#include "Robot.h"
#include "Device.h"
#include <map>
#include <vector>
#include <string>
#include <yarp/sig/Matrix.h>

namespace icub2gazebo {
    
    struct DeviceWrapper;
    struct MotorParameters;
    struct PIDValues;
    struct PIDsMotorWrapper;
    struct CoupledJointInfo;
    
    struct DeviceWrapper {
        RobotInterface::Device device;
        
        int numberOfJoints; /*!< Number of joints in the device */
        
        std::map<int, double> encoders; /*!< Maps the index of the joint to its encoder angle value. I use a map because the index number is important (I can't use a vector) */
        std::map<int, PIDValues> pids; /*!< Maps the index of the joint to its PID values. I use a map because the index number is important (I can't use a vector) */
        std::map<int, MotorParameters> motors; /*!< Maps the index of the joint to its motor parameters. I use a map because the index number is important (I can't use a vector) */
        std::vector<CoupledJointInfo> coupledJoints; /*!< List of coupled joints for the device */
        
        std::map<int, PIDsMotorWrapper> newPIDs; /*!< Maps the index of the joint to the new computed PID value. I use a map because the index number is important (I can't use a vector) */
        
        DeviceWrapper();
    };

    struct MotorParameters {
        double motorResistance;
        double motorInductance;
        double backEMFConstant;
        double weight;
        double inertia;
        double numberOfPoles;
        double maxStaticFriction;
        double viscousDamping;
        double peakTorque;
        double stallTorque;
        double reductionFactor;
        double maxVoltage; //here?
        
        MotorParameters();
    };
    
    struct PIDValues {
        double p; //they must hold also the updated value which is a floating point number
        double d;
        double i;
        
        int shiftFactor;
        int maxPWM;
        int minPWM;
        
        PIDValues();
    };
    
    struct PIDsMotorWrapper {
        struct PIDValues pidValues;
        double motorGain;
        
        PIDsMotorWrapper();
    };
    
    struct CoupledJointInfo {
        std::string groupName;
        unsigned int numberOfJoints; /*!< Number of motors and joints coupled */
        std::vector<unsigned int> jointIndexes; /*!< indexes of the joints (w.r.t. the device joints) that are coupled */
        
        double velocityTransformationMatrixCoefficient;
        yarp::sig::Matrix velocityTransformationMatrix;
        double torqueTransformationMatrixCoefficient;
        yarp::sig::Matrix torqueTransformationMatrix;
        
        CoupledJointInfo(unsigned int numberOfJoints);
    };
    
    class iCubConfigParser {
    private:
        RobotInterface::Robot _robot;
        std::map<std::string, DeviceWrapper> _devices;
        
        bool parseDevice(const RobotInterface::Device &device, DeviceWrapper*);
        bool parseGeneralGroup(const RobotInterface::Device &device, DeviceWrapper* deviceWrapper);
        bool parsePIDs(const RobotInterface::Device &device, DeviceWrapper* wrapper);
        bool parseMotors(const RobotInterface::Device &device, DeviceWrapper* wrapper);
        bool parseCoupledJoints(const RobotInterface::Device &device, DeviceWrapper* wrapper);
        std::string getRelativeDeviceName(const RobotInterface::Device &device);
        
        bool computeNewPIDsValues();
        
    public:        
        iCubConfigParser(RobotInterface::Robot &robot);
        
        void startParsing();
        std::string outputPIDsForDevice(std::string deviceName);
        std::string outputPIDs();
        
    };
}

#endif
