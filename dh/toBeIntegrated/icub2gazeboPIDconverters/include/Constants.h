//
//  Contants.h
//  iCub2GazeboPIDConverters
//
//  Created by Francesco Romano on 05/12/13.
//
//

#ifndef ICUB2GAZEBOPIDCONVERTERS_CONSTANTS_H
#define ICUB2GAZEBOPIDCONVERTERS_CONSTANTS_H

#define PI 3.1415926536
#define RAD_2_DEG(x) ((x) * (180 / PI))
#define DEG_2_RAD(x) ((x) * (PI / 180))

const std::string DEVICE_TYPE_MOTIONCONTORL = "canmotioncontrol"; /*! Type of device to parse. It contains PIDs, Encoder */
const std::string DEVICE_TYPE_MOTORSPECIFICATIONS = "motorspecifications"; /*! Type of device to parse. It contains Motor specifications */

/** General section */
const std::string GENERAL_GROUP_NAME = "GENERAL";
const std::string GENERAL_JOINTS_PARAM_NAME = "Joints";
const std::string GENERAL_ENCODERS_PARAM_NAME = "Encoder";

/** PIDs Section */
const std::string PID_GROUP_NAME = "PIDS";
const std::string PID_PARAM_BASE_NAME = "Pid"; /*! This will be concatened with the joint index to find the pid specifications */

/**********************/
/** Motor Specification file */

/** Device section */
const std::string MOTOR_DEVICE_GROUP_NAME = "DEVICE";
const std::string MOTOR_DEVICE_RELDEVICE_PARAM_NAME = "rel_candevice";

/* Motor Section */
const std::string MOTOR_GROUP_NAME = "MOTORS";
const std::string MOTOR_SUBGROUP_NAME = "Motor"; /*! This will be concatened with the joint index to find the motor specifications */

const std::string MOTOR_RESISTANCE_PARAM_NAME = "Resistance";
const std::string MOTOR_INDUCTANCE_PARAM_NAME = "Inductance";
const std::string MOTOR_BACKEMF_PARAM_NAME = "BackEMF";
const std::string MOTOR_WEIGHT_PARAM_NAME = "Weight";
const std::string MOTOR_INERTIA_PARAM_NAME = "Inertia";
const std::string MOTOR_NUMPOLES_PARAM_NAME = "NumMagnPoles";
const std::string MOTOR_STATICFRICTION_PARAM_NAME = "MaxStaticFriction";
const std::string MOTOR_VISCOUSDAMPING_PARAM_NAME = "ViscousDamping";
const std::string MOTOR_PEAKTORQUE_PARAM_NAME = "PeakTorque";
const std::string MOTOR_STALLTORQUE_PARAM_NAME = "StallTorque";
const std::string MOTOR_REDUCTION_PARAM_NAME = "ReductionFactor";
const std::string MOTOR_MAXVOLTAGE_PARAM_NAME = "MaxVoltage";

/** Coupled joints Section*/
const std::string COUPLEDJOINT_GROUP_NAME = "COUPLED_JOINTS";
const std::string COUPLEDJOINT_PARAM_JOINT_NUMBER_NAME = "JointNumber";
const std::string COUPLEDJOINT_PARAM_JOINT_INDEXES_NAME = "JointIndexes";
const std::string COUPLEDJOINT_PARAM_JOINT_VELMATRIXCOEFF_NAME = "VelocityTransfMatrixCoefficient";
const std::string COUPLEDJOINT_PARAM_JOINT_VELMATRIX_NAME = "VelocityTransfMatrix";
const std::string COUPLEDJOINT_PARAM_JOINT_TORQUEMATRIX_NAME = "TorqueTransfMatrix";
const std::string COUPLEDJOINT_PARAM_JOINT_TORQUEMATRIXCOEFF_NAME = "TorqueTransfMatrixCoefficient";

#endif
