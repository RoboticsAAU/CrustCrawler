#pragma once
// Generel includes

// Library includes
#include <DynamixelShield.h>
#include <Dynamixel2Arduino.h>
#include <actuator.h>

// Custom includes
#include "DataStructures.h"
#include "JointConfigs.h"

#define DYNAMIXEL_SERIAL Serial
const int DirectionPin{ 2 };

class DynamixelConnection
{
public:
	DynamixelConnection();

	void EmergencyStop();

	JointAngles getJointAngles();
	Velocities getJointVelocities();

	void setJointPWM(JointTorques& correctionTorques, Velocities& correctionVelocities, JointAngles& currentJointAngles);
private:
	Dynamixel2Arduino dynamixel;

	JointTorques currentJointTorques;

	double _typeConverter(double& variable, double& desiredVel, double& currentJointAngle, Joint& joint, OutputType type);
	void _getPWMConstants(double& desiredTorque, double& desiredVel, ServoType servoType);
	double torqueConstant, velocityConstant;
	bool _isWithinAngleBoundaries(Joint& inputJoint, double inputAngle);

};

