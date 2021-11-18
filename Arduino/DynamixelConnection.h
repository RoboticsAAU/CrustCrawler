#pragma once
// Generel includes

// Library includes
#include <DynamixelShield.h>
#include <Dynamixel2Arduino.h>
#include <actuator.h>

// Custom includes
#include "DataStructures.h"
#include "JointConfigs.h"
#include "ComputerConnection.h"

#define DYNAMIXEL_SERIAL Serial
const int DirectionPin{ 2 };

class DynamixelConnection
{
public:
	DynamixelConnection(ComputerConnection* pointer);

	void EmergencyStop();

	JointAngles getJointAngles();
	JointAngles getJointAngle(unsigned int& jointID);
	Velocities getJointVelocities();
	Velocities getJointVelocity(unsigned int& jointID);

	void setJointPWM(JointTorques& correctionTorques, Velocities& correctionVelocities, JointAngles& currentJointAngles);
private:
	Dynamixel2Arduino dynamixel;
	ComputerConnection* pComCon;

	JointTorques currentJointTorques;
	double currentJointPWM[6] = { 0,0,0,0,0,0 };

	double _typeConverter(double& variable, double& desiredVel, Joint& joint, OutputType type);
	void _getPWMConstants(double& desiredTorque, double& desiredVel, ServoType servoType);
	double torqueConstant, velocityConstant;
	bool _isWithinAngleBoundaries(Joint& inputJoint, double inputAngle);

};

