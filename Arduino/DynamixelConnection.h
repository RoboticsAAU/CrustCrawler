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

//#define VELOCITY_CONTROL
#define PWM_CONTROL
#define DYNAMIXEL_SERIAL Serial
const int DirectionPin{ 2 };

class DynamixelConnection
{
public:
	DynamixelConnection(ComputerConnection* pointer);

	void EmergencyStop();

	JointAngles getJointAngles();
	double getJointAngle(Joint& joint);
	Velocities getJointVelocities();
	double getJointVelocity(unsigned int& jointID);

	void setJointVelocity(Velocities& goalVelocities);
	void setJointPWM(JointTorques& updateTorques, Velocities& currentVelocities);
private:
	Dynamixel2Arduino dynamixel;
	ComputerConnection* pComCon;

	double _typeConverter(double& variable, double& currentVel, ServoType& servoType, OutputType type);
	void _getPWMConstants(double& desiredTorque, double& currentVel, ServoType& servoType);
	double torqueConstant, velocityConstant;
	bool _isWithinAngleBoundaries(Joint& inputJoint, double inputAngle);

};

