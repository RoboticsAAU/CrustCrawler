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
	JointVelocities getJointVelocities();

	void setJointPWM(JointTorques& correctionTorques);
private:
	Dynamixel2Arduino dynamixel;

	JointTorques currentJointTorques;

	double _typeConverter(double variable, OutputType type);

};

