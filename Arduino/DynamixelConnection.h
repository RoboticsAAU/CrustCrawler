#pragma once
// Generel includes

// Library includes
#include <DynamixelShield.h>
#include <Dynamixel2Arduino.h>
#include <actuator.h>

// Custom includes
#include "JointConfigs.h"
#include "DataStructures.h"

#define DYNAMIXEL_SERIAL Serial
const int DirectionPin{ 2 };

class DynamixelConnection
{
public:
	DynamixelConnection();

	JointAngles getJointAngles();
private:
	Dynamixel2Arduino dynamixel;
};

