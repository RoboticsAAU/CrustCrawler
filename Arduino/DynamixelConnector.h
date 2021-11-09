#pragma once
#include "DataStructures.h"

//Include dynamixel libraries
#include "Dynamixel2Arduino.h"
#include "DynamixelShield.h"

class DynamixelConnector
{
public:
	//Constructors and desctructor
	DynamixelConnector();
	~DynamixelConnector();

	//Custom methods
	//void setPWM(int * );

	JointAngles getJointAngles(UnitType unitType);

	void AngleConverter(JointAngles &inputAngles, UnitType desiredUnit);

private:
	//Dynamixel connector object pointer (Declared on the HEAP)
	Dynamixel2Arduino* p_dynamixel;

	//Raw dynamixel angles read directly from the servos.
	JointAngles internalJointAngles;


	// Custom Private methods
	void _UpdateDynamixelAngles();
};

