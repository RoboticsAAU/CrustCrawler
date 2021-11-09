#pragma once
#include "DataStructures.h"

class DynamixelConnector
{
public:
	//Constructors and desctructor
	DynamixelConnector();
	~DynamixelConnector();

	//Custom methods
	void setPWM(int * );

	JointAngles getJointAngles(UnitType unitType);



private:
	//Dynamixel connector object pointer (Declared on the HEAP)
	Dynamixel2Arduino* p_dynamixel;

	//Raw dynamixel angles read directly from the servos.
	JointAngles rawDynamixelAngles;
	//Joint angles in the desired unit
	JointAngles outputAngles;


	// Custom Private methods
	void _UpdateDynamixelAngles();
	void _AngleConverter(UnitType desiredUnit);
};

