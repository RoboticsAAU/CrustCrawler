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

	void AngleConverter(JointAngles &inputAngles, UnitType desiredUnit);

	//Torque off for every servo.
	void EmergencyStop();


	//Getters and setters
	void setPWM(uint8_t id, uint16_t pwmData);
	uint16_t getCurrentPWM(uint8_t id);
	JointAngles getJointAngles(UnitType unitType);






private:
	//Dynamixel connector object pointer (Declared on the HEAP)
	Dynamixel2Arduino* p_dynamixel;

	//Raw dynamixel angles read directly from the servos.
	JointAngles internalJointAngles;

	Joint _Joint1, _Joint2, _Joint3, _Joint4, _Joint5;
	Joint _joints[5] = { _Joint1, _Joint2, _Joint3, _Joint4, _Joint5 };

	// Custom Private methods
	void _UpdateDynamixelAngles();

	void _SetupDynamixelServos();
};

