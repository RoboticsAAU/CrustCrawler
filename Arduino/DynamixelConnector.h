#pragma once
#include "DataStructures.h"
#include "CrustCrawlerData.h"

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

	Joint joints[5] = { Joint1, Joint2, Joint3, Joint4, Joint5 };

	// Custom Private methods
	void _UpdateDynamixelAngles();

	void _SetupDynamixelServos();
};

