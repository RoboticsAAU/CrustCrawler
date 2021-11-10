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


	// ------ Getters and setters

	// PWM
	float getCurrentPWM(uint8_t id);
	void setPWM(uint8_t id, uint16_t pwmData);

	// Velocity
	float getVelocity(uint8_t id);
	bool isJointMoving(uint8_t id);

	JointAngles getJointAngles(UnitType unitType);






private:
	//Dynamixel connector object pointer (Declared on the HEAP)
	Dynamixel2Arduino* p_dynamixel;

	//Raw dynamixel angles read directly from the servos.
	JointAngles _internalJointAngles;

	//Array of joints, in order to simplify the setup function.
	Joint _joints[5] = { Joint1, Joint2, Joint3, Joint4, Joint5 };

	//Threshold to determine wether the servo is moving. Data is RPM.
	const static int _MovingThreshold = 5;

	// Custom Private methods
	void _UpdateDynamixelAngles();

	void _SetupDynamixelServos();
};

