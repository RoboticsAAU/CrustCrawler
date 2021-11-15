#pragma once
// Arduino includes

// Library includes
#include "Dynamixel2Arduino.h"
#include "DynamixelShield.h"

// Custom includes
#include "CommonData.h"


class DynamixelConnector
{
public:
	DynamixelConnector();
	~DynamixelConnector();

	//Custom methods
	void AngleConverter(JointAngles &inputAngles, UnitType desiredUnit);

	//Torque off for every servo.
	void EmergencyStop();


	// ------ Getters and setters -------

	// PWM
	float getCurrentPWM(uint8_t id);
	void setPWM(uint8_t id, uint16_t pwmData);

	// Velocity
	float getVelocity(uint8_t id);
	bool isJointMoving(uint8_t id);

	void getJointAngles(UnitType unitType, JointAngles &jointAnglesObject);

private:
	//Dynamixel connector object pointer (Declared on the HEAP)
	Dynamixel2Arduino* p_dynamixel = new Dynamixel2Arduino(DYNAMIXEL_SERIAL,DIRECTION_PIN);


	//Threshold to determine wether the servo is moving. Data is RPM.
	const int _MovingThreshold = 5;

	// Custom Private methods
	void _UpdateDynamixelAngles(JointAngles& JointAnglesObject);

	void _SetupDynamixelServos();
};

