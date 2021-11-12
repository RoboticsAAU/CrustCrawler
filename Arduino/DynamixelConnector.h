#pragma once
#include "DataStructures.h"
#include "CrustCrawlerData.h"

//Include dynamixel libraries
#include "Dynamixel2Arduino.h"
#include "DynamixelShield.h"

#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560) // When using DynamixelShield
#define DYNAMIXEL_SERIAL Serial
const uint8_t DIRECTION_PIN = 2; // DYNAMIXEL Shield DIR PIN
const unsigned long DYNAMIXEL_BAUDRATE = 57600;
#endif

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

	void getJointAngles(UnitType unitType, JointAngles &jointAnglesObject);






private:
	//Dynamixel connector object pointer (Declared on the HEAP)
	Dynamixel2Arduino* p_dynamixel = new Dynamixel2Arduino(DYNAMIXEL_SERIAL,DIRECTION_PIN);

	//Array of joints, in order to simplify the setup function.
	Joint _joints[5] = { CrustCrawler::Joint1, CrustCrawler::Joint2, CrustCrawler::Joint3, CrustCrawler::Joint4, CrustCrawler::Joint5 };

	//Threshold to determine wether the servo is moving. Data is RPM.
	const static int _MovingThreshold = 5;

	// Custom Private methods
	void _UpdateDynamixelAngles(JointAngles& JointAnglesObject);

	void _SetupDynamixelServos();
};

