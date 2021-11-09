#pragma once
#include "ComputerConnector.h"

// For dynamixel control
#include "Dynamixel2Arduino.h"
#include "DynamixelShield.h"

// For mathematics
#include <BasicLinearAlgebra.h>
#include <math.h>

// Custom headers
#include "DataStructures.h"

#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560) // When using DynamixelShield
#define DYNAMIXEL_SERIAL Serial2 //REMEMBER TO CHANGE TO JUST "Serial"
#define DEBUG_SERIAL Serial1
const uint8_t DIRECTION_PIN = 2; // DYNAMIXEL Shield DIR PIN
#endif

class Controller
{
public:
	Controller();
	~Controller();

	void debugPrint();
	void Print();

	void _ComputerOutputToVelocity(bool emergencyStop, unsigned int controlMode, bool sign, int speed);

	//Updates everything
	void _UpdateChain();
	void _UpdateAngles();

	void _AngleConverter(UnitType desiredUnit);
	void _SpaceConverter(SpaceType desiredSpace);

	//Functions to go between motion states 
	double _DifferentiationOperator(double currentValue, double previousValue);
	double _IntegrationOperator(double currentValue, double& inputIntegrationVal);
	double m_lastValue{ 0 };


	void _ForwardKinematics();

	// Needs to return torque
	void _InverseDynamics();

	Dynamixel2Arduino* p_dynamixel = NULL;

	JointAngles inputAngles;

	Joint m_Joint1, m_Joint2, m_Joint3, m_Joint4, m_Joint5;
	eePosition m_eePosition;

	// All the motion profiles from the computer output:
	Motion inputMotion;
	double prev_Vel1{ 0 }, prev_Vel2{ 0 }, prev_Vel3{ 0 };

	//Defining PID controller variables
	double _PID(double desiredValue, double currentValue);
	double m_proportional{ 0.0 }, m_integral{ 0.0 }, m_derivative{ 0.0 }, m_lastError{ 100.0 }, samplingTime = 1.0/200.0; //Sampling time should be changed

	//Defining constants for PWM equation and torque to PWM function. 
	void _UpdatePWMConstants();
	double _TorqueToPWM();

private:
	double m_currentTime;
};