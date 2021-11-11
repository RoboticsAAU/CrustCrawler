#pragma once

// For mathematics
#include <BasicLinearAlgebra.h>
#include <math.h>

// Custom headers
#include "DataStructures.h"
#include "Kinematics.h"
#include "Dynamics.h"
#include "Controlsystem.h"
#include "ComputerConnector.h"
#include "DynamixelConnector.h"

#include "CrustCrawlerData.h"
#include "Methods.h"

#define DYNAMIXEL_SERIAL Serial
const uint8_t DIRECTION_PIN = 2; // DYNAMIXEL Shield DIR PIN

class Controller
{
public:
	Controller();
	~Controller();

	void main();
	void debugPrint();
	void Print();

	double Looptime;


private:

	ComputerConnector* computerConnector;
	DynamixelConnector* dynamixelConnector;
	Kinematics* kinematics;
	Dynamics* dynamics;
	

	unsigned long _PrevTime = millis(); 
	unsigned long _NewTime;

	unsigned long _UpdateLoopTime();

	//---------------- This is old, need review---------------------//
	//void _ComputerOutputToVelocity(unsigned int controlMode, bool sign, unsigned int speed);

	//Updates everything
	//void _UpdateChain();
	//void _UpdateAngles();

	//void _SpaceConverter(SpaceType desiredSpace);

	//Functions to go between motion states 
	//double _DifferentiationOperator(double currentValue, double previousValue);
	//double _IntegrationOperator(double currentValue, double inputIntegrationVal);
	double m_lastValue{ 0 };

	//void _EmergencyStop();

	//void _ForwardKinematics();

	// Needs to return torque
	//void _InverseDynamics();

	//Dynamixel2Arduino* p_dynamixel = NULL;

	//JointAngles inputAngles;
	//Motion inputMotion;
	//double prevVel1{ 0 },prevVel2{ 0 }, prevVel3{ 0 };

	//eePosition m_eePosition;

	//Defining PID controller variables
	double _PID(double desiredValue, double currentValue);
	double m_proportional{ 0.0 }, m_integral{ 0.0 }, m_derivative{ 0.0 }, m_lastError{ 100.0 }, samplingTime = 1.0/200.0; //Sampling time should be changed

	//Functions for converting the torques to PWM values.
	void _GetJointPWMConstants(Joint& inputJoint);
	void _TorqueToPWM(Joint& inputJoint);
};