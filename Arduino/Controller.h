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

#define DYNAMIXEL_SERIAL Serial
const uint8_t DIRECTION_PIN = 2; // DYNAMIXEL Shield DIR PIN

class Controller
{
public:
	Controller();
	~Controller();

	void debugPrint();


private:
	void _ComputerOutputToVelocity(bool emergencyStop, unsigned int controlMode, bool sign, unsigned int speed);

	//Updates everything
	void _UpdateChain();
	void _UpdateAngles();

	void _AngleConverter(UnitType desiredUnit);
	void _SpaceConverter(SpaceType desiredSpace);

	//Functions to go between motion states 
	double _CalculusOperator(OperationType operationType, double currentValue, double& previousValue);
	double m_lastValue{ 0 };

	void emergancystop();


	void _ForwardKinematics();

	// Needs to return torque
	void _InverseDynamics();

	JointAngles inputAngles;
	Velocities inputVelocities;
	Joint m_Joint1, m_Joint2, m_Joint3, m_Joint4, m_Joint5;
	eePosition m_eePosition;


//Defining PID controller variables
	double _PID(double desiredValue, double currentValue);
	double m_proportional{ 0 }, m_integral{ 0 }, m_derivative{ 0 }, m_lastError{ 0 }, samplingTime = 1/200; //Sampling time should be changed


};

