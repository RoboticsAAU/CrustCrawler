#pragma once

// For dynamixel control
#include "Dynamixel2Arduino.h"
#include "DynamixelShield.h"

// For mathematics
#include <BasicLinearAlgebra.h>
#include <math.h>


// Custom headers
#include "DataStructures.h"


class Controller
{
public:
	Controller();
	~Controller();

	void debugPrint();


private:
//Updates everything
	void _UpdateChain();
	void _UpdateAngles();

	void _AngleConverter(UnitType desiredUnit);
	void _SpaceConverter(SpaceType desiredSpace);

	//Functions to go between motion states 
	double _CalculusOperator(OperationType operationType, double desiredValue, double currentValue, int numberOfTimes);


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

