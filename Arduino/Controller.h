#pragma once

// For dynamixel control
//#include "Dynamixel2Arduino.h"
//#include "DynamixelShield.h"

// For mathematics
//#include <BasicLinearAlgebra.h>
#include <math.h>

// Custom headers
#include "DataStructures.hpp"


class Controller
{
public:
		
	Controller() {};
	~Controller() {};

	

private:
//Defining PID controller variables
	double _PID(double desiredValue, double currentValue);
	double m_proportional{ 0 }, m_integral{ 0 }, m_derivative{ 0 }, m_lastError{ 0 };

};

