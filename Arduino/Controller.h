#pragma once
// General includes
#include <math.h>

// Library includes
#include <BasicLinearAlgebra.h>

// Custom includes
#include "DataStructures.h"
#include "Kinematics.h"
#include "Dynamics.h"
#include "ControlSystem.h"
#include "ComputerConnector.h"
#include "DynamixelConnector.h"
#include "Methods.h"

class Controller
{
public:
	Controller();
	~Controller();

	void main();
	void debugPrint();
	void print();

	unsigned long deltaTime;

private:
	ComputerConnector* computerConnector;
	DynamixelConnector* dynamixelConnector;
	Kinematics* kinematics;
	Dynamics* dynamics;
	
	void _initialize();

	unsigned long _prevTime; 
	unsigned long _newTime;
	void _updateDeltaTime();

