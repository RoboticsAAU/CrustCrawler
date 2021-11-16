#pragma once
// Generel includes

// Lib includes

// Custom includes
#include "DataStructures.h"
#include "JointConfigs.h"
#include "ComputerConnection.h"
#include "DynamixelConnection.h"
#include "Kinematics.h"
#include "Dynamics.h"
#include "ControlSystem.h"


class Controller
{
public:
	Controller();
	void run();

private:
	ComputerConnection comCon;
	DynamixelConnection dynCon;
	Kinematics kin;
	Dynamics dyn;
	ControlSystem conSys;

	void _updateDeltaTime();
	unsigned long deltaTime;
	unsigned long previousTime;

	Joint Joints[6];
};

