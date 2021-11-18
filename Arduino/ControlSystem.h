#pragma once
// Generel includes

// Lib includes

// Custom includes
#include "CalculusOperations.h"
#include "ComputerConnection.h"
#include "DataStructures.h"
#include "JointConfigs.h"

class ControlSystem : public CalculusOperations
{
public:
	ControlSystem(ComputerConnection* pointer);
	Velocities Control(Velocities& currentJointVel, Velocities& desiredJointVel, double& deltaTime);
private:
	ComputerConnection* pComCon;
	double _PID(double& desiredValue, double& currentValue, int&& iterator, double& deltaTime);
	double integral[6];
	double lastError[6];

};

