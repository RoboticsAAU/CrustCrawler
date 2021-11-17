#pragma once
// Generel includes

// Lib includes

// Custom includes
#include "CalculusOperations.h"
#include "DataStructures.h"
#include "ComputerConnection.h"

class ControlSystem : public CalculusOperations
{
public:
	ControlSystem(ComputerConnection* pointer);
	Velocities Control(Velocities& currentJointVel, Velocities& desiredJointVel, double& deltaTime);
private:
	ComputerConnection* pComCon;
	double _PID(double& desiredValue, double& currentValue, double& deltaTime);
	double _proportional, _integral, _derivative, _lastError;

};

