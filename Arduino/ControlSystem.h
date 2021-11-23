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
	JointTorques Control(Velocities& errorVelocities, JointAngles& currentAngles, double& deltaTime);
private:
	ComputerConnection* pComCon;
	double _PID(double& error, int&& iterator, double& deltaTime);
	double integral[6] = { 0,0,0,0,0,0 };
	double lastError[6];

	bool _isWithinAngleBoundaries(Joint& inputJoint, double inputAngle);

};

