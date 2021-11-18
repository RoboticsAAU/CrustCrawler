#pragma once
// Generel includes

// Library includes

// Custom includes
#include "CalculusOperations.h"
#include "DataStructures.h"
#include "ComputerConnection.h"

class Dynamics : public CalculusOperations
{
public:
	Dynamics(ComputerConnection* pointer);
	JointTorques InverseDynamics(Velocities& velocities, double& deltaTime);
	void SetStartPos(JointAngles& startAngles);
private:
	ComputerConnection* pComCon;
	Velocities _previousVelocities;
	JointTorques _returnTorques;
	JointAngles _anglePosition;
	Accelerations _angleAccelerations;
};

