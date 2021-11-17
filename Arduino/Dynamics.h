#pragma once
// Generel includes

// Library includes

// Custom includes
#include "CalculusOperations.h"
#include "DataStructures.h"

class Dynamics : public CalculusOperations
{
public:
	JointTorques InverseDynamics(Velocities& velocities, double& deltaTime);
	void SetStartPos(JointAngles& startAngles);
private:
	Velocities _previousVelocities;
	JointTorques _returnTorques;
	JointAngles _anglePosition;
	Accelerations _angleAccelerations;
};

