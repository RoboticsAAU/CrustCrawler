#pragma once
// Generel includes

// Library includes

// Custom includes
#include "CalculusOperations.h"
#include "DataStructures.h"

class Dynamics : public CalculusOperations
{
public:
	JointTorques InverseDynamics(Velocities& velocities, unsigned long loopTime);
private:
	Velocities _previousVelocities;
	JointTorques _returnTorques;
	JointAngles _previousDesiredPos;
	JointAngles _desiredPos;
	Accelerations _angleAccelerations;

};

