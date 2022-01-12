#pragma once
// Generel includes

// Library includes

// Custom includes
#include "DataStructures.h"
#include "ComputerConnection.h"

class Dynamics
{
public:
	Dynamics(ComputerConnection* pointer);
	JointTorques InverseDynamics(JointAngles& positions, Velocities& velocities, Accelerations& accelerations);

private:
	ComputerConnection* pComCon; // Pointer to store the address of ComputerConnection object that is passed to the constructor
};

