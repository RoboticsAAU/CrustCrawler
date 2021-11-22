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
	JointTorques InverseDynamics(MotionSnapshot& snapshot);
private:
	ComputerConnection* pComCon;
};

