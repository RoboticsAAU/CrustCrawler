#pragma once
// Generel includes

// Library includes

// Custom includes
#include "CalculusOperations.h"
#include "DataStructures.h"

class Dynamics : public CalculusOperations
{
public:
	JointTorques InverseDynamics(Velocities& velocities);
private:

};

