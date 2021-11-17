#pragma once
// Generel includes

// Lib includes

// Custom includes
#include "DataStructures.h"

class ControlSystem
{
public:
	JointVelocities Control(JointVelocities& currentVel, JointVelocities& desiredVel);
};

