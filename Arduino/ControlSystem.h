#pragma once
// Generel includes

// Lib includes

// Custom includes
#include "DataStructures.h"

class ControlSystem
{
public:
	Velocities Control(Velocities& currentJointVel, Velocities& desiredJointVel);
};

