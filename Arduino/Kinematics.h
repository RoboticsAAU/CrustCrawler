#pragma once
// Generel includes

// Library includes

// Custom includes
#include "DataStructures.h"
#include "JointConfigs.h"

class Kinematics
{
public:
	eePosition ForwardKinematics(JointAngles& JointAngles);
};

