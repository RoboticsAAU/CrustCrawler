#pragma once
// Generel includes

// Lib includes
#include "lib/BasicLinearAlgebra/BasicLinearAlgebra.h"

// Custom includes
#include "DataStructures.h"
#include "JointConfigs.h"
#include "ComputerConnection.h"
#include "DynamixelConnection.h"
#include "Kinematics.h"
#include "Dynamics.h"
#include "ControlSystem.h"


class Controller
{
public:
	Controller();
	void run();

private:
	ComputerConnection comCon;
	DynamixelConnection dynCon;
	Kinematics kin;
	Dynamics dyn;
	ControlSystem conSys;

	JointAngles _getJointAngles(ControlMode controlMode);
	Velocities _getJointVelocities(ControlMode controlMode);

	void _updateDeltaTime();
	double deltaTime = 0;
	unsigned long previousTime = 0;

	Velocities _toJointVel(JointAngles& jointAngles, Package& instructions);
	Velocities _toVel(Package& instructions);
	Velocities _spaceConverter(JointAngles& jointAngles, Velocities& instructionVelocities, SpaceType desiredSpace);
	double _maxJointLength;
	double _maxAngularVelocity;
	double _LinearToAngularRatio;
};

