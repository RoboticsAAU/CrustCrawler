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

	void _updateDeltaTime();
	double deltaTime;
	unsigned long previousTime;

	Velocities _toJointVel(JointAngles& jointAngles, Package& instructions);
	Velocities _toVel(Package& instructions);
	JointAngles _angleConverter(JointAngles& inputAngles, AngleUnitType desiredUnit);
	Velocities _angleConverter(Velocities& inputVelocities, AngleUnitType desiredUnit);
	Velocities _spaceConverter(JointAngles& jointAngles, Velocities& instructionVelocities, SpaceType desiredSpace);
	double _maxJointLength;
	double _maxAngularVelocity;
	double _LinearToAngularRatio;
};

