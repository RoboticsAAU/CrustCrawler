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
#include "CalculusOperations.h"

#define DYNAMICS_TEST


class Controller : public CalculusOperations
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

	// Returns the necessary angles based on the control mode
	JointAngles _getJointAngles(ControlMode controlMode);

	// Returns the necessary joint velocities based on control mode
	Velocities _getJointVelocities(ControlMode controlMode);

	// Updates the current delta/loop-time
	void _updateDeltaTime();
	double deltaTime = 0;
	unsigned long previousTime = 0;

	// Returns the desired velocities - in jointspace - from the instructions, based on the current posiiton of the CrustCrawler
	Velocities _toJointVel(JointAngles& jointAngles, Package& instructions);

	// Return the instruction velocities in cartesian or joint space, based on the control mode
	Velocities _toVel(Package& instructions);
	double _maxJointLength;
	double _maxAngularVelocity;
	double _LinearToAngularRatio;

	// Converts currently only from cartesian to joint space
	Velocities _spaceConverter(JointAngles& jointAngles, Velocities& instructionVelocities, SpaceType desiredSpace);

	// Returns a motion snapshot based on a desired goal velocity
	//MotionSnapshot _toMotion(JointAngles& currentPositions, JointAngles& goalPositions, double& deltaTime);
	//MotionSnapshot _toMotion(Velocities& currentVelocities, Velocities& goalVelocities, double& deltaTime);
	//MotionSnapshot _toMotion(Accelerations& currentAccelerations, Accelerations& goalAccelrations, double& deltaTime);
};

