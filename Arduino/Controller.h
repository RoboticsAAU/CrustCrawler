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

//#define DYNAMICS_TEST

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
	unsigned long deltaTime = 0;
	unsigned long previousTime = 0;

	unsigned long accumulatedTime = 0;
	unsigned long fixedSendTime = 10 * 1000;

	// Returns the desired velocities - in jointspace - from the instructions, based on the current posiiton of the CrustCrawler
	Velocities _toJointVel(JointAngles& jointAngles, Package& instructions);
	double getDeterminant(BLA::Matrix<3, 3> matrix);

	// Return the instruction velocities in cartesian or joint space, based on the control mode
	Velocities _toVel(Package& instructions);
	double _maxJointLength;
	double _maxAngularVelocity;
	double _LinearToAngularRatio;
	double _GripperCloseConstant = 1; // Velocity in Radians
	bool _isClosing = false;

	// Converts currently only from cartesian to joint space
	Velocities _spaceConverter(JointAngles& jointAngles, Velocities& instructionVelocities, SpaceType desiredSpace);
	double determinantThreshold = 5.0, determinantShift = 2.0;
	int directionSign = 0, prevDirectionSign = 0;

	void breakVelocityAtSingularity(double& velocity, double determinant);

	// Slows down the velocities when close to angle limits.
	void breakVelocitiesAtLimit(JointAngles& jointAngles, Velocities& instructionJointVelocities);
	void breakVelocityAtLimit(double& velocity, double angleDiff);
	double limitBoundary = 100; // Unit: Raw
	
};

