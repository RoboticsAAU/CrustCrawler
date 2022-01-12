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

class Controller : public CalculusOperations
{
public:
	Controller();
	void run();

private:
	// Creating objects for the different classes
	ComputerConnection comCon;
	DynamixelConnection dynCon;
	Kinematics kin;
	Dynamics dyn;
	ControlSystem conSys;

	// Determining deltatime
	void _updateDeltaTime();
	unsigned long deltaTime = 0;             // Time duration of previous runtime
	unsigned long previousTime = 0;          // Time stamp that is set to the current time at the start of each loop
	unsigned long accumulatedTime = 0;       // Variable to store the accumulated time since last time instructions were sent to servos
	unsigned long fixedSendTime = 10 * 1000; // Constant fixed send time (in microseconds) corresponding to 10 milliseconds


	// Converting instructions to joint velocity
	Velocities _toJointVel(JointAngles& jointAngles, Package& instructions);

	Velocities _toVel(Package& instructions);
	double _maxJointLength;             // The length of the CrustCrawler when it is fully extended (e.g. when all angles are 0)
	double _maxAngularVelocity;			// The maximum permissible angular velocity based on CrustCrawler's max length and max linear velocity
	double _LinearToAngularRatio;		// The ratio between linear and angular velocity. Derived from the formula for relation between tangential and angular velocity
	double _GripperCloseConstant = 1.0; // Constant closing reference velocity of the grippers (in radians)
	bool _isClosing = false;            // True if the gripper is closing (only used in PWM operating mode -> always false in Velocity operating mode)

	Velocities _spaceConverter(JointAngles& jointAngles, Velocities& instructionVelocities, SpaceType desiredSpace);
	double getDeterminant(BLA::Matrix<3, 3> matrix);
	void brakeVelocityAtSingularity(double& velocity, double determinant);
	double determinantThreshold = 5.0;			  // A threshold for when velocity braking should begin
	double determinantShift = 2.0;		          // Value used to shift the singularity stop point
	int directionSign = 0, prevDirectionSign = 0; // The current and previous direction of movement determined from the current velocity

	// Slows down the velocities when close to angle limits.
	void brakeVelocitiesAtLimit(JointAngles& jointAngles, Velocities& instructionJointVelocities);
	void brakeVelocityAtLimit(double& velocity, double angleDiff);
	double limitBoundary = 100.0;	// Boundary/threshold (in terms of angle difference) for when velocity braking should begin (in Raw)
};

