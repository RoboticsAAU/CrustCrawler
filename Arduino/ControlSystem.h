#pragma once
// Generel includes

// Lib includes

// Custom includes
#include "CalculusOperations.h"
#include "ComputerConnection.h"
#include "DataStructures.h"
#include "JointConfigs.h"

class ControlSystem : public CalculusOperations
{
public:
	ControlSystem(ComputerConnection* pointer);
	JointTorques Control(Velocities& errorVelocities, JointAngles& currentAngles, unsigned long& deltaTime);
private:
	ComputerConnection* pComCon; // Pointer to store the address of ComputerConnection object that is passed to the constructor

	// Creating a PID controller
	double _PID(double& error, int&& iterator, unsigned long& deltaTime);

	double _P(double& Kp, double& error);
	double Kp[6] = { 0, 1.1, 2.8, 1.2, 0.4, 0.4 }; // Kp gains for all five joints (manually tuned)

	double _I(double& Ki, double& error, double& integral, unsigned long& deltaTime);
	double Ki[6] = { 0, 0, 0, 0, 0, 0 };	       // Ki gains for all five joints (only P-controller, so all are 0)
	double integral[6] = { 0, 0, 0, 0, 0, 0 };     // Used for storing the accumulated sum of all five integral terms

	double _D(double& Kd, double& error, double& lastError, unsigned long& deltaTime);
	double Kd[6] = { 0, 0, 0, 0, 0, 0 };	       // Kd gains for all five joints (only P-controller, so all are 0)
	double lastError[6];					       // Error measured in the last loop


	/////////////////////////////////////////////////////////////////////////////
	// THE FOLLOWING FUNCTIONS ARE NOT IMPLEMENTED, I.E. THEY ARE NEVER CALLED //
	/////////////////////////////////////////////////////////////////////////////

	double _PD(double& error, int&& iterator, unsigned long& deltaTime);
	
	void _handleJointLimitations(Velocities& errorVelocities, JointAngles& currentAngles);

	double _velocityBraker(int&& iterator, double& inputAngle);
	double brakingConstant;

	bool _isWithinAngleBoundaries(Joint& inputJoint, double inputAngle);

	bool _isWithinBrakingThreshold(Joint& inputJoint, double inputAngle);
	double brakingThreshold;

	void _gripperSynchronisation(Velocities& errorVelocities, JointAngles& currentAngles);
	double gripperZeroAngle;
	double gripperSyncGain = 1;
};

