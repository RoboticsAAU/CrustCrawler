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
	ComputerConnection* pComCon;
	double _PID(double& error, int&& iterator, unsigned long& deltaTime);
	double _PD(double& error, int&& iterator, unsigned long& deltaTime);

	double _P(double& Kp, double& error);
	//double Kp[6] = { 0, 1.1, 2.8, 1.2, 0.6, 0.6 };
	double Kp[6] = { 0, 1.1, 2.8, 1.2, 0.4, 0.4 };
	double gripperSyncGain = 1;

	double _I(double& Ki, double& error, double& integral, unsigned long& deltaTime);
	double Ki[6] = { 0, 0, 0, 0, 0, 0 };
	double integral[6] = { 0,0,0,0,0,0 };

	double _D(double& Kd, double& error, double& lastError, unsigned long& deltaTime);
	double Kd[6] = { 0, 0, 0, 0, 0, 0 };
	double lastError[6];


	// Used for error handling
	void _handleJointLimitations(Velocities& errorVelocities, JointAngles& currentAngles);

	double _velocityBraker(int&& iterator, double& inputAngle);
	double brakingConstant;

	bool _isWithinAngleBoundaries(Joint& inputJoint, double inputAngle);

	bool _isWithinBrakingThreshold(Joint& inputJoint, double inputAngle);
	double brakingThreshold;

	void _gripperSynchronisation(Velocities& errorVelocities, JointAngles& currentAngles);
	double gripperZeroAngle;

};

