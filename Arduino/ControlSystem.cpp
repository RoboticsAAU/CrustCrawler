#include "ControlSystem.h"
ControlSystem::ControlSystem(ComputerConnection* pointer) : pComCon(pointer) {}

JointTorques ControlSystem::Control(Velocities& errorVelocities, JointAngles& currentAngles, double& deltaTime)
{
	if (errorVelocities.currentUnitType != RadiansPerSec) {	errorVelocities.ConvertTo(RadiansPerSec); }
	JointTorques returnJointTorques;
	// double correctedValues[6]; 
	for (size_t i = 1; i < 6; i++)
	{
		// Error handling
		/*
		if (!_isWithinAngleBoundaries(*Joints[i], currentAngles.thetas[i]))
		{
			double _boundaryMidPoint = (Joints[i]->MaxTheta + Joints[i]->MinTheta) / 2;
			errorVelocities.velocities[i] = currentAngles.thetas[i] > _boundaryMidPoint ? -constant : constant;
		}
		else if (_isWithinBreakingThreshold(*Joints[i], currentAngles.thetas[i])){
			
		}

		*/

		// Regulating
		returnJointTorques.torques[i] = _PID(errorVelocities.velocities[i], i, deltaTime);
	}
	return returnJointTorques;
}

double ControlSystem::_PID(double& error, int&& iterator, double& deltaTime){
	double Kp{ 0.7 }, Ki{ 0.001 }, Kd{ 0.05 };
	
	double proportional = Kp * error;

	/*
	if (error > -0.02 && error < 0.02) {
		integral[iterator] = Ki * Integrate(error, integral[iterator], deltaTime);
	}
	else {
		integral[iterator] = 0;
	}
	*/

	double derivative = Kd * Differentiate(error, lastError[iterator], deltaTime);
	
	lastError[iterator] = error;

	return (proportional + integral[iterator] + derivative);
}

bool ControlSystem::_isWithinAngleBoundaries(Joint& inputJoint, double inputAngle) {
	return (inputAngle >= inputJoint.MinTheta) && (inputAngle <= inputJoint.MaxTheta);
}