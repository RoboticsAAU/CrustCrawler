#include "ControlSystem.h"
ControlSystem::ControlSystem(ComputerConnection* pointer) : pComCon(pointer) {}

JointTorques ControlSystem::Control(Velocities& errorVelocities, JointAngles& currentAngles, double& deltaTime)
{
	if (errorVelocities.currentUnitType != RadiansPerSec) {	errorVelocities.ConvertTo(RadiansPerSec); }
	JointTorques returnJointTorques;
	errorVelocities;
	// double correctedValues[6]; 
	for (size_t i = 1; i < 6; i++)
	{
		if (i >= 4) {
			// We add the correction of the fingers, in order to mirror them in each other. 
			errorVelocities.velocities[i] += _alignFingers(currentAngles).velocities[i];
		}
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
	return (inputAngle > inputJoint.MinTheta) && (inputAngle < inputJoint.MaxTheta);
}

Velocities ControlSystem::_alignFingers(JointAngles& currentJointAngles) {

	if (!currentJointAngles.currentUnitType == Radians) {
		currentJointAngles.ConvertTo(Radians);
	}

	//Both fingers have the same zero point, and positive rotation direction, meaning that one angle is 3/4 ticks larger than the other. 
	//Therefore we rotate their zero point to be at where the gripper is open, and calculate the difference between them from this point. 
	double angleDifference = currentJointAngles.thetas[5] - M_PI_2 + currentJointAngles.thetas[4] - 3*M_PI_2;

	//needs to be in 1/s.
	double Kp = 1;

	Velocities gripperVelocities;

	gripperVelocities.velocities[4] = -Kp * angleDifference;
	gripperVelocities.velocities[5] = Kp * angleDifference;

	return gripperVelocities;
}
