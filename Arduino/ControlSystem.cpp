#include "ControlSystem.h"
ControlSystem::ControlSystem(ComputerConnection* pointer) : pComCon(pointer) {}

Velocities ControlSystem::Control(Velocities& currentJointVel, Velocities& desiredJointVel, double& deltaTime)
{
	if (currentJointVel.currentUnitType != desiredJointVel.currentUnitType)
	{
		currentJointVel.ConvertTo(desiredJointVel.currentUnitType);
	}
	Velocities returnVelocities;
	for (size_t i = 1; i < 6; i++)
	{
		returnVelocities.velocities[i] = _PID(desiredJointVel.velocities[i], currentJointVel.velocities[i], Joints[i]->ID, deltaTime);
	}
	returnVelocities.currentUnitType = currentJointVel.currentUnitType;
	returnVelocities.currentSpaceType = currentJointVel.currentSpaceType;
	return returnVelocities;
}

double ControlSystem::_PID(double& desiredValue, double& currentValue, int&& iterator, double& deltaTime){
	double Kp{ 0.7 }, Ki{ 0.001 }, Kd{ 0.05 };
	double error = desiredValue - currentValue;
	
	double proportional = Kp * error;
	if (currentValue > 0.98 * desiredValue || currentValue < 1.02 * desiredValue)
	{
		integral[iterator] = Ki * Integrate(error, integral[iterator], deltaTime);
	}
	else if (currentValue < 0.3 * desiredValue || currentValue > 1.7 * desiredValue)
	{
		integral[iterator] = 0;
	}
	double derivative = Kd * Differentiate(error, lastError[iterator], deltaTime);
	
	lastError[iterator] = error;

	return (proportional + integral[iterator] + derivative);
}