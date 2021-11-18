#include "ControlSystem.h"
ControlSystem::ControlSystem(ComputerConnection* pointer) : pComCon(pointer) {}

Velocities ControlSystem::Control(Velocities& currentJointVel, Velocities& desiredJointVel, double& deltaTime)
{
	Velocities returnVelocities;
	for (size_t i = 1; i < 6; i++)
	{
		returnVelocities.velocities[i] = _PID(desiredJointVel.velocities[i], currentJointVel.velocities[i], Joints[i]->ID, deltaTime);
	}
	returnVelocities.currentSpaceType = JointSpace;
	return returnVelocities;
}

double ControlSystem::_PID(double& desiredValue, double& currentValue, int&& iterator, double& deltaTime){
	double Kp{ 0.7 }, Ki{ 0.001 }, Kd{ 0.05 };
	double error = desiredValue - currentValue;
	
	double proportional = Kp * error;
	integral[iterator] = Ki * Integrate(error, integral[iterator], deltaTime);
	double derivative = Kd * Differentiate(error, lastError[iterator], deltaTime);
	
	lastError[iterator] = error;

	return (proportional + integral[iterator] + derivative);
}