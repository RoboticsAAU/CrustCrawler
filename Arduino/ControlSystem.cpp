#include "ControlSystem.h"
ControlSystem::ControlSystem(ComputerConnection* pointer) : pComCon(pointer) {}

Velocities ControlSystem::Control(Velocities& currentJointVel, Velocities& desiredJointVel, double& deltaTime)
{
	Velocities returnVelocities;
	returnVelocities.velocities[1] = _PID(desiredJointVel.velocities[1], currentJointVel.velocities[1], deltaTime);
	//returnVelocities.velocities[2] = _PID(desiredJointVel.velocities[2], currentJointVel.velocities[2], deltaTime);
	//returnVelocities.velocities[3] = _PID(desiredJointVel.velocities[3], currentJointVel.velocities[3], deltaTime);

	//for (size_t i = 1; i < 6; i++)
	//{
	//	returnVelocities.velocities[i] = _PID(desiredJointVel.velocities[i], currentJointVel.velocities[i], deltaTime);
	//}
	returnVelocities.currentSpaceType = JointSpace;
	return returnVelocities;
}

double ControlSystem::_PID(double& desiredValue, double& currentValue, double& deltaTime){
	double Kp{ 0.7 }, Ki{ 0.01 }, Kd{ 0.2 };
	double error = desiredValue - currentValue;
	
	_proportional = Kp * error;
	_integral = Ki * Integrate(error, _integral, deltaTime);
	_derivative = Kd * Differentiate(error, _lastError, deltaTime);
	
	_lastError = error;
	pComCon->Print<char*>("\nError: ");
	pComCon->Print<double>(error);
	pComCon->Print<char*>("\nProportional: ");
	pComCon->Print<double>(_proportional);
	pComCon->Print<char*>("\nIntegral: ");
	pComCon->Print<double>(_integral);
	pComCon->Print<char*>("\nDerivative: ");
	pComCon->Print<double>(_derivative);

	return (_proportional + _integral + _derivative);
}