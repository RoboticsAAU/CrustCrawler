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
	Velocities Control(Velocities& currentJointVel, Velocities& desiredJointVel, double& deltaTime);
private:
	ComputerConnection* pComCon;
	double _PID(double& desiredValue, double& currentValue, int&& iterator, double& deltaTime);
	double integral[6];
	double lastError[6];

	bool _isWithinAngleBoundaries(Joint& inputJoint, double inputAngle);


	// TEMP
	//double currentJointPWM;
	//if (!_isWithinAngleBoundaries(*Joints[i], currentJointAngles.thetas[i]))
	//{
	//	double _boundaryMidPoint = (Joints[i]->MaxTheta + Joints[i]->MinTheta) / 2;
	//	currentJointPWM = currentJointAngles.thetas[i] > _boundaryMidPoint ? -0.8 * Joints[i]->PWMlimit : 0.8 * Joints[i]->PWMlimit;
	//
	//}
	//// If the joint is a gripper joint, then we set the PWM to a constant
	//else if (Joints[i]->ID == 4 || Joints[i]->ID == 5) {
	//	currentJointPWM = correctionVelocities.velocities[i] * 0.8 * Joints[i]->PWMlimit; // desiredVel only represents the direction (+ or -)
	//}
	//else {
	//	currentJointPWM = _typeConverter(updateTorques.torques[i], correctionVelocities.velocities[i], *Joints[i], PWM);
	//}
	//currentJointPWM *= 0.113;
	//bool set = dynamixel.setGoalPWM(Joints[i]->ID, currentJointPWM);

};

