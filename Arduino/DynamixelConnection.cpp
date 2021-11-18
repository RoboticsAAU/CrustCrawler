#include "DynamixelConnection.h"

DynamixelConnection::DynamixelConnection() : dynamixel(DYNAMIXEL_SERIAL, DirectionPin)
{
	dynamixel.begin(57600);

	for (size_t i = 1; i < 6; i++)
	{
		if (dynamixel.getTorqueEnableStat(i))
		{
			dynamixel.torqueOff(Joints[i]->ID);
		}

		dynamixel.writeControlTableItem(ControlTableItem::OPERATING_MODE, Joints[i]->ID, OperatingMode::OP_PWM);
		dynamixel.writeControlTableItem(ControlTableItem::MAX_POSITION_LIMIT, Joints[i]->ID, Joints[i]->MaxTheta);
		dynamixel.writeControlTableItem(ControlTableItem::MIN_POSITION_LIMIT, Joints[i]->ID, Joints[i]->MaxTheta);
		dynamixel.writeControlTableItem(ControlTableItem::PWM_LIMIT, Joints[i]->ID, Joints[i]->PWMlimit);
		dynamixel.writeControlTableItem(ControlTableItem::MOVING_THRESHOLD, Joints[i]->ID, MovingThreshold);

		if (!dynamixel.getTorqueEnableStat(i))
		{
			dynamixel.torqueOn(Joints[i]->ID);
		}
	}
}

JointAngles DynamixelConnection::getJointAngles()
{
	JointAngles returnJointAngles;
	for (size_t i = 1; i < 6; i++)
	{
		returnJointAngles.thetas[i] = dynamixel.getPresentPosition(Joints[i]->ID);
	}
	returnJointAngles.currentUnitType = Raw;
	return returnJointAngles;
}

Velocities DynamixelConnection::getJointVelocities()
{
	Velocities returnJointVelocities;
	for (size_t i = 1; i < 6; i++)
	{
		returnJointVelocities.velocities[i] = dynamixel.getPresentVelocity(Joints[i]->ID) * ((2 * M_PI) / 4095);
	}
	returnJointVelocities.currentSpaceType = JointSpace;
	return returnJointVelocities;
}

void DynamixelConnection::EmergencyStop()
{
	for (size_t i = 1; i < 6; i++)
	{
		dynamixel.setGoalPWM(Joints[i]->ID, 0);
	}
}

void DynamixelConnection::setJointPWM(JointTorques& correctionTorques, Velocities& correctionVelocities, JointAngles& currentJointAngles)
{
	for (size_t i = 1; i < 6; i++)
	{
		currentJointTorques.torques[i] += correctionTorques.torques[i];
		double JointPWM = _typeConverter(currentJointTorques.torques[i], correctionVelocities.velocities[i],
										 currentJointAngles.thetas[i], *Joints[i], PWM);
		dynamixel.setGoalPWM(Joints[i]->ID, JointPWM);
	}
}

double DynamixelConnection::_typeConverter(double& variable, double& desiredVel, double& currentJointAngle, Joint& joint, OutputType type)
{
	switch (type)
	{
	case Torque:
		// Doesn't work for now
		return (variable - desiredVel * velocityConstant) / torqueConstant;
	case PWM: {
		if (!_isWithinAngleBoundaries(joint, currentJointAngle))
		{
			double _boundaryMidPoint = (joint.MaxTheta + joint.MinTheta) / 2;
			return currentJointAngle > _boundaryMidPoint ? -joint.PWMlimit : joint.PWMlimit;
		}

		// If the joint is a gripper joint, then we set the PWM to a constant
		if (joint.ID == 4 || joint.ID == 5) { 
			return desiredVel * joint.PWMlimit; // desiredVel only represents the direction (+ or -)
		}
		_getPWMConstants(variable, desiredVel, joint.ServoType);
		return variable * torqueConstant + desiredVel * velocityConstant;
	}
	default:
		// Error invalid type
		break;
	}
}

void DynamixelConnection::_getPWMConstants(double& desiredTorque, double& desiredVel, ServoType servoType) {
	int constantPicker = copysign(1, (desiredTorque * desiredVel));

	switch(servoType) {
	case MX28R: {
		if (constantPicker < 0) { torqueConstant = 211.7; }
		else if (constantPicker == 0) { torqueConstant = 427.4; }
		else if (constantPicker > 0) { torqueConstant = 642.0; }
		velocityConstant = 115.3;
	}
	case MX64R: {
		if (constantPicker < 0) { torqueConstant = 80.9; }
		else if (constantPicker == 0) { torqueConstant = 152.7; }
		else if (constantPicker > 0) { torqueConstant = 224.5; }
		velocityConstant = 105.3;
	}
	case MX106R: {
		if (constantPicker < 0) { torqueConstant = 40.4; }
		else if (constantPicker == 0) { torqueConstant = 83.9; }
		else if (constantPicker > 0) { torqueConstant = 127.5; }
		velocityConstant = 160.6;
	}
	}
}

bool DynamixelConnection::_isWithinAngleBoundaries(Joint& inputJoint, double inputAngle) {
	return (inputAngle >= inputJoint.MinTheta) && (inputAngle <= inputJoint.MaxTheta);
}