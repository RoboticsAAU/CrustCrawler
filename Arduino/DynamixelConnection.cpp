#include "DynamixelConnection.h"

DynamixelConnection::DynamixelConnection(ComputerConnection* pointer) : dynamixel(DYNAMIXEL_SERIAL, DirectionPin), pComCon(pointer)
{
	dynamixel.begin(57600);

	for (size_t i = 1; i < 6; i++)
	{
		if (dynamixel.getTorqueEnableStat(Joints[i]->ID))
		{
			dynamixel.torqueOff(Joints[i]->ID);
		}
		bool PWM = dynamixel.writeControlTableItem(ControlTableItem::OPERATING_MODE, Joints[i]->ID, OperatingMode::OP_PWM);
		bool Maxtheta = dynamixel.writeControlTableItem(ControlTableItem::MAX_POSITION_LIMIT, Joints[i]->ID, Joints[i]->MaxTheta);
		bool Mintheta = dynamixel.writeControlTableItem(ControlTableItem::MIN_POSITION_LIMIT, Joints[i]->ID, Joints[i]->MinTheta);
		bool PWMlimit = dynamixel.writeControlTableItem(ControlTableItem::PWM_LIMIT, Joints[i]->ID, Joints[i]->PWMlimit);
		bool movingthreshold = dynamixel.writeControlTableItem(ControlTableItem::MOVING_THRESHOLD, Joints[i]->ID, MovingThreshold);

		if (!dynamixel.getTorqueEnableStat(Joints[i]->ID))
		{
			bool torquetrue = dynamixel.torqueOn(Joints[i]->ID);
		}
	}
}

JointAngles DynamixelConnection::getJointAngles()
{
	JointAngles returnJointAngles;
	for (size_t i = 1; i < 6; i++)
	{
		returnJointAngles.thetas[i] = dynamixel.getPresentPosition(Joints[i]->ID, ParamUnit::UNIT_RAW);
	}
	returnJointAngles.currentUnitType = Raw;
	return returnJointAngles;
}

JointAngles DynamixelConnection::getJointAngle(unsigned int& jointID)
{
	JointAngles returnJointAngles;
	returnJointAngles.thetas[jointID] = dynamixel.getPresentPosition(jointID, ParamUnit::UNIT_RAW);
	returnJointAngles.currentUnitType = Raw;
	return returnJointAngles;
}

Velocities DynamixelConnection::getJointVelocities()
{
	Velocities returnJointVelocities;
	for (size_t i = 1; i < 6; i++)
	{
		returnJointVelocities.velocities[i] = dynamixel.getPresentVelocity(Joints[i]->ID, ParamUnit::UNIT_RAW);
	}
	returnJointVelocities.currentUnitType = RawsPerSec;
	returnJointVelocities.currentSpaceType = JointSpace;
	return returnJointVelocities;
}

Velocities DynamixelConnection::getJointVelocity(unsigned int& jointID)
{
	Velocities returnJointVelocities;
	returnJointVelocities.velocities[jointID] = dynamixel.getPresentVelocity(jointID, ParamUnit::UNIT_RAW);
	returnJointVelocities.currentUnitType = RawsPerSec;
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
		if (!_isWithinAngleBoundaries(*Joints[i], currentJointAngles.thetas[i]))
		{
			double _boundaryMidPoint = (Joints[i]->MaxTheta + Joints[i]->MinTheta) / 2;
			currentJointPWM[i] = currentJointAngles.thetas[i] > _boundaryMidPoint ? -Joints[i]->PWMlimit : Joints[i]->PWMlimit;
		}
		// If the joint is a gripper joint, then we set the PWM to a constant
		if (Joints[i]->ID == 4 || Joints[i]->ID == 5) {
			currentJointPWM[i] = correctionVelocities.velocities[i] * Joints[i]->PWMlimit; // desiredVel only represents the direction (+ or -)
		}

		if (currentJointTorques.torques[i] != 0) // Possibly useless
		{
			currentJointTorques.torques[i] += correctionTorques.torques[i];
			currentJointPWM[i] = _typeConverter(currentJointTorques.torques[i], correctionVelocities.velocities[i], *Joints[i], PWM);
		}
		dynamixel.setGoalPWM(Joints[i]->ID, currentJointPWM[i]);
	}
}

double DynamixelConnection::_typeConverter(double& variable, double& desiredVel, Joint& joint, OutputType type)
{
	switch (type)
	{
	case Torque:
		// Doesn't work for now
		return (variable - desiredVel * velocityConstant) / torqueConstant;
	case PWM: {
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