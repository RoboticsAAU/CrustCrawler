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
	return returnJointAngles;
}

JointVelocities DynamixelConnection::getJointVelocities()
{
	JointVelocities returnJointVelocities;
	for (size_t i = 1; i < 6; i++)
	{
		returnJointVelocities.velocities[i] = dynamixel.getPresentVelocity(Joints[i]->ID);
	}
	return returnJointVelocities;
}

void DynamixelConnection::EmergencyStop()
{
	for (size_t i = 1; i < 6; i++)
	{
		dynamixel.setGoalPWM(Joints[i]->ID, 0);
	}
}

void DynamixelConnection::setJointPWM(JointTorques& correctionTorques)
{
	for (size_t i = 1; i < 6; i++)
	{
		currentJointTorques.torques[i] += correctionTorques.torques[i];
		double JointPWM = _typeConverter(currentJointTorques.torques[i], PWM);
		dynamixel.setGoalPWM(Joints[i]->ID, JointPWM);
	}
}

double DynamixelConnection::_typeConverter(double variable, OutputType type)
{
	double conversionFunction;
	switch (type)
	{
	case Torque:
		return variable * conversionFunction;
	case PWM:
		return variable * conversionFunction;
	default:
		// Error invalid type
		break;
	}
}
