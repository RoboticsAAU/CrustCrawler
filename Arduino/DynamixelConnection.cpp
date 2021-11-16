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

	return JointAngles();
}
