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
#ifdef VELOCITY_CONTROL
		bool ControlMode = dynamixel.writeControlTableItem(ControlTableItem::OPERATING_MODE, Joints[i]->ID, OperatingMode::OP_VELOCITY);
#endif // VELOCITY_CONTROL
#ifdef PWM_CONTROL
		bool ControlMode = dynamixel.writeControlTableItem(ControlTableItem::OPERATING_MODE, Joints[i]->ID, OperatingMode::OP_PWM);
#endif // PWM_CONTROL

		//bool Maxtheta = dynamixel.writeControlTableItem(ControlTableItem::MAX_POSITION_LIMIT, Joints[i]->ID, Joints[i]->MaxTheta);
		//bool Mintheta = dynamixel.writeControlTableItem(ControlTableItem::MIN_POSITION_LIMIT, Joints[i]->ID, Joints[i]->MinTheta);
		//bool PWMlimit = dynamixel.writeControlTableItem(ControlTableItem::PWM_LIMIT, Joints[i]->ID, Joints[i]->PWMlimit);
		//bool movingthreshold = dynamixel.writeControlTableItem(ControlTableItem::MOVING_THRESHOLD, Joints[i]->ID, MovingThreshold);

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
		returnJointAngles.thetas[i] = getJointAngle(*Joints[i]);
	}
	returnJointAngles.currentUnitType = Raw;
	return returnJointAngles;
}

double DynamixelConnection::getJointAngle(Joint& joint)
{
	double readAngle = dynamixel.getPresentPosition(joint.ID, ParamUnit::UNIT_RAW);
	//if (readAngle > joint.MaxTheta) 
	//{ 
	//	int jointMultiplier = readAngle / 4095;
	//	readAngle -= jointMultiplier * 4095; 
	//}
	//else if (readAngle < joint.MinTheta) 
	//{ 
	//	int jointMultiplier = readAngle / -4095;
	//	readAngle += (1 + jointMultiplier) * 4095; 
	//}
	return readAngle;
}

Velocities DynamixelConnection::getJointVelocities()
{
	Velocities returnJointVelocities;
	for (size_t i = 1; i < 6; i++)
	{
		returnJointVelocities.velocities[i] = getJointVelocity(Joints[i]->ID);
	}
	returnJointVelocities.currentUnitType = RawsPerSec;
	returnJointVelocities.currentSpaceType = JointSpace;
	return returnJointVelocities;
}

double DynamixelConnection::getJointVelocity(unsigned int& jointID)
{
	return dynamixel.getPresentVelocity(jointID, ParamUnit::UNIT_RAW);
}

void DynamixelConnection::EmergencyStop()
{
	for (size_t i = 1; i < 6; i++)
	{
		dynamixel.torqueOff(Joints[i]->ID);
	}
}

void DynamixelConnection::setJointVelocity(Velocities& goalVelocities)
{
	for (size_t i = 1; i < 6; i++)
	{
		bool set = dynamixel.setGoalVelocity(Joints[i]->ID, goalVelocities.velocities[i], ParamUnit::UNIT_RAW);
	}
}

void DynamixelConnection::setJointPWM(JointTorques& updateTorques, Velocities& currentVelocities)
{
	for (size_t i = 1; i < 6; i++)
	{
		double jointPWM = _typeConverter(updateTorques.torques[i], currentVelocities.velocities[i], Joints[i]->ServoType, PWM);
		bool set = dynamixel.setGoalPWM(Joints[i]->ID, jointPWM);
	}
}

double DynamixelConnection::_typeConverter(double& variable, double& currentVel, ServoType& servoType, OutputType type)
{
	switch (type)
	{
	case Torque:
		// Doesn't work for now
		return (variable - currentVel * velocityConstant) / torqueConstant;
	case PWM: {
		_getPWMConstants(variable, currentVel, servoType);
		return variable * torqueConstant + currentVel * velocityConstant;
	}
	default:
		return 0;
	}
}

void DynamixelConnection::_getPWMConstants(double& desiredTorque, double& currentVel, ServoType& servoType) {
	int constantPicker = copysign(1, (desiredTorque * currentVel));

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
