#include "DynamixelConnection.h"

DynamixelConnection::DynamixelConnection(ComputerConnection* pointer) : dynamixel(DYNAMIXEL_SERIAL, DirectionPin), pComCon(pointer)
{
	dynamixel.begin(57600);

	for (size_t i = 1; i < 6; i++) {
		if (dynamixel.getTorqueEnableStat(Joints[i]->ID))
		{
			dynamixel.torqueOff(Joints[i]->ID);
		}
		dynamixel.writeControlTableItem(ControlTableItem::BAUD_RATE, Joints[i]->ID, 3);
	}
	dynamixel.begin(1000000);

	for (size_t i = 1; i < 6; i++)
	{	
		if (dynamixel.getTorqueEnableStat(Joints[i]->ID))
		{
			dynamixel.torqueOff(Joints[i]->ID);
		}

#ifdef VELOCITY_OPERATINGMODE
		bool ControlMode = dynamixel.writeControlTableItem(ControlTableItem::OPERATING_MODE, Joints[i]->ID, OperatingMode::OP_VELOCITY);
#endif // VELOCITY_OPERATINGMODE
#ifdef PWM_OPERATINGMODE
		bool ControlMode = dynamixel.writeControlTableItem(ControlTableItem::OPERATING_MODE, Joints[i]->ID, OperatingMode::OP_PWM);
#endif // PWM_OPERATINGMODE

		//bool Maxtheta = dynamixel.writeControlTableItem(ControlTableItem::MAX_POSITION_LIMIT, Joints[i]->ID, Joints[i]->MaxTheta);
		//bool Mintheta = dynamixel.writeControlTableItem(ControlTableItem::MIN_POSITION_LIMIT, Joints[i]->ID, Joints[i]->MinTheta);
		//bool PWMlimit = dynamixel.writeControlTableItem(ControlTableItem::PWM_LIMIT, Joints[i]->ID, Joints[i]->PWMlimit);
		//bool movingthreshold = dynamixel.writeControlTableItem(ControlTableItem::MOVING_THRESHOLD, Joints[i]->ID, MovingThreshold);

		if (!dynamixel.getTorqueEnableStat(Joints[i]->ID))
		{
			bool torquetrue = dynamixel.torqueOn(Joints[i]->ID);
		}
		
		currentAngle = dynamixel.getPresentPosition(Joints[i]->ID, ParamUnit::UNIT_RAW);
		
		double offset = copysign(2047, currentAngle);

		rawOffsets[i] = (-(currentAngle + offset) / 4095);
		rawOffsets[i] *= 4095;
		
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

	readAngle += rawOffsets[joint.ID];
	
	return readAngle;	
}

Velocities DynamixelConnection::getJointVelocities()
{
	Velocities returnJointVelocities;
	for (size_t i = 1; i < 6; i++)
	{
		returnJointVelocities.velocities[i] = getJointVelocity(*Joints[i]);
	}
	returnJointVelocities.currentUnitType = RPM;
	returnJointVelocities.currentSpaceType = JointSpace;
	return returnJointVelocities;
}

double DynamixelConnection::getJointVelocity(Joint& joint)
{
	return dynamixel.getPresentVelocity(joint.ID, ParamUnit::UNIT_RPM);
}

double DynamixelConnection::getJointLoad(Joint& joint) {
	//getPresentCurrent returns the load percentage. 
	return dynamixel.getPresentCurrent(joint.ID, ParamUnit::UNIT_PERCENT);
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
	goalVelocities.ConvertTo(RPM);
	for (size_t i = 1; i < 6; i++)
	{
		bool set = dynamixel.setGoalVelocity(Joints[i]->ID, (int)goalVelocities.velocities[i], ParamUnit::UNIT_RPM);
	}
}

void DynamixelConnection::setJointPWM(JointTorques& updateTorques, Velocities& currentVelocities)
{
	currentVelocities.ConvertTo(RadiansPerSec);
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
	
	int constantPicker = 0;

	//If the value is approximately equal to zero
	if( !( abs(desiredTorque * currentVel) < 1e-6 ) ){
		constantPicker = (int)copysign(1, (desiredTorque * currentVel));
	}
	switch(servoType) {
	case MX28R: {
		if (constantPicker < 0) { torqueConstant = 211.7; }
		else if (constantPicker == 0) { torqueConstant = 427.4; }
		else if (constantPicker > 0) { torqueConstant = 642.0; }
		velocityConstant = 115.3;
		break;
	}
	case MX64R: {
		if (constantPicker < 0) { torqueConstant = 80.9; }
		else if (constantPicker == 0) { torqueConstant = 152.7; }
		else if (constantPicker > 0) { torqueConstant = 224.5; }
		velocityConstant = 105.3;
		break;
	}
	case MX106R: {
		if (constantPicker < 0) { torqueConstant = 40.4; }
		else if (constantPicker == 0) { torqueConstant = 83.9; }
		else if (constantPicker > 0) { torqueConstant = 127.5; }
		velocityConstant = 160.6;
		break;
	}
	}
}
